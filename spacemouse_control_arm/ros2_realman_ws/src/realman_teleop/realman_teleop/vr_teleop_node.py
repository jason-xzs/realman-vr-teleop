"""
VR 遥操作节点 - 将 VR 位姿转换为机械臂笛卡尔位姿

功能：
1. 订阅 VR 话题获取控制器位姿
2. 订阅 /rm/state_joint_state 获取机械臂当前状态
3. 计算位姿差分并进行坐标变换
4. 发布笛卡尔位姿到 /rm/action_pose 供 realman_driver_node 使用

数据流：
-------
/vr/right/pose, /vr/right/trigger
        ↓
    vr_teleop_node
        ├── 订阅 VR 话题
        ├── 订阅 /rm/state_joint_state (来自 realman_driver_node)
        ├── 计算: 位置差分 + 旋转变换
        └── 发布: /rm/action_pose (笛卡尔位姿)
                ↓
        realman_driver_node (使用 movep_canfd)

控制逻辑（参照 HMI vr_control_interpolation.py）：
1. trigger 释放时: 持续同步机械臂位姿作为锚点
2. trigger 按下时: 
   - 记录 VR 初始位姿和机械臂初始位姿
   - 计算位置差分: delta = transform(vr_curr - vr_init)
   - 计算目标位置: target = arm_init + delta
   - 计算目标旋转: 四元数变换
   - 直接发送笛卡尔位姿（由 SDK 内部进行 IK 反解）
3. 平滑滤波: EMA 位置滤波 + 四元数滤波
"""
import time
import threading
from typing import Optional, List

import numpy as np
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import json

from .vr_utils import (
    VRCoordinateTransform,
    ROTATION_MAPPING_PRESETS,
    PoseFilter,
    VRControlState,
    QuaternionUtils,
)
from .utils import add_repo_paths, ROBOT_JOINT_NAMES

# 添加 Robotic_Arm 路径（保留用于可能的 FK 计算）
add_repo_paths()


def get_local_ip() -> str:
    """获取本机 IP"""
    import socket
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        local_ip = sock.getsockname()[0]
        sock.close()
        return local_ip
    except Exception:
        return "127.0.0.1"


class VRTeleopNode(Node):
    """
    VR 遥操作节点
    
    订阅 VR 话题，计算目标笛卡尔位姿，发布到 /rm/action_pose。
    由 realman_driver_node 使用 movep_canfd 发送到机械臂。
    """
    
    def __init__(self):
        super().__init__("vr_teleop_node")
        
        # ===== 机械臂连接参数（保留用于可能的 FK 计算）=====
        self.declare_parameter("ip", "192.168.1.18")
        self.declare_parameter("port", 8080)
        self.declare_parameter("dry_run", False)
        
        # ===== 状态话题参数 =====
        self.declare_parameter("state_topic", "/rm/state_joint_state")
        
        # ===== 控制参数 =====
        self.declare_parameter("control_hz", 50.0)
        self.declare_parameter("trigger_threshold", 0.85)
        self.declare_parameter("grip_threshold", 0.05)
        self.declare_parameter("grip_release_threshold", 0.02)
        self.declare_parameter("grip_toggle_cooldown", 1.2)
        self.declare_parameter("alpha_pos", 0.3)
        self.declare_parameter("alpha_rot", 0.3)
        self.declare_parameter("side", "right")

        # ===== 旋转映射方案 =====
        self.declare_parameter("rotation_preset", "current")
        
        # ===== 话题参数 =====
        self.declare_parameter("action_pose_topic", "/rm/action_pose")  # 笛卡尔位姿话题
        self.declare_parameter("service_prefix", "/vr")
        self.declare_parameter("status_topic", "/vr/teleop_status")
        self.declare_parameter("driver_service_prefix", "/driver")
        self.declare_parameter("frame_id", "rm_base")
        self.declare_parameter("joint_names", ROBOT_JOINT_NAMES)
        
        # ===== 读取参数 =====
        self.ip = self.get_parameter("ip").get_parameter_value().string_value
        self.port = self.get_parameter("port").get_parameter_value().integer_value
        self.dry_run = self.get_parameter("dry_run").get_parameter_value().bool_value
        self.state_topic = self.get_parameter("state_topic").get_parameter_value().string_value
        self.control_hz = self.get_parameter("control_hz").get_parameter_value().double_value
        self.trigger_threshold = self.get_parameter("trigger_threshold").get_parameter_value().double_value
        self.grip_threshold = self.get_parameter("grip_threshold").get_parameter_value().double_value
        self.grip_release_threshold = self.get_parameter("grip_release_threshold").get_parameter_value().double_value
        self.grip_toggle_cooldown = self.get_parameter("grip_toggle_cooldown").get_parameter_value().double_value
        self.alpha_pos = self.get_parameter("alpha_pos").get_parameter_value().double_value
        self.alpha_rot = self.get_parameter("alpha_rot").get_parameter_value().double_value
        self.side = self.get_parameter("side").get_parameter_value().string_value
        self.rotation_preset = self.get_parameter("rotation_preset").get_parameter_value().string_value
        self.action_pose_topic = self.get_parameter("action_pose_topic").get_parameter_value().string_value
        self.service_prefix = self.get_parameter("service_prefix").get_parameter_value().string_value.rstrip("/") or "/vr"
        self.status_topic = self.get_parameter("status_topic").get_parameter_value().string_value
        self.driver_service_prefix = self.get_parameter("driver_service_prefix").get_parameter_value().string_value.rstrip("/") or "/driver"
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.joint_names = list(self.get_parameter("joint_names").value)
        
        # ===== 初始化组件 =====
        self._control_enabled = False
        
        # 状态缓存（从 /rm/state_joint_state 获取）
        self._cached_state: Optional[JointState] = None
        self._cached_eef_pose: Optional[np.ndarray] = None
        self._state_lock = threading.Lock()
        
        # 坐标变换器（传入旋转映射方案）
        self.coord_transform = VRCoordinateTransform(
            side=self.side,
            rotation_preset=self.rotation_preset
        )
        
        # 位姿滤波器
        self.pose_filter = PoseFilter(self.alpha_pos, self.alpha_rot)
        
        # 控制状态
        self.state = VRControlState()
        self._lock = threading.Lock()
        
        # 当前 VR 输入
        self._current_pose: Optional[PoseStamped] = None
        self._current_trigger: float = 0.0
        self._current_grip: float = 1.0  # 初始状态：打开
        self._vr_last_update: float = 0.0
        
        # 夹爪toggle控制状态
        self._gripper_closed = False  # False=打开(1.0), True=闭合(0.0)
        self._last_grip_value = 0.0   # 用于边缘检测
        self._grip_pressed = False
        self._last_grip_toggle_time = 0.0
        self._last_grip_toggle_time = 0.0
        
        # ===== 订阅 VR 话题 =====
        pose_topic = f"/vr/{self.side}/pose"
        trigger_topic = f"/vr/{self.side}/trigger"
        grip_topic = f"/vr/{self.side}/grip"
        
        self.pose_sub = self.create_subscription(PoseStamped, pose_topic, self._on_pose, 10)
        self.trigger_sub = self.create_subscription(Float32, trigger_topic, self._on_trigger, 10)
        self.grip_sub = self.create_subscription(Float32, grip_topic, self._on_grip, 10)
        
        # ===== 订阅机械臂状态话题 =====
        self.state_sub = self.create_subscription(
            JointState,
            self.state_topic,
            self._on_arm_state,
            10
        )
        self.get_logger().info(f"订阅状态话题: {self.state_topic}")
        
        # ===== 创建发布者（笛卡尔位姿）=====
        self.action_pub = self.create_publisher(JointState, self.action_pose_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        
        # ===== 创建 Service =====
        self.enable_srv = self.create_service(Trigger, f"{self.service_prefix}/enable_control", self._handle_enable)
        self.disable_srv = self.create_service(Trigger, f"{self.service_prefix}/disable_control", self._handle_disable)
        
        # ===== Driver 跟随控制 Service 客户端 =====
        self.driver_enable_follow_client = self.create_client(Trigger, f"{self.driver_service_prefix}/enable_follow")
        self.driver_disable_follow_client = self.create_client(Trigger, f"{self.driver_service_prefix}/disable_follow")
        self.driver_gripper_toggle_client = self.create_client(Trigger, f"{self.driver_service_prefix}/gripper_toggle_lock")
        
        # ===== 控制定时器 =====
        period = 1.0 / self.control_hz if self.control_hz > 0 else 0.02
        self.timer = self.create_timer(period, self._on_timer)
        
        # ===== 统计日志 =====
        self.stats_timer = self.create_timer(10.0, self._log_stats)
        self._cmd_count = 0
        
        self.get_logger().info(f"VR 遥操作节点已启动（笛卡尔位姿模式）")
        self.get_logger().info(f"  机械臂 IP: {self.ip}, 控制频率: {self.control_hz}Hz")
        self.get_logger().info(f"  VR 侧: {self.side}, trigger 阈值: {self.trigger_threshold}")
        self.get_logger().info(f"  发布话题: {self.action_pose_topic}")
        
        if not self.dry_run:
            self.get_logger().warn(f"⚠️ VR 控制已禁用，请调用 {self.service_prefix}/enable_control 启用")
    
    def _on_arm_state(self, msg: JointState):
        """
        接收机械臂状态（来自 realman_driver_node）
        
        状态格式 (14维):
            position[0:7]  = 关节角度（弧度）
            position[7]    = 夹爪开合度（0-1）
            position[8:14] = 末端位姿 [x, y, z, rx, ry, rz]（米/弧度）
        """
        with self._state_lock:
            self._cached_state = msg
            
            # 从状态话题中获取末端位姿
            if len(msg.position) >= 14:
                self._cached_eef_pose = np.array(msg.position[8:14], dtype=np.float64)
            else:
                # 如果状态不含末端位姿，记录警告
                if self._cached_eef_pose is None:
                    self.get_logger().warn("状态话题不含末端位姿，VR 控制可能不准确")
    
    def _get_current_arm_state(self) -> Optional[tuple]:
        """
        获取机械臂当前状态（从缓存读取，数据来自 /rm/state_joint_state）
        
        返回: (joint_rad, eef_pose, gripper) 或 None
            joint_rad: 关节角度（弧度）
            eef_pose: [x, y, z, rx, ry, rz]（米/弧度）
            gripper: 夹爪开合度
        """
        with self._state_lock:
            if self._cached_state is None:
                return None
            
            if len(self._cached_state.position) < 7:
                return None
            
            joint_rad = np.array(self._cached_state.position[:7])
            gripper = self._cached_state.position[7] if len(self._cached_state.position) > 7 else 0.0
            eef_pose = self._cached_eef_pose
            
            return joint_rad, eef_pose, gripper
    
    def _on_pose(self, msg: PoseStamped):
        """接收 VR 位姿"""
        with self._lock:
            self._current_pose = msg
            self._vr_last_update = time.time()
            
            # 更新状态
            self.state.current_vr_pos = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ])
            self.state.current_vr_quat = np.array([
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ])
            self.state.last_update_time = time.time()
    
    def _on_trigger(self, msg: Float32):
        """接收 trigger 值"""
        with self._lock:
            old_trigger = self._current_trigger
            self._current_trigger = msg.data
            self.state.current_trigger = msg.data
            
            was_active = old_trigger >= self.trigger_threshold
            is_active = msg.data >= self.trigger_threshold
            
            if not was_active and is_active:
                self._activate_control()
            elif was_active and not is_active:
                self._deactivate_control()
    
    def _on_grip(self, msg: Float32):
        """
        接收 grip 值，实现toggle控制
        
        Grip键（侧面握把）：
        - 按下一次 → 夹爪闭合（0.0）
        - 再按一次 → 夹爪打开（1.0）
        - 使用边缘检测避免重复触发
        """
        should_publish_gripper = False
        with self._lock:
            current_grip = msg.data
            now = time.monotonic()
            
            # 边缘检测：按下阈值和释放阈值分开，兼容只上报小幅 analog grip 的 VR App。
            if (not self._grip_pressed
                    and current_grip >= self.grip_threshold
                    and now - self._last_grip_toggle_time >= self.grip_toggle_cooldown):
                self._grip_pressed = True
                self._last_grip_toggle_time = now
                self._gripper_closed = not self._gripper_closed
                should_publish_gripper = True
                status = "闭合" if self._gripper_closed else "打开"
                self.get_logger().info(
                    f"🤏 夹爪切换: {status} (grip={current_grip:.3f}, threshold={self.grip_threshold:.3f})"
                )
            elif self._grip_pressed and current_grip <= self.grip_release_threshold:
                self._grip_pressed = False
            
            # 更新状态
            self._last_grip_value = current_grip
            self._current_grip = 0.0 if self._gripper_closed else 1.0
            self.state.current_grip = self._current_grip

        if should_publish_gripper:
            self.get_logger().info(
                f"调用夹爪服务: {self.driver_service_prefix}/gripper_toggle_lock"
            )
            self._call_driver_service(
                self.driver_gripper_toggle_client,
                f"{self.driver_service_prefix}/gripper_toggle_lock"
            )
    
    def _activate_control(self):
        """激活控制（trigger 按下）"""
        self.state.is_active = True
        
        # 记录 VR 初始位姿
        self.state.init_vr_pos = self.state.current_vr_pos.copy()
        self.state.init_vr_quat = self.state.current_vr_quat.copy()
        
        # 获取机械臂当前状态
        arm_state = self._get_current_arm_state()
        if arm_state is not None:
            joint_rad, eef_pose, gripper = arm_state
            if eef_pose is not None:
                # 位置
                self.state.init_arm_pos = np.array(eef_pose[:3])
                # 欧拉角转四元数（内旋 xyz，匹配 Realman SDK 的 RPY 约定）
                euler_rad = eef_pose[3:6]
                rot = R.from_euler('xyz', euler_rad)
                self.state.init_arm_quat = rot.as_quat()
                
                self.state.is_initialized = True
                self.get_logger().info(f"VR 控制激活，初始位置: {self.state.init_arm_pos}")
            else:
                self.get_logger().warn("无法获取末端位姿")
                self.state.is_initialized = False
        else:
            self.get_logger().warn("无法获取机械臂状态，VR 控制可能不准确")
            self.state.is_initialized = False
        
        # 重置滤波器
        self.pose_filter.reset()
    
    def _deactivate_control(self):
        """停止控制（trigger 释放）"""
        self.state.is_active = False
        self.state.is_initialized = False
        self.get_logger().info("VR 控制停止")
    
    def _on_timer(self):
        """控制循环"""
        if not self._control_enabled:
            return
        
        with self._lock:
            # 检查 VR 数据是否超时
            if time.time() - self._vr_last_update > 0.5:
                return
            
            # 如果未激活，同步当前状态
            if not self.state.is_active:
                self._sync_arm_state()
                return
            
            # 如果未初始化，跳过
            if not self.state.is_initialized:
                return
            
            # 计算目标笛卡尔位姿
            target_pose = self._calculate_target_pose()
            if target_pose is None:
                return
        
        # 发布控制指令
        self._publish_action(target_pose)
    
    def _sync_arm_state(self):
        """同步机械臂状态（trigger 未按下时）"""
        arm_state = self._get_current_arm_state()
        if arm_state is not None:
            joint_rad, eef_pose, gripper = arm_state
            if eef_pose is not None:
                self.state.init_arm_pos = np.array(eef_pose[:3])
                # 欧拉角转四元数（内旋 xyz，匹配 Realman SDK 的 RPY 约定）
                euler_rad = eef_pose[3:6]
                rot = R.from_euler('xyz', euler_rad)
                self.state.init_arm_quat = rot.as_quat()
        
        # 同步 VR 初始位姿
        self.state.init_vr_pos = self.state.current_vr_pos.copy()
        self.state.init_vr_quat = self.state.current_vr_quat.copy()
    
    def _calculate_target_pose(self) -> Optional[list]:
        """
        计算目标笛卡尔位姿
        
        返回:
            [x, y, z, qw, qx, qy, qz] 格式的位姿列表，或 None
        """
        # 1. 计算位置差分
        position_diff = self.state.current_vr_pos - self.state.init_vr_pos
        
        # 2. 坐标变换
        position_diff_transformed = self.coord_transform.transform_position(position_diff)
        
        # 3. 目标位置
        target_pos = self.state.init_arm_pos + position_diff_transformed
        
        # 4. 目标旋转
        target_quat = self.coord_transform.transform_rotation(
            self.state.current_vr_quat,
            self.state.init_vr_quat,
            self.state.init_arm_quat,
        )
        
        # 5. 滤波
        filtered_pos, filtered_quat = self.pose_filter.filter_pose(target_pos, target_quat)
        
        # 6. 构造目标位姿 [x, y, z, qw, qx, qy, qz]
        # 注意：Realman SDK 的四元数格式是 [qw, qx, qy, qz]
        # scipy 的四元数格式是 [qx, qy, qz, qw]
        qx, qy, qz, qw = filtered_quat
        target_pose = [
            float(filtered_pos[0]),
            float(filtered_pos[1]),
            float(filtered_pos[2]),
            float(qw),  # Realman SDK 格式：qw 在前
            float(qx),
            float(qy),
            float(qz),
        ]
        
        return target_pose
    
    def _publish_action(self, target_pose: list):
        """
        发布控制指令到 /rm/action_pose
        
        参数:
            target_pose: [x, y, z, qw, qx, qy, qz] 格式的位姿
        """
        # 夹爪开合度（grip 值映射到 0-1）
        gripper_open = float(np.clip(self._current_grip, 0.0, 1.0))
        
        # 构造消息：位姿 + 夹爪
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.name = ["x", "y", "z", "qw", "qx", "qy", "qz", "gripper"]
        msg.position = target_pose + [gripper_open]
        
        self.action_pub.publish(msg)
        self._cmd_count += 1
    
    def _call_driver_service(self, client, service_name: str) -> bool:
        """
        调用 driver 服务（非阻塞方式）
        
        参数:
            client: Service 客户端
            service_name: 服务名称（用于日志）
        
        返回:
            是否调用成功
        """
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"服务 {service_name} 不可用")
            return False
        
        try:
            future = client.call_async(Trigger.Request())

            def _log_result(done_future):
                result = done_future.result()
                if result.success:
                    self.get_logger().info(f"✅ {service_name}: {result.message}")
                else:
                    self.get_logger().warn(f"⚠️ {service_name}: {result.message}")

            future.add_done_callback(_log_result)
            return True
        except Exception as e:
            self.get_logger().error(f"❌ {service_name} 调用失败: {e}")
            return False
    
    def _handle_enable(self, request, response):
        """
        启用 VR 控制
        
        同时自动启用 driver 跟随，实现一键启用
        """
        if self._control_enabled:
            response.success = False
            response.message = "VR 控制已启用"
        else:
            # 真机模式下，先启用 driver 跟随
            if not self.dry_run:
                self.get_logger().info("🔗 正在启用 driver 跟随...")
                self._call_driver_service(
                    self.driver_enable_follow_client, 
                    f"{self.driver_service_prefix}/enable_follow"
                )
            
            self._control_enabled = True
            response.success = True
            response.message = "VR 控制已启用，按住 trigger 开始控制"
            self.get_logger().info("🎮 VR 控制已启用")
            self._publish_status()
        return response
    
    def _handle_disable(self, request, response):
        """
        禁用 VR 控制
        
        同时自动禁用 driver 跟随，确保机械臂停止
        """
        if not self._control_enabled:
            response.success = False
            response.message = "VR 控制未启用"
        else:
            self._control_enabled = False
            self.state.is_active = False
            self.state.is_initialized = False
            
            # 真机模式下，同时禁用 driver 跟随
            if not self.dry_run:
                self.get_logger().info("🔗 正在禁用 driver 跟随...")
                self._call_driver_service(
                    self.driver_disable_follow_client,
                    f"{self.driver_service_prefix}/disable_follow"
                )
            
            response.success = True
            response.message = "VR 控制已禁用"
            self.get_logger().info("⏸️ VR 控制已禁用")
            self._publish_status()
        return response
    
    def _log_stats(self):
        """输出统计信息"""
        status = "启用" if self._control_enabled else "禁用"
        active = "激活" if self.state.is_active else "待机"
        
        self.get_logger().info(
            f"[VR 遥操作] {status}/{active} | "
            f"指令: {self._cmd_count}"
        )
        
        # 定期发布状态
        self._publish_status()
    
    def _publish_status(self):
        """发布 VR 遥操作状态供 Web 控制面板使用"""
        msg = String()
        msg.data = json.dumps({
            "control_enabled": self._control_enabled,
            "active": self.state.is_active if hasattr(self, 'state') else False
        })
        self.status_pub.publish(msg)
    
    def destroy_node(self):
        """清理资源"""
        self.get_logger().info("正在关闭 VR 遥操作节点...")
        super().destroy_node()


def main():
    rclpy.init()
    node = VRTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
