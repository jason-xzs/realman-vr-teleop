import atexit
import json
import os
import signal
import socket
import sys
import threading
import time
from queue import Queue, Empty
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .hardware import RealmanDriver, UDPStateReceiver
from .utils import ROBOT_JOINT_NAMES, gripper_open_from_pos

# Realman SDK IK 参数构造类（lazy import 兼容性，参考 vr_utils.py:323 已用例）
try:
    from Robotic_Arm.rm_robot_interface import rm_inverse_kinematics_params_t
except ImportError:  # 测试或 SDK 不可用环境下保持模块可 import
    rm_inverse_kinematics_params_t = None  # type: ignore[assignment]


# === 诊断日志（写文件，用于对比 async_client / dagger_node 链路）===
# 日志写到 dagger/logs/ 目录，和其他诊断日志放一起
_DRIVER_DIAG_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))))),
    "dagger", "logs"
)
_DRIVER_DIAG_LOG_PATH = os.path.join(_DRIVER_DIAG_LOG_DIR, "driver_diag.log")
_driver_diag_log_file = None

def _driver_diag_log(msg: str):
    """写诊断日志到文件"""
    global _driver_diag_log_file
    if _driver_diag_log_file is None:
        os.makedirs(os.path.dirname(_DRIVER_DIAG_LOG_PATH), exist_ok=True)
        _driver_diag_log_file = open(_DRIVER_DIAG_LOG_PATH, "a")
        _driver_diag_log_file.write(f"\n{'='*60}\n")
        _driver_diag_log_file.write(f"[driver_node] Session started: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        _driver_diag_log_file.write(f"{'='*60}\n")
    _now = time.time()
    _ms = int((_now % 1) * 1000)
    line = f"[{time.strftime('%H:%M:%S', time.localtime(_now))}.{_ms:03d}] {msg}\n"
    _driver_diag_log_file.write(line)
    _driver_diag_log_file.flush()


# 软启动相关常量
SOFT_START_POSITION_THRESHOLD_DEG = 2.0  # 位置误差阈值（度），所有关节误差小于此值视为到位
SOFT_START_CHECK_INTERVAL = 0.1          # 检测间隔（秒）


def get_local_ip() -> str:
    """
    获取本机局域网 IP 地址。
    
    用于配置机械臂 UDP 上报的目标 IP。
    通过创建一个临时 UDP 连接来获取本机的出口 IP。
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        local_ip = sock.getsockname()[0]
        sock.close()
        return local_ip
    except Exception:
        return "127.0.0.1"


def _pose_to_target_joint_deg(
    arm,
    pose: list,
    q_in_deg: list,
    arm_angle: float,
) -> Tuple[bool, Optional[list]]:
    """把 base 系绝对 pose 经臂角法 IK（RM-75 7-DoF 专用）转成目标关节角度（度）。

    使用 rm_algo_inverse_kinematics_rm75_for_arm_angle 而非通用 IK：
    - RM-75 是 7-DoF 冗余臂，每个末端 pose 对应 1 维零空间（手臂角参数化）。
    - 通用 IK 不严格约束手臂角，会在 home 等多解附近跨解（实测 J1 跳 ±75°）。
    - 臂角法锁定零空间 → 唯一解 → 关节空间命令连续。

    Args:
        arm: RoboticArm 实例。
        pose: 6 维 [x,y,z,rx,ry,rz]（米/弧度）或 7 维 [x,y,z,qw,qx,qy,qz]。
        q_in_deg: 当前关节角度参考（度，长度 7）。
        arm_angle: RM-75 手臂角约束（度），由
            rm_algo_calculate_arm_angle_from_config_rm75 算得，整 episode lock 不变。

    Returns:
        (True, q_out_deg list[7]) 求解成功；(False, None) 求解失败。

    Raises:
        ValueError: pose 维度不是 6 或 7。
    """
    if len(pose) == 6:
        flag = 1  # 欧拉角
    elif len(pose) == 7:
        flag = 0  # 四元数 [qw,qx,qy,qz]
    else:
        raise ValueError(f"pose dim must be 6 or 7, got {len(pose)}")

    ik_params = rm_inverse_kinematics_params_t(
        q_in=list(q_in_deg),
        q_pose=list(pose),
        flag=flag,
    )
    ret, q_out_deg = arm.rm_algo_inverse_kinematics_rm75_for_arm_angle(ik_params, arm_angle)
    if ret != 0:
        return False, None
    return True, list(q_out_deg)


class RealmanDriverNode(Node):
    """
    睿尔曼机械臂 ROS2 驱动节点。
    
    功能说明：
    ----------
    1. 订阅动作话题，发送控制指令到机械臂
    2. 通过 UDP 接收机械臂状态，发布到状态话题
    
    设计原则：
    ----------
    - 关节控制：在主回调中同步执行（CANFD 透传，真正非阻塞）
    - 夹爪控制：在独立线程中异步执行（TCP 通信可能阻塞，不影响关节控制）
    - 状态读取：从 UDP 缓存读取，不阻塞
    
    通信架构：
    ----------
    - 关节控制：CANFD 透传（movej_canfd，非阻塞，~1ms）
    - 夹爪控制：TCP 协议（异步线程，不阻塞主循环）
    - 状态上报：UDP 协议（低延迟，主动推送）
    """
    
    def __init__(self):
        super().__init__("realman_driver_node")
        
        # ===== 机械臂 TCP 连接参数 =====
        self.declare_parameter("ip", "192.168.1.18")
        self.declare_parameter("port", 8080)
        self.declare_parameter("follow", False)
        self.declare_parameter("high_follow", False)  # 软启动：最终是否切换到高跟随模式
        self.declare_parameter("canfd_traj_mode", 0)
        self.declare_parameter("canfd_radio", 30)
        self.declare_parameter("rm_plus_baud", 460800)
        self.declare_parameter("dry_run", False)
        
        # ===== UDP 状态上报参数 =====
        self.declare_parameter("udp_port", 8089)
        self.declare_parameter("udp_cycle", 1)
        self.declare_parameter("local_ip", "")
        
        # ===== 话题与发布参数 =====
        self.declare_parameter("state_hz", 50.0)
        self.declare_parameter("action_topic", "/rm/action_joint_state")
        self.declare_parameter("action_pose_topic", "/rm/action_pose")  # 笛卡尔位姿话题
        self.declare_parameter("state_topic", "/rm/state_joint_state")
        self.declare_parameter("frame_id", "rm_base")
        self.declare_parameter("joint_names", ROBOT_JOINT_NAMES)
        self.declare_parameter("service_prefix", "/driver")
        self.declare_parameter("status_topic", "/driver/status")
        self.declare_parameter("gripper_lock_status_topic", "/driver/gripper_lock_status")
        
        # ===== 夹爪控制参数 =====
        # deadband=20 与 async_client 的 GripperAsyncController 一致，避免夹爪振荡
        self.declare_parameter("gripper_deadband", 20)
        self.declare_parameter("gripper_lock_default_closed", False)

        # ===== Home 位姿参数 =====
        self.declare_parameter("home_joint_deg", [-0.263, -0.505, -0.157, 43.953, 1.225, 121.751, -0.187])
        self.declare_parameter("home_velocity", 20)

        # ===== IK 控制参数 =====
        self.declare_parameter("pose_to_joints_via_ik", True)

        # ===== 读取参数 =====
        ip = self.get_parameter("ip").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value
        follow = self.get_parameter("follow").get_parameter_value().bool_value
        self.high_follow = self.get_parameter("high_follow").get_parameter_value().bool_value
        canfd_traj_mode = self.get_parameter("canfd_traj_mode").get_parameter_value().integer_value
        canfd_radio = self.get_parameter("canfd_radio").get_parameter_value().integer_value
        rm_plus_baud = self.get_parameter("rm_plus_baud").get_parameter_value().integer_value
        dry_run = self.get_parameter("dry_run").get_parameter_value().bool_value
        
        # 软启动模式：如果 high_follow=True，启动时强制使用低跟随，后续切换
        if self.high_follow:
            self.get_logger().info("🚀 软启动模式：启动时使用低跟随，待机械臂到位后切换到高跟随")
            follow = False  # 强制使用低跟随启动
        
        # 软启动状态
        self._high_follow_active = False  # 高跟随是否已激活
        self._soft_start_thread: Optional[threading.Thread] = None
        
        # 跟随使能状态（真机模式下默认禁用，需要用户手动启用）
        self._follow_enabled = dry_run  # 仿真模式默认启用，真机模式默认禁用
        
        # UDP 参数
        udp_port = self.get_parameter("udp_port").get_parameter_value().integer_value
        udp_cycle = self.get_parameter("udp_cycle").get_parameter_value().integer_value
        local_ip = self.get_parameter("local_ip").get_parameter_value().string_value
        
        state_hz = self.get_parameter("state_hz").get_parameter_value().double_value
        action_topic = self.get_parameter("action_topic").get_parameter_value().string_value
        action_pose_topic = self.get_parameter("action_pose_topic").get_parameter_value().string_value
        state_topic = self.get_parameter("state_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.service_prefix = self.get_parameter("service_prefix").get_parameter_value().string_value.rstrip("/") or "/driver"
        self.status_topic = self.get_parameter("status_topic").get_parameter_value().string_value
        self.gripper_lock_status_topic = self.get_parameter("gripper_lock_status_topic").get_parameter_value().string_value
        
        # 夹爪死区：位置变化小于此值时不发送指令，避免频繁通信
        self.gripper_deadband = self.get_parameter("gripper_deadband").get_parameter_value().integer_value
        self._last_gripper_cmd: Optional[int] = None
        # 夹爪锁定状态（hex 任务用；香蕉任务 yaml 默认 false）
        self._gripper_lock_closed: bool = self.get_parameter(
            "gripper_lock_default_closed"
        ).get_parameter_value().bool_value

        # Home 位姿参数
        self.home_joint_deg = list(self.get_parameter("home_joint_deg").value)
        self.home_velocity = self.get_parameter("home_velocity").get_parameter_value().integer_value

        # IK 控制参数
        self.pose_to_joints_via_ik = (
            self.get_parameter("pose_to_joints_via_ik").get_parameter_value().bool_value
        )

        # ===== 夹爪异步控制队列和线程 =====
        self._gripper_queue: Queue = Queue(maxsize=1)  # 只保留最新指令
        self._gripper_stop_event = threading.Event()
        self._gripper_thread: Optional[threading.Thread] = None

        # ===== 初始化机械臂驱动（TCP，用于发送控制指令） =====
        self._rm_plus_baud = rm_plus_baud  # 保存用于日志显示
        self.driver = RealmanDriver(
            ip=ip,
            port=port,
            follow=follow,
            canfd_traj_mode=canfd_traj_mode,
            canfd_radio=canfd_radio,
            rm_plus_baud=rm_plus_baud,
            dry_run=dry_run,
        )
        
        # 打印 RM+ 协议状态
        if not dry_run and rm_plus_baud > 0:
            self.get_logger().info(f"🔌 RM+ 生态协议已开启 (波特率={rm_plus_baud})，夹爪控制可用")

        # ===== 初始化 UDP 状态接收器 =====
        if not local_ip:
            local_ip = get_local_ip()
        self.get_logger().info(f"本机 IP: {local_ip}，UDP 监听端口: {udp_port}")
        
        self.udp_receiver = UDPStateReceiver(port=udp_port, arm_dof=7)
        self.udp_receiver.start()
        
        # 配置机械臂开启 UDP 上报
        if not dry_run:
            success = self.driver.enable_udp_realtime_push(
                target_ip=local_ip,
                target_port=udp_port,
                cycle=udp_cycle,
                enable=True,
            )
            if success:
                self.get_logger().info(f"机械臂 UDP 上报已开启，周期: {udp_cycle * 5}ms ({1000 // (udp_cycle * 5)}Hz)")
            else:
                raise RuntimeError("机械臂 UDP 上报配置失败，请检查网络连接")

        # ===== 创建 ROS2 订阅/发布 =====
        # 夹爪锁定状态发布（供 Web UI 实时刷新按钮文字）
        self.gripper_lock_status_pub = self.create_publisher(
            Bool, self.gripper_lock_status_topic, 1
        )
        self.subscription = self.create_subscription(JointState, action_topic, self._on_action, 10)
        self.pose_subscription = self.create_subscription(JointState, action_pose_topic, self._on_action_pose, 10)
        self.publisher = self.create_publisher(JointState, state_topic, 10)
        
        # ===== 发布 driver 状态（供 UI 监控） =====
        self._dry_run = dry_run
        self._driver_status_pub = self.create_publisher(String, self.status_topic, 10)
        self._driver_status_timer = self.create_timer(0.5, self._publish_driver_status)

        # ===== 创建状态发布定时器 =====
        period = 1.0 / state_hz if state_hz > 0 else 0.02
        self.timer = self.create_timer(period, self._on_timer)
        self.last_state: Optional[np.ndarray] = None

        # IK pose→joints 路径状态
        self._last_ik_joint_deg: Optional[list] = None  # IK hold-last 缓冲
        self._ik_fail_count: int = 0                    # 连续 IK 失败计数
        self._arm_angle: Optional[float] = None         # RM-75 手臂角（度），首帧 IK 时延迟初始化并 lock
        
        # 统计日志定时器（每 10 秒打印一次 UDP 接收状态）
        # 注意：只在软启动完成后才开始打印，避免启动阶段日志干扰
        self._udp_log_enabled = not self.high_follow  # 非软启动模式直接启用
        self.stats_timer = self.create_timer(10.0, self._log_udp_stats)
        
        # ===== 启动夹爪异步控制线程 =====
        self._gripper_thread = threading.Thread(target=self._gripper_control_loop, daemon=True)
        self._gripper_thread.start()
        
        self.get_logger().info(
            f"  pose_to_joints_via_ik: {self.pose_to_joints_via_ik}"
        )
        self.get_logger().info("驱动节点已启动（关节同步 + 夹爪异步）")
        if not dry_run:
            self.get_logger().warn("⚠️ 真机模式：跟随已禁用，请在 Web UI 点击「启用跟随」按钮")
        
        # ===== 创建 Service =====
        self.high_follow_srv = self.create_service(
            Trigger, f"{self.service_prefix}/enable_high_follow", self._handle_enable_high_follow
        )
        self.enable_follow_srv = self.create_service(
            Trigger, f"{self.service_prefix}/enable_follow", self._handle_enable_follow
        )
        self.disable_follow_srv = self.create_service(
            Trigger, f"{self.service_prefix}/disable_follow", self._handle_disable_follow
        )
        self.move_to_home_srv = self.create_service(
            Trigger, f"{self.service_prefix}/move_to_home", self._handle_move_to_home
        )
        self.gripper_toggle_srv = self.create_service(
            Trigger, f"{self.service_prefix}/gripper_toggle_lock", self._handle_gripper_toggle_lock
        )

        # ===== 软启动：如果需要高跟随，启动监控线程 =====
        if self.high_follow and not dry_run:
            self._soft_start_thread = threading.Thread(target=self._soft_start_monitor, daemon=True)
            self._soft_start_thread.start()

        # 发布初始锁定状态
        self._publish_gripper_lock_status()

        # 上电即锁闭：主动入队一次（香蕉任务 _gripper_lock_closed=False，跳过）
        if self._gripper_lock_closed and not self._dry_run:
            self.get_logger().info("🔒 夹爪锁定模式启动，立即发送闭合指令")
            try:
                while not self._gripper_queue.empty():
                    try:
                        self._gripper_queue.get_nowait()
                    except Empty:
                        break
                self._gripper_queue.put_nowait(0)
            except Exception as e:
                self.get_logger().warn(f"启动闭合指令入队失败: {e}")

    def _on_action(self, msg: JointState):
        """
        处理动作指令回调。

        收到动作指令后：
        1. 同步发送关节指令（CANFD 透传，真正非阻塞，~1ms）
        2. 异步发送夹爪指令（放入队列，由独立线程处理，不阻塞本回调）

        安全机制：真机模式下，必须先启用跟随才会发送指令。
        """
        if not msg.position or len(msg.position) < 7:
            return

        # 安全检查：跟随未启用时不发送关节指令
        if not self._follow_enabled:
            return

        # 1. 发送关节指令（同步，CANFD 透传是真正非阻塞的）
        cmd_rad = np.array(msg.position[:7], dtype=float)
        cmd_deg = np.degrees(cmd_rad)

        # 检测新介入（间隔 >200ms 认为是新一轮 POLICY 介入）
        if not hasattr(self, '_joint_last_time'):
            self._joint_last_time = 0.0
            self._joint_burst_count = 0
            self._joint_recv_count = 0
        self._joint_recv_count += 1

        _t0 = time.monotonic()
        _gap = _t0 - self._joint_last_time if self._joint_last_time > 0 else 0
        if _gap > 0.2:
            self._joint_burst_count = 0
            # 过渡帧：用缓存的当前关节位置发送 2 帧 movej_canfd，
            # 让 Realman 控制器从 movep 内部状态平滑过渡到 movej 模式，
            # 避免首帧 movej_canfd 被静默丢弃
            if self.last_state is not None:
                current_joints_rad = self.last_state[:7]
                current_joints_deg = np.degrees(current_joints_rad).tolist()
                if any(v != 0.0 for v in current_joints_deg):
                    for _i in range(2):
                        self.driver.send_joint_deg(current_joints_deg)
                        time.sleep(0.005)
                    _driver_diag_log(
                        f"[JOINT-TRANSITION] 发送 2 帧过渡帧, gap={_gap*1000:.0f}ms, "
                        f"joints=[{','.join(f'{v:.2f}' for v in current_joints_deg)}]")
        self._joint_burst_count += 1
        self._joint_last_time = _t0

        self.driver.send_joint_deg(cmd_deg)

        # 2. 发送夹爪指令（异步，放入队列由独立线程处理）
        if len(msg.position) > 7:
            gripper_open = float(msg.position[7])
            # 锁定模式：硬覆盖 incoming gripper 值
            if self._gripper_lock_closed:
                gripper_open = 0.0
            gripper_pos = int(np.clip(gripper_open, 0.0, 1.0) * 1000.0)

            if not hasattr(self, '_action_recv_count'):
                self._action_recv_count = 0
            self._action_recv_count += 1
            will_send = (self._last_gripper_cmd is None or
                         abs(gripper_pos - self._last_gripper_cmd) >= self.gripper_deadband)

            # === DIAG-COMPARE: 每帧夹爪命令接收写文件 ===
            _driver_diag_log(
                f"[RECV-GRIP] #{self._action_recv_count}: "
                f"grip_open={gripper_open:.4f}, grip_pos={gripper_pos}, "
                f"last_cmd={self._last_gripper_cmd}, deadband={self.gripper_deadband}, "
                f"will_send={will_send}"
            )

            # 死区判断：只有当位置变化超过阈值时才放入队列
            if self._last_gripper_cmd is None or abs(gripper_pos - self._last_gripper_cmd) >= self.gripper_deadband:
                # 非阻塞放入队列，如果队列满则丢弃旧指令
                try:
                    # 先清空队列中的旧指令（只保留最新）
                    while not self._gripper_queue.empty():
                        try:
                            self._gripper_queue.get_nowait()
                        except Empty:
                            break
                    self._gripper_queue.put_nowait(gripper_pos)
                    self._last_gripper_cmd = gripper_pos
                except Exception:
                    pass  # 队列操作失败时静默忽略

    def _queue_gripper_from_open(self, gripper_open: float) -> None:
        """Queue latest gripper command without blocking arm control."""
        if self._gripper_lock_closed:
            gripper_open = 0.0
        gripper_pos = int(np.clip(gripper_open, 0.0, 1.0) * 1000.0)

        if self._last_gripper_cmd is None or abs(gripper_pos - self._last_gripper_cmd) >= self.gripper_deadband:
            try:
                while not self._gripper_queue.empty():
                    try:
                        self._gripper_queue.get_nowait()
                    except Empty:
                        break
                self._gripper_queue.put_nowait(gripper_pos)
                self._last_gripper_cmd = gripper_pos
            except Exception:
                pass

    def _on_action_pose(self, msg: JointState):
        """
        处理笛卡尔位姿指令回调（用于 VR 遥操作 / DAgger 策略推理）。

        通过 msg.name 字段显式识别格式（优先），回退到长度判断（向后兼容）：

        欧拉角格式（name 含 "rx"）：
        - position[0:6] = [x, y, z, rx, ry, rz]（米/弧度）
        - position[6]   = 夹爪开合度（0-1，可选）

        四元数格式（name 含 "qw"）：
        - position[0:7] = [x, y, z, qw, qx, qy, qz]
        - position[7]   = 夹爪开合度（0-1，可选）

        安全机制：真机模式下，必须先启用跟随才会发送指令。
        """
        if not msg.position or len(msg.position) < 6:
            return
        
        # 安全检查：跟随未启用时不发送指令
        if not self._follow_enabled:
            return
        
        # 判断位姿格式：优先通过 msg.name 字段显式识别，回退到长度判断
        names = list(msg.name) if msg.name else []
        if "rx" in names:
            # 欧拉角格式：[x, y, z, rx, ry, rz, (gripper)]
            pose = list(msg.position[:6])
            gripper_idx = 6
        elif "qw" in names:
            # 四元数格式：[x, y, z, qw, qx, qy, qz, (gripper)]
            pose = list(msg.position[:7])
            gripper_idx = 7
        elif len(msg.position) >= 7 and len(msg.position) <= 8:
            # 回退：无 name 字段时按长度判断（向后兼容）
            pose = list(msg.position[:7])
            gripper_idx = 7
        elif len(msg.position) == 6:
            pose = list(msg.position[:6])
            gripper_idx = 6
        else:
            self.get_logger().warning(f"位姿格式错误: 长度={len(msg.position)}, names={names}")
            return

        # 夹爪控制独立于 IK/位姿控制，避免 IK 冷启动或单帧失败时丢失夹爪命令。
        if len(msg.position) > gripper_idx:
            self._queue_gripper_from_open(float(msg.position[gripper_idx]))
        
        # 1. 发送指令路径：默认走 IK→movej_canfd（消除 movej/movep 切换抖动）
        if not hasattr(self, '_pose_recv_count'):
            self._pose_recv_count = 0
        self._pose_recv_count += 1

        if self.pose_to_joints_via_ik:
            # 新路径：SDK 本地 IK → movej_canfd
            if self.last_state is None:
                # 冷启动保护：UDP 状态尚未到达，跳过该帧
                return
            if self.driver.arm is None:
                # dry_run 模式下 arm 未连接，IK 不可用，跳过该帧
                return
            # IK 种子：优先用上一帧 IK 目标关节，保证关节空间连续性，
            # 防止 home 等多解附近 measured 滞后导致 J1 跨解抖动。
            # 仅冷启动第一帧 fallback 到 measured 关节做初始种子。
            if self._last_ik_joint_deg is not None:
                seed_joint_deg = self._last_ik_joint_deg
            else:
                seed_joint_deg = np.degrees(self.last_state[:7]).tolist()

            # RM-75 手臂角：首帧用 measured 关节算出并 lock，整 episode 不变，
            # 锁定 7-DoF 冗余零空间 → 臂角法 IK 解唯一连续。
            if self._arm_angle is None:
                measured_joint_deg = np.degrees(self.last_state[:7]).tolist()
                aa_ret, aa_value = self.driver.arm.rm_algo_calculate_arm_angle_from_config_rm75(
                    measured_joint_deg
                )
                if aa_ret != 0:
                    self.get_logger().warning(
                        f"手臂角计算失败 ret={aa_ret}，跳帧（待下一帧 measured 重试）",
                        throttle_duration_sec=2.0,
                    )
                    return
                self._arm_angle = float(aa_value)
                self.get_logger().info(
                    f"🔧 RM-75 手臂角已锁定: {self._arm_angle:+.3f}°（基于 measured 关节）"
                )

            try:
                ok, target_joint_deg = _pose_to_target_joint_deg(
                    self.driver.arm, pose, seed_joint_deg, self._arm_angle
                )
            except (ValueError, TypeError) as e:
                # ValueError: pose 维度异常；TypeError: SDK 不可用导致 IK 类构造失败
                self.get_logger().warning(
                    f"IK 调用异常: {e}",
                    throttle_duration_sec=2.0,
                )
                return

            rejected = False
            if ok:
                # post-IK 跨解保护：偶发臂角法 IK 输出与 hold seed 任一关节差超过 30°
                # 视为单帧跨解（实测 SDK 在 ~3% 帧会出现，会污染 measured state → action[t]=state[t+1] 训练数据）
                # reject 后不发命令、不更新 hold，下一帧用旧 seed 重新 IK 通常能恢复。
                if self._last_ik_joint_deg is not None:
                    max_delta_deg = max(
                        abs(t - s) for t, s in zip(target_joint_deg, self._last_ik_joint_deg)
                    )
                    if max_delta_deg > 30.0:
                        self.get_logger().warning(
                            f"IK 跨解拒绝: max_delta={max_delta_deg:.1f}°，沿用上一帧目标关节",
                            throttle_duration_sec=2.0,
                        )
                        target_joint_deg = self._last_ik_joint_deg
                        rejected = True

                if not rejected:
                    self._ik_fail_count = 0
                    self._last_ik_joint_deg = list(target_joint_deg)
            else:
                # IK 调用失败：hold last solution
                self._ik_fail_count += 1
                if self._ik_fail_count >= 5:
                    self.get_logger().warning(
                        f"IK 连续失败 {self._ik_fail_count} 帧，沿用上一帧关节解",
                        throttle_duration_sec=2.0,
                    )
                if self._last_ik_joint_deg is None:
                    return  # 第一次就失败，没有可 hold 的解
                target_joint_deg = self._last_ik_joint_deg

            # 走 joints 路径（与 _on_action 同一通道，无 movej/movep 切换）
            self.driver.send_joint_deg(np.array(target_joint_deg, dtype=float))
        else:
            # 旧路径：movep_canfd（pose_to_joints_via_ik=false 时保留作 A/B 对照）
            result = self.driver.send_pose_canfd(pose)
            if result != 0:
                self.get_logger().warning(
                    f"位姿指令发送失败: result={result}, count={self._pose_recv_count}"
                )

    
    def _gripper_control_loop(self):
        """
        夹爪控制线程：独立于主循环，不阻塞关节控制。

        从队列中取出夹爪指令并发送，TCP 通信延迟只影响本线程。
        """
        gripper_cmd_count = 0
        import time as _time
        while not self._gripper_stop_event.is_set():
            try:
                # 阻塞等待队列中的指令，超时 100ms
                _t_wait_start = _time.monotonic()
                gripper_pos = self._gripper_queue.get(timeout=0.1)
                _t_wait_done = _time.monotonic()

                # 发送夹爪指令（这里的阻塞不会影响主循环）
                _t_send_start = _time.monotonic()
                result = self.driver.send_gripper(gripper_pos)
                _t_send_done = _time.monotonic()
                gripper_cmd_count += 1

                # === DIAG-COMPARE: 每次夹爪实际发送写文件 ===
                _send_ms = (_t_send_done - _t_send_start) * 1000
                _wait_ms = (_t_wait_done - _t_wait_start) * 1000
                _driver_diag_log(
                    f"[GRIP-SEND] #{gripper_cmd_count}: "
                    f"pos={gripper_pos}, result={result}, "
                    f"send={_send_ms:.1f}ms, wait={_wait_ms:.1f}ms"
                )
                if gripper_cmd_count <= 10 or gripper_cmd_count % 20 == 0:
                    self.get_logger().info(
                        f"[DIAG-GRIP-SEND] #{gripper_cmd_count}: "
                        f"pos={gripper_pos}, result={result}, latency={_send_ms:.1f}ms")
            except Empty:
                continue
            except Exception as exc:
                # 出错时不崩溃，继续运行
                self.get_logger().warning(f"夹爪控制异常: {exc}")

    def _on_timer(self):
        """
        定时器回调：从 UDP 缓存读取状态并发布到 ROS2 话题。
        
        UDP 模式下读取是瞬时的（从内存缓存读取），不会阻塞。
        状态向量为 14 维：7 关节角度（弧度）+ 1 夹爪 + 6 末端位姿（米/弧度）
        """
        # 从 UDP 缓存读取最新状态（含末端位姿）
        joint_deg, gripper_act, eef_pose = self.udp_receiver.read_full_state()

        # 如果没有数据且没有历史状态，跳过
        if joint_deg is None and self.last_state is None:
            return

        # 如果当前没有数据但有历史状态，使用历史状态
        if joint_deg is None and self.last_state is not None:
            state_vec = self.last_state.copy()
        else:
            # 构造状态向量：7 个关节角度（弧度）+ 1 个夹爪开合度 + 6 个末端位姿
            joint_rad = np.radians(joint_deg[:7]).astype(np.float32)
            if gripper_act is None:
                gripper_act = 0
            gripper_open_udp = gripper_open_from_pos(gripper_act)

            # BUG-018v2 修复: 直接用 UDP RM+ gripper 数据
            # - rm_get_gripper_state() 读的是标准 Modbus 夹爪，不反映 RM+ 夹爪位置
            # - UDP rm_plus_state.pos[0] 才是 RM+ 夹爪真实位置
            # - 与 async_inference_client 的 UDP source 行为一致
            gripper_source = "UDP_rmplus"
            gripper_open = gripper_open_udp
            # 回退: UDP gripper 为 0 且有命令记录时，用 last_cmd
            if gripper_act == 0 and self._last_gripper_cmd is not None:
                gripper_open = gripper_open_from_pos(self._last_gripper_cmd)
                gripper_source = "last_cmd_fallback"

            # 末端位姿：如果没有则用零填充
            if eef_pose is None:
                eef_pose = np.zeros(6, dtype=np.float32)

            state_vec = np.concatenate([joint_rad, [gripper_open], eef_pose.astype(np.float32)])

            # === DIAG-COMPARE: 每帧 gripper 来源写文件 ===
            if not hasattr(self, '_state_pub_count'):
                self._state_pub_count = 0
                self._tcp_actpos_diag = None
            self._state_pub_count += 1
            _driver_diag_log(
                f"[PUB-STATE] #{self._state_pub_count}: "
                f"gripper={gripper_open:.4f}, source={gripper_source}, "
                f"udp_raw={gripper_act}, last_cmd={self._last_gripper_cmd}"
            )
            # 每 200 次读一次 TCP actpos 做对比（不影响发布值）
            if self._state_pub_count % 200 == 0 and not self._dry_run and self.driver.arm is not None:
                try:
                    ret_code, grip_info = self.driver.arm.rm_get_gripper_state()
                    if ret_code == 0 and isinstance(grip_info, dict):
                        self._tcp_actpos_diag = grip_info.get('actpos', None)
                except Exception:
                    pass
            if self._state_pub_count <= 30 or self._state_pub_count % 200 == 0:
                _tcp_diag = f"{float(self._tcp_actpos_diag)/1000.0:.4f}" if self._tcp_actpos_diag is not None else "N/A"
                self.get_logger().info(
                    f"[DIAG-GRIP-v2] #{self._state_pub_count}: "
                    f"PUBLISH={gripper_open:.4f}, source={gripper_source}, "
                    f"udp_raw={gripper_act}, udp_open={gripper_open_udp:.4f}, "
                    f"tcp_actpos={_tcp_diag}, "
                    f"last_cmd={self._last_gripper_cmd}")

        self.last_state = state_vec
        
        # 发布状态消息
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.name = self.joint_names
        msg.position = list(state_vec.astype(float))
        self.publisher.publish(msg)
    
    def _log_udp_stats(self):
        """定时打印 UDP 接收统计信息（每 10 秒一次）。"""
        # dry_run 模式或软启动未完成时不打印日志
        if self._dry_run or not self._udp_log_enabled:
            return
        stats = self.udp_receiver.get_stats()
        is_ok = self.udp_receiver.is_receiving(timeout=1.0)
        status = "正常" if is_ok else "⚠️ 超时"
        follow_status = "高跟随" if self._high_follow_active else "低跟随"
        self.get_logger().info(
            f"[UDP] {status} | 已接收: {stats['recv_count']} 条 | 错误: {stats['error_count']} 次 | 模式: {follow_status}"
        )
    
    def _publish_driver_status(self):
        """发布 driver 状态（供 UI 监控）。"""
        msg = String()
        msg.data = json.dumps({
            "dry_run": self._dry_run,
            "follow_enabled": self._follow_enabled,  # 跟随是否已启用
            "high_follow_enabled": self.high_follow,  # 配置中是否启用软启动
            "high_follow_active": self._high_follow_active,  # 当前是否已切换到高跟随
        })
        self._driver_status_pub.publish(msg)

    def _handle_enable_high_follow(self, request, response):
        """Service 回调：启用高跟随模式。"""
        if self._high_follow_active:
            response.success = False
            response.message = "高跟随模式已启用"
        else:
            self.driver.set_follow_mode(True)
            self._high_follow_active = True
            self._udp_log_enabled = True  # 软启动完成，启用 UDP 日志
            self.get_logger().info("🎉 已通过 Service 切换到高跟随模式")
            response.success = True
            response.message = "已切换到高跟随模式"
        return response

    def _handle_enable_follow(self, request, response):
        """Service 回调：启用跟随（真机模式安全开关）。"""
        if self._follow_enabled:
            response.success = False
            response.message = "跟随已启用"
        else:
            self._follow_enabled = True
            self.get_logger().info("🤖 跟随已启用，机械臂开始跟随外骨骼")
            response.success = True
            response.message = "跟随已启用，请确保外骨骼已就位"
        return response

    def _handle_disable_follow(self, request, response):
        """Service 回调：禁用跟随（让操作者休息）。"""
        if not self._follow_enabled:
            response.success = False
            response.message = "跟随未启用"
        else:
            self._follow_enabled = False
            self.get_logger().info("⏸️ 跟随已停止，操作者可以休息")
            response.success = True
            response.message = "跟随已停止，可以休息"
        return response

    def _handle_move_to_home(self, request, response):
        """Service 回调：移动到 home 位姿（阻塞式）。"""
        # Sanity check: home 不能是占位 [0]*7
        if all(abs(v) < 1e-9 for v in self.home_joint_deg):
            response.success = False
            response.message = (
                "home_joint_deg 全为 0，可能未标定。"
                "如果是 hex 任务，请检查 yaml 的 hex_home_joint_deg 字段后重启。"
            )
            self.get_logger().error(response.message)
            return response

        # 记录回home前的跟随状态，用于之后恢复
        was_follow_enabled = self._follow_enabled

        # 回home前禁用跟随，防止VR输入干扰回home运动
        if self._follow_enabled:
            self._follow_enabled = False
            self.get_logger().info("⏸️ 回 home 前自动禁用跟随")

        self.get_logger().info(f"🏠 开始移动到 home 位姿: {self.home_joint_deg}")

        # 夹爪锁定模式（hex 任务）下不要张开夹爪——否则刚夹住的六角块会掉下来
        success, msg = self.driver.move_to_home(
            self.home_joint_deg,
            self.home_velocity,
            open_gripper=not self._gripper_lock_closed,
        )

        if success:
            self.get_logger().info(f"✅ {msg}")
        else:
            self.get_logger().error(f"❌ {msg}")

        # 锁定模式下，home 后主动重发一次闭合指令——保证夹爪状态确定。
        # hardware.move_to_home 的 open_gripper=False 路径不发任何夹爪指令，
        # 若 movej 期间发生 SDK reset / RM+ 重连等小概率事件让夹爪状态漂移，
        # 这里的幂等强制状态可兜底（与启动时主动入队 0 对称）。
        if success and self._gripper_lock_closed and not self._dry_run:
            try:
                while not self._gripper_queue.empty():
                    try:
                        self._gripper_queue.get_nowait()
                    except Empty:
                        break
                self._gripper_queue.put_nowait(0)
            except Exception as e:
                self.get_logger().warn(f"home 后重发闭合指令失败: {e}")

        # 恢复之前的跟随状态，实现无缝控制
        if was_follow_enabled:
            self._follow_enabled = True
            # 回 home 改变了关节配置，arm_angle 与 IK seed 失效，置 None 让下一帧 IK 用新 measured 重新初始化
            self._arm_angle = None
            self._last_ik_joint_deg = None
            self.get_logger().info("🔗 回 home 完成，自动恢复跟随")

        response.success = success
        response.message = msg
        return response

    def _handle_gripper_toggle_lock(self, request, response):
        """切换夹爪锁定状态（Web UI 按钮触发）。

        - True → False：解锁，立即发开（gripper=1000）
        - False → True：锁定，立即发闭（gripper=0）

        toggle 是用户显式指令，绕过 deadband，必须执行。
        """
        self._gripper_lock_closed = not self._gripper_lock_closed
        target_pos = 0 if self._gripper_lock_closed else 1000

        if not self._dry_run:
            try:
                # 清空队列旧指令，立即下发新指令
                while not self._gripper_queue.empty():
                    try:
                        self._gripper_queue.get_nowait()
                    except Empty:
                        break
                self._gripper_queue.put_nowait(target_pos)
                self._last_gripper_cmd = target_pos
            except Exception as e:
                response.success = False
                response.message = f"夹爪指令入队失败: {e}"
                return response

        state_str = "🔒 锁闭" if self._gripper_lock_closed else "🔓 解锁打开"
        self.get_logger().info(f"夹爪状态切换: {state_str}")
        self._publish_gripper_lock_status()

        response.success = True
        response.message = f"gripper now {state_str}"
        return response

    def _publish_gripper_lock_status(self):
        """发布当前锁定状态，供 Web UI 实时刷新按钮文字。"""
        msg = Bool()
        msg.data = self._gripper_lock_closed
        self.gripper_lock_status_pub.publish(msg)

    def _soft_start_monitor(self):
        """
        软启动监控线程：等待机械臂到位后，提示用户按回车切换到高跟随模式。
        
        适配多终端启动流程：
        1. 先启动 driver_node（本节点）
        2. 启动 encoder_node
        3. 人工对齐外骨骼和机械臂姿态
        4. 启动 teleop_node（开始发送指令）
        5. 在 driver_node 终端按回车，切换到高跟随模式
        
        流程：
        1. 等待 UDP 开始接收数据
        2. 等待 teleop_node 启动（开始收到动作指令）
        3. 等待机械臂跟随到目标位置（误差足够小）
        4. 用户按回车后，切换到高跟随模式
        """
        import time
        
        print("\n" + "=" * 60)
        print("🚀 软启动模式已启用")
        print("=" * 60)
        print("当前状态：低跟随模式（安全模式）")
        print("目标状态：高跟随模式（低延迟模式）")
        print("-" * 60)
        
        # 1. 等待 UDP 开始接收数据
        print("⏳ [1/3] 等待 UDP 状态上报...")
        while not self.udp_receiver.is_receiving(timeout=1.0):
            time.sleep(SOFT_START_CHECK_INTERVAL)
        print("✅ UDP 状态上报正常")
        
        # 2. 等待 teleop_node 启动（开始收到动作指令）
        print("⏳ [2/3] 等待 teleop_node 启动（订阅 action 话题）...")
        print("   💡 请先人工对齐外骨骼姿态，然后启动 teleop_node")
        
        while self.last_state is None:
            time.sleep(SOFT_START_CHECK_INTERVAL)
        print("✅ teleop_node 已启动，开始接收动作指令")
        
        # 3. 等待机械臂到位（误差足够小）
        print(f"⏳ [3/3] 等待机械臂跟随到外骨骼姿态（误差阈值: {SOFT_START_POSITION_THRESHOLD_DEG}°）...")
        
        ready_count = 0  # 连续满足条件的次数
        required_count = 10  # 需要连续满足 10 次才认为稳定（1秒）
        
        while ready_count < required_count:
            time.sleep(SOFT_START_CHECK_INTERVAL)
            
            # 获取机械臂实际关节角度
            joint_deg_actual = self.udp_receiver.read_joint_deg()
            if joint_deg_actual is None:
                ready_count = 0
                continue
            
            # 获取最后一次发送的目标角度（从 last_state 推算）
            if self.last_state is None:
                ready_count = 0
                continue
            
            # last_state 是 [7 个关节弧度 + 1 个夹爪]
            joint_rad_target = self.last_state[:7]
            joint_deg_target = np.degrees(joint_rad_target)
            
            # 计算误差
            error_deg = np.abs(joint_deg_actual[:7] - joint_deg_target)
            max_error = np.max(error_deg)
            
            if max_error < SOFT_START_POSITION_THRESHOLD_DEG:
                ready_count += 1
                print(f"\r✅ 机械臂已跟随到位，最大误差: {max_error:.2f}° ({ready_count}/{required_count})", end="", flush=True)
            else:
                ready_count = 0
                print(f"\r⏳ 跟随中... 最大误差: {max_error:.2f}° (需要 < {SOFT_START_POSITION_THRESHOLD_DEG}°)    ", end="", flush=True)
        
        print()  # 换行
        print("-" * 60)
        print("✅ 机械臂已稳定跟随到外骨骼姿态！")
        print()
        print("🎯 请通过 Web UI 或命令行启用高跟随模式：")
        print(f"   ros2 service call {self.service_prefix}/enable_high_follow std_srvs/srv/Trigger")
        print("-" * 60)
        
        # 等待 Service 调用（不再阻塞 stdin）
        # Service 回调 _handle_enable_high_follow 会设置 _high_follow_active

    def destroy_node(self):
        """销毁节点，释放资源。"""
        self.get_logger().info("🛑 正在关闭驱动节点...")
        try:
            # 停止夹爪控制线程
            self._gripper_stop_event.set()
            if self._gripper_thread is not None and self._gripper_thread.is_alive():
                self._gripper_thread.join(timeout=1.0)
            
            self.udp_receiver.stop()
            
            # 关闭机械臂连接（会自动关闭 RM+ 协议）
            if hasattr(self, '_rm_plus_baud') and self._rm_plus_baud > 0 and not self._dry_run:
                self.get_logger().info("🔌 正在关闭 RM+ 生态协议...")
            self.driver.close()
            self.get_logger().info("✅ 驱动节点已安全关闭")
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = RealmanDriverNode()
    _cleanup_done = False

    def _cleanup():
        nonlocal _cleanup_done
        if _cleanup_done:
            return
        _cleanup_done = True
        node.destroy_node()
        rclpy.shutdown()

    def _sigterm_handler(signum, frame):
        """SIGTERM handler: ensure RM+ protocol is closed before exit."""
        node.get_logger().warn("收到 SIGTERM 信号，正在安全关闭 RM+ 生态协议...")
        _cleanup()
        sys.exit(0)

    signal.signal(signal.SIGTERM, _sigterm_handler)
    atexit.register(_cleanup)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _cleanup()


if __name__ == "__main__":
    main()
