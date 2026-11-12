"""
VR 输入节点 - 接收 VR 控制器数据并发布到 ROS2 话题

支持三种数据源:
1. ADB (USB): 通过 adb logcat 读取 VR 设备日志 - 默认推荐
2. UDP Binary (WiFi): 接收 UDP 12345 二进制数据
3. UDP JSON (兼容): 接收 UDP 6666 JSON 数据 (需要 hmi-user-dev)

数据流:
-------
VR 设备 → [ADB/UDP 12345/UDP 6666] → vr_input_node → ROS2 话题

发布话题:
- /vr/right/pose (geometry_msgs/PoseStamped)
- /vr/right/trigger (std_msgs/Float32)
- /vr/right/grip (std_msgs/Float32)
- /vr/connected (std_msgs/Bool)
- /vr/source_info (std_msgs/String) - 数据源信息

使用示例:
    # USB 连接 (默认)
    ros2 launch realman_teleop vr_teleop.launch.py
    
    # WiFi 连接
    ros2 launch realman_teleop vr_teleop.launch.py vr_data_source:=udp_binary
    
    # 兼容模式 (需要 hmi-user-dev)
    ros2 launch realman_teleop vr_teleop.launch.py vr_data_source:=udp_json
"""
import json
import time
from typing import Optional, Dict, Any

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Float32, Bool, String

from .vr_bridge import VRDataManager


class VRControllerData:
    """单个 VR 控制器数据"""
    
    def __init__(self):
        self.position = np.zeros(3, dtype=np.float32)
        self.rotation = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.grip = 0.0
        self.trigger = 0.0
        self.joystick = np.zeros(2, dtype=np.float32)
        self.buttons: Dict[str, int] = {}
        self.timestamp = 0.0


class VRInputNode(Node):
    """
    VR 输入 ROS2 节点
    
    支持三种数据源:
    - adb: USB 连接，通过 adb logcat 读取
    - udp_binary: WiFi 连接，UDP 12345 二进制数据
    - udp_json: 兼容模式，UDP 6666 JSON 数据
    """
    
    VALID_SOURCES = ["adb", "udp_binary", "udp_json"]
    
    def __init__(self):
        super().__init__("vr_input_node")
        
        # ===== 参数声明 =====
        # 数据源选择
        self.declare_parameter("data_source", "adb")
        
        # ADB 参数
        self.declare_parameter("adb_device_id", "")
        self.declare_parameter("adb_check_interval", 3.0)
        
        # UDP 参数
        self.declare_parameter("udp_binary_port", 12345)
        self.declare_parameter("udp_json_port", 6666)
        
        # 通用参数
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("timeout", 1.0)
        self.declare_parameter("side", "right")
        self.declare_parameter("sides", ["left", "right"])
        
        # 向后兼容的 udp_port 参数
        self.declare_parameter("udp_port", 6666)
        
        # ===== 读取参数 =====
        self.data_source = self.get_parameter("data_source").get_parameter_value().string_value
        self.adb_device_id = self.get_parameter("adb_device_id").get_parameter_value().string_value
        self.adb_check_interval = self.get_parameter("adb_check_interval").get_parameter_value().double_value
        self.udp_binary_port = self.get_parameter("udp_binary_port").get_parameter_value().integer_value
        self.udp_json_port = self.get_parameter("udp_json_port").get_parameter_value().integer_value
        publish_hz = self.get_parameter("publish_hz").get_parameter_value().double_value
        self.timeout = self.get_parameter("timeout").get_parameter_value().double_value
        self.side = self.get_parameter("side").get_parameter_value().string_value
        self.sides = list(self.get_parameter("sides").value)
        if not self.sides:
            self.sides = [self.side]
        
        # 向后兼容: 如果 udp_port 被设置且 data_source 是默认值，使用 udp_json 模式
        legacy_port = self.get_parameter("udp_port").get_parameter_value().integer_value
        if legacy_port != 6666 and self.data_source == "adb":
            self.get_logger().warn(f"检测到旧版 udp_port 参数 ({legacy_port})，使用 udp_json 模式")
            self.data_source = "udp_json"
            self.udp_json_port = legacy_port
        
        # 验证数据源
        if self.data_source not in self.VALID_SOURCES:
            self.get_logger().error(f"无效的数据源: {self.data_source}, 回退到 adb")
            self.data_source = "adb"
        
        # ===== VR 数据管理器 =====
        self._vr_manager = VRDataManager()
        
        # ===== 本地数据缓存 =====
        self._controllers = {side: VRControllerData() for side in self.sides}
        self._last_recv_time: Optional[float] = None
        
        # ===== 统计 =====
        self._recv_count = 0
        self._error_count = 0
        
        # ===== 创建发布者 =====
        self.pub_pose = {side: self.create_publisher(PoseStamped, f'/vr/{side}/pose', 10) for side in self.sides}
        self.pub_trigger = {side: self.create_publisher(Float32, f'/vr/{side}/trigger', 10) for side in self.sides}
        self.pub_grip = {side: self.create_publisher(Float32, f'/vr/{side}/grip', 10) for side in self.sides}
        self.pub_connected = self.create_publisher(Bool, '/vr/connected', 10)
        self.pub_source_info = self.create_publisher(String, '/vr/source_info', 10)
        
        # ===== 启动数据源 =====
        self._start_data_source()
        
        # ===== 创建定时器 =====
        period = 1.0 / publish_hz if publish_hz > 0 else 0.02
        self.timer = self.create_timer(period, self._on_timer)
        
        # 统计日志定时器
        self.stats_timer = self.create_timer(10.0, self._log_stats)
        
        # 数据源信息定时发布
        self.source_info_timer = self.create_timer(1.0, self._publish_source_info)
        
        self.get_logger().info(f"VR 输入节点已启动")
        self.get_logger().info(f"  数据源: {self.data_source}")
        self.get_logger().info(f"  控制器: {self.sides}, 发布频率: {publish_hz}Hz")
    
    def _start_data_source(self):
        """启动数据源"""
        kwargs = {}
        
        if self.data_source == "adb":
            if self.adb_device_id:
                kwargs["adb_device_id"] = self.adb_device_id
            self.get_logger().info(f"启动 ADB 数据源...")
            
        elif self.data_source == "udp_binary":
            kwargs["udp_port"] = self.udp_binary_port
            self.get_logger().info(f"启动 UDP Binary 数据源，端口: {self.udp_binary_port}")
            
        else:  # udp_json
            kwargs["udp_port"] = self.udp_json_port
            self.get_logger().info(f"启动 UDP JSON 数据源，端口: {self.udp_json_port}")
        
        if not self._vr_manager.start(self.data_source, **kwargs):
            self.get_logger().error(f"数据源 {self.data_source} 启动失败")
        else:
            self.get_logger().info(f"数据源 {self.data_source} 启动成功")
    
    def _on_timer(self):
        """定时获取数据并发布"""
        # 从 VRDataManager 获取左右手柄数据
        for side in self.sides:
            side_data = self._vr_manager.get_controller_data(side)
            if side_data is None:
                continue
            ctrl = self._controllers[side]
            ctrl.position = np.array(side_data.position, dtype=np.float32)
            ctrl.rotation = np.array(side_data.rotation, dtype=np.float32)
            ctrl.grip = side_data.grip
            ctrl.trigger = side_data.trigger
            ctrl.joystick = np.array(side_data.joystick, dtype=np.float32)
            ctrl.buttons = side_data.buttons
            ctrl.timestamp = side_data.timestamp
            self._last_recv_time = time.time()
            self._recv_count += 1

        # 检查连接状态
        is_connected = self._vr_manager.is_connected()
        self._publish_connected(is_connected)

        # 发布控制器数据
        if is_connected:
            stamp = self.get_clock().now().to_msg()
            for side, ctrl in self._controllers.items():
                self._publish_controller(side, ctrl, stamp)
    
    def _effective_grip(self, ctrl: VRControllerData) -> float:
        grip = float(ctrl.grip)
        for name, value in ctrl.buttons.items():
            if name.lower() in (
                "grip",
                "gripbutton",
                "grip_button",
                "sidegrip",
                "side_grip",
                "squeeze",
            ) and int(value) != 0:
                grip = max(grip, 1.0)
        return grip

    def _publish_controller(self, side: str, ctrl: VRControllerData, stamp):
        """发布控制器数据"""
        # 发布位姿
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = f"vr_{side}"
        pose_msg.pose.position = Point(
            x=float(ctrl.position[0]),
            y=float(ctrl.position[1]),
            z=float(ctrl.position[2])
        )
        pose_msg.pose.orientation = Quaternion(
            x=float(ctrl.rotation[0]),
            y=float(ctrl.rotation[1]),
            z=float(ctrl.rotation[2]),
            w=float(ctrl.rotation[3])
        )
        self.pub_pose[side].publish(pose_msg)
        
        # 发布 trigger
        trigger_msg = Float32(data=float(ctrl.trigger))
        self.pub_trigger[side].publish(trigger_msg)
        
        # 发布 grip。部分 VR App 会把侧握键作为 button 字段上报，这里合并成同一个 grip topic。
        grip_msg = Float32(data=self._effective_grip(ctrl))
        self.pub_grip[side].publish(grip_msg)
    
    def _publish_connected(self, connected: bool):
        """发布连接状态"""
        msg = Bool(data=connected)
        self.pub_connected.publish(msg)
    
    def _publish_source_info(self):
        """发布数据源信息"""
        status = self._vr_manager.get_connection_status()
        
        info = {
            "source": status["source"],
            "connected": status["connected"],
            "recv_count": status["recv_count"],
            "adb_device_id": status.get("adb_device_id"),
            "udp_port": status.get("udp_port"),
        }
        
        msg = String(data=json.dumps(info))
        self.pub_source_info.publish(msg)
    
    def _log_stats(self):
        """定时输出统计信息"""
        status = self._vr_manager.get_connection_status()
        
        source_desc = {
            "adb": "USB/ADB",
            "udp_binary": "WiFi/UDP Binary",
            "udp_json": "UDP JSON"
        }.get(self.data_source, self.data_source)
        
        conn_status = "正常" if status["connected"] else "断开"
        
        extra_info = ""
        if self.data_source == "adb" and status.get("adb_device_id"):
            extra_info = f" | 设备: {status['adb_device_id']}"
        elif self.data_source in ["udp_binary", "udp_json"] and status.get("udp_port"):
            extra_info = f" | 端口: {status['udp_port']}"
        
        self.get_logger().info(
            f"[VR 输入] {source_desc} | {conn_status} | "
            f"已接收: {status['recv_count']} 条 | 错误: {status['error_count']} 次{extra_info}"
        )
    
    def destroy_node(self):
        """清理资源"""
        self.get_logger().info("正在关闭 VR 输入节点...")
        
        if self._vr_manager:
            self._vr_manager.stop()
        
        super().destroy_node()


def main():
    rclpy.init()
    node = VRInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
