import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import numpy as np

from .hardware import EncoderCapture
from .utils import EXO_JOINT_NAMES, format_angle, gripper_open_from_pos


class EncoderNode(Node):
    """
    外骨骼编码器 ROS2 节点。
    
    数据流：
    -------
    编码器硬件 (0-360°)
         ↓
    EncoderCapture.read()  ← 原始角度，不做滤波
         ↓
    format_angle()  ← 将 0-360° 映射为相对于 MIDDLE_POINT 的连续角度
         ↓
    EMA 滤波  ← 在连续角度上滤波，避免 0°/360° 边界跳变问题
         ↓
    发布到 /exo/joint_state (弧度)
    
    重要：EMA 必须在 format_angle 之后进行！
    如果在 format_angle 之前做 EMA，当原始角度跨越 0°/360° 边界时
    （如 359° → 1°，实际只转了 2°），EMA 会计算出错误的中间值（如 180°）。
    """
    def __init__(self):
        super().__init__("encoder_node")
        self.declare_parameter("encoder_port", "/dev/ttyUSB0")
        self.declare_parameter("encoder_fps", 200)
        self.declare_parameter("ema_alpha", 0.8)
        self.declare_parameter("publish_hz", 200.0)
        self.declare_parameter("max_consecutive_failures", 50)
        self.declare_parameter("frame_id", "exo")
        self.declare_parameter("joint_names", EXO_JOINT_NAMES)
        self.declare_parameter("topic", "/exo/joint_state")

        port = self.get_parameter("encoder_port").get_parameter_value().string_value
        fps = self.get_parameter("encoder_fps").get_parameter_value().integer_value
        alpha = self.get_parameter("ema_alpha").get_parameter_value().double_value
        publish_hz = self.get_parameter("publish_hz").get_parameter_value().double_value
        max_consecutive_failures = (
            self.get_parameter("max_consecutive_failures").get_parameter_value().integer_value
        )
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.joint_names = list(self.get_parameter("joint_names").value)
        topic = self.get_parameter("topic").get_parameter_value().string_value

        self.encoder = EncoderCapture(
            port=port,
            fps=fps,
            alpha=alpha,
            max_consecutive_failures=max_consecutive_failures,
        )
        self.encoder.start()

        self.publisher = self.create_publisher(JointState, topic, 10)
        period = 1.0 / publish_hz if publish_hz > 0 else 0.01
        self.timer = self.create_timer(period, self._on_timer)
        
        # EMA 滤波状态（在 format_angle 之后进行，避免 0°/360° 边界问题）
        self.ema_alpha = alpha
        self.filtered_angles: np.ndarray = None

    def _on_timer(self):
        fatal_error = self.encoder.get_fatal_error()
        if fatal_error is not None:
            self.get_logger().error(fatal_error)
            rclpy.shutdown()
            raise RuntimeError(fatal_error)
        data = self.encoder.read()
        if data is None:
            return
        
        # 先做角度映射（将 0-360° 转为连续的相对角度）
        angles = format_angle(np.asarray(data["angle"]))
        
        # 再做 EMA 滤波（此时角度已经是连续的，不会有跳变问题）
        if self.filtered_angles is None:
            self.filtered_angles = angles.copy()
        else:
            self.filtered_angles = self.ema_alpha * angles + (1.0 - self.ema_alpha) * self.filtered_angles
        
        positions = list(self.filtered_angles[:7].astype(float))
        gripper_pos = gripper_open_from_pos(float(self.filtered_angles[7]))
        positions.append(gripper_pos)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.name = self.joint_names
        msg.position = positions
        self.publisher.publish(msg)

    def destroy_node(self):
        try:
            self.encoder.stop()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = EncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
