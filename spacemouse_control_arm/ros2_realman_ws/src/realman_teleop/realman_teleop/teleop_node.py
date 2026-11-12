from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .utils import EXO_JOINT_NAMES, exo_to_robot_cmd_deg


class TeleopNode(Node):
    """
    遥操作节点：将外骨骼关节状态映射为机械臂控制指令。
    
    编码器端已做 EMA 滤波，此处直接透传映射后的角度指令，
    不再做额外的角速度限制，以保证跟随性。
    """
    
    def __init__(self):
        super().__init__("teleop_node")
        self.declare_parameter("exo_topic", "/exo/joint_state")
        self.declare_parameter("action_topic", "/rm/action_joint_state")
        self.declare_parameter("control_hz", 50.0)
        self.declare_parameter("frame_id", "rm_base")
        self.declare_parameter("joint_names", EXO_JOINT_NAMES)

        exo_topic = self.get_parameter("exo_topic").get_parameter_value().string_value
        action_topic = self.get_parameter("action_topic").get_parameter_value().string_value
        self.control_hz = self.get_parameter("control_hz").get_parameter_value().double_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.joint_names = list(self.get_parameter("joint_names").value)

        self.last_exo: Optional[JointState] = None

        self.subscription = self.create_subscription(JointState, exo_topic, self._on_exo, 10)
        self.publisher = self.create_publisher(JointState, action_topic, 10)

        period = 1.0 / self.control_hz if self.control_hz > 0 else 0.02
        self.timer = self.create_timer(period, self._on_timer)

    def _on_exo(self, msg: JointState):
        self.last_exo = msg

    def _on_timer(self):
        if self.last_exo is None or not self.last_exo.position:
            return
        positions = list(self.last_exo.position)
        if len(positions) < 7:
            return

        exo_rad = np.array(positions[:7], dtype=float)
        gripper_open = float(positions[7]) if len(positions) > 7 else 0.0

        # 外骨骼角度映射为机械臂角度（编码器端已做 EMA 滤波，直接透传）
        cmd_deg = exo_to_robot_cmd_deg(exo_rad)
        cmd_rad = np.radians(cmd_deg)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.name = self.joint_names
        msg.position = list(cmd_rad.astype(float)) + [gripper_open]
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
