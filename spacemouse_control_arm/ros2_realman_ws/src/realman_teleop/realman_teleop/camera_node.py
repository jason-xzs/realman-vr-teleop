import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from .hardware import CameraCapture


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        
        # 从节点名推断相机名（节点名格式: camera_node_cam0）
        node_name = self.get_name()  # 例如 "camera_node_cam0"
        default_cam_name = node_name.replace("camera_node_", "") if node_name.startswith("camera_node_") else "cam0"
        
        self.declare_parameter("serial", "")
        self.declare_parameter("name", default_cam_name)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("use_depth", True)
        self.declare_parameter("convert_bgr_to_rgb", True)
        self.declare_parameter("flush_frames", 0)
        self.declare_parameter("frame_id", default_cam_name)
        self.declare_parameter("rotate_deg", 0)  # 旋转角度：0, 90, 180, 270

        serial = self.get_parameter("serial").get_parameter_value().string_value
        name = self.get_parameter("name").get_parameter_value().string_value
        fps = self.get_parameter("fps").get_parameter_value().double_value
        use_depth = self.get_parameter("use_depth").get_parameter_value().bool_value
        convert_bgr_to_rgb = self.get_parameter("convert_bgr_to_rgb").get_parameter_value().bool_value
        flush_frames = self.get_parameter("flush_frames").get_parameter_value().integer_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.rotate_deg = self.get_parameter("rotate_deg").get_parameter_value().integer_value

        # 话题名自动生成
        color_topic = f"/camera/{name}/color/image_raw"
        depth_topic = f"/camera/{name}/depth/image_raw"

        rotate_info = f" rotate={self.rotate_deg}°" if self.rotate_deg != 0 else ""
        self.get_logger().info(f"📷 启动相机 [{name}] serial={serial or '自动'}{rotate_info} → {color_topic}")

        self.bridge = CvBridge()
        self.camera = CameraCapture(
            serial=serial if serial else None,
            use_depth=use_depth,
            name=name,
            convert_bgr_to_rgb=convert_bgr_to_rgb,
            flush_frames=flush_frames,
        )

        self.color_pub = self.create_publisher(Image, color_topic, 10)
        self.depth_pub = self.create_publisher(Image, depth_topic, 10) if use_depth else None
        self.color_encoding = "rgb8" if convert_bgr_to_rgb else "bgr8"

        period = 1.0 / fps if fps > 0 else 0.03
        self.timer = self.create_timer(period, self._on_timer)

    def _rotate_image(self, img):
        """根据配置旋转图像。"""
        if self.rotate_deg == 0 or img is None:
            return img
        elif self.rotate_deg == 90:
            return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotate_deg == 180:
            return cv2.rotate(img, cv2.ROTATE_180)
        elif self.rotate_deg == 270:
            return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        else:
            return img

    def _on_timer(self):
        color, depth = self.camera.get()
        stamp = self.get_clock().now().to_msg()
        
        # 旋转图像
        color = self._rotate_image(color)
        depth = self._rotate_image(depth)

        color_msg = self.bridge.cv2_to_imgmsg(color, encoding=self.color_encoding)
        color_msg.header.stamp = stamp
        color_msg.header.frame_id = self.frame_id
        self.color_pub.publish(color_msg)

        if self.depth_pub is not None and depth is not None:
            depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="16UC1")
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = self.frame_id
            self.depth_pub.publish(depth_msg)


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
