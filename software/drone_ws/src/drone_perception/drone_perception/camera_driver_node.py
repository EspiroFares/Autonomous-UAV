import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraDriverNode(Node):
    def __init__(self):
        super().__init__("camera_driver_node")

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self.timer = self.create_timer(1.0 / 30.0, self.update)

        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera")
        else:
            self.get_logger().info("camera_driver_node started")

    def update(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame")
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
