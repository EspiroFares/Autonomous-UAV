#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera2 import Picamera2


class CameraDriverNode(Node):
    def __init__(self):
        super().__init__("camera_driver_node")

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, "/camera/image_raw", 10)

        self.picam = Picamera2()
        config = self.picam.create_preview_configuration(
            main={"format": "BGR888", "size": (640, 480)}
        )
        self.picam.configure(config)
        self.picam.start()

        self.timer = self.create_timer(1.0 / 30.0, self.capture)
        self.get_logger().info("camera_driver_node started (picamera2)")

    def capture(self):
        frame = self.picam.capture_array()
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraDriverNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
