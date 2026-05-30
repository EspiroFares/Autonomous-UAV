#!/usr/bin/env python3

import socket
import struct
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

HOST = "host.docker.internal"
PORT = 8485


class FakeCameraDriverNode(Node):
    def __init__(self):
        super().__init__("fake_camera_driver_node")

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, "/camera/image_raw", 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((HOST, PORT))
        self.get_logger().info(f"Connected to webcam stream at {HOST}:{PORT}")

        self.timer = self.create_timer(0.033, self.read_frame)

    def read_frame(self):
        try:
            raw_len = self._recv_exact(4)
            if raw_len is None:
                return
            msg_len = struct.unpack(">I", raw_len)[0]
            data = self._recv_exact(msg_len)
            if data is None:
                return
            frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                return
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Frame read error: {e}")

    def _recv_exact(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf


def main(args=None):
    rclpy.init(args=args)
    node = FakeCameraDriverNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
