#!/usr/bin/env python3

import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from drone_interfaces.msg import Detection
import mediapipe as mp


class PersonDetectorNode(Node):
    def __init__(self):
        super().__init__("person_detector_node")

        self.bridge = CvBridge()

        self.pub = self.create_publisher(
            Detection,
            "/target/detections",
            10,
        )

        self.sub = self.create_subscription(
            Image,
            "/camera/image_preprocessed",
            self.on_image,
            10,
        )

        self.pose = mp.solutions.pose.Pose(
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        self._lock = threading.Lock()
        self._latest_frame = None
        self._latest_header = None

        self._thread = threading.Thread(
            target=self._inference_loop,
            daemon=True,
        )
        self._thread.start()

        self.get_logger().info("person_detector_node started")

    def on_image(self, msg):
        # Non-blocking — just store latest frame, drop old ones
        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8",
        )

        with self._lock:
            self._latest_frame = frame
            self._latest_header = msg.header

    def _inference_loop(self):
        while rclpy.ok():
            with self._lock:
                frame = self._latest_frame
                header = self._latest_header
                self._latest_frame = None

            if frame is None:
                time.sleep(0.005)
                continue

            h, w = frame.shape[:2]
            rgb = frame[:, :, ::-1]

            results = self.pose.process(rgb)

            det = Detection()
            det.header = header

            if results.pose_landmarks:
                lm = results.pose_landmarks.landmark

                ls = lm[
                    mp.solutions.pose.PoseLandmark.LEFT_SHOULDER
                ]
                rs = lm[
                    mp.solutions.pose.PoseLandmark.RIGHT_SHOULDER
                ]

                cx = (ls.x + rs.x) / 2.0
                cy = (ls.y + rs.y) / 2.0

                shoulder_width_px = abs(ls.x - rs.x) * w

                det.detected = True
                det.confidence = 0.9

                det.bbox_center_x = float(cx)
                det.bbox_center_y = float(cy)

                det.bbox_width = float(
                    shoulder_width_px / w
                )

                det.bbox_height = float(
                    shoulder_width_px / w
                ) * 2.0

                det.shoulder_width_px = float(
                    shoulder_width_px
                )

            else:
                det.detected = False
                det.confidence = 0.0

                det.bbox_center_x = 0.0
                det.bbox_center_y = 0.0

                det.bbox_width = 0.0
                det.bbox_height = 0.0

                det.shoulder_width_px = 0.0

            self.pub.publish(det)


def main(args=None):
    rclpy.init(args=args)

    node = PersonDetectorNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()