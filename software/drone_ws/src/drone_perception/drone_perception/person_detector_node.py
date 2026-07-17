#!/usr/bin/env python3

import threading
import time
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from drone_interfaces.msg import Detection
import mediapipe as mp
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class PersonDetectorNode(Node):
    def __init__(self):
        super().__init__("person_detector_node")

        self.bridge = CvBridge()

        self.pub = self.create_publisher(
            Detection,
            "/target/detections",
            10,
        )

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.sub = self.create_subscription(
            Image, "/camera/image_preprocessed", self.on_image, qos
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

                ls = lm[mp.solutions.pose.PoseLandmark.LEFT_SHOULDER]
                rs = lm[mp.solutions.pose.PoseLandmark.RIGHT_SHOULDER]
                lh = lm[mp.solutions.pose.PoseLandmark.LEFT_HIP]
                rh = lm[mp.solutions.pose.PoseLandmark.RIGHT_HIP]

                shoulder_visibility = min(
                    float(ls.visibility),
                    float(rs.visibility),
                )
                torso_visibility = min(
                    shoulder_visibility,
                    float(lh.visibility),
                    float(rh.visibility),
                )

                measurements_finite = all(
                    math.isfinite(value)
                    for value in (
                        ls.x,
                        ls.y,
                        rs.x,
                        rs.y,
                        lh.x,
                        lh.y,
                        rh.x,
                        rh.y,
                        ls.visibility,
                        rs.visibility,
                        lh.visibility,
                        rh.visibility,
                    )
                )

                shoulders_in_frame = (
                    0.02 <= ls.x <= 0.98
                    and 0.02 <= ls.y <= 0.98
                    and 0.02 <= rs.x <= 0.98
                    and 0.02 <= rs.y <= 0.98
                )
                torso_in_frame = (
                    shoulders_in_frame
                    and 0.02 <= lh.x <= 0.98
                    and 0.02 <= lh.y <= 0.98
                    and 0.02 <= rh.x <= 0.98
                    and 0.02 <= rh.y <= 0.98
                )

                shoulder_cx = (ls.x + rs.x) / 2.0
                shoulder_cy = (ls.y + rs.y) / 2.0
                hip_cx = (lh.x + rh.x) / 2.0
                hip_cy = (lh.y + rh.y) / 2.0

                shoulder_width_px = math.hypot(
                    (ls.x - rs.x) * w,
                    (ls.y - rs.y) * h,
                )
                torso_height_px = math.hypot(
                    (shoulder_cx - hip_cx) * w,
                    (shoulder_cy - hip_cy) * h,
                )

                shoulder_valid = (
                    measurements_finite
                    and shoulder_visibility >= 0.65
                    and shoulders_in_frame
                    and shoulder_width_px >= 35.0
                )
                torso_valid = (
                    measurements_finite
                    and torso_visibility >= 0.65
                    and torso_in_frame
                    and torso_height_px >= 30.0
                )

                if shoulder_valid or torso_valid:
                    det.detected = True
                    det.confidence = float(max(
                        shoulder_visibility if shoulder_valid else 0.0,
                        torso_visibility if torso_valid else 0.0,
                    ))

                    det.bbox_center_x = float(shoulder_cx)
                    det.bbox_center_y = float(shoulder_cy)
                    det.bbox_width = float(shoulder_width_px / w)
                    det.bbox_height = float(torso_height_px / h)
                    det.shoulder_width_px = float(shoulder_width_px)
                    det.torso_height_px = float(torso_height_px)
                    det.shoulder_valid = bool(shoulder_valid)
                    det.torso_valid = bool(torso_valid)
                else:
                    det.detected = False
                    det.confidence = 0.0
                    det.bbox_center_x = 0.0
                    det.bbox_center_y = 0.0
                    det.bbox_width = 0.0
                    det.bbox_height = 0.0
                    det.shoulder_width_px = 0.0
                    det.torso_height_px = 0.0
                    det.shoulder_valid = False
                    det.torso_valid = False
            else:
                det.detected = False
                det.confidence = 0.0
                det.bbox_center_x = 0.0
                det.bbox_center_y = 0.0
                det.bbox_width = 0.0
                det.bbox_height = 0.0
                det.shoulder_width_px = 0.0
                det.torso_height_px = 0.0
                det.shoulder_valid = False
                det.torso_valid = False

            self.pub.publish(det)


def main(args=None):
    rclpy.init(args=args)

    node = PersonDetectorNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
