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
                    for landmark in (ls, rs, lh, rh)
                    for value in (
                        landmark.x,
                        landmark.y,
                        landmark.visibility,
                    )
                )

                shoulders_in_frame = all(
                    0.02 <= value <= 0.98
                    for value in (ls.x, ls.y, rs.x, rs.y)
                )
                torso_in_frame = all(
                    0.02 <= value <= 0.98
                    for value in (
                        ls.x, ls.y, rs.x, rs.y,
                        lh.x, lh.y, rh.x, rh.y,
                    )
                )

                shoulder_cx = (ls.x + rs.x) / 2.0
                shoulder_cy = (ls.y + rs.y) / 2.0
                hip_cx = (lh.x + rh.x) / 2.0
                hip_cy = (lh.y + rh.y) / 2.0

                shoulder_width_px = abs(ls.x - rs.x) * w
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
                    det.confidence = float(
                        max(shoulder_visibility, torso_visibility)
                    )

                    det.bbox_center_x = float(shoulder_cx)
                    det.bbox_center_y = float(shoulder_cy)
                    det.bbox_width = float(shoulder_width_px / w)
                    det.bbox_height = float(torso_height_px / h)
                    det.shoulder_width_px = float(shoulder_width_px)

                else:
                    det.detected = False
                    det.confidence = 0.0
                    det.bbox_center_x = 0.0
                    det.bbox_center_y = 0.0
                    det.bbox_width = 0.0
                    det.bbox_height = 0.0
                    det.shoulder_width_px = 0.0

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
