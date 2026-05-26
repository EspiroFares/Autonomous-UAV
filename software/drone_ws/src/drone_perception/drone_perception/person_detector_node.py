#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from drone_interfaces.msg import TargetState
import mediapipe as mp


class PersonDetectorNode(Node):
    def __init__(self):
        super().__init__("person_detector_node")

        self.bridge = CvBridge()
        self.pub = self.create_publisher(TargetState, "/target/state", 10)
        self.sub = self.create_subscription(
            Image, "/camera/image_raw", self.on_image, 10
        )

        self.pose = mp.solutions.pose.Pose(
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        self.get_logger().info("person_detector_node started")

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame.shape[:2]
        rgb = frame[:, :, ::-1]
        results = self.pose.process(rgb)
        target = TargetState()

        if results.pose_landmarks:
            lm = results.pose_landmarks.landmark
            left_shoulder = lm[mp.solutions.pose.PoseLandmark.LEFT_SHOULDER]
            right_shoulder = lm[mp.solutions.pose.PoseLandmark.RIGHT_SHOULDER]

            cx = (left_shoulder.x + right_shoulder.x) / 2.0
            shoulder_width_px = abs(left_shoulder.x - right_shoulder.x) * w
            known_shoulder_width = 0.45
            focal_length = 600.0
            distance = (known_shoulder_width * focal_length) / (
                shoulder_width_px + 1e-6
            )

            target.detected = True
            target.confidence = 0.9
            target.yaw_error = float(cx - 0.5) * 2.0
            target.distance_estimate = float(distance)
        else:
            target.detected = False
            target.confidence = 0.0
            target.yaw_error = 0.0
            target.distance_estimate = 0.0

        self.pub.publish(target)


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()
