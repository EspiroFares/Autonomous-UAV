#!/usr/bin/env python3

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
        self.pub = self.create_publisher(Detection, "/target/detections", 10)
        self.sub = self.create_subscription(
            Image, "/camera/image_preprocessed", self.on_image, 10
        )

        self.pose = mp.solutions.pose.Pose(
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        self.get_logger().info("person_detector_node started")

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame.shape[:2]
        rgb = frame[:, :, ::-1]

        results = self.pose.process(rgb)
        det = Detection()
        det.header = msg.header

        if results.pose_landmarks:
            lm = results.pose_landmarks.landmark
            left_shoulder = lm[mp.solutions.pose.PoseLandmark.LEFT_SHOULDER]
            right_shoulder = lm[mp.solutions.pose.PoseLandmark.RIGHT_SHOULDER]

            cx = (left_shoulder.x + right_shoulder.x) / 2.0
            cy = (left_shoulder.y + right_shoulder.y) / 2.0

            shoulder_width_px = abs(left_shoulder.x - right_shoulder.x) * w

            det.detected = True
            det.confidence = 0.9
            det.bbox_center_x = float(cx)
            det.bbox_center_y = float(cy)
            det.bbox_width = float(shoulder_width_px / w)
            det.bbox_height = float(shoulder_width_px / w) * 2.0
            det.shoulder_width_px = float(shoulder_width_px)
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
