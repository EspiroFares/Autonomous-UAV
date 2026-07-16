#!/usr/bin/env python3
"""
hud_overlay_node — draws a slick "autonomous targeting" HUD on top of the
camera feed for demo / recording purposes.

Subscribes:
  /camera/image_raw   (sensor_msgs/Image)        — base frame
  /target/track       (drone_interfaces/Track)   — smoothed target box (normalized)
  /mission/state      (drone_interfaces/MissionState) — state machine phase
  /target/state       (drone_interfaces/TargetState)  — distance + confidence

Publishes:
  /camera/overlay     (sensor_msgs/Image, bgr8)  — annotated frame to record

Purely a visualization node — it never touches detection or control.
"""

import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from drone_interfaces.msg import Track, MissionState, TargetState


# BGR colors keyed by mission state.
STATE_COLORS = {
    "IDLE": (180, 180, 180),
    "TRACKING": (0, 200, 255),       # amber
    "FOLLOWING": (0, 230, 90),       # green
    "TARGET_LOST": (60, 60, 255),    # red
    "SAFETY_HOLD": (0, 120, 255),    # orange
}
DEFAULT_COLOR = (200, 200, 200)


class HudOverlayNode(Node):
    def __init__(self):
        super().__init__("hud_overlay_node")

        self.bridge = CvBridge()
        self.declare_parameter("track_fresh_timeout", 0.5)

        # latest state, drawn on each incoming frame
        self.track = None
        self.track_stamp = 0.0
        self.mission = None
        self.target = None
        self.t0 = time.time()

        self.create_subscription(Image, "/camera/image_raw", self.on_image, 10)
        self.create_subscription(Track, "/target/track", self.on_track, 10)
        self.create_subscription(MissionState, "/mission/state", self.on_mission, 10)
        self.create_subscription(TargetState, "/target/state", self.on_target, 10)

        self.pub = self.create_publisher(Image, "/camera/overlay", 10)

        self.get_logger().info("hud_overlay_node started — publishing /camera/overlay")

    # ---- subscriptions (just cache latest) ----
    def on_track(self, msg):
        self.track = msg
        self.track_stamp = time.time()

    def on_mission(self, msg):
        self.mission = msg

    def on_target(self, msg):
        self.target = msg

    # ---- main draw loop, driven by the camera ----
    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame.shape[:2]

        state = self.mission.state if self.mission else "IDLE"
        color = STATE_COLORS.get(state, DEFAULT_COLOR)

        # Keep the latest valid box through short detector scheduling gaps.
        track_fresh_timeout = float(
            self.get_parameter("track_fresh_timeout").value)
        track_live = (
            self.track is not None
            and self.track.valid
            and (time.time() - self.track_stamp) < track_fresh_timeout
        )

        self._draw_frame_chrome(frame, w, h, color)
        self._draw_crosshair(frame, w, h, color)

        if track_live:
            self._draw_target_box(frame, w, h, color)
        else:
            self._draw_searching(frame, w, h)

        self._draw_hud_text(frame, w, h, state, color, track_live)
        self._draw_rec(frame, w)

        out = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out.header = msg.header
        self.pub.publish(out)

    # ---- drawing helpers ----
    def _draw_frame_chrome(self, img, w, h, color):
        """Cyan corner brackets for a drone-cam framing look."""
        m, L = 18, 46  # margin, bracket length
        c = (220, 220, 220)
        for (cx, cy, dx, dy) in [
            (m, m, 1, 1), (w - m, m, -1, 1),
            (m, h - m, 1, -1), (w - m, h - m, -1, -1),
        ]:
            cv2.line(img, (cx, cy), (cx + dx * L, cy), c, 2, cv2.LINE_AA)
            cv2.line(img, (cx, cy), (cx, cy + dy * L), c, 2, cv2.LINE_AA)

    def _draw_crosshair(self, img, w, h, color):
        cx, cy, g, s = w // 2, h // 2, 8, 16
        c = (150, 150, 150)
        cv2.line(img, (cx - g - s, cy), (cx - g, cy), c, 1, cv2.LINE_AA)
        cv2.line(img, (cx + g, cy), (cx + g + s, cy), c, 1, cv2.LINE_AA)
        cv2.line(img, (cx, cy - g - s), (cx, cy - g), c, 1, cv2.LINE_AA)
        cv2.line(img, (cx, cy + g), (cx, cy + g + s), c, 1, cv2.LINE_AA)

    def _draw_target_box(self, img, w, h, color):
        t = self.track
        # Track fields are normalized [0,1]; expand height for a body-like box.
        bw = max(t.width, 0.04) * w
        bh = max(t.height, 0.04) * h * 1.6
        cx, cy = t.center_x * w, t.center_y * h
        x1, y1 = int(cx - bw / 2), int(cy - bh / 2)
        x2, y2 = int(cx + bw / 2), int(cy + bh / 2)
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w - 1, x2), min(h - 1, y2)

        # corner-style lock-on box
        cl = max(12, int(0.18 * (x2 - x1)))
        for (px, py, dx, dy) in [
            (x1, y1, 1, 1), (x2, y1, -1, 1),
            (x1, y2, 1, -1), (x2, y2, -1, -1),
        ]:
            cv2.line(img, (px, py), (px + dx * cl, py), color, 2, cv2.LINE_AA)
            cv2.line(img, (px, py), (px, py + dy * cl), color, 2, cv2.LINE_AA)

        # pulsing reticle on the target center
        pulse = int(6 + 3 * (1 + np.sin((time.time() - self.t0) * 6)))
        icx, icy = int(cx), int(cy)
        cv2.circle(img, (icx, icy), pulse, color, 1, cv2.LINE_AA)
        cv2.drawMarker(img, (icx, icy), color, cv2.MARKER_CROSS, 12, 1, cv2.LINE_AA)

        # leader line from screen center to target (shows steering intent)
        cv2.line(img, (w // 2, h // 2), (icx, icy), color, 1, cv2.LINE_AA)

        # label tag
        self._tag(img, x1, y1 - 8, "TARGET LOCKED", color)

    def _draw_searching(self, img, w, h):
        if int((time.time() - self.t0) * 2) % 2 == 0:  # blink
            self._tag(img, w // 2 - 70, h // 2 - 40, "SEARCHING", (60, 60, 255))

    def _draw_hud_text(self, img, w, h, state, color, track_live):
        # status panel, top-left
        self._panel(img, 14, 18, 230, 78)
        cv2.putText(img, f"STATE: {state}", (24, 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

        follow = "ENABLED" if (self.mission and self.mission.follow_enabled) else "—"
        cv2.putText(img, f"FOLLOW: {follow}", (24, 68),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

        # telemetry, bottom-left
        if self.target and track_live:
            dist = self.target.distance_estimate
            yaw = self.target.yaw_error
            self._panel(img, 14, h - 64, 200, h - 10)
            cv2.putText(img, f"DIST: {dist:4.2f} m", (24, h - 42),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 2, cv2.LINE_AA)
            cv2.putText(img, f"YAW ERR: {yaw:+4.2f}", (24, h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 2, cv2.LINE_AA)

    def _panel(self, img, x1, y1, x2, y2, alpha=0.45):
        """Translucent dark backing panel for HUD text readability."""
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(img.shape[1], x2), min(img.shape[0], y2)
        if x2 <= x1 or y2 <= y1:
            return
        roi = img[y1:y2, x1:x2]
        dark = np.zeros_like(roi)
        cv2.addWeighted(dark, alpha, roi, 1 - alpha, 0, roi)

    def _draw_rec(self, img, w):
        t = time.time() - self.t0
        if int(t * 2) % 2 == 0:
            cv2.circle(img, (w - 150, 33), 7, (60, 60, 255), -1, cv2.LINE_AA)
        cv2.putText(img, "REC", (w - 136, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (240, 240, 240), 2, cv2.LINE_AA)
        cv2.putText(img, f"{t:06.1f}", (w - 150, 62),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)

    def _tag(self, img, x, y, text, color):
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        y = max(y, th + 4)
        cv2.rectangle(img, (x, y - th - 4), (x + tw + 8, y + 4), (0, 0, 0), -1)
        cv2.putText(img, text, (x + 4, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


def main(args=None):
    rclpy.init(args=args)
    node = HudOverlayNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
