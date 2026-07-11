#!/usr/bin/env python3
"""SIM ONLY — highlights the tracked person in the Gazebo GUI.

Draws a pulsing ground ring + a translucent light column around the walking
actor, color-coded by mission state (green=FOLLOWING, amber=TRACKING,
red blinking=TARGET_LOST, hidden otherwise). Markers are sent to the GUI's
/marker service only, so they are NOT visible to the drone camera and cannot
disturb the perception pipeline.

The actor never publishes its pose on /world/.../pose/info, so its position
is recomputed here: the node parses the same trajectory waypoints out of the
world SDF that Gazebo animates the actor with, and interpolates them against
sim time from /world/<world>/stats. Linear interpolation vs. Gazebo's
tension-spline differs by a few cm at most — invisible with a 1.1 m ring.

Requires the gz python bindings (python3-gz-transport13 / python3-gz-msgs10)
and GZ_PARTITION to match the running server (defaults to drone_sim).
"""
import math
import os
import re
import sys

# Must be set before gz.transport is imported — the partition is read once.
os.environ.setdefault("GZ_PARTITION", "drone_sim")

import rclpy
from rclpy.node import Node

from drone_interfaces.msg import MissionState

try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.marker_pb2 import Marker
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.world_stats_pb2 import WorldStatistics
    GZ_AVAILABLE = True
except ImportError:
    GZ_AVAILABLE = False

# Palette tuned for the warehouse scene: cool teal reads "locked" against the
# warm brick without neon-green overload; emissive stays at ~40% of the base
# color so the markers keep shading instead of glowing flat.
STATE_STYLE = {
    #                 r     g     b   breathe  lost-pulse
    "FOLLOWING":   (0.00, 0.90, 0.72, True,  False),
    "TRACKING":    (1.00, 0.72, 0.15, False, False),
    "TARGET_LOST": (1.00, 0.20, 0.12, False, True),
}

RING_ID = 1
GEM_ID = 2
UPDATE_HZ = 10.0

RING_DIAMETER = 0.95
RING_ALPHA = 0.30
GEM_SIZE = 0.13
GEM_BASE_Z = 2.05
LOCK_ANIM_S = 0.45  # one-shot lock-on: ring eases 1.8x -> 1.0x on FOLLOWING


def quat_axis(axis, angle):
    """Quaternion (x,y,z,w) for rotation about a base axis: 0=X, 1=Y, 2=Z."""
    q = [0.0, 0.0, 0.0, math.cos(angle / 2)]
    q[axis] = math.sin(angle / 2)
    return tuple(q)


def quat_z(angle):
    return quat_axis(2, angle)


def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


class SimTargetHighlightNode(Node):
    def __init__(self):
        super().__init__("sim_target_highlight_node")
        self.declare_parameter(
            "world_sdf",
            os.path.expanduser("~/ardupilot_gazebo/worlds/iris_classroom.sdf"))
        self.declare_parameter("world_name", "iris_classroom")
        self.declare_parameter("actor_name", "perception_test_person")

        world_sdf = self.get_parameter("world_sdf").value
        world_name = self.get_parameter("world_name").value
        actor_name = self.get_parameter("actor_name").value

        self.waypoints_ = self.parse_waypoints(world_sdf, actor_name)
        self.loop_time_ = self.waypoints_[-1][0]
        self.get_logger().info(
            f"{len(self.waypoints_)} waypoints, loop {self.loop_time_:.1f}s")

        self.sim_time_ = None
        self.state_ = "IDLE"
        self.state_stamp_ = None  # wall time of last /mission/state
        self.state_change_sim_t_ = None  # sim time of last state switch
        self.visible_ = False  # markers currently shown in the GUI

        self.gz_ = GzNode()
        self.gz_.subscribe(
            WorldStatistics, f"/world/{world_name}/stats", self.stats_cb)

        self.create_subscription(
            MissionState, "/mission/state", self.mission_cb, 10)
        self.create_timer(1.0 / UPDATE_HZ, self.update)

    @staticmethod
    def parse_waypoints(path, actor_name):
        sdf = open(path).read()
        actor = re.search(
            r'<actor name="%s">.*?</actor>' % re.escape(actor_name), sdf, re.S)
        if actor is None:
            raise RuntimeError(f"actor '{actor_name}' not found in {path}")
        wps = []
        for t, pose in re.findall(
                r"<waypoint>\s*<time>([^<]+)</time>\s*<pose>([^<]+)</pose>",
                actor.group(0)):
            vals = [float(v) for v in pose.split()]
            wps.append((float(t), vals[0], vals[1]))
        if len(wps) < 2:
            raise RuntimeError(f"no usable waypoints for '{actor_name}'")
        return wps

    def stats_cb(self, msg):
        self.sim_time_ = msg.sim_time.sec + msg.sim_time.nsec * 1e-9

    def mission_cb(self, msg):
        if msg.state != self.state_:
            self.state_change_sim_t_ = self.sim_time_
        self.state_ = msg.state
        self.state_stamp_ = self.get_clock().now()

    def person_xy(self, t):
        t = math.fmod(t, self.loop_time_)
        for (t0, x0, y0), (t1, x1, y1) in zip(self.waypoints_,
                                              self.waypoints_[1:]):
            if t0 <= t <= t1:
                f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                return x0 + f * (x1 - x0), y0 + f * (y1 - y0)
        return self.waypoints_[-1][1], self.waypoints_[-1][2]

    def update(self):
        # mission_manager streams at 10Hz — silence means the stack is down
        stale = (self.state_stamp_ is None or
                 (self.get_clock().now() - self.state_stamp_).nanoseconds
                 > 1.0e9)
        style = STATE_STYLE.get(self.state_)
        if stale or style is None or self.sim_time_ is None:
            if self.visible_:
                self.delete_markers()
            return

        r, g, b, breathe, lost = style
        t = self.sim_time_
        x, y = self.person_xy(t)

        ring_d = RING_DIAMETER
        ring_a = RING_ALPHA
        gem_a = 0.9

        if breathe:
            ring_d *= 1.0 + 0.04 * math.sin(2 * math.pi * 1.1 * t)
        if lost:
            # soft red pulse instead of a hard on/off blink
            f = 0.5 + 0.5 * math.sin(2 * math.pi * 2.0 * t)
            ring_a = 0.15 + 0.30 * f
            gem_a = 0.45 + 0.45 * f

        # one-shot lock-on: ring eases in from 1.8x on entering FOLLOWING
        if (self.state_ == "FOLLOWING" and self.state_change_sim_t_ is not None
                and t - self.state_change_sim_t_ < LOCK_ANIM_S):
            p = (t - self.state_change_sim_t_) / LOCK_ANIM_S
            ease = 1.0 - (1.0 - p) ** 3
            ring_d *= 1.8 - 0.8 * ease
            ring_a *= 0.4 + 0.6 * ease

        self.send_marker(RING_ID, Marker.CYLINDER, x, y, 0.02,
                         ring_d, ring_d, 0.025, r, g, b, ring_a)

        # small spinning gem bobbing above the head — points the eye at the
        # person without covering him
        gem_z = GEM_BASE_Z + 0.04 * math.sin(2 * math.pi * 0.5 * t)
        self.send_marker(GEM_ID, Marker.BOX, x, y, gem_z,
                         GEM_SIZE, GEM_SIZE, GEM_SIZE, r, g, b, gem_a,
                         spin=0.7 * t)
        self.visible_ = True

    def send_marker(self, mid, mtype, x, y, z, sx, sy, sz, r, g, b, a,
                    spin=None):
        m = Marker()
        m.ns = "target_highlight"
        m.id = mid
        m.action = Marker.ADD_MODIFY
        m.type = mtype
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        if spin is None:
            m.pose.orientation.w = 1.0
        else:
            # corner-up diamond stance (45 deg about X, then Y) spinning in Z
            qx, qy, qz, qw = quat_mul(
                quat_z(spin), quat_mul(quat_axis(0, math.pi / 4),
                                       quat_axis(1, math.pi / 4)))
            m.pose.orientation.x = qx
            m.pose.orientation.y = qy
            m.pose.orientation.z = qz
            m.pose.orientation.w = qw
        m.scale.x, m.scale.y, m.scale.z = sx, sy, sz
        m.material.diffuse.r, m.material.diffuse.g = r, g
        m.material.diffuse.b, m.material.diffuse.a = b, a
        m.material.ambient.CopyFrom(m.material.diffuse)
        # emissive at ~40% keeps shading instead of flat neon glow
        m.material.emissive.r = 0.4 * r
        m.material.emissive.g = 0.4 * g
        m.material.emissive.b = 0.4 * b
        m.material.emissive.a = a
        self.gz_.request("/marker", m, Marker, Empty, 50)

    def delete_markers(self):
        for mid in (RING_ID, GEM_ID):
            m = Marker()
            m.ns = "target_highlight"
            m.id = mid
            m.action = Marker.DELETE_MARKER
            self.gz_.request("/marker", m, Marker, Empty, 50)
        self.visible_ = False


def main():
    rclpy.init()
    if not GZ_AVAILABLE:
        print("sim_target_highlight_node: gz python bindings missing "
              "(python3-gz-transport13 + python3-gz-msgs10) — exiting",
              file=sys.stderr)
        return
    node = SimTargetHighlightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.delete_markers()


if __name__ == "__main__":
    main()
