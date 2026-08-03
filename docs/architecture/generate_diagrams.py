#!/usr/bin/env python3
"""Generate the architecture figures in docs/architecture/.

Node names, topic names and message flow below are transcribed from the actual
sources under software/drone_ws/src -- keep them in sync when the graph changes,
then re-run:

    python3 docs/architecture/generate_diagrams.py

PNG output additionally needs librsvg (`brew install librsvg`).
"""

import os
import subprocess

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

SANS = "Inter, 'Helvetica Neue', Helvetica, Arial, sans-serif"

BG = "#0B0E14"
PANEL_FILL = "#11151D"
PANEL_STROKE = "rgba(255,255,255,0.07)"
PILL_STROKE = "rgba(255,255,255,0.10)"
LANE_TEXT = "#98A1AE"
TITLE = "#F2F5F9"
SUB = "#8B93A1"
EDGE = "#D2D7DE"
LABEL = "#A9B1BE"
PLAN = "#E8A24C"

# box gradients: (top, bottom, border)
KINDS = {
    "blue":   ("#5B93D6", "#35679F", "#7FB0E6"),
    "orange": ("#DD8C3C", "#B96A1C", "#EFA85E"),
    "green":  ("#4CAE72", "#2E8551", "#6ECB92"),
    "red":    ("#D75E52", "#AE3A30", "#EC8074"),
    "purple": ("#B15FD0", "#8A34A9", "#C982E2"),
    "gold":   ("#D8B03C", "#AF8618", "#EAC85F"),
}


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def wbold(s, size):
    return len(s) * size * 0.565


class Fig:
    def __init__(self, w, h):
        self.w, self.h = w, h
        self.body = []
        self.top = []          # drawn above boxes (labels)

    # ── primitives ──────────────────────────────────────────────────────────
    def text(self, x, y, s, size=13, weight=400, fill=LABEL, anchor="start",
             ls=0, layer=None):
        (layer if layer is not None else self.body).append(
            f'<text x="{x:.1f}" y="{y:.1f}" font-family="{SANS}" '
            f'font-size="{size}" font-weight="{weight}" fill="{fill}" '
            f'text-anchor="{anchor}" letter-spacing="{ls}">{esc(s)}</text>'
        )

    def rect(self, x, y, w, h, r=10, fill="none", stroke="none", sw=1,
             dash=None, opacity=1.0, stroke_opacity=1.0):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.body.append(
            f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" '
            f'rx="{r}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}" '
            f'stroke-opacity="{stroke_opacity}" opacity="{opacity}"{d}/>'
        )

    def line(self, pts, stroke=EDGE, sw=2, dash=None, marker="arrow",
             marker_start=None):
        d = "M " + " L ".join(f"{x:.1f} {y:.1f}" for x, y in pts)
        a = f' stroke-dasharray="{dash}"' if dash else ""
        m = f' marker-end="url(#{marker})"' if marker else ""
        ms = f' marker-start="url(#{marker_start})"' if marker_start else ""
        self.body.append(
            f'<path d="{d}" fill="none" stroke="{stroke}" stroke-width="{sw}" '
            f'stroke-linecap="butt" stroke-linejoin="miter"{a}{m}{ms}/>'
        )

    def junction(self, x, y, fill=EDGE):
        self.body.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="4" '
                         f'fill="{fill}"/>')

    # ── composites ──────────────────────────────────────────────────────────
    def lane(self, x, y, w, h, title):
        self.rect(x, y - 68, w, 44, r=9, fill="#0E1219", stroke="#FFFFFF",
                  sw=1, stroke_opacity=0.10)
        self.text(x + w / 2, y - 39, title, size=14.5, weight=600,
                  fill=LANE_TEXT, anchor="middle", ls=1.6)
        self.rect(x, y, w, h, r=15, fill=PANEL_FILL, stroke="#FFFFFF", sw=1,
                  stroke_opacity=0.07)

    def box(self, x, y, w, h, label, kind, ghost=False, caption=None):
        top, bot, border = KINDS[kind]
        gid = f"grad_{kind}"
        op = 0.42 if ghost else 1.0
        dash = "7 5" if ghost else None
        self.body.append(f'<g opacity="{op}">')
        if not ghost:
            self.body.append('<g filter="url(#shadow)">')
        self.rect(x, y, w, h, r=11, fill=f"url(#{gid})", stroke=border,
                  sw=1.3, stroke_opacity=0.55, dash=dash)
        if not ghost:
            self.body.append('</g>')
        size = 16
        while wbold(label, size) > w - 26 and size > 11:
            size -= 0.5
        self.text(x + w / 2, y + h / 2 + size * 0.36, label, size=size,
                  weight=700, fill="#FFFFFF", anchor="middle", ls=0.1)
        self.body.append('</g>')
        if caption:
            self.text(x + w / 2, y + h + 17, caption, size=11.5,
                      fill="#6E7889", anchor="middle")

    def header(self, title, subtitle=None):
        self.text(30, 56, title, size=30, weight=700, fill=TITLE, ls=-0.3)
        if subtitle:
            self.text(30, 92, subtitle, size=15.5, fill=SUB)

    def footnote(self, y, text):
        self.text(30, y, text, size=13, fill="#6E7889")

    def legend(self, x, y, rows, w=None):
        """rows = [(colour, dash, text)] — bordered box, top-right style."""
        tw = max(len(t) for _, _, t in rows) * 7.6 + 96
        w = w or tw
        h = 22 * len(rows) + 24
        self.rect(x - w, y, w, h, r=9, fill="#0E1219", stroke="#FFFFFF", sw=1,
                  stroke_opacity=0.12)
        ry = y + 24
        for col, dash, txt in rows:
            self.line([(x - w + 18, ry - 4), (x - w + 62, ry - 4)], stroke=col,
                      sw=2.4, dash=dash, marker=None)
            self.text(x - w + 76, ry, txt, size=13.5, fill="#AFB7C4")
            ry += 22

    def render(self):
        defs = ['<defs>']
        for kind, (top, bot, _) in KINDS.items():
            defs.append(
                f'<linearGradient id="grad_{kind}" x1="0" y1="0" x2="0.25" y2="1">'
                f'<stop offset="0" stop-color="{top}"/>'
                f'<stop offset="1" stop-color="{bot}"/></linearGradient>'
            )
        for name, col in (("arrow", EDGE), ("arrow_plan", PLAN)):
            defs.append(
                f'<marker id="{name}" viewBox="0 0 10 10" refX="9.2" refY="5" '
                f'markerWidth="6.5" markerHeight="6.5" '
                f'orient="auto-start-reverse">'
                f'<path d="M 0 0.6 L 10 5 L 0 9.4 Z" fill="{col}"/></marker>'
            )
        defs.append(
            '<filter id="shadow" x="-25%" y="-25%" width="150%" height="160%">'
            '<feDropShadow dx="0" dy="4" stdDeviation="6" flood-color="#000000" '
            'flood-opacity="0.4"/></filter>'
        )
        defs.append('</defs>')
        return (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" '
            f'height="{self.h}" viewBox="0 0 {self.w} {self.h}" '
            f'font-family="{SANS}">\n' + "\n".join(defs) + "\n"
            f'<rect width="{self.w}" height="{self.h}" fill="{BG}"/>\n'
            + "\n".join(self.body) + "\n"
            + "\n".join(self.top) + "\n</svg>\n"
        )


# ═════════════════════════════════════════════════════════════════════════════
# System Architecture Overview
# ═════════════════════════════════════════════════════════════════════════════
def system_overview():
    W, H = 2010, 880
    f = Fig(W, H)
    f.header("System Architecture Overview",
             "Vision on the companion computer decides where to go; the flight "
             "controller keeps the aircraft in the air.")

    f.legend(W - 30, 30, [
        (EDGE, None, "Primary data / command flow"),
        (PLAN, "8 6", "Built, not yet wired into the flight path"),
    ])

    PY = 180
    # each lane panel only reaches as far down as its own content
    lanes = [
        ("SENSORS / HARDWARE", 30,   210, 800),
        ("PERCEPTION",         270,  300, 680),
        ("STATE",              600,  240, 440),
        ("BEHAVIOR / MISSION", 870,  240, 440),
        ("CONTROL / SAFETY",   1140, 280, 806),
        ("VEHICLE INTERFACE",  1450, 250, 680),
        ("FLIGHT CONTROLLER",  1730, 250, 800),
    ]
    for title, x, w, bottom in lanes:
        f.lane(x, PY, w, bottom - PY, title)

    def box(i, y, label, kind, ghost=False, caption=None):
        _, x, w, _ = lanes[i]
        f.box(x + 18, y, w - 36, 60, label, kind, ghost=ghost, caption=caption)

    box(0, 230, "Pi Camera", "blue")
    box(0, 470, "Downward Lidar", "blue")
    box(0, 590, "Optical Flow", "blue")
    box(0, 710, "IMU / Baro", "blue")

    box(1, 230, "Camera Driver", "green")
    box(1, 350, "Person Detection", "green")
    box(1, 470, "Person Tracking", "green")
    box(1, 590, "Target Estimation", "green")

    box(2, 350, "World Model", "orange")
    box(3, 350, "Mission Manager", "red")

    box(4, 350, "Follow Controller", "purple")
    box(4, 470, "Setpoint Validation", "purple")
    box(4, 590, "Hold / Failsafe", "purple")
    box(4, 710, "Safety Supervision", "purple")

    box(5, 470, "FCU Bridge", "orange")
    box(5, 590, "MAVROS", "orange")

    box(6, 350, "Motor Mixer + ESC", "gold")
    box(6, 470, "Attitude & Rate Loops", "gold")
    box(6, 590, "GUIDED Velocity Control", "gold")
    box(6, 710, "EKF3 Sensor Fusion", "gold")

    # ── perception chain ────────────────────────────────────────────────────
    f.line([(222, 260), (288, 260)])
    f.line([(420, 290), (420, 350)])
    f.line([(420, 410), (420, 470)])
    f.line([(420, 530), (420, 590)])

    # ── perception → state → behaviour → control ────────────────────────────
    f.line([(552, 620), (585, 620), (585, 380), (618, 380)])
    f.line([(822, 372), (888, 372)])
    f.line([(1092, 372), (1158, 372)])
    f.line([(700, 350), (700, 288), (1230, 288), (1230, 350)])

    # ── control → vehicle interface → flight controller ─────────────────────
    f.line([(1280, 410), (1280, 470)])
    f.line([(1402, 490), (1468, 490)])
    f.line([(1575, 530), (1575, 590)])
    f.line([(1682, 620), (1748, 620)])
    f.line([(1855, 710), (1855, 650)])
    f.line([(1855, 590), (1855, 530)])
    f.line([(1855, 470), (1855, 410)])

    # ── feedback ────────────────────────────────────────────────────────────
    f.line([(1650, 470), (1650, 250), (760, 250), (760, 350)])
    f.text(1200, 238, "vehicle state  ·  /vehicle/status  ·  /vehicle/odom",
           size=13, fill=LABEL, anchor="middle")
    f.line([(1468, 485), (1428, 485), (1428, 380), (1402, 380)])
    f.text(1440, 440, "altitude · /vehicle/height", size=13, fill=LABEL)

    # ── safety chain ────────────────────────────────────────────────────────
    f.line([(1240, 530), (1240, 590)])
    f.line([(1330, 710), (1330, 650)])
    f.text(1342, 688, "veto", size=13, fill=LABEL)
    # Built and running, but fcu_bridge still reads setpoint_validated. The
    # switch is the last step, after both nodes are ground-tested.
    f.line([(1402, 620), (1442, 620), (1442, 515), (1468, 515)],
           stroke=PLAN, dash="8 6", marker="arrow_plan")
    f.text(1452, 570, "not wired in yet", size=12, fill=PLAN)

    # ── flight-critical sensors are wired to the FC, never to the Pi ────────
    for y in (500, 620, 740):
        f.line([(222, y), (252, y), (252, 840)], marker=None)
        f.junction(252, y, EDGE)
    f.line([(252, 840), (1855, 840), (1855, 770)])
    f.text(1040, 828,
           "wired directly to the flight controller — the Pi never reads them",
           size=13.5, fill=LABEL, anchor="middle")

    return f


# ═════════════════════════════════════════════════════════════════════════════
# ROS Node & Topic Graph
# ═════════════════════════════════════════════════════════════════════════════
def ros_graph():
    W, H = 1965, 880
    f = Fig(W, H)
    f.header("ROS Node & Topic Graph",
             "Arrows follow ROS publish → subscribe direction; labels show the "
             "topic name carried on each link.")

    f.legend(W - 30, 30, [
        (EDGE, None, "ROS 2 topic"),
        (PLAN, "8 6", "Published, no subscriber yet"),
    ])

    PY = 180
    # each lane panel only reaches as far down as its own content
    lanes = [
        ("EXTERNAL",          30,   200, 800),
        ("PERCEPTION",        265,  330, 680),
        ("STATE",             630,  270, 440),
        ("BEHAVIOR",          935,  280, 440),
        ("CONTROL / SAFETY",  1250, 310, 800),
        ("VEHICLE INTERFACE", 1595, 340, 720),
    ]
    for title, x, w, bottom in lanes:
        f.lane(x, PY, w, bottom - PY, title)

    def box(i, y, label, kind, ghost=False):
        _, x, w, _ = lanes[i]
        f.box(x + 18, y, w - 36, 60, label, kind, ghost=ghost)

    box(0, 230, "Pi Camera", "blue")
    box(0, 710, "Flight Controller", "blue")

    box(1, 230, "camera_driver_node", "green")
    box(1, 350, "person_detector_node", "green")
    box(1, 470, "person_tracker_node", "green")
    box(1, 590, "target_estimator_node", "green")

    box(2, 350, "world_model_node", "orange")
    box(3, 350, "mission_manager_node", "red")

    box(4, 350, "follow_controller_node", "purple")
    box(4, 470, "setpoint_validation_node", "purple")
    box(4, 590, "hold_failsafe_node", "purple")
    box(4, 710, "safety_supervision_node", "purple")

    box(5, 470, "fcu_bridge_node", "orange")
    box(5, 630, "mavros", "orange")

    def lab(x, y, s, anchor="start", size=13.5, fill=LABEL):
        f.text(x, y, s, size=size, fill=fill, anchor=anchor)

    # ── perception chain ────────────────────────────────────────────────────
    f.line([(212, 260), (283, 260)])
    lab(247, 248, "CSI-2", anchor="middle")

    f.line([(430, 290), (430, 350)])
    lab(442, 326, "/camera/image_raw")

    f.line([(430, 410), (430, 470)])
    lab(442, 446, "/target/detections")

    f.line([(430, 530), (430, 590)])
    lab(442, 566, "/target/track")

    # ── perception → state → behaviour → control ────────────────────────────
    f.line([(577, 620), (612, 620), (612, 380), (648, 380)])
    lab(624, 432, "/target/state")

    f.line([(882, 372), (953, 372)])
    lab(917, 336, "/world/target_valid", anchor="middle")

    f.line([(1197, 372), (1268, 372)])
    lab(1232, 336, "/mission/follow_enabled", anchor="middle")

    f.line([(720, 350), (720, 290), (1330, 290), (1330, 350)])
    lab(1025, 278, "/world/target_pos_relative", anchor="middle")

    # ── control chain ───────────────────────────────────────────────────────
    f.line([(1405, 410), (1405, 470)])
    lab(1417, 446, "/control/setpoint_raw")

    f.line([(1542, 500), (1613, 500)])
    lab(1577, 458, "/control/setpoint_validated", anchor="middle")

    # ── fcu_bridge ↔ mavros ↔ flight controller ────────────────────────────
    f.line([(1625, 530), (1625, 630)], marker="arrow", marker_start="arrow")
    lab(1641, 555, "↓ /mavros/setpoint_velocity/cmd_vel_unstamped", size=12)
    lab(1641, 575, "↑ /mavros/state  ·  /mavros/imu/data", size=12)
    lab(1641, 595, "↑ /mavros/rangefinder/rangefinder", size=12)
    lab(1641, 615, "↑ /mavros/local_position/odom  (empty — no GPS)", size=12)

    f.line([(1765, 690), (1765, 840), (130, 840), (130, 770)],
           marker="arrow", marker_start="arrow")
    lab(950, 828, "MAVLink 2  ·  UART /dev/ttyAMA5 @ 921600", anchor="middle")

    # ── feedback ────────────────────────────────────────────────────────────
    f.line([(1740, 470), (1740, 240), (800, 240), (800, 350)])
    lab(1270, 228, "/vehicle/status   ·   /vehicle/odom", anchor="middle")

    f.line([(1660, 470), (1660, 310), (1470, 310), (1470, 350)])
    lab(1565, 298, "/vehicle/height", anchor="middle")

    # ── drone_safety ────────────────────────────────────────────────────────
    f.line([(1340, 530), (1340, 590)])
    lab(1328, 566, "/control/setpoint_validated", anchor="end")

    f.line([(1268, 515), (1232, 515), (1232, 740), (1268, 740)])
    lab(1220, 645, "/control/setpoint_validated", anchor="end")

    f.line([(1470, 710), (1470, 650)])
    lab(1482, 688, "/safety/status")

    f.line([(1917, 500), (1945, 500), (1945, 740), (1542, 740)])
    lab(1750, 728, "/vehicle/status  ·  /vehicle/height", anchor="middle")

    # Published every tick, but fcu_bridge still subscribes to
    # /control/setpoint_validated. Switching it over is the last step.
    f.line([(1542, 620), (1556, 620), (1556, 515), (1613, 515)],
           stroke=PLAN, dash="8 6", marker="arrow_plan")
    lab(1530, 560, "/control/setpoint_safe", anchor="end", fill=PLAN)

    return f


def write(fig, name):
    path = os.path.join(OUT_DIR, name + ".svg")
    with open(path, "w") as fh:
        fh.write(fig.render())
    png = os.path.join(OUT_DIR, name + ".png")
    try:
        subprocess.run(["rsvg-convert", "-o", png, path], check=True)
        print(f"wrote {path} and {png}")
    except (OSError, subprocess.CalledProcessError):
        print(f"wrote {path} (install librsvg for PNG output)")


if __name__ == "__main__":
    write(system_overview(), "system_overview")
    write(ros_graph(), "ros_architecture")
