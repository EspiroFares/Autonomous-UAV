# CLAUDE.md — Person-Following Drone

## What this is

Autonomous **person-following drone**. ROS 2 Jazzy running **natively** on a Raspberry Pi 4
(Ubuntu 24.04 — no Docker). MediaPipe Pose detects a person → world model → mission state
machine → P-controller → MAVROS → ArduPilot in GUIDED mode. Altitude from downward lidar,
horizontal position from optical flow, **no GPS**. Flight-tested and working.

**Goal:** portfolio-grade project for GitHub, CV, and interviews.

Pipeline (one line per hop, all custom nodes):

```
camera_driver(.py) → image_preprocessing → person_detector(.py, MediaPipe)
→ person_tracker (EMA) → target_estimator (pinhole) → world_model
→ mission_manager (state machine) → follow_controller (P + altitude hold)
→ setpoint_validation → fcu_bridge → MAVROS → ArduPilot
```

## Environment

| | |
|---|---|
| Pi workspace | `~/Autonomous-UAV/software/drone_ws` (build + run here) |
| Mac repo | `~/Desktop/Drone/Drone` (editing only — cannot build ROS) |
| FC link | UART `serial:///dev/ttyAMA5:921600`, MAVLink 2 via MAVROS |
| FC | ArduPilot (ArduCopter 4.7.0-dev), 3-inch quad, hovers fine in LOITER |
| RMW | **CycloneDDS required**: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in every terminal (FastDDS SHM crashes on the Pi) |
| MAVROS config | `mav_frame: BODY_NED` is set permanently in `/opt/ros/jazzy/share/mavros/launch/apm_config.yaml` under `setpoint_velocity` — an apt upgrade of MAVROS may silently revert it |

## Architecture principles (non-negotiable)

1. **`fcu_bridge_node` is the sole ROS ↔ FC gateway.** No other node touches MAVROS topics for control.
2. **Safety has veto** — `safety_supervision_node` (planned) can override any setpoint.
3. **FC owns stabilization.** ROS sends high-level velocity setpoints (vx, vz, yaw_rate) only.
4. **Optical flow + lidar stay on the FC.** They are stability-critical; the Pi never takes them over.

## Commands

```bash
# Start everything (MAVROS + stream requests + all 10 nodes, respawn on crash):
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash && source ~/Autonomous-UAV/software/drone_ws/install/setup.bash
ros2 launch drone_bringup real_chain.launch.py

# Build routine — ALWAYS kill nodes first (building a running binary corrupts it):
pkill -9 -f <node_name>
colcon build --packages-select <pkg> && source install/setup.bash

# Full cleanup when topics/nodes act dead:
pkill -9 -f mavros; pkill -9 -f _node; ros2 daemon stop && ros2 daemon start

# Pre-flight sanity (manual until preflight script exists):
ros2 topic echo /vehicle/height        # ~0.3 on ground; SILENT = do not fly
ros2 topic hz /target/detections       # expect 5–7 Hz
vcgencmd measure_temp                  # < 75°C
```

**Flight procedure:** arm in LOITER on open floor → take off → throttle stick to mid →
switch CH6 high = GUIDED (FLTMODE6=4) → ROS takes over. **LOITER switch is the rescue** —
RC sticks are ignored in GUIDED.

## Gotchas — read before debugging

- **`Exec format error` on a node** = corrupt binary from building while the node ran.
  Fix: `rm -rf build/<pkg> install/<pkg>` + rebuild. Prevent: pkill before build.
- **FC does not stream data unsolicited.** `/vehicle/height` (rangefinder, msg 132/173)
  dies after every FC reboot until `set_message_interval` is called again — the launch
  file does this, but only at launch time. Silent height = altitude hold silently dead.
- **`/mavros/local_position/*` is empty without GPS** — no EKF origin means ArduPilot
  never sends LOCAL_POSITION_NED, regardless of stream requests. That's why altitude
  uses `/mavros/rangefinder/rangefinder` (via `/vehicle/height`), not odom.
- **MAVROS publishers are BEST_EFFORT.** Subscribing (or `ros2 topic echo`) with default
  reliable QoS shows nothing: add `--qos-reliability best_effort`.
- **FC params:** `ros2 service call /mavros/param/pull ... "{force_pull: true}"` before
  `ros2 param get /mavros/param <NAME>` — the cache starts empty.
- **Mode check:** use `msg->guided` from `/mavros/state`, not the mode string (MAVROS
  reports some modes wrong on 4.7.0-dev, e.g. GUIDED_NOGPS shows as LOITER).
- **Velocity command `vz=0` is NOT altitude hold** — it means "hold zero climb rate" and
  drifts. This is why follow_controller runs a continuous lidar-based altitude P-loop
  (target 1.5 m, no deadband — a z-deadband creates a drift/correct limit cycle).
- **Lidar measures the surface below.** Flying over furniture jumps the height reference
  → the drone climbs. Keep the flight path over open floor.
- **EKF "stopped aiding" spam on the ground is normal** — optical flow has no texture
  until airborne. It cannot be validated on the ground.
- **EKF sources** (set, verified): `EK3_SRC1_POSXY=0, VELXY=5 (flow), POSZ=2 (lidar), VELZ=0`.

## Control design (decisions, not code — read the code for details)

- **Yaw from angle** (`atan2(y,x)`), not lateral offset — offset scales with distance and
  caused violent overshoot. Deadband ±5° so perception jitter doesn't wiggle the drone.
- **No deadband on altitude** — lidar is clean; continuous correction is what LOITER does.
- **Latency budget matters more than FPS.** 5–7 Hz detection is fine; ~1 s of pipeline
  latency was not. Fixes that got it to ~240 ms at `/target/track`: QoS depth 1 +
  BEST_EFFORT on image topics, latest-frame-only threading in the detector, EMA α=0.6,
  camera at 15 fps. Don't undo these.
- **Watchdog pattern (all timer-driven nodes):** input older than 0.5 s = invalid →
  degrade to safe output (zeros / altitude-hold-only). mission_manager additionally has a
  1.0 s grace period before FOLLOWING → TARGET_LOST so single missed detections don't
  cause stop-start lurching.
- Any node crash → system degrades to hover within ~0.5–1.5 s; launch respawns the node.

## Conventions

- `ament_cmake`, C++17, `-Wall -Wextra -Wpedantic`; MediaPipe/camera nodes in Python.
- Node classes `PascalCase`, members `trailing_underscore_`, timers for periodic output.
- `drone_interfaces` builds first; message definitions live there — read the `.msg` files.
- Mock nodes (`mock_fcu_node`, `mock_target_node`) exist for hardware-free testing.

## Current focus (2026-07-08)

1. **Safety layer**: `safety_supervision_node` + `hold_failsafe_node` (drone_safety pkg).
2. **Preflight check script** — designed, not yet implemented (`drone_bringup/scripts/`).
3. ROS parameters instead of hardcoded gains (tune without rebuilding).
4. Camera calibration (focal length 600 px is a guess → distance estimate ±30 %).
5. Real confidence from MediaPipe visibility scores (currently hardcoded 0.9).
6. LinkedIn video (~20 s hook) + GitHub README with architecture diagram.
