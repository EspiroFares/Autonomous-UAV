# CLAUDE.md — Drone Project Context

## Project Summary

DIY indoor **face-following drone** using ROS 2 (Jazzy) on a Raspberry Pi 4 companion computer. The companion computer runs all autonomy logic; a separate Flight Controller handles low-level stabilization and hover. The stack is developed and tested in Docker, targeting deployment on real hardware once the new FC arrives.

**Goal:** Strong enough for GitHub, CV, interviews, and eventual report/presentation.

**Current milestone:** "Target in → yaw command out via mock FCU"

---

## Hardware Context

| Sensor | Where it lives | Why |
|---|---|---|
| Pi Camera | Raspberry Pi (ROS side) | Perception input |
| Optical Flow sensor | Flight Controller | Stability/state estimation — stays on FC |
| Downward Lidar/Rangefinder | Flight Controller | Hover/altitude — stays on FC |
| Flight Controller | FC hardware | Stabilisation, hover, motor control |

**The old FC broke. A new one is on order (~2 weeks).** The current phase is a software architecture sprint — build the full autonomy stack using `mock_fcu_node` so that when the new FC arrives, it's just an integration task, not a starting point.

**Raspberry Pi runs:** perception, world model, mission logic, control, safety — all high-level.
**FC runs:** stabilisation, hover, low-level control loops. ROS does NOT replace these.

---

## Technology Stack

| Layer | Technology |
|---|---|
| OS | Ubuntu (Docker, ROS Jazzy base image) |
| Framework | ROS 2 Jazzy |
| Language | C++17 (rclcpp) |
| Vision | OpenCV 4 — Haar Cascade face detection |
| Video I/O | cv_bridge, image_transport |
| Messages | Custom ROS 2 `.msg` definitions |
| Build | CMake + ament_cmake |
| Container | Docker + Docker Compose |
| Hardware (target) | Raspberry Pi 4, Pi Camera, Flight Controller |

---

## Repository Structure

```
Drone/
├── CLAUDE.md                            ← this file
├── README.md
├── haarcascade_frontalface_default.xml  ← should move to models/ or data/ eventually
├── docs/architecture/
│   ├── ROS_architecture-3.drawio        ← current ROS node/topic diagram (draw.io)
│   ├── ROS_architecture.drawio-3.png    ← rendered PNG
│   ├── System_Overview-2.drawio         ← current system layers diagram (draw.io)
│   └── System_Overview.drawio.png       ← rendered PNG
├── hardware/                            ← placeholder (empty)
└── software/
    ├── Dockerfile                       ← ROS Jazzy base image
    ├── compose.yml                      ← mounts drone_ws into container
    └── drone_ws/src/
        ├── drone_interfaces/            ← custom message definitions ✓
        ├── drone_state/                 ← mock_fcu_node ✓, world_model_node ✓
        ├── drone_vision/                ← vision_node ✓ (to be renamed drone_perception)
        ├── drone_control/               ← PLACEHOLDER
        ├── drone_behavior/              ← PLACEHOLDER
        ├── drone_bringup/               ← PLACEHOLDER
        ├── drone_perception/            ← EMPTY (future home of full perception pipeline)
        ├── drone_safety/                ← EMPTY
        ├── drone_sim/                   ← PLANNED (not yet created)
        ├── drone_description/           ← PLANNED (not yet created)
        └── drone_test/                  ← PLANNED (not yet created)
```

---

## Architectural Principles (non-negotiable)

### 1. FCU Bridge is the sole gateway
All ROS ↔ FC communication goes **only** through `fcu_bridge_node`. No direct connections between:
- `follow_controller_node` and Flight Controller
- `safety_supervision_node` and Flight Controller
- `setpoint_validation_node` and Flight Controller

Flow: ROS nodes → `fcu_bridge_node` → FC → `fcu_bridge_node` → ROS topics

### 2. Safety has veto
`safety_supervision_node` can block any setpoint and trigger hold/failsafe at any time, overriding mission logic.

### 3. FC handles stabilisation — ROS does not
ROS generates high-level setpoints (vx, vy, vz, yaw_rate). The FC's inner loops remain untouched.

### 4. Optical flow and downward lidar stay on FC
They are stability-critical. Pi does not take them over.

---

## ROS Packages

### `drone_interfaces` — Custom Messages ✓ DONE
Defines all shared message types. Must be built first — all other packages depend on it.

| Message | Purpose | Key Fields |
|---|---|---|
| `VehicleStatus.msg` | FCU state | `connected`, `armed`, `offboard_ready`, `mode`, `hovering` |
| `ControlSetpoint.msg` | Velocity command to FCU | `vx`, `vy`, `vz`, `yaw_rate`, `hold` |
| `TargetState.msg` | Detected face info | `detected`, `confidence`, `yaw_error`, `distance_estimate`, `bbox_*` |
| `MissionState.msg` | Mission phase | `state`, `follow_enabled`, `target_valid` |

---

### `drone_state` — State Management
**Package deps:** rclcpp, std_msgs, nav_msgs, geometry_msgs, drone_interfaces

#### `mock_fcu_node` ✓ DONE
Simulates a Flight Controller for testing without hardware.

- **Subscribes:** `/control/setpoint_validated` (ControlSetpoint)
- **Publishes:** `/vehicle/status` (VehicleStatus) @ 10Hz, `/vehicle/odom` (nav_msgs/Odometry) @ 10Hz
- **Behavior:** Fixed position (0,0,1) in map frame; echoes `yaw_rate`; always armed + OFFBOARD mode; `hovering = !hold`

#### `world_model_node` ✓ DONE
Fuses perception + vehicle state into a unified world representation.

- **Subscribes:** `/vehicle/status`, `/vehicle/odom`, `/target/state`
- **Publishes:** `/world/target_valid` (std_msgs/Bool), `/world/target_pos_relative` (geometry_msgs/Point) @ 10Hz
- **Logic:**
  - `target_valid = vehicle.connected && vehicle.armed && vehicle.offboard_ready && target.detected && target.confidence >= 0.5`
  - `pos.x = distance * cos(yaw_error)` (forward)
  - `pos.y = distance * sin(yaw_error)` (lateral)
  - Guards on all three readiness flags before publishing

---

### `drone_vision` — Face Detection (to be renamed/replaced by drone_perception)
**Package deps:** rclcpp, sensor_msgs, cv_bridge, image_transport, opencv2

#### `vision_node` ✓ DONE
Reads test video, detects faces, publishes annotated frames.

- **Publishes:** `/camera/image_raw` (sensor_msgs/Image) @ 30Hz
- **Video source:** `/home/fares/Drone/test.mp4` (hardcoded — fix later)
- **Cascade:** `/home/fares/Drone/haarcascade_frontalface_default.xml` (hardcoded — fix later)
- **Snapshot on first detection:** `/home/fares/Drone/bevis.jpg`
- **Processing:** rotate 180°, scale 1/4, grayscale, histogram equalization, Haar Cascade (scale=1.1, minNeighbors=3)
- **Note:** Comments in Swedish

---

### `drone_control` — PLACEHOLDER
Will contain: `follow_controller_node`, `setpoint_validation_node`

### `drone_behavior` — PLACEHOLDER
Will contain: `mission_manager_node`

### `drone_bringup` — PLACEHOLDER
Will contain: launch files for full system startup

### `drone_perception` — EMPTY
Will contain: full perception pipeline nodes (see below)

### `drone_safety` — EMPTY
Will contain: `safety_supervision_node`, `hold_failsafe_node`

---

## Full Node & Topic Graph

### Perception pipeline (drone_perception)
```
Pi Camera (hardware)
  → camera_driver_node          → /camera/image_raw
  → image_preprocessing_node    → /camera/image_preprocessed
  → face_detector_node          → /target/detections
  → face_tracker_node           → /target/track
  → target_estimator_node       → /target/state
                                   (also feeds safety_supervision_node)
```

### State layer (drone_state)
```
Flight Controller (hardware)
  ↔ fcu_bridge_node             → /vehicle/status
                                 → /vehicle/odom

world_model_node  ← /vehicle/status, /vehicle/odom, /target/state
                  → /world/target_valid
                  → /world/target_pos_relative
```

### Behavior layer (drone_behavior)
```
mission_manager_node  ← /world/target_valid, /world/target_pos_relative
                      → /mission/follow_enabled
                      → /mission/state
```
**Mission states:** `IDLE` → `SEARCH` → `TRACKING` → `FOLLOWING` → `TARGET_LOST` → `SAFETY_HOLD`
Following is state-driven (state machine), not ad-hoc logic.

### Control & safety layer (drone_control / drone_safety)
```
follow_controller_node  ← /mission/follow_enabled
                        ← /world/target_pos_relative      ← IMPORTANT: needs position
                        ← /vehicle/odom                   ← IMPORTANT: needs drone state
                        → /control/setpoint_raw

setpoint_validation_node  ← /control/setpoint_raw
                          → /control/setpoint_validated

safety_supervision_node  ← /vehicle/status
                         ← /control/setpoint_validated
                         ← /mission/state
                         ← /target/state (or /world/* for target validity)
                         → /control/hold_cmd

hold_failsafe_node  ← /control/hold_cmd
                   → /control/setpoint_safe

fcu_bridge_node  ← /control/setpoint_safe
                 ↔ Flight Controller (hardware)  ← SOLE gateway
```

### Mock FCU (testing only, replaces fcu_bridge_node)
```
mock_fcu_node  ← /control/setpoint_validated      ← NOTE: takes validated, not safe
               → /vehicle/status
               → /vehicle/odom
```

---

## System Architecture Layers (from diagrams)

```
┌─────────────────────────────────────────────────────┐
│ EXTERNAL / SENSORS / HARDWARE                       │
│  Pi Camera  |  Optical Flow  |  Lidar  |  FC        │
└─────────────────────────────────────────────────────┘
                         ↕
┌─────────────────────────────────────────────────────┐
│ VEHICLE INTERFACE                                   │
│  camera_driver_node  |  fcu_bridge_node             │
└─────────────────────────────────────────────────────┘
                         ↕
┌─────────────────────────────────────────────────────┐
│ PERCEPTION  (green)                                 │
│  image_preprocess, face_detector,                   │
│  face_tracker, target_estimator                     │
└─────────────────────────────────────────────────────┘
                         ↕
┌─────────────────────────────────────────────────────┐
│ STATE / WORLD MODEL                                 │
│  world_model_node                                   │
└─────────────────────────────────────────────────────┘
                         ↕
┌─────────────────────────────────────────────────────┐
│ BEHAVIOR / MISSION  (red)                           │
│  mission_manager_node  (state machine)              │
└─────────────────────────────────────────────────────┘
                         ↕
┌─────────────────────────────────────────────────────┐
│ CONTROL / SAFETY  (purple)                          │
│  follow_controller, setpoint_validation,            │
│  safety_supervision, hold_failsafe                  │
└─────────────────────────────────────────────────────┘
                         ↕ (via fcu_bridge only)
┌─────────────────────────────────────────────────────┐
│ FLIGHT CONTROLLER  (yellow)                         │
│  Stabilisation  |  Hover  |  Motor control          │
└─────────────────────────────────────────────────────┘
```

Diagram legend: solid line = primary data/command flow | dashed line = safety override/fallback

---

## Implementation Phases

| Phase | Focus | Contents |
|---|---|---|
| 1 | ✓ Packages + interfaces + skeleton | drone_interfaces (4 msgs), package shells |
| 2 | ✓ Mock chain | mock_fcu, world_model — done; mission_manager, follow_controller, setpoint_validation — next |
| 3 | Perception | camera_driver, image_preprocess, face_detector, face_tracker, target_estimator |
| 4 | Integration | Connect full perception pipeline to the rest |
| 5 | Safety | target timeout, stale vehicle status, hold/failsafe |

---

## Development Status

| Component | Package | Status |
|---|---|---|
| 4 custom messages | drone_interfaces | ✓ Done |
| `mock_fcu_node` | drone_state | ✓ Done |
| `vision_node` | drone_vision | ✓ Done |
| `world_model_node` | drone_state | ✓ Done |
| `mission_manager_node` | drone_behavior | ✗ Not started |
| `follow_controller_node` | drone_control | ✗ Not started |
| `setpoint_validation_node` | drone_control | ✗ Not started |
| `safety_supervision_node` | drone_safety | ✗ Not started |
| `hold_failsafe_node` | drone_safety | ✗ Not started |
| `fcu_bridge_node` | drone_state | ✗ Not started (needs real FC) |
| Full perception pipeline | drone_perception | ✗ Not started |
| Launch files | drone_bringup | ✗ Not started |

---

## Docker Setup

**Dockerfile:** `software/Dockerfile`
- Base: `ros:jazzy-ros-base`
- Installs: build-essential, cmake, git, colcon, rosdep, vcstool
- Sources `/opt/ros/jazzy/setup.bash` in `.bashrc`
- WORKDIR: `/workspaces/drone_ws`

**compose.yml:** `software/compose.yml`
- Service: `drone_ros2`
- Mounts `./drone_ws` → `/workspaces/drone_ws`
- TTY + stdin_open for interactive use

**Build order:** `drone_interfaces` must be built before all other packages.

---

## Hardcoded Paths (fix before deployment)

| Path | Used In | Fix |
|---|---|---|
| `/home/fares/Drone/test.mp4` | vision_node | ROS parameter |
| `/home/fares/Drone/haarcascade_frontalface_default.xml` | vision_node | Move to `models/`, use ROS parameter |
| `/home/fares/Drone/bevis.jpg` | vision_node | ROS parameter |

---

## Conventions

- All packages: `ament_cmake`, C++17 (`-Wall -Wextra -Wpedantic`)
- Executables installed to `lib/${PROJECT_NAME}` for `ros2 run`
- Message package uses `rosidl_generate_interfaces`
- Node class names: `PascalCase` extending `rclcpp::Node`
- Member variables: `trailing_underscore_`
- Timer-based publishing for periodic state (not event-driven)
- `drone_interfaces` must be built first

---

## What NOT to focus on right now

- Perfect face detection or ML-based tracking
- Obstacle avoidance
- Full 3D follow
- Low-level PID tuning
- Diagram perfection
- Backwards-compat shims or over-engineering

**Focus: get the architecture to execute end-to-end.**

---

## Diagram Tools

| Tool | Use case |
|---|---|
| draw.io / diagrams.net | Architecture diagrams (high-level + ROS) |
| Excalidraw | Quick whiteboard sketches |
| Mermaid | Diagrams embedded in GitHub README |
| rqt_graph | Live runtime ROS graph visualization |

---

## Next Steps (ordered)

1. Implement `mission_manager_node` in `drone_behavior` — state machine (IDLE/SEARCH/TRACKING/FOLLOWING/TARGET_LOST/SAFETY_HOLD)
2. Implement `follow_controller_node` in `drone_control` — needs `/mission/follow_enabled` + `/world/target_pos_relative` + `/vehicle/odom`
3. Implement `setpoint_validation_node` in `drone_control`
4. Create launch file in `drone_bringup` for the full mock chain
5. Implement `safety_supervision_node` and `hold_failsafe_node` in `drone_safety`
6. Build out `drone_perception` with full camera pipeline
7. Implement real `fcu_bridge_node` (MAVLink/MAVROS or PX4 uXRCE-DDS) when new FC arrives
8. Replace hardcoded paths with ROS parameters
9. Move haarcascade to `models/` directory
