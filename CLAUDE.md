# CLAUDE.md — Drone Project Context

## Project Summary

DIY indoor **face-following drone** using ROS 2 (Jazzy) on a Raspberry Pi 4 companion computer. The companion computer runs all autonomy logic; a separate Flight Controller handles low-level stabilization and hover. The stack is developed and tested in Docker, targeting deployment on real hardware.

**Goal:** Strong enough for GitHub, CV, interviews, and eventual report/presentation.

**Current milestone:** "Mock chain verified end-to-end — writing fcu_bridge_node with MAVROS"

---

## Hardware Context

| Sensor | Where it lives | Why |
|---|---|---|
| Pi Camera | Raspberry Pi (ROS side) | Perception input |
| Optical Flow sensor | Flight Controller | Stability/state estimation — stays on FC |
| Downward Lidar/Rangefinder | Flight Controller | Hover/altitude — stays on FC |
| Flight Controller | FC hardware | Stabilisation, hover, motor control |

**New FC has arrived (ArduPilot). The drone can hover in place.** FC connected to Raspberry Pi via USB (`/dev/ttyACM0`). Mock chain verified end-to-end. Writing `fcu_bridge_node` via MAVROS now.

**FC connection:** USB (temporary during development — UART via GPIO pins planned for final deployment to avoid vibration issues).
**FC protocol:** MAVLink 2 via MAVROS.
**ArduPilot mode needed:** GUIDED (for velocity commands from ROS).

**Raspberry Pi runs:** perception, world model, mission logic, control, safety — all high-level.
**Raspberry Pi OS:** Raspberry Pi OS 64-bit, running ROS 2 in Docker.
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
| FC comms | MAVROS (ros-jazzy-mavros + ros-jazzy-mavros-extras) |
| FC protocol | MAVLink 2 |
| Hardware (target) | Raspberry Pi 4, Pi Camera, ArduPilot FC |

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
    ├── Dockerfile                       ← ROS Jazzy base image + MAVROS
    ├── compose.yml                      ← mounts drone_ws, exposes /dev/ttyACM0
    └── drone_ws/src/
        ├── drone_interfaces/            ← custom message definitions ✓
        ├── drone_state/                 ← mock_fcu_node ✓, world_model_node ✓, mock_target_node ✓, fcu_bridge_node (in progress)
        ├── drone_vision/                ← vision_node ✓ (old perception test code removed)
        ├── drone_control/               ← follow_controller_node ✓, setpoint_validation_node ✓
        ├── drone_behavior/              ← mission_manager_node ✓
        ├── drone_bringup/               ← launch/mock_chain.launch.py ✓
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
**Package deps:** rclcpp, std_msgs, nav_msgs, geometry_msgs, drone_interfaces, mavros_msgs

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

#### `mock_target_node` ✓ DONE
Simulates `target_estimator_node` for testing without perception pipeline.

- **Publishes:** `/target/state` (TargetState) @ 10Hz
- **Behavior:** `detected=true`, `confidence=0.9`, `yaw_error=sin(t)*0.3` (oscillates), `distance_estimate=1.5m`

#### `fcu_bridge_node` 🔧 IN PROGRESS
Sole gateway between ROS stack and ArduPilot FC via MAVROS.

- **Subscribes:** `/mavros/state`, `/mavros/local_position/odom`, `/control/setpoint_validated`
- **Publishes:** `/vehicle/status`, `/vehicle/odom`, `/mavros/setpoint_velocity/cmd_vel_unstamped`
- **Service client:** `/mavros/set_mode` — keeps ArduPilot in GUIDED mode
- **Logic:**
  - Converts MAVROS State → VehicleStatus (`offboard_ready = mode == "GUIDED"`)
  - Forwards odom directly
  - Streams velocity commands @ 10Hz (ArduPilot requires continuous stream)
  - `hold=true` or `!armed` → sends zero velocity (keeps stream alive)

---

### `drone_vision` — Face Detection (to be replaced by drone_perception)
Old perception test code removed. `vision_node` kept as-is for now.

---

### `drone_control` ✓ DONE
**Package deps:** rclcpp, std_msgs, geometry_msgs, nav_msgs, drone_interfaces

#### `follow_controller_node` ✓ DONE
Converts target position into velocity setpoints.

- **Subscribes:** `/mission/follow_enabled`, `/world/target_pos_relative`, `/vehicle/odom`
- **Publishes:** `/control/setpoint_raw` (ControlSetpoint) @ 10Hz
- **Logic:**
  - `follow_enabled = false` → publish `hold = true`
  - `yaw_rate = clamp(KP_YAW × target_y_, ±1.0)` — `KP_YAW = 1.2`
  - `vx = clamp(KP_VX × target_x_, ±0.5)` — `KP_VX = 0.5`

#### `setpoint_validation_node` ✓ DONE
Validates and clamps setpoints before FCU.

- **Subscribes:** `/control/setpoint_raw`
- **Publishes:** `/control/setpoint_validated`
- **Logic:** NaN/Inf check → force hold. Clamps: `vx ±1.0`, `vy ±1.0`, `vz ±0.5`, `yaw_rate ±1.5`
- Event-driven — no timer

---

### `drone_behavior` ✓ DONE
**Package deps:** rclcpp, std_msgs, geometry_msgs, drone_interfaces

#### `mission_manager_node` ✓ DONE
State machine controlling mission phases.

- **Subscribes:** `/world/target_valid`, `/world/target_pos_relative`
- **Publishes:** `/mission/state` @ 10Hz, `/mission/follow_enabled` @ 10Hz
- **States:** `IDLE` → `TRACKING` → `FOLLOWING` ↔ `TARGET_LOST`, `SAFETY_HOLD`
- `follow_enabled = true` only in `FOLLOWING`

---

### `drone_bringup` ✓ DONE
**Launch file:** `launch/mock_chain.launch.py`
Starts: `mock_fcu_node`, `mock_target_node`, `world_model_node`, `mission_manager_node`, `follow_controller_node`, `setpoint_validation_node`

---

### `drone_perception` — EMPTY
Will contain full perception pipeline. Old test code in `drone_vision` removed.
`mock_target_node` in `drone_state` replaces `target_estimator_node` during testing.

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
**Mission states:** `IDLE` → `TRACKING` → `FOLLOWING` ↔ `TARGET_LOST`, `SAFETY_HOLD`

### Control & safety layer (drone_control / drone_safety)
```
follow_controller_node  ← /mission/follow_enabled
                        ← /world/target_pos_relative
                        ← /vehicle/odom
                        → /control/setpoint_raw

setpoint_validation_node  ← /control/setpoint_raw
                          → /control/setpoint_validated

safety_supervision_node  ← /vehicle/status
                         ← /control/setpoint_validated
                         ← /mission/state
                         → /control/hold_cmd

hold_failsafe_node  ← /control/hold_cmd
                   → /control/setpoint_safe

fcu_bridge_node  ← /control/setpoint_safe
                 ↔ Flight Controller (hardware)  ← SOLE gateway
```

### Mock chain (testing only)
```
mock_target_node → /target/state        ← replaces full perception pipeline
mock_fcu_node  ← /control/setpoint_validated
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

---

## Implementation Phases

| Phase | Focus | Contents |
|---|---|---|
| 1 | ✓ Packages + interfaces + skeleton | drone_interfaces (4 msgs), package shells |
| 2 | ✓ Mock chain | mock_fcu ✓, world_model ✓, mission_manager ✓, follow_controller ✓, setpoint_validation ✓, mock_target ✓ |
| 3 | ✓ Launch + FC integration | mock_chain.launch.py ✓, fcu_bridge_node (in progress) |
| 4 | Perception | camera_driver, image_preprocess, face_detector, face_tracker, target_estimator |
| 5 | Safety | safety_supervision_node, hold_failsafe_node |

---

## Development Status

| Component | Package | Status |
|---|---|---|
| 4 custom messages | drone_interfaces | ✓ Done |
| `mock_fcu_node` | drone_state | ✓ Done |
| `vision_node` | drone_vision | ✓ Done |
| `world_model_node` | drone_state | ✓ Done |
| `mock_target_node` | drone_state | ✓ Done |
| `mission_manager_node` | drone_behavior | ✓ Done |
| `follow_controller_node` | drone_control | ✓ Done |
| `setpoint_validation_node` | drone_control | ✓ Done |
| Launch file (mock chain) | drone_bringup | ✓ Done |
| `fcu_bridge_node` | drone_state | 🔧 In progress |
| `safety_supervision_node` | drone_safety | ✗ Not started |
| `hold_failsafe_node` | drone_safety | ✗ Not started |
| Full perception pipeline | drone_perception | ✗ Not started |

---

## Docker Setup

**Dockerfile:** `software/Dockerfile`
- Base: `ros:jazzy-ros-base`
- Installs: build-essential, cmake, git, colcon, rosdep, vcstool, ros-jazzy-mavros, ros-jazzy-mavros-extras
- Runs `install_geographiclib_datasets.sh` (required for MAVROS)
- Sources `/opt/ros/jazzy/setup.bash` in `.bashrc`
- WORKDIR: `/workspaces/drone_ws`

**compose.yml:** `software/compose.yml`
- Service: `drone_ros2`
- Mounts `./drone_ws` → `/workspaces/drone_ws`
- TTY + stdin_open for interactive use
- `privileged: true` — required for hardware access
- `devices: /dev/ttyACM0` — FC connected via USB

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

1. ✓ Implement `mission_manager_node` in `drone_behavior`
2. ✓ Implement `follow_controller_node` in `drone_control`
3. ✓ Implement `setpoint_validation_node` in `drone_control`
4. ✓ Implement `mock_target_node` in `drone_state`
5. ✓ Create launch file in `drone_bringup`
6. Implement `fcu_bridge_node` using ArduPilot + MAVROS ← in progress
7. Implement `safety_supervision_node` and `hold_failsafe_node` in `drone_safety`
8. Build out `drone_perception` with full camera pipeline
9. Replace hardcoded paths with ROS parameters
10. Move haarcascade to `models/` directory
