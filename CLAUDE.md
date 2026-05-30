# CLAUDE.md — Drone Project Context

## Project Summary

DIY indoor **face-following drone** using ROS 2 (Jazzy) on a Raspberry Pi 4 companion computer. The companion computer runs all autonomy logic; a separate Flight Controller handles low-level stabilization and hover. The stack is developed and tested in Docker, targeting deployment on real hardware.

**Goal:** Strong enough for GitHub, CV, interviews, and eventual report/presentation.

**Current milestone:** "Moved to Linux — running full Gazebo simulation with simulated camera feeding into real perception pipeline (MediaPipe) via ros_gz_bridge"

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
| Vision | OpenCV 4 + MediaPipe Pose — body/person detection via shoulder landmarks |
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
        ├── drone_bringup/               ← mock_chain.launch.py ✓, fake_video_chain.launch.py ✓, real_chain.launch.py ✓
        ├── drone_perception/            ← IN PROGRESS — camera_driver ✓, fake_camera_driver ✓, person_detector ✓, person_tracker ✓, target_estimator ✓, image_preprocessing ✓
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
| `TargetState.msg` | Detected person info | `detected`, `confidence`, `yaw_error`, `distance_estimate` |
| `MissionState.msg` | Mission phase | `state`, `follow_enabled`, `target_valid` |
| `Detection.msg` | Raw detector output | `header`, `detected`, `confidence`, `bbox_center_x/y`, `bbox_width/height`, `shoulder_width_px` |
| `Track.msg` | Smoothed tracker output | `header`, `valid`, `track_id`, `center_x/y`, `width`, `height`, `velocity_x/y` |

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
  - `yaw_rate = clamp(KP_YAW × -target_y_, ±1.0)` — `KP_YAW = 1.2`
  - `vx = clamp(KP_VX × (target_x_ - DESIRED_DISTANCE), ±0.5)` — `KP_VX = 0.5`, `DESIRED_DISTANCE = 0.5m`

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
Three launch files:

| Launch file | What it starts | When to use |
|---|---|---|
| `mock_chain.launch.py` | mock_fcu + mock_target + world_model + mission + control | Fully mocked — no hardware or camera needed |
| `fake_video_chain.launch.py` | fake_camera_driver + full perception pipeline + fcu_bridge + world_model + mission + control | Real perception with fake camera (webcam streamed from Mac), real or SITL FC |
| `real_chain.launch.py` | fcu_bridge + mock_target + world_model + mission + control | Real FC, mocked perception |

---

### `drone_perception` — 🔧 IN PROGRESS
Full perception pipeline implemented. Uses **MediaPipe Pose** (not Haar Cascade) — detects persons by shoulder landmarks.

**Nodes (all in package `drone_perception`):**

#### `camera_driver_node` ✓ (C++)
Reads Pi Camera via OpenCV (`/dev/video0`) @ 30fps.
- **Publishes:** `/camera/image_raw` (sensor_msgs/Image)

#### `fake_camera_driver_node` ✓ (Python) — MAC ONLY WORKAROUND
For development on Mac without Pi Camera or Gazebo. Connects via TCP socket to `host.docker.internal:8485` and streams webcam frames from the Mac into Docker.
- **Publishes:** `/camera/image_raw` (sensor_msgs/Image) @ ~30fps
- **NOT used on Linux** — on Linux, Gazebo provides the camera via `ros_gz_bridge`

#### `image_preprocessing_node` ✓ (C++)
Preprocesses raw camera frames before detection.
- **Subscribes:** `/camera/image_raw`
- **Publishes:** `/camera/image_preprocessed`

#### `person_detector_node` ✓ (Python — MediaPipe)
Detects person using MediaPipe Pose. Uses left/right shoulder landmarks to compute bounding box center and shoulder width.
- **Subscribes:** `/camera/image_preprocessed`
- **Publishes:** `/target/detections` (Detection)
- **Logic:** shoulder midpoint → `bbox_center_x/y`; shoulder pixel width → `shoulder_width_px`; confidence fixed at 0.9 when detected

#### `person_tracker_node` ✓ (C++)
EMA (exponential moving average) smoothing of detections. Alpha = 0.3.
- **Subscribes:** `/target/detections` (Detection)
- **Publishes:** `/target/track` (Track)

#### `target_estimator_node` ✓ (C++)
Converts track to `TargetState` using pinhole camera geometry.
- **Subscribes:** `/target/track` (Track)
- **Publishes:** `/target/state` (TargetState)
- **Logic:**
  - `distance = (known_shoulder_width × focal_length) / shoulder_width_px` — known_shoulder=0.45m, focal=600px, image_width=640px
  - `yaw_error = (center_x - 0.5) × 2.0` — normalized [-1, 1]

### `drone_safety` — EMPTY
Will contain: `safety_supervision_node`, `hold_failsafe_node`

---

## Full Node & Topic Graph

### Perception pipeline (drone_perception)
```
Pi Camera (hardware) OR fake_camera_driver_node (webcam via TCP from Mac)
  → camera_driver_node / fake_camera_driver_node  → /camera/image_raw
  → image_preprocessing_node                       → /camera/image_preprocessed
  → person_detector_node (MediaPipe Pose)          → /target/detections  (Detection)
  → person_tracker_node (EMA smoother, C++)        → /target/track       (Track)
  → target_estimator_node (pinhole geometry, C++)  → /target/state       (TargetState)
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
| 1 | ✓ Packages + interfaces + skeleton | drone_interfaces (6 msgs), package shells |
| 2 | ✓ Mock chain | mock_fcu ✓, world_model ✓, mission_manager ✓, follow_controller ✓, setpoint_validation ✓, mock_target ✓ |
| 3 | ✓ Launch + FC integration | mock_chain.launch.py ✓, real_chain.launch.py ✓, fcu_bridge_node (in progress) |
| 4 | 🔧 Perception | fake_camera_driver ✓, camera_driver ✓, image_preprocess ✓, person_detector ✓, person_tracker ✓, target_estimator ✓, fake_video_chain.launch.py ✓ |
| 5 | Safety | safety_supervision_node, hold_failsafe_node |

---

## Development Status

| Component | Package | Status |
|---|---|---|
| 6 custom messages | drone_interfaces | ✓ Done |
| `mock_fcu_node` | drone_state | ✓ Done |
| `vision_node` | drone_vision | ✓ Done (legacy) |
| `world_model_node` | drone_state | ✓ Done |
| `mock_target_node` | drone_state | ✓ Done |
| `mission_manager_node` | drone_behavior | ✓ Done |
| `follow_controller_node` | drone_control | ✓ Done |
| `setpoint_validation_node` | drone_control | ✓ Done |
| Launch files (mock/fake/real chain) | drone_bringup | ✓ Done |
| `camera_driver_node` | drone_perception | ✓ Done |
| `fake_camera_driver_node` | drone_perception | ✓ Done |
| `image_preprocessing_node` | drone_perception | ✓ Done |
| `person_detector_node` (MediaPipe) | drone_perception | ✓ Done |
| `person_tracker_node` (EMA) | drone_perception | ✓ Done |
| `target_estimator_node` (pinhole) | drone_perception | ✓ Done |
| `fcu_bridge_node` | drone_state | 🔧 In progress |
| `safety_supervision_node` | drone_safety | ✗ Not started |
| `hold_failsafe_node` | drone_safety | ✗ Not started |

---

## Gazebo + ArduPilot SITL Setup

Full simulation stack: **Gazebo Harmonic** (physics + visuals) + **ArduPilot SITL** (flight controller) + **MAVROS** (ROS ↔ MAVLink) + **ROS stack**.

**Platform: Linux** — the whole reason for switching from Mac. On Linux, Gazebo runs natively alongside Docker/ROS without TCP webcam hacks.

**Prerequisites (installed on Linux host, outside Docker):**
- `~/ardupilot/` — ArduPilot source with SITL (`sim_vehicle.py`)
- `~/ardupilot_gazebo/` — ArduPilot Gazebo plugin (built to `~/ardupilot_gazebo/build/`)
- Gazebo Harmonic (`gz sim`)

**Architecture:**
```
Gazebo (physics + simulated camera)
  ↔ ArduPilot SITL (flight dynamics)
  ↔ TCP:5770 ↔ MAVROS (in Docker) ↔ fcu_bridge_node → /vehicle/status, /vehicle/odom

Gazebo camera plugin
  → ros_gz_bridge → /camera/image_raw
  → image_preprocessing_node → person_detector_node → person_tracker_node → target_estimator_node
  → /target/state → world_model_node → mission_manager → follow_controller → fcu_bridge_node → SITL
```
**Key point:** Gazebo provides the camera — NO fake TCP webcam stream needed. `fake_camera_driver_node` is a Mac-only workaround and NOT used on Linux.

### Startup sequence (6 terminals — use tmux)

**Step 1 — Kill leftover processes** (always run first):
```bash
kill $(lsof -t -i :5770) 2>/dev/null; kill $(lsof -t -i :5760) 2>/dev/null; kill $(lsof -t -i :9002) 2>/dev/null; killall arducopter 2>/dev/null
```

**Step 2 — Start Gazebo server** (headless, no GUI):
```bash
unset GZ_SIM_SYSTEM_PLUGIN_PATH && unset GZ_SIM_RESOURCE_PATH && \
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build && \
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds && \
export GZ_PARTITION=drone_sim && \
gz sim -v4 -s -r ~/ardupilot_gazebo/worlds/iris_warehouse.sdf
```

**Step 3 — Start Gazebo GUI** (separate terminal, same env vars):
```bash
unset GZ_SIM_SYSTEM_PLUGIN_PATH && unset GZ_SIM_RESOURCE_PATH && \
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build && \
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds && \
export GZ_PARTITION=drone_sim && \
gz sim -v4 -g
```

**Step 4 — Start ArduPilot SITL** (connects to Gazebo via JSON):
```bash
cd ~/ardupilot && sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console \
  --out tcpin:0.0.0.0:5770
```
> SITL listens on TCP port 5770 — Docker will connect to this.

**Step 5 — Start Docker + MAVROS** (on Linux, use `172.17.0.1` or `host.docker.internal` depending on distro):
```bash
cd /Users/fares/Desktop/Drone/Drone/software && docker compose up -d && \
docker exec -it drone_ros2 bash -c \
  'cd /workspaces/drone_ws && source install/setup.bash && \
   ros2 launch mavros apm.launch fcu_url:=tcp://host.docker.internal:5770'
```

**Step 5b — Start ros_gz_bridge** (bridges Gazebo camera into ROS `/camera/image_raw`):
```bash
# Inside Docker or on host (TBD depending on setup)
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image[gz.msgs.Image
```
> This is the bridge that makes Gazebo camera feed available to the perception pipeline. Topic name may differ depending on Gazebo world/camera plugin config.

**Step 6 — Start ROS stack** (new terminal, inside Docker):
```bash
docker exec -it drone_ros2 bash
# Inside container:
cd /workspaces/drone_ws && source install/setup.bash && ros2 launch drone_bringup real_chain.launch.py
```

### Port mapping
| Port | Protocol | What |
|---|---|---|
| 5770 | TCP | SITL ↔ MAVROS (primary link) |
| 5760 | TCP | SITL default (not used in this setup) |
| 9002 | UDP | Gazebo ↔ SITL (JSON plugin) |

> Docker connects **out** to `host.docker.internal:5770` — no inbound port mapping needed in compose.yml for this.

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
5. ✓ Create launch files in `drone_bringup` (mock, fake_video, real)
6. ✓ Build out `drone_perception` with full camera pipeline (MediaPipe Pose)
7. Finish `fcu_bridge_node` using ArduPilot + MAVROS ← next
8. Test `fake_video_chain.launch.py` end-to-end (fake camera → perception → control)
9. Implement `safety_supervision_node` and `hold_failsafe_node` in `drone_safety`
10. Replace hardcoded paths with ROS parameters
11. Move haarcascade to `models/` directory (low priority)
