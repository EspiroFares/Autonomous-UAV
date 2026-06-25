<div align="center">

# Autonomous Indoor Person-Following Drone

**A sim-to-real ROS 2 autonomy stack — from a Gazebo digital twin to a real ArduPilot quadcopter.**

Perception, world modeling, mission logic, control and safety run on a Raspberry Pi companion computer.
A separate flight controller owns stabilization and hover. Everything is developed and validated in
simulation first, then deployed to hardware.

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](https://isocpp.org/)
[![Python](https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-Copter-792EE5?logo=dronedeploy&logoColor=white)](https://ardupilot.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-FB8C00?logo=gazebo&logoColor=white)](https://gazebosim.org/)
[![MAVROS](https://img.shields.io/badge/MAVLink-MAVROS-FF6F00)](https://github.com/mavlink/mavros)
[![OpenCV](https://img.shields.io/badge/OpenCV-4-5C3EE8?logo=opencv&logoColor=white)](https://opencv.org/)
[![MediaPipe](https://img.shields.io/badge/MediaPipe-Pose-00B0FF?logo=google&logoColor=white)](https://developers.google.com/mediapipe)
[![Docker](https://img.shields.io/badge/Docker-ready-2496ED?logo=docker&logoColor=white)](https://www.docker.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

</div>

<!-- Hero demo video. Swap this attachment URL for your strongest demo clip (the loiter / disturbance-rejection clip works great here). A bare github.com/user-attachments/... URL on its own line renders as an inline video player on GitHub. -->

https://github.com/user-attachments/assets/7d0be21c-c6fa-4e51-9add-62b9557a809c

---

## Highlights

- **Full sim-to-real workflow.** A complete digital twin — **Gazebo Harmonic + ArduPilot SITL + MAVROS** — runs the *exact same ROS 2 stack* that targets the real aircraft. Behavior is validated in simulation before it ever touches hardware.
- **GPS-denied stable hover on real hardware.** Active disturbance rejection by fusing a **TF-Luna LiDAR** (altitude) and an **optical-flow sensor** (lateral) into the flight controller's estimator. Push the drone — it returns to position on its own. *(This milestone demo reached 120k+ impressions on LinkedIn.)*
- **Real computer-vision perception.** Monocular person detection with **MediaPipe Pose**, exponential-moving-average tracking, and **pinhole-geometry distance estimation** — all in a streaming ROS 2 pipeline.
- **Architecture built like production robotics.** A layered stack with a strict hardware boundary: a **single bridge node** is the only path to the flight controller, and **safety holds a veto** over mission logic by design.
- **Solo-built, end to end** — physics, flight dynamics, perception, control, and the full ROS 2 graph.

---

## Table of Contents

- [What it does](#what-it-does)
- [System architecture](#system-architecture)
- [Perception pipeline](#perception-pipeline)
- [Simulation stack (the digital twin)](#simulation-stack-the-digital-twin)
- [Engineering deep-dives](#engineering-deep-dives)
- [Technology stack](#technology-stack)
- [Project status](#project-status)
- [Repository structure](#repository-structure)
- [Getting started](#getting-started)
- [Roadmap](#roadmap)
- [Author](#author)

---

## What it does

The drone autonomously **detects a person, locks on, and follows them indoors** — where GPS is unavailable.
The companion computer turns a camera feed into a target estimate, decides what to do via a mission state
machine, and produces high-level velocity commands. The flight controller turns those into stable, hovered
flight.

The whole point is the **sim-to-real split**: the same nodes run against a simulated quadcopter in Gazebo
and against the real one, so the simulator is a true development and regression environment — not a toy.

---

## System architecture

The stack is organized in clean layers. Data flows up from sensors into a world model, through mission and
control logic, and back down to the flight controller — crossing the hardware boundary at exactly one place.

### System overview
![System Overview](docs/architecture/System_Overview.drawio.png)

### ROS 2 node / topic graph
![ROS 2 Architecture](docs/architecture/ROS_architecture.drawio-3.png)

This is what makes the sim-to-real claim real: a bug caught in SITL is a bug that would have happened on the
drone.

> **Demo — full autonomy stack in Gazebo + ArduPilot SITL.** The complete perception → mission → follow
> loop running against the simulated quadcopter. Work in progress; the on-hardware port is the next milestone.


https://github.com/user-attachments/assets/931f9e90-d432-4f46-a7b8-8be1f02cf99d




---

### Design principles (fixed from day one)

| # | Principle | Why it matters |
|---|-----------|----------------|
| 1 | **`fcu_bridge_node` is the sole gateway to the flight controller** | No other node talks to the FC. One bridge keeps the hardware boundary clean and the rest of the stack fully testable in simulation. |
| 2 | **Safety has veto** | The safety layer can block any setpoint and force a hold/failsafe at any time, overriding mission logic. |
| 3 | **ROS sends high-level commands only** | ROS outputs `vx, vy, vz, yaw_rate`. The FC keeps its inner stabilization/hover loops — ROS never reimplements them. |
| 4 | **Stability-critical sensors stay on the FC** | Optical flow and the downward rangefinder belong to the low-level flight stack and are not moved onto the Pi. |

---

## Perception pipeline

A streaming ROS 2 pipeline turns raw frames into a metric target estimate:

```
camera  ─►  image_preprocessing  ─►  person_detector   ─►  person_tracker  ─►  target_estimator  ─►  /target/state
(Gazebo /         (resize,            (MediaPipe Pose,       (EMA smoothing)    (pinhole geometry:
 ros_gz_bridge     normalize)          shoulder landmarks)                       distance + yaw error)
 or Pi camera)
```

- **`person_detector_node`** (Python, MediaPipe Pose) — locates a person from shoulder landmarks and emits a bounding box + shoulder width in pixels.
- **`person_tracker_node`** (C++) — exponential-moving-average smoothing to suppress detection jitter.
- **`target_estimator_node`** (C++) — converts pixel measurements to a **distance estimate** via a calibrated pinhole camera model, plus a normalized yaw error for steering.

Downstream, `world_model_node` fuses perception with vehicle state, `mission_manager_node` runs the
behavior state machine (`IDLE → TRACKING → FOLLOWING ↔ TARGET_LOST / SAFETY_HOLD`), and
`follow_controller_node` produces clamped, validated velocity setpoints.

---

## Simulation stack (the digital twin)

```
Gazebo Harmonic (physics + simulated camera)
   ↕  ArduPilot SITL (real flight-control firmware, simulated dynamics)
   ↕  MAVROS  (MAVLink ⇄ ROS 2)
   ↕  fcu_bridge_node ─► the full ROS 2 autonomy stack
```

The simulator runs **the real ArduPilot firmware** (Software-In-The-Loop), not an approximation — so flight
modes, arming logic, and the MAVLink control contract behave exactly as on hardware. Gazebo provides a
camera feed bridged into the perception pipeline via `ros_gz_bridge`, and an **animated walking actor** acts
as the follow target. One launch file (`sim_chain.launch.py`) brings up the entire autonomy graph against the
simulated aircraft.

This is what makes the sim-to-real claim real: a bug caught in SITL is a bug that would have happened on the
drone.

---

## Engineering deep-dives

A few of the harder problems solved along the way — the kind of thing that doesn't show up in a feature list.

<details>
<summary><b>GPS-denied stable hover: a feedback loop hiding in the EKF logs</b></summary>

<br>

Indoor flight has no GPS, so position must come from a downward **TF-Luna LiDAR** (Z) and an
**optical-flow sensor** (X/Y) fused into ArduPilot's EKF3 estimator. Early hover was unstable and oscillated.
Digging into the **EKF3 dataflash logs** revealed the optical-flow and gyro contributions were effectively
**180° out of phase** — the correction was reinforcing the disturbance instead of cancelling it, a positive
feedback loop. Fixing the orientation/sign of the fused signal turned the oscillation into crisp **active
disturbance rejection**: nudge the drone and it drives itself back to where it started.

</details>

<details>
<summary><b>Monocular distance estimation — and a 3× calibration bug that drove the drone into its target</b></summary>

<br>

Range to the target is estimated from a **pinhole camera model** using the person's shoulder width in pixels.
During SITL testing the drone kept accelerating straight through the target instead of holding distance.
The cause: the camera's **focal length was hardcoded for the wrong field of view**, so every distance estimate
came out **~3× too large** — the controller always believed the target was farther than the stop distance and
never slowed down. Found it by recording the pipeline with `ros2 bag` and cross-referencing the ArduPilot
crash logs, then recomputed the focal length from the camera's actual horizontal FOV. A reminder that a
perception bug shows up as a *control* failure.

</details>

<details>
<summary><b>GUIDED-mode takeoff vs. a continuous setpoint stream</b></summary>

<br>

ArduPilot's GUIDED mode requires a continuous velocity-setpoint stream as a keepalive. But that same stream
(`vz = 0`) silently **overrides an in-progress `NAV_TAKEOFF`** — the vehicle would arm, accept the takeoff,
never actually climb, and then auto-disarm on the ground-idle safety timer (a plain disarm, *not* a crash).
Isolating this required separating ArduPilot's own behavior from the ROS stack's, and it surfaced a real
ordering constraint in the offboard-control contract that the bridge node has to respect.

</details>

---

## Technology stack

| Layer | Technology |
|-------|-----------|
| **Framework** | ROS 2 Jazzy |
| **Languages** | C++17 (`rclcpp`), Python 3 |
| **Perception** | OpenCV 4, MediaPipe Pose, `cv_bridge` |
| **Flight control** | ArduPilot (Copter), MAVLink 2 via MAVROS |
| **Simulation** | Gazebo Harmonic, ArduPilot SITL, `ros_gz_bridge` |
| **Build / tooling** | CMake, `ament_cmake`, colcon, Docker |
| **Target hardware** | Raspberry Pi 4 · ArduPilot FC · Pi Camera · TF-Luna LiDAR · optical-flow sensor |

---

## Project status

Honest split between what runs in simulation and what's proven on the real aircraft.

| Capability | Simulation (Gazebo + SITL) | Real hardware |
|-----------|:--------------------------:|:-------------:|
| Stable hover / position hold (GPS-denied) | ✅ | ✅ active disturbance rejection |
| Full perception pipeline (MediaPipe Pose → distance) | ✅ end-to-end | 🔜 porting to Pi companion |
| Mission state machine + follow control | ✅ end-to-end | 🔜 |
| `fcu_bridge_node` (MAVROS gateway) | ✅ functional | 🔜 |
| Safety supervisor / failsafe nodes | 🔧 next build target | — |

**Component breakdown**

- **Done:** `drone_interfaces` (custom messages) · mock chain (`mock_fcu`, `mock_target`, `world_model`, `mission_manager`, `follow_controller`, `setpoint_validation`) · full perception pipeline (`image_preprocessing`, `person_detector`, `person_tracker`, `target_estimator`) · `fcu_bridge_node` · full Gazebo + SITL + MAVROS simulation (`sim_chain.launch.py`)
- **In progress:** on-hardware autonomy port · Raspberry Pi companion integration
- **Next:** `safety_supervision_node`, `hold_failsafe_node` (the `drone_safety` package)

---

## Repository structure

<details>
<summary>Expand tree</summary>

<br>

```text
Autonomous-UAV/
├── docs/architecture/        # System + ROS graph diagrams (draw.io + PNG)
└── software/
    ├── Dockerfile            # ROS 2 Jazzy + MAVROS environment
    ├── compose.yml
    └── drone_ws/src/
        ├── drone_interfaces/ # Custom .msg definitions (built first)
        ├── drone_perception/ # Camera → MediaPipe Pose → tracker → distance estimate
        ├── drone_state/      # fcu_bridge, world_model, mock FC/target
        ├── drone_behavior/   # Mission state machine
        ├── drone_control/    # Follow controller + setpoint validation
        ├── drone_bringup/    # Launch files (sim / mock / fake-video / real)
        ├── drone_safety/     # Safety supervisor + failsafe (planned)
        └── drone_vision/     # Legacy single-node vision (superseded by drone_perception)
```

</details>

---

## Getting started

The full simulation runs natively on Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic installed.

<details>
<summary>Build the workspace</summary>

<br>

```bash
cd software/drone_ws
colcon build            # drone_interfaces builds first automatically
source install/setup.bash
```

</details>

<details>
<summary>Run the full Gazebo + ArduPilot SITL simulation</summary>

<br>

Prerequisites: ArduPilot SITL (`sim_vehicle.py`), the `ardupilot_gazebo` plugin, and Gazebo Harmonic.

1. **Gazebo** (server + GUI) with the warehouse world.
2. **ArduPilot SITL** — `sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --out tcpin:0.0.0.0:5770`
3. **MAVROS** — `ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5770`
4. **Camera bridge** — `ros2 run ros_gz_bridge parameter_bridge ...camera...image`
5. **Autonomy stack** — `ros2 launch drone_bringup sim_chain.launch.py`

The mock chain needs none of the above and is the fastest way to see the architecture run:

```bash
ros2 launch drone_bringup mock_chain.launch.py
```

</details>

---

## Roadmap

- [x] End-to-end mock chain
- [x] Full perception pipeline (MediaPipe Pose)
- [x] Gazebo + ArduPilot SITL digital twin with `fcu_bridge_node`
- [x] GPS-denied stable hover with disturbance rejection on real hardware
- [ ] Safety supervisor + failsafe (`drone_safety`)
- [ ] Raspberry Pi 4 companion-computer integration
- [ ] Autonomous person-following on the real aircraft

---

## Author

**Fares Espiro** — M.Sc. student in Autonomous Systems @ Linköping University.
Robotics · Computer Vision · Embedded AI. Building robots end-to-end, from physics simulation to sim-to-real deployment.

[![LinkedIn](https://img.shields.io/badge/LinkedIn-espiro--fares-0A66C2?logo=linkedin&logoColor=white)](https://linkedin.com/in/espiro-fares)
[![GitHub](https://img.shields.io/badge/GitHub-EspiroFares-181717?logo=github&logoColor=white)](https://github.com/EspiroFares)
[![Email](https://img.shields.io/badge/Email-faresespiro535%40gmail.com-EA4335?logo=gmail&logoColor=white)](mailto:faresespiro535@gmail.com)

</div>
