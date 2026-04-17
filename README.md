# Indoor Face-Following Drone

ROS 2-based autonomy stack for a DIY indoor drone built around a Raspberry Pi 4 companion computer and a separate ArduPilot flight controller.

The idea is to keep the split clean:

- the **Raspberry Pi** runs perception, world modeling, mission logic, control, and safety
- the **flight controller** handles stabilization, hover, and low-level motor control

## Current status

At the moment:

- the **mock chain is working end-to-end**
- the **ArduPilot FC is installed**
- the drone can **hover in place**
- the FC is connected to the Raspberry Pi over **USB** (`/dev/ttyACM0`) (for now)
- I am currently writing `fcu_bridge_node` using **MAVROS** and **MAVLink 2**

Current milestone:

**Mock chain verified end-to-end — writing `fcu_bridge_node` with MAVROS**

The goal right now is not fancy perception or polished flight behavior. The goal is to get the full architecture working cleanly from ROS all the way to the flight controller.

---

## Architecture

### System overview

![System Overview](docs/architecture/System_Overview.drawio.png)

### ROS node / topic architecture

![ROS 2 Architecture](docs/architecture/ROS_architecture.drawio-3.png)

---

## Design rules

A few design decisions are fixed from the start.

### 1. `fcu_bridge_node` is the only gateway to the FC

No ROS node should talk directly to the flight controller except `fcu_bridge_node`.

That means no direct FC connection from:

- `follow_controller_node`
- `setpoint_validation_node`
- `safety_supervision_node`

Everything goes through one bridge node. That keeps the hardware boundary clear and makes the rest of the stack easier to test.

### 2. Safety can override everything else

Mission logic does not get the final word.  
If safety decides the drone should hold or stop, it should be able to block commands immediately.

### 3. ROS sends high-level commands only

The ROS side outputs commands like:

- `vx`
- `vy`
- `vz`
- `yaw_rate`

The FC still owns the inner loops, stabilization, hover, and motor control.

### 4. Stability-critical sensors stay on the FC

Optical flow and the downward rangefinder stay on the flight controller. They are part of the low-level flight stack and are not being moved onto the Pi.

---

## What is implemented

### Done

- `drone_interfaces`
- `mock_fcu_node`
- `world_model_node`
- `mock_target_node`
- `mission_manager_node`
- `follow_controller_node`
- `setpoint_validation_node`
- `mock_chain.launch.py`
- Docker-based ROS 2 Jazzy environment
- MAVROS installed in the container
- FC connected to Pi and hovering in real hardware

### In progress

- `fcu_bridge_node` via MAVROS

### Not started yet

- `safety_supervision_node`
- `hold_failsafe_node`
- full perception pipeline in `drone_perception`

---

## Repository layout

```text
Drone/
├── README.md
├── CLAUDE.md
├── haarcascade_frontalface_default.xml
├── docs/architecture/
│   ├── ROS_architecture-3.drawio
│   ├── ROS_architecture.drawio-3.png
│   ├── System_Overview-2.drawio
│   └── System_Overview.drawio.png
├── hardware/
└── software/
    ├── Dockerfile
    ├── compose.yml
    └── drone_ws/src/
        ├── drone_interfaces/
        ├── drone_state/
        ├── drone_vision/
        ├── drone_control/
        ├── drone_behavior/
        ├── drone_bringup/
        ├── drone_perception/
        ├── drone_safety/
        ├── drone_sim/
        ├── drone_description/
        └── drone_test/
