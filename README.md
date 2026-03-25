# DIY Indoor Face-Follow Drone

## Goal
Indoor companion-computer autonomy stack using ROS 2 on Raspberry Pi 4.

## Architecture
- Flight controller: stabilization, hover, low-level control
- ROS 2 companion computer: perception, mission logic, safety, high-level setpoints

## Packages
- drone_interfaces
- drone_perception
- drone_state
- drone_behavior
- drone_control
- drone_safety
- drone_bringup

## Current milestone
Target in -> yaw command out via mock FCU