#!/usr/bin/env bash
SESSION=drone
ENV='export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; source /opt/ros/jazzy/setup.bash; source ~/Autonomous-UAV/software/drone_ws/install/setup.bash'

run_node() {  # $1 = pane-target, $2 = command
    tmux send-keys -t "$1" "$ENV; while true; do $2; echo '*** NODE EXITED - restarting in 2s ***'; sleep 2; done" C-m
}

tmux kill-session -t $SESSION 2>/dev/null
pkill -9 -f mavros; pkill -9 -f camera_driver; pkill -9 -f person_detector; pkill -9 -f _node
sleep 1

# ── Window 0: MAVROS ──────────────────────────────────────────
tmux new-session -d -s $SESSION -n mavros
tmux send-keys -t $SESSION:mavros "$ENV; ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyAMA5:921600 2>&1 | grep --line-buffered -vE 'aiding|EKF3 IMU'" C-m
# ── Window 1: perception (5) ──────────────────────────
tmux new-window -t $SESSION -n perception
tmux split-window -t $SESSION:perception -h
tmux split-window -t $SESSION:perception.0 -v
tmux split-window -t $SESSION:perception.2 -v
tmux split-window -t $SESSION:perception.3 -v
run_node "$SESSION:perception.0" "ros2 run drone_perception camera_driver_node.py"
# person_detector runs in its own window at the very end (started last) so it
# doesn't lose the CycloneDDS discovery race against the camera on a cold start.
tmux send-keys -t "$SESSION:perception.2" "$ENV; echo 'person_detector runs in the [detector] window - started last'" C-m
run_node "$SESSION:perception.3" "ros2 run drone_perception person_tracker_node"
run_node "$SESSION:perception.4" \
  "ros2 run drone_perception target_estimator_node --ros-args --params-file \$(ros2 pkg prefix drone_bringup)/share/drone_bringup/config/real_perception.yaml"

# ── Window 2: control (5 ) ─────────────────────────────
tmux new-window -t $SESSION -n control
tmux split-window -t $SESSION:control -h
tmux split-window -t $SESSION:control.0 -v
tmux split-window -t $SESSION:control.2 -v
tmux split-window -t $SESSION:control.3 -v
run_node "$SESSION:control.0" "ros2 run drone_state fcu_bridge_node"
run_node "$SESSION:control.1" "ros2 run drone_state world_model_node"
run_node "$SESSION:control.2" "ros2 run drone_behavior mission_manager_node"
run_node "$SESSION:control.3" "ros2 run drone_control follow_controller_node"
run_node "$SESSION:control.4" "ros2 run drone_control setpoint_validation_node"

# ── Window 3: monitor (stream setup + live data + free shell) ─
tmux new-window -t $SESSION -n monitor
tmux split-window -t $SESSION:monitor -h
tmux split-window -t $SESSION:monitor.0 -v

tmux send-keys -t $SESSION:monitor.0 "$ENV; echo 'Waiting 15s for MAVROS...'; sleep 15; \
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval '{message_id: 132, message_rate: 20.0}'; \
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval '{message_id: 173, message_rate: 20.0}'; \
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval '{message_id: 74, message_rate: 10.0}'; \
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval '{message_id: 147, message_rate: 2.0}'; \
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval '{message_id: 30, message_rate: 10.0}'; \
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval '{message_id: 65, message_rate: 5.0}';    \
ros2 topic echo /vehicle/height" C-m

tmux send-keys -t $SESSION:monitor.1 "$ENV; sleep 18; ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped" C-m

tmux send-keys -t $SESSION:monitor.2 "$ENV; echo 'Free shell - e.g. ros2 param set /follow_controller_node kff 0.7'" C-m

# Pane 3: automatic flight logging (rosbag)
tmux split-window -t $SESSION:monitor.2 -v
tmux send-keys -t $SESSION:monitor.3 "$ENV; mkdir -p ~/flight_logs; sleep 20; \
ros2 bag record -o ~/flight_logs/flight_\$(date +%Y%m%d_%H%M%S) \
/target/detections /target/track /target/state /world/target_valid \
/world/target_pos_relative /mission/state /mission/follow_enabled \
/control/setpoint_raw /control/setpoint_validated \
/mavros/rc/in \
/mavros/setpoint_velocity/cmd_vel_unstamped /vehicle/height /vehicle/status /mavros/state \
/mavros/vfr_hud /mavros/battery /mavros/imu/data" C-m

# ── Window 4: detector (started last, after the camera is up) ───
# Avoids the CycloneDDS discovery race: it subscribes to /camera/image_raw only
# once the camera is established. Sleep 20 once, then a restart loop so the
# watchdog can respawn it without waiting 20s again.
tmux new-window -t $SESSION -n detector
tmux send-keys -t $SESSION:detector "$ENV; sleep 20; while true; do ros2 run drone_perception person_detector_node.py; echo '*** NODE EXITED - restarting in 2s ***'; sleep 2; done" C-m

tmux select-window -t $SESSION:monitor
tmux attach -t $SESSION
