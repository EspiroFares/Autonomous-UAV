from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Perception — no camera_driver_node here.
        # In Gazebo sim, ros_gz_bridge publishes /camera/image_raw directly.
        Node(package="drone_perception", executable="image_preprocessing_node",
             name="image_preprocessing_node", output="screen"),
        Node(package="drone_perception", executable="person_detector_node.py",
             name="person_detector_node", output="screen"),
        Node(package="drone_perception", executable="person_tracker_node",
             name="person_tracker_node", output="screen",
             parameters=[{"max_missed_frames": 7}]),
        Node(package="drone_perception", executable="target_estimator_node",
             name="target_estimator_node", output="screen",
             # Sim-only camera/subject calibration. The estimator's DEFAULTS are
             # the real Pi-Camera values (0.45 m / 600 px) — unchanged on hardware.
             # The Gazebo camera runs horizontal_fov 0.9 rad (focal 662.2 px) and
             # the walking mannequin's MediaPipe shoulder span is ~0.315 m, so the
             # unchanged shoulder-width range formula now reads the true distance
             # while preserving the pinhole distance formula.
             parameters=[{
                 "known_shoulder_width": 0.315,
                 "focal_length": 662.2,
                 "enable_torso_fusion": True,
                 "torso_range_constant": 326.7,
                 "shoulder_fusion_weight": 0.35,
                 "max_distance": 7.0,
             }]),

        # HUD overlay for demo recording — publishes /camera/overlay
        Node(package="drone_perception", executable="hud_overlay_node.py",
             name="hud_overlay_node", output="screen",
             parameters=[{"track_fresh_timeout": 1.2}]),

        # SIM ONLY: highlights the tracked person in the Gazebo GUI
        # (breathing ground ring + spinning gem above the head, teal when
        # FOLLOWING, amber TRACKING, red pulse TARGET_LOST, 0.45s lock-on
        # animation on lock). GUI-side markers only — invisible to the
        # drone camera/perception.
        # world_name/world_sdf must match the world the Gazebo server runs
        # (iris_classroom is the current demo world; iris_warehouse is the
        # old warehouse fallback).
        Node(package="drone_perception", executable="sim_target_highlight_node.py",
             name="sim_target_highlight_node", output="screen",
             parameters=[{
                 "world_name": "iris_classroom",
                 "world_sdf": "/home/home/ardupilot_gazebo/worlds/iris_classroom.sdf",
             }]),

        # State + Control
        Node(package="drone_state", executable="fcu_bridge_node",
             name="fcu_bridge_node", output="screen",
             parameters=[{"use_odom_height_fallback": True}]),
        Node(package="drone_state", executable="world_model_node",
             name="world_model_node", output="screen",
             parameters=[{"target_fresh_timeout": 1.2}]),
        Node(package="drone_behavior", executable="mission_manager_node",
             name="mission_manager_node", output="screen",
             parameters=[{"target_lost_timeout": 1.5}]),
        Node(package="drone_control", executable="follow_controller_node",
             name="follow_controller_node", output="screen",
             parameters=[{
                 "desired_distance": 4.0,
                 "desired_altitude": 1.65,
                 "max_vx": 0.5,
                 "vx_slew_fast": 0.05,
                 "vx_slew_slow": 0.02,
                 "target_fresh_timeout": 1.2,
             }]),
        Node(package="drone_control", executable="setpoint_validation_node",
             name="setpoint_validation_node", output="screen"),
    ])
