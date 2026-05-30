from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # --- Perception pipeline ---
            Node(
                package="drone_perception",
                executable="fake_camera_driver_node.py",
                name="fake_camera_driver_node",
                output="screen",
            ),
            Node(
                package="drone_perception",
                executable="image_preprocessing_node",
                name="image_preprocessing_node",
                output="screen",
            ),
            Node(
                package="drone_perception",
                executable="person_detector_node.py",
                name="person_detector_node",
                output="screen",
            ),
            Node(
                package="drone_perception",
                executable="person_tracker_node",
                name="person_tracker_node",
                output="screen",
            ),
            Node(
                package="drone_perception",
                executable="target_estimator_node",
                name="target_estimator_node",
                output="screen",
            ),
            # --- State layer ---
            Node(
                package="drone_state",
                executable="fcu_bridge_node",
                name="fcu_bridge_node",
                output="screen",
            ),
            Node(
                package="drone_state",
                executable="world_model_node",
                name="world_model_node",
                output="screen",
            ),
            # --- Behavior ---
            Node(
                package="drone_behavior",
                executable="mission_manager_node",
                name="mission_manager_node",
                output="screen",
            ),
            # --- Control ---
            Node(
                package="drone_control",
                executable="follow_controller_node",
                name="follow_controller_node",
                output="screen",
            ),
            Node(
                package="drone_control",
                executable="setpoint_validation_node",
                name="setpoint_validation_node",
                output="screen",
            ),
        ]
    )
