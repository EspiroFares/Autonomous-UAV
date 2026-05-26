from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="drone_perception",
                executable="camera_driver_node",
                name="camera_driver_node",
                output="screen",
            ),
            Node(
                package="drone_perception",
                executable="person_detector_node",
                name="person_detector_node",
                output="screen",
            ),
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
            Node(
                package="drone_behavior",
                executable="mission_manager_node",
                name="mission_manager_node",
                output="screen",
            ),
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
