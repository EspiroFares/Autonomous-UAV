import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("drone_bringup")
    perception_params = os.path.join(
        bringup_share,
        "config",
        "real_perception.yaml",
    )

    # 1. MAVROS
    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory("mavros"), "launch", "apm.launch")
        ),
        launch_arguments={"fcu_url": "serial:///dev/ttyAMA5:921600"}.items(),
    )

    # 2. Request the FC data streams (rangefinder)
    stream_requests = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/mavros/set_message_interval",
                    "mavros_msgs/srv/MessageInterval",
                    "{message_id: 132, message_rate: 10.0}",
                ]
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/mavros/set_message_interval",
                    "mavros_msgs/srv/MessageInterval",
                    "{message_id: 173, message_rate: 10.0}",
                ]
            ),
        ],
    )

    # 3. The full ROS stack (respawn=True)
    def n(pkg, exe, parameters=None):
        return Node(
            package=pkg,
            executable=exe,
            output="screen",
            respawn=True,
            parameters=parameters or [],
        )

    stack = TimerAction(
        period=12.0,
        actions=[
            # Perception
            n("drone_perception", "camera_driver_node.py"),
            n("drone_perception", "person_detector_node.py"),
            n("drone_perception", "person_tracker_node"),
            n(
                "drone_perception",
                "target_estimator_node",
                parameters=[perception_params],
            ),
            # State + Behavior + Control
            n("drone_state", "fcu_bridge_node"),
            n("drone_state", "world_model_node"),
            n("drone_behavior", "mission_manager_node"),
            n("drone_control", "follow_controller_node"),
            n("drone_control", "setpoint_validation_node"),
        ],
    )

    return LaunchDescription([mavros, stream_requests, stack])
