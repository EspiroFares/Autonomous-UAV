from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='drone_state',
            executable='mock_fcu_node',
            name='mock_fcu_node',
            output='screen'
        ),

        Node(
            package='drone_state',
            executable='mock_target_node',
            name='mock_target_node',
            output='screen'
        ),

        Node(
            package='drone_state',
            executable='world_model_node',
            name='world_model_node',
            output='screen'
        ),

        Node(
            package='drone_behavior',
            executable='mission_manager_node',
            name='mission_manager_node',
            output='screen'
        ),

        Node(
            package='drone_control',
            executable='follow_controller_node',
            name='follow_controller_node',
            output='screen'
        ),

        Node(
            package='drone_control',
            executable='setpoint_validation_node',
            name='setpoint_validation_node',
            output='screen'
        ),

    ])
