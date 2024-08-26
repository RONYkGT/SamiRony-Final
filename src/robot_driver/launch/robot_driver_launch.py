from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_driver',
            executable='turn_right_action_server',
            name='turn_right_action_server',
            output='screen'
        ),
        Node(
            package='robot_driver',
            executable='turn_left_action_server',
            name='turn_left_action_server',
            output='screen'
        )
    ])
