from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Launch the turn_action_servers executable
    turn_action_servers_node = Node(
        package='robot_driver_cpp',
        executable='turn_action_servers',
        name='turn_action_servers',
        output='screen'
    )

    return LaunchDescription([
        turn_action_servers_node
    ])