from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Retrieve the package directories
    behavior_tree_dir = get_package_share_directory('behavior_tree')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time if true'),
        
        Node(
            package='behavior_tree',
            executable='trashbot_main',
            name='trashbot_main',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            # Uncomment the line below to specify the node namespace
            # namespace='some_namespace'
        ),
    ])
