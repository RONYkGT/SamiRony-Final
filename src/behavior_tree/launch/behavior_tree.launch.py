from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Retrieve the package directory
    behavior_tree_dir = get_package_share_directory('behavior_tree')

    # Path to the YAML configuration file
    config_file = os.path.join(behavior_tree_dir, 'config', 'config.yaml')

    return LaunchDescription([
        Node(
            package='behavior_tree',
            executable='trashbot_main',
            name='trashbot_main',
            output='screen',
            parameters=[config_file],  # Correctly pass the YAML file
            # Uncomment the line below to specify the node namespace
            # namespace='some_namespace'
        ),
    ])
