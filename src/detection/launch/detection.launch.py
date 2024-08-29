import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define the path to the configuration file
    config_file = os.path.join(
        get_package_share_directory('detection'), 'config', 'config.yaml'
    )

    # Declare the configuration file as a launch argument
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the configuration file'
    )

    return launch.LaunchDescription([
        # Declare the configuration file argument
        declare_config_file_cmd,

        # Launch the can_detection_node with parameters from the configuration file
        launch_ros.actions.Node(
            package='detection',
            executable='can_detection_node',
            name='can_detection_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        ),

      
    ])
