import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define the path to the config file for the robot_driver_cpp package
    config_file = os.path.join(
        get_package_share_directory('robot_driver_cpp'), 'config', 'config.yaml'
    )

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='robot_driver_cpp',
            executable='turn_action_servers',
            name='turn_action_servers',
            output='screen',
            parameters=[config_file]
        ),
        
        
    ])
