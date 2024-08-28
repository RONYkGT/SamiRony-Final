from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Retrieve the package directories
    package1_dir = get_package_share_directory('detection') 
    package2_dir = get_package_share_directory('robot_driver_cpp')  
    package3_dir = get_package_share_directory('behavior_tree')  
    
    # Define the paths to the launch files
    package1_launch_file = os.path.join(package1_dir, 'launch', 'detection.launch.py')
    package2_launch_file = os.path.join(package2_dir, 'launch', 'robot_driver.launch.py')
    package3_launch_file = os.path.join(package3_dir, 'launch', 'behavior_tree.launch.py')


    # Include launch files
    launch_file1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(package1_launch_file)
    )
    launch_file2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(package2_launch_file)
    )
    launch_file3_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(package3_launch_file)
    )


    # Return the combined LaunchDescription object
    return LaunchDescription([
        launch_file1_include,
        launch_file2_include,
        launch_file3_include
    ])