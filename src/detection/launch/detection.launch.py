from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Launch the can_detection_node
    can_detection_node = Node(
        package='detection',
        executable='can_detection_node',
        name='can_detection_node',
        output='screen'
    )



    # Return the combined LaunchDescription object
    return LaunchDescription([
        can_detection_node,
    ])
