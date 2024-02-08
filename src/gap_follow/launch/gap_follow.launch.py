from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gap_follow',
            executable='gap_follow',
            name='gap_follow',
        ),
    ])
