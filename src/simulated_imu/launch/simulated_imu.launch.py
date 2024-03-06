from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simulated_imu',
            executable='simulated_imu',
            name='simulated_imu',
        ),
    ])