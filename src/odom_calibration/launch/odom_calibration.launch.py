import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("odom_calibration"), "config", "params.yaml"
    )
    node = Node(
        package="odom_calibration",
        executable="odom_calibration",
        name="odom_calibration",
    )
    node2 = Node(
        package="odom_calibration",
        executable="square_movement",
        name="square_movement",
    )
    
    ld.add_action(node)
    ld.add_action(node2)
    
    return ld