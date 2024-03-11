import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("robot_localization_pkg"), "config", "params.yaml"
    )
    node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[config],
        output="screen",
    )

    imuNode = Node(
        package="robot_localization_pkg",
        executable="simulated_imu",
        name="simulated_imu",
    )

    odomNode = Node(
        package="robot_localization_pkg",
        executable="noisy_odom",
        name="noisy_odom",
    )

    tfNode = Node(
        package="robot_localization_pkg",
        executable="tf_transform",
        name="tf_transform",       
    )

    ld.add_action(node)
    ld.add_action(imuNode)
    ld.add_action(odomNode)
    ld.add_action(tfNode)

    return ld
