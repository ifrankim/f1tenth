import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('gap_follow'),
        'config', 'params.yaml'
    )
    node = Node(
        package="gap_follow",
        executable="gap_follow",
        name="gap_follow",
        parameters = [config]
    )

    ld.add_action(node)
    return ld
