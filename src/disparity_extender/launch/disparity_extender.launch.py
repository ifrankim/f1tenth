import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('disparity_extender'),
        'config', 'params.yaml'
    )
    node = Node(
        package="disparity_extender",
        executable="disparity_extender",
        name="disparity_extender",
        parameters = [config]
    )

    ld.add_action(node)
    return ld
