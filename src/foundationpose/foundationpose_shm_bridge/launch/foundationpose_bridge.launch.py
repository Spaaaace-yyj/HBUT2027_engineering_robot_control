from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("foundationpose_shm_bridge")
    )
    default_config = package_share / "config" / "foundationpose_bridge.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(default_config),
                description="Bridge parameter YAML file",
            ),
            Node(
                package="foundationpose_shm_bridge",
                executable="zed_shm_bridge_node",
                name="foundationpose_bridge",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
            ),
        ]
    )
