from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("rm_handeye_calibration"))
    default_config = package_share / "config" / "handeye.yaml"

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=str(default_config),
        description="Path to the hand-eye calibration parameter YAML",
    )

    node = Node(
        package="rm_handeye_calibration",
        executable="handeye_calibration_node",
        name="handeye_calibration",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("config")],
    )

    return LaunchDescription([config_arg, node])
