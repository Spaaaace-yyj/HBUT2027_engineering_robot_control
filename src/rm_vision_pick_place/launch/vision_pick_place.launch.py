from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_share = Path(get_package_share_directory("rm_vision_pick_place"))

    moveit_config = (
        MoveItConfigsBuilder(
            "rm_engineering_robot",
            package_name="rm_engineering_moveit_config",
        )
        .robot_description(
            file_path="config/rm_engineering_robot.urdf.xacro",
            mappings={"hardware_type": "real"},
        )
        .to_moveit_configs()
    )

    node = Node(
        package="rm_vision_pick_place",
        executable="vision_pick_place_node",
        name="vision_pick_place",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            str(package_share / "config" / "pick_place.yaml"),
        ],
    )

    return LaunchDescription([node])
