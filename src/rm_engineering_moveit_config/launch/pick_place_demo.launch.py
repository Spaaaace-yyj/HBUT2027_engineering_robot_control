from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "rm_engineering_robot",
            package_name="rm_engineering_moveit_config",
        )
        .robot_description(
            file_path="config/rm_engineering_robot.urdf.xacro",
            mappings={
                "hardware_type": "isaac",
            },
        )
        .to_moveit_configs()
    )

    pick_place_demo = Node(
        package="mtc_tutorial",
        executable="mtc_node",
        name="mtc_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([
        pick_place_demo,
    ])