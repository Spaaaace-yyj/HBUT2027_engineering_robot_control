from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("rm_vision_pick_place")
    )

    hardware_type = LaunchConfiguration("hardware_type")
    use_sim_time = LaunchConfiguration("use_sim_time")
    auto_execute = LaunchConfiguration("auto_execute")

    # 这个节点会自己创建 RobotModel、ComputeIK 和 PipelinePlanner，
    # 因此不能只依赖 /move_group 的参数；必须把完整 MoveIt 配置再次传入本节点。
    moveit_config = (
        MoveItConfigsBuilder(
            "rm_engineering_robot",
            package_name="rm_engineering_moveit_config",
        )
        .robot_description(
            file_path="config/rm_engineering_robot.urdf.xacro",
            mappings={"hardware_type": hardware_type},
        )
        .robot_description_semantic()
        .robot_description_kinematics()
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    node = Node(
        package="rm_vision_pick_place",
        executable="vision_pick_place_node",
        name="vision_pick_place",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            str(package_share / "config" / "pick_place.yaml"),
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "auto_execute": ParameterValue(auto_execute, value_type=bool),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_type",
                default_value="real",
                description="URDF ros2_control hardware type: real or isaac",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock",
            ),
            DeclareLaunchArgument(
                "auto_execute",
                default_value="true",
                description="Execute the MTC solution after planning",
            ),
            node,
        ]
    )
