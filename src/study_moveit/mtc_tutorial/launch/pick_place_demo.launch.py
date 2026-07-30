from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 只启动 MTC 节点；MoveIt/Isaac 仿真环境由 demo_isaac_sim.launch.py 另行启动。
    # MTC 节点必须自己获得 URDF、SRDF、kinematics.yaml 和 OMPL 参数，
    # 它不会从 /move_group 自动继承这些参数。
    moveit_config = (
        MoveItConfigsBuilder(
            "rm_engineering_robot",
            package_name="rm_engineering_moveit_config",
        )
        .robot_description(
            file_path="config/rm_engineering_robot.urdf.xacro",
            mappings={"hardware_type": "isaac"},
        )
        .robot_description_semantic(
            file_path="config/rm_engineering_robot.srdf",
        )
        .robot_description_kinematics(
            file_path="config/kinematics.yaml",
        )
        .planning_pipelines(
            pipelines=["ompl"],
        )
        .to_moveit_configs()
    )

    mtc_node = Node(
        package="mtc_tutorial",
        executable="mtc_node",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([mtc_node])
