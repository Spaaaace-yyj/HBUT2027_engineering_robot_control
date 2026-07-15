from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "rm_engineering_robot",
            package_name="rm_engineering_moveit_config",
        )
        # todo：后续要替换config,在这里添加硬件参数，硬件描述也要有对应的参数（如果要用仿真的话）
        .robot_description(
            file_path="config/rm_engineering_robot.urdf.xacro",
            mappings={
                "hardware_type": "isaac",
            },
        )
        .to_moveit_configs()
    )

    return generate_demo_launch(moveit_config)