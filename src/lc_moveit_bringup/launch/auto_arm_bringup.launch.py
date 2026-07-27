import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    foundationpose_share = get_package_share_directory("foundationpose_shm_bridge")
    moveit_config_share = get_package_share_directory("rm_engineering_moveit_config")
    zed_wrapper_share = get_package_share_directory("zed_wrapper")

    default_zed_config = os.path.join(
        foundationpose_share, "config", "zed_shm_bridge.yaml"
    )

    hardware_type = LaunchConfiguration("hardware_type")
    zed_config_file = LaunchConfiguration("zed_config_file")
    use_rviz = LaunchConfiguration("use_rviz")

    declared_args = [
        DeclareLaunchArgument(
            "hardware_type",
            default_value="real",
            choices=["real", "isaac"],
            description="硬件类型: 'real'（真实机械臂）或 'isaac'（Isaac Sim 仿真）",
        ),
        DeclareLaunchArgument(
            "zed_config_file",
            default_value=default_zed_config,
            description="zed_shm_bridge 的 YAML 配置文件路径",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            choices=["true", "false"],
            description="是否启动 RViz2",
        ),
    ]

    is_real = PythonExpression(["'", hardware_type, "' == 'real'"])
    is_isaac = PythonExpression(["'", hardware_type, "' == 'isaac'"])

    # ZED 相机驱动禁止发布 map/odom TF
    zed_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_share, "launch", "zed_camera.launch.py")
        ),
        launch_arguments={
            "camera_model": "zed",
            "camera_name": "zed",
            "publish_urdf": "true",       # 保留 ZED 内部静态 TF
            "publish_tf": "false",        # 禁止接管机器人 TF 树
            "publish_map_tf": "false",    # 禁止发布 map TF
            "publish_imu_tf": "false",    # 不发布 IMU TF
        }.items(),
    )

    # ZED → 共享内存桥接
    zed_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(foundationpose_share, "launch", "zed_shm_bridge.launch.py")
        ),
        launch_arguments={
            "config_file": zed_config_file,
        }.items(),
    )

    # MoveIt2 环境 — 真实机械臂
    moveit_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, "launch", "demo.launch.py")
        ),
        launch_arguments={
            "use_rviz": use_rviz,
        }.items(),
        condition=IfCondition(is_real),
    )

    # MoveIt2 环境 — Isaac Sim 仿真
    moveit_isaac = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_config_share, "launch", "demo_isaac_sim.launch.py"
            )
        ),
        launch_arguments={
            "use_rviz": use_rviz,
        }.items(),
        condition=IfCondition(is_isaac),
    )

    return LaunchDescription(
        [
            *declared_args,
            zed_camera,
            zed_bridge,
            moveit_real,
            moveit_isaac,
        ]
    )
