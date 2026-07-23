from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("foundationpose_shm_bridge")
    )
    default_config = package_share / "config" / "foundationpose_bridge.yaml"

    config_file = LaunchConfiguration("config_file")
    start_worker = LaunchConfiguration("start_worker")
    conda_executable = LaunchConfiguration("conda_executable")
    conda_env = LaunchConfiguration("conda_env")
    worker_script = LaunchConfiguration("worker_script")
    weights_file = LaunchConfiguration("weights_file")
    mesh_file = LaunchConfiguration("mesh_file")
    mesh_scale = LaunchConfiguration("mesh_scale")
    device = LaunchConfiguration("device")
    half = LaunchConfiguration("half")
    show_window = LaunchConfiguration("show_window")
    rgbd_shm_name = LaunchConfiguration("rgbd_shm_name")
    control_shm_name = LaunchConfiguration("control_shm_name")
    result_shm_name = LaunchConfiguration("result_shm_name")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=str(default_config)),
            DeclareLaunchArgument("start_worker", default_value="true"),
            DeclareLaunchArgument(
                "conda_executable",
                default_value="/home/spaaaaace/anaconda3/bin/conda",
            ),
            DeclareLaunchArgument("conda_env", default_value="foundationpose"),
            DeclareLaunchArgument(
                "worker_script",
                default_value=(
                    "/home/spaaaaace/Code/FoundationPose/"
                    "foundationpose_server/foundationpose_worker.py"
                ),
            ),
            DeclareLaunchArgument(
                "weights_file",
                default_value=(
                    "/home/spaaaaace/Code/FoundationPose/"
                    "foundationpose_server/weiget/best.pt"
                ),
            ),
            DeclareLaunchArgument(
                "mesh_file",
                default_value=(
                    "/home/spaaaaace/Code/FoundationPose/"
                    "foundationpose_server/obj_model/real_color_obj.obj"
                ),
            ),
            DeclareLaunchArgument("mesh_scale", default_value="0.001"),
            DeclareLaunchArgument("device", default_value="0"),
            DeclareLaunchArgument("half", default_value="1"),
            DeclareLaunchArgument("show_window", default_value="0"),
            DeclareLaunchArgument("rgbd_shm_name", default_value="/foundationpose_rgbd"),
            DeclareLaunchArgument("control_shm_name", default_value="/foundationpose_control"),
            DeclareLaunchArgument("result_shm_name", default_value="/foundationpose_result"),
            Node(
                package="foundationpose_shm_bridge",
                executable="zed_shm_bridge_node",
                name="foundationpose_bridge",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "rgbd_shm_name": rgbd_shm_name,
                        "control_shm_name": control_shm_name,
                        "result_shm_name": result_shm_name,
                    },
                ],
            ),
            ExecuteProcess(
                condition=IfCondition(start_worker),
                output="screen",
                cmd=[
                    conda_executable,
                    "run",
                    "--no-capture-output",
                    "-n",
                    conda_env,
                    "python",
                    worker_script,
                    "--weights",
                    weights_file,
                    "--mesh-file",
                    mesh_file,
                    "--mesh-scale",
                    mesh_scale,
                    "--device",
                    device,
                    "--half",
                    half,
                    "--show-window",
                    show_window,
                    "--rgbd-shm-name",
                    rgbd_shm_name,
                    "--control-shm-name",
                    control_shm_name,
                    "--result-shm-name",
                    result_shm_name,
                ],
            ),
        ]
    )
