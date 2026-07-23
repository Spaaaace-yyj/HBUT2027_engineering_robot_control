from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "rgb_topic",
            default_value="/zed/zed_node/rgb/color/rect/image",
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/zed/zed_node/depth/depth_registered",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value=(
                "/zed/zed_node/depth/depth_registered/camera_info"
            ),
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value="~/foundationpose_capture",
        ),
        DeclareLaunchArgument(
            "scene_name",
            default_value="my_object",
        ),
        DeclareLaunchArgument(
            "mesh_source_dir",
            default_value="",
        ),
        DeclareLaunchArgument("max_depth_m", default_value="10.0"),
        DeclareLaunchArgument("record_on_start", default_value="false"),
        DeclareLaunchArgument("save_every_n_frames", default_value="1"),
        DeclareLaunchArgument("max_recorded_frames", default_value="0"),
        DeclareLaunchArgument("writer_queue_size", default_value="60"),
        DeclareLaunchArgument("save_depth_preview", default_value="false"),
        DeclareLaunchArgument("log_every_n_saved", default_value="30"),
    ]

    recorder = Node(
        package="foundationpose_data_recorder",
        executable="foundationpose_data_recorder",
        name="foundationpose_data_recorder",
        output="screen",
        parameters=[
            {
                "rgb_topic": LaunchConfiguration("rgb_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration(
                    "camera_info_topic"
                ),
                "output_dir": LaunchConfiguration("output_dir"),
                "scene_name": LaunchConfiguration("scene_name"),
                "mesh_source_dir": LaunchConfiguration("mesh_source_dir"),
                "max_depth_m": ParameterValue(
                    LaunchConfiguration("max_depth_m"), value_type=float
                ),
                "record_on_start": ParameterValue(
                    LaunchConfiguration("record_on_start"), value_type=bool
                ),
                "save_every_n_frames": ParameterValue(
                    LaunchConfiguration("save_every_n_frames"), value_type=int
                ),
                "max_recorded_frames": ParameterValue(
                    LaunchConfiguration("max_recorded_frames"), value_type=int
                ),
                "writer_queue_size": ParameterValue(
                    LaunchConfiguration("writer_queue_size"), value_type=int
                ),
                "save_depth_preview": ParameterValue(
                    LaunchConfiguration("save_depth_preview"), value_type=bool
                ),
                "log_every_n_saved": ParameterValue(
                    LaunchConfiguration("log_every_n_saved"), value_type=int
                ),
            }
        ],
    )

    return LaunchDescription(arguments + [recorder])
