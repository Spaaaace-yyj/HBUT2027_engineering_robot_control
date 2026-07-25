from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    zed_wrapper_share = get_package_share_directory("zed_wrapper")

    zed_launch = os.path.join(
        zed_wrapper_share,
        "launch",
        "zed_camera.launch.py",
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_launch),
            launch_arguments={
                "camera_model": "zed",
                "camera_name": "zed",

                # 保留ZED自己的内部静态TF
                "publish_urdf": "true",

                # 禁止ZED接管机器人TF树
                "publish_tf": "false",
                "publish_map_tf": "false",

                # 当前抓取项目不需要单独发布IMU TF
                "publish_imu_tf": "false",
            }.items(),
        )
    ])