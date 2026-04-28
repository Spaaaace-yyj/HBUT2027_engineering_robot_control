import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    # ===== 1. 你的机器人（替换成你的）=====
    moveit_config = (
        MoveItConfigsBuilder("rm_engineering")
        .robot_description(file_path="config/rm_engineering_robot.urdf.xacro")
        .robot_description_kinematics(
            file_path="config/kinematics.yaml"
        )
        .to_moveit_configs()
    )

    # ===== 2. Servo参数 =====
    servo_yaml = load_yaml(
        "lc_moveit_bringup",
        "config/servo.yaml"
    )
    servo_params = {"moveit_servo": servo_yaml}

    # RViz
    rviz_config_file = (
            get_package_share_directory("rm_engineering_moveit_config") + "/config/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    # ===== 3. ros2_control（用 velocity controller）=====
    ros2_controllers_path = os.path.join(
        get_package_share_directory("rm_engineering_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
        ],
        output="screen",
    )

    # ===== 4. controller 启动 =====
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rm_robot_arm_controller", "-c", "/controller_manager"],
    )

    # ===== 6. 组件容器 =====
    container = ComposableNodeContainer(
        name="servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[

            # TF
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),

            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "/base_link", "frame_id": "/world"}],
            ),

            # 手柄
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),

            # 手柄 → Servo
            # ComposableNode(
            #     package="moveit_servo",
            #     plugin="moveit_servo::JoyToServoPub",
            #     name="controller_to_servo_node",
            # ),
        ],
        output="screen",
    )

    # ===== 5. Servo节点 =====
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([
        rviz_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        velocity_controller_spawner,
        servo_node,
        container,
        move_group_node,
    ])