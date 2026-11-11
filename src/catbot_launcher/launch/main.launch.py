from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_model = LaunchConfiguration("camera_model")
    camera_name = LaunchConfiguration("camera_name")
    serial_number = LaunchConfiguration("serial_number")
    config_file = LaunchConfiguration("config_file")

    zed_launch_file = PythonLaunchDescriptionSource(
        [
            FindPackageShare("zed_wrapper"),
            "/launch/zed_camera.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_camera", default_value="true"),
            DeclareLaunchArgument("enable_foxglove", default_value="true"),
            DeclareLaunchArgument(
                "camera_model",
                default_value="zed2i",
                description="ZED camera model (for example: zedx, zed2i, or zedm)",
            ),
            DeclareLaunchArgument(
                "camera_name",
                default_value="zed",
                description="ROS namespace used for the camera topics",
            ),
            DeclareLaunchArgument(
                "serial_number",
                default_value="0",
                description="Camera serial number; 0 selects the first available camera",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("catbot_launcher"),
                        "config",
                        "zed_params.yaml",
                    ]
                ),
                description="ROS parameter overrides for the ZED wrapper",
            ),
            IncludeLaunchDescription(
                zed_launch_file,
                condition=IfCondition(LaunchConfiguration("enable_camera")),
                launch_arguments={
                    "camera_model": camera_model,
                    "camera_name": camera_name,
                    "serial_number": serial_number,
                    "node_log_type": "screen",
                    "enable_ipc": "false",
                    "ros_params_override_path": config_file,
                }.items(),
            ),
            Node(
                package="foxglove_bridge",
                condition=IfCondition(LaunchConfiguration("enable_foxglove")),
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                parameters=[
                    {
                        "address": "0.0.0.0",
                        "port": 8765,
                    }
                ],
            ),
        ]
    )
