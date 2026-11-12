"""Run Orin simulation and a leased foundation/policy session on the Pi."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    defaults = {
        "remote_host": "192.168.0.3",
        "remote_environment": "/opt/ros/humble/setup.bash",
        "device": "cpu",
        "uv_executable": "uv",
        "command_timeout": "0.25",
        "startup_timeout": "120.0",
    }
    required = [
        "remote_user",
        "remote_workspace",
        "policy_bundle",
        "remote_mapping_file",
        "rollout_repository",
        "run_directory",
        "mapping_file",
    ]
    declarations = [DeclareLaunchArgument(name) for name in required]
    declarations += [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ]
    return LaunchDescription(
        declarations
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("catbot_launcher"), "/launch/main.launch.py"]
                )
            ),
            Node(
                package="catbot_sim",
                executable="remote",
                output="screen",
                on_exit=Shutdown(),
                arguments=[
                    "--host",
                    LaunchConfiguration("remote_host"),
                    "--user",
                    LaunchConfiguration("remote_user"),
                    "--workspace",
                    LaunchConfiguration("remote_workspace"),
                    "--environment",
                    LaunchConfiguration("remote_environment"),
                    "--policy-bundle",
                    LaunchConfiguration("policy_bundle"),
                    "--mapping-file",
                    LaunchConfiguration("remote_mapping_file"),
                ],
            ),
            Node(
                package="catbot_sim",
                executable="bridge",
                output="screen",
                on_exit=Shutdown(),
                parameters=[
                    {
                        name: LaunchConfiguration(name)
                        for name in (
                            "rollout_repository",
                            "run_directory",
                            "mapping_file",
                            "device",
                            "uv_executable",
                            "command_timeout",
                            "startup_timeout",
                        )
                    }
                ],
            ),
        ]
    )
