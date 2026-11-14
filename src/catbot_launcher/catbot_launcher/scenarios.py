"""Single source of launch defaults, placement and topic wiring."""

import os
import shlex
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SCENARIOS = {
    "physical_pi": ("hardware", "pi", "both"),
    "physical_orin": ("hardware", "orin", "foundation"),
    "simulation_pi": ("simulation", "pi", "policy"),
    "simulation_orin": ("simulation", "orin", None),
}
BRIDGE_KEYS = (
    "rollout_repository",
    "run_directory",
    "mapping_file",
    "device",
    "uv_executable",
    "command_timeout",
    "worker_timeout",
    "visualization_rate",
    "startup_timeout",
    "sensor_timeout",
    "sensor_max_skew",
    "ground_height_offset",
    "contact_height",
    "estimate_contacts",
    "body_from_camera_wxyz",
    "body_from_imu_wxyz",
    "body_to_camera_xyz",
    "imu_includes_gravity",
    "imu_topic",
    "odom_topic",
    "contact_topic",
)


def decode_setting(original: Any, value: str) -> Any:
    """Preserve the YAML default's type when applying launch overrides."""
    if isinstance(original, str):
        return value
    parsed = yaml.safe_load(value)
    if isinstance(original, bool):
        if not isinstance(parsed, bool):
            raise ValueError("Boolean launch options must be true or false")
        return parsed
    if isinstance(original, list):
        if not isinstance(parsed, list):
            raise ValueError("Array launch options must be YAML lists")
        return [float(v) for v in parsed]
    if isinstance(original, float):
        return float(parsed)
    if isinstance(original, int):
        return int(parsed)
    raise ValueError("Unsupported launch default type")


def build_launch(default_scenario="physical_pi"):
    share = Path(get_package_share_directory("catbot_launcher"))
    defaults_path = Path(
        os.environ.get("CATBOT_LAUNCH_DEFAULTS", share / "config/scenarios.yaml")
    )
    defaults = yaml.safe_load(defaults_path.read_text())
    defaults.setdefault("camera_config", str(share / "config/zed_policy.yaml"))
    defaults.setdefault("enable_camera", "auto")
    declarations = [
        DeclareLaunchArgument(
            "scenario", default_value=default_scenario, choices=list(SCENARIOS)
        )
    ]
    for key, value in defaults.items():
        encoded = (
            yaml.safe_dump(value, default_flow_style=True).split("\n...")[0].strip()
            if not isinstance(value, str)
            else value
        )
        declarations.append(DeclareLaunchArgument(key, default_value=encoded))

    def start(context):
        name = LaunchConfiguration("scenario").perform(context)
        backend, policy_host, remote_role = SCENARIOS[name]
        raw = {key: LaunchConfiguration(key).perform(context) for key in defaults}
        values: dict[str, Any] = {}
        for key, original in defaults.items():
            value = raw[key].format(
                workspace=raw["workspace"], camera_name=raw["camera_name"]
            )
            values[key] = decode_setting(original, value)
        for key in (
            "policy_bundle",
            "mapping_file",
            "run_directory",
            "rollout_repository",
            "policy_worker_project",
        ):
            values[key] = str(Path(values[key]).expanduser())
        camera = values["enable_camera"]
        if isinstance(camera, bool):
            pass
        elif camera == "auto":
            camera = backend == "hardware"
        elif isinstance(camera, str) and camera.lower() in ("true", "false"):
            camera = camera.lower() == "true"
        else:
            raise ValueError("enable_camera must be auto, true or false")
        actions = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(share / "launch/main.launch.py")),
                launch_arguments={
                    "enable_camera": str(camera).lower(),
                    "enable_foxglove": str(values["enable_foxglove"]).lower(),
                    "camera_model": values["camera_model"],
                    "camera_name": values["camera_name"],
                    "serial_number": str(values["serial_number"]),
                    "config_file": values["camera_config"],
                    "foxglove_address": str(values["foxglove_address"]),
                    "foxglove_port": str(values["foxglove_port"]),
                }.items(),
            )
        ]
        actions.append(
            Node(
                package="catbot_sim",
                executable="bridge",
                output="screen",
                on_exit=Shutdown(),
                parameters=[
                    {"backend": backend, **{key: values[key] for key in BRIDGE_KEYS}}
                ],
            )
        )
        if policy_host == "orin":
            actions.append(
                Node(
                    package="catbot_policy",
                    executable="policy",
                    prefix=shlex.quote(values["policy_ros_python"]),
                    output="screen",
                    on_exit=Shutdown(),
                    parameters=[
                        {
                            "policy_bundle": values["policy_bundle"],
                            "mapping_file": values["mapping_file"],
                            "simulation_mode": True,
                            "policy_python": ParameterValue(
                                values["policy_python"], value_type=str
                            ),
                            "policy_worker_project": values["policy_worker_project"],
                            "policy_uv_executable": values["policy_uv_executable"],
                            "policy_worker_timeout": values["policy_worker_timeout"],
                            "policy_startup_timeout": values["policy_startup_timeout"],
                            "feedback_timeout": values["feedback_timeout"],
                        }
                    ],
                    remappings=[("/simulation/contract", "/policy/contract")],
                )
            )
        if remote_role is not None:
            actions.append(
                Node(
                    package="catbot_sim",
                    executable="remote",
                    output="screen",
                    on_exit=Shutdown(),
                    arguments=[
                        "--policy-ros-python",
                        values["remote_policy_ros_python"],
                        "--policy-worker-project",
                        values["remote_policy_worker_project"],
                        "--policy-uv-executable",
                        values["remote_policy_uv_executable"],
                        "--policy-worker-timeout",
                        str(values["policy_worker_timeout"]),
                        "--policy-startup-timeout",
                        str(values["policy_startup_timeout"]),
                        "--policy-python",
                        values["remote_policy_python"],
                        "--feedback-timeout",
                        str(values["feedback_timeout"]),
                        "--role",
                        remote_role,
                        "--host",
                        values["remote_host"],
                        "--user",
                        values["remote_user"],
                        "--workspace",
                        values["remote_workspace"],
                        "--environment",
                        values["remote_environment"],
                        "--policy-bundle",
                        values["remote_policy_bundle"],
                        "--mapping-file",
                        values["remote_mapping_file"],
                    ],
                )
            )
        return actions

    return LaunchDescription([*declarations, OpaqueFunction(function=start)])
