"""Single source of launch defaults, placement and topic wiring."""

import json
import os
import shlex
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from catbot_policy.artifacts import available_models, resolve_downloaded
from catbot_policy.configuration import POLICY_PARAMETERS
from catbot_policy.worker_launch import WorkerLaunchSpec
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
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
    "worker_environment",
    "worker_sync",
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


class ExpandedSettings(dict):
    """Resolve path templates once, including references to other defaults."""

    def __init__(self, raw):
        super().__init__()
        self.raw = raw
        self.active = set()

    def __missing__(self, key):
        if key not in self.raw or key in self.active:
            raise ValueError(f"Unknown or cyclic launch template: {key}")
        self.active.add(key)
        try:
            value = self.raw[key].format_map(self)
        finally:
            self.active.remove(key)
        self[key] = value
        return value


worker_environment_options = WorkerLaunchSpec.resolve


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
        expanded = ExpandedSettings(raw)
        values: dict[str, Any] = {}
        for key, original in defaults.items():
            value = expanded[key]
            values[key] = decode_setting(original, value)
        if values["run_id"] or values["checkpoint"]:
            selected = resolve_downloaded(
                values["policy_artifacts_directory"],
                values["run_id"],
                values["checkpoint"],
                entity=values["policy_entity"],
                project=values["policy_project"],
            )
            values["run_directory"] = str(selected)
            values["policy_bundle"] = str(selected / "bundle")
            # Pin the same checkpoint on the Pi, even if it has newer local downloads.
            source = json.loads((selected / "bundle/source.json").read_text())
            values["run_id"] = source["run"]
            values["checkpoint"] = source["checkpoint"]
        if not (Path(values["run_directory"]).expanduser() / "config.json").is_file():
            raise ValueError(
                f"Missing downloaded run: {values['run_directory']}\n"
                + available_models(values["policy_artifacts_directory"])
            )
        for key in (
            "policy_bundle",
            "mapping_file",
            "run_directory",
            "rollout_repository",
            "policy_worker_project",
        ):
            values[key] = str(Path(values[key]).expanduser())
        for device, project, environment, sync in (
            ("device", "rollout_repository", "worker_environment", "worker_sync"),
            (
                "policy_device",
                "policy_worker_project",
                "policy_worker_environment",
                "policy_worker_sync",
            ),
            (
                "remote_policy_device",
                "remote_policy_worker_project",
                "remote_policy_worker_environment",
                "remote_policy_worker_sync",
            ),
        ):
            values[environment], values[sync] = worker_environment_options(
                values[device], values[project], values[environment], values[sync]
            )
            if sync != "worker_sync":
                # Policy nodes accept auto/true/false strings in every launch path.
                values[sync] = str(values[sync]).lower()
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
                            **{
                                key: values[key]
                                for key in POLICY_PARAMETERS
                                if key != "simulation_mode"
                            },
                            "simulation_mode": True,
                            "policy_python": ParameterValue(
                                values["policy_python"], value_type=str
                            ),
                            "policy_worker_environment": ParameterValue(
                                values["policy_worker_environment"], value_type=str
                            ),
                            "policy_worker_sync": ParameterValue(
                                values["policy_worker_sync"], value_type=str
                            ),
                        }
                    ],
                    remappings=[("/simulation/contract", "/policy/contract")],
                )
            )
        if remote_role is not None:
            actions.append(
                ExecuteProcess(
                    output="screen",
                    on_exit=Shutdown(),
                    cmd=[
                        "ros2",
                        "run",
                        "catbot_sim",
                        "remote",
                        *[
                            argument
                            for name in POLICY_PARAMETERS
                            if name != "simulation_mode"
                            for value in [
                                values.get("remote_" + name, values.get(name))
                            ]
                            for argument in (
                                "--" + name.replace("_", "-"),
                                str(value).lower()
                                if isinstance(value, bool)
                                else str(value),
                            )
                        ],
                        "--policy-ros-python",
                        values["remote_policy_ros_python"],
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
                    ],
                )
            )
        return actions

    return LaunchDescription([*declarations, OpaqueFunction(function=start)])
