import pytest
from catbot_launcher.scenarios import (
    ExpandedSettings,
    decode_setting,
    worker_environment_options,
)
from hypothesis import given
from hypothesis import strategies as st


@pytest.mark.parametrize(
    ("original", "text", "expected"),
    [
        ("x", "true", "true"),
        (True, "false", False),
        (1, "12", 12),
        (1.0, "2.5", 2.5),
        ([0.0, 0.0], "[1, 2]", [1.0, 2.0]),
    ],
)
def test_decode(original, text, expected):
    result = decode_setting(original, text)
    assert result == expected and type(result) is type(expected)


@pytest.mark.parametrize(
    ("original", "text"),
    [
        (True, "1"),
        ([0], "1"),
        ([0], "[]"),
        (1, "1.5"),
        (None, "0"),
        (1.0, "true"),
        (1, ".inf"),
        ([0], "[.nan]"),
    ],
)
def test_invalid_decode(original, text):
    with pytest.raises(ValueError):
        decode_setting(original, text)


@pytest.mark.parametrize(
    ("device", "env", "sync", "expected"),
    [
        ("cpu", "auto", "auto", ("", True)),
        ("cuda:0", "auto", "auto", ("/worker/.venv-cuda", False)),
        ("cuda", "/custom", "true", ("/custom", True)),
        ("cpu", "", "false", ("", False)),
    ],
)
def test_worker_options(device, env, sync, expected):
    assert worker_environment_options(device, "/worker", env, sync) == expected


@pytest.mark.parametrize("sync", ["invalid", 0, None])
def test_invalid_worker_options(sync):
    with pytest.raises(ValueError):
        worker_environment_options("cpu", "/worker", "", sync)


def test_expansion_is_recursive_and_cached():
    settings = ExpandedSettings(
        {"root": "/tmp", "bundle": "{root}/bundle", "file": "{bundle}/policy"}
    )
    assert settings["file"] == "/tmp/bundle/policy"
    settings.raw["root"] = "/changed"
    assert settings["root"] == "/tmp"


@pytest.mark.parametrize(
    "raw", [{"a": "{a}"}, {"a": "{b}", "b": "{a}"}, {"a": "{missing}"}]
)
def test_cycle_and_unknown_template(raw):
    settings = ExpandedSettings(raw)
    with pytest.raises(ValueError, match="Unknown or cyclic"):
        settings["a"]
    assert not settings.active


@given(
    st.sampled_from([0, 0.0, [0.0]]),
    st.sampled_from(["true", "false", ".nan", ".inf", "-.inf", "null"]),
)
def test_numeric_settings_never_accept_invalid_scalars(original, invalid):
    with pytest.raises(ValueError):
        decode_setting(
            original, "[" + invalid + "]" if isinstance(original, list) else invalid
        )


@given(st.integers(min_value=-100000, max_value=100000))
def test_integer_roundtrip(value):
    assert decode_setting(0, str(value)) == value


@given(st.lists(st.integers(-100, 100), max_size=12).filter(lambda v: len(v) != 4))
def test_array_length_cannot_change(values):
    import json

    with pytest.raises(ValueError):
        decode_setting([0.0] * 4, json.dumps(values))


@pytest.mark.parametrize(
    "scenario,camera,monitor_camera,motors,start_pi",
    [
        ("simulation_orin", "auto", "false", "false", False),
        ("simulation_orin", "auto", "false", "false", True),
        ("simulation_pi", "auto", "false", "false", False),
        ("simulation_pi", "auto", "false", "false", True),
        ("physical_orin", "auto", "true", "true", True),
        ("physical_orin", "auto", "true", "true", False),
        ("physical_pi", "auto", "true", "true", False),
        ("physical_orin", "false", "true", "true", False),
        ("simulation_orin", "true", "true", "false", False),
    ],
)
@pytest.mark.parametrize("saved_mount", [False, True])
def test_launch_applies_overrides_and_routes_workers(
    monkeypatch,
    tmp_path,
    scenario,
    camera,
    monitor_camera,
    motors,
    start_pi,
    saved_mount,
):
    """Execute launch assembly using controlled YAML and CLI overrides."""
    import sys
    from pathlib import Path
    from types import ModuleType, SimpleNamespace
    from unittest.mock import Mock

    from catbot_launcher.scenarios import build_launch

    share = Path(__file__).resolve().parents[1] / "src/catbot_launcher"
    declarations, opaque, description = Mock(), Mock(), Mock()
    modules = {
        "ament_index_python.packages": {
            "get_package_share_directory": Mock(return_value=str(share))
        },
        "launch": {"LaunchDescription": description},
        "launch.actions": {
            "DeclareLaunchArgument": declarations,
            "OpaqueFunction": opaque,
            "ExecuteProcess": Mock(),
            "IncludeLaunchDescription": Mock(),
            "Shutdown": Mock(),
        },
        "launch.launch_description_sources": {"PythonLaunchDescriptionSource": Mock()},
        "launch.substitutions": {"LaunchConfiguration": Mock()},
        "launch_ros.actions": {"Node": Mock()},
        "launch_ros.parameter_descriptions": {"ParameterValue": Mock()},
    }
    for name, members in modules.items():
        module = ModuleType(name)
        module.__dict__.update(members)
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setenv(
        "CATBOT_LAUNCH_DEFAULTS",
        str(Path(__file__).with_name("fixtures") / "launch_defaults.yaml"),
    )
    assert build_launch(scenario) is description.return_value
    defaults = {
        call.args[0]: call.kwargs["default_value"]
        for call in declarations.call_args_list[1:]
    }
    # Execute the deferred callback, checking the wiring in every scenario.
    (tmp_path / "config.json").write_text("{}")
    raw = {
        **defaults,
        "scenario": scenario,
        "run_directory": str(tmp_path),
        "enable_camera": camera,
        "start_pi": str(start_pi).lower(),
        "imu_topic": "/custom/imu",
        "odom_topic": "/custom/odom",
        "worker_timeout": "7.5",
        "body_to_camera_xyz": "[0.4, -0.2, 0.1]",
        "estimate_contacts": "true",
        "enable_camera_calibration": str(start_pi).lower(),
    }
    expected_xyz = [0.4, -0.2, 0.1]
    if saved_mount:
        from catbot_sim.camera_calibration import save_calibration

        xml = "<mujoco/>"
        (tmp_path / "model.xml").write_text(xml)
        expected_xyz = [0.35, 0.02, 0.07]
        calibration = tmp_path / "camera.json"
        save_calibration(
            calibration,
            {
                "body_from_camera_wxyz": [1, 0, 0, 0],
                "body_to_camera_xyz": expected_xyz,
            },
            xml,
        )
        raw["camera_calibration_file"] = str(calibration)
    modules["launch.substitutions"]["LaunchConfiguration"].side_effect = lambda key: (
        SimpleNamespace(perform=lambda context: raw[key])
    )
    if scenario == "physical_pi":
        with pytest.raises(ValueError, match="deprecated"):
            opaque.call_args.kwargs["function"](None)
        return
    opaque.call_args.kwargs["function"](None)
    call = modules["launch.actions"]["IncludeLaunchDescription"].call_args
    arguments = dict(call.kwargs["launch_arguments"])
    assert arguments["enable_diagnostics"] == "false"
    assert arguments["enable_foxglove"] == "false"
    assert arguments["monitor_camera"] == monitor_camera
    assert arguments["monitor_motors"] == motors
    assert arguments["monitor_odometry"] == motors
    assert arguments["monitor_robot"] == "true"
    assert arguments["imu_topic"] == "/custom/imu"
    assert arguments["odom_topic"] == "/custom/odom"
    nodes = {
        call.kwargs["package"]: call.kwargs
        for call in modules["launch_ros.actions"]["Node"].call_args_list
    }
    bridge = nodes["catbot_sim"]["parameters"][0]
    assert bridge["backend"] == (
        "hardware" if scenario.startswith("physical") else "simulation"
    )
    assert bridge["worker_timeout"] == 7.5
    assert bridge["camera_ground_height"] == 0.28
    assert bridge["camera_height_frame"] == "front_center"
    assert bridge["body_to_camera_xyz"] == expected_xyz
    assert bridge["estimate_contacts"] is True
    assert bridge["mapping_file"] == "/test workspace/mapping.json"
    calibrators = [
        call.kwargs
        for call in modules["launch_ros.actions"]["Node"].call_args_list
        if call.kwargs["executable"] == "camera_calibration"
    ]
    assert bool(calibrators) is (scenario == "physical_orin" and start_pi)
    if calibrators:
        parameters = calibrators[0]["parameters"][0]
        assert parameters["body_to_camera_xyz"] == expected_xyz
        assert parameters["run_directory"] == str(tmp_path)
        assert parameters["imu_topic"] == "/custom/imu"
    if scenario == "simulation_orin":
        assert "catbot_supervisor" not in nodes
        policy_node = nodes["catbot_policy"]
        assert policy_node["executable"] == "policy"
        assert policy_node["prefix"] == "/test/python3"
        policy = policy_node["parameters"][0]
        assert policy["policy_bundle"] == "/test workspace/bundle"
        assert policy["feedback_timeout"] == 0.6
        assert policy["simulation_mode"] is True
        assert policy["supervised_actions"] is False
        assert policy["require_provider_contract"] is True
        assert policy["provider_contract_topic"] == "/policy/contract"
    elif scenario == "physical_orin":
        assert "catbot_policy" not in nodes
        policy = nodes["catbot_supervisor"]["parameters"][0]
        assert policy["policy_bundle"] == "/test workspace/bundle"
        assert policy["feedback_timeout"] == 0.6
        assert policy["simulation_mode"] is scenario.startswith("simulation")
        assert policy["scenario"] == scenario
        assert nodes["catbot_supervisor"]["executable"] == "orin"
    else:
        assert "catbot_policy" not in nodes
    processes = modules["launch.actions"]["ExecuteProcess"].call_args_list
    if scenario == "physical_orin" and start_pi:
        assert len(processes) == 1
        command = processes[0].kwargs["cmd"]
        options = dict(zip(command[4::2], command[5::2], strict=True))
        assert options == {
            "--role": "supervisor",
            "--host": "test-pi.local",
            "--user": "test-user",
            "--workspace": "/remote workspace",
            "--environment": "/remote environment.bash",
            "--ros-overlay": "/remote overlay/setup.bash",
            "--network-interface": "test-eth",
            "--mapping-file": "/remote workspace/mapping.json",
        }
    elif scenario.endswith("orin"):
        assert not processes
    else:
        command = processes[0].kwargs["cmd"]
        options = dict(zip(command[4::2], command[5::2], strict=True))
        assert (
            options["--role"]
            == {
                "simulation_pi": "policy",
                "physical_orin": "foundation",
                "physical_pi": "both",
            }[scenario]
        )
        assert options["--policy-bundle"] == "/remote workspace/bundle"
        assert options["--feedback-timeout"] == "0.6"
        assert options["--host"] == "test-pi.local"
