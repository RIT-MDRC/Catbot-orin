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


def test_launch_declares_typed_defaults(monkeypatch):
    """Check declaration assembly; ROS executes the deferred callback in its CI job."""
    import sys
    from pathlib import Path
    from types import ModuleType
    from unittest.mock import Mock

    from catbot_launcher.scenarios import SCENARIOS, build_launch

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
    monkeypatch.delenv("CATBOT_LAUNCH_DEFAULTS", raising=False)
    assert build_launch("simulation_orin") is description.return_value
    assert declarations.call_args_list[0].args == ("scenario",)
    assert declarations.call_args_list[0].kwargs == {
        "default_value": "simulation_orin",
        "choices": list(SCENARIOS),
    }
    defaults = {
        call.args[0]: call.kwargs["default_value"]
        for call in declarations.call_args_list[1:]
    }
    assert defaults["worker_timeout"] == "2.0"
    assert defaults["enable_foxglove"] == "true"
    assert decode_setting([0, 0, 0], defaults["body_to_camera_xyz"]) == [0, 0, 0]
    assert callable(opaque.call_args.kwargs["function"])
