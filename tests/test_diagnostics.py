"""Health transitions and ROS serialization without requiring ROS locally."""

import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import pytest
from catbot_launcher.health import (
    ERROR,
    OK,
    WARN,
    StreamHealth,
    image_status,
    joint_status,
    motor_status,
)
from catbot_launcher.zed_watchdog import Health


def stamped(second=1, **kwargs):
    return SimpleNamespace(stamp=SimpleNamespace(sec=second, nanosec=0), **kwargs)


def motor(**overrides):
    values = {
        "actuator_name": "fl_knee",
        "axis_id": 3,
        "axis_state": 8,
        "heartbeat_available": True,
        "ready": True,
    }
    for group in ("axis", "motor", "encoder", "sensorless", "controller"):
        values[group + "_error_flags"] = 0
        values[group + "_error_messages"] = []
    return SimpleNamespace(**(values | overrides))


def test_stream_start_failure_stall_and_recovery():
    health = StreamHealth(0, timeout=2, startup_timeout=5)
    assert health.status(0)[0] == WARN
    assert health.status(5)[0] == ERROR
    health.observe(stamped(), 6)
    assert health.status(6)[0] == OK
    assert "Stream stopped" in health.status(8)[1]
    health.observe(stamped(), 8)
    assert "Timestamp stopped" in health.status(8)[1]
    health.observe(stamped(2), 9)
    assert health.status(9)[0] == OK
    health.observe(stamped(0), 10)
    assert "zero timestamp" in health.status(10)[1]


def test_unstamped_commands_use_arrival_time():
    health = StreamHealth(0)
    health.observe(SimpleNamespace(), 2)
    assert health.status(3)[0] == OK
    assert health.status(4)[0] == ERROR


def test_camera_wait_failure_suspicion_and_recovery():
    health = Health(0)
    assert image_status(health, 1)[0] == WARN
    assert image_status(health, 60) == (ERROR, "no first image")
    health.last_frame, health.stamp_changed = 61, 61
    health.bad_since = 61
    assert image_status(health, 62)[0] == WARN
    assert image_status(health, 66)[0] == ERROR
    health.last_frame, health.stamp_changed, health.bad_since = 67, 67, None
    assert image_status(health, 67)[0] == OK
    assert image_status(health, 77) == (ERROR, "image stream stopped")


@pytest.mark.parametrize(
    "actuators,level",
    [
        ([], ERROR),
        ([SimpleNamespace(actuator_name="a", valid=False)], ERROR),
        ([SimpleNamespace(actuator_name="a", valid=True)], OK),
    ],
)
def test_joint_validity(actuators, level):
    assert joint_status(SimpleNamespace(actuators=actuators))[0] == level


@pytest.mark.parametrize(
    "overrides,level,phrase",
    [
        ({}, OK, "ready"),
        ({"ready": False}, WARN, "idle/not ready"),
        ({"heartbeat_available": False}, ERROR, "Missing heartbeat"),
        (
            {"axis_error_flags": 16, "axis_error_messages": ["INVALID_STATE"]},
            ERROR,
            "INVALID_STATE",
        ),
        ({"encoder_error_flags": 8}, ERROR, "0x8"),
    ],
)
def test_motor_faults(overrides, level, phrase):
    actual, message, values = motor_status(
        SimpleNamespace(actuators=[motor(**overrides)])
    )
    assert actual == level and phrase in message
    assert values["fl_knee/axis_id"] == 3


def test_empty_motor_roster_is_not_healthy():
    assert motor_status(SimpleNamespace(actuators=[]))[0] == ERROR


@pytest.fixture
def adapter(monkeypatch):
    class Message:
        def __init__(self, **values):
            self.header = SimpleNamespace(stamp=None)
            self.__dict__.update(values)

    for name, members in {
        "diagnostic_msgs.msg": {
            "DiagnosticArray": Message,
            "DiagnosticStatus": Message,
            "KeyValue": Message,
        },
        "nav_msgs.msg": {"Odometry": Message},
        "sensor_msgs.msg": {"Image": Message, "Imu": Message},
        "rclpy.qos": {
            "DurabilityPolicy": SimpleNamespace(TRANSIENT_LOCAL=1),
            "QoSProfile": Mock(),
            "qos_profile_sensor_data": object(),
        },
        "rclpy.clock": {"Clock": Mock(), "ClockType": SimpleNamespace(STEADY_TIME=1)},
        "catbot_interfaces.msg": {
            name: Message
            for name in (
                "ActuatorPositionCommand",
                "BodyErrorState",
                "BodyKinematicState",
                "PolicyFeedback",
            )
        },
    }.items():
        module = ModuleType(name)
        module.__dict__.update(members)
        monkeypatch.setitem(sys.modules, name, module)
    path = (
        Path(__file__).resolve().parents[1]
        / "src/catbot_launcher/catbot_launcher/diagnostics.py"
    )
    spec = importlib.util.spec_from_file_location("diagnostics_test_adapter", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_monitor(adapter, **parameters):
    node = Mock()
    node.declare_parameter.side_effect = lambda name, default: SimpleNamespace(
        value=parameters.get(name, default)
    )
    node.get_clock.return_value.now.return_value.to_msg.return_value = "ros stamp"
    monitor = adapter.Diagnostics(node)
    return node, monitor


def statuses(node):
    return {
        s.name: s
        for s in node.create_publisher.return_value.publish.call_args.args[0].status
    }


def test_camera_names_topics_and_serialization(adapter):
    node, _monitor = make_monitor(
        adapter, camera_name="front", monitor_odometry=True, imu_topic="/custom/imu"
    )
    topics = {call.args[1] for call in node.create_subscription.call_args_list}
    assert topics == {
        "/front/zed_node/rgb/image_rect_color",
        "/custom/imu",
        "/front/zed_node/odom",
    }
    status = statuses(node)["catbot/zed/image"]
    assert status.level == WARN and status.hardware_id
    assert {v.key: v.value for v in status.values}[
        "topic"
    ] == "/front/zed_node/rgb/image_rect_color"
    assert node.create_publisher.call_args.args[1] == "/diagnostics"
    assert "clock" in node.create_timer.call_args.kwargs


def test_simulation_does_not_expect_camera_or_motors(adapter):
    node, _ = make_monitor(
        adapter, monitor_camera=False, monitor_robot=True, monitor_motors=False
    )
    assert set(statuses(node)) == {
        "catbot/body/joints",
        "catbot/policy/feedback",
        "catbot/policy/output",
    }


@pytest.mark.parametrize("name", ["zed/imu", "zed/odometry"])
def test_unavailable_camera_sensor_then_recovery(adapter, name):
    node, monitor = make_monitor(adapter, monitor_odometry=True)
    covariance = [-1.0] + [0.0] * 35
    message = stamped(
        angular_velocity_covariance=covariance,
        pose=SimpleNamespace(covariance=covariance),
        twist=SimpleNamespace(covariance=covariance),
    )
    monitor.streams[name][1].observe(message, adapter.time.monotonic())
    monitor.publish()
    assert statuses(node)["catbot/" + name].level == ERROR
    covariance[0] = 0.0
    monitor.publish()
    assert statuses(node)["catbot/" + name].level == OK


def test_latched_motor_faults_survive_and_clear(adapter):
    node, monitor = make_monitor(adapter, monitor_camera=False, monitor_robot=True)
    health = monitor.streams["body/joints"][1]
    health.observe(
        stamped(actuators=[SimpleNamespace(valid=True, actuator_name="fl_knee")]),
        adapter.time.monotonic(),
    )
    monitor.receive_errors(SimpleNamespace(actuators=[motor(axis_error_flags=16)]))
    monitor.publish()
    assert statuses(node)["catbot/body/motors"].level == ERROR
    monitor.receive_errors(SimpleNamespace(actuators=[motor()]))
    monitor.publish()
    assert statuses(node)["catbot/body/motors"].level == OK
    health.last_received -= health.timeout + 1
    monitor.publish()
    assert statuses(node)["catbot/body/motors"].level == ERROR


def test_missing_streams_become_errors(adapter, monkeypatch):
    node, monitor = make_monitor(adapter, monitor_robot=True)
    monkeypatch.setattr(adapter.time, "monotonic", lambda: monitor.started + 61)
    monitor.publish()
    assert all(s.level == ERROR for s in statuses(node).values())


@pytest.mark.parametrize(
    "name,value",
    [
        ("stream_timeout", 0),
        ("startup_timeout", float("nan")),
        ("frame_timeout", -1),
        ("green_threshold", 2),
    ],
)
def test_invalid_diagnostics_settings(adapter, name, value):
    with pytest.raises(ValueError):
        make_monitor(adapter, **{name: value})
