"""Bridge behavior with ROS boundaries doubled; real ROS has a separate CI job."""

import importlib.util
import io
import json
import math
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import pytest

ROOT = Path(__file__).resolve().parents[1]
GOLDEN = json.loads((ROOT / "external/Catbot/tests/fixtures/contract.json").read_text())


class Message:
    def __init__(self, **values):
        self.actuators = []
        self.__dict__.update(values)


class Node:
    def __init__(self, name):
        self.node_name = name
        self.publishers = {}
        self.subscriptions = {}
        self.timers = []
        self.logger = Mock()

    def declare_parameter(self, name, default):
        return SimpleNamespace(value=default)

    def create_publisher(self, message, topic, qos):
        self.publishers[topic] = Mock()
        return self.publishers[topic]

    def create_subscription(self, message, topic, callback, qos):
        self.subscriptions[topic] = callback

    def create_timer(self, interval, callback):
        self.timers.append((interval, callback))

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: "stamp"))


@pytest.fixture
def bridge_module(monkeypatch):
    def module(name, **members):
        value = ModuleType(name)
        value.__dict__.update(members)
        monkeypatch.setitem(sys.modules, name, value)
        return value

    module(
        "rclpy", init=Mock(), spin=Mock(), ok=Mock(return_value=True), shutdown=Mock()
    )
    module("rclpy.node", Node=Node)
    module(
        "rclpy.qos",
        QoSProfile=Message,
        DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=1),
        ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=1),
    )
    module(
        "catbot_interfaces.msg",
        **{
            name: Message
            for name in (
                "ActuatorKinematicState",
                "ActuatorPositionCommand",
                "BodyErrorState",
                "BodyKinematicState",
                "PolicyFeedback",
            )
        },
    )
    module("geometry_msgs.msg", Twist=Message)
    module("std_msgs.msg", Empty=Message, String=Message)
    module(
        "catbot_sim.physical",
        PhysicalInputs=Mock(return_value=Mock(timeout=0.25, estimate_contacts=True)),
    )
    module("catbot_sim.visualization", ScenePublisher=Mock())
    spec = importlib.util.spec_from_file_location(
        "catbot_sim.bridge", ROOT / "src/catbot_sim/catbot_sim/bridge.py"
    )
    bridge = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, "catbot_sim.bridge", bridge)
    spec.loader.exec_module(bridge)
    monkeypatch.setattr(bridge, "time", SimpleNamespace(monotonic=lambda: 100.0))
    return bridge


@pytest.fixture
def node(bridge_module, monkeypatch):
    from catbot_sim.config import BridgeConfig

    config = BridgeConfig(mapping=GOLDEN["mapping"])

    def read_config(param):
        return config

    monkeypatch.setattr(
        bridge_module, "BridgeConfig", SimpleNamespace(from_parameters=read_config)
    )
    monkeypatch.setattr(
        bridge_module,
        "WorkerLaunchSpec",
        SimpleNamespace(
            build=Mock(return_value=SimpleNamespace(command=["worker"], environment={}))
        ),
    )
    monkeypatch.setattr(
        bridge_module.subprocess,
        "Popen",
        Mock(return_value=Mock(stdin=io.StringIO(), stdout=io.StringIO())),
    )
    monkeypatch.setattr(bridge_module.threading, "Thread", Mock())
    instance = bridge_module.Bridge()
    instance.config_for_test = config
    return instance


def test_initialization_wires_simulation(node):
    assert node.physical is None
    assert node.subscriptions["/body/control/position"] == node.position
    callbacks = [callback for _, callback in node.timers]
    assert node.publish in callbacks
    assert node.publish_visualization in callbacks
    assert node.report_metrics in callbacks
    node.thread.start.assert_called_once()
    node.backend_pub.publish.assert_called_once()
    node.subscriptions["/body/control/script/home"](Message())
    node.logger.warning.assert_called_once()


def test_hardware_initialization(node, bridge_module):
    node.config_for_test.backend = "hardware"
    hardware = bridge_module.Bridge()
    assert hardware.physical is not None
    assert "/body/control/position" not in hardware.subscriptions
    bridge_module.ScenePublisher.assert_called_with(hardware, prefix="robot")


def test_positions_require_full_initial_command_and_update_atomically(node):
    node.position(Message(actuator_names=node.names[:1], positions_degrees=[0]))
    assert node.targets is None
    node.position(
        Message(actuator_names=node.names, positions_degrees=GOLDEN["degrees"])
    )
    assert node.targets == pytest.approx(GOLDEN["inverse_radians"])
    previous = node.targets.copy()
    node.position(Message(actuator_names=node.names[:1], positions_degrees=[0]))
    assert node.targets[1:] == previous[1:]
    previous = node.targets.copy()
    with pytest.raises(ValueError):
        node.position(
            Message(actuator_names=[node.names[0], "unknown"], positions_degrees=[1, 0])
        )
    assert node.targets == previous


@pytest.mark.parametrize("names,values", [([], []), (["a"], []), (["a", "a"], [0, 0])])
def test_malformed_positions(node, names, values):
    with pytest.raises(ValueError, match="Malformed"):
        node.position(Message(actuator_names=names, positions_degrees=values))


def test_velocity_validation(node):
    message = Message(linear=Message(x=1, y=2), angular=Message(z=3))
    node.velocity(message)
    assert node.command == [1, 2, 3]
    message.linear.x = float("nan")
    with pytest.raises(ValueError, match="Non-finite"):
        node.velocity(message)
    assert node.command == [1, 2, 3]


def test_latest_keeps_visualization_when_feedback_is_replaced(node):
    node.latest({"scene": "scene", "poses": "poses"})
    node.latest({"feedback": GOLDEN["feedback"]})
    assert node.messages.qsize() == 1
    node.publish_visualization()
    node.visualization.publish.assert_called_once_with("scene", "poses")


def test_hardware_visualization_requires_fresh_pose(node):
    node.backend = "hardware"
    node.physical = Mock(timeout=0.25)
    node.latest({"scene": "scene", "poses": "initial", "contract": {}})
    assert node.visual_poses is None
    node.publish_visualization()
    node.visualization.publish.assert_not_called()
    node.latest({"poses": "fresh"})
    node.last_visual_sample = 99
    node.publish_visualization()
    node.visualization.publish.assert_not_called()
    node.last_visual_sample = 100
    node.publish_visualization()
    node.visualization.publish.assert_called_once_with("scene", "fresh")


def test_publishes_contract_feedback_and_mapped_actuators(node):
    node.contract = {"test": True}
    node.latest({"feedback": GOLDEN["feedback"]})
    node.publish()
    feedback = node.feedback_pub.publish.call_args.args[0]
    assert feedback.joint_positions == GOLDEN["feedback"]["joint_positions"]
    state = node.state_pub.publish.call_args.args[0]
    assert len(state.actuators) == 12
    for i, actuator in enumerate(state.actuators):
        assert actuator.actuator_name == node.names[i]
        assert actuator.valid
        assert actuator.position_degrees == pytest.approx(
            math.degrees(feedback.joint_positions[i]) * node.mapping["signs"][i]
            + node.mapping["offsets_degrees"][i]
        )
    assert node.last_feedback == 100
    node.publish()
    node.contract_pub.publish.assert_called_once()
    node.feedback_pub.publish.assert_called_once()
    node.error_pub.publish.assert_called_once()


def test_hardware_drops_stale_sample(node):
    node.backend = "hardware"
    node.physical = Mock(timeout=0.25)
    node.latest({"feedback": GOLDEN["feedback"], "sample_time": 99})
    node.publish()
    node.feedback_pub.publish.assert_not_called()
    node.latest({"feedback": GOLDEN["feedback"], "sample_time": 99.9})
    node.publish()
    assert node.last_feedback == 99.9
    node.state_pub.publish.assert_not_called()


@pytest.mark.parametrize(
    "failure", ["worker", "startup", "commands", "feedback", "error"]
)
def test_publish_watchdogs(node, failure):
    if failure == "worker":
        node.request_started = 90
    elif failure == "startup":
        node.started = -100
    elif failure == "commands":
        node.contract = {}
        node.started = -100
    elif failure == "feedback":
        node.received = 99.9
        node.last_feedback = 90
    else:
        node.errors.put(ValueError("bad worker"))
    with pytest.raises(RuntimeError):
        node.publish()


@pytest.mark.parametrize(
    "backend,estimate", [("simulation", False), ("hardware", False), ("hardware", True)]
)
def test_metrics(node, backend, estimate):
    node.backend = backend
    if backend == "hardware":
        node.physical = Mock(
            timeout=0.25, estimate_contacts=estimate, blocked_reason="Waiting for depth"
        )
    node.metrics_started = 95
    node.sim_steps, node.sim_elapsed = 10, 0.2
    node.report_metrics()
    metrics = json.loads(node.metrics_pub.publish.call_args.args[0].data)
    assert metrics["physics_control_hz"] == 2
    assert metrics["real_time_factor"] == pytest.approx(0.04)
    assert metrics["feedback_status"] == "waiting_for_fresh_valid_inputs"
    assert metrics["feedback_detail"] == (
        "Waiting for depth" if backend == "hardware" else None
    )
    node.last_feedback = node.received = 100
    node.metrics_started = 95
    node.report_metrics()
    assert (
        json.loads(node.metrics_pub.publish.call_args.args[0].data)["command_age_ms"]
        == 0
    )
    assert (node.sim_steps, node.sim_elapsed) == (0, 0)


@pytest.mark.parametrize(
    "backend,targets",
    [("simulation", False), ("simulation", True), ("hardware", False)],
)
def test_worker_loop(node, backend, targets):
    node.backend = backend
    first = json.loads(json.dumps(GOLDEN["rollout"]))
    node.read_worker = Mock(side_effect=[first, {"feedback": GOLDEN["feedback"]}])
    node.stopping = Mock()
    node.stopping.wait.side_effect = [False, True]
    if targets:
        node.targets, node.received = [0.0] * 12, 100
    if backend == "hardware":
        node.physical = Mock()
        node.physical.snapshot.return_value = {
            "sample_time": 99.9,
            "command": [0, 0, 0],
        }
    node.run_worker()
    assert node.errors.empty()
    assert node.contract["mapping"] == node.mapping
    payload = node.messages.get_nowait()
    assert payload["feedback"] == GOLDEN["feedback"]
    if targets:
        assert json.loads(node.process.stdin.getvalue())["targets"] == node.targets
        assert node.sim_steps == 1
    if backend == "hardware":
        assert payload["sample_time"] == 99.9
        assert node.request_started is None


@pytest.mark.parametrize("failure", ["scene", "timestep", "commands", "read"])
def test_worker_failures_reach_error_queue(node, failure):
    first = json.loads(json.dumps(GOLDEN["rollout"]))
    if failure == "scene":
        del first["scene"]
    elif failure == "timestep":
        first["contract"]["policy_contract"]["control_timestep"] = 0
    elif failure == "commands":
        node.targets, node.received = [0.0] * 12, 90
    node.read_worker = (
        Mock(side_effect=RuntimeError("broken pipe"))
        if failure == "read"
        else Mock(return_value=first)
    )
    node.stopping = Mock()
    node.stopping.wait.side_effect = [False, True]
    node.run_worker()
    assert isinstance(node.errors.get_nowait(), (ValueError, RuntimeError))


@pytest.mark.parametrize(
    "failure", [None, KeyboardInterrupt(), ValueError("spin failed")]
)
def test_main_always_closes_and_shuts_down(bridge_module, monkeypatch, failure):
    node = Mock()
    monkeypatch.setattr(bridge_module, "Bridge", Mock(return_value=node))
    bridge_module.rclpy.spin.side_effect = failure
    if isinstance(failure, ValueError):
        with pytest.raises(ValueError):
            bridge_module.main()
    else:
        bridge_module.main()
    node.close.assert_called_once()
    node.destroy_node.assert_called_once()
    bridge_module.rclpy.shutdown.assert_called_once()


def test_hardware_preview_publishes_pose_without_policy_feedback(node):
    node.backend = "hardware"
    node.physical = Mock(timeout=0.25)
    node.physical.snapshot.return_value = {
        "sample_time": 99.9,
        "preview_only": True,
        "command": [0, 0, 0],
    }
    first = json.loads(json.dumps(GOLDEN["rollout"]))
    node.read_worker = Mock(
        side_effect=[first, {"poses": "preview", "feedback": GOLDEN["feedback"]}]
    )
    node.stopping = Mock()
    node.stopping.wait.side_effect = [False, True]
    node.run_worker()
    assert node.errors.empty()
    assert node.messages.empty()
    assert node.last_feedback is None
    assert "preview_only" not in json.loads(node.process.stdin.getvalue())
    node.publish_visualization()
    node.visualization.publish.assert_called_once_with(first["scene"], "preview")
