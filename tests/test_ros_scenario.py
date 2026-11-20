"""Launch the real scenario, both ROS adapters, and stdlib fake workers."""

import json
import os
import signal
import subprocess
import time
from pathlib import Path

import pytest
import rclpy
from catbot_interfaces.srv import GetSupervisorStatus, SupervisorJob
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu
from std_msgs.msg import String, UInt64

from catbot_interfaces.msg import (
    ActuatorKinematicState,
    ActuatorPositionCommand,
    BodyKinematicState,
    ClockProbe,
    ClockReply,
    OrinHeartbeat,
    OrinTelemetry,
    PolicyAction,
    PolicyActionAck,
    PolicyMetrics,
    PolicyReady,
)

pytestmark = pytest.mark.ros
ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "external/Catbot/tests/fixtures"


@pytest.mark.parametrize("scenario", ["simulation_orin", "physical_orin"])
def test_orin_entrypoint(tmp_path, scenario):
    simulation = scenario == "simulation_orin"
    fixture = json.loads((FIXTURES / "contract.json").read_text())
    mapping = tmp_path / "mapping.json"
    mapping.write_text(json.dumps(fixture["mapping"]))
    # Only the external workers are fake: launch and both adapter executables are real.
    (tmp_path / "pyproject.toml").touch()
    (tmp_path / "uv.lock").touch()
    (tmp_path / "run_worker.py").touch()
    (tmp_path / "config.json").write_text("{}")
    (tmp_path / "src/catbot_rollout").mkdir(parents=True)
    (tmp_path / "src/catbot_rollout/worker.py").touch()
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    (bundle / "policy.json").write_text(json.dumps(fixture["metadata"]))
    sensors = tmp_path / "sys/class/thermal/thermal_zone0"
    sensors.mkdir(parents=True)
    (sensors / "temp").write_text("50000")
    command = [
        "ros2",
        "launch",
        "catbot_launcher",
        f"{scenario}.launch.py",
        "enable_camera:=false",
        "enable_foxglove:=false",
        f"enable_diagnostics:={str(simulation).lower()}",
        "imu_topic:=/test/imu",
        "odom_topic:=/test/odom",
        "orin_device:=cpu",
        "supervisor_git_commit:=test-commit",
        f"supervisor_resource_sys_root:={tmp_path / 'sys'}",
        f"mapping_file:={mapping}",
        f"policy_bundle:={bundle}",
        f"rollout_repository:={tmp_path}",
        f"run_directory:={tmp_path}",
        f"policy_worker_project:={tmp_path}",
        f"uv_executable:={FIXTURES / 'fake_worker.py'}",
        f"policy_uv_executable:={FIXTURES / 'fake_worker.py'}",
        "startup_timeout:=15.0",
        "policy_startup_timeout:=10.0",
    ]
    log = tmp_path / "launch.log"
    rclpy.init()
    node = rclpy.create_node("scenario_test_probe")
    contracts, actions, diagnostics = [], [], []
    metrics, readiness = [], []
    heartbeats, telemetry = [], []
    node.create_subscription(
        OrinHeartbeat, "/orin/supervisor/heartbeat", heartbeats.append, 1
    )
    node.create_subscription(
        OrinTelemetry, "/orin/supervisor/telemetry", telemetry.append, 1
    )
    sequence = [0]
    standby_since = None
    saw_standby = False

    def probe():
        sequence[0] += 1
        probes.publish(UInt64(data=sequence[0]))

    def clock(message):
        stamp = node.get_clock().now().to_msg()
        replies.publish(
            ClockReply(
                id=message.id,
                origin_stamp=message.stamp,
                received_stamp=stamp,
                reply_stamp=node.get_clock().now().to_msg(),
            )
        )

    def accept(message):
        positions.publish(message.command)
        acknowledgements.publish(
            PolicyActionAck(
                session=message.session, sequence=message.sequence, forwarded=True
            )
        )

    if not simulation:
        # Only hardware scenarios have a Pi peer. The local simulation must
        # produce actions without leases, clock replies, jobs or forwarding.
        probes = node.create_publisher(UInt64, "/orin/supervisor/probe", 1)
        replies = node.create_publisher(ClockReply, "/rpi/supervisor/clock_reply", 1)
        acknowledgements = node.create_publisher(
            PolicyActionAck, "/rpi/supervisor/action_ack", 1
        )
        positions = node.create_publisher(
            ActuatorPositionCommand, "/body/control/position", 1
        )
        node.create_timer(0.1, probe)
        node.create_subscription(ClockProbe, "/orin/supervisor/clock_probe", clock, 1)
        node.create_subscription(PolicyAction, "/orin/policy/action", accept, 1)
        job = node.create_client(SupervisorJob, "/orin/supervisor/job")
        status = node.create_client(GetSupervisorStatus, "/orin/supervisor/get_status")
        imu = node.create_publisher(Imu, "/test/imu", 1)
        odom = node.create_publisher(Odometry, "/test/odom", 1)
        joints = node.create_publisher(BodyKinematicState, "/body/state/position", 1)

        def sensors():
            stamp = node.get_clock().now().to_msg()
            inertial = Imu()
            inertial.header.stamp = stamp
            imu.publish(inertial)
            pose = Odometry()
            pose.header.stamp = stamp
            pose.header.frame_id = "test/world"
            pose.pose.pose.orientation.w = 1.0
            odom.publish(pose)
            joints.publish(
                BodyKinematicState(
                    stamp=stamp,
                    actuators=[
                        ActuatorKinematicState(actuator_name=name, valid=True)
                        for name in fixture["mapping"]["actuator_names"]
                    ],
                )
            )

        node.create_timer(0.05, sensors)
    start_future = None
    node.create_subscription(PolicyMetrics, "/policy/metrics", metrics.append, 1)
    node.create_subscription(
        PolicyReady,
        "/orin/policy/ready",
        readiness.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    node.create_subscription(DiagnosticArray, "/diagnostics", diagnostics.append, 10)
    node.create_subscription(
        String,
        "/policy/contract",
        contracts.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    node.create_subscription(
        ActuatorPositionCommand, "/body/control/position", actions.append, 1
    )
    process = None
    try:
        with log.open("w") as output:
            process = subprocess.Popen(
                command, stdout=output, stderr=subprocess.STDOUT, start_new_session=True
            )
            deadline = time.monotonic() + 25
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                names = node.get_node_names()
                if (
                    not simulation
                    and start_future is None
                    and heartbeats
                    and heartbeats[-1].preflight_passed
                    and job.service_is_ready()
                ):
                    assert not actions and "catbot_policy" not in names
                    assert not heartbeats[-1].policy_active
                    assert not heartbeats[-1].degraded
                    if standby_since is None:
                        standby_since = time.monotonic()
                    if time.monotonic() - standby_since >= 0.3:
                        saw_standby = True
                        start_future = job.call_async(
                            SupervisorJob.Request(name="policy", operation="start")
                        )
                if start_future is not None and start_future.done():
                    assert start_future.result().success, start_future.result().message
                if (
                    contracts
                    and actions
                    and metrics
                    and readiness
                    and readiness[-1].ready
                    and {"catbot_sim", "catbot_policy"} <= set(names)
                    and (
                        not simulation
                        or (
                            diagnostics
                            and all(
                                s.level == DiagnosticStatus.OK
                                for s in diagnostics[-1].status
                            )
                        )
                    )
                ):
                    break
                assert process.poll() is None, log.read_text()
            assert contracts and actions, log.read_text()
            assert readiness[-1].ready and readiness[-1].verified
            assert readiness[-1].bundle_id == fixture["metadata"]["policy_sha256"]
            if simulation:
                assert "orin_supervisor" not in node.get_node_names()
                assert not heartbeats and not telemetry
            else:
                assert saw_standby, (
                    "Hardware launch did not wait for an explicit policy start request"
                )
                assert heartbeats[-1].policy_ready and heartbeats[-1].policy_verified
                assert heartbeats[-1].bundle_id == fixture["metadata"]["policy_sha256"]
                assert telemetry and telemetry[-1].scenario == scenario
                assert telemetry[-1].bundle_id == fixture["metadata"]["policy_sha256"]
                assert (
                    telemetry[-1].run_id and telemetry[-1].git_commit == "test-commit"
                )
                query = status.call_async(GetSupervisorStatus.Request(name="policy"))
                deadline = time.monotonic() + 3
                while not query.done() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.02)
                assert query.done() and query.result().success
                state = json.loads(query.result().status_json)
                assert state["version"] == 1 and state["policy"]["verified"]
            assert metrics and metrics[-1].published_hz > 0, log.read_text()
            assert metrics[-1].sample_count > 0
            assert metrics[-1].control_interval_ms == pytest.approx(
                fixture["metadata"]["policy_contract"]["control_timestep"] * 1000
            )
            assert {"catbot_sim", "catbot_policy"} <= set(node.get_node_names())
            if simulation:
                assert diagnostics, log.read_text()
                assert {s.name for s in diagnostics[-1].status} == {
                    "catbot/body/joints",
                    "catbot/policy/feedback",
                    "catbot/policy/output",
                }
                assert all(
                    s.level == DiagnosticStatus.OK for s in diagnostics[-1].status
                )
            contract = json.loads(contracts[-1].data)
            assert contract["policy_contract"] == fixture["metadata"]["policy_contract"]
            assert list(actions[-1].positions_degrees) == pytest.approx(
                fixture["degrees"]
            )
    finally:
        try:
            if process is not None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait(timeout=3)
        finally:
            node.destroy_node()
            rclpy.shutdown()
