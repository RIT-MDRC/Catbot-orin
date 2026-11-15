"""Launch the real scenario, both ROS adapters, and stdlib fake workers."""

import json
import os
import signal
import subprocess
import time
from pathlib import Path

import pytest
import rclpy
from catbot_interfaces.msg import ActuatorPositionCommand
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

pytestmark = pytest.mark.ros
ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "external/Catbot/tests/fixtures"


def test_simulation_orin_entrypoint(tmp_path):
    fixture = json.loads((FIXTURES / "contract.json").read_text())
    mapping = tmp_path / "mapping.json"
    mapping.write_text(json.dumps(fixture["mapping"]))
    # Only the external workers are fake: launch and both adapter executables are real.
    (tmp_path / "pyproject.toml").touch()
    (tmp_path / "uv.lock").touch()
    (tmp_path / "run_worker.py").touch()
    (tmp_path / "src/catbot_rollout").mkdir(parents=True)
    (tmp_path / "src/catbot_rollout/worker.py").touch()
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    (bundle / "policy.json").write_text(json.dumps(fixture["metadata"]))
    command = [
        "ros2",
        "launch",
        "catbot_launcher",
        "simulation_orin.launch.py",
        "enable_camera:=false",
        "enable_foxglove:=false",
        "orin_device:=cpu",
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
    contracts, actions = [], []
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
                    contracts
                    and actions
                    and {"catbot_sim", "catbot_policy"} <= set(names)
                ):
                    break
                assert process.poll() is None, log.read_text()
            assert contracts and actions, log.read_text()
            assert {"catbot_sim", "catbot_policy"} <= set(node.get_node_names())
            contract = json.loads(contracts[-1].data)
            assert contract["policy_contract"] == fixture["metadata"]["policy_contract"]
            assert list(actions[-1].positions_degrees) == pytest.approx(
                fixture["degrees"]
            )
    finally:
        if process is not None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
        node.destroy_node()
        rclpy.shutdown()
