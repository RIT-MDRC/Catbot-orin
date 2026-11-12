"""ROS Humble adapter supervising a Python 3.14 simulator over local pipes."""

import json
import math
import os
import queue
import signal
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from catbot_interfaces.msg import ActuatorPositionCommand, PolicyFeedback
from catbot_sim.visualization import ScenePublisher


class Bridge(Node):
    def __init__(self):
        super().__init__("catbot_sim")

        def param(name, default):
            return self.declare_parameter(name, default).value

        repository = Path(str(param("rollout_repository", ""))).expanduser().resolve()
        if (
            not (repository / "pyproject.toml").is_file()
            or not (repository / "src/catbot_rollout/worker.py").is_file()
        ):
            raise ValueError(
                "rollout_repository must point to the initialized "
                "external/catbot_sim_rollout submodule"
            )
        run = Path(str(param("run_directory", ""))).expanduser().resolve()
        mapping_path = Path(str(param("mapping_file", ""))).expanduser()
        self.mapping = json.loads(mapping_path.read_text())
        self.names = self.mapping["actuator_names"]
        if (
            len(self.names) != 12
            or any(not isinstance(n, str) or not n.strip() for n in self.names)
            or len(set(self.names)) != 12
        ):
            raise ValueError("Expected 12 unique mapping names")
        for key in ("signs", "offsets_degrees", "minimum_degrees", "maximum_degrees"):
            values = self.mapping[key]
            if len(values) != 12 or not all(math.isfinite(v) for v in values):
                raise ValueError("Invalid mapping " + key)
        if any(v not in (-1, 1) for v in self.mapping["signs"]):
            raise ValueError("Mapping signs must be +/-1")
        if any(
            lo >= hi
            for lo, hi in zip(
                self.mapping["minimum_degrees"],
                self.mapping["maximum_degrees"],
                strict=True,
            )
        ):
            raise ValueError("Mapping minimum must be below maximum")
        self.timeout = float(param("command_timeout", 0.25))
        self.startup_timeout = float(param("startup_timeout", 120.0))
        if not all(
            math.isfinite(v) and v > 0 for v in (self.timeout, self.startup_timeout)
        ):
            raise ValueError("Timeouts must be positive and finite")
        self.command = [0.0] * 3
        self.targets = None
        self.received = None
        self.messages = queue.Queue(maxsize=1)
        self.errors = queue.Queue()
        self.stopping = threading.Event()
        self.lock = threading.Lock()
        self.feedback_pub = self.create_publisher(
            PolicyFeedback,
            "/policy/feedback",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.contract_pub = self.create_publisher(
            String,
            "/simulation/contract",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            ActuatorPositionCommand, "/simulation/control/position", self.position, 1
        )
        self.create_subscription(Twist, "/policy/cmd_vel", self.velocity, 1)
        self.visualization = ScenePublisher(self)
        self.visual_scene = None
        self.visual_poses = None
        self.create_timer(1 / 30, self.publish_visualization)
        self.process = subprocess.Popen(
            [
                str(param("uv_executable", "uv")),
                "run",
                "--locked",
                "--project",
                str(repository),
                "python",
                "-m",
                "catbot_rollout.worker",
                "--run-directory",
                str(run),
                "--device",
                str(param("device", "cpu")),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        self.thread = threading.Thread(target=self.run_worker, daemon=True)
        self.started = time.monotonic()
        self.last_feedback = None
        self.sim_steps = 0
        self.sim_elapsed = 0.0
        self.metrics_started = self.started
        self.metrics_pub = self.create_publisher(String, "/simulation/metrics", 1)
        self.create_timer(5.0, self.report_metrics)
        self.create_timer(0.002, self.publish)
        self.thread.start()

    def position(self, message):
        names, values = list(message.actuator_names), list(message.positions_degrees)
        if not names or len(names) != len(values) or len(set(names)) != len(names):
            raise ValueError("Malformed actuator command")
        with self.lock:
            # First target must establish all joints; subsequent commands may be sparse.
            if self.targets is None and set(names) != set(self.names):
                return
            targets = list(self.targets) if self.targets is not None else [0.0] * 12
            for name, value in zip(names, values, strict=True):
                i = self.names.index(name)
                if (
                    not math.isfinite(value)
                    or not self.mapping["minimum_degrees"][i]
                    <= value
                    <= self.mapping["maximum_degrees"][i]
                ):
                    raise ValueError("Invalid actuator target")
                targets[i] = math.radians(
                    (value - self.mapping["offsets_degrees"][i])
                    / self.mapping["signs"][i]
                )
            self.targets, self.received = targets, time.monotonic()

    def velocity(self, message):
        values = [message.linear.x, message.linear.y, message.angular.z]
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Non-finite velocity command")
        with self.lock:
            self.command = values

    def latest(self, payload):
        # Keep visualization independently: physics feedback uses a latest-only queue.
        with self.lock:
            if "scene" in payload:
                self.visual_scene = payload["scene"]
            if "poses" in payload:
                self.visual_poses = payload["poses"]
        try:
            self.messages.get_nowait()
        except queue.Empty:
            pass
        self.messages.put_nowait(payload)

    def publish_visualization(self):
        with self.lock:
            scene, poses = self.visual_scene, self.visual_poses
        if scene is not None and poses is not None:
            self.visualization.publish(scene, poses)

    def read_worker(self):
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError("Simulation worker exited")
        return json.loads(line)

    def run_worker(self):
        try:
            first = self.read_worker()
            if "scene" not in first or "poses" not in first:
                raise ValueError(
                    "Update the rollout submodule: worker lacks visualization data"
                )
            contract = first["contract"]
            contract["mapping"] = self.mapping
            self.contract = contract
            dt = float(contract["policy_contract"]["control_timestep"])
            if not math.isfinite(dt) or dt <= 0:
                raise ValueError("Invalid simulation timestep")
            self.latest(first)
            deadline = time.monotonic()
            while not self.stopping.wait(max(0, deadline - time.monotonic())):
                with self.lock:
                    targets, received, command = (
                        self.targets,
                        self.received,
                        list(self.command),
                    )
                if targets is None:
                    # Keep the initial pose until the Pi starts responding.
                    self.latest({"feedback": first["feedback"]})
                else:
                    if time.monotonic() - received > self.timeout:
                        raise RuntimeError(
                            "Pi commands timed out; restart the entire launch"
                        )
                    request = {"version": 1, "targets": targets, "command": command}
                    self.process.stdin.write(
                        json.dumps(request, allow_nan=False) + "\n"
                    )
                    self.process.stdin.flush()
                    self.latest(self.read_worker())
                    with self.lock:
                        self.sim_steps += 1
                        self.sim_elapsed += dt
                # No catch-up bursts if simulation is slower than real time.
                deadline = max(deadline + dt, time.monotonic())
        except (OSError, ValueError, KeyError, RuntimeError) as error:
            self.errors.put(error)

    def report_metrics(self):
        now = time.monotonic()
        elapsed = now - self.metrics_started
        with self.lock:
            steps, simulated = self.sim_steps, self.sim_elapsed
            self.sim_steps, self.sim_elapsed = 0, 0.0
        self.metrics_pub.publish(
            String(
                data=json.dumps(
                    {
                        "window_seconds": elapsed,
                        "physics_control_hz": steps / elapsed,
                        "real_time_factor": simulated / elapsed,
                        "command_age_ms": None
                        if self.received is None
                        else (now - self.received) * 1000,
                    }
                )
            )
        )
        self.metrics_started = now

    def publish(self):
        if not self.errors.empty():
            raise RuntimeError(str(self.errors.get_nowait()))
        now = time.monotonic()
        if (
            self.received is None or self.last_feedback is None
        ) and now - self.started > self.startup_timeout:
            raise RuntimeError(
                "Timed out waiting for simulation initialization and Pi commands"
            )
        if self.last_feedback is not None and now - self.last_feedback > max(
            2.0, self.timeout
        ):
            raise RuntimeError("Simulation worker stalled")
        if hasattr(self, "contract") and not getattr(self, "contract_sent", False):
            self.contract_pub.publish(String(data=json.dumps(self.contract)))
            self.contract_sent = True
        try:
            payload = self.messages.get_nowait()
        except queue.Empty:
            return
        message = PolicyFeedback()
        message.stamp = self.get_clock().now().to_msg()
        for name, value in payload["feedback"].items():
            setattr(message, name, value)
        self.feedback_pub.publish(message)
        self.last_feedback = now

    def close(self):
        self.stopping.set()
        try:
            os.killpg(self.process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(self.process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.process.wait()
        # uv may exit before its worker; also clean up remaining group members.
        try:
            os.killpg(self.process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.thread.join(timeout=1)
        self.process.stdin.close()
        self.process.stdout.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Bridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
