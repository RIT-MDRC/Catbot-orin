"""Launch the real scenario, both ROS adapters, and stdlib fake workers."""

import json
import math
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import String, UInt64
from tf2_ros import StaticTransformBroadcaster

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
from catbot_interfaces.srv import GetSupervisorStatus, SupervisorJob

pytestmark = pytest.mark.ros
ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "external/Catbot/tests/fixtures"


def test_colcon_discovers_one_copy_of_each_shared_package(tmp_path):
    output = subprocess.check_output(
        ["colcon", "--log-base", str(tmp_path / "colcon-log"), "list"],
        cwd=ROOT,
        text=True,
    )
    packages = {}
    for line in output.splitlines():
        name, path, _ = line.split("\t")
        packages.setdefault(name, []).append(path)
    for name in (
        "catbot_interfaces",
        "catbot_policy",
        "foundation",
        "catbot_supervisor",
    ):
        assert len(packages.get(name, [])) == 1, output
        source = ROOT / packages[name][0]
        assert source.absolute() == ROOT / "src" / name
        assert source.resolve() == ROOT / "external/Catbot/src" / name


@pytest.mark.parametrize("camera_aligned_odom", [False, True])
@pytest.mark.parametrize("measured_camera_height", [False, True])
def test_vertical_camera_mount_aligns_body_cloud_and_factory_imu(
    camera_aligned_odom, measured_camera_height
):
    from catbot_sim.physical import PhysicalInputs, inverse, multiply
    from catbot_sim.visualization import ScenePublisher

    # Right lens on top, pitched down 30 degrees; explicit inputs, independent
    # of deployment tuning. Native cloud and factory IMU frames stay camera-owned.
    mount = [
        0.6830127018922194,
        -0.6830127018922193,
        0.1830127018922193,
        0.1830127018922193,
    ]
    c, s = math.cos(math.pi / 6), math.sin(math.pi / 6)
    mapping = json.loads((FIXTURES / "contract.json").read_text())["mapping"]
    rclpy.init()
    node = rclpy.create_node(
        "camera_mount_test",
        parameter_overrides=[
            Parameter("body_from_camera_wxyz", value=mount),
            Parameter("body_to_camera_xyz", value=[0.3, 0.0, -0.03]),
            Parameter("imu_mount_from_tf", value=True),
            Parameter("world_frame", value="mount/map"),
            Parameter("ground_height_offset", value=0.35),
            Parameter(
                "camera_ground_height", value=0.43 if measured_camera_height else 0.0
            ),
            Parameter("camera_height_frame", value="mount/cloud"),
            Parameter("imu_includes_gravity", value=False),
        ],
    )
    node.lock = threading.Lock()
    try:
        inputs = PhysicalInputs(node, mapping)
        native_tf = StaticTransformBroadcaster(node)

        def transform(parent, child, position, quaternion):
            message = TransformStamped()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id, message.child_frame_id = parent, child
            (
                message.transform.translation.x,
                message.transform.translation.y,
                message.transform.translation.z,
            ) = position
            (
                message.transform.rotation.w,
                message.transform.rotation.x,
                message.transform.rotation.y,
                message.transform.rotation.z,
            ) = quaternion
            return message

        # ZED's local odom starts at the tilted camera: identity odom->camera
        # while map->odom supplies the gravity alignment. Also exercise an
        # already gravity-aligned odom source.
        odom_rotation = mount if camera_aligned_odom else [1.0, 0.0, 0.0, 0.0]
        camera_rotation = [1.0, 0.0, 0.0, 0.0] if camera_aligned_odom else mount
        camera_position = (
            [0.9 * c + 0.13 * s, -0.9 * s + 0.13 * c, 2.2]
            if camera_aligned_odom
            else [0.9, 2.2, -0.13]
        )
        world_odom = transform(
            "mount/map", "mount/odom", [0.4, -0.2, 0.1], odom_rotation
        )
        camera = transform(
            "mount/odom", "mount/camera", camera_position, camera_rotation
        )
        cloud = transform(
            "mount/camera", "mount/cloud", [0.0, 0.04, 0.0], [1.0, 0.0, 0.0, 0.0]
        )
        native_tf.sendTransform([camera, cloud])

        def until(predicate):
            deadline = time.monotonic() + 5.0
            while not predicate() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
            assert predicate()

        def sample(orientation=None):
            stamp = node.get_clock().now().to_msg()
            imu, odom = Imu(), Odometry()
            imu.header.stamp = odom.header.stamp = stamp
            imu.header.frame_id = "mount/imu"
            # Factory IMU is rotated 90 degrees around camera Z. These sensor
            # values describe body angular velocity [0,0,1], not camera rates.
            imu.angular_velocity.x, imu.angular_velocity.y = -c, s
            odom.header.frame_id, odom.child_frame_id = "mount/odom", "mount/camera"
            (
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                odom.pose.pose.position.z,
            ) = camera_position
            (
                odom.pose.pose.orientation.w,
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
            ) = camera_rotation if orientation is None else orientation
            (
                odom.twist.twist.linear.x,
                odom.twist.twist.linear.y,
                odom.twist.twist.linear.z,
            ) = [0.2 * c, -0.2 * s, 0.6]
            joints = BodyKinematicState(
                stamp=stamp,
                actuators=[
                    ActuatorKinematicState(actuator_name=name, valid=True)
                    for name in mapping["actuator_names"]
                ],
            )
            for name, message in (("imu", imu), ("odom", odom), ("joints", joints)):
                inputs.receive(name, message)
            return inputs.snapshot([0.0, 0.0, 0.0])

        until(
            lambda: inputs.tf_buffer.can_transform(
                "mount/camera", "mount/cloud", Time()
            )
        )
        assert (
            sample() is None
        )  # Missing gravity alignment must not silently use identity.
        native_tf.sendTransform([camera, cloud, world_odom])
        until(lambda: inputs.tf_buffer.can_transform("mount/map", "mount/odom", Time()))
        if measured_camera_height:
            inputs.camera_height_frame = "mount/missing_center"
            assert sample() is None
            assert inputs.odom_frame is None
            inputs.camera_height_frame = "mount/cloud"
        assert sample() is None  # Factory IMU calibration is still required.
        h = math.sqrt(0.5)
        imu_tf = transform(
            "mount/camera", "mount/imu", [0.0, 0.0, 0.0], [h, 0.0, 0.0, h]
        )
        native_tf.sendTransform([camera, cloud, world_odom, imu_tf])
        until(
            lambda: inputs.tf_buffer.can_transform("mount/camera", "mount/imu", Time())
        )
        state = sample()
        assert state is not None
        expected_height = 0.43 + 0.03 + 0.04 * c if measured_camera_height else 0.35
        assert state["qpos"][:7] == pytest.approx(
            [1.0, 2.0, expected_height, 1.0, 0.0, 0.0, 0.0]
        )
        assert state["qvel"][:6] == pytest.approx([0.2, 0.3, 0.0, 0.0, 0.0, 1.0])
        scene = {
            "bodies": [
                {"name": "world", "parent": 0},
                {"name": "quadruped", "parent": 0},
            ],
            "geoms": [],
        }
        display = ScenePublisher(node, prefix="robot")
        display.publish(
            scene,
            [
                {
                    "body": 1,
                    "position": state["qpos"][:3],
                    "quaternion": state["qpos"][3:7],
                }
            ],
        )
        body = "robot/body_1_quadruped"
        until(lambda: inputs.tf_buffer.can_transform(body, "mount/cloud", Time()))
        result = inputs.tf_buffer.lookup_transform(
            body, "mount/cloud", Time()
        ).transform
        assert [
            result.translation.x,
            result.translation.y,
            result.translation.z,
        ] == pytest.approx([0.3 - 0.04 * s, 0.0, -0.03 - 0.04 * c])
        assert [
            result.rotation.w,
            result.rotation.x,
            result.rotation.y,
            result.rotation.z,
        ] == pytest.approx(mount)
        # Correcting the world frame must still preserve real body roll.
        body_roll = [math.cos(math.radians(10)), math.sin(math.radians(10)), 0.0, 0.0]
        tilted_camera = multiply(inverse(odom_rotation), multiply(body_roll, mount))
        tilted = sample(tilted_camera)
        assert tilted is not None
        assert tilted["qpos"][3:7] == pytest.approx(body_roll)
        frame = inputs.tf_buffer.lookup_transform(
            "mount/map", "robot/world", Time()
        ).transform
        assert [
            frame.rotation.w,
            frame.rotation.x,
            frame.rotation.y,
            frame.rotation.z,
        ] == pytest.approx([1.0, 0.0, 0.0, 0.0])
        assert frame.translation.z == pytest.approx(-expected_height)
        if measured_camera_height:
            center = inputs.tf_buffer.lookup_transform(
                "robot/world", "mount/cloud", Time()
            ).transform
            assert center.translation.z == pytest.approx(0.43)
            # Measured height establishes the floor once; it must not pin the
            # body at a constant height as odometry subsequently moves upward.
            from catbot_sim.physical import rotate

            upward = rotate(inverse(odom_rotation), [0.0, 0.0, 0.1])
            camera_position[:] = [a + b for a, b in zip(camera_position, upward)]
            moved = sample()
            assert moved["qpos"][2] == pytest.approx(expected_height + 0.1)
            assert inputs.height == pytest.approx(expected_height)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_depth_floor_updates_model_tf_and_feedback_and_freezes_during_policy():
    import numpy as np
    from catbot_sim.physical import PhysicalInputs
    from sensor_msgs.msg import CameraInfo, Image

    mapping = json.loads((FIXTURES / "contract.json").read_text())["mapping"]
    rclpy.init()
    node = rclpy.create_node(
        "depth_floor_test",
        parameter_overrides=[
            Parameter("estimate_ground_from_depth", value=True),
            Parameter("world_frame", value="depth/map"),
            Parameter("body_from_camera_wxyz", value=[0.0, 1.0, 0.0, 0.0]),
            Parameter("body_to_camera_xyz", value=[0.1, 0.0, 0.05]),
            Parameter("imu_includes_gravity", value=False),
        ],
    )
    node.lock = threading.Lock()
    inputs = PhysicalInputs(node, mapping)
    broadcaster = StaticTransformBroadcaster(node)

    def spin_until(predicate):
        deadline = time.monotonic() + 3
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
        assert predicate()

    def sample(camera_z, distance, active=False, include_depth=True):
        tf = TransformStamped()
        tf.header.stamp = node.get_clock().now().to_msg()
        tf.header.frame_id = "depth/map"
        tf.child_frame_id = "depth/optical"
        tf.transform.translation.x = 0.1
        tf.transform.translation.z = camera_z
        tf.transform.rotation.x = 1.0
        tf.transform.rotation.w = 0.0
        broadcaster.sendTransform(tf)
        spin_until(
            lambda: inputs.tf_buffer.can_transform("depth/map", "depth/optical", Time())
        )
        # Wait for the replacement static pose, not just the existing frame.
        spin_until(
            lambda: (
                abs(
                    inputs.tf_buffer.lookup_transform(
                        "depth/map", "depth/optical", Time()
                    ).transform.translation.z
                    - camera_z
                )
                < 1e-6
            )
        )
        stamp = node.get_clock().now().to_msg()
        imu = Imu()
        imu.header.stamp = stamp
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "depth/map"
        odom.child_frame_id = "depth/optical"
        odom.pose.pose.position.x = 0.1
        odom.pose.pose.position.z = camera_z
        odom.pose.pose.orientation.x = 1.0
        odom.pose.pose.orientation.w = 0.0
        joints = BodyKinematicState(
            stamp=stamp,
            actuators=[
                ActuatorKinematicState(actuator_name=n, valid=True)
                for n in mapping["actuator_names"]
            ],
        )
        for key, msg in (("imu", imu), ("odom", odom), ("joints", joints)):
            inputs.receive(key, msg)
        inputs.receive(
            "supervisor",
            String(
                data=json.dumps(
                    {
                        "mode": "running" if active else "holding",
                        "orin_policy_active": active,
                        "script": "",
                    }
                )
            ),
        )
        if include_depth:
            image = Image()
            image.header.stamp = stamp
            image.header.frame_id = "depth/optical"
            image.width = image.height = 128
            image.step = 512
            image.encoding = "32FC1"
            image.data = np.full((128, 128), distance, dtype="<f4").tobytes()
            info = CameraInfo()
            info.header = image.header
            info.width = info.height = 128
            info.p = [100.0, 0.0, 64.0, 0.0, 0.0, 100.0, 64.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            inputs.receive("depth", image)
            inputs.receive("intrinsics", info)
        return inputs.snapshot([0.0, 0.0, 0.0])

    def floor_z():
        spin_until(
            lambda: inputs.tf_buffer.can_transform("depth/map", "robot/world", Time())
        )
        spin_until(
            lambda: (
                abs(
                    inputs.tf_buffer.lookup_transform(
                        "depth/map", "robot/world", Time()
                    ).transform.translation.z
                    + inputs.height
                )
                < 1e-6
            )
        )
        return inputs.tf_buffer.lookup_transform(
            "depth/map", "robot/world", Time()
        ).transform.translation.z

    try:
        assert sample(0.13, 0.43, include_depth=False) is None
        time.sleep(0.55)
        first = sample(0.13, 0.43)
        assert first is not None
        assert first["qpos"][2] == pytest.approx(0.38, abs=1e-6)
        assert floor_z() == pytest.approx(-0.30, abs=1e-6)
        time.sleep(0.55)
        shifted = sample(-0.07, 0.43)
        assert shifted["qpos"][2] == pytest.approx(0.38, abs=1e-6)
        assert floor_z() == pytest.approx(-0.50, abs=1e-6)
        time.sleep(0.55)
        running = sample(-0.17, 0.43, active=True)
        assert running["qpos"][2] == pytest.approx(0.28, abs=1e-6)
        assert floor_z() == pytest.approx(-0.50, abs=1e-6)
        time.sleep(0.55)
        stopped = sample(-0.17, 0.43)
        assert stopped["qpos"][2] == pytest.approx(0.38, abs=1e-6)
        assert floor_z() == pytest.approx(-0.60, abs=1e-6)
        time.sleep(0.55)
        invalid = sample(-0.17, float("nan"))
        assert invalid["qpos"][2] == pytest.approx(0.38, abs=1e-6)
        assert floor_z() == pytest.approx(-0.60, abs=1e-6)
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    ("scenario", "start_pi"),
    [("simulation_orin", False), ("physical_orin", False), ("physical_orin", True)],
)
def test_orin_entrypoint(tmp_path, scenario, start_pi):
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
        # The fake worker has no saved robot geometry; calibration has its own
        # real depth/TF/service integration test.
        "enable_camera_calibration:=false",
        "enable_foxglove:=false",
        f"enable_diagnostics:={str(simulation).lower()}",
        "imu_topic:=/test/imu",
        "odom_topic:=/test/odom",
        "world_frame:=test/world",
        "imu_mount_from_tf:=false",
        "body_from_camera_wxyz:=[1.0,0.0,0.0,0.0]",
        "body_from_imu_wxyz:=[1.0,0.0,0.0,0.0]",
        "body_to_camera_xyz:=[0.0,0.0,0.0]",
        "camera_ground_height:=0.0",
        "estimate_ground_from_depth:=false",
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
    if scenario == "physical_orin" and not start_pi:
        command.append("start_pi:=false")
    # Exercise the real remote process/lease without connecting to hardware.
    ssh_marker = tmp_path / "ssh.json"
    bin_directory = tmp_path / "bin"
    bin_directory.mkdir()
    ssh = bin_directory / "ssh"
    ssh.write_text(
        "#!/usr/bin/python3\n"
        "import json, signal, sys\n"
        "from pathlib import Path\n"
        f"Path({str(ssh_marker)!r}).write_text(json.dumps(sys.argv))\n"
        "signal.signal(signal.SIGINT, lambda *_: sys.exit(0))\n"
        "signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))\n"
        "for line in sys.stdin:\n    pass\n"
    )
    ssh.chmod(0o755)
    launch_environment = {
        **os.environ,
        "PATH": str(bin_directory) + ":" + os.environ["PATH"],
    }
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
                command,
                stdout=output,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                env=launch_environment,
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
            assert ssh_marker.exists() is start_pi, log.read_text()
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
