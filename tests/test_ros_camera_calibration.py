"""Real ROS services, depth/TF geometry, stationary gate and save/load lifecycle."""

import json
import math
import time

import numpy as np
import pytest
import rclpy
from catbot_sim.camera_calibration import load_calibration, quaternion
from catbot_sim.camera_calibration_node import CameraCalibration
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster

from catbot_interfaces.msg import (
    ActuatorErrorState,
    ActuatorKinematicState,
    BodyErrorState,
    BodyKinematicState,
)

pytestmark = pytest.mark.ros


@pytest.mark.parametrize("measured_height", [False, True])
def test_four_foot_calibration_service_preview_save_and_motion_rejection(
    tmp_path, measured_height
):
    xml = (
        '<mujoco><worldbody><body name="quadruped">'
        + "".join(
            f'<body name="{leg}_shin"><geom name="{leg}_foot" type="sphere" pos="0 0 -0.3" size="0.02"/></body>'
            for leg in ("fl", "fr", "rl", "rr")
        )
        + "</body></worldbody></mujoco>"
    )
    (tmp_path / "model.xml").write_text(xml)
    names = [f"axis_{i}" for i in range(12)]
    (tmp_path / "mapping.json").write_text(json.dumps({"actuator_names": names}))
    output = tmp_path / "camera.json"
    rclpy.init()
    node = CameraCalibration(
        parameter_overrides=[
            Parameter(k, value=v)
            for k, v in {
                "run_directory": str(tmp_path),
                "mapping_file": str(tmp_path / "mapping.json"),
                "body_from_camera_wxyz": quaternion(np.deg2rad([178, 2, 0])).tolist(),
                "body_to_camera_xyz": [0.4, 0.0, 0.02],
                "camera_frame": "calibration/camera",
                "world_frame": "calibration/world",
                "camera_ground_height": 0.44 if measured_height else 0.0,
                "camera_height_frame": "calibration/height_point",
                "camera_calibration_output": str(output),
            }.items()
        ]
    )
    peer = rclpy.create_node("camera_calibration_test_peer")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(peer)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    specs = {
        "depth": (
            Image,
            "/zed/zed_node/depth/depth_registered",
            qos_profile_sensor_data,
        ),
        "intrinsics": (
            CameraInfo,
            "/zed/zed_node/depth/camera_info",
            qos_profile_sensor_data,
        ),
        "imu": (Imu, "/zed/zed_node/imu/data", qos_profile_sensor_data),
        "joints": (BodyKinematicState, "/body/state/position", qos_profile_sensor_data),
        "errors": (BodyErrorState, "/body/state/error", latched),
        "supervisor": (String, "/rpi/supervisor/status", latched),
    }
    publishers = {
        k: peer.create_publisher(cls, topic, qos)
        for k, (cls, topic, qos) in specs.items()
    }
    frames = StaticTransformBroadcaster(peer)
    transforms = []
    for i, (x, y) in enumerate(
        [(0.25, 0.15), (0.25, -0.15), (-0.25, 0.15), (-0.25, -0.15)]
    ):
        tf = TransformStamped()
        tf.header.frame_id = node.body_frame
        tf.child_frame_id = node.feet[i][0]
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = -0.03
        tf.transform.rotation.w = 1.0
        transforms.append(tf)
    tf = TransformStamped()
    tf.header.frame_id = "calibration/world"
    tf.child_frame_id = "calibration/camera"
    tf.transform.translation.x = 0.4
    tf.transform.translation.z = 0.4
    tf.transform.rotation.x = 1.0
    tf.transform.rotation.w = 0.0
    transforms.append(tf)
    tf = TransformStamped()
    tf.header.frame_id = "calibration/camera"
    tf.child_frame_id = "calibration/optical"
    tf.transform.translation.x = 0.01
    tf.transform.translation.z = -0.01
    tf.transform.rotation.y = math.sin(math.radians(5))
    tf.transform.rotation.w = math.cos(math.radians(5))
    transforms.append(tf)
    frames.sendTransform(transforms)
    tf = TransformStamped()
    tf.header.frame_id = "calibration/camera"
    tf.child_frame_id = "calibration/height_point"
    tf.transform.translation.z = -0.04
    tf.transform.rotation.w = 1.0
    transforms.append(tf)
    frames.sendTransform(transforms)
    state = {"policy": False, "velocity": 0.0, "encoder_fault": 0}
    received = {"mount": [], "heights": []}
    from geometry_msgs.msg import PoseStamped, Vector3Stamped

    _subscriptions = [
        peer.create_subscription(
            cls,
            "/camera/calibration/" + key,
            lambda m, k=key: received[k].append(m),
            latched,
        )
        for key, cls in [("mount", PoseStamped), ("heights", Vector3Stamped)]
    ]
    clients = {
        op: peer.create_client(Trigger, "/camera/calibration/" + op)
        for op in ("start", "save", "cancel")
    }

    def publish():
        stamp = peer.get_clock().now().to_msg()
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = "calibration/optical"
        image.width = image.height = 96
        image.step = 96 * 4
        image.encoding = "32FC1"
        # Known floor viewed through an independent tilted/translated factory
        # optical frame. Treating optical pixels as camera-link XYZ biases Z.
        _, pixels_x = np.mgrid[0:96, 0:96]
        depth = 0.41 / (
            math.cos(math.radians(10))
            - math.sin(math.radians(10)) * (pixels_x - 48) / 100
        )
        # A biased depth distance must not replace an explicit tape measurement.
        image.data = (
            (depth * (1.05 if measured_height else 1.0)).astype("<f4").tobytes()
        )
        info = CameraInfo()
        info.header = image.header
        info.width = info.height = 96
        info.p = [100.0, 0.0, 48.0, 0.0, 0.0, 100.0, 48.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        imu = Imu()
        imu.header = image.header
        joints = BodyKinematicState()
        joints.stamp = stamp
        errors = BodyErrorState()
        errors.stamp = stamp
        for name in names:
            a = ActuatorKinematicState()
            a.actuator_name = name
            a.valid = True
            a.velocity_degrees_per_second = state["velocity"]
            joints.actuators.append(a)
            e = ActuatorErrorState()
            e.actuator_name = name
            e.heartbeat_available = True
            e.encoder_error_flags = state["encoder_fault"]
            errors.actuators.append(e)
        for key, msg in {
            "depth": image,
            "intrinsics": info,
            "imu": imu,
            "joints": joints,
            "errors": errors,
            "supervisor": String(
                data=json.dumps(
                    {
                        "mode": "holding",
                        "script": "",
                        "orin_policy_active": state["policy"],
                    }
                )
            ),
        }.items():
            publishers[key].publish(msg)

    def pump(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            publish()
            until = time.monotonic() + 0.05
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=0.005)

    def call(operation):
        assert clients[operation].wait_for_service(timeout_sec=2)
        future = clients[operation].call_async(Trigger.Request())
        deadline = time.monotonic() + 3
        while not future.done() and time.monotonic() < deadline:
            pump(0.05)
        assert future.done()
        return future.result()

    try:
        pump(0.6)
        assert not call("save").success
        state["policy"] = True
        pump(0.2)
        assert not call("start").success
        state["policy"] = False
        state["encoder_fault"] = 256
        pump(0.2)
        assert not call("start").success
        state["encoder_fault"] = 0
        pump(0.2)
        assert call("start").success
        pump(3.7)
        assert node.state == "preview", node.detail
        assert node.result["body_to_camera_xyz"] == pytest.approx(
            [0.4, 0, 0.05], abs=1e-5
        )
        assert node.result["body_height_m"] == pytest.approx(0.35, abs=1e-5)
        assert node.result["camera_height_m"] == pytest.approx(0.4, abs=1e-5)
        assert node.result["ground_height_offset_at_capture"] == pytest.approx(
            0, abs=1e-5
        )
        assert node.result["height_source"] == (
            "measured" if measured_height else "depth"
        )
        if measured_height:
            assert node.result["depth_camera_height_m"] > 0.42
            assert node.result["measured_point_height_m"] == 0.44
        assert received["mount"] and received["heights"]
        assert received["mount"][-1].pose.position.z == pytest.approx(0.05, abs=1e-5)
        assert received["heights"][-1].vector.x == pytest.approx(0.4, abs=1e-5)
        assert received["heights"][-1].vector.y == pytest.approx(0.35, abs=1e-5)
        assert not output.exists()
        assert call("save").success
        assert load_calibration(output, xml)["body_to_camera_xyz"] == pytest.approx(
            [0.4, 0, 0.05], abs=1e-5
        )
        assert node.prior_xyz == [0.4, 0, 0.02]  # Save never applies a live transform.
        assert call("cancel").success
        assert not call("save").success
        assert call("start").success
        state["velocity"] = 2.0
        pump(0.2)
        assert node.state == "failed" and node.result is None
        state["velocity"] = 0.0
        pump(0.2)
        assert call("start").success
        node.started -= 16
        pump(0.2)
        assert node.state == "failed" and "timed out" in node.detail
        assert output.exists()
    finally:
        executor.shutdown()
        peer.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
