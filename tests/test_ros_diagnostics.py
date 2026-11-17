"""Exercise /diagnostics over real ROS, including failure and recovery."""

import time

import pytest
import rclpy
from catbot_launcher.diagnostics import Diagnostics
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu

pytestmark = pytest.mark.ros


def test_camera_failure_and_recovery():
    rclpy.init()
    node = rclpy.create_node(
        "diagnostics_under_test",
        parameter_overrides=[
            Parameter("camera_name", value="diagnostics_test_camera"),
            Parameter("startup_timeout", value=0.5),
            Parameter("frame_timeout", value=0.5),
            Parameter("bad_seconds", value=0.1),
        ],
    )
    probe = rclpy.create_node("diagnostics_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(probe)
    received = []
    probe.create_subscription(DiagnosticArray, "/diagnostics", received.append, 10)
    images = probe.create_publisher(
        Image,
        "/diagnostics_test_camera/zed_node/rgb/image_rect_color",
        qos_profile_sensor_data,
    )
    imu = probe.create_publisher(
        Imu, "/diagnostics_test_camera/zed_node/imu/data", qos_profile_sensor_data
    )
    Diagnostics(node)

    def wait_for(level, phrase, pixel=None):
        received.clear()
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if pixel is not None:
                stamp = probe.get_clock().now().to_msg()
                image = Image()
                image.header.stamp = stamp
                image.height, image.width, image.step = 2, 2, 6
                image.encoding, image.data = "rgb8", bytes(pixel * 4)
                images.publish(image)
                sample = Imu()
                sample.header.stamp = stamp
                imu.publish(sample)
            executor.spin_once(timeout_sec=0.02)
            if any(
                s.name == "catbot/zed/image"
                and s.level == bytes([level])
                and phrase in s.message
                for array in received
                for s in array.status
            ):
                return
        pytest.fail(
            f"Missing camera diagnostic: {level}, {phrase}; received {received}"
        )

    try:
        wait_for(DiagnosticStatus.ERROR, "no first image")
        wait_for(DiagnosticStatus.OK, "Receiving images", (50, 50, 50))
        wait_for(DiagnosticStatus.ERROR, "persistent green", (0, 255, 0))
        wait_for(DiagnosticStatus.OK, "Receiving images", (50, 50, 50))
        wait_for(DiagnosticStatus.ERROR, "image stream stopped")
        wait_for(DiagnosticStatus.OK, "Receiving images", (50, 50, 50))
    finally:
        executor.shutdown()
        node.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()
