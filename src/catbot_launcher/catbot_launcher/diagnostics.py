"""Publish camera and robot health for Foxglove's ROS Diagnostics panels."""

import math
import socket
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu

from catbot_interfaces.msg import (
    ActuatorPositionCommand,
    BodyErrorState,
    BodyKinematicState,
    PolicyFeedback,
)
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


class Diagnostics:
    def __init__(self, node):
        self.node = node
        self.hardware_id = socket.gethostname()

        def param(name, default):
            return node.declare_parameter(name, default).value

        self.started = time.monotonic()
        self.startup_timeout = float(param("startup_timeout", 60.0))
        self.stream_timeout = float(param("stream_timeout", 2.0))
        frame_timeout = float(param("frame_timeout", 10.0))
        bad_seconds = float(param("bad_seconds", 5.0))
        green_threshold = float(param("green_threshold", 0.98))
        if (
            any(
                not math.isfinite(v) or v <= 0
                for v in (
                    self.startup_timeout,
                    self.stream_timeout,
                    frame_timeout,
                    bad_seconds,
                )
            )
            or not 0 < green_threshold <= 1
        ):
            raise ValueError(
                "Diagnostics timeouts must be positive; green threshold in (0, 1]"
            )
        self.streams = {}
        self.image_health = None
        self.errors = None
        self.subscriptions = []
        self.publisher = node.create_publisher(DiagnosticArray, "/diagnostics", 10)
        if param("monitor_camera", True):
            camera = param("camera_name", "zed").strip("/")
            self.image_topic = param(
                "image_topic", f"/{camera}/zed_node/rgb/image_rect_color"
            )
            self.image_health = Health(
                self.started,
                self.startup_timeout,
                frame_timeout,
                bad_seconds,
                green_threshold,
            )
            self.subscriptions.append(
                node.create_subscription(
                    Image,
                    self.image_topic,
                    lambda msg: self.image_health.observe(msg, time.monotonic()),
                    qos_profile_sensor_data,
                )
            )
            self.add_stream(
                "zed/imu", Imu, param("imu_topic", f"/{camera}/zed_node/imu/data")
            )
            # Camera-only launches may intentionally disable positional tracking.
            if param("monitor_odometry", False):
                self.add_stream(
                    "zed/odometry",
                    Odometry,
                    param("odom_topic", f"/{camera}/zed_node/odom"),
                )
        if param("monitor_robot", False):
            self.add_stream("body/joints", BodyKinematicState, "/body/state/position")
            self.add_stream("policy/feedback", PolicyFeedback, "/policy/feedback")
            self.add_stream(
                "policy/output", ActuatorPositionCommand, "/body/control/position"
            )
            if param("monitor_motors", True):
                self.subscriptions.append(
                    node.create_subscription(
                        BodyErrorState,
                        "/body/state/error",
                        self.receive_errors,
                        QoSProfile(
                            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
                        ),
                    )
                )
                self.monitor_motors = True
            else:
                self.monitor_motors = False
        else:
            self.monitor_motors = False
        # A wall/steady timer keeps diagnostics alive if simulated ROS time pauses.
        from rclpy.clock import Clock, ClockType

        self.timer = node.create_timer(
            1.0, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.publish()

    def add_stream(self, name, message_type, topic):
        health = StreamHealth(self.started, self.stream_timeout, self.startup_timeout)
        self.streams[name] = (topic, health)
        self.subscriptions.append(
            self.node.create_subscription(
                message_type,
                topic,
                lambda msg: health.observe(msg, time.monotonic()),
                qos_profile_sensor_data,
            )
        )

    def receive_errors(self, message):
        self.errors = message

    def status(self, name, level, message, values):
        return DiagnosticStatus(
            name=f"catbot/{name}",
            hardware_id=self.hardware_id,
            level=level,
            message=message,
            values=[
                KeyValue(key=key, value=str(value)) for key, value in values.items()
            ],
        )

    def publish(self):
        now = time.monotonic()
        statuses = []
        if self.image_health is not None:
            health = self.image_health
            level, message = image_status(health, now)
            statuses.append(
                self.status(
                    "zed/image",
                    level,
                    message,
                    {
                        "topic": self.image_topic,
                        "last_message_age_seconds": "never"
                        if health.last_frame is None
                        else now - health.last_frame,
                        "frame_timeout_seconds": health.frame_timeout,
                        "green_threshold": health.green_threshold,
                        "bad_seconds": health.bad_seconds,
                    },
                )
            )
        for name, (topic, health) in self.streams.items():
            level, message = health.status(now)
            if level == OK and name == "body/joints":
                level, message = joint_status(health.message)
            if (
                level == OK
                and name == "zed/imu"
                and health.message.angular_velocity_covariance[0] < 0
            ):
                level, message = ERROR, "IMU angular velocity unavailable"
            if (
                level == OK
                and name == "zed/odometry"
                and (
                    health.message.pose.covariance[0] < 0
                    or health.message.twist.covariance[0] < 0
                )
            ):
                level, message = ERROR, "Odometry pose or velocity unavailable"
            statuses.append(
                self.status(
                    name,
                    level,
                    message,
                    {
                        "topic": topic,
                        "last_message_age_seconds": "never"
                        if health.last_received is None
                        else now - health.last_received,
                        "timeout_seconds": health.timeout,
                    },
                )
            )
        if self.monitor_motors:
            values = {
                "topic": "/body/state/error",
                "reporting": "latched; published on change",
            }
            if self.errors is None:
                level = WARN if now - self.started < self.startup_timeout else ERROR
                message = "Waiting for motor fault state"
            else:
                level, message, details = motor_status(self.errors)
                values.update(details)
                # A latched OK report cannot prove that foundation is still alive.
                joints = self.streams["body/joints"][1]
                joint_level, _ = joints.status(now)
                if joint_level != OK and level != ERROR:
                    level, message = (
                        joint_level,
                        "Motor state unconfirmed; joint stream unavailable",
                    )
            statuses.append(self.status("body/motors", level, message, values))
        message = DiagnosticArray()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.status = statuses
        self.publisher.publish(message)


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = rclpy.create_node("catbot_diagnostics")
    try:
        Diagnostics(node)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
