"""Fresh camera/encoder snapshots in the policy's model coordinate system."""

import math
import time

from geometry_msgs.msg import TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt8MultiArray
from tf2_ros import StaticTransformBroadcaster

from catbot_interfaces.msg import BodyKinematicState


def multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return [
        w * v - x * i - y * j - z * k,
        w * i + x * v + y * k - z * j,
        w * j - x * k + y * v + z * i,
        w * k + x * j - y * i + z * v,
    ]


def inverse(q):
    return [q[0], -q[1], -q[2], -q[3]]


def rotate(q, vector):
    return multiply(multiply(q, [0.0, *vector]), inverse(q))[1:]


def normalized(values):
    if len(values) != 4 or not all(math.isfinite(v) for v in values):
        raise ValueError("Expected finite quaternion w,x,y,z")
    norm = math.sqrt(sum(v * v for v in values))
    if norm < 1e-6:
        raise ValueError("Zero orientation quaternion")
    return [v / norm for v in values]


def xyz(value):
    return [value.x, value.y, value.z]


class PhysicalInputs:
    def __init__(self, node, mapping):
        self.node, self.mapping = node, mapping
        self.samples = {}
        self.world_tf = StaticTransformBroadcaster(node)
        self.odom_frame = None
        self.last_snapshot = None

        def param(name, default):
            return node.declare_parameter(name, default).value

        self.timeout = float(param("sensor_timeout", 0.25))
        self.skew = float(param("sensor_max_skew", 0.1))
        self.height = float(param("ground_height_offset", 0.35))
        self.threshold = float(param("contact_height", 0.02))
        self.estimate_contacts = bool(param("estimate_contacts", True))
        self.camera_rotation = normalized(
            list(param("body_from_camera_wxyz", [1.0, 0.0, 0.0, 0.0]))
        )
        self.imu_rotation = normalized(
            list(param("body_from_imu_wxyz", [1.0, 0.0, 0.0, 0.0]))
        )
        self.camera_offset = list(param("body_to_camera_xyz", [0.0, 0.0, 0.0]))
        self.remove_gravity = bool(param("imu_includes_gravity", True))
        if (
            not all(
                math.isfinite(v)
                for v in [
                    self.timeout,
                    self.skew,
                    self.height,
                    self.threshold,
                    *self.camera_offset,
                ]
            )
            or self.timeout <= 0
            or self.skew < 0
            or len(self.camera_offset) != 3
        ):
            raise ValueError("Invalid physical sensor settings")
        self.acceleration = node.create_publisher(
            Vector3Stamped, "/body/state/linear_acceleration", 1
        )
        for key, message, topic in (
            ("imu", Imu, param("imu_topic", "/zed/zed_node/imu/data")),
            ("odom", Odometry, param("odom_topic", "/zed/zed_node/odom")),
            ("joints", BodyKinematicState, "/body/state/position"),
        ):
            node.create_subscription(
                message,
                topic,
                lambda msg, name=key: self.receive(name, msg),
                qos_profile_sensor_data,
            )
        node.create_subscription(
            UInt8MultiArray,
            str(param("contact_topic", "/body/state/foot_contacts")),
            lambda msg: self.receive("contacts", msg),
            qos_profile_sensor_data,
        )
        node.get_logger().warning(
            "Physical feedback requires calibrated camera/IMU mounting and ground_height_offset. "
            + (
                "Foot contact is estimated from flat-ground kinematics when no contact sample is available."
                if self.estimate_contacts
                else "A fresh four-foot contact topic is required."
            )
        )

    def receive(self, name, message):
        with self.node.lock:
            self.samples[name] = (message, time.monotonic())

    def snapshot(self, command):
        now = time.monotonic()
        with self.node.lock:
            samples = dict(self.samples)
        if any(
            k not in samples or now - samples[k][1] > self.timeout
            for k in ("imu", "odom", "joints")
        ):
            return None
        imu, odom, joints = [samples[k][0] for k in ("imu", "odom", "joints")]
        stamps = [imu.header.stamp, odom.header.stamp, joints.stamp]
        times = [s.sec + s.nanosec * 1e-9 for s in stamps]
        ros_now = self.node.get_clock().now().nanoseconds * 1e-9
        if (
            any(t <= 0 or not -self.skew <= ros_now - t <= self.timeout for t in times)
            or max(times) - min(times) > self.skew
        ):
            return None
        if (
            imu.angular_velocity_covariance[0] < 0
            or odom.pose.covariance[0] < 0
            or odom.twist.covariance[0] < 0
        ):
            return None
        by_name = {a.actuator_name: a for a in joints.actuators}
        if len(by_name) != len(joints.actuators) or any(
            n not in by_name or not by_name[n].valid
            for n in self.mapping["actuator_names"]
        ):
            return None
        if not odom.header.frame_id or (
            self.odom_frame and odom.header.frame_id != self.odom_frame
        ):
            return None
        if self.odom_frame is None:
            self.odom_frame = odom.header.frame_id
            transform = TransformStamped()
            transform.header.stamp = odom.header.stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = "robot/world"
            transform.transform.translation.z = -self.height
            transform.transform.rotation.w = 1.0
            self.world_tf.sendTransform(transform)
        orientation = odom.pose.pose.orientation
        try:
            camera_q = normalized(
                [orientation.w, orientation.x, orientation.y, orientation.z]
            )
        except ValueError:
            return None
        body_q = multiply(camera_q, inverse(self.camera_rotation))
        offset_world = rotate(body_q, self.camera_offset)
        position = [
            p - o
            for p, o in zip(xyz(odom.pose.pose.position), offset_world, strict=True)
        ]
        position[2] += self.height
        angular = rotate(self.imu_rotation, xyz(imu.angular_velocity))
        # Odometry twist is in child_frame_id. Remove velocity due to the camera lever arm.
        camera_velocity_body = rotate(
            self.camera_rotation, xyz(odom.twist.twist.linear)
        )
        x, y, z = angular
        a, b, c = self.camera_offset
        cross = [y * c - z * b, z * a - x * c, x * b - y * a]
        velocity_world = rotate(
            body_q, [v - o for v, o in zip(camera_velocity_body, cross, strict=True)]
        )
        positions, velocities = [], []
        for i, name in enumerate(self.mapping["actuator_names"]):
            state = by_name[name]
            sign = self.mapping["signs"][i]
            positions.append(
                math.radians(
                    (state.position_degrees - self.mapping["offsets_degrees"][i]) / sign
                )
            )
            velocities.append(math.radians(state.velocity_degrees_per_second / sign))
        qpos, qvel = (
            [*position, *body_q, *positions],
            [*velocity_world, *angular, *velocities],
        )
        if not all(math.isfinite(v) for v in [*qpos, *qvel]):
            return None
        contacts = None
        if "contacts" in samples and now - samples["contacts"][1] <= self.timeout:
            values = list(samples["contacts"][0].data)
            if len(values) != 4 or any(v not in (0, 1) for v in values):
                return None
            contacts = [bool(v) for v in values]
        elif not self.estimate_contacts:
            return None
        if imu.linear_acceleration_covariance[0] >= 0:
            accel = rotate(self.imu_rotation, xyz(imu.linear_acceleration))
            if self.remove_gravity:
                gravity = rotate(inverse(body_q), [0.0, 0.0, -9.81])
                accel = [a + g for a, g in zip(accel, gravity, strict=True)]
            if all(math.isfinite(v) for v in accel):
                msg = Vector3Stamped()
                msg.header.stamp = imu.header.stamp
                msg.header.frame_id = "robot/body_1_quadruped"
                msg.vector.x, msg.vector.y, msg.vector.z = accel
                self.acceleration.publish(msg)
        elapsed = (
            min(now - self.last_snapshot, self.timeout) if self.last_snapshot else 0.001
        )
        self.last_snapshot = now
        return {
            "sample_time": min(
                min(samples[k][1] for k in ("imu", "odom", "joints")),
                now - max(ros_now - t for t in times),
            ),
            "version": 1,
            "qpos": qpos,
            "qvel": qvel,
            "command": command,
            "elapsed": max(elapsed, 1e-6),
            "contacts": contacts,
            "contact_height": self.threshold,
        }
