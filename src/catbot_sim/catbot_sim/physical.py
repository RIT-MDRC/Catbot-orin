"""Fresh camera/encoder snapshots in the policy's model coordinate system."""

import json
import math
import time

from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String, UInt8MultiArray
from tf2_ros import (
    Buffer,
    StaticTransformBroadcaster,
    TransformException,
    TransformListener,
)

from catbot_interfaces.msg import BodyKinematicState, SupervisorStatus
from catbot_sim.ground import GroundFilter, ground_reference, ground_updates_allowed


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
        self.published_ground_height = None
        self.last_ground_attempt = float("-inf")
        self.ground_filter = GroundFilter()
        self.ground_frozen = False
        self.blocked_reason = "Waiting for IMU, odometry and joint states"
        self.ground_detail = "Waiting for depth ground reference"

        def param(name, default):
            return node.declare_parameter(name, default).value

        self.timeout = float(param("sensor_timeout", 0.25))
        self.skew = float(param("sensor_max_skew", 0.1))
        # Depth processing is slower than control telemetry. Only floor fitting
        # uses this bound; IMU/odometry/encoder freshness stays unchanged.
        self.ground_depth_timeout = float(param("ground_depth_timeout", 0.75))
        self.height = float(param("ground_height_offset", 0.35))
        self.estimate_ground = bool(param("estimate_ground_from_depth", False))
        self.ground_ready = not self.estimate_ground
        self.camera_ground_height = float(param("camera_ground_height", 0.0))
        self.camera_height_frame = str(param("camera_height_frame", ""))
        self.threshold = float(param("contact_height", 0.02))
        self.estimate_contacts = bool(param("estimate_contacts", True))
        self.camera_rotation = normalized(
            list(param("body_from_camera_wxyz", [1.0, 0.0, 0.0, 0.0]))
        )
        self.imu_rotation = normalized(
            list(param("body_from_imu_wxyz", [1.0, 0.0, 0.0, 0.0]))
        )
        self.world_frame = str(param("world_frame", ""))
        self.imu_mount_from_tf = bool(param("imu_mount_from_tf", False))
        if (
            self.imu_mount_from_tf
            or self.world_frame
            or self.camera_ground_height > 0
            or self.estimate_ground
        ):
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, node)
        self.camera_offset = list(param("body_to_camera_xyz", [0.0, 0.0, 0.0]))
        self.remove_gravity = bool(param("imu_includes_gravity", True))
        if (
            not all(
                math.isfinite(v)
                for v in [
                    self.timeout,
                    self.skew,
                    self.ground_depth_timeout,
                    self.height,
                    self.camera_ground_height,
                    self.threshold,
                    *self.camera_offset,
                ]
            )
            or self.timeout <= 0
            or self.ground_depth_timeout <= 0
            or self.skew < 0
            or self.camera_ground_height < 0
            or (self.camera_ground_height > 0 and not self.camera_height_frame)
            or (self.estimate_ground and self.camera_ground_height > 0)
            or len(self.camera_offset) != 3
        ):
            raise ValueError("Invalid physical sensor settings")
        self.acceleration = node.create_publisher(
            Vector3Stamped, "/body/state/linear_acceleration", 1
        )
        if self.estimate_ground:
            self.ground_status = node.create_publisher(
                String, "/camera/ground/status", 1
            )
            self.ground_pose = node.create_publisher(
                PoseStamped, "/camera/ground/reference", 1
            )
            for key, message, topic, qos in (
                (
                    "depth",
                    Image,
                    param("depth_topic", "/zed/zed_node/depth/depth_registered"),
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                ),
                (
                    "intrinsics",
                    CameraInfo,
                    param("camera_info_topic", "/zed/zed_node/depth/camera_info"),
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                ),
                (
                    "supervisor",
                    SupervisorStatus,
                    "/rpi/supervisor/status",
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                ),
            ):
                node.create_subscription(
                    message,
                    str(topic),
                    lambda msg, name=key: self.receive(name, msg),
                    qos,
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
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
        node.create_subscription(
            UInt8MultiArray,
            str(param("contact_topic", "/body/state/foot_contacts")),
            lambda msg: self.receive("contacts", msg),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        node.get_logger().warning(
            "Physical feedback requires calibrated camera/IMU mounting. "
            + (
                "Ground height is estimated from depth. "
                if self.estimate_ground
                else "Ground height uses the configured startup reference. "
            )
            + (
                "Foot contact is estimated from flat-ground kinematics when no contact sample is available."
                if self.estimate_contacts
                else "A fresh four-foot contact topic is required."
            )
        )

    def receive(self, name, message):
        with self.node.lock:
            self.samples[name] = (message, time.monotonic())

    def update_ground(self, samples, odom, now):
        """Update the same offset used by model TF and policy height, never during policy motion."""
        allowed = False
        if "supervisor" in samples:
            status, received = samples["supervisor"]
            allowed = ground_updates_allowed(
                {
                    "mode": status.mode,
                    "script": status.script,
                    "orin_policy_active": status.orin_policy_active,
                },
                now - received,
            )
        if self.ground_ready and not allowed:
            if not self.ground_frozen:
                self.ground_status.publish(
                    String(
                        data=json.dumps(
                            {
                                "state": "frozen",
                                "detail": "Policy active or stopped-state confirmation unavailable",
                                "ground_world_z": -self.height,
                            }
                        )
                    )
                )
                self.ground_frozen = True
            return
        self.ground_frozen = False
        if now - self.last_ground_attempt < 0.5:
            return
        self.last_ground_attempt = now
        try:
            if "depth" not in samples or "intrinsics" not in samples:
                raise ValueError("Waiting for depth and camera intrinsics")
            image, received = samples["depth"]
            info = samples["intrinsics"][0]
            captured = Time.from_msg(image.header.stamp)
            age = (
                self.node.get_clock().now().nanoseconds - captured.nanoseconds
            ) * 1e-9
            if (
                captured.nanoseconds <= 0
                or not 0 <= now - received <= self.ground_depth_timeout
                or not -self.skew <= age <= self.ground_depth_timeout
            ):
                raise ValueError("Depth image is stale or its clock is out of sync")
            # CameraInfo describes calibration, not a synchronized pose sample.
            # depth_points validates its frame, dimensions and projection. Resolve
            # every moving transform at capture time instead of comparing delayed
            # depth against independently arriving, newer odometry/IMU messages.
            frame = self.world_frame or odom.header.frame_id
            transform = self.tf_buffer.lookup_transform(
                frame, image.header.frame_id, captured
            ).transform
            camera = self.tf_buffer.lookup_transform(
                frame, odom.child_frame_id, captured
            ).transform
            camera_q = camera.rotation
            body_q = multiply(
                normalized([camera_q.w, camera_q.x, camera_q.y, camera_q.z]),
                inverse(self.camera_rotation),
            )
            body_position = [
                p - o
                for p, o in zip(
                    xyz(camera.translation),
                    rotate(body_q, self.camera_offset),
                    strict=True,
                )
            ]
            q = transform.rotation
            imu = samples["imu"][0]
            acceleration = xyz(imu.linear_acceleration)
            magnitude = math.sqrt(sum(v * v for v in acceleration))
            if (
                imu.linear_acceleration_covariance[0] < 0
                or not 8.5 <= magnitude <= 11.0
            ):
                raise ValueError("Stationary IMU gravity unavailable")
            imu_tf = self.tf_buffer.lookup_transform(
                frame, imu.header.frame_id, captured
            ).transform.rotation
            gravity_up = rotate(
                [imu_tf.w, imu_tf.x, imu_tf.y, imu_tf.z],
                [v / magnitude for v in acceleration],
            )
            height, quality = ground_reference(
                image,
                info,
                [q.w, q.x, q.y, q.z],
                xyz(transform.translation),
                body_position,
                gravity_up=gravity_up,
            )
            height = self.ground_filter.update(height, quality)
            self.height, self.ground_ready = height, True
            reference = PoseStamped()
            reference.header.frame_id, reference.header.stamp = (
                frame,
                image.header.stamp,
            )
            reference.pose.position.x, reference.pose.position.y = body_position[:2]
            reference.pose.position.z = -height
            reference.pose.orientation.w = 1.0
            self.ground_pose.publish(reference)
            report = {"state": "tracking", "ground_world_z": -height, **quality}
        except (TransformException, ValueError, KeyError, TypeError) as error:
            report = {
                "state": "holding" if self.ground_ready else "waiting",
                "detail": str(error),
            }
        self.ground_detail = report.get("detail", "Ground reference ready")
        self.ground_status.publish(String(data=json.dumps(report, allow_nan=False)))

    def blocked(self, reason):
        self.blocked_reason = reason

    def snapshot(self, command):
        with self.node.lock:
            samples = dict(self.samples)
        now = time.monotonic()
        unavailable = [
            k
            for k in ("imu", "odom", "joints")
            if k not in samples or not 0 <= now - samples[k][1] <= self.timeout
        ]
        if unavailable:
            return self.blocked("Missing or stale inputs: " + ", ".join(unavailable))
        imu, odom, joints = [samples[k][0] for k in ("imu", "odom", "joints")]
        stamps = [imu.header.stamp, odom.header.stamp, joints.stamp]
        times = [s.sec + s.nanosec * 1e-9 for s in stamps]
        ros_now = self.node.get_clock().now().nanoseconds * 1e-9
        if (
            any(t <= 0 or not -self.skew <= ros_now - t <= self.timeout for t in times)
            or max(times) - min(times) > self.skew
        ):
            return self.blocked(
                "Sensor timestamps are stale or disagree; check Pi/Orin clock sync"
            )
        if (
            imu.angular_velocity_covariance[0] < 0
            or odom.pose.covariance[0] < 0
            or odom.twist.covariance[0] < 0
        ):
            return self.blocked(
                "IMU or odometry covariance marks the measurement invalid"
            )
        by_name = {a.actuator_name: a for a in joints.actuators}
        if len(by_name) != len(joints.actuators) or any(
            n not in by_name or not by_name[n].valid
            for n in self.mapping["actuator_names"]
        ):
            return self.blocked(
                "Joint state roster is incomplete, duplicated or invalid"
            )
        if not odom.header.frame_id or (
            self.odom_frame and odom.header.frame_id != self.odom_frame
        ):
            return self.blocked("Odometry frame is missing or changed")
        orientation = odom.pose.pose.orientation
        try:
            camera_q = normalized(
                [orientation.w, orientation.x, orientation.y, orientation.z]
            )
            camera_position = xyz(odom.pose.pose.position)
            if self.world_frame and self.world_frame != odom.header.frame_id:
                # ZED's odometry starts at the camera's pose, including its tilt.
                # Convert to the gravity-aligned map before deriving body pose.
                world_from_odom = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    odom.header.frame_id,
                    Time.from_msg(odom.header.stamp),
                ).transform
                q = world_from_odom.rotation
                rotation = normalized([q.w, q.x, q.y, q.z])
                camera_q = multiply(rotation, camera_q)
                camera_position = [
                    a + b
                    for a, b in zip(
                        rotate(rotation, camera_position),
                        xyz(world_from_odom.translation),
                        strict=True,
                    )
                ]
        except (TransformException, ValueError):
            return self.blocked("Waiting for world-to-odometry TF")
        if (
            self.odom_frame is None
            and not self.estimate_ground
            and self.camera_ground_height > 0
        ):
            try:
                # The lens midpoint differs from the mounting link used by
                # odometry. Transform that measured point before anchoring.
                camera_to_measured = self.tf_buffer.lookup_transform(
                    odom.child_frame_id,
                    self.camera_height_frame,
                    Time.from_msg(odom.header.stamp),
                ).transform.translation
            except TransformException:
                return self.blocked("Waiting for measured camera mounting TF")
            measured_offset = rotate(camera_q, xyz(camera_to_measured))
            measured_world_z = camera_position[2] + measured_offset[2]
            if not math.isfinite(measured_world_z):
                return self.blocked("Measured camera height is not finite")
            self.height = self.camera_ground_height - measured_world_z
            self.node.get_logger().info(
                f"Anchored ground from {self.camera_height_frame}: "
                f"camera height {self.camera_ground_height:.3f} m, "
                f"world height offset {self.height:.3f} m"
            )
        body_q = multiply(camera_q, inverse(self.camera_rotation))
        imu_rotation = self.imu_rotation
        if self.imu_mount_from_tf:
            if not odom.child_frame_id or not imu.header.frame_id:
                return self.blocked("Camera or IMU frame is missing")
            try:
                camera_from_imu = self.tf_buffer.lookup_transform(
                    odom.child_frame_id, imu.header.frame_id, Time()
                ).transform.rotation
                imu_rotation = multiply(
                    self.camera_rotation,
                    normalized(
                        [
                            camera_from_imu.w,
                            camera_from_imu.x,
                            camera_from_imu.y,
                            camera_from_imu.z,
                        ]
                    ),
                )
            except (TransformException, ValueError):
                return self.blocked("Waiting for camera-to-IMU mounting TF")
        offset_world = rotate(body_q, self.camera_offset)
        position = [p - o for p, o in zip(camera_position, offset_world, strict=True)]
        if self.estimate_ground:
            self.update_ground(samples, odom, now)
            if not self.ground_ready:
                return self.blocked(self.ground_detail)
        position[2] += self.height
        angular = rotate(imu_rotation, xyz(imu.angular_velocity))
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
            return self.blocked("Body pose or velocity is not finite")
        contacts = None
        if "contacts" in samples and now - samples["contacts"][1] <= self.timeout:
            values = list(samples["contacts"][0].data)
            if len(values) != 4 or any(v not in (0, 1) for v in values):
                return self.blocked("Foot contacts must contain four binary values")
            contacts = [bool(v) for v in values]
        elif not self.estimate_contacts:
            return self.blocked("Waiting for fresh foot contacts")
        if self.published_ground_height != self.height:
            transform = TransformStamped()
            transform.header.stamp = odom.header.stamp
            transform.header.frame_id = self.world_frame or odom.header.frame_id
            transform.child_frame_id = "robot/world"
            transform.transform.translation.z = -self.height
            transform.transform.rotation.w = 1.0
            self.world_tf.sendTransform(transform)
            self.published_ground_height = self.height
        self.odom_frame = odom.header.frame_id
        if imu.linear_acceleration_covariance[0] >= 0:
            accel = rotate(imu_rotation, xyz(imu.linear_acceleration))
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
        self.blocked_reason = None
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
