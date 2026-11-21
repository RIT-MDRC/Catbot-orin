"""Explicit, non-moving camera mount calibration from four planted feet."""

import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from catbot_interfaces.msg import BodyErrorState, BodyKinematicState
from catbot_sim.camera_calibration import (
    depth_points,
    estimate_mount,
    fit_ground,
    foot_frames,
    foot_plane,
    rotation,
    save_calibration,
)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def transform_values(transform):
    q, t = transform.rotation, transform.translation
    return rotation([q.w, q.x, q.y, q.z]), np.array([t.x, t.y, t.z])


class CameraCalibration(Node):
    def __init__(self, **kwargs):
        super().__init__("camera_calibration", **kwargs)

        def param(name, default):
            return self.declare_parameter(name, default).value

        self.model_xml = (
            Path(param("run_directory", "")).expanduser() / "model.xml"
        ).read_text()
        self.body_frame, self.feet = foot_frames(self.model_xml)
        mapping = json.loads(Path(param("mapping_file", "")).expanduser().read_text())
        self.names = mapping["actuator_names"]
        self.prior_q = list(param("body_from_camera_wxyz", [1.0, 0.0, 0.0, 0.0]))
        self.prior_xyz = list(param("body_to_camera_xyz", [0.0, 0.0, 0.0]))
        self.prior_rotation = rotation(self.prior_q)
        self.camera_frame = str(param("camera_frame", "zed_camera_link"))
        self.world_frame = str(param("world_frame", "map"))
        self.output = str(
            param(
                "camera_calibration_output", "~/.config/catbot/camera_calibration.json"
            )
        )
        self.samples = {}
        self.active = False
        self.result = None
        self.records = []
        self.capture_specs = []
        self.capture_subscriptions = []
        self.state = "idle"
        self.detail = "Place all four feet on a flat floor, verify joint zeros, stop policy, then call start."
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(
            String, "/camera/calibration/status", latched
        )
        self.mount = self.create_publisher(
            PoseStamped, "/camera/calibration/mount", latched
        )
        self.heights = self.create_publisher(
            Vector3Stamped, "/camera/calibration/heights", latched
        )
        for key, cls, topic, qos in (
            (
                "depth",
                Image,
                param("depth_topic", "/zed/zed_node/depth/depth_registered"),
                qos_profile_sensor_data,
            ),
            (
                "intrinsics",
                CameraInfo,
                param("camera_info_topic", "/zed/zed_node/depth/camera_info"),
                qos_profile_sensor_data,
            ),
            (
                "imu",
                Imu,
                param("imu_topic", "/zed/zed_node/imu/data"),
                qos_profile_sensor_data,
            ),
            (
                "joints",
                BodyKinematicState,
                "/body/state/position",
                qos_profile_sensor_data,
            ),
            ("errors", BodyErrorState, "/body/state/error", latched),
            ("supervisor", String, "/rpi/supervisor/status", latched),
        ):
            if key in ("depth", "intrinsics"):
                self.capture_specs.append((key, cls, str(topic), qos))
            else:
                self.create_subscription(
                    cls, str(topic), lambda m, k=key: self.receive(k, m), qos
                )
        for operation in ("start", "cancel", "save"):
            self.create_service(
                Trigger, "/camera/calibration/" + operation, getattr(self, operation)
            )
        self.create_timer(0.1, self.tick)
        self.create_timer(0.5, self.publish_status)

    def receive(self, key, message):
        self.samples[key] = (message, time.monotonic())

    def stop_capture(self):
        for subscription in self.capture_subscriptions:
            self.destroy_subscription(subscription)
        self.capture_subscriptions.clear()
        for key in ("depth", "intrinsics"):
            self.samples.pop(key, None)

    def fresh(self, key):
        sample = self.samples.get(key)
        if sample is None or not 0 <= time.monotonic() - sample[1] < 0.5:
            raise ValueError(f"Waiting for fresh {key}")
        message = sample[0]
        stamp = getattr(message, "stamp", None) or getattr(
            getattr(message, "header", None), "stamp", None
        )
        if stamp is not None:
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp_seconds(stamp)
            if not -0.1 <= age < 0.5 or stamp_seconds(stamp) <= 0:
                raise ValueError(f"Invalid or stale {key} timestamp")
        return message

    def policy_stopped(self):
        status = json.loads(self.fresh("supervisor").data)
        if (
            status.get("orin_policy_active") is not False
            or status.get("mode") in ("running", "ready")
            or status.get("script")
        ):
            raise ValueError(
                "Stop policy and finish motor scripts before camera calibration"
            )

    def stationary(self):
        self.policy_stopped()
        joints, errors, imu = (
            self.fresh("joints"),
            self.fresh("errors"),
            self.fresh("imu"),
        )
        states = {a.actuator_name: a for a in joints.actuators}
        faults = {a.actuator_name: a for a in errors.actuators}
        if (
            len(states) != len(joints.actuators)
            or set(states) != set(self.names)
            or set(faults) != set(self.names)
        ):
            raise ValueError("Need the complete unique actuator roster")
        for name in self.names:
            a, e = states[name], faults[name]
            if not a.valid or not all(
                math.isfinite(v)
                for v in (a.position_degrees, a.velocity_degrees_per_second)
            ):
                raise ValueError(f"Invalid encoder for {name}")
            if not e.heartbeat_available or any(
                getattr(e, field)
                for field in (
                    "axis_error_flags",
                    "motor_error_flags",
                    "encoder_error_flags",
                    "sensorless_error_flags",
                    "controller_error_flags",
                )
            ):
                raise ValueError(f"Clear actuator fault on {name} before calibrating")
            if abs(a.velocity_degrees_per_second) > 1:
                raise ValueError("Robot moved; keep all four feet stationary")
        gyro = np.array(
            [imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z]
        )
        if (
            imu.angular_velocity_covariance[0] < 0
            or not np.isfinite(gyro).all()
            or np.linalg.norm(gyro) > 0.05
        ):
            raise ValueError("Robot moved or gyro is invalid")
        return np.array([states[n].position_degrees for n in self.names])

    def start(self, request, response):
        try:
            self.stationary()
            if self.active:
                raise ValueError("Calibration is already collecting")
            self.baseline = self.stationary()
            self.active, self.result, self.records = True, None, []
            self.started, self.last_depth = time.monotonic(), None
            for key, cls, topic, qos in self.capture_specs:
                self.capture_subscriptions.append(
                    self.create_subscription(
                        cls, topic, lambda m, k=key: self.receive(k, m), qos
                    )
                )
            self.state, self.detail = (
                "collecting",
                "Keep all four feet planted and the robot still for at least three seconds.",
            )
            response.success, response.message = True, self.detail
        except (ValueError, KeyError, TypeError) as error:
            response.success, response.message = False, str(error)
        self.publish_status()
        return response

    def cancel(self, request, response):
        self.active, self.result, self.records = False, None, []
        self.stop_capture()
        self.state, self.detail = "idle", "Calibration cancelled; no mount changed."
        response.success, response.message = True, self.detail
        self.publish_status()
        return response

    def save(self, request, response):
        try:
            self.policy_stopped()
            if self.active or self.result is None:
                raise ValueError("Collect and review a successful calibration first")
            save_calibration(self.output, self.result, self.model_xml)
            self.state = "saved"
            self.detail = f"Saved mount to {Path(self.output).expanduser()}; load with camera_calibration_file on the next launch."
            response.success, response.message = True, self.detail
        except (OSError, ValueError, KeyError, TypeError) as error:
            response.success, response.message = False, str(error)
        self.publish_status()
        return response

    def collect(self):
        image, info = self.fresh("depth"), self.fresh("intrinsics")
        if (
            abs(stamp_seconds(image.header.stamp) - stamp_seconds(info.header.stamp))
            > 0.1
            or abs(
                stamp_seconds(image.header.stamp)
                - stamp_seconds(self.fresh("joints").stamp)
            )
            > 0.1
        ):
            raise ValueError(
                "Depth, intrinsics and joint timestamps must agree within 0.1 s"
            )
        image_time = stamp_seconds(image.header.stamp)
        if image_time == self.last_depth:
            return
        stamp = Time.from_msg(image.header.stamp)
        centers, radii = [], []
        for frame, offset, radius in self.feet:
            r, t = transform_values(
                self.tf_buffer.lookup_transform(self.body_frame, frame, stamp).transform
            )
            centers.append(r @ offset + t)
            radii.append(radius)
        body_normal, body_offset = foot_plane(centers, radii)
        r, t = transform_values(
            self.tf_buffer.lookup_transform(
                self.camera_frame, image.header.frame_id, stamp
            ).transform
        )
        points = depth_points(image, info) @ r.T + t
        predicted_body = points @ self.prior_rotation.T + self.prior_xyz
        points = points[
            (predicted_body[:, 2] < -0.05) & (np.abs(predicted_body[:, 1]) < 1.5)
        ]
        normal, offset, quality = fit_ground(
            points, self.prior_rotation.T @ body_normal
        )
        result = estimate_mount(
            (normal, offset), (body_normal, body_offset), self.prior_q, self.prior_xyz
        )
        world_rotation, world_position = transform_values(
            self.tf_buffer.lookup_transform(
                self.world_frame, self.camera_frame, stamp
            ).transform
        )
        world_normal = world_rotation @ normal
        if world_normal[2] < math.cos(math.radians(10)):
            raise ValueError(
                "Calibration requires a level floor in the gravity-aligned world frame"
            )
        body_world = world_position - world_rotation @ rotation(
            result["body_from_camera_wxyz"]
        ).T @ np.array(result["body_to_camera_xyz"])
        result.update(quality)
        result.update(
            camera_height_m=float(offset / world_normal[2]),
            body_height_m=float(body_offset / world_normal[2]),
            ground_height_offset_at_capture=float(
                body_offset / world_normal[2] - body_world[2]
            ),
        )
        self.records.append(result)
        self.last_depth = image_time

    def finish(self):
        records = self.records
        angles = np.array([r["roll_pitch_yaw_degrees"] for r in records])
        heights = np.array([r["body_to_camera_xyz"][2] for r in records])
        if np.max(np.std(angles[:, :2], axis=0)) > 0.5 or np.std(heights) > 0.005:
            raise ValueError(
                "Calibration is inconsistent; check floor view, camera mount and joint zeros"
            )
        result = dict(records[-1])
        from catbot_sim.camera_calibration import quaternion

        result["body_from_camera_wxyz"] = quaternion(
            np.deg2rad(np.median(angles, axis=0))
        ).tolist()
        result["roll_pitch_yaw_degrees"] = np.median(angles, axis=0).tolist()
        result["body_to_camera_xyz"] = [*self.prior_xyz[:2], float(np.median(heights))]
        for key in (
            "camera_height_m",
            "body_height_m",
            "ground_height_offset_at_capture",
            "rms_m",
            "inlier_fraction",
        ):
            result[key] = float(np.median([r[key] for r in records]))
        result.update(
            samples=len(records),
            captured_at=datetime.now(timezone.utc).isoformat(),
            assumptions="Four planted feet; calibrated joint zeros/leg geometry; level floor. X/Y/yaw retained from prior.",
            world_frame=self.world_frame,
        )
        self.result, self.active = result, False
        self.stop_capture()
        self.state, self.detail = (
            "preview",
            "Review mount and height estimates, then call save. Existing camera transforms remain active.",
        )
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp = (
            self.body_frame,
            self.get_clock().now().to_msg(),
        )
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = result[
            "body_to_camera_xyz"
        ]
        (
            pose.pose.orientation.w,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
        ) = result["body_from_camera_wxyz"]
        self.mount.publish(pose)
        heights_msg = Vector3Stamped()
        heights_msg.header = pose.header
        heights_msg.vector.x, heights_msg.vector.y, heights_msg.vector.z = (
            result["camera_height_m"],
            result["body_height_m"],
            result["rms_m"],
        )
        self.heights.publish(heights_msg)

    def tick(self):
        if not self.active:
            return
        try:
            if np.max(np.abs(self.stationary() - self.baseline)) > 0.5:
                raise ValueError(
                    "Robot pose changed; restart calibration with four planted feet"
                )
            if time.monotonic() - self.started > 15:
                raise ValueError("Calibration timed out: " + self.detail)
        except (ValueError, KeyError, TypeError) as error:
            self.active, self.result = False, None
            self.stop_capture()
            self.state, self.detail = "failed", str(error)
            self.publish_status()
            return
        try:
            self.collect()
            if len(self.records) >= 15 and time.monotonic() - self.started >= 3:
                self.finish()
        except (TransformException, ValueError, KeyError, TypeError) as error:
            self.detail = str(error)

    def publish_status(self):
        self.status.publish(
            String(
                data=json.dumps(
                    {
                        "state": self.state,
                        "detail": self.detail,
                        "samples": len(self.records),
                        "result": self.result,
                    },
                    allow_nan=False,
                )
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraCalibration()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
