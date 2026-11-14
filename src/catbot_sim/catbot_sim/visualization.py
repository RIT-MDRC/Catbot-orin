"""Render compiled MuJoCo primitives through ROS TF and MarkerArray."""

import time

from geometry_msgs.msg import TransformStamped
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


class ScenePublisher:
    """Keep simulation frames separate from the real camera's TF tree."""

    def __init__(self, node, prefix="sim"):
        self.prefix = prefix
        self.node = node
        self.tf = TransformBroadcaster(node)
        self.publisher = node.create_publisher(
            MarkerArray,
            "/body/model",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.scene = None
        self.markers = None
        self.last_markers = float("-inf")

    def frame(self, body):
        if self.scene is None:
            raise RuntimeError("Model scene is not initialized")
        if body == 0:
            return f"{self.prefix}/world"
        # Numeric IDs ensure uniqueness even for unnamed or similarly named bodies.
        name = self.scene["bodies"][body]["name"].replace("/", "_")
        return f"{self.prefix}/body_{body}_{name}"

    def build_markers(self):
        if self.scene is None:
            raise RuntimeError("Model scene is not initialized")
        markers = []
        clear = Marker()
        clear.header.frame_id = f"{self.prefix}/world"
        clear.action = Marker.DELETEALL
        markers.append(clear)
        for geom in self.scene["geoms"]:
            kind, size = geom["kind"], geom["size"]
            radius = size[0]
            if kind == "box":
                pieces = [(Marker.CUBE, [2 * v for v in size], 0.0)]
            elif kind == "plane":
                # MuJoCo planes are infinite; display a finite patch with top at z=0.
                pieces = [
                    (Marker.CUBE, [2 * (size[0] or 5), 2 * (size[1] or 5), 0.02], -0.01)
                ]
            elif kind == "ellipsoid":
                pieces = [(Marker.SPHERE, [2 * v for v in size], 0.0)]
            elif kind == "sphere":
                pieces = [(Marker.SPHERE, [2 * radius] * 3, 0.0)]
            elif kind in ("cylinder", "capsule"):
                pieces = [(Marker.CYLINDER, [2 * radius, 2 * radius, 2 * size[1]], 0.0)]
                if kind == "capsule":
                    pieces += [
                        (Marker.SPHERE, [2 * radius] * 3, z)
                        for z in (-size[1], size[1])
                    ]
            else:
                raise ValueError(f"Unsupported visualization primitive: {kind}")
            w, x, y, z = geom["quaternion"]
            # Geometry-local Z axis expressed in the owning body's frame.
            axis = [2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)]
            for piece_id, (marker_type, scale, offset) in enumerate(pieces):
                marker = Marker()
                marker.header.frame_id = self.frame(geom["body"])
                marker.ns = "catbot_sim"
                marker.id = geom["id"] * 3 + piece_id
                marker.type = marker_type
                marker.action = Marker.ADD
                marker.frame_locked = True
                (
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z,
                ) = [
                    p + offset * a for p, a in zip(geom["position"], axis, strict=True)
                ]
                marker.pose.orientation.w = w
                marker.pose.orientation.x = x
                marker.pose.orientation.y = y
                marker.pose.orientation.z = z
                marker.scale.x, marker.scale.y, marker.scale.z = scale
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = geom[
                    "rgba"
                ]
                markers.append(marker)
        return MarkerArray(markers=markers)

    def publish(self, scene, poses):
        if self.scene is None:
            self.scene = scene
            self.markers = self.build_markers()
        stamp = self.node.get_clock().now().to_msg()
        transforms = []
        for pose in poses:
            body = pose["body"]
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.frame(scene["bodies"][body]["parent"])
            transform.child_frame_id = self.frame(body)
            t, q = transform.transform.translation, transform.transform.rotation
            t.x, t.y, t.z = pose["position"]
            q.w, q.x, q.y, q.z = pose["quaternion"]
            transforms.append(transform)
        self.tf.sendTransform(transforms)
        now = time.monotonic()
        if now - self.last_markers >= 1:
            # Also repeat for volatile late subscribers, including reconnecting viewers.
            self.publisher.publish(self.markers)
            self.last_markers = now
