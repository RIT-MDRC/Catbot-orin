"""Stationary four-foot camera calibration, independent of ROS and motor I/O."""

import hashlib
import json
import math
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def rotation(wxyz):
    q = np.asarray(wxyz, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-6:
        raise ValueError("Invalid camera quaternion")
    w, x, y, z = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def quaternion(rpy):
    r, p, y = np.asarray(rpy) / 2
    cr, cp, cy = np.cos([r, p, y])
    sr, sp, sy = np.sin([r, p, y])
    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
    )


def roll_pitch_yaw(q):
    r = rotation(q)
    pitch = math.asin(float(np.clip(-r[2, 0], -1, 1)))
    if abs(math.cos(pitch)) < 0.1:
        raise ValueError("Camera prior is too close to an Euler singularity")
    return np.array([math.atan2(r[2, 1], r[2, 2]), pitch, math.atan2(r[1, 0], r[0, 0])])


def plane(points):
    points = np.asarray(points, dtype=float)
    if (
        points.ndim != 2
        or points.shape[1] != 3
        or len(points) < 3
        or not np.isfinite(points).all()
    ):
        raise ValueError("Need at least three finite 3D points")
    center = points.mean(axis=0)
    _, singular, vectors = np.linalg.svd(points - center, full_matrices=False)
    if singular[1] / math.sqrt(len(points)) < 0.03:
        raise ValueError("Ground points do not span a plane")
    normal = vectors[-1]
    return normal, -float(normal @ center)


def foot_plane(centers, radii, tolerance=0.01):
    """Ground is one foot radius below each sphere center along its normal."""
    centers, radii = np.asarray(centers), np.asarray(radii)
    if (
        centers.shape != (4, 3)
        or radii.shape != (4,)
        or not np.isfinite(radii).all()
        or (radii <= 0).any()
    ):
        raise ValueError("Need four spherical feet with positive radii")
    normal, offset = plane(centers)
    if normal[2] < 0:
        normal, offset = -normal, -offset
    offset += float(radii.mean())
    residual = centers @ normal + offset - radii
    if (
        normal[2] < math.cos(math.radians(20))
        or np.max(np.abs(residual)) > tolerance
        or not 0.1 < offset < 1
    ):
        raise ValueError(
            "Four feet must support the body on one approximately level plane"
        )
    return normal, offset


def fit_ground(points, expected_normal, threshold=0.012):
    """Fit a broad floor patch; reject walls, sparse patches and poor consensus."""
    points = np.asarray(points, dtype=float)
    expected = np.asarray(expected_normal, dtype=float)
    expected /= np.linalg.norm(expected)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("Invalid depth points")
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 100:
        raise ValueError("Not enough valid floor depth")
    points = points[:: max(1, math.ceil(len(points) / 3000))]
    rng = np.random.default_rng(17)
    best = np.zeros(len(points), dtype=bool)
    alignment = math.cos(math.radians(25))
    for _ in range(100):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        length = np.linalg.norm(normal)
        if length < 1e-8:
            continue
        normal /= length
        if abs(normal @ expected) < alignment:
            continue
        mask = np.abs((points - a) @ normal) < threshold
        if mask.sum() > best.sum():
            best = mask
    if best.sum() < max(100, len(points) * 0.5):
        raise ValueError(
            "No reliable ground plane; move obstacles out of the floor view"
        )
    normal, offset = plane(points[best])
    if normal @ expected < 0:
        normal, offset = -normal, -offset
    if normal @ expected < alignment or not 0.08 < offset < 1.5:
        raise ValueError("Detected plane is not plausible ground below the camera")
    rms = float(np.sqrt(np.mean((points[best] @ normal + offset) ** 2)))
    return normal, offset, {"inlier_fraction": float(best.mean()), "rms_m": rms}


def estimate_mount(camera_plane, body_plane, prior_wxyz, prior_xyz):
    """Solve roll/pitch/Z conditional on retained yaw/X/Y; never claim full extrinsics."""
    camera_normal, camera_offset = camera_plane
    body_normal, body_offset = body_plane
    camera_normal, body_normal = np.asarray(camera_normal), np.asarray(body_normal)
    initial = roll_pitch_yaw(prior_wxyz)
    angles = initial.copy()
    for _ in range(15):
        current = rotation(quaternion(angles)) @ camera_normal
        residual = current - body_normal
        jacobian = np.empty((3, 2))
        for i in range(2):
            perturbed = angles.copy()
            perturbed[i] += 1e-5
            jacobian[:, i] = (
                rotation(quaternion(perturbed)) @ camera_normal - current
            ) / 1e-5
        if np.linalg.svd(jacobian, compute_uv=False)[-1] < 0.05:
            raise ValueError("Floor view cannot resolve tilt with the retained yaw")
        delta = np.linalg.lstsq(jacobian, -residual, rcond=None)[0]
        angles[:2] += delta
        if np.linalg.norm(delta) < 1e-8:
            break
    if (
        np.linalg.norm(rotation(quaternion(angles)) @ camera_normal - body_normal)
        > 1e-4
    ):
        raise ValueError("Could not align camera and foot ground planes")
    change = (angles - initial + math.pi) % (2 * math.pi) - math.pi
    if np.max(np.abs(change[:2])) > math.radians(25):
        raise ValueError(
            "Tilt correction exceeds 25 degrees; check joint zeros and camera prior"
        )
    xyz = np.asarray(prior_xyz, dtype=float).copy()
    if xyz.shape != (3,) or not np.isfinite(xyz).all():
        raise ValueError("Invalid camera translation")
    xyz[2] = (camera_offset - body_offset - body_normal[:2] @ xyz[:2]) / body_normal[2]
    if abs(xyz[2] - prior_xyz[2]) > 0.25:
        raise ValueError(
            "Height correction exceeds 25 cm; check leg geometry and joint zeros"
        )
    return {
        "body_from_camera_wxyz": quaternion(angles).tolist(),
        "body_to_camera_xyz": xyz.tolist(),
        "roll_pitch_yaw_degrees": np.rad2deg(angles).tolist(),
        "estimated": ["roll", "pitch", "z"],
        "retained": ["x", "y", "yaw"],
    }


def foot_frames(model_xml):
    """Use the selected saved model's foot geometry and existing bridge TF names."""
    root = ET.fromstring(model_xml)
    bodies = list(root.findall("./worldbody//body"))
    base = next(
        (i for i, b in enumerate(bodies, 1) if b.get("name") == "quadruped"), None
    )
    if base is None:
        raise ValueError("Saved model has no quadruped body")
    found = {}
    for i, body in enumerate(bodies, 1):
        for geom in body.findall("geom"):
            for leg in ("fl", "fr", "rl", "rr"):
                if geom.get("name") == f"{leg}_foot":
                    if geom.get("type") != "sphere":
                        raise ValueError("Calibration requires spherical foot geometry")
                    position = [float(v) for v in geom.get("pos", "0 0 0").split()]
                    radius = float(geom.get("size", "nan").split()[0])
                    if (
                        len(position) != 3
                        or not np.isfinite(position).all()
                        or not math.isfinite(radius)
                        or radius <= 0
                    ):
                        raise ValueError("Invalid foot geometry")
                    found[leg] = (
                        f"robot/body_{i}_{body.get('name')}",
                        position,
                        radius,
                    )
    if set(found) != {"fl", "fr", "rl", "rr"}:
        raise ValueError("Saved model must contain all four feet")
    return f"robot/body_{base}_quadruped", [found[k] for k in ("fl", "fr", "rl", "rr")]


def depth_points(image, info, stride=8):
    """Backproject a rectified metric depth image; honor row padding and endianness."""
    if (
        image.encoding != "32FC1"
        or image.width != info.width
        or image.height != info.height
        or image.header.frame_id != info.header.frame_id
    ):
        raise ValueError("Need aligned 32FC1 depth and camera calibration")
    if image.step < image.width * 4 or len(image.data) < image.step * image.height:
        raise ValueError("Malformed depth image")
    fx, fy, cx, cy = info.p[0], info.p[5], info.p[2], info.p[6]
    if not all(math.isfinite(v) for v in (fx, fy, cx, cy)) or min(fx, fy) <= 0:
        raise ValueError("Invalid rectified camera intrinsics")
    depth = np.ndarray(
        (image.height, image.width),
        dtype=">f4" if image.is_bigendian else "<f4",
        buffer=bytes(image.data),
        strides=(image.step, 4),
    )[::stride, ::stride]
    v, u = np.mgrid[0 : image.height : stride, 0 : image.width : stride]
    valid = np.isfinite(depth) & (depth > 0.15) & (depth < 3)
    z = depth[valid]
    return np.column_stack(((u[valid] - cx) * z / fx, (v[valid] - cy) * z / fy, z))


def load_calibration(path, model_xml=None):
    data = json.loads(Path(path).expanduser().read_text())
    if data.get("schema_version") != 1:
        raise ValueError("Unsupported camera calibration file")
    if (
        model_xml is not None
        and data.get("model_sha256") != hashlib.sha256(model_xml.encode()).hexdigest()
    ):
        raise ValueError(
            "Camera calibration was measured with different robot geometry; recalibrate"
        )
    parameters = data["parameters"]
    q, xyz = parameters["body_from_camera_wxyz"], parameters["body_to_camera_xyz"]
    rotation(q)
    if len(xyz) != 3 or not np.isfinite(xyz).all():
        raise ValueError("Invalid saved camera translation")
    return {
        "body_from_camera_wxyz": (np.asarray(q) / np.linalg.norm(q)).tolist(),
        "body_to_camera_xyz": list(xyz),
    }


def save_calibration(path, result, model_xml):
    """Atomically save mount parameters only; world ground height is session-specific."""
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "schema_version": 1,
        "model_sha256": hashlib.sha256(model_xml.encode()).hexdigest(),
        "parameters": {
            k: result[k] for k in ("body_from_camera_wxyz", "body_to_camera_xyz")
        },
        "report": result,
    }
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", dir=path.parent, delete=False
        ) as stream:
            temporary = stream.name
            json.dump(data, stream, indent=2, allow_nan=False)
            stream.write("\n")
        load_calibration(temporary)
        os.replace(temporary, path)
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)
