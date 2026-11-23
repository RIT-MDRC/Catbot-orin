"""Depth-derived height of the horizontal model floor beneath the robot."""

import math
from collections import deque
from statistics import median

import numpy as np

from catbot_sim.camera_calibration import depth_points, fit_ground, rotation


def ground_updates_allowed(status, age):
    """Only refine an established reference while the Pi confirms policy is stopped."""
    return (
        isinstance(status, dict)
        and 0 <= age <= 0.5
        and status.get("orin_policy_active") is False
        and status.get("mode") not in ("running", "ready")
        and status.get("script") == ""
    )


def ground_reference(
    image, info, world_quaternion, world_position, body_position, gravity_up=None
):
    """Locate a level floor in world coordinates without a supplied camera height."""
    world_rotation = rotation(world_quaternion)
    world_position = np.asarray(world_position, dtype=float)
    body_position = np.asarray(body_position, dtype=float)
    if (
        world_position.shape != (3,)
        or body_position.shape != (3,)
        or not np.isfinite(world_position).all()
        or not np.isfinite(body_position).all()
    ):
        raise ValueError("Invalid floor reference position")
    points = depth_points(image, info)
    world = points @ world_rotation.T + world_position
    points = points[
        (world[:, 2] < world_position[2] - 0.08)
        & (np.abs(world[:, 1] - body_position[1]) < 1.5)
    ]
    up = np.asarray([0.0, 0.0, 1.0] if gravity_up is None else gravity_up, dtype=float)
    if up.shape != (3,) or not np.isfinite(up).all() or np.linalg.norm(up) < 1e-6:
        raise ValueError("Invalid IMU gravity anchor")
    up /= np.linalg.norm(up)
    normal, offset, quality = fit_ground(points, world_rotation.T @ up)
    if gravity_up is not None:
        normal = world_rotation.T @ up
        offset = -float(np.median(points @ normal))
        residual = np.abs(points @ normal + offset)
        quality = {
            "inlier_fraction": float(np.mean(residual < 0.012)),
            "rms_m": float(np.sqrt(np.mean(np.minimum(residual, 0.1) ** 2))),
        }
    if quality["inlier_fraction"] < 0.8 or quality["rms_m"] > 0.015:
        raise ValueError("Marginal or occluded floor; holding previous reference")
    world_normal = world_rotation @ normal
    if world_normal[2] < math.cos(math.radians(10)):
        raise ValueError("Observed floor is not approximately level")
    world_offset = offset - float(world_normal @ world_position)
    height_offset = float(
        (world_offset + world_normal[:2] @ body_position[:2]) / world_normal[2]
    )
    quality.update(
        camera_height_m=float(offset / world_normal[2]),
        tilt_degrees=math.degrees(math.acos(float(np.clip(world_normal[2], -1, 1)))),
    )
    return height_offset, quality


class GroundFilter:
    """Hold the last confident plane, reject jumps, median then EMA accepted fits."""

    def __init__(self, window=5, alpha=0.2, max_jump=0.05):
        if window < 1 or not 0 < alpha <= 1 or max_jump <= 0:
            raise ValueError("Invalid ground filter settings")
        self.values = deque(maxlen=window)
        self.alpha, self.max_jump = alpha, max_jump
        self.height = None
        self.confidence = 0.0

    def update(self, height, quality):
        confidence = quality["inlier_fraction"]
        if (
            not math.isfinite(height)
            or not math.isfinite(confidence)
            or confidence < 0.8
            or quality["rms_m"] > 0.015
        ):
            raise ValueError("Low confidence ground fit")
        if self.height is not None and abs(height - self.height) > self.max_jump:
            raise ValueError("Ground height jump rejected")
        self.values.append(height)
        target = median(self.values)
        self.height = (
            target
            if self.height is None
            else (1 - self.alpha) * self.height + self.alpha * target
        )
        self.confidence = confidence
        return self.height
