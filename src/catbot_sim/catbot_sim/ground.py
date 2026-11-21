"""Depth-derived height of the horizontal model floor beneath the robot."""

import math

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


def ground_reference(image, info, world_quaternion, world_position, body_position):
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
    normal, offset, quality = fit_ground(
        points, world_rotation.T @ np.array([0.0, 0.0, 1.0])
    )
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
