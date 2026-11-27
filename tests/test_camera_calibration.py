"""Recover known geometry and reject unobservable, noisy or malformed captures."""

import json
import math
from types import SimpleNamespace

import numpy as np
import pytest
from catbot_sim.camera_calibration import (
    depth_points,
    estimate_mount,
    fit_ground,
    foot_frames,
    foot_plane,
    load_calibration,
    measured_plane_offset,
    plane,
    quaternion,
    roll_pitch_yaw,
    rotation,
    save_calibration,
)


@pytest.mark.parametrize(
    "roll,pitch,yaw", [(-90, 30, 0), (-87, 27, 15), (175, -8, -20)]
)
def test_recovers_mount_tilt_and_height_preserving_unobservable_components(
    roll, pitch, yaw
):
    true_q = quaternion(np.deg2rad([roll, pitch, yaw]))
    body_normal = np.array([0.08, -0.04, 1.0])
    body_normal /= np.linalg.norm(body_normal)
    camera_normal = rotation(true_q).T @ body_normal
    true_xyz = [0.43, -0.02, 0.07]
    offset = 0.36 + body_normal @ true_xyz
    result = estimate_mount(
        (camera_normal, offset),
        (body_normal, 0.36),
        quaternion(np.deg2rad([roll + 2, pitch - 3, yaw])),
        [*true_xyz[:2], 0.02],
    )
    assert result["roll_pitch_yaw_degrees"] == pytest.approx(
        [roll, pitch, yaw], abs=1e-5
    )
    assert result["body_to_camera_xyz"] == pytest.approx(true_xyz, abs=1e-6)
    assert result["estimated"] == ["roll", "pitch", "z"]
    assert result["retained"] == ["x", "y", "yaw"]


def test_floor_cannot_reveal_x_y_or_yaw():
    # Horizontal ground has identical camera observations for any horizontal
    # translation and heading. The result must follow the prior, not invent them.
    for xyz, yaw in [([0.2, -0.1, 0], 0), ([0.6, 0.15, 0], 40)]:
        result = estimate_mount(
            ([0, 0, 1], 0.42),
            ([0, 0, 1], 0.35),
            quaternion(np.deg2rad([0, 0, yaw])),
            xyz,
        )
        assert result["body_to_camera_xyz"] == pytest.approx([*xyz[:2], 0.07])
        assert result["roll_pitch_yaw_degrees"][2] == pytest.approx(yaw)


def test_measured_center_height_accounts_for_reference_point_and_body_height():
    # Camera link is upside down; measured center is 4 cm above that link.
    offset = measured_plane_offset(
        [0, 0, -1], np.diag([1, -1, -1]), [0, 0, -0.04], 0.44
    )
    assert offset == pytest.approx(0.40)
    result = estimate_mount(
        ([0, 0, -1], offset), ([0, 0, 1], 0.35), [0, 1, 0, 0], [0.4, 0, 0]
    )
    assert result["body_to_camera_xyz"] == pytest.approx([0.4, 0, 0.05])
    with pytest.raises(ValueError):
        measured_plane_offset([0, 0, -1], np.eye(3), [0, 0, 0], 0.44)


def test_spherical_feet_locate_ground_below_centers():
    feet = [
        [x, y, -0.33] for x, y in [(-0.3, -0.2), (-0.3, 0.2), (0.3, -0.2), (0.3, 0.2)]
    ]
    normal, offset = foot_plane(feet, [0.02] * 4)
    assert normal == pytest.approx([0, 0, 1])
    assert offset == pytest.approx(0.35)
    feet[0][2] += 0.08
    with pytest.raises(ValueError, match="Four feet"):
        foot_plane(feet, [0.02] * 4)


def test_ransac_rejects_obstacles_and_estimates_noisy_floor():
    rng = np.random.default_rng(4)
    xy = rng.uniform(-0.7, 0.7, (1000, 2))
    floor = np.column_stack([xy, -0.4 + rng.normal(0, 0.002, 1000)])
    outliers = rng.uniform(-0.8, 0.8, (250, 3))
    normal, offset, quality = fit_ground(np.vstack([floor, outliers]), [0, 0, 1])
    assert normal == pytest.approx([0, 0, 1], abs=0.002)
    assert offset == pytest.approx(0.4, abs=0.001)
    assert quality["inlier_fraction"] > 0.79
    assert quality["rms_m"] < 0.003


@pytest.mark.parametrize(
    "points",
    [
        np.zeros((20, 3)),
        np.array([[x, 0, -0.4] for x in np.linspace(-1, 1, 200)]),
        np.array(
            [
                [0.5, y, z]
                for y in np.linspace(-1, 1, 15)
                for z in np.linspace(-1, 1, 15)
            ]
        ),
    ],
)
def test_floor_rejects_insufficient_collinear_and_wall_only_points(points):
    with pytest.raises(ValueError):
        fit_ground(points, [0, 0, 1])


@pytest.mark.parametrize(
    "q,xyz",
    [
        ([0, 0, 0, 0], [0, 0, 0]),
        (quaternion([0, math.pi / 2, 0]), [0, 0, 0]),
        (quaternion([0, 0, 0]), [0, 0, -0.3]),
    ],
)
def test_rejects_invalid_or_large_mount_correction(q, xyz):
    with pytest.raises(ValueError):
        estimate_mount(([0, 0, 1], 0.4), ([0, 0, 1], 0.35), q, xyz)


@pytest.mark.parametrize("bigendian", [False, True])
def test_depth_backprojection_uses_calibration_padding_endianness_and_invalid_filter(
    bigendian,
):
    depth = np.array(
        [[1, np.nan, 2], [0, 0.1, np.inf]], dtype=">f4" if bigendian else "<f4"
    )
    header = SimpleNamespace(frame_id="optical")
    image = SimpleNamespace(
        encoding="32FC1",
        width=3,
        height=2,
        header=header,
        step=16,
        data=b"".join(row.tobytes() + b"pad!" for row in depth),
        is_bigendian=bigendian,
    )
    info = SimpleNamespace(
        width=3, height=2, header=header, p=[2, 0, 1, 0, 0, 2, 0.5, 0, 0, 0, 1, 0]
    )
    assert depth_points(image, info, stride=1) == pytest.approx(
        np.array([[-0.5, -0.25, 1], [1, -0.5, 2]])
    )
    image.encoding = "16UC1"
    with pytest.raises(ValueError, match="32FC1"):
        depth_points(image, info)


def model_xml():
    return (
        '<mujoco><worldbody><body name="quadruped">'
        + "".join(
            f'<body name="{leg}_shin"><geom name="{leg}_foot" type="sphere" pos="0 0 -0.3" size="0.02"/></body>'
            for leg in ("fl", "fr", "rl", "rr")
        )
        + "</body></worldbody></mujoco>"
    )


def test_uses_saved_model_geometry_and_existing_tf_names():
    body, feet = foot_frames(model_xml())
    assert body == "robot/body_1_quadruped"
    assert feet == [
        (f"robot/body_{i}_{leg}_shin", [0, 0, -0.3], 0.02)
        for i, leg in enumerate(("fl", "fr", "rl", "rr"), 2)
    ]
    with pytest.raises(ValueError, match="all four"):
        foot_frames(model_xml().replace('name="rr_foot"', 'name="missing"'))


def test_save_load_mount_only_rejects_geometry_change_and_preserves_existing_on_failure(
    tmp_path,
):
    path = tmp_path / "config" / "camera.json"
    result = {
        "body_from_camera_wxyz": [1, 0, 0, 0],
        "body_to_camera_xyz": [0.4, 0, 0.07],
        "ground_height_offset_at_capture": 0.32,
    }
    save_calibration(path, result, model_xml())
    assert load_calibration(path, model_xml()) == {
        k: result[k] for k in ("body_from_camera_wxyz", "body_to_camera_xyz")
    }
    with pytest.raises(ValueError, match="different robot geometry"):
        load_calibration(path, model_xml() + " ")
    before = path.read_bytes()
    with pytest.raises(ValueError):
        save_calibration(path, {**result, "bad": float("nan")}, model_xml())
    assert path.read_bytes() == before
    assert list(path.parent.iterdir()) == [path]
    data = json.loads(path.read_text())
    data["parameters"]["body_from_camera_wxyz"] = [0, 0, 0, 0]
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_calibration(path)


def test_known_rotation_maps_camera_axes_and_roundtrips():
    q = quaternion([math.pi / 2, 0, 0])
    assert rotation(q) @ [0, 1, 0] == pytest.approx([0, 0, 1], abs=1e-12)
    assert roll_pitch_yaw(q) == pytest.approx([math.pi / 2, 0, 0])
    with pytest.raises(ValueError):
        plane([[float("nan"), 0, 0]] * 4)
