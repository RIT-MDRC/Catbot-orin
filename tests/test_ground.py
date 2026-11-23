"""The observed floor locates model zero without a manual camera-height input."""

from types import SimpleNamespace

import numpy as np
import pytest
from catbot_sim.ground import ground_reference, ground_updates_allowed


def depth_sample(distance):
    header = SimpleNamespace(frame_id="optical")
    image = SimpleNamespace(
        width=128,
        height=128,
        step=512,
        encoding="32FC1",
        is_bigendian=False,
        header=header,
        data=np.full((128, 128), distance, dtype="<f4").tobytes(),
    )
    info = SimpleNamespace(
        width=128,
        height=128,
        header=header,
        p=[100.0, 0.0, 64.0, 0.0, 0.0, 100.0, 64.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    )
    return image, info


@pytest.mark.parametrize(
    "camera_z,distance", [(-0.17, 0.43), (-0.37, 0.23), (0.2, 0.8)]
)
def test_detected_floor_stays_at_world_minus_point_six(camera_z, distance):
    image, info = depth_sample(distance)
    height, quality = ground_reference(
        image, info, [0, 1, 0, 0], [0.4, 0, camera_z], [0, 0, 0]
    )
    assert height == pytest.approx(0.6, abs=1e-6)
    assert quality["camera_height_m"] == pytest.approx(distance, abs=1e-6)
    assert quality["inlier_fraction"] == 1
    assert quality["rms_m"] < 1e-6


def test_invalid_depth_cannot_supply_a_ground_reference():
    image, info = depth_sample(float("nan"))
    with pytest.raises(ValueError):
        ground_reference(image, info, [0, 1, 0, 0], [0, 0, 0], [0, 0, 0])
    image, info = depth_sample(0.4)
    with pytest.raises(ValueError):
        ground_reference(image, info, [1, 0, 0, 0], [0, 0, 0], [0, 0, 0])


@pytest.mark.parametrize(
    "changes,age,expected",
    [
        ({}, 0.1, True),
        ({"orin_policy_active": True}, 0.1, False),
        ({"mode": "running"}, 0.1, False),
        ({"script": "home_robot"}, 0.1, False),
        ({}, 0.6, False),
        ({}, -0.1, False),
    ],
)
def test_ground_refinement_requires_fresh_explicit_stopped_confirmation(
    changes, age, expected
):
    status = {"mode": "holding", "orin_policy_active": False, "script": "", **changes}
    assert ground_updates_allowed(status, age) is expected
    assert not ground_updates_allowed({}, age)


def test_filter_holds_on_occlusion_and_jumps_and_smooths_noise():
    from catbot_sim.ground import GroundFilter

    filter = GroundFilter(window=3, alpha=0.5, max_jump=0.05)
    quality = {"inlier_fraction": 0.95, "rms_m": 0.002}
    assert filter.update(0.4, quality) == 0.4
    assert 0.4 < filter.update(0.42, quality) < 0.42
    previous = filter.height
    with pytest.raises(ValueError, match="jump"):
        filter.update(0.6, quality)
    with pytest.raises(ValueError, match="confidence"):
        filter.update(0.41, {**quality, "inlier_fraction": 0.6})
    assert filter.height == previous


@pytest.mark.parametrize(
    "rms,confidence", [(float("nan"), 0.9), (-0.1, 0.9), (0.0, 1.1)]
)
def test_filter_rejects_invalid_quality_without_mutating_state(rms, confidence):
    from catbot_sim.ground import GroundFilter

    filter = GroundFilter()
    with pytest.raises(ValueError):
        filter.update(0.4, {"rms_m": rms, "inlier_fraction": confidence})
    assert filter.height is None and not filter.values


def test_gravity_constraint_uses_fixed_normal_without_ransac(monkeypatch):
    from catbot_sim import ground

    monkeypatch.setattr(
        ground, "fit_ground", lambda *args: pytest.fail("redundant RANSAC")
    )
    image, info = depth_sample(0.4)
    height, quality = ground.ground_reference(
        image, info, [0, 1, 0, 0], [0, 0, 0], [0, 0, 0], gravity_up=[0, 0, 1]
    )
    assert height == pytest.approx(0.4)
    assert quality["inlier_fraction"] == 1


def test_fixed_normal_fit_rejects_occlusion_and_narrow_support(monkeypatch):
    from catbot_sim import ground

    image, info = depth_sample(0.4)
    depth = np.full((128, 128), 0.4, dtype="<f4")
    depth[:32] = 0.25
    image.data = depth.tobytes()
    with pytest.raises(ValueError, match="occluded"):
        ground.ground_reference(
            image, info, [0, 1, 0, 0], [0, 0, 0], [0, 0, 0], gravity_up=[0, 0, 1]
        )
    points = np.column_stack(
        [np.linspace(-0.2, 0.2, 200), np.zeros(200), np.full(200, 0.4)]
    )
    monkeypatch.setattr(ground, "depth_points", lambda *_: points)
    with pytest.raises(ValueError, match="span a plane"):
        ground.ground_reference(
            image, info, [0, 1, 0, 0], [0, 0, 0], [0, 0, 0], gravity_up=[0, 0, 1]
        )


def test_gravity_input_is_not_modified_by_normalization():
    image, info = depth_sample(0.4)
    up = np.array([0.0, 0.0, 9.81])
    up.setflags(write=False)
    height, _ = ground_reference(
        image, info, [0, 1, 0, 0], [0, 0, 0], [0, 0, 0], gravity_up=up
    )
    assert height == pytest.approx(0.4)
    assert up[2] == 9.81


def test_floor_neighborhood_excludes_distant_clutter_but_rejects_local_occlusion(
    monkeypatch,
):
    from catbot_sim import ground

    image, info = depth_sample(0.4)
    x, y = np.meshgrid(np.linspace(-0.3, 0.3, 20), np.linspace(-0.3, 0.3, 20))
    floor = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, 0.4)))
    clutter = floor.copy()
    clutter[:, 0] += 1.5
    clutter[:, 2] = 0.2
    monkeypatch.setattr(ground, "depth_points", lambda *_: np.vstack((floor, clutter)))
    height, quality = ground.ground_reference(
        image, info, [0, 1, 0, 0], [0, 0, 0], [0, 0, 0], gravity_up=[0, 0, 1]
    )
    assert height == pytest.approx(0.4)
    assert quality["inlier_fraction"] == 1
    clutter[:, 0] -= 1.5
    with pytest.raises(ValueError):
        ground.ground_reference(
            image, info, [0, 1, 0, 0], [0, 0, 0], [0, 0, 0], gravity_up=[0, 0, 1]
        )
