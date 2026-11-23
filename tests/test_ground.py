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
