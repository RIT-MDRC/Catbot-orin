from types import SimpleNamespace as NS

from catbot_sim.sensor_timing import align_samples, stamp_seconds


def sample(stamp, received, *, joint=False):
    value = NS(sec=int(stamp), nanosec=round((stamp % 1) * 1e9))
    return (NS(stamp=value) if joint else NS(header=NS(stamp=value))), received


def test_delayed_odometry_pairs_with_historical_imu_and_joints():
    imu, joint = sample(100, 100), sample(100.005, 100.005, joint=True)
    latest = {
        "odom": sample(100, 100.15),
        "imu": sample(100.15, 100.15),
        "joints": sample(100.15, 100.15, joint=True),
    }
    aligned = align_samples(
        latest,
        {"imu": [imu, latest["imu"]], "joints": [joint, latest["joints"]]},
        100.16,
        0.25,
    )
    assert aligned["imu"] is imu
    assert aligned["joints"] is joint
    assert latest["imu"] is not imu
    assert (
        abs(stamp_seconds(aligned["imu"][0]) - stamp_seconds(aligned["joints"][0]))
        < 0.01
    )


def test_expired_history_and_clock_jumps_do_not_supply_old_measurements():
    latest = {"odom": sample(100, 100.15), "imu": sample(100.15, 100.15)}
    for old in [sample(100, 99), sample(90, 100.15), sample(100, 101)]:
        assert (
            align_samples(latest, {"imu": [old]}, 100.16, 0.25)["imu"] == latest["imu"]
        )


def test_startup_without_odometry_keeps_existing_missing_input_gate():
    assert align_samples({}, {}, 100, 0.25) == {}


def test_duplicate_capture_stamp_uses_the_latest_received_measurement():
    first, last = sample(100, 100), sample(100, 100.01)
    assert (
        align_samples(
            {"odom": sample(100, 100.02)}, {"imu": [first, last]}, 100.03, 0.25
        )["imu"]
        is last
    )


def test_history_cannot_hide_a_clock_jump_in_the_latest_joint_report():
    old = sample(100, 100, joint=True)
    jumped = sample(90, 100.02, joint=True)
    result = align_samples(
        {"odom": sample(100, 100.02), "joints": jumped},
        {"joints": [old, jumped]},
        100.03,
        0.25,
    )
    assert result["joints"] is jumped
