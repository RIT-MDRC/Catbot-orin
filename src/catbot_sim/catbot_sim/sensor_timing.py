"""Pair delayed camera poses with measurements captured at the same time."""


def stamp_seconds(message):
    stamp = message.header.stamp if hasattr(message, "header") else message.stamp
    return stamp.sec + stamp.nanosec * 1e-9


def align_samples(samples, history, now, timeout):
    """Keep odometry as the reference; existing snapshot gates validate the pair.

    Receipt freshness alone is insufficient: also bound capture age relative to
    the latest measurement so clock jumps cannot select an ancient sample.
    """
    if "odom" not in samples:
        return samples
    target = stamp_seconds(samples["odom"][0])
    result = dict(samples)
    for name in ("imu", "joints"):
        # A fresh report from a different clock epoch must remain visible to
        # snapshot validation, even if older history still matches odometry.
        if name in samples and abs(stamp_seconds(samples[name][0]) - target) > timeout:
            continue
        candidates = [
            sample
            for sample in history.get(name, ())
            if 0 <= now - sample[1] <= timeout
            and abs(stamp_seconds(sample[0]) - target) <= timeout
        ]
        if candidates:
            result[name] = min(
                candidates,
                key=lambda sample: (abs(stamp_seconds(sample[0]) - target), -sample[1]),
            )
    return result
