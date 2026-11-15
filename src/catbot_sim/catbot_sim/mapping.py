"""Atomic conversion used by the bridge's actuator command callback."""

import math


def apply_positions(mapping, indices, previous, names, values):
    targets = list(previous) if previous is not None else [0.0] * 12
    for name, value in zip(names, values, strict=True):
        if name not in indices:
            raise ValueError("Unknown actuator name: " + name)
        i = indices[name]
        if (
            not math.isfinite(value)
            or not mapping["minimum_degrees"][i]
            <= value
            <= mapping["maximum_degrees"][i]
        ):
            raise ValueError("Invalid actuator target")
        targets[i] = math.radians(
            (value - mapping["offsets_degrees"][i]) / mapping["signs"][i]
        )
    return targets
