import json
from pathlib import Path

import pytest
from catbot_policy.protocol import OBSERVATION_FIELDS, validate_feedback
from catbot_sim.mapping import apply_positions

FIXTURE = (
    Path(__file__).resolve().parents[1] / "external/Catbot/tests/fixtures/contract.json"
)


def test_shared_golden_contract():
    fixture = json.loads(FIXTURE.read_text())
    assert [list(x) for x in OBSERVATION_FIELDS] == fixture["layout"]
    assert validate_feedback(fixture["feedback"]) == fixture["feedback"]
    mapping = fixture["mapping"]
    indices = {name: i for i, name in enumerate(mapping["actuator_names"])}
    result = apply_positions(
        mapping, indices, None, mapping["actuator_names"], fixture["degrees"]
    )
    assert result == pytest.approx(fixture["inverse_radians"])
    previous = result.copy()
    with pytest.raises(ValueError):
        apply_positions(
            mapping, indices, previous, mapping["actuator_names"], [1e10] * 12
        )
    assert previous == result


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -1e10, 1e10])
def test_invalid_sparse_target_is_atomic(value):
    mapping = json.loads(FIXTURE.read_text())["mapping"]
    names = mapping["actuator_names"]
    indices = {name: i for i, name in enumerate(names)}
    previous = [0.1] * 12
    with pytest.raises(ValueError, match="target"):
        apply_positions(mapping, indices, previous, names[:2], [0, value])
    assert previous == [0.1] * 12


def test_unknown_actuator_is_rejected():
    mapping = json.loads(FIXTURE.read_text())["mapping"]
    with pytest.raises(ValueError, match="Unknown actuator"):
        apply_positions(mapping, {}, None, ["unknown"], [0])
