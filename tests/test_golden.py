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
