import json
from pathlib import Path

import pytest
from catbot_sim.config import BridgeConfig

from catbot_policy import configuration


@pytest.fixture
def parameters(tmp_path):
    (tmp_path / "pyproject.toml").touch()
    worker = tmp_path / "src/catbot_rollout/worker.py"
    worker.parent.mkdir(parents=True)
    worker.touch()
    mapping = tmp_path / "mapping.json"
    mapping.write_text(
        Path(configuration.__file__).with_name("default_mapping.json").read_text()
    )
    return {"rollout_repository": str(tmp_path), "mapping_file": str(mapping)}


@pytest.mark.parametrize("backend", ["simulation", "hardware"])
def test_configuration_preserves_settings(parameters, backend):
    parameters.update(
        backend=backend,
        worker_environment="/env",
        worker_sync=False,
        device="cuda:0",
        uv_executable="/uv",
    )
    config = BridgeConfig.from_parameters(
        lambda name, default: parameters.get(name, default)
    )
    assert config.backend == backend
    assert (config.environment, config.sync, config.device, config.uv) == (
        "/env",
        False,
        "cuda:0",
        "/uv",
    )


@pytest.mark.parametrize(
    "key",
    ["worker_timeout", "visualization_rate", "command_timeout", "startup_timeout"],
)
@pytest.mark.parametrize("value", [0, -1, float("inf"), float("nan")])
def test_invalid_timing(parameters, key, value):
    parameters[key] = value
    with pytest.raises(ValueError, match="positive and finite"):
        BridgeConfig.from_parameters(
            lambda name, default: parameters.get(name, default)
        )


@pytest.mark.parametrize("missing", ["pyproject.toml", "src/catbot_rollout/worker.py"])
def test_uninitialized_repository(parameters, missing):
    (Path(parameters["rollout_repository"]) / missing).unlink()
    with pytest.raises(ValueError, match="initialized"):
        BridgeConfig.from_parameters(
            lambda name, default: parameters.get(name, default)
        )


def test_invalid_mapping(parameters):
    mapping_file = Path(parameters["mapping_file"])
    mapping = json.loads(mapping_file.read_text())
    mapping["signs"] = [0] * 12
    mapping_file.write_text(json.dumps(mapping))
    with pytest.raises(ValueError):
        BridgeConfig.from_parameters(
            lambda name, default: parameters.get(name, default)
        )
