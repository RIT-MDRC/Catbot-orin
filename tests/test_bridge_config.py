import json
from pathlib import Path

import pytest
from catbot_sim.config import BridgeConfig


@pytest.fixture
def parameters(tmp_path):
    (tmp_path / "pyproject.toml").touch()
    worker = tmp_path / "src/catbot_rollout/worker.py"
    worker.parent.mkdir(parents=True)
    worker.touch()
    mapping = tmp_path / "mapping.json"
    mapping.write_text(
        json.dumps(
            {
                "actuator_names": [f"joint_{i}" for i in range(12)],
                "signs": [1, -1] * 6,
                "offsets_degrees": list(range(12)),
                "minimum_degrees": [-60] * 12,
                "maximum_degrees": [75] * 12,
            }
        )
    )
    return {
        "rollout_repository": str(tmp_path),
        "mapping_file": str(mapping),
        "run_directory": str(tmp_path / "saved run"),
    }


@pytest.mark.parametrize("backend", ["simulation", "hardware"])
def test_configuration_preserves_settings(parameters, backend):
    parameters.update(
        backend=backend,
        worker_environment="/env",
        worker_sync=False,
        device="cuda:0",
        uv_executable="/uv",
        worker_timeout=1.75,
        visualization_rate=17.0,
        command_timeout=0.4,
        startup_timeout=45.0,
    )
    config = BridgeConfig.from_parameters(
        lambda name, default: parameters.get(name, default)
    )
    assert config.backend == backend
    assert config.repository == Path(parameters["rollout_repository"]).resolve()
    assert config.run == Path(parameters["run_directory"]).resolve()
    assert (
        config.worker_timeout,
        config.visualization_rate,
        config.timeout,
        config.startup_timeout,
    ) == (
        1.75,
        17.0,
        0.4,
        45.0,
    )
    assert config.mapping == json.loads(Path(parameters["mapping_file"]).read_text())
    assert (config.environment, config.sync, config.device, config.uv) == (
        "/env",
        False,
        "cuda:0",
        "/uv",
    )


def test_invalid_backend(parameters):
    parameters["backend"] = "invalid"
    with pytest.raises(ValueError, match="backend"):
        BridgeConfig.from_parameters(
            lambda name, default: parameters.get(name, default)
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
