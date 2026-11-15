"""ROS-independent bridge parameter parsing and validation."""

import json
import math
from dataclasses import dataclass
from pathlib import Path

from catbot_policy.configuration import validate_mapping


@dataclass
class BridgeConfig:
    backend: str = "simulation"
    repository: Path = Path()
    run: Path = Path()
    mapping: dict | None = None
    worker_timeout: float = 2.0
    visualization_rate: float = 30.0
    timeout: float = 0.25
    startup_timeout: float = 120.0
    device: str = "cpu"
    environment: str = ""
    sync: bool | str = True
    uv: str = "uv"

    @classmethod
    def from_parameters(cls, param):
        """Read and validate all bridge settings before creating ROS resources."""
        self = cls()
        self.backend = str(param("backend", "simulation"))
        if self.backend not in ("simulation", "hardware"):
            raise ValueError("backend must be simulation or hardware")
        self.repository = (
            Path(str(param("rollout_repository", ""))).expanduser().resolve()
        )
        if (
            not (self.repository / "pyproject.toml").is_file()
            or not (self.repository / "src/catbot_rollout/worker.py").is_file()
        ):
            raise ValueError(
                "rollout_repository must point to the initialized "
                "external/catbot_sim_rollout submodule"
            )
        self.run = Path(str(param("run_directory", ""))).expanduser().resolve()
        mapping_path = Path(str(param("mapping_file", ""))).expanduser()
        self.mapping = json.loads(mapping_path.read_text())
        validate_mapping(self.mapping)
        self.worker_timeout = float(param("worker_timeout", 2.0))
        self.visualization_rate = float(param("visualization_rate", 30.0))
        self.timeout = float(param("command_timeout", 0.25))
        self.startup_timeout = float(param("startup_timeout", 120.0))
        if not all(
            math.isfinite(v) and v > 0
            for v in (
                self.timeout,
                self.startup_timeout,
                self.worker_timeout,
                self.visualization_rate,
            )
        ):
            raise ValueError("Timeouts must be positive and finite")
        self.device = str(param("device", "cpu"))
        self.environment = str(param("worker_environment", ""))
        self.sync = param("worker_sync", True)
        self.uv = str(param("uv_executable", "uv"))
        return self
