# Catbot Orin deployment

## Ownership and dependencies

This repository owns scenario launch/orchestration, sensing and diagnostics,
remote supervision, and ROS simulation/kinematics bridges. `external/Catbot` is a
pinned Git submodule providing messages, policy runtime, and the shared golden
contract. It is separate from a sibling `catbot` checkout. Inspect nested status
and gitlinks before editing or updating submodules; preserve existing edits.
CI checks out the gitlink commit, not the submodule's configured branch tip.

`src/catbot_interfaces`, `src/catbot_policy`, `src/foundation`, and
`src/catbot_supervisor` link to those packages in `external/Catbot`. Keep one
source copy and use normal colcon discovery from this workspace root in both
deployment and CI. Do not bypass these links with external `--base-paths` in a
shared build directory: CMake caches the source path, including this choice.

`external/catbot_sim_rollout` is a separately pinned simulation worker whose uv
dependency pins a training revision. It does not automatically use the sibling
`catbot_mujoco` working tree. Check the worker and policy/run artifacts actually
selected by the scenario before changing their dependencies.

## Deployment boundaries

- ROS Humble adapters use its Python 3.10 ABI. CPU inference and simulation use
  separate Python 3.14 environments; Jetson CUDA inference uses a separately
  provisioned environment. Do not combine them or move ROS to the training Python.
- Physical and simulated paths share named actuator commands and policy feedback.
  Preserve units, coordinate frames, sparse-command atomic validation, timeout
  behavior, and contract/model validation across both paths.
- Use the authoritative fixture at `external/Catbot/tests/fixtures/contract.json`;
  do not create an Orin copy. Coordinate shared interface changes in Catbot first.
- Scenario defaults live in `src/catbot_launcher/config/scenarios.yaml`; trace
  launch and remote overrides before changing placement or paths. Policy selection
  uses local prepared artifacts. Download/preparation may mutate the selected
  worker's dependency/lock; it is not a read-only compatibility check.

## Required verification

Read `.github/workflows/policy.yml` and `tests/README.md` before changing code.
Run its complete required checks against the intended combined tree:

- `unit`: run Catbot's Foundation/UI and policy suites from `external/Catbot`,
  including its coverage floors, then Orin's non-ROS suite from this root using
  `uv run --locked --project external/Catbot/tests`.
- Keep separate coverage floors of 51% for `catbot_launcher/scenarios.py` and
  95% for each listed `catbot_sim` worker/supervisor/mapping/config/remote/bridge
  module. Use the live workflow's module list and commands.
- `ros`: build Catbot interfaces/policy plus the simulation/launcher packages in
  Humble and run both required scenario and diagnostics ROS tests. Disable pytest
  plugin autoload as the workflow specifies; unavailable ROS must not silently skip.

Do not lower floors or weaken checks to obtain green results. Report the exact
unverified job when its environment is unavailable. Unit/ROS CI with fake workers
does not establish SSH, ZED hardware recovery, CUDA parity, or physical calibration.

Use the `catbot-workspace` skill when available for cross-project compatibility
validation. When publishing coordinated changes is in scope, publish dependency
commits before recording their gitlinks here. Verify final CI against those exact
pins; a passing sibling checkout alone does not validate this consumer.
