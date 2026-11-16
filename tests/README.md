# Tests

Run `uv run --locked --project external/Catbot/tests python -m pytest -m 'not ros'`
for pure scenario helpers, randomized settings validation, mapping contract and
real subprocess supervision checks. CI also runs Catbot's behavioral and policy
suites from the pinned submodule and enforces its per-file coverage floors.

The ROS job builds `catbot_interfaces`, `catbot_policy`, `catbot_sim`, and
`catbot_launcher`, then runs `pytest -m ros tests/test_ros_scenario.py` with pytest
plugin autoload disabled. It launches the real `simulation_orin.launch.py` entry
point with camera/Foxglove disabled and the shared stdlib fake worker replacing
both external workers. It requires both adapter nodes, `/policy/contract`, and
actuator commands matching the shared fixture. No hardware or model artifacts
are required. Missing ROS bindings fail this job instead of skipping it.

The single golden file lives in `external/Catbot/tests/fixtures/contract.json`.
Do not copy it here. The 51% scenarios.py floor measures the pure paths; ROS launch
assembly is checked separately by the scenario integration test.
