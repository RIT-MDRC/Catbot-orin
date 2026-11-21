# Tests

Run `uv run --locked --project external/Catbot/tests python -m pytest -m 'not ros'`
for pure scenario helpers, randomized settings validation, mapping contract and
real subprocess supervision checks. CI also runs Catbot's behavioral and policy
suites from the pinned submodule and enforces its per-file coverage floors.

The ROS job uses normal colcon discovery to build the workspace, including the
shared packages linked from `src/` into the pinned Catbot source. It checks that
colcon discovers exactly one copy of each shared package at its `src/` path,
then runs `pytest -m ros tests/test_ros_scenario.py` with pytest
plugin autoload disabled. It launches the real `simulation_orin.launch.py` entry
point with camera/Foxglove disabled and the shared stdlib fake worker replacing
both external workers. It requires both adapter nodes, `/policy/contract`, and
actuator commands matching the shared fixture, without a Pi protocol peer or
policy-start request. No hardware or model artifacts are required. Missing ROS
bindings fail this job instead of skipping it.

Unit launch tests use `fixtures/launch_defaults.yaml` and explicit CLI overrides,
then check typed node parameters and local/remote placement. Mapping and SSH tests
also supply controlled inputs; deployment tuning in `scenarios.yaml` and robot
calibration should not change their expectations. Assert resulting behavior
rather than reproducing parameter tables or counting internal resources.

The single golden file lives in `external/Catbot/tests/fixtures/contract.json`.
Do not copy it here. CI requires 51% coverage for scenarios.py and 95% separately
for worker_process.py, mapping.py, config.py, remote.py, remote_supervisor.py, and
bridge.py. Each module is explicitly included in collection, so an unimported
module cannot disappear from the report. The shared check_coverage.py reads these
floors from the workflow arguments; it does not hard-code module names.

Bridge unit tests double the ROS boundaries and exercise initialization, command
validation, feedback, watchdogs, metrics and worker loops. The separate ROS job
checks actual message types and launch behavior. Supervisor tests cover lease
expiry/renewal, disconnect, shutdown signals, escalation and descendant cleanup
after the launch leader exits. Real local process tests verify workers are reaped
after lease expiry, disconnect, signal shutdown and SIGKILL escalation, including
a stubborn descendant surviving its launch leader. SSH and target hardware are
not required.

The Orin-hosted scenario checks require the `catbot_supervisor` package from the
pinned Catbot dependency. The physical Orin ROS scenario test uses synthetic
sensor input and a Pi protocol peer to exercise
probe/clock replies, preflight, explicit policy start, forwarding acknowledgements,
verified readiness, telemetry tags and the status service. It does not launch Pi
hardware or introduce a separate joint-repository test framework. The physical
Orin launch starts the Pi services over SSH by default, with `start_pi:=false`
preserving separate Pi management. The ROS test covers both paths, replacing SSH
with a lease-reading stand-in so no hardware connection is made. Remote command
tests execute the shared-environment/ROS-overlay chain with a harmless ros2 probe;
the existing process-group tests cover lease expiry and descendant cleanup.

The ROS mount test uses a vertical, downward-tilted camera, an independent factory
IMU rotation, and a nonzero lever arm. It checks body pose/velocity, refuses
missing IMU TF, and resolves the native cloud frame against the model body frame
through their common gravity-aligned map root. It covers camera-aligned and
gravity-aligned odometry origins, requires map-to-odom TF, and preserves real
body tilt while correcting the reference axes. Mount inputs are explicit test values;
deployment offsets can be tuned without changing the test.
