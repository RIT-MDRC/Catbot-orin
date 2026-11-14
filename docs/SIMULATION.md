# Catbot operation and load-testing scenarios

All four commands share `catbot_launcher/scenarios.py` and
`config/scenarios.yaml`. DDS configuration is inherited from the host; nothing
rewrites it. These are mutually exclusive sessions: stop the previous launch and
any independently started policy/foundation nodes before switching scenarios.

## Commands

```bash
# 1. Physical robot: Orin camera/IMU + Foxglove, Pi foundation + policy
ros2 launch catbot_launcher physical_pi.launch.py

# 2. Physical robot: Orin camera/IMU + Foxglove + policy, Pi foundation
ros2 launch catbot_launcher physical_orin.launch.py

# 3. Nonphysical load test: Orin simulation + Foxglove, Pi policy only
ros2 launch catbot_launcher simulation_pi.launch.py

# 4. All-local load test: Orin simulation + Foxglove + policy; no Pi connection
ros2 launch catbot_launcher simulation_orin.launch.py
```

Equivalent: `ros2 launch catbot_launcher scenario.launch.py scenario:=physical_pi`.
`simulation.launch.py` is a compatibility alias for `simulation_pi`. It now starts
no camera and no Pi foundation relay. The original `main.launch.py` remains the
standalone camera/Foxglove launch.

Physical launches start a kinematics/observation adapter on Orin, not a physics
rollout. The adapter needs the same model/configuration as the policy to compute
foot history and render measured joint positions. Simulation launches omit the
camera by default. Every launch includes exactly one Foxglove bridge.

## Defaults and overrides

The checked-in `src/catbot_launcher/config/scenarios.yaml` holds shared values.
Edit it and rebuild, or point `CATBOT_LAUNCH_DEFAULTS` at a customized copy before
launch. Every key is also a launch argument; CLI values override the defaults.
`camera_config` and `enable_camera` are additional arguments, defaulting to the
installed `zed_policy.yaml` and `auto` (on for physical, off for simulation).

Default paths are deliberately concrete and must match your deployment:

- Orin workspace: `/home/catbot/ROS/ros2_ws`; `workspace` changes the default
  `rollout_repository` and `mapping_file` paths together.
- Pi: `catbot@192.168.0.3`, workspace `/home/catbot/Catbot`.
- `run_directory`: `/home/catbot/.local/share/catbot/run`, containing the selected
  run's `config.json` and `model.xml`, on **Orin in every scenario**.
- `policy_bundle`: `/home/catbot/.local/share/catbot/policies/latest`, on Orin for
  scenarios 2/4. `remote_policy_bundle` selects the Pi path for scenarios 1/3.
- `mapping_file`: `{workspace}/default_mapping.json`; `remote_mapping_file` is the
  matching mapping in the Pi checkout. These must describe the same model order,
  signs, offsets and limits. The supplied mapping remains provisional.
- `remote_environment`: `/opt/ros/humble/setup.bash`. Override with your existing
  ROS/DDS environment setup script if its variables aren't in the login shell.
- `policy_python` and `remote_policy_python`: `3.14`. These select the inference
  interpreter through uv (`--python` accepts a version or absolute interpreter path).
- `policy_ros_python` and `remote_policy_ros_python`: `/usr/bin/python3`. These
  select only the ROS adapter interpreter and must match the installed ROS ABI.
- `policy_worker_project` / `remote_policy_worker_project`: the inference uv project
  inside `Catbot/src/catbot_policy/catbot_policy/inference` on the respective host.
  Orin's default uses its `external/Catbot` submodule. The worker uses its own
  `.venv`, ignoring the adapter's inherited Python/virtualenv settings.

Example:

```bash
ros2 launch catbot_launcher physical_orin.launch.py \
  workspace:=/home/robot/catbot-orin \
  run_directory:=/home/robot/runs/selected \
  policy_bundle:=/home/robot/policies/selected \
  policy_python:=3.14 \
  policy_ros_python:=/usr/bin/python3 \
  remote_user:=catbot remote_workspace:=/home/catbot/Catbot \
  body_to_camera_xyz:='[0.1, 0.0, 0.05]' ground_height_offset:=0.35
```

Other common overrides: `enable_camera:=false`, `enable_foxglove:=false`,
`camera_model`, `camera_name`, `serial_number`, `camera_config`, `foxglove_address`,
`foxglove_port`, `device`, `uv_executable`, `visualization_rate`, `command_timeout`,
`worker_timeout`, `startup_timeout`, `feedback_timeout`, `sensor_timeout` and
`sensor_max_skew`. Use a single Foxglove listener on the chosen port.

## One policy implementation on either machine

Orin's `src/catbot_policy/catbot_policy` is a symlink to the pinned
`external/Catbot/src/catbot_policy/catbot_policy` source. Its lightweight ament
wrapper installs that source without requiring foundation/CAN on Orin. No policy
logic is copied. The Pi uses the same package in Catbot. Keep Pi deployment and
Orin's Catbot gitlink aligned when changing policy interfaces.

```bash
git submodule update --init external/Catbot external/catbot_sim_rollout
# On Orin: install locked simulator/kinematics dependencies in separate Python 3.14
uv sync --locked --project external/catbot_sim_rollout
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select catbot_interfaces catbot_policy catbot_sim catbot_launcher
source install/setup.bash
```

`external/COLCON_IGNORE` prevents colcon traversing either external project.
Install ROS dependencies from the package manifests, including `nav_msgs`,
`sensor_msgs`, `tf2_ros`, `visualization_msgs`, and Python YAML. The ROS adapter requires no PyTorch installation. Its interpreter must retain
ROS Humble's Python ABI and see the sourced ROS/workspace `PYTHONPATH`. A
system-site-packages virtual environment based on `/usr/bin/python3` is suitable.
Inference runs in a separate uv-selected Python 3.14 environment, with the exact
PyTorch version required by the bundle. Prepare it on whichever host runs policy:

```bash
# Orin (scenarios 2/4)
uv sync --locked --python 3.14 --project external/Catbot/src/catbot_policy/catbot_policy/inference
# Pi (scenarios 1/3), from its Catbot checkout
uv sync --locked --python 3.14 --project src/catbot_policy/catbot_policy/inference
```

The adapter starts `uv run --locked --python <policy_python>` automatically. It
strips ROS `PYTHONPATH`, `PYTHONHOME` and inherited venv overrides from the worker's
environment so Python 3.10 ROS extensions cannot leak into Python 3.14. The
worker project and its `run_worker.py` must be deployed alongside the updated
shared policy code. No ROS package is imported by the worker.

Configure `policy_uv_executable` or `remote_policy_uv_executable` if uv is not on
the host's PATH. `policy_startup_timeout` (120 s) bounds worker startup; prepare
the environment first if package/Python downloads would take longer.
`policy_worker_timeout` (2 s) bounds each request and its acknowledgement.
Deploy the same Catbot commit pinned by Orin's `external/Catbot` submodule
to the Pi, then build/source its updated policy package for scenarios 1–3. The
isolated-worker source is consolidated on Catbot's `Dev` branch.
Scenario 4 never invokes SSH.
SSH uses existing key authentication and known-host configuration, with BatchMode.

Download artifacts with `uv run --locked --project external/catbot_sim_rollout
catbot-download-policy --entity YOUR_ENTITY`. Its printed run directory supplies
Orin's `run_directory`; its exported `policy_bundle` supplies whichever machine
runs inference. Both machines must load the same exported policy/model contract.

## Common foundation/simulation interface

Both backends consume `catbot_interfaces/ActuatorPositionCommand` on
`/body/control/position` and provide `BodyKinematicState` on `/body/state/position`.
Sparse named logical-degree targets are validated atomically. The simulation
requires a complete first target, then permits sparse updates. Its state contains
model-derived positions/velocities converted back to the same logical degrees.
No `/simulation/control/position` relay is used by these launches.

Both modes publish complete `PolicyFeedback` on `/policy/feedback`, a matching
model/mapping contract on `/policy/contract`, geometry on `/body/model`, and TF.
The policy's existing contract subscription is remapped to `/policy/contract`;
its contract gate is enabled for both measured and simulated observations.
`/policy/cmd_vel` controls the desired motion in every scenario.

Simulation labels `/foundation/backend` as `simulation`, and publishes an empty
`/body/state/error` actuator list because it has no ODrive diagnostics. Its axis IDs
in kinematic messages are model-order indices, not physical CAN addresses.
The physical backend retains foundation's actual state/error messages. Hardware
home/calibrate/full_calibrate/clear_errors topics are accepted as logged no-ops in
simulation. Reset by restarting the complete session, so policy and provider
history reset together. The interchangeable interface is the position-control and
observation interface; simulation does not emulate CAN maintenance or faults.

The simulator applies physical backlash to already smoothed/mapped targets.
Automatic episode resets and randomization are disabled. Physics is paced against
wall time, without catch-up bursts. Missing commands abort simulation after
`command_timeout` (0.25 s). Worker initialization/request stalls have separate
configurable timeouts. Either main scenario process exiting stops the rest;
SSH disconnects terminate the remote process group through its lease supervisor.

## Physical observations and calibration

The ZED policy profile explicitly enables IMU publishing and visual-inertial
positional tracking using the [wrapper's documented parameters](https://github.com/stereolabs/zed-ros2-wrapper/blob/master/zed_wrapper/config/common_stereo.yaml). An IMU alone is insufficient for drift-free translational
velocity. The adapter subscribes to:

- `imu_topic` (default `/{camera_name}/zed_node/imu/data`): gyro and acceleration.
- `odom_topic` (default `/{camera_name}/zed_node/odom`): fused pose and linear twist.
- `/body/state/position`: all twelve valid foundation encoder positions/velocities.
- `contact_topic` (default `/body/state/foot_contacts`): optional `UInt8MultiArray`
  containing four 0/1 contacts in FL, FR, RL, RR order.

Configure `body_from_camera_wxyz` for the odometry child frame, `body_from_imu_wxyz`
for the IMU message frame, and `body_to_camera_xyz` for the camera lever arm in body
coordinates. Defaults are identity rotation and zero translation; these are
placeholders, not a measured camera mount. Base position, tilt, heading-frame
velocity, body angular velocity and model-radian joint state are assembled using
these transforms. Linear velocity includes the camera lever-arm correction.

`ground_height_offset` aligns odometry's vertical origin to the ground plane;
0.35 m is a standing-height starting assumption. It is added to odometry Z after
the mount correction. Calibrate this for the actual startup origin/terrain.
`estimate_contacts:=true` permits flat-ground geometric contact estimates when a
fresh contact topic is absent: foot sphere bottoms within `contact_height` (0.02 m)
of that plane count as contact. Use `estimate_contacts:=false` to require measured
contacts. No sensor supplied here measures actual foot forces. Air time, stance
history and normalized joint travel are derived at observation time with the
training configuration. These estimates are not proof of physical gait accuracy.

`/body/state/linear_acceleration` publishes body-aligned acceleration at the IMU
location, with gravity removed when `imu_includes_gravity:=true`. It is diagnostic:
the current 90-value policy contract has no acceleration field. Rotational
lever-arm acceleration is not estimated. Tilt comes from the fused odometry
orientation; angular velocity comes from the rotated IMU gyro.

Missing, stale, non-finite, invalid or time-skewed required measurements stop
feedback publication; stale physical visualization also stops refreshing TF.
Messages must have valid stamps; synchronize the Pi/Orin clocks. Defaults allow
0.25 s age and 0.1 s inter-sensor skew. Policy stale-feedback handling pauses
commands but **does not idle physical motors**: foundation holds its last target.
The physical foundation startup sequence retains its existing calibration and
hardware behavior. The physical adapter never synthesizes encoder readiness.

## Foxglove and performance

Connect to `ws://<orin-ip>:8765`, add a 3D panel and enable `/body/model`.
For simulations select `sim/world`. For physical scenarios select `robot/world`
or the camera's map/odom root: `robot/world` is connected below the received odom
frame using the configured ground-height offset. Body transforms are derived
from measured encoders and base pose, never merely the commanded targets.

The saved model provides primitive geometry and colors; capsules are rendered
with cylinders and spheres. Infinite ground is shown as a finite patch. Textures
and unsupported mesh types are not reproduced. Frames/poses refresh up to
`visualization_rate` (30 Hz by default); markers repeat once per second for viewers
that connect later. Stop/restart the session when changing odometry frames.

`/policy/metrics` reports actor p50/p95/p99 timing, end-to-end adapter/worker
round-trip timing, publish rate, missed timer intervals, busy ticks, discarded
results, and separate adapter/worker CPU and peak memory on the policy host.
Only one inference request can be in flight. Results based on expired or
invalidated feedback are discarded; the worker advances action history only
after ROS acknowledges successful command publication. Worker exit or timeout
ends the policy node and session; workers are not silently restarted. `/simulation/metrics`
reports the selected backend, feedback freshness, contact mode and simulation
real-time factor (zero physics steps in hardware mode). This distinguishes Pi
inference limits from Orin simulation/kinematics limits. These measurements remain
uncollected until the target systems run.

## Verification boundary

Only static lint, syntax, XML/YAML, source-interface and launch-placement checks
were performed on the development machine. Focused ty checks passed with
unavailable ROS imports excluded; full ROS-dependent type resolution was not
possible in this environment. No ROS nodes, simulator, camera,
hardware, SSH workload or Foxglove session was started. Target package installation,
calibration, DDS QoS/clock alignment and all four live scenarios remain unverified.
