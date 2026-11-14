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

Confirmed target setup: Orin workspace `/home/catbot/projects/ros2_ws`, Pi
`catbot@192.168.0.3` (alternatively `catbot-pi.local`), passwordless SSH, Ubuntu
22.04 on both machines, uv available, DDS already configured without an extra
setup script, and NTP active. `/opt/ros/humble/setup.bash` and `/usr/bin/python3`
remain the expected ROS defaults. The Pi workspace path remains unconfirmed;
`remote_workspace` currently retains `/home/catbot/Catbot`. Set it once and the
remote model/mapping/inference paths follow it automatically.

The launch uses only locally selected artifacts. It never picks a W&B project,
queries W&B, or automatically changes the run at startup. Download a fixed run
once on each host using the existing policy package script:

```bash
# Orin, from /home/catbot/projects/ros2_ws:
uv run src/catbot_policy/pull_policy.py ENTITY/PROJECT/RUN_ID model_N.pt
# Pi, from its Catbot workspace:
uv run src/catbot_policy/pull_policy.py ENTITY/PROJECT/RUN_ID model_N.pt
```

The Orin script and its lockfile are symlinks to the shared Catbot implementation.
Downloads stay in that policy package's ignored `artifacts/` directory. Its
`selected` symlink selects one validated run, containing `config.json`, `model.xml`,
`manifest.json` and `bundle/policy.pt` plus `bundle/policy.json`. This supplies
both `run_directory` and the host's policy bundle automatically. A failed export
keeps the previous selection. The compatibility `latest` alias points to
`selected/bundle`; it does not mean W&B is queried during launch.

An explicit run is required by default; omit the checkpoint only if selecting
the highest available checkpoint in that **fixed run at download time** is
intended. Use the same run/checkpoint on both hosts to avoid contract mismatch.
`--login` remains available. `--latest --entity YOUR_ENTITY` explicitly opts into
selecting a newer run when downloading. Existing files from the separate rollout
repository's downloader remain usable via explicit `run_directory` and
`policy_bundle` overrides; they are not silently scanned or substituted.

Defaults derive from the workspace paths:

- `run_directory`: `{workspace}/external/Catbot/src/catbot_policy/artifacts/selected`.
- `policy_bundle`: the same directory's `bundle` child for Orin inference.
- `remote_policy_bundle`: `{remote_workspace}/src/catbot_policy/artifacts/selected/bundle`.
- `mapping_file`: `{workspace}/default_mapping.json`; the remote mapping is inside
  `{remote_workspace}/src/catbot_policy/catbot_policy/`. It remains provisional.
- `rollout_repository` and local/remote inference worker projects are derived from
  their workspace paths. Templates support references to other defaults, with
  unknown keys and cycles rejected.
- `remote_environment`: `/opt/ros/humble/setup.bash`; DDS is inherited unchanged.
- `policy_python` / `remote_policy_python`: `3.14` selected by uv.
- `policy_ros_python` / `remote_policy_ros_python`: `/usr/bin/python3`, used only
  for ROS adapters and required to match the installed ROS ABI.

Example:

```bash
ros2 launch catbot_launcher physical_orin.launch.py \
  workspace:=/home/robot/catbot-orin \
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
Install the ROS package dependencies, including `nav_msgs`, `sensor_msgs`,
`tf2_ros`, `visualization_msgs`, and Python YAML. The ROS adapter requires no
PyTorch installation. Its interpreter must match ROS Humble's Python ABI and see
the sourced ROS/workspace `PYTHONPATH`. A system-site-packages virtual environment
based on `/usr/bin/python3` is also suitable. The inference worker remains separate
on Python 3.14, with the exact PyTorch version required by the downloaded bundle.
Deploy the same Catbot `Dev` revision pinned by Orin's submodule to the Pi and
build/source its updated packages for scenarios 1–3. Scenario 4 never invokes SSH.

## Orin CUDA and Pi CPU

`orin_device` defaults to `cuda:0` and supplies both simulation/kinematics `device`
and Orin `policy_device`. Pi `remote_policy_device` defaults to `cpu`. To switch
all Orin computation to CPU, use a single override with any scenario:

```bash
ros2 launch catbot_launcher simulation_orin.launch.py orin_device:=cpu
ros2 launch catbot_launcher simulation_pi.launch.py orin_device:=cpu
```

For mixed placement, override `device` and `policy_device` independently. The
selected policy device is included in `/policy/metrics`. CUDA actor timing is
synchronized so kernel launch time alone is not mistaken for execution time.

**CUDA package compatibility still requires the Orin's JetPack version.**
The shipped inference lock explicitly installs CPU PyTorch; it cannot enable
Jetson CUDA. NVIDIA builds must match JetPack, Python 3.14, and the bundle's exact
PyTorch version. A prerelease or different version remains rejected. See
[NVIDIA's Jetson installation guidance](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html).
A configured CUDA default is not evidence that a compatible binary is installed.

With CUDA selected, `worker_environment: auto` and
`policy_worker_environment: auto` select `.venv-cuda` beside each worker's
`pyproject.toml`. These environments must already contain the compatible GPU
packages and required dependencies. Their `*_sync: auto` choices disable syncing
and use `uv run --no-sync` with `UV_PROJECT_ENVIRONMENT` so the CPU lock does not
replace a prepared NVIDIA build. Both the simulator and inference environments
need appropriate dependencies. They are distinct projects and virtual environments.
Override environment paths and sync flags for another prepared environment.
Missing CUDA interpreters or unavailable CUDA cause an explicit startup failure;
there is no automatic CPU fallback. Until CUDA compatibility is established,
`orin_device:=cpu` selects the shipped locked CPU environments instead.

For CPU inference, prepare the environment on whichever host runs the policy:

```bash
# Orin, from its workspace:
uv sync --locked --python 3.14 --project external/Catbot/src/catbot_policy/catbot_policy/inference
# Pi, from its Catbot workspace:
uv sync --locked --python 3.14 --project src/catbot_policy/catbot_policy/inference
```

CPU simulation uses the rollout project's normal `uv sync --locked` environment.
The worker launcher clears inherited ROS Python paths and virtualenv overrides
before selecting its own environment. `policy_uv_executable` and
`remote_policy_uv_executable` can override uv's executable path. Worker/startup
limits remain adjustable through the existing timeout parameters.

To report versions without starting ROS nodes, policy, cameras or motors, run
these commands **on the target machines**, not on the development Mac:

```bash
/usr/bin/python3 --version
ls /opt/ros
uv python list --only-installed
# Orin only; nvidia-jetpack may be absent even when L4T is installed:
dpkg-query -W nvidia-jetpack nvidia-l4t-core
```

Camera model, IMU/odometry topics, tracking configuration, mounting transforms,
contact estimation and physical calibration settings were deliberately left
unchanged while simulation is verified first.

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
