# Catbot operation and load-testing scenarios

All four commands share `catbot_launcher/scenarios.py` and
`config/scenarios.yaml`. Local nodes inherit the Orin environment. Remote Pi nodes
source the configured shared ROS/DDS setup file before the Pi workspace; they do
not depend on interactive `.bashrc` exports. These are mutually exclusive sessions:
stop the previous launch and
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

`simulation_orin` starts the policy automatically and sends its actuator commands
directly to the simulator after validating `/policy/contract`. It needs no Pi
connection or supervisor job request. `physical_orin` uses the Orin supervisor
and retains the Pi lease, preflight and explicit policy-start requirements.

## Defaults and overrides

The checked-in `src/catbot_launcher/config/scenarios.yaml` holds shared values.
Edit it and rebuild, or point `CATBOT_LAUNCH_DEFAULTS` at a customized copy before
launch. Every key is also a launch argument; CLI values override the defaults.
`camera_config` and `enable_camera` are additional arguments, defaulting to the
installed `zed_policy.yaml` and `auto` (on for physical, off for simulation).

Confirmed target setup: Orin workspace `/home/catbot/projects/ros2_ws`, Pi
`catbot@192.168.0.3` (alternatively `catbot-pi.local`), passwordless SSH, Ubuntu
22.04 on both machines, uv available, and NTP active. The Pi's shared environment
file is `/home/catbot/.config/catbot/ros_environment.bash`; it sources
`/opt/ros/humble/setup.bash` and exports the DDS discovery settings.
`/usr/bin/python3` remains the expected ROS interpreter. The confirmed Pi workspace is
`/home/catbot/main`; remote model, mapping and inference paths follow it
automatically. The Orin reports `nvidia-jetpack 6.2.3+b81` and
`nvidia-l4t-core 36.5.2-20260716114719`. These are reported target versions,
not measurements from the development machine.

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
- `remote_environment`: `/home/catbot/.config/catbot/ros_environment.bash`.
  This file must exist on the Pi and source ROS plus export its DDS settings
  (`ROS_DISCOVERY_SERVER`, `ROS_SUPER_CLIENT`, `RMW_IMPLEMENTATION`). The launcher
  sources it, then `{remote_workspace}/install/setup.bash`, then starts the nodes
  in the same shell. A source failure prevents startup. `.bashrc` can source the
  same shared file; SSH does not rely on `.bashrc` being read. An explicit
  `remote_environment:=/other/setup.bash` still overrides the default.
- `policy_python` / `remote_policy_python`: `auto` selects 3.10 for CUDA and
  3.14 for CPU; a version or interpreter path overrides this selection.
- `policy_ros_python` / `remote_policy_ros_python`: `/usr/bin/python3`, used only
  for ROS adapters and required to match the installed ROS ABI.

Example:

```bash
ros2 launch catbot_launcher physical_orin.launch.py \
  workspace:=/home/robot/catbot-orin \
  policy_python:=auto \
  policy_ros_python:=/usr/bin/python3 \
  remote_user:=catbot remote_workspace:=/home/catbot/main \
  body_to_camera_xyz:='[0.1, 0.0, 0.05]' ground_height_offset:=0.35
```

Health is published on `/diagnostics` for Foxglove's Diagnostics panels; see the
[health checks and thresholds](../README.md#foxglove-health-diagnostics).
Physical scenarios monitor the camera even with `enable_camera:=false` for an
external watchdog. Simulation omits hardware checks by default.

Other common overrides: `enable_camera:=false`, `enable_foxglove:=false`, `enable_diagnostics:=false`,
`camera_model`, `camera_name`, `serial_number`, `camera_config`, `foxglove_address`,
`foxglove_port`, `device`, `uv_executable`, `visualization_rate`, `command_timeout`,
`worker_timeout`, `startup_timeout`, `feedback_timeout`, `sensor_timeout` and
`sensor_max_skew`. Use a single Foxglove listener on the chosen port.


### Select policies by run ID

The default W&B project is `hiromon-rochester-institute-of-technology/catbot`.
From the Orin workspace, download a run's latest checkpoint, or a specific number:

```bash
uv run src/catbot_policy/pull_policy.py ow4nsasy
uv run src/catbot_policy/pull_policy.py ow4nsasy 19999
# Named alternatives:
uv run src/catbot_policy/pull_policy.py --run-id ow4nsasy --checkpoint 19999
```

Omitting the checkpoint downloads the highest numbered `model_N.pt` **in that
specific W&B run**. Full `ENTITY/PROJECT/RUN_ID` and `model_N.pt` inputs still
work; `--entity` / `--project` or `WANDB_ENTITY` / `WANDB_PROJECT` override the
downloader defaults. Download/export selects the result for subsequent default
launches. Failed preparation preserves the previous selected link.

Launch without arguments to keep the selected local download. To select from the
local cache without downloading or changing the saved selection:

```bash
ros2 launch catbot_launcher simulation_orin.launch.py run_id:=ow4nsasy
ros2 launch catbot_launcher simulation_orin.launch.py run_id:=ow4nsasy checkpoint:=19999
```

`run_id` alone chooses the highest **downloaded** checkpoint of that run, not a
newer checkpoint from W&B. `checkpoint` is optional, uses ROS syntax `:=`, and
requires `run_id`. Missing/invalid selections list downloaded run IDs, checkpoint
numbers and paths. Multiple exports of the same run/checkpoint are treated as
ambiguous; select explicit paths in that case. Restart the launch to switch.

These arguments apply to all four Orin scenarios and Catbot's standalone launch.
The helper selects both simulator model/config and policy bundle together. Pi
policy scenarios pass the exact resolved run/checkpoint to the Pi; that same
checkpoint must already be downloaded on both hosts. The Pi resolves its own
local paths and lists its own cache on selection failure. Custom local cache
roots use `policy_artifacts_directory` / `remote_policy_artifacts_directory`;
`policy_entity` and `policy_project` override the launch's default project.
Existing explicit `run_directory` / `policy_bundle` paths remain supported when
`run_id` and `checkpoint` are omitted. CUDA/CPU environment selection is unchanged.

The downloader also discovers `external/catbot_sim_rollout`, pins its
`catbot-mujoco` dependency to the downloaded manifest's `git_revision`, updates
its lockfile, and syncs only the simulator's `.venv`. It validates the saved
environment config before changing `selected`; GPU policy `.venv-cuda` is never
synced. Run downloads with ROS stopped, since simulator dependencies may change.
Use `--rollout-repository PATH` for a custom simulator project, or
`--skip-rollout-setup` for policy-only preparation. If simulator preparation fails,
dependency files are restored and the previous selection stays in place; its
`.venv` may already have changed, so rerun preparation before launching. A missing
or invalid training revision fails explicitly; dirty training revisions produce
a warning because uncommitted training code cannot be recovered from the SHA.

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
based on `/usr/bin/python3` is also suitable. The inference worker uses a separate CPU or CUDA environment as described below.
Deploy the same Catbot `Dev` revision pinned by Orin's submodule to the Pi and
build/source its updated packages for scenarios 1–3. Scenario 4 never invokes SSH.

## Orin CUDA and Pi CPU

`orin_device` defaults to `cuda:0` for Orin policy inference. Pi
`remote_policy_device` defaults to `cpu`. Simulation/kinematics `device` now defaults
to `cpu`: it retains the Python 3.14 MuJoCo/training dependencies, independent of
policy inference. Changing the policy interpreter does not port that training
stack to older Python or JetPack CUDA.

| Policy device | Python with `policy_python:=auto` | Environment | Dependency handling |
| --- | --- | --- | --- |
| CUDA | 3.10 | `<inference>/.venv-cuda` | Prepared Jetson Torch; no automatic sync |
| CPU | 3.14 | `<inference>/.venv` | Locked PyTorch 2.14 CPU environment |

The same selection applies on the Pi through `remote_policy_*` options and in
Catbot's standalone launch. ROS adapters keep `/usr/bin/python3`. A venv does not
change system Python or the ROS ABI. No environment activation is necessary for
launching: uv selects the policy environment explicitly.

On the **Orin**, from `/home/catbot/projects/ros2_ws`, prepare CPU inference:

```bash
python3 external/Catbot/src/catbot_policy/catbot_policy/inference/setup_environment.py --device cpu
```

Prepare CUDA inference using a trusted wheel built for **JetPack 6.2 / CUDA 12.6,
Python 3.10, Jetson aarch64**, supplying the actual wheel path or URL:

```bash
python3 external/Catbot/src/catbot_policy/catbot_policy/inference/setup_environment.py \
  --device cuda --torch-wheel /absolute/path/to/jetson-torch-cp310-linux_aarch64.whl
```

Alternatively, if CUDA Torch is already installed for the system Python, reuse it
in a separate venv (a new environment is required for system-site-packages):

```bash
python3 external/Catbot/src/catbot_policy/catbot_policy/inference/setup_environment.py \
  --device cuda --python /usr/bin/python3 --system-site-packages
```

These are alternative CUDA provisioning methods. The script verifies Python/Torch
versions and CUDA allocation **on the target**; it starts no policy or ROS nodes.
Use `--python` and `--environment` to customize setup, and matching `policy_python`
and `policy_worker_environment` launch overrides for custom values. On the Pi,
the same script lives at `src/catbot_policy/catbot_policy/inference/setup_environment.py`.
Use `--device cpu` there by default.

[NVIDIA's compatibility matrix](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform-release-notes/pytorch-jetson-rel.html)
lists PyTorch 2.7/2.8 prereleases for JetPack 6.2. It does not list a standalone
wheel for those releases, so this repository does not invent a download URL or
install a desktop/SBSA ARM wheel as a substitute. The reported target is JetPack
6.2.3 / L4T 36.5.2; choosing a compatible vendor build and validating it on the
actual Orin remain necessary. System CUDA libraries and any wheel-specific
prerequisites must already be installed.

Launch simulation on CPU, policy on Orin CUDA, and Foxglove:

```bash
ros2 launch catbot_launcher simulation_orin.launch.py
# Explicit equivalent:
ros2 launch catbot_launcher simulation_orin.launch.py device:=cpu policy_device:=cuda:0
# Modern CPU policy instead, with its independent locked venv:
ros2 launch catbot_launcher simulation_orin.launch.py orin_device:=cpu
```

`policy_python:=auto`, `policy_worker_environment:=auto` and
`policy_worker_sync:=auto` derive defaults from the policy device. CUDA uses a
separate `inference/cuda` uv project that permits Python >=3.10, with the prepared
`.venv-cuda` beside the CPU project's `.venv`. CPU dependency resolution remains
unchanged. CUDA syncing is rejected so it cannot remove vendor packages; missing
CUDA environments/devices fail explicitly without falling back to CPU. The
simulator's `worker_environment` / `worker_sync` are independent: explicitly
selecting `device:=cuda:0` still requires a separately prepared **modern** CUDA
simulation environment.

`policy_torch_compatibility:=auto` uses `exact` for CPU and `load` for CUDA.
`exact` requires the exported Torch version to match. `load` permits a version
mismatch with a warning, retains bundle checksums and observation/action contract
validation, and requires TorchScript loading plus a finite `(1, 12)` startup
output. Loading newer TorchScript on older Torch is not guaranteed; an unsupported
operator still stops startup. A successful load is not proof of numerical parity
or learned gait correctness. Validate representative observations against the
modern CPU reference before physical operation. Set
`policy_torch_compatibility:=exact` to restore strict matching, or
`remote_policy_torch_compatibility` for remote inference. Policy downloads/exports
remain on their own modern/tooling environment; they do not install into CUDA.

`/policy/metrics` reports the selected device; startup logs report worker Python.
CUDA timing synchronizes kernel execution. Worker launch removes inherited ROS
Python paths and virtualenv overrides. CPU/CUDA runtime and policy parity have
not been tested on the development Mac; verification here is static only.

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

`/policy/metrics` uses `catbot_interfaces/msg/PolicyMetrics`, with directly plottable
numeric fields such as `inference_p95_ms`, `round_trip_p99_ms`, and `published_hz`.
Rebuild/source interfaces and policy on both hosts after upgrading from the
former JSON String topic. Unavailable measurements are NaN; `sample_count`
identifies empty timing windows. It reports actor p50/p95/p99 timing, end-to-end adapter/worker
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
