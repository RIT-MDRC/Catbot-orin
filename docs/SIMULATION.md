# Orin simulation with Pi inference

The Orin runs a single headless MuJoCo world. The Pi at `192.168.0.3`
runs the existing policy runtime and foundation's simulation entry point:

```text
Orin worker -> catbot_sim -> /policy/feedback -> Pi catbot_policy
Orin worker <- catbot_sim <- /simulation/control/position <- Pi foundation
                                                          ^
                                               /body/control/position
```

DDS configuration is inherited, not changed. SSH is only used to supervise the
remote processes. The simulation backend never initializes GPIO/CAN and does not
claim hardware readiness. Hardware mode remains the default for Catbot's normal
launch. This measures policy inference and simulated command validation; it does
not measure the physical CAN/sensor workload.

## Prepare on the target machines

Deploy the changes in **all three repositories** before building:

- Orin: this workspace and the updated `catbot_mujoco_sim` checkout.
- Pi: updated Catbot `feat/update-main` checkout.

The Orin workspace vendors `catbot_interfaces` from Catbot commit
`bab64ffb21f62149cc60ba26ed8143af47d32ab4`. Keep the message files identical on
both machines when updating Catbot. No training dependencies are needed on the Pi.

On the Pi, build the changed packages after sourcing its existing ROS/DDS setup:

```bash
cd /absolute/path/to/Catbot
colcon build --symlink-install --packages-select catbot_interfaces foundation catbot_policy
```

On the Orin, prepare the simulation environment separately from ROS's Python:

```bash
cd /absolute/path/to/catbot_mujoco_sim
uv sync --locked
cd /absolute/path/to/catbot-orin
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select catbot_interfaces catbot_sim catbot_launcher
source install/setup.bash
```

The simulation repository currently requires Python 3.14 and its locked mjlab,
MuJoCo, Warp, and PyTorch dependencies. Their installation on the target Orin must
succeed before launch; the bridge uses `uv run --locked` and does not import them
into ROS's Python. CPU simulation is the default. `device:=cuda:0` can be selected
only with a compatible target CUDA environment.

Keep the selected training run's **config.json and model.xml** together in an
Orin directory. Use the policy bundle exported from that same run on the Pi
(containing policy.pt and policy.json). A bundle alone does not contain the
simulation model/configuration. Copy the same actuator mapping JSON to both
machines. All path arguments below must be absolute and refer to the indicated
machine; `~` is not expanded in remote shell arguments.

SSH key authentication and the Pi's host key must already be configured. The
launcher uses BatchMode and will fail rather than prompt. If the Pi's existing DDS
environment is not supplied by its login shell, set `remote_environment` to an
existing shell setup file that sources ROS and exports that DDS configuration.

## Launch from the Orin

```bash
ros2 launch catbot_launcher simulation.launch.py \
  remote_host:=192.168.0.3 \
  remote_user:=catbot \
  remote_workspace:=/absolute/pi/path/to/Catbot \
  remote_environment:=/absolute/pi/path/to/ros-dds-setup.bash \
  policy_bundle:=/absolute/pi/path/to/policy_bundle \
  remote_mapping_file:=/absolute/pi/path/to/default_mapping.json \
  simulation_repository:=/absolute/orin/path/to/catbot_mujoco_sim \
  run_directory:=/absolute/orin/path/to/training_run \
  mapping_file:=/absolute/orin/path/to/default_mapping.json
```

The worker validates its run configuration, initializes one world, and publishes
its contract. The Pi checks model hash, policy contract (including control period),
deployment settings, and mapping before accepting observations. Physics remains
at the initial pose until the first complete 12-actuator command returns through
foundation. Later commands may be sparse; validation is atomic.

The worker accepts model-radian position targets after the inverse logical-degree
mapping. Policy smoothing/clipping happens once, on the Pi; simulator backlash is
then applied. Physics and contact/motion histories use the training configuration.
Domain randomization and automatic episode resets are disabled for this compute
experiment. A fallen robot stays fallen. Restart the **entire launch** to reset
both physics and policy history. There is no independent reset service or native MuJoCo viewer; use Foxglove below.

The default velocity command is zero. For example, on either machine:

```bash
ros2 topic pub --once /policy/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'
```

Physics runs at its configured timestep, wall-time paced, holding the latest
accepted target. Slow simulation does not trigger catch-up bursts. The bridge
aborts after `command_timeout` (default 0.25 seconds) without a fresh command,
worker failure/stall, or `startup_timeout` (default 120 seconds) without a first
command. Either launch child exiting stops the other. Ctrl+C closes the SSH lease;
the remote supervisor interrupts the ROS process group, then escalates if needed.
Loss of the SSH lease also stops that group (10-second lease plus cleanup grace).

Use one simulation session per DDS domain and stop any existing hardware/policy
launch before starting this one: the existing absolute ROS topic names are shared.
This launch includes `main.launch.py`, starting the camera and exactly one Foxglove
bridge alongside simulation. The regular camera launch still works by itself.
Camera arguments (`camera_model`, `camera_name`, `serial_number`, `config_file`)
are also accepted by the simulation launch. Use `enable_camera:=false` to run
simulation and Foxglove without the camera. If a Foxglove bridge is already running,
use `enable_foxglove:=false` to avoid a second listener on port 8765.

## View the simulated robot in Foxglove

Connect Foxglove to `ws://<orin-ip>:8765`, add a **3D** panel, select `sim/world`
as the display/fixed frame, and enable `/simulation/model`. Enable the TF frames
in that panel to inspect individual links. The geometry arrives as
`visualization_msgs/msg/MarkerArray`, with body transforms on `/tf` at up to 30 Hz.
Geometry is retained and repeated once per second so reconnecting viewers can
load it. No URDF, mesh download, or browser-side MuJoCo renderer is needed.

The worker reads geometry and colors from the selected run's compiled model,
including joint offsets and the actual simulated base pose. Boxes, cylinders,
spheres, ellipsoids and capsules are supported; capsules are drawn as a cylinder
and two spheres. Infinite planes appear as finite thin boxes. Textures are not
reproduced; unsupported geometry types are reported in the worker log and omitted.
Rendering snapshots use forward kinematics from simulated joint positions, not
requested motor targets. The Pi does no visualization work.

Simulation uses its own `sim/world` TF root with uniquely prefixed body frames.
The physical ZED camera retains its existing TF root: use another 3D panel with
the camera's frame for its point cloud, or an Image panel for video/depth. There
is no fabricated transform aligning the real room with the simulated world.
A calibrated transform would be needed to overlay those two scenes accurately.

Update both this workspace and the simulation repository for visualization;
the Pi code does not change. Rebuild `catbot_sim` and `catbot_launcher` on the
Orin after installing their declared `tf2_ros` and `visualization_msgs` dependencies.

## Observe compute capacity on the Pi

```bash
ros2 topic echo /policy/metrics
ros2 topic echo /simulation/metrics
ros2 topic hz /body/control/position
ros2 topic hz /policy/feedback
```

Every five seconds `/policy/metrics` publishes JSON with actor inference and
policy-step callback p50/p95/p99 milliseconds, successful publish rate, missed timer
intervals, callback overruns, CPU use (100% = one CPU core), and peak RSS in KiB on
Linux. Inference timing covers the CPU actor call; callback timing also includes
observation assembly, validation, mapping, publication and policy history updates.
Feedback age is measured from **local Pi receipt**, not cross-host transport time.

`/simulation/metrics` reports actual simulated control steps per wall second,
real-time factor, and local age of commands received on the Orin. Compare policy
p99 to `control_interval_ms`, check that successful publish rate is near its
reciprocal, and investigate missed deadlines or overruns during a sustained run.
A simulation real-time factor below 1 identifies an Orin-side bottleneck; it does
not by itself establish that Pi inference is too slow. No synchronized clocks or
one-way network-latency estimates are assumed.

## Verification status

Implementation was checked with static analysis only. No ROS nodes, simulator,
SSH sessions, builds, hardware tests, or performance benchmarks were run on the
development Mac. Target dependency availability, DDS traffic, shutdown behavior,
and sustained Pi timing still require observation on the Orin/Pi.
