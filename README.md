# CatBot ROS 2 workspace

This workspace contains the `catbot_launcher` package and StereoLabs' official
ROS 2 wrapper. The main launch file opens the camera, publishes its video,
depth, point-cloud, sensor, odometry, and TF data as ROS 2 topics, and starts a
Foxglove bridge for remote diagnostics.

## Build

```bash
cd /home/catbot/projects/ros2_ws
git submodule update --init external/Catbot external/catbot_sim_rollout
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

The shared ROS packages under `src/` link to the single pinned source copy in
`external/Catbot`. Normal `colcon build` discovers them once; use it instead of
passing `external/Catbot` paths through `--base-paths`. See the
[scenario build instructions](docs/SIMULATION.md#one-policy-implementation-on-either-machine)
for the one-time cleanup when upgrading an existing workspace.

## Launch

```bash
ros2 launch catbot_launcher main.launch.py
```

Foxglove can connect to `ws://<robot-ip>:8765`. The bridge listens on all
network interfaces, so the robot's firewall must allow inbound TCP port 8765.

The default model is `zed2i`. Select the correct model and, optionally, a camera
serial number at launch time:

```bash
ros2 launch catbot_launcher main.launch.py \
  camera_model:=zedx camera_name:=zed serial_number:=0
```

Common camera model values include `zedx`, `zedxm`, `zed2`, `zed2i`, `zed`, and
`zedm`. Press Ctrl+C to stop the camera node.

Runtime parameters can be overridden in
`src/catbot_launcher/config/zed_params.yaml`. Its examples are commented out,
so the StereoLabs defaults are used until an entry is uncommented. A different
parameter file can also be selected without rebuilding:

```bash
ros2 launch catbot_launcher main.launch.py config_file:=/path/to/zed_params.yaml
```

Inspect the published data with:

```bash
ros2 topic list | grep '^/zed/'
ros2 topic hz /zed/zed_node/rgb/image_rect_color
ros2 topic echo /zed/zed_node/pose --once
```

## Robot operation and load testing

Four scenario launch commands share one configuration:

```bash
ros2 launch catbot_launcher physical_pi.launch.py      # Pi foundation + policy
ros2 launch catbot_launcher physical_orin.launch.py    # Pi foundation, Orin policy
ros2 launch catbot_launcher simulation_pi.launch.py    # Orin sim, Pi policy only
ros2 launch catbot_launcher simulation_orin.launch.py  # Orin sim + policy; Pi idle
```

All start Foxglove and publish the live model on `/body/model`. Physical scenarios
start the ZED camera and map odometry/IMU plus motor feedback into policy inputs.
Simulation uses the same position-control and joint-state topics as foundation.
See [the scenario guide](docs/SIMULATION.md) for shared defaults, deployment,
calibration, sensor estimates and the static-only verification boundary.

Configured Orin workspace: `/home/catbot/projects/ros2_ws`. Orin policy defaults to
CUDA with Python 3.10 and a separately prepared Jetson Torch environment.
`orin_device:=cpu` selects Python 3.14 and the locked CPU environment.
Simulation/kinematics retain their modern Python stack and default to CPU.
See the scenario guide for the environment setup script and CUDA prerequisites.
Download a fixed run with `uv run src/catbot_policy/pull_policy.py ENTITY/PROJECT/RUN_ID model_N.pt`;
launches use that module's selected local artifacts. Camera settings are unchanged.

## Foxglove health diagnostics

Rebuild and source the workspace after updating. The main launch and all four
scenarios start `catbot_diagnostics` automatically, publishing
`diagnostic_msgs/msg/DiagnosticArray` on `/diagnostics` once per second.
In Foxglove, add **Diagnostics – Summary (ROS)**, select `/diagnostics`, and use a
stale timeout of 5 seconds. Click a row for **Diagnostics – Detail (ROS)** to see
the topic, message age, thresholds, and decoded motor faults. This uses
[Foxglove's standard diagnostics panels](https://docs.foxglove.dev/docs/visualization/panels/diagnostics).
The ZED driver's own diagnostics can share this topic.

The `catbot/` rows cover:

| Row | Checks |
| --- | --- |
| `zed/image` | No first image, stopped stream, nonadvancing timestamps, persistent green or malformed/unsupported raw images |
| `zed/imu` | Missing/stopped IMU, nonadvancing or zero timestamps, unavailable angular velocity |
| `zed/odometry` | Missing/stopped odometry, nonadvancing or zero timestamps, unavailable pose/velocity (physical scenarios) |
| `body/joints` | Missing/stopped joint feedback, nonadvancing or zero timestamps, invalid/empty actuator feedback |
| `policy/feedback` | Missing/stopped policy observations and nonadvancing or zero timestamps |
| `policy/output` | Missing/stopped position commands from the policy |
| `body/motors` | Missing CAN heartbeats, decoded ODrive error flags, motors idle/not ready (physical scenarios) |

Rows start at **WARN** while waiting for initial data, become **ERROR** after the
startup grace, and return to **OK** when fresh valid data resumes. Suspect images
are WARN during the persistence check. Idle/disarmed motors are WARN. Motor error
reports are retained because foundation publishes them only on changes; a lost
joint stream also makes the motor row unconfirmed. Simulation omits camera and
motor rows unless the camera is explicitly enabled. Camera-only launches omit
robot and odometry checks because positional tracking may be disabled.

Defaults are a 60-second startup grace for the camera-only launch (the larger
bridge/policy startup timeout for scenarios), 2 seconds for robot/IMU/odometry
streams, and 10 seconds for images. The green-image check shares the watchdog's
98%/5-second defaults and limitations described below. Scenario overrides include
`diagnostics_stream_timeout:=2.0`, `diagnostics_frame_timeout:=10.0`, and
`enable_diagnostics:=false`. The camera name and custom IMU/odometry topics follow
the scenario's settings.

For an externally launched camera/watchdog with only the Foxglove bridge, use:

```bash
ros2 launch catbot_launcher main.launch.py enable_camera:=false monitor_camera:=true
```

Physical scenarios keep monitoring the expected camera even with
`enable_camera:=false`, supporting the separate watchdog below. The watchdog's
child camera launch disables its own monitor to avoid duplicate health reports.
The monitor only reports health; it does not restart devices or change motor
control. It can report camera loss while the rest of the launch stays alive. If
the whole launch or ROS/network connection stops, Foxglove's stale timeout is
needed to mark the last reports stale. Host resource exhaustion and faults not
represented by these inputs require system logs or other diagnostics publishers.

## ZED green-frame recovery

After rebuilding `catbot_launcher`, run the camera through its watchdog on the
Orin (source ROS and `install/setup.bash` first):

```bash
ros2 run catbot_launcher zed_watchdog --camera-model zed2i --serial-number 0
```

This command **owns the camera launch**. Stop any existing camera launch first.
For Foxglove or a physical scenario, start that launch with `enable_camera:=false`
and let this watchdog supply the camera. Pass the same camera name and parameter
file that the scenario would use, for example:

```bash
ros2 run catbot_launcher zed_watchdog --camera-model zed2i \
  --config-file "$PWD/src/catbot_launcher/config/zed_policy.yaml"
```

The watchdog samples raw RGB/BGR/RGBA/BGRA frames. By default, at least 98% of
sampled pixels must be strongly green for 5 seconds before restarting. Missing
frames, nonadvancing timestamps, malformed/unsupported images, or a launch exit
also trigger recovery. First-image grace is 60 seconds; stream timeout is 10
seconds. Inspect `--help` to tune these values or select a different raw `--topic`.
A real scene filling the camera with green can trigger this heuristic; validate
thresholds using recordings from this camera. A frozen scene whose timestamps
continue advancing is not detected. Corruption only in the viewer/compressed
transport may also escape this raw-image check.

Recovery stops the dedicated launch process group with SIGINT, escalating to
SIGTERM/SIGKILL if necessary, then waits 10 seconds before reopening the camera.
It never kills unrelated ROS nodes. After three restarts over the supervisor's
lifetime it stops the camera and exits with failure, keeping the logs visible
instead of entering an endless restart loop. Ctrl+C stops its camera too.

Green output does not establish an out-of-memory cause. Stereolabs identifies
USB stream corruption as one cause and describes the SDK's
[`enable_image_validity_check`](https://community.stereolabs.com/t/getting-a-green-image-sometimes/9121)
as a way to reject some corrupt frames. Check the installed SDK/wrapper support
before configuring it; this is an SDK initialization option, not necessarily an
exposed ROS parameter. Stereolabs also recommends closing/reopening the node
for a stuck stream and checking compatible SDK/wrapper updates in its
[recovery guidance](https://community.stereolabs.com/t/zed-x-camera-publishes-frozen-stale-image-frames-indefinitely-requires-node-restart-to-recover/11118).

When this recurs, retain the camera console output and inspect `journalctl -k -b`
for OOM/USB faults and `tegrastats` for RAM/GPU pressure. Compare the existing
`zed_usb_test.yaml` (720p/15 FPS) against the normal profile to test bandwidth/load
sensitivity. For USB cameras check cable, power and connection stability too.
Restarting releases process resources but cannot repair a driver fault or a
persistent memory shortage. Hardware recovery and detection thresholds still
need validation on the Orin. This watchdog does not disarm motors or invalidate
policy observations; keep the robot stationary during recovery testing, and
require a separate stale-sensor stop in motion control before unattended use.

## Orin Supervisor

The Orin-hosted physical and simulation scenarios now launch the managed Orin
Supervisor next to the feedback bridge. Pi startup, body scripts and action
forwarding remain on the independently launched Pi Supervisor. See
[deployment and API verification](docs/SUPERVISOR.md) and the Orin-owned
[`orin_supervisor.yaml`](src/catbot_launcher/config/orin_supervisor.yaml).
Recording and offline/shadow simulation remain deferred.
