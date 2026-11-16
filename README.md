# CatBot ROS 2 workspace

This workspace contains the `catbot_launcher` package and StereoLabs' official
ROS 2 wrapper. The main launch file opens the camera, publishes its video,
depth, point-cloud, sensor, odometry, and TF data as ROS 2 topics, and starts a
Foxglove bridge for remote diagnostics.

## Build

```bash
cd /home/catbot/ROS/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Launch

```bash
ros2 launch catbot_launcher main.launch.py
```

Foxglove can connect to `ws://<robot-ip>:8765`. The bridge listens on all
network interfaces, so the robot's firewall must allow inbound TCP port 8765.

The default model is `zedx`. Select the correct model and, optionally, a camera
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

Configured Orin workspace: `/home/catbot/projects/ros2_ws`. Orin computation defaults
to CUDA (`orin_device:=cuda:0`), with `orin_device:=cpu` available for the locked CPU
environments. CUDA needs a compatible JetPack/Python/PyTorch environment first.
Download a fixed run with `uv run src/catbot_policy/pull_policy.py ENTITY/PROJECT/RUN_ID model_N.pt`;
launches use that module's selected local artifacts. Camera settings are unchanged.

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
