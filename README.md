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

## Simulation with policy inference on the Pi

Use `catbot_launcher simulation.launch.py` for a headless simulation on the Orin
and policy/foundation on `192.168.0.3`, together with the existing camera and
Foxglove bridge. The simulated model is available on `/simulation/model` in the
`sim/world` frame. Use `enable_camera:=false` when no camera is attached. See [the simulation setup guide](docs/SIMULATION.md)
for the shared rollout submodule setup, artifacts, launch arguments,
and compute metrics. This launch uses the existing DDS configuration.
