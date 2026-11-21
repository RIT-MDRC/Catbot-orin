# Orin Supervisor deployment

`physical_orin.launch.py` starts the Catbot `catbot_supervisor/orin` service host
alongside the sensing/kinematics bridge. By default it also starts the Pi's
Foundation/RPi Supervisor launch over SSH. It does not start policy inference or
enable motion. Use `start_pi:=false` when the Pi services are managed separately.
`physical_pi` is deprecated because it bypasses this ownership model;
`simulation_pi` retains its legacy inference experiment path.

Orin-owned settings live in `src/catbot_launcher/config/orin_supervisor.yaml`.
Existing scenario arguments select policy artifacts, device and worker paths.
The supervisor receives the scenario name and all policy settings; policy process
start is an explicit operator action after Orin preflight succeeds. Hardware
policy validates the bridge's `/policy/contract` before reporting ready.

```bash
# On Orin, after building and sourcing the pinned packages:
ros2 launch catbot_launcher physical_orin.launch.py
# Orin-only orchestration when the Pi services are already running separately:
ros2 launch catbot_launcher physical_orin.launch.py start_pi:=false
# With start_pi:=false, start the Pi separately after sourcing its ROS overlay:
ros2 launch catbot_supervisor rpi.launch.py
```

The SSH path uses `remote_host`, `remote_user`, `remote_environment`,
`remote_ros_overlay`, `remote_mapping_file`, and `remote_network_interface` from
`scenarios.yaml`. Prepare/build the Pi first and configure passwordless SSH from
Orin. The default overlay is `/home/catbot/.local/share/catbot/ros/install/setup.bash`,
created by `catbot setup` on the Pi. The shared environment file supplies ROS and
DDS settings before the overlay is sourced. Stop separate Pi/Orin launches before
using the combined launch. Stopping it closes the SSH lease; the remote supervisor
shuts down its Pi process group, with escalation if it does not exit.

These launches start infrastructure only: camera/sensors, kinematics/visualization
bridge, diagnostics, Foxglove Bridge and the Orin service host. Pi boot only
disarms. No policy, calibration, homing or registered control script starts
automatically. With fresh Pi probes and passing preflight, stopped policy is a
healthy `standby` heartbeat, not a degraded condition. Actual resource, clock,
worker and action-stream failures remain reported; a crash is not standby.
Pi status is `idle` (disarmed) until a script is explicitly requested. Service
readiness does not mean that the robot has already been armed or calibrated.

Import the pinned Catbot's
`external/Catbot/src/catbot_supervisor/foxglove/operator-controls.json` into
Foxglove (Layouts > Import from file). This is exported-layout JSON, not the SDK
notebook format. Connect to `ws://<ORIN-IP>:8765` using the shared ROS domain and
discovery settings. Then choose operations in Foxglove:

- **Home robot** requests the ordered `home_robot` script on the Pi. It requires
  operator clearance before abduction homing and leaves homed axes armed/holding.
- **Motor calibration** requests full calibration for all 12 actuators in
  parallel, then homes and arms knees, flexion, and back abduction in order. It
  moves `flf, frf` to +30 degrees, waits for clearance confirmation, then homes
  and arms `fla, fra`. Each homing group must fully return to IDLE before arming;
  the sequence ends in local holding with policy forwarding disabled. See the
  pinned Catbot supervisor README for the completion checks and timeouts.
  Script arming requests closed loop (state 8) only and confirms readiness; it
  does not write trajectory limits, controller modes, or a hold-position target.
  Explicit position moves select position/trapezoidal mode and enter closed loop
  automatically, preserving speed/acceleration limits. Policy forwarding still
  requires the separate RUN request. Emergency latches Foundation move/arm
  commands until the Pi's explicit recovery confirms idle and clears the latch.
- **Start policy (no motor commands)** requests only the Orin `policy` job.
  It does not run any robot script, clear faults, home or arm motors.
- **RUN — ENABLE POLICY MOTION** is a separate explicit request after the motors
  are ready and policy readiness/fresh actions are verified.
- **Pause** holds the last targets. **Stop policy — keep holding** closes Pi
  forwarding before stopping inference; **E-STOP** disarms and stops policy.
- **Run named script** accepts any installed registry name shown in
  `available_scripts` on `/rpi/supervisor/status`, including future entry points.

Starting inference and executing a robot control script are independent. Local
scripts need Pi/body health, not an active policy; scripts needing the Orin link
must include an explicit `wait_link` step. Starting policy may fail until
preflight/providers are healthy, and never implicitly enables robot motion.
The CPU kinematics worker is infrastructure and runs at launch even in hardware
mode; it needs the saved run's model/config to match its installed training
revision. A config-schema mismatch in that worker must be repaired with the
existing `catbot_policy.rollout_setup.prepare_rollout` helper before the full
stack can become healthy. It is not evidence that a trained policy was started.

`simulation_orin.launch.py` is all-local: it starts its policy automatically and
never starts Pi hardware services, regardless of `start_pi`. It validates the
simulator contract and sends commands directly to the simulator. Use the physical
launch above for operator-controlled hardware standby.

The shared Catbot package documents the API. Useful endpoints:

- `/orin/supervisor/heartbeat` (20 Hz): link facts, process existence, verified
  readiness, bundle hash, load error and preflight verdict.
- `/orin/supervisor/telemetry` (1 Hz): CPU/GPU, thermal, memory/disk and run tags.
- `/orin/supervisor/job`: legacy start bool plus explicit start/stop/restart.
- `/orin/supervisor/get_status`: version 1 JSON, optionally one job.
- `/rpi/supervisor/script` with `restart_policy`: safe explicit robot-side restart;
  the Pi returns to ready, awaiting an explicit run request.

Watchdog decision: keep fail-latched behavior. There is no automatic restart.
Sequence gaps are judged on the Pi. A restart requested directly while the Pi is
ready/running will trip its safety gate; use the Pi restart script instead.

Readiness is not process existence. Orin must pass timestamp-based skew bounds,
resource limits and a device probe before a policy starts. Actual actor checksum
verification/load and the bridge contract then determine readiness. Missing or
stale facts fail closed. ROS clocks must be synchronized; clock probing measures
an offset bound and does not configure NTP/chrony.

Recording and offline/shadow-simulation jobs remain deferred. This change adds
resource telemetry and basic run tags, not a recording or job-queue coordinator.

## Verification boundary

Each repository tests its own code and shared ROS contracts. Catbot exercises its
real Pi/Orin nodes with hardware/resource doubles; Orin tests its real scenario
launch with a Pi protocol peer and fake inference/kinematics workers. No separate
joint-repository test framework is introduced.

Before marking target verification complete, use an isolated ROS domain with no
physical Foundation subscribed and check on the real Orin: heartbeat/telemetry,
preflight including skew, Pi-script-driven start/stop/restart, verified bundle ID,
stale action degradation, probe-lease expiry, and process cleanup. A simulated
peer does not establish physical CAN safety, camera health, or CUDA compatibility.
