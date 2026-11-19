# Orin Supervisor deployment

`physical_orin.launch.py` and `simulation_orin.launch.py` start the Catbot
`catbot_supervisor/orin` service host alongside the sensing/kinematics bridge.
They do not launch a policy directly, start the Pi over SSH, or enable motion.
The operator starts the Pi's own Foundation/RPi Supervisor launch separately.
`physical_pi` is deprecated because it bypasses this ownership model;
`simulation_pi` retains its legacy inference experiment path.

Orin-owned settings live in `src/catbot_launcher/config/orin_supervisor.yaml`.
Existing scenario arguments select policy artifacts, device and worker paths.
The supervisor receives the scenario name and all policy settings; policy process
start belongs to Pi scripts, after Orin preflight succeeds. Both managed hardware
and simulation validate the bridge's `/policy/contract` before reporting ready.
No unsupervised policy publisher is added by either Orin-hosted scenario.

```bash
# On Orin, after building and sourcing the pinned packages:
ros2 launch catbot_launcher physical_orin.launch.py
# For simulation instead:
ros2 launch catbot_launcher simulation_orin.launch.py enable_camera:=false
# On Pi, build/source the SAME Catbot interface revision, then:
ros2 launch catbot_supervisor rpi.launch.py
ros2 service call /rpi/supervisor/script catbot_interfaces/srv/RunScript '{name: startup}'
# Startup can move hardware. Inspect ready before requesting run.
ros2 service call /rpi/supervisor/run std_srvs/srv/Trigger '{}'
```

Simulation also waits for a Pi protocol peer to start the policy and forward/ack
its proposals. The scenario test supplies a peer, not a hardware Pi. A full robot
simulation of Pi body maintenance is not added here. Do not direct simulation
commands at a physical Foundation instance.

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
