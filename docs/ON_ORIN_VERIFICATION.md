# On-Orin verification — 2026-11-19

The supervisor was built and exercised on the aarch64 Orin with ROS Humble,
using an isolated temporary checkout, localhost-only ROS domains 187/188, and a
local Pi protocol peer. The physical Pi was offline and was not contacted.
The deployed `/home/catbot/projects/ros2_ws` checkout was not modified.

Runtime sources: Catbot `c9592c0`, Orin `395ab07`. The ROS test cleanup guard
added with this report changes the test harness only. The build included the
interfaces, policy, supervisor, simulation bridge and Orin launcher packages.

## Real supervisor, real CUDA worker

The existing selected policy was used without modifying its metadata or actor.
Its actor SHA-256 was
`87b931d4c908736aa48827c84878f5f8b3bd7e7d2ae73e0406f1f84387b74bea`.
The prepared CUDA environment used Python 3.10.12 / PyTorch 2.8.0 and identified
the device as Orin. The export metadata specified PyTorch 2.14.0; the existing
explicit load-compatibility path was used.

Verified on the target:

- Heartbeat, status queries and real CPU/GPU/thermal/memory/disk telemetry.
- An injected 100 ms clock offset rejected policy start; restoring the local test
  peer clock allowed preflight after the real resource and CUDA allocation checks.
- The selected actor loaded, its checksum verified, and CUDA produced finite
  12-actuator proposals with synthetic feedback.
- Verified bundle ID and run ID/scenario/git-commit tags were published.
- Restart returned acceptance, cleaned up the old policy, and produced a new PID,
  session and verified readiness without a cleanup-pending retry.
- Stale feedback produced stale-action degradation without automatic restart.
- Explicit stop cleaned up the policy process group.
- Stopping only the Pi-probe stream expired the lease and stopped the policy,
  even while clock replies continued; resumed probes did not restart it.
- A temporary copy with a deliberately wrong checksum failed loading and never
  reported verified/ready.

The real-CUDA check acknowledged every proposal as unforwarded. No body-control
commands were published by that check, and accepted-publication/history counters
stayed at zero. This was not a physical control session.

A representative preflight resource sample was 20.07% aggregate CPU, 0% sampled
GPU load, 49.625 C, 2.28 GB RAM used of 7.98 GB, and 386.6 GB free disk. This is a
point sample, not a steady-state policy benchmark or a Pi headroom comparison.

## Orin scenario and diagnostics

Both repository ROS tests passed on the Orin: `2 passed in 14.40s`. These launch
the real scenario/supervisor/bridge adapters with fake policy/kinematics workers
and a local protocol peer. They do not launch or contact a physical Pi or
Foundation. Simulated body messages stay in the localhost-only test domain.

The first scenario run, concurrent with separate Torch comparison processes,
hit the policy watchdog and exited. Its cleanup then raised ProcessLookupError,
masking the original failure and leaving the test's rclpy context unclosed.
The cleanup now tolerates an already-exited launch process and always closes
rclpy. An isolated scenario repeat passed in 8.19s, followed by the successful
sequential two-test run. No watchdog threshold or test assertion was relaxed.
This does not establish that concurrent background workloads are safe.

## Numerical comparison and remaining boundaries

Five fixed synthetic feedback/command steps compared 60 actuator target values:

| Reference / candidate | Maximum absolute target difference |
| --- | --- |
| PyTorch 2.14 CPU / PyTorch 2.8 CPU | 0.000263214 degrees |
| PyTorch 2.8 CPU / PyTorch 2.8 CUDA | 0.003704071 degrees |
| PyTorch 2.14 CPU / PyTorch 2.8 CUDA | 0.003967285 degrees |

The additional diagnostic tolerance of 0.0001 degrees was not met. The tolerance
was not widened and runtime precision was not changed. These measurements do
not establish strict CPU/CUDA parity, broad numerical equivalence or learned
gait quality.

Still pending: actual Pi-to-Orin clock/network behavior and Pi-script-driven
physical operation, motor safety/calibration, and Pi CPU headroom. Catbot's
required review/merge and the final merged dependency pin also remain separate
completion gates; a tested candidate is not a merged deployment.

Raw check reports and logs were retained in the target's temporary directory
`/tmp/catbot-supervisor-check-5Yv1yt` for inspection. Test processes were stopped
when verification ended.
