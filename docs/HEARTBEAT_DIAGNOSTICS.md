# Lightweight Foundation heartbeat log

After restarting Foundation, diagnostics are written to:

```text
/home/catbot/.local/state/catbot/heartbeat/heartbeat.jsonl
```

The existing CAN receiver supplies counters and timestamps. No additional CAN
socket, ROS subscriber, or recording process is created. Writes run on a bounded
background queue. Baseline snapshots occur every five seconds; heartbeat changes
are coalesced to at most one record per second. The active file rotates at 5 MiB
and retains two backups (`heartbeat.jsonl.1` and `.2`).

Each JSON line contains the axis states, heartbeat and encoder ages, decoded
faults, and heartbeat lost/restored transitions. It also includes:

- `receive`: processed-frame counts, per-axis heartbeat counts and ages,
  kernel-to-callback delay, callback duration, notifier exceptions and the
  existing socket's receive-buffer size.
- `receive.*.tx_commands_accepted`: successful CAN send requests by command ID.
  `0xc` is position input. These counters confirm socket acceptance, not motor
  execution or controller acknowledgement.
- `host`: process CPU seconds, system load and CAN interface error/drop counters.
  Compare changes between records to derive rates. CPU seconds divided by elapsed
  wall time gives CPU usage relative to one core.
- `receive.*.socket_memory`: Linux socket receive allocation, buffer limit,
  backlog bytes and cumulative `drops_total`, read from the existing socket with
  `SO_MEMINFO`. Unsupported platforms return null. Compare socket-drop deltas as
  well as interface counters: unchanged interface drops do not establish that
  the individual receiver socket kept every frame. Linux defines these fields in
  [sock_diag.h](https://github.com/torvalds/linux/blob/master/include/uapi/linux/sock_diag.h).
- `writer_delay_ms` and `queue_dropped`: diagnostic backlog and discarded log
  records. A full queue drops diagnostics instead of blocking control callbacks.

The receipt timestamps distinguish delayed processing from missing controller
messages. A rising kernel-to-callback delay points to queued receive work; a
notifier exception identifies a failed receive callback. Wall-clock steps can
affect the kernel-delay calculation, so both wall and monotonic times are kept.
Snapshots are diagnostic samples, not an authoritative actuation acknowledgement.

Restart the normal launch once to load the code, reproduce the issue normally,
then retain the active log and both backups. No separate live capture is needed:

```bash
tail -n 20 ~/.local/state/catbot/heartbeat/heartbeat.jsonl
```

The Foundation startup ROS parameter `heartbeat_diagnostic_log` changes the path;
an empty string disables this logger. Logging failures are reported once through
the normal Foundation ROS logger and do not stop CAN processing.

Routine encoder debug logs are limited to once per motor per second by default;
all samples still update feedback and all fault/state-transition logs remain
active. `encoder_log_period_seconds` configures this Foundation startup setting
through the Pi launch, or `rpi_encoder_log_period_seconds` through the Orin
physical launch. This limiter removes per-frame file output from the hot path
without raising motor freshness thresholds.
