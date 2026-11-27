# Pi settings from the Orin launch

The physical launch forwards Pi settings with the `rpi_` prefix:

```bash
ros2 launch catbot_launcher physical_orin.launch.py \
  rpi_heartbeat_timeout_seconds:=1.0 \
  rpi_encoder_timeout_seconds:=0.5
```

These are startup overrides. Restart the existing hardware launch to apply them;
omitting the overrides restores the Pi defaults (0.5 seconds for motor heartbeat
and 0.25 seconds for encoder freshness). A larger budget accepts older feedback;
it does not remove the CAN receive backlog.

Each node retains its own settings: Pi action/link and state-stream limits are
separate from Foundation's per-motor freshness budgets and Orin policy feedback
limits. Only shared firmware meanings, such as motor-state numbers, use the same
Foundation protocol data.

`rpi_hardware_config_file` selects the existing Foundation `pinconfig.json` on the
Pi. It owns CAN bus settings, actuator axis IDs, controller board representatives
and motion profiles. `rpi_protocol_config_file` selects Foundation's protocol JSON
and referenced DBC on the Pi. File paths are remote Pi paths, not Orin paths.

See [the complete Pi configuration reference](../external/Catbot/src/catbot_supervisor/CONFIGURATION.md)
for the remaining health, recovery, script, homing and scheduler settings. Use
`ros2 launch catbot_launcher physical_orin.launch.py --show-args` to inspect the
installed launch arguments without starting robot nodes.
