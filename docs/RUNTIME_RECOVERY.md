# Sensor timing and endstop recovery

Physical feedback uses odometry capture time as its reference. IMU and joint
callbacks retain up to 128 samples, and snapshot construction selects the
nearest fresh samples to that reference. Camera pose processing can lag live
IMU publication by more than the allowed inter-sensor skew even when both
streams are healthy. Pairing the newest arrivals mixes different capture times.
The existing 250 ms freshness and 100 ms skew defaults still reject stale,
missing, or genuinely unsynchronized measurements. No timestamps are rewritten.

## Recover an endstop trip without calibration

An endstop fault does not by itself require motor calibration or homing. This
procedure assumes the controller has retained its calibrated position reference;
a reboot, lost reference, or encoder fault requires separate diagnosis.

1. Support the robot and use the operator emergency stop. Wait for
   `/rpi/supervisor/status` to show emergency and `/body/state/error` to show all
   axes IDLE (state 1). Policy must remain stopped.
2. With axes disarmed, physically release the triggered limit switch if the
   mechanism permits this safely. Do not disable the switch or command motion
   farther into the stop. Clear requests cannot repair an active physical cause.
3. Clear errors on the affected axes. In Foxglove call `/body/control/motor`
   (`catbot_interfaces/srv/MotorCommand`) with:

   ```yaml
   actuator_names: [blf, fra]
   operation: clear_errors
   state: 0
   ```

   Replace the example names with the axes reporting the endstop flags. This
   sends only Clear_Errors; it neither calibrates nor arms. Confirm fresh
   heartbeats and zero axis/detail fault flags on `/body/state/error`.
4. Call `/rpi/supervisor/recover` (`std_srvs/srv/Trigger`, request `{}`). Wait for
   idle status. Recovery clears the motion latch without moving or arming axes.
5. When ready to enable holding torque, call `/rpi/supervisor/script`
   (`catbot_interfaces/srv/RunScript`) with `{name: clear_errors}`.
   This existing script idles and clears errors, then explicitly arms all axes;
   it performs no calibration or homing and sends no position target. Wait for
   holding status and ready state 8 on all axes before requesting policy motion.

If faults recur, stop here and inspect the switch, wiring, and mechanism.
Service acceptance is not completion; inspect supervisor status after each step.
