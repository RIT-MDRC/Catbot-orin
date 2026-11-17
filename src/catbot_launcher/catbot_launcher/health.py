"""ROS-independent health checks used by the diagnostics publisher."""

from dataclasses import dataclass

OK, WARN, ERROR, STALE = range(4)


@dataclass
class StreamHealth:
    started: float
    timeout: float = 2.0
    startup_timeout: float = 60.0
    last_received: float | None = None
    last_stamp: tuple | None = None
    stamp_changed: float | None = None
    message: object = None

    def observe(self, message, now):
        self.last_received, self.message = now, message
        stamp = getattr(getattr(message, "header", message), "stamp", None)
        if stamp is not None:
            stamp = (stamp.sec, stamp.nanosec)
            if stamp != self.last_stamp:
                self.last_stamp, self.stamp_changed = stamp, now

    def status(self, now):
        if self.last_received is None:
            if now - self.started < self.startup_timeout:
                return WARN, "Waiting for first message"
            return ERROR, "No messages received; check the publisher and connection"
        if now - self.last_received >= self.timeout:
            return ERROR, "Stream stopped; no recent messages"
        if self.last_stamp is not None:
            if self.last_stamp == (0, 0):
                return ERROR, "Invalid zero timestamp"
            if now - self.stamp_changed >= self.timeout:
                return ERROR, "Timestamp stopped advancing"
        return OK, "Receiving messages"


def image_status(health, now):
    reason = health.failure(now)
    if reason:
        return ERROR, reason
    if health.last_frame is None:
        return WARN, "Waiting for first image"
    if health.bad_since is not None:
        return WARN, "Green or unsupported/malformed images; checking persistence"
    return OK, "Receiving images with advancing timestamps"


def joint_status(message):
    if not message.actuators:
        return ERROR, "No actuator feedback"
    invalid = [a.actuator_name for a in message.actuators if not a.valid]
    if invalid:
        return ERROR, "Invalid actuator feedback: " + ", ".join(invalid)
    return OK, "Actuator feedback valid"


def motor_status(message):
    """Fault reports are latched/change-only, so do not expire them by age."""
    faults, missing, idle, values = [], [], [], {}
    for actuator in message.actuators:
        name = actuator.actuator_name
        values[f"{name}/axis_id"] = actuator.axis_id
        values[f"{name}/axis_state"] = actuator.axis_state
        values[f"{name}/ready"] = actuator.ready
        values[f"{name}/heartbeat_available"] = actuator.heartbeat_available
        if not actuator.heartbeat_available:
            missing.append(name)
        elif not actuator.ready:
            idle.append(name)
        for group in ("axis", "motor", "encoder", "sensorless", "controller"):
            flags = getattr(actuator, f"{group}_error_flags")
            messages = getattr(actuator, f"{group}_error_messages")
            values[f"{name}/{group}_error_flags"] = hex(flags)
            if messages:
                values[f"{name}/{group}_errors"] = ", ".join(messages)
            if flags:
                faults.append(f"{name} {group}: {', '.join(messages) or hex(flags)}")
    if faults or missing:
        reasons = faults + (
            ["Missing heartbeat: " + ", ".join(missing)] if missing else []
        )
        return ERROR, "; ".join(reasons), values
    if idle:
        return WARN, "Motors idle/not ready: " + ", ".join(idle), values
    if not message.actuators:
        return ERROR, "No motor fault state reported", values
    return OK, "No reported motor faults; motors ready", values
