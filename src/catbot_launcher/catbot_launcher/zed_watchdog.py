"""Supervise a dedicated ZED launch process using raw image health."""

import argparse
import math
import os
import signal
import subprocess
import time
from dataclasses import dataclass


def green_fraction(msg):
    """Sample a bounded grid, respecting channel order and padded rows.

    Return None for unsupported encodings or malformed images. Treat these as
    unhealthy rather than silently disabling monitoring.
    """
    formats = {
        "rgb8": (3, 0, 1, 2),
        "bgr8": (3, 2, 1, 0),
        "rgba8": (4, 0, 1, 2),
        "bgra8": (4, 2, 1, 0),
    }
    if msg.encoding.lower() not in formats:
        return None
    channels, red, green, blue = formats[msg.encoding.lower()]
    if (
        msg.width < 1
        or msg.height < 1
        or msg.step < msg.width * channels
        or len(msg.data) < msg.step * msg.height
    ):
        return None
    count = bad = 0
    for y in range(0, msg.height, max(1, msg.height // 32)):
        for x in range(0, msg.width, max(1, msg.width // 32)):
            offset = y * msg.step + x * channels
            r, g, b = (msg.data[offset + c] for c in (red, green, blue))
            bad += g >= 80 and g >= r + 50 and g >= b + 50
            count += 1
    return bad / count


@dataclass
class Health:
    started: float
    startup_timeout: float = 60.0
    frame_timeout: float = 10.0
    bad_seconds: float = 5.0
    green_threshold: float = 0.98
    last_frame: float | None = None
    last_stamp: tuple | None = None
    stamp_changed: float | None = None
    bad_since: float | None = None

    def observe(self, msg, now):
        self.last_frame = now
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp != self.last_stamp:
            self.last_stamp, self.stamp_changed = stamp, now
        fraction = green_fraction(msg)
        if fraction is None or fraction >= self.green_threshold:
            if self.bad_since is None:
                self.bad_since = now
        else:
            self.bad_since = None

    def failure(self, now):
        if self.last_frame is None:
            return (
                "no first image" if now - self.started >= self.startup_timeout else None
            )
        if now - self.last_frame >= self.frame_timeout:
            return "image stream stopped"
        if (
            self.stamp_changed is not None
            and now - self.stamp_changed >= self.frame_timeout
        ):
            return "image timestamp stopped advancing"
        if self.bad_since is not None and now - self.bad_since >= self.bad_seconds:
            return "persistent green or unsupported/malformed images"
        return None


def stop_group(process, timeout=10.0):
    """Close the SDK gracefully, then clear surviving children before reopening."""
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            process.poll()  # Reap the launch parent, even if its children survive.
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                break
            time.sleep(0.1)
        else:
            continue
        break
    process.wait(timeout=timeout)
    try:
        os.killpg(process.pid, 0)
    except ProcessLookupError:
        return
    raise RuntimeError("Camera process group still exists; refusing overlapping launch")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-model", default="zed2i")
    parser.add_argument("--camera-name", default="zed")
    parser.add_argument("--serial-number", default="0")
    parser.add_argument("--config-file")
    parser.add_argument("--topic", help="Raw sensor_msgs/Image topic")
    parser.add_argument("--startup-timeout", type=float, default=60.0)
    parser.add_argument("--frame-timeout", type=float, default=10.0)
    parser.add_argument("--bad-seconds", type=float, default=5.0)
    parser.add_argument("--green-threshold", type=float, default=0.98)
    parser.add_argument("--cooldown", type=float, default=10.0)
    parser.add_argument("--max-restarts", type=int, default=3)
    args = parser.parse_args(argv)
    for name in ("startup_timeout", "frame_timeout", "bad_seconds", "cooldown"):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    if not 0 < args.green_threshold <= 1 or args.max_restarts < 0:
        parser.error(
            "green threshold must be in (0, 1]; max restarts must be nonnegative"
        )

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    command = [
        "ros2",
        "launch",
        "catbot_launcher",
        "main.launch.py",
        "enable_foxglove:=false",
        f"camera_model:={args.camera_model}",
        f"camera_name:={args.camera_name}",
        f"serial_number:={args.serial_number}",
    ]
    if args.config_file:
        command.append(f"config_file:={args.config_file}")
    topic = args.topic or f"/{args.camera_name}/zed_node/rgb/image_rect_color"
    rclpy.init(args=[])
    node = rclpy.create_node("zed_watchdog")
    process = None
    subscription = None
    restarts = 0

    def terminate(signum, frame):
        raise KeyboardInterrupt

    previous_sigterm = signal.signal(signal.SIGTERM, terminate)
    try:
        while rclpy.ok():
            # New subscription drops queued frames from the previous camera instance.
            health = Health(
                time.monotonic(),
                args.startup_timeout,
                args.frame_timeout,
                args.bad_seconds,
                args.green_threshold,
            )
            subscription = node.create_subscription(
                Image,
                topic,
                lambda msg, health=health: health.observe(msg, time.monotonic()),
                qos_profile_sensor_data,
            )
            node.get_logger().info(f"Starting camera; watching {topic}")
            process = subprocess.Popen(command, start_new_session=True)
            reason = None
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.2)
                reason = health.failure(time.monotonic())
                if process.poll() is not None:
                    reason = f"camera launch exited ({process.returncode})"
                if reason:
                    break
            stop_group(process)
            process = None
            node.destroy_subscription(subscription)
            subscription = None
            if not rclpy.ok():
                break
            node.get_logger().error(f"Camera unhealthy: {reason}")
            if restarts >= args.max_restarts:
                node.get_logger().fatal(
                    "Restart budget exhausted; camera stopped. Inspect camera/SDK logs."
                )
                return 1
            restarts += 1
            deadline = time.monotonic() + args.cooldown
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)
        if process is not None:
            stop_group(process)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
