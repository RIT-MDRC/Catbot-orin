#!/usr/bin/env python3
"""Exercise ZED SDK capture without ROS or a display; stop on the first error."""

import argparse
from datetime import datetime, timezone
import time


def report(message):
    print(f"{datetime.now(timezone.utc).isoformat()} {message}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fps", type=int, choices=(15, 30), default=30)
    parser.add_argument("--seconds", type=int, default=900)
    parser.add_argument("--serial", type=int, default=39859437)
    parser.add_argument("--depth", choices=("NONE", "NEURAL_LIGHT"), default="NONE")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")

    import pyzed.sl as sl

    camera = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = args.fps
    init.depth_mode = getattr(sl.DEPTH_MODE, args.depth)
    init.set_from_serial_number(args.serial)
    init.sdk_verbose = 1
    report(f"SDK={sl.Camera.get_sdk_version()} HD720 fps={args.fps} "
           f"depth={args.depth} serial={args.serial} duration={args.seconds}s")
    frames = 0
    started = None
    try:
        status = camera.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            report(f"OPEN FAILED: {status}")
            return 1
        config = camera.get_camera_information().camera_configuration
        report(f"Opened {config.resolution.width}x{config.resolution.height} "
               f"at {config.fps} FPS")
        runtime = sl.RuntimeParameters()
        started = last_report = time.monotonic()
        last_frames = 0
        while time.monotonic() - started < args.seconds:
            status = camera.grab(runtime)
            if status != sl.ERROR_CODE.SUCCESS:
                report(f"GRAB FAILED after {time.monotonic() - started:.1f}s, "
                       f"{frames} successful frames: {status}")
                return 1
            frames += 1
            now = time.monotonic()
            if now - last_report >= 10:
                report(f"elapsed={now - started:.1f}s frames={frames} "
                       f"successful_grabs_per_second="
                       f"{(frames - last_frames) / (now - last_report):.1f}")
                last_frames, last_report = frames, now
        report(f"PASS: {frames} successful grabs over "
               f"{time.monotonic() - started:.1f}s; no grab errors")
        return 0
    except KeyboardInterrupt:
        report(f"INTERRUPTED: {frames} successful grabs; test incomplete")
        return 130
    finally:
        camera.close()
        report("Camera closed")


if __name__ == "__main__":
    raise SystemExit(main())
