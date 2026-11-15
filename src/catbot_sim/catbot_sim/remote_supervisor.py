import json
import os
import select
import signal
import subprocess
import sys
import time


def main():
    config = json.loads(sys.argv[1])
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    for sig in (signal.SIGHUP, signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    child = subprocess.Popen(
        ["bash", "-lc", config["command"]],
        start_new_session=True,
        stdin=subprocess.DEVNULL,
    )
    try:
        deadline = time.monotonic() + 10
        while child.poll() is None and not stopping:
            ready, _, _ = select.select([sys.stdin], [], [], 1)
            if ready:
                if not os.read(sys.stdin.fileno(), 4096):
                    break
                deadline = time.monotonic() + 10
            if time.monotonic() > deadline:
                break
    finally:
        try:
            os.killpg(child.pid, signal.SIGINT)
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGTERM)
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        except ProcessLookupError:
            pass
        finally:
            # A reaped launch leader does not prove that every descendant exited.
            try:
                os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    sys.exit(child.returncode or 0)


if __name__ == "__main__":
    main()
