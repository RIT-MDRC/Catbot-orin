"""SSH supervisor with a stdin lease; remote process group dies on disconnect."""

import argparse
import json
import shlex
import signal
import subprocess
import threading

REMOTE = r"""
import json, os, select, signal, subprocess, sys, time
config = json.loads(sys.argv[1])
stopping = False
def stop(*_):
    global stopping
    stopping = True
for sig in (signal.SIGHUP, signal.SIGINT, signal.SIGTERM):
    signal.signal(sig, stop)
child = subprocess.Popen(["bash", "-lc", config["command"]], start_new_session=True, stdin=subprocess.DEVNULL)
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
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.0.3")
    parser.add_argument("--user", required=True)
    parser.add_argument("--workspace", required=True)
    parser.add_argument("--policy-bundle", required=True)
    parser.add_argument("--mapping-file", required=True)
    parser.add_argument("--environment", default="/opt/ros/humble/setup.bash")
    args, _ = parser.parse_known_args()
    if (
        args.host.startswith("-")
        or args.user.startswith("-")
        or any(c.isspace() for c in args.host + args.user)
    ):
        raise ValueError("Invalid SSH destination")
    quote = shlex.quote
    command = " && ".join(
        [
            "source " + quote(args.environment),
            "source " + quote(args.workspace.rstrip("/") + "/install/setup.bash"),
            "exec ros2 launch catbot_policy catbot.launch.py backend:=simulation "
            + " ".join(
                quote(key + ":=" + value)
                for key, value in (
                    ("policy_bundle", args.policy_bundle),
                    ("mapping_file", args.mapping_file),
                )
            ),
        ]
    )
    remote_command = shlex.join(
        ["python3", "-u", "-c", REMOTE, json.dumps({"command": command})]
    )
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    child = subprocess.Popen(
        [
            "ssh",
            "-T",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=10",
            "-o",
            "ServerAliveInterval=3",
            "-o",
            "ServerAliveCountMax=3",
            args.user + "@" + args.host,
            remote_command,
        ],
        stdin=subprocess.PIPE,
    )
    try:
        while child.poll() is None and not stop.wait(1):
            child.stdin.write(b"lease\n")
            child.stdin.flush()
    except BrokenPipeError:
        pass
    finally:
        try:
            child.stdin.close()
        except BrokenPipeError:
            pass
        try:
            child.wait(timeout=12)
        except subprocess.TimeoutExpired:
            child.terminate()
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
    if not stop.is_set() and child.returncode:
        raise SystemExit(child.returncode)
