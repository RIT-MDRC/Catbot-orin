"""SSH supervisor with a stdin lease; remote process group dies on disconnect."""

import argparse
import json
import math
import shlex
import signal
import subprocess
import threading
from pathlib import Path

from catbot_policy.configuration import POLICY_DEFAULTS, POLICY_PARAMETERS

REMOTE = Path(__file__).with_name("remote_supervisor.py").read_text()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--role", choices=["both", "foundation", "policy"], default="both"
    )
    parser.add_argument("--policy-device", default="cpu")
    for name in (
        "run_id",
        "checkpoint",
        "policy_entity",
        "policy_project",
        "policy_artifacts_directory",
    ):
        parser.add_argument(
            "--" + name.replace("_", "-"), default=POLICY_DEFAULTS[name]
        )
    parser.add_argument("--policy-worker-environment", default="auto")
    parser.add_argument(
        "--policy-worker-sync", choices=["auto", "true", "false"], default="auto"
    )
    parser.add_argument("--policy-python", default="auto")
    parser.add_argument(
        "--policy-torch-compatibility",
        choices=["auto", "exact", "load"],
        default="auto",
    )
    parser.add_argument("--policy-ros-python", default="/usr/bin/python3")
    parser.add_argument("--policy-worker-project", required=True)
    parser.add_argument("--policy-uv-executable", default="uv")
    parser.add_argument("--policy-worker-timeout", type=float, default=2.0)
    parser.add_argument("--policy-startup-timeout", type=float, default=120.0)
    parser.add_argument("--feedback-timeout", type=float, default=0.25)
    parser.add_argument("--host", default="192.168.0.3")
    parser.add_argument("--user", required=True)
    parser.add_argument("--workspace", required=True)
    parser.add_argument("--policy-bundle", required=True)
    parser.add_argument("--mapping-file", required=True)
    parser.add_argument("--environment", default="/opt/ros/humble/setup.bash")
    args = parser.parse_args()
    if not all(
        math.isfinite(v) and v > 0
        for v in (
            args.feedback_timeout,
            args.policy_worker_timeout,
            args.policy_startup_timeout,
        )
    ):
        parser.error("Policy timeouts must be positive and finite")
    if (
        args.host.startswith("-")
        or args.user.startswith("-")
        or any(c.isspace() for c in args.host + args.user)
    ):
        raise ValueError("Invalid SSH destination")
    quote = shlex.quote
    setup = [
        "source " + quote(args.environment),
        "source " + quote(args.workspace.rstrip("/") + "/install/setup.bash"),
    ]
    commands = []
    if args.role in ("both", "foundation"):
        # Preserve the foundation package's existing module-relative import layout.
        locate = "from importlib.util import find_spec; from pathlib import Path; print(Path(find_spec('foundation').origin).parent)"
        setup.append(
            'export PYTHONPATH="$(python3 -c ' + quote(locate) + '):${PYTHONPATH:-}"'
        )
        commands.append(shlex.join(["ros2", "run", "foundation", "main"]))
    if args.role in ("both", "policy"):
        policy_parameters = {
            name: getattr(args, name)
            for name in POLICY_PARAMETERS
            if name != "simulation_mode"
        }
        policy_parameters["simulation_mode"] = True
        invocation = [args.policy_ros_python, "-m", "catbot_policy.node", "--ros-args"]
        for name, value in policy_parameters.items():
            invocation += ["-p", name + ":=" + json.dumps(value)]
        invocation += ["-r", "/simulation/contract:=/policy/contract"]
        commands.append(shlex.join(invocation))
    # Exit as soon as either node stops; the outer supervisor owns their process group.
    stack = (
        "exec " + commands[0]
        if len(commands) == 1
        else "{ " + " & ".join(commands) + " & wait -n; }"
    )
    command = " && ".join([*setup, stack])
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
    stdin = child.stdin
    if stdin is None:
        raise RuntimeError("SSH lease pipe was not created")
    try:
        while child.poll() is None and not stop.wait(1):
            stdin.write(b"lease\n")
            stdin.flush()
    except BrokenPipeError:
        pass
    finally:
        try:
            stdin.close()
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
