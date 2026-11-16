"""ROS-free pipe framing and process-group cleanup used by the bridge."""

import json
import os
import signal
import subprocess

from catbot_policy.protocol import MAX_LINE_BYTES, VERSION


class WorkerProcess:
    def check_startup(self, now):
        if not hasattr(self, "contract") and now - self.started > self.startup_timeout:
            raise RuntimeError("Worker initialization timed out")

    def read_worker(self):
        line = self.process.stdout.readline(MAX_LINE_BYTES + 1)
        if not line:
            raise RuntimeError("Simulation worker exited")
        if len(line.encode()) > MAX_LINE_BYTES or not line.endswith("\n"):
            raise ValueError("Invalid or oversized worker response")
        value = json.loads(line)
        if not isinstance(value, dict):
            raise ValueError("Worker response must be an object")  # noqa: TRY004 - invalid protocol value
        if "version" in value and value["version"] != VERSION:
            raise ValueError("Unsupported worker protocol")
        return value

    def close(self):
        self.stopping.set()
        try:
            os.killpg(self.process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(self.process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.process.wait()
        # uv may exit before its worker; also clean up remaining group members.
        try:
            os.killpg(self.process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.thread.join(timeout=1)
        self.process.stdin.close()
        self.process.stdout.close()
