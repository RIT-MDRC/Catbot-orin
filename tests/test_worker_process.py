import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest
from catbot_sim.worker_process import WorkerProcess

FAKE = (
    Path(__file__).resolve().parents[1]
    / "external/Catbot/tests/fixtures/fake_worker.py"
)


@pytest.fixture
def worker():
    workers = []

    def spawn(mode):
        instance = WorkerProcess()
        instance.stopping = threading.Event()
        instance.process = subprocess.Popen(
            [sys.executable, str(FAKE)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
            start_new_session=True,
            env={**os.environ, "FAKE_WORKER_MODE": mode},
        )
        instance.thread = threading.Thread(target=lambda: None)
        instance.thread.start()
        instance.started, instance.startup_timeout = time.monotonic(), 0.1
        workers.append(instance)
        return instance

    yield spawn
    for instance in workers:
        instance.close()
        assert instance.process.poll() is not None


@pytest.mark.parametrize(
    ("mode", "error"),
    [
        ("oversized", "oversized"),
        ("malformed", "Expecting"),
        ("version", "Unsupported"),
        ("array", "object"),
    ],
)
def test_bridge_rejects_bad_worker(worker, mode, error):
    with pytest.raises(ValueError, match=error):
        worker(mode).read_worker()


def test_bridge_startup_timeout(worker):
    instance = worker("timeout")
    time.sleep(0.15)
    with pytest.raises(RuntimeError, match="timed out"):
        instance.check_startup(time.monotonic())


def test_bridge_shutdown_escalates(worker):
    instance = worker("ignore_term")
    assert instance.read_worker()["type"] == "ready"
    instance.close()
    assert instance.process.returncode == -signal.SIGKILL


def test_bridge_roundtrip(worker):
    instance = worker("ready")
    instance.read_worker()
    instance.process.stdin.write('{"type":"ping"}\n')
    instance.process.stdin.flush()
    assert instance.read_worker()["request"] == {"type": "ping"}
