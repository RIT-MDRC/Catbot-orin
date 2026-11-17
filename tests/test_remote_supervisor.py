"""Exercise lease decisions and signal escalation without SSH or hardware."""

import json
import os
import shlex
import signal
import subprocess
import sys
import time
from unittest.mock import Mock, call

import pytest

from catbot_sim import remote_supervisor as supervisor


@pytest.fixture
def supervise(monkeypatch):
    monkeypatch.setattr(sys, "stdin", Mock())
    child = Mock(pid=123, returncode=0)
    child.poll.return_value = None
    spawn = Mock(return_value=child)
    kill = Mock()
    handlers = {}
    monkeypatch.setattr(sys, "argv", ["supervisor", json.dumps({"command": "worker"})])
    monkeypatch.setattr(supervisor.subprocess, "Popen", spawn)
    monkeypatch.setattr(supervisor.os, "killpg", kill)
    monkeypatch.setattr(
        supervisor.signal, "signal", lambda sig, fn: handlers.update({sig: fn})
    )
    monkeypatch.setattr(
        supervisor.select, "select", Mock(return_value=([sys.stdin], [], []))
    )
    monkeypatch.setattr(supervisor.os, "read", Mock(return_value=b""))
    return child, kill, handlers


def run_supervisor(code=0):
    with pytest.raises(SystemExit) as result:
        supervisor.main()
    assert result.value.code == code


def test_lease_expires_with_open_idle_pipe(supervise, monkeypatch):
    child, kill, _ = supervise
    clock = Mock(side_effect=[100, 109, 111])
    monkeypatch.setattr(supervisor.time, "monotonic", clock)
    select = Mock(return_value=([], [], []))
    monkeypatch.setattr(supervisor.select, "select", select)
    run_supervisor()
    assert select.call_count == 2
    supervisor.os.read.assert_not_called()
    assert kill.call_args_list == [
        call(child.pid, signal.SIGINT),
        call(child.pid, signal.SIGKILL),
    ]


def test_heartbeat_renews_lease_until_disconnect(supervise, monkeypatch):
    child, kill, _ = supervise
    # A heartbeat at t=109 must keep the worker alive beyond the original t=110 lease.
    monkeypatch.setattr(
        supervisor.time, "monotonic", Mock(side_effect=[100, 109, 111, 118])
    )
    monkeypatch.setattr(
        supervisor.select,
        "select",
        Mock(side_effect=[([sys.stdin], [], []), ([], [], []), ([sys.stdin], [], [])]),
    )
    monkeypatch.setattr(supervisor.os, "read", Mock(side_effect=[b"lease\n", b""]))
    run_supervisor()
    assert supervisor.select.select.call_count == 3
    assert supervisor.os.read.call_count == 2
    child.wait.assert_called_once_with(timeout=5)
    kill.assert_any_call(child.pid, signal.SIGKILL)


@pytest.mark.parametrize("sig", [signal.SIGHUP, signal.SIGINT, signal.SIGTERM])
def test_shutdown_signal_cleans_group(supervise, monkeypatch, sig):
    child, kill, handlers = supervise

    def interrupt(*args):
        handlers[sig](sig, None)
        return [], [], []

    monkeypatch.setattr(supervisor.select, "select", interrupt)
    run_supervisor()
    child.wait.assert_called_once_with(timeout=5)
    assert kill.call_args_list == [
        call(child.pid, signal.SIGINT),
        call(child.pid, signal.SIGKILL),
    ]


@pytest.mark.parametrize("timeouts", [1, 2])
def test_shutdown_escalates_in_order(supervise, timeouts):
    child, kill, _ = supervise
    child.wait.side_effect = [subprocess.TimeoutExpired("worker", 0)] * timeouts + [0]
    run_supervisor()
    expected = [signal.SIGINT, signal.SIGTERM]
    waits = [call(timeout=5), call(timeout=3)]
    if timeouts == 2:
        expected.append(signal.SIGKILL)
        waits.append(call())
    assert kill.call_args_list == [
        call(child.pid, sig) for sig in [*expected, signal.SIGKILL]
    ]
    assert child.wait.call_args_list == waits


@pytest.mark.parametrize("already_gone", [False, True])
def test_exited_leader_still_cleans_descendants(supervise, already_gone):
    child, kill, _ = supervise
    child.poll.return_value = 7
    child.returncode = 7
    if already_gone:
        kill.side_effect = ProcessLookupError
    run_supervisor(7)
    assert kill.call_args_list == [
        call(child.pid, signal.SIGINT),
        call(child.pid, signal.SIGKILL),
    ]


def test_select_failure_still_cleans_group(supervise, monkeypatch):
    child, kill, _ = supervise
    monkeypatch.setattr(
        supervisor.select, "select", Mock(side_effect=OSError("pipe failed"))
    )
    with pytest.raises(OSError, match="pipe failed"):
        supervisor.main()
    kill.assert_any_call(child.pid, signal.SIGKILL)


@pytest.mark.parametrize("shutdown", ["disconnect", "signal", "kill", "lease"])
def test_real_worker_is_reaped(tmp_path, shutdown):
    """Real local process groups exercise the same code shipped over SSH."""
    pid_file = tmp_path / "worker.pid"
    worker = tmp_path / "worker.py"
    worker.write_text(
        "import os, signal, time\n"
        + (
            "signal.signal(signal.SIGINT, signal.SIG_IGN)\nsignal.signal(signal.SIGTERM, signal.SIG_IGN)\n"
            if shutdown == "kill"
            else ""
        )
        + f"open({str(pid_file)!r}, 'w').write(str(os.getpid()))\n"
        + "while True: time.sleep(1)\n"
    )
    command = "exec " + shlex.join([sys.executable, str(worker)])
    process = subprocess.Popen(
        [sys.executable, supervisor.__file__, json.dumps({"command": command})],
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    worker_pid = None
    try:
        deadline = time.monotonic() + 5
        while not pid_file.exists() and time.monotonic() < deadline:
            assert process.poll() is None
            time.sleep(0.01)
        assert pid_file.exists(), "worker did not start"
        worker_pid = int(pid_file.read_text())
        if shutdown == "signal":
            process.send_signal(signal.SIGTERM)
        elif shutdown != "lease":
            process.stdin.close()
        process.wait(timeout=15)
        expected = -signal.SIGKILL if shutdown == "kill" else None
        if expected is not None:
            # sys.exit(-SIGKILL) is represented as an unsigned Unix exit status.
            assert process.returncode == expected % 256
        with pytest.raises(ProcessLookupError):
            os.kill(worker_pid, 0)
    finally:
        if worker_pid is not None:
            try:
                os.killpg(worker_pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        if process.poll() is None:
            process.kill()
        process.wait(timeout=5)
        process.stdin.close()
        process.stderr.close()


def test_real_descendant_is_killed_after_leader_exits(tmp_path):
    """A stubborn descendant must not survive a successfully exited leader."""
    group_file = tmp_path / "group.pid"
    ready_file = tmp_path / "descendant.ready"
    descendant = tmp_path / "descendant.py"
    descendant.write_text(
        "import signal, time\n"
        "signal.signal(signal.SIGINT, signal.SIG_IGN)\n"
        "signal.signal(signal.SIGTERM, signal.SIG_IGN)\n"
        f"open({str(ready_file)!r}, 'w').close()\n"
        "while True: time.sleep(1)\n"
    )
    leader = tmp_path / "leader.py"
    leader.write_text(
        "import os, subprocess, time\n"
        "from pathlib import Path\n"
        f"open({str(group_file)!r}, 'w').write(str(os.getpid()))\n"
        f"subprocess.Popen({[sys.executable, str(descendant)]!r})\n"
        f"while not Path({str(ready_file)!r}).exists(): time.sleep(0.01)\n"
    )
    command = "exec " + shlex.join([sys.executable, str(leader)])
    process = subprocess.Popen(
        [sys.executable, supervisor.__file__, json.dumps({"command": command})],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    try:
        # Keep the lease open: shutdown must follow the leader's exit, not EOF.
        process.wait(timeout=8)
        assert ready_file.exists()
        assert process.returncode == 0
        # The descendant inherits both pipes. EOF proves it also exited, even on
        # Linux hosts whose init leaves an adopted child as a zombie briefly.
        process.communicate(timeout=3)
    finally:
        if group_file.exists():
            try:
                os.killpg(int(group_file.read_text()), signal.SIGKILL)
            except ProcessLookupError:
                pass
        if process.poll() is None:
            process.kill()
        process.communicate(timeout=5)
