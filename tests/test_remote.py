"""Verify SSH command construction, lease renewal and local teardown."""

import json
import os
import shlex
import signal
import subprocess
import sys
from unittest.mock import Mock, call

import pytest

from catbot_sim import remote

REAL_POPEN = subprocess.Popen


@pytest.fixture
def ssh(monkeypatch):
    argv = [
        "remote",
        "--user",
        "test-user",
        "--host",
        "test-pi.local",
        "--environment",
        "/test environment/ros.bash",
        "--workspace",
        "/workspace with spaces",
        "--policy-worker-project",
        "/worker",
        "--policy-bundle",
        "/bundle",
        "--mapping-file",
        "/mapping",
    ]
    monkeypatch.setattr(sys, "argv", argv)
    child = Mock(returncode=0)
    child.poll.return_value = 0
    spawn = Mock(return_value=child)
    monkeypatch.setattr(remote.subprocess, "Popen", spawn)
    monkeypatch.setattr(remote.signal, "signal", Mock())
    return argv, child, spawn


@pytest.mark.parametrize("role", ["both", "foundation", "policy", "supervisor"])
def test_remote_roles_and_quoting(ssh, role):
    argv, child, spawn = ssh
    argv += ["--role", role]
    remote.main()
    command = spawn.call_args.args[0]
    assert command[:3] == ["ssh", "-T", "-o"]
    assert command[-2] == "test-user@test-pi.local"
    shipped = shlex.split(command[-1])
    assert shipped[3] == remote.REMOTE
    script = json.loads(shipped[4])["command"]
    assert script.startswith("source '/test environment/ros.bash' && ")
    assert "source '/workspace with spaces/install/setup.bash'" in script
    assert ("ros2 run foundation main" in script) == (role in ("both", "foundation"))
    assert ("catbot_policy.node" in script) == (role in ("both", "policy"))
    assert ("ros2 launch catbot_supervisor rpi.launch.py" in script) == (
        role == "supervisor"
    )
    assert ("wait -n" in script) == (role == "both")
    child.stdin.close.assert_called_once()


@pytest.mark.parametrize("missing", ["--policy-bundle", "--policy-worker-project"])
def test_legacy_roles_require_policy_arguments(ssh, missing):
    argv, _, spawn = ssh
    index = argv.index(missing)
    del argv[index : index + 2]
    with pytest.raises(SystemExit) as error:
        remote.main()
    assert error.value.code == 2
    spawn.assert_not_called()


@pytest.mark.parametrize("protocol_path", ["", "/test protocol/hardware.json"])
def test_supervisor_sources_cli_overlay_without_policy_artifacts(
    ssh, tmp_path, monkeypatch, protocol_path
):
    argv, _, spawn = ssh
    for option in ("--policy-bundle", "--policy-worker-project"):
        index = argv.index(option)
        del argv[index : index + 2]
    environment = tmp_path / "shared environment.bash"
    environment.write_text("export CATBOT_TEST_DDS=shared\n")
    overlay = tmp_path / "CLI overlay/setup.bash"
    overlay.parent.mkdir()
    overlay.write_text('test "$CATBOT_TEST_DDS" = shared\n')
    bin_directory = tmp_path / "bin"
    bin_directory.mkdir()
    probe = bin_directory / "ros2"
    probe.write_text('#!/bin/sh\nprintf \'%s\\n\' "$CATBOT_TEST_DDS" "$@"\n')
    probe.chmod(0o755)
    argv += [
        "--role",
        "supervisor",
        "--environment",
        str(environment),
        "--ros-overlay",
        str(overlay),
        "--network-interface",
        "test eth",
        "--rpi-launch-arguments",
        json.dumps(
            {
                "heartbeat_timeout_seconds": 0.85,
                "encoder_timeout_seconds": 0.45,
                "hardware_config_file": "",
                "protocol_config_file": protocol_path,
            }
        ),
    ]
    remote.main()
    shipped = shlex.split(spawn.call_args.args[0][-1])
    script = json.loads(shipped[4])["command"]
    monkeypatch.setenv("PATH", str(bin_directory) + ":" + os.environ["PATH"])
    with monkeypatch.context() as execution:
        execution.setattr(subprocess, "Popen", REAL_POPEN)
        result = subprocess.run(
            ["bash", "-c", script], capture_output=True, text=True, check=False
        )
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [
        "shared",
        "launch",
        "catbot_supervisor",
        "rpi.launch.py",
        "network_interface:=test eth",
        "mapping_file:=/mapping",
        "heartbeat_timeout_seconds:=0.85",
        "encoder_timeout_seconds:=0.45",
        *(["protocol_config_file:=" + protocol_path] if protocol_path else []),
    ]


@pytest.mark.parametrize("environment_exists", [True, False])
def test_policy_process_inherits_sourced_environment(
    ssh, tmp_path, monkeypatch, environment_exists
):
    argv, _, spawn = ssh
    environment = tmp_path / "shared environment.bash"
    if environment_exists:
        environment.write_text(
            "export ROS_DISCOVERY_SERVER='192.168.0.2:11811;192.168.0.3:11811'\n"
            "export ROS_SUPER_CLIENT=TRUE\n"
            "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n"
        )
    install = tmp_path / "install"
    install.mkdir()
    (install / "setup.bash").write_text(
        # Prove the shared exports are already present when the overlay is sourced.
        'export CATBOT_TEST_SOURCE_ORDER="$ROS_DISCOVERY_SERVER"\n'
    )
    probe = tmp_path / "policy probe"
    probe.write_text("#!/bin/sh\n/usr/bin/env\n")
    probe.chmod(0o755)
    argv += [
        "--role",
        "policy",
        "--environment",
        str(environment),
        "--workspace",
        str(tmp_path),
        "--policy-ros-python",
        str(probe),
    ]
    remote.main()
    shipped = shlex.split(spawn.call_args.args[0][-1])
    script = json.loads(shipped[4])["command"]
    # Bypass the mocked SSH transport but execute its real shell setup/exec chain.
    with monkeypatch.context() as execution:
        execution.setattr(subprocess, "Popen", REAL_POPEN)
        result = subprocess.run(
            ["bash", "-c", script], capture_output=True, text=True, check=False
        )
    if not environment_exists:
        assert result.returncode != 0
        assert not result.stdout  # A missing setup must prevent policy startup.
        return
    assert result.returncode == 0, result.stderr
    values = dict(
        line.split("=", 1) for line in result.stdout.splitlines() if "=" in line
    )
    assert values["ROS_DISCOVERY_SERVER"] == "192.168.0.2:11811;192.168.0.3:11811"
    assert values["ROS_SUPER_CLIENT"] == "TRUE"
    assert values["RMW_IMPLEMENTATION"] == "rmw_fastrtps_cpp"
    assert values["CATBOT_TEST_SOURCE_ORDER"] == values["ROS_DISCOVERY_SERVER"]


@pytest.mark.parametrize(
    "option,value",
    [("--host", "-bad"), ("--user", "bad user"), ("--host", "bad\nhost")],
)
def test_invalid_destination_never_spawns(ssh, option, value):
    argv, _, spawn = ssh
    argv += [option + "=" + value]
    with pytest.raises(ValueError, match="destination"):
        remote.main()
    spawn.assert_not_called()


@pytest.mark.parametrize(
    "option",
    ["--feedback-timeout", "--policy-worker-timeout", "--policy-startup-timeout"],
)
@pytest.mark.parametrize("value", ["0", "-1", "nan", "inf"])
def test_invalid_timeout_never_spawns(ssh, option, value):
    argv, _, spawn = ssh
    argv += [option + "=" + value]
    with pytest.raises(SystemExit) as error:
        remote.main()
    assert error.value.code == 2
    spawn.assert_not_called()


def test_unknown_option_never_spawns(ssh):
    argv, _, spawn = ssh
    argv.append("--misspelled-option")
    with pytest.raises(SystemExit) as error:
        remote.main()
    assert error.value.code == 2
    spawn.assert_not_called()


def test_policy_arguments_survive_shell_quoting(ssh, tmp_path, monkeypatch):
    argv, _, spawn = ssh
    environment = tmp_path / "ros environment.bash"
    environment.touch()
    install = tmp_path / "install"
    install.mkdir()
    (install / "setup.bash").touch()
    probe = tmp_path / "policy probe"
    probe.write_text("#!/bin/sh\nprintf '%s\\n' \"$@\"\n")
    probe.chmod(0o755)
    argv += [
        "--role",
        "policy",
        "--environment",
        str(environment),
        "--workspace",
        str(tmp_path),
        "--policy-ros-python",
        str(probe),
        "--policy-bundle",
        "/bundle with spaces/'quoted'",
        "--mapping-file",
        "/mapping with spaces.json",
        "--feedback-timeout",
        "0.75",
        "--policy-worker-timeout",
        "4.5",
        "--policy-device",
        "cpu",
    ]
    remote.main()
    shipped = shlex.split(spawn.call_args.args[0][-1])
    script = json.loads(shipped[4])["command"]
    with monkeypatch.context() as execution:
        execution.setattr(subprocess, "Popen", REAL_POPEN)
        result = subprocess.run(
            ["bash", "-c", script], capture_output=True, text=True, check=False
        )
    assert result.returncode == 0, result.stderr
    arguments = result.stdout.splitlines()
    assert arguments[:2] == ["-m", "catbot_policy.node"]
    parameters = dict(
        arguments[i + 1].split(":=", 1)
        for i, argument in enumerate(arguments)
        if argument == "-p"
    )
    assert json.loads(parameters["policy_bundle"]) == "/bundle with spaces/'quoted'"
    assert json.loads(parameters["mapping_file"]) == "/mapping with spaces.json"
    assert json.loads(parameters["feedback_timeout"]) == 0.75
    assert json.loads(parameters["policy_worker_timeout"]) == 4.5
    assert json.loads(parameters["policy_device"]) == "cpu"


def test_lease_is_written_and_flushed(ssh, monkeypatch):
    _, child, _ = ssh
    child.poll.side_effect = [None, 0]
    stop = Mock()
    stop.wait.return_value = False
    stop.is_set.return_value = False
    monkeypatch.setattr(remote.threading, "Event", Mock(return_value=stop))
    remote.main()
    assert child.stdin.method_calls == [
        call.write(b"lease\n"),
        call.flush(),
        call.close(),
    ]


@pytest.mark.parametrize("operation", ["write", "flush", "close"])
def test_broken_lease_pipe_still_waits(ssh, monkeypatch, operation):
    _, child, _ = ssh
    child.poll.side_effect = [None, 0]
    stop = Mock()
    stop.wait.return_value = False
    stop.is_set.return_value = False
    monkeypatch.setattr(remote.threading, "Event", Mock(return_value=stop))
    getattr(child.stdin, operation).side_effect = BrokenPipeError
    remote.main()
    child.wait.assert_called_once_with(timeout=12)


@pytest.mark.parametrize("timeouts", [1, 2])
def test_unresponsive_ssh_is_terminated_then_killed(ssh, timeouts):
    _, child, _ = ssh
    child.wait.side_effect = [subprocess.TimeoutExpired("ssh", 0)] * timeouts + [0]
    remote.main()
    child.terminate.assert_called_once()
    assert child.kill.call_count == timeouts - 1
    assert child.wait.call_args_list == [call(timeout=12), call(timeout=3)] + (
        [call()] if timeouts == 2 else []
    )


def test_missing_lease_pipe(ssh):
    _, child, _ = ssh
    child.stdin = None
    with pytest.raises(RuntimeError, match="lease pipe"):
        remote.main()


def test_ssh_failure_propagates(ssh):
    _, child, _ = ssh
    child.returncode = 23
    with pytest.raises(SystemExit) as error:
        remote.main()
    assert error.value.code == 23


def test_requested_stop_suppresses_ssh_signal_exit(ssh, monkeypatch):
    _, child, _ = ssh
    child.poll.return_value = None
    child.returncode = -signal.SIGTERM
    stop = Mock()
    stop.wait.return_value = True
    stop.is_set.return_value = True
    monkeypatch.setattr(remote.threading, "Event", Mock(return_value=stop))
    # Invoke the registered handler to ensure signals actually request a stop.
    monkeypatch.setattr(remote.signal, "signal", lambda sig, handler: handler())
    remote.main()
    assert stop.set.call_count == 2
    child.stdin.write.assert_not_called()


@pytest.mark.parametrize(
    "settings",
    [
        {"unknown": 1},
        {"heartbeat_timeout_seconds": 0},
        {"encoder_timeout_seconds": float("nan")},
        {"hardware_config_file": 3},
        [],
        {"heartbeat_timeout_seconds": True},
    ],
)
def test_invalid_pi_launch_arguments_never_start_ssh(ssh, settings):
    argv, _, spawn = ssh
    argv += ["--role", "supervisor", "--rpi-launch-arguments", json.dumps(settings)]
    with pytest.raises(SystemExit):
        remote.main()
    spawn.assert_not_called()
