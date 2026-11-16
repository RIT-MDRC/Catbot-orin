"""Check bridge configuration and SSH supervisor without ROS or hardware."""

import json
import shlex
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from catbot_sim import remote, remote_supervisor
from catbot_sim.config import BridgeConfig


class ConfigurationTests(unittest.TestCase):
    def test_bridge_configuration(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "pyproject.toml").touch()
            (root / "src/catbot_rollout").mkdir(parents=True)
            (root / "src/catbot_rollout/worker.py").touch()
            from catbot_policy import configuration

            mapping = Path(configuration.__file__).with_name("default_mapping.json")
            values = {"rollout_repository": str(root), "mapping_file": str(mapping)}
            read = lambda name, default: values.get(name, default)
            config = BridgeConfig.from_parameters(read)
            self.assertEqual(config.repository, root.resolve())
            self.assertEqual(len(config.mapping["actuator_names"]), 12)
            for key, value in [
                ("backend", "invalid"),
                ("worker_timeout", 0),
                ("visualization_rate", float("nan")),
            ]:
                values[key] = value
                with self.subTest(key=key), self.assertRaises(ValueError):
                    BridgeConfig.from_parameters(read)
                del values[key]

    def test_remote_rejects_unknown_argument_before_ssh(self):
        args = [
            "remote",
            "--user",
            "user",
            "--workspace",
            "/workspace",
            "--policy-worker-project",
            "/worker",
            "--policy-bundle",
            "/bundle",
            "--mapping-file",
            "/mapping",
            "--misspelled-option",
        ]
        with (
            patch.object(sys, "argv", args),
            patch.object(remote.subprocess, "Popen") as spawn,
        ):
            with self.assertRaises(SystemExit) as error:
                remote.main()
            self.assertEqual(error.exception.code, 2)
            spawn.assert_not_called()

    def test_remote_policy_command_preserves_shared_parameters(self):
        args = [
            "remote",
            "--role",
            "policy",
            "--user",
            "user",
            "--workspace",
            "/workspace",
            "--policy-worker-project",
            "/worker",
            "--policy-bundle",
            "/bundle with spaces",
            "--mapping-file",
            "/mapping",
        ]
        child = Mock(returncode=0)
        child.poll.return_value = 0
        with (
            patch.object(sys, "argv", args),
            patch.object(remote.signal, "signal"),
            patch.object(remote.subprocess, "Popen", return_value=child) as spawn,
        ):
            remote.main()
        ssh = spawn.call_args.args[0]
        shipped = shlex.split(ssh[-1])
        self.assertEqual(shipped[3], remote.REMOTE)
        command = json.loads(shipped[4])["command"]
        from catbot_policy.configuration import POLICY_PARAMETERS

        for name in POLICY_PARAMETERS:
            self.assertIn(name + ":=", command)
        self.assertIn("/bundle with spaces", command)

    def test_supervisor_disconnect_cleans_process_group(self):
        child = Mock(pid=123, returncode=0)
        child.poll.return_value = None
        with (
            patch.object(
                sys, "argv", ["supervisor", json.dumps({"command": "example"})]
            ),
            patch.object(
                remote_supervisor.subprocess, "Popen", return_value=child
            ) as spawn,
            patch.object(remote_supervisor.signal, "signal"),
            patch.object(
                remote_supervisor.select, "select", return_value=([sys.stdin], [], [])
            ),
            patch.object(sys.stdin, "fileno", return_value=0),
            patch.object(remote_supervisor.os, "read", return_value=b""),
            patch.object(remote_supervisor.os, "killpg") as kill,
        ):
            with self.assertRaises(SystemExit) as error:
                remote_supervisor.main()
            self.assertEqual(error.exception.code, 0)
            spawn.assert_called_once_with(
                ["bash", "-lc", "example"],
                start_new_session=True,
                stdin=remote_supervisor.subprocess.DEVNULL,
            )
            self.assertEqual(kill.call_count, 2)
            child.wait.assert_called_once_with(timeout=5)
