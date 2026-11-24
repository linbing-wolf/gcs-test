import subprocess
import shutil
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import log_investigation as logs


class LogInvestigationTests(unittest.TestCase):
    def test_log_command_starts_with_valid_shell_command(self):
        self.assertEqual(logs.LOG_COMMAND.splitlines()[0],
                         r"printf '%s\n' '--- recent application log ---'")
        shell = shutil.which("sh")
        if shell:
            result = subprocess.run([shell, "-c", logs.LOG_COMMAND],
                                    capture_output=True, text=True, check=True)
            self.assertIn("--- recent application log ---", result.stdout)
            self.assertIn("--- recent serial session ---", result.stdout)
            self.assertIn("--- latest H diagnostic ---", result.stdout)

    def test_redacts_password_and_limits_peer_input(self):
        ssh = Mock(return_value="\n".join(
            ["old diagnostic " + ("x" * 500)] * 128
            + ["RX H from 100.118.92.10", "password=secret-value", "TX NONE,H"]
        ))
        with patch.object(logs, "PEER", Mock(is_file=lambda: True)), \
                patch.object(logs.subprocess, "run", return_value=SimpleNamespace(
                    returncode=0, stdout="看最后一条 TX"
                )) as run:
            result = logs.investigate(ssh, "board", "secret-value", "为什么返回 NONE,H")
        ssh.assert_called_once_with("board", logs.LOG_COMMAND, password="secret-value")
        prompt = run.call_args.kwargs["input"]
        self.assertIn("TX NONE,H", result["excerpt"])
        self.assertIn("看最后一条 TX", result["analysis"])
        self.assertNotIn("secret-value", prompt)
        self.assertNotIn("100.118.92.10", prompt)
        self.assertLess(len(prompt), 15000)

    def test_timeout_releases_investigation_lock(self):
        with patch.object(logs, "PEER", Mock(is_file=lambda: True)), \
                patch.object(logs.subprocess, "run",
                             side_effect=subprocess.TimeoutExpired(["fake"], 75)):
            result = logs.investigate(lambda *args, **kwargs: "RX H", "board", None, "")
        self.assertIn("超过 75 秒", result["warning"])
        self.assertEqual(result["analysis"], "")
        self.assertIn("RX H", result["excerpt"])
        self.assertFalse(logs.BUSY.locked())

    def test_alias_and_failed_model_keep_excerpt(self):
        ssh = Mock(return_value="RX D\nTX DISC,309,217,-4,0")
        reply = SimpleNamespace(returncode=1, stdout="", stderr="provider HTTP 429")
        with patch.object(logs, "PEER", Mock(is_file=lambda: True)), \
                patch.object(logs.subprocess, "run", return_value=reply):
            result = logs.investigate(ssh, "tailscale-board", None, "D 为什么回 0", "trusted-lan")
        ssh.assert_called_once_with("tailscale-board", logs.LOG_COMMAND, password=None,
                                    host_key_alias="trusted-lan")
        self.assertIn("限流", result["warning"])
        self.assertIn("DISC,309,217,-4,0", result["excerpt"])

    def test_missing_logs_do_not_call_opencode(self):
        with patch.object(logs, "PEER", Mock(is_file=lambda: True)), \
                patch.object(logs.subprocess, "run") as run:
            with self.assertRaisesRegex(ValueError, "没有可读"):
                logs.investigate(lambda *args, **kwargs: "--- log ---\n", "board", None, "")
        run.assert_not_called()
