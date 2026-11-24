from contextlib import redirect_stdout
from io import BytesIO, StringIO
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from unittest.mock import MagicMock
import server


class CalibrationTests(unittest.TestCase):
    def test_current_defaults_and_round_trip(self):
        values = server.validate(server.defaults())
        self.assertEqual(values['ALIGN_DY_OFFSET_E'], 40)
        self.assertEqual(values['PLATE_TARGET_X'] - values['PLATE_DX_OFFSET'], 314)
        self.assertEqual(len(server.serialize(values).splitlines()), 10)

    def test_invalid_values(self):
        for value in (float('nan'), float('inf'), True, '2', 201, 1.5):
            with self.subTest(value=value):
                values = server.defaults()
                values['ALIGN_DX_OFFSET_D'] = value
                with self.assertRaises(ValueError):
                    server.validate(values)

    def test_ssh_host_injection_rejected_without_process(self):
        with patch('server.subprocess.run') as run:
            with self.assertRaises(ValueError):
                server.ssh('-oProxyCommand=whoami', 'cat /tmp/plate_bias')
            run.assert_not_called()

    def test_password_transport_uses_known_hosts_and_stdin(self):
        client = MagicMock()
        stdin, stdout, stderr = MagicMock(), MagicMock(), MagicMock()
        stdout.read.return_value = b'ok'
        stderr.read.return_value = b''
        stdout.channel.recv_exit_status.return_value = 0
        client.exec_command.return_value = (stdin, stdout, stderr)
        with patch('paramiko.SSHClient', return_value=client), patch('server.subprocess.run') as run:
            self.assertEqual(server.ssh('192.168.3.51', 'fixed', 'payload', password='secret'), 'ok')
            client.load_system_host_keys.assert_called_once()
            self.assertEqual(client.set_missing_host_key_policy.call_args.args[0].__class__.__name__, 'RejectPolicy')
            stdin.write.assert_called_once_with('payload')
            client.close.assert_called_once()
            run.assert_not_called()


class HCalibrationTests(unittest.TestCase):
    def test_handler_routes_h_requests(self):
        profile = server.h_defaults()
        handler = server.Handler.__new__(server.Handler)
        handler.allowed = MagicMock(return_value=True)
        handler.reply = MagicMock()
        handler.path = '/api/h/defaults'
        handler.do_GET()
        handler.reply.assert_called_once_with(200, profile)

        for route, endpoint in (('read', 'read_h'), ('apply', 'apply_h')):
            with self.subTest(route=route):
                result = {'values': profile, 'temporary': route == 'apply'}
                data = json.dumps({'host': 'maix', 'password': 'pwd', 'values': profile}).encode()
                handler.path = f'/api/h/{route}'
                handler.headers = {'Content-Type': 'application/json', 'Content-Length': str(len(data))}
                handler.rfile = BytesIO(data)
                handler.reply.reset_mock()
                with patch(f'server.{endpoint}', return_value=result) as operation:
                    handler.do_POST()
                args = ('maix', 'pwd') if route == 'read' else ('maix', profile, 'pwd')
                operation.assert_called_once_with(*args)
                handler.reply.assert_called_once_with(200, result)

    def test_default_values_and_strict_request_types(self):
        defaults = server.h_defaults()
        self.assertEqual(defaults['target_y'], 318)
        for key, value in (('mode', 1), ('invert_move', 'false'), ('settle_ms', 1.5),
                           ('max_moves', True), ('gain', float('nan')), ('target_y', 960)):
            with self.subTest(key=key, value=value), patch('server.ssh') as connect:
                with self.assertRaises(ValueError):
                    server.apply_h('maix', {**defaults, key: value})
                connect.assert_not_called()

    def test_apply_uses_fixed_command_and_verifies_response(self):
        values = {**server.h_defaults(), 'max_moves': 0}
        reply = {'values': values, 'temporary': True}
        with patch('server.ssh', return_value=json.dumps(reply)) as connect:
            self.assertEqual(server.apply_h('maix', values, 'pwd'), reply)
            host, command, content = connect.call_args.args
            self.assertEqual((host, command), ('maix', server.H_APPLY_COMMAND))
            self.assertEqual(json.loads(content), values)
            self.assertEqual(connect.call_args.kwargs, {'password': 'pwd'})
        reply['temporary'] = False
        with patch('server.ssh', return_value=json.dumps(reply)):
            with self.assertRaisesRegex(ValueError, '回读'):
                server.apply_h('maix', values)

    def test_board_script_defaults_partial_override_and_atomic_apply(self):
        board_config = (server.APP_ROOT / 'config.py').read_text(encoding='utf-8-sig')
        with TemporaryDirectory(dir=server.ROOT) as work:
            app = Path(work)
            profile = app / 'data' / 'h_material.json'
            script = server.H_BOARD_SCRIPT.replace("'/maixapp/apps/yyb_mover'", repr(str(app)))
            (app / 'config.py').write_text(board_config + '\nraise AssertionError("config ran")\n', encoding='utf-8')
            with self.assertRaisesRegex(SystemExit, '不支持 H 参数'):
                self.run_board(apply=False, script=script, profile=profile)
            (app / 'h_material_config.py').touch()

            baseline = self.run_board(apply=False, script=script, profile=profile)
            self.assertEqual(baseline, {'values': server.h_defaults(), 'temporary': False})
            profile.parent.mkdir()
            profile.write_text('{"mode": false}', encoding='utf-8')
            result = self.run_board(apply=False, script=script, profile=profile)
            self.assertEqual(result, {'values': {**baseline['values'], 'mode': False}, 'temporary': True})

            values = {**baseline['values'], 'target_y': 231.5}
            result = self.run_board(apply=True, script=script, profile=profile, values=values)
            self.assertEqual(result, {'values': values, 'temporary': True})
            self.assertEqual(json.loads(profile.read_text(encoding='utf-8')), values)
            self.assertEqual({path.name for path in profile.parent.iterdir()},
                             {'h_material.json', 'h_material.json.lock'})
            with self.assertRaises(SystemExit):
                self.run_board(apply=True, script=script, profile=profile, values={})
            self.assertEqual(json.loads(profile.read_text(encoding='utf-8')), values)

    @staticmethod
    def run_board(*, apply, script, profile, values=None):
        output = StringIO()
        stdin = StringIO(json.dumps(values))
        fcntl = SimpleNamespace(LOCK_EX=2, flock=MagicMock())
        with patch.dict(sys.modules, {'fcntl': fcntl}), patch.object(sys, 'path', sys.path.copy()), \
                patch.object(sys, 'argv', ['-c', 'apply' if apply else 'read']), \
                patch.object(sys, 'stdin', stdin), patch.object(server.h_material_config, 'PATH', str(profile)), \
                redirect_stdout(output):
            exec(compile(script, 'h_board_script', 'exec'), {})
        return json.loads(output.getvalue())


if __name__ == '__main__':
    unittest.main()
