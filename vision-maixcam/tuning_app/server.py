"""Loopback-only calibration UI. No dependencies; SSH uses existing keys."""
import argparse
import ast
import json
import math
from pathlib import Path
import re
import shlex
import socket
import subprocess
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from log_investigation import investigate
import webbrowser

ROOT = Path(__file__).resolve().parent
APP_ROOT = ROOT.parent / 'maixcam_app_v2'
sys.path.insert(0, str(APP_ROOT))
import h_material_config

KEYS = ('PLATE_TARGET_X', 'PLATE_TARGET_Y', 'PLATE_DX_OFFSET', 'PLATE_DY_OFFSET',
        'ALIGN_DX_OFFSET_D', 'ALIGN_DY_OFFSET_D', 'ALIGN_DX_OFFSET_E', 'ALIGN_DY_OFFSET_E',
        'STACK_ALIGN_DX_OFFSET', 'STACK_ALIGN_DY_OFFSET')


def defaults():
    tree = ast.parse((APP_ROOT / 'config.py').read_text(encoding='utf-8-sig'))
    values = {}
    for node in tree.body:
        if isinstance(node, ast.Assign):
            try:
                value = ast.literal_eval(node.value)
            except (ValueError, TypeError):
                continue
            for target in node.targets:
                if isinstance(target, ast.Name):
                    values[target.id] = value
    return {k: (values['ALIGN_DY_OFFSET'] if k == 'ALIGN_DY_OFFSET_E' and values[k] is None else values[k]) for k in KEYS}


def runtime_summary():
    """Return the current source baseline shown by the read-only dashboard."""
    namespace = {}
    source = (APP_ROOT / 'config.py').read_text(encoding='utf-8-sig')
    exec(compile(source, str(APP_ROOT / 'config.py'), 'exec'), namespace)
    configured_delays = namespace.get('DISC_TERMINAL_CAPTURE_DELAYS_MS')
    if configured_delays is None:
        configured_delays = (namespace['DISC_TERMINAL_CAPTURE_DELAY_MS'],)
    capture_delays_ms = sorted({
        max(0, int(value))
        for value in configured_delays
        if max(0, int(value)) > 0
    })
    width = int(namespace['W'])
    height = int(namespace['H'])
    return {
        'resolution': [width, height],
        'h_material_roi': [int(v) for v in namespace['H_MATERIAL_ROI']],
        'h_direction': {
            'enabled': bool(namespace['H_DIRECTION_PACKET_ENABLED']),
            'fallback': int(namespace['H_DIRECTION_FALLBACK']),
            'format': 'HCOLORS / HDISC',
        },
        'h_material': {
            'single_stop_mode': bool(namespace['H_MATERIAL_SINGLE_STOP_MODE']),
            'extended_packet': bool(namespace['H_MATERIAL_EXTENDED_PACKET_ENABLED']),
            'target_y': float(namespace['H_MATERIAL_TARGET_Y']),
            'center_frames': int(namespace['H_MATERIAL_CENTER_FRAMES']),
        },
        'disc': {
            'one_shot': bool(namespace['ALIGN_ONE_SHOT']),
            'return_after_move': True,
            'move_scale': float(namespace['MOVE_SCALE']),
            'movey_scale': float(namespace['MOVEY_SCALE']),
            'terminal_capture': {
                'enabled': bool(namespace['DISC_TERMINAL_CAPTURE_ENABLED']),
                'delay_ms': int(namespace['DISC_TERMINAL_CAPTURE_DELAY_MS']),
                'delays_ms': capture_delays_ms,
            },
        },
        'j': {
            'confirm_ms': int(namespace['MATERIAL_STATE_CONFIRM_MS']),
            'hold_ms': int(namespace['MATERIAL_STATE_HOLD_MS']),
            'tolerance_px': float(namespace['MATERIAL_STATE_TOL_PX']),
        },
    }


def validate(data):
    if set(data) != set(KEYS):
        raise ValueError('必须包含且仅包含十个白名单参数')
    result = {}
    for key in KEYS:
        value = data[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('参数必须为有限数字')
        limit = 640 if key.endswith('_X') else 480
        if 'OFFSET' in key:
            if value != int(value) or abs(value) > 200:
                raise ValueError('偏移必须为 -200 到 200 的整数')
        elif not 0 <= value <= limit:
            raise ValueError('目标坐标超出画面范围')
        result[key] = value
    x = result['PLATE_TARGET_X'] - result['PLATE_DX_OFFSET']
    y = result['PLATE_TARGET_Y'] - result['PLATE_DY_OFFSET']
    if not (0 <= x <= 640 and 0 <= y <= 480):
        raise ValueError('有效目标超出 640×480 画面')
    for xkey, ykey in (('ALIGN_DX_OFFSET_D', 'ALIGN_DY_OFFSET_D'), ('ALIGN_DX_OFFSET_E', 'ALIGN_DY_OFFSET_E'), ('STACK_ALIGN_DX_OFFSET', 'STACK_ALIGN_DY_OFFSET')):
        if not (0 <= 320-result[xkey] <= 640 and 0 <= 240-result[ykey] <= 480):
            raise ValueError('D/E/F 有效目标超出画面')
    return result


def serialize(data):
    return ''.join(f'{key}={data[key]:g}\n' for key in KEYS)


H_BOARD_SCRIPT = r'''
import fcntl
import json
import os
from pathlib import Path
import sys
import tempfile

app = Path('/maixapp/apps/yyb_mover')


def read_values(path, defaults):
    try:
        values = json.loads(path.read_text(encoding='utf-8-sig'))
    except FileNotFoundError:
        return {'values': defaults, 'temporary': False}
    if not isinstance(values, dict):
        raise ValueError('H 覆盖文件必须是 JSON 对象')
    return {'values': validate({**defaults, **values}), 'temporary': True}


try:
    if not (app / 'h_material_config.py').is_file():
        raise ValueError('板端版本不支持 H 参数配置，请更新视觉端应用')
    sys.path.insert(0, str(app))
    from h_material_config import PATH, defaults_from_source, validate

    path = Path(PATH)
    defaults = defaults_from_source((app / 'config.py').read_text(encoding='utf-8-sig'))
    if sys.argv[1] == 'apply':
        values = validate(json.load(sys.stdin))
        path.parent.mkdir(parents=True, exist_ok=True)
        with (path.parent / (path.name + '.lock')).open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            tmp = None
            try:
                with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                                                 dir=path.parent, delete=False) as stream:
                    tmp = Path(stream.name)
                    json.dump(values, stream, allow_nan=False)
                    stream.write('\n')
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(tmp, path)
                result = read_values(path, defaults)
                if not result['temporary'] or result['values'] != values:
                    raise ValueError('H 参数回读与提交值不一致')
            finally:
                if tmp is not None:
                    tmp.unlink(missing_ok=True)
    else:
        result = read_values(path, defaults)
    print(json.dumps(result, allow_nan=False))
except (ImportError, KeyError, OSError, SyntaxError, ValueError) as exc:
    sys.exit(str(exc))
'''

H_READ_COMMAND = 'python3 -B -c ' + shlex.quote(H_BOARD_SCRIPT) + ' read'
H_APPLY_COMMAND = 'python3 -B -c ' + shlex.quote(H_BOARD_SCRIPT) + ' apply'


def h_defaults():
    source = (APP_ROOT / 'config.py').read_text(encoding='utf-8-sig')
    return h_material_config.defaults_from_source(source)


def read_h(host, password=None):
    return _h_result(ssh(host, H_READ_COMMAND, password=password))


def apply_h(host, values, password=None):
    values = h_material_config.validate(values)
    result = _h_result(ssh(host, H_APPLY_COMMAND, json.dumps(values, allow_nan=False), password=password))
    if not result['temporary'] or result['values'] != values:
        raise ValueError('H 参数回读与提交值不一致')
    return result


def _h_result(raw):
    result = json.loads(raw)
    if (not isinstance(result, dict) or set(result) != {'values', 'temporary'}
            or not isinstance(result['temporary'], bool)):
        raise ValueError('板端 H 参数应答格式无效')
    return {'values': h_material_config.validate(result['values']), 'temporary': result['temporary']}


def ssh(host, command, content=None, password=None, host_key_alias=None):
    if not isinstance(host, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9.-]{0,252}', host):
        raise ValueError('主机仅支持 IPv4、域名或 SSH 别名')
    if host_key_alias is not None and (not isinstance(host_key_alias, str) or
            not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9.-]{0,252}', host_key_alias)):
        raise ValueError('SSH 主机密钥别名格式无效')
    if password:
        try:
            import paramiko
        except ImportError as exc:
            raise ValueError('当前 Python 环境没有 paramiko；请使用密钥连接，或另行授权安装') from exc
        client = paramiko.SSHClient()
        client.load_system_host_keys()
        client.set_missing_host_key_policy(paramiko.RejectPolicy())
        route = None
        try:
            if host_key_alias and host_key_alias != host:
                route = socket.create_connection((host, 22), timeout=12)
            client.connect(host_key_alias or host, sock=route, username='root', password=password, timeout=12,
                           auth_timeout=5, banner_timeout=5, look_for_keys=False, allow_agent=False)
            # Password authentication with paramiko uses the hostname/IP directly, not SSH aliases.
            stdin, stdout, stderr = client.exec_command(command, timeout=12)
            if content is not None:
                stdin.write(content)
                stdin.flush()
            stdin.channel.shutdown_write()
            result = stdout.read().decode('utf-8', errors='replace')
            error = stderr.read().decode('utf-8', errors='replace')
            if stdout.channel.recv_exit_status():
                raise ValueError(error.strip() or 'SSH 操作失败')
            return result
        except paramiko.SSHException as exc:
            raise ValueError('SSH 认证或主机指纹校验失败：' + str(exc)) from exc
        finally:
            client.close()
            if route is not None:
                route.close()
    args = ['ssh', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=12']
    if host_key_alias:
        args += ['-o', f'HostKeyAlias={host_key_alias}']
    proc = subprocess.run([*args, f'root@{host}', command],
                          input=content, encoding='utf-8', errors='replace', capture_output=True, timeout=20)
    if proc.returncode:
        raise ValueError(proc.stderr.strip() or 'SSH 操作失败')
    return proc.stdout


class Handler(BaseHTTPRequestHandler):
    def reply(self, status, value, content_type='application/json; charset=utf-8'):
        body = value if isinstance(value, bytes) else json.dumps(value, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def allowed(self):
        port = self.server.server_port
        hosts = (f'127.0.0.1:{port}', f'localhost:{port}')
        return (self.headers.get('Host') in hosts and
                self.headers.get('Origin') in (None, *(f'http://{host}' for host in hosts)) and
                self.headers.get('Sec-Fetch-Site') not in ('cross-site',))

    def do_GET(self):
        if not self.allowed():
            return self.reply(403, {'error': '仅允许本机同源访问'})
        if self.path == '/':
            return self.reply(200, (ROOT / 'index.html').read_bytes(), 'text/html; charset=utf-8')
        if self.path == '/api/defaults':
            return self.reply(200, defaults())
        if self.path == '/api/runtime':
            try:
                return self.reply(200, runtime_summary())
            except (KeyError, OSError, SyntaxError, TypeError, ValueError) as exc:
                return self.reply(500, {'error': str(exc)})
        if self.path == '/api/h/defaults':
            try:
                return self.reply(200, h_defaults())
            except (OSError, SyntaxError, ValueError) as exc:
                return self.reply(500, {'error': str(exc)})
        self.reply(404, {'error': '路径不存在'})

    def do_POST(self):
        if not self.allowed() or self.headers.get('Content-Type') != 'application/json':
            return self.reply(403, {'error': '仅允许本机同源 JSON 请求'})
        try:
            length = int(self.headers.get('Content-Length', 0))
            if not 0 < length < 8192:
                raise ValueError('请求长度无效')
            data = json.loads(self.rfile.read(length))
            if not isinstance(data, dict):
                raise ValueError('请求必须为 JSON 对象')
            host = data.get('host', '')
            password = data.get('password') or None
            if password is not None and (not isinstance(password, str) or len(password) > 1024):
                raise ValueError('密码格式无效')
            if self.path == '/api/log-investigation':
                alias = data.get('host_key_alias') or None
                self.reply(200, investigate(ssh, host, password, data.get('question', ''), alias))
            elif self.path == '/api/h/read':
                self.reply(200, read_h(host, password))
            elif self.path == '/api/h/apply':
                self.reply(200, apply_h(host, data['values'], password))
            elif self.path == '/api/read':
                raw = ssh(host, "if test -f /tmp/plate_bias; then cat /tmp/plate_bias; fi", password=password)
                values = defaults()
                for line in raw.splitlines():
                    key, sep, value = line.partition('=')
                    if sep and key.strip() in KEYS:
                        values[key.strip()] = float(value.split('#')[0].strip())
                self.reply(200, {'values': validate(values), 'temporary': bool(raw.strip())})
            elif self.path == '/api/apply':
                values = validate(data['values'])
                # Fixed commands and path; user-controlled numeric data travels through stdin.
                ssh(host, 'set -e; umask 077; f=$(mktemp /tmp/plate_bias.XXXXXX); trap \'rm -f "$f"\' EXIT; cat > "$f"; mv "$f" /tmp/plate_bias', serialize(values), password=password)
                self.reply(200, {'ok': True})
            elif self.path == '/api/reset':
                ssh(host, 'rm -f /tmp/plate_bias', password=password)
                self.reply(200, {'ok': True, 'values': defaults()})
            else:
                self.reply(404, {'error': '路径不存在'})
        except (ValueError, KeyError, TypeError, OSError, subprocess.TimeoutExpired) as exc:
            self.reply(400, {'error': str(exc)})


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--open', action='store_true', help='启动后在默认浏览器打开界面')
    args = parser.parse_args()
    with ThreadingHTTPServer(('127.0.0.1', args.port), Handler) as server:
        url = f'http://127.0.0.1:{server.server_port}'
        print(f'打开 {url} ，Ctrl+C 退出。不会自动连接设备。', flush=True)
        if args.open:
            webbrowser.open(url)
        server.serve_forever()
