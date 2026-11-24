"""Deploy capture settings or H improvements with backups and readback checks."""
import argparse
import ast
import getpass
import hashlib
from pathlib import Path
import re
import time
import paramiko

ROOT = Path(__file__).resolve().parent.parent / 'maixcam_app_v2'
DEBUG = ('PLATE_DEBUG_SAVE', 'PLATE_DEBUG_MAX_FRAMES', 'PLATE_DEBUG_QUALITY',
         'PLATE_DEBUG_KEEP_TASKS', 'PLATE_DEBUG_MIN_INTERVAL_MS')


def settings(text):
    result = {}
    for node in ast.parse(text).body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            try:
                result[node.targets[0].id] = ast.literal_eval(node.value)
            except (ValueError, TypeError):
                continue
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--capture-only', action='store_true')
    parser.add_argument('--verify', action='store_true')
    args = parser.parse_args()
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    client.connect('192.168.3.51', username='root', password=getpass.getpass('Device password: '),
                   look_for_keys=False, allow_agent=False, timeout=8)

    def run(command):
        _, out, err = client.exec_command(command, timeout=45)
        result, error = out.read().decode(errors='replace'), err.read().decode(errors='replace')
        if out.channel.recv_exit_status():
            raise RuntimeError(error or result)
        return result

    try:
        if args.verify:
            dest = ROOT.parent / '_from_device/h_latest/device_source'
            dest.mkdir(exist_ok=True)
            with client.open_sftp() as sftp:
                for name in ('config.py', 'vision.py', 'tasks.py', 'app.yaml'):
                    sftp.get('/maixapp/apps/yyb_mover/' + name, str(dest / name))
                    same = (dest / name).read_bytes() == (ROOT / name).read_bytes()
                    print(name + ': ' + ('identical' if same else 'differs; snapshot saved'), flush=True)
            return
        stamp = time.strftime('%Y%m%d-%H%M%S')
        backup = '/root/yyb_mover-before-h-' + stamp + '.tar.gz'
        run('tar -czf ' + backup + ' -C /maixapp/apps yyb_mover')
        print('Backup: ' + backup, flush=True)
        with client.open_sftp() as sftp:
            base = '/maixapp/apps/yyb_mover/'
            with sftp.open(base + 'config.py', 'rb') as source:
                config_text = source.read().decode('utf-8')
            local_config = (ROOT / 'config.py').read_text(encoding='utf-8-sig')
            local_values = settings(local_config)
            keys = list(DEBUG)
            if not args.capture_only:
                keys += ['PLATE_INSIDE_MIN_RATIO', 'PLATE_ARC_SKIP_ALT_ON']
                keys += [k for k in local_values if k.startswith(('PLATE_OCC_', 'PLATE_EDGE_', 'PLATE_SEGMENT_', 'PLATE_RIM_'))]
                keys += [k for k in ('PLATE_ARC_COVERAGE_CHECK', 'PLATE_ARC_MIN_SPAN_DEG') if k in local_values]
            for key in dict.fromkeys(keys):
                assignment = key + ' = ' + repr(local_values[key])
                pattern = r'(?m)^' + re.escape(key) + r'\s*=[^\r\n]*'
                if re.search(pattern, config_text):
                    config_text = re.sub(pattern, assignment, config_text)
                else:
                    config_text += '\n' + assignment + '\n'
            payloads = {'config.py': config_text.encode('utf-8')}
            if not args.capture_only:
                for name in ('vision.py', 'tasks.py', 'app.yaml'):
                    payloads[name] = (ROOT / name).read_bytes()
                version_match = re.search(rb'(?m)^version:\s*([^\r\n]+)', payloads['app.yaml'])
                if version_match is None:
                    raise RuntimeError('Missing release version')
                version = version_match.group(1).decode('ascii').strip()
                archive = ROOT / 'dist' / ('maix-yyb_mover-v' + version + '.zip')
                if archive.exists():
                    remote_zip = '/root/' + archive.name
                    sftp.put(str(archive), remote_zip)
                    with sftp.open(remote_zip, 'rb') as source:
                        if hashlib.sha256(source.read()).digest() != hashlib.sha256(archive.read_bytes()).digest():
                            raise RuntimeError('Release archive readback mismatch')
                    print('Verified: ' + remote_zip, flush=True)
            for name, data in payloads.items():
                if name.endswith('.py'):
                    compile(data, name, 'exec')
                with sftp.open(base + name + '.new', 'wb') as target:
                    target.write(data)
            for name, data in payloads.items():
                sftp.posix_rename(base + name + '.new', base + name)
                with sftp.open(base + name, 'rb') as source:
                    if hashlib.sha256(source.read()).digest() != hashlib.sha256(data).digest():
                        raise RuntimeError('Readback mismatch: ' + name)
                print('Verified: ' + name, flush=True)
        active = run("ps aux | grep '[p]ython3 /maixapp/apps/yyb_mover/main.py' || true")
        if active.strip():
            run("pkill -9 -f '[m]ain\\.py' || true; pkill -9 -f '[l]auncher' || true")
            time.sleep(3)
            run("find /maixapp/apps/yyb_mover -type f -name '*.pyc' -delete; ulimit -c unlimited; printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &")
            time.sleep(7)
            print(run("ps aux | grep '[p]ython3 /maixapp/apps/yyb_mover/main.py'; tail -17 /maixapp/tmp/last_run.log"), flush=True)
        else:
            print('YYB inactive; changes load on next YYB launch.', flush=True)
    finally:
        client.close()


if __name__ == '__main__':
    main()
