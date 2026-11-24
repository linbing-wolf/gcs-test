"""Build and deploy the runtime bias extension; password is prompted, never stored."""
import getpass
import hashlib
import sys
from pathlib import Path
import time
import zipfile
import paramiko

ROOT = Path(__file__).resolve().parent.parent / 'maixcam_app_v2'


def run(client, command):
    _, stdout, stderr = client.exec_command(command, timeout=45)
    output = stdout.read().decode(errors='replace')
    error = stderr.read().decode(errors='replace')
    code = stdout.channel.recv_exit_status()
    if code:
        raise RuntimeError(f'Command failed ({code}): {error}\n{output}')
    print(output, flush=True)
    return output


def main():
    password = getpass.getpass('Device SSH password: ')
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    client.connect('192.168.3.51', username='root', password=password,
                   look_for_keys=False, allow_agent=False, timeout=8,
                   auth_timeout=8, banner_timeout=8)
    password = None
    try:
        if '--verify' in sys.argv or '--logs' in sys.argv:
            with client.open_sftp() as sftp:
                for name in ('config.py', 'tasks.py', 'app.yaml'):
                    with sftp.open('/maixapp/apps/yyb_mover/' + name, 'rb') as remote:
                        digest = hashlib.sha256(remote.read()).hexdigest()
                    if digest != hashlib.sha256((ROOT / name).read_bytes()).hexdigest():
                        raise RuntimeError('Device file differs: ' + name)
                    print(name + ': SHA256 matches', flush=True)
            if '--logs' in sys.argv:
                run(client, "echo '=== process ==='; ps aux | grep -E '[m]ain.py|[l]auncher'; echo '=== stop markers ==='; grep -Ein 'segmentation|traceback|fatal|MemoryError|Frame Buffer|killed|oom|exception|error|exit|stop|停止|超时|急停|reset|R 中断' /maixapp/tmp/last_run.log | tail -100; echo '=== session events ==='; grep -E '\"event\": \"(exit|crash|start)\"' /root/yyb_mover_serial/sessions.jsonl | tail -15; echo '=== kernel crash ==='; dmesg | grep -Ei 'segfault|oom|killed process|python3' | tail -30; echo '=== bias ==='; cat /tmp/plate_bias 2>/dev/null || true")
            else:
                run(client, "tail -8 /maixapp/tmp/last_run.log; cat /tmp/plate_bias 2>/dev/null || test ! -f /tmp/plate_bias")
            return
        run(client, "cat /maixapp/apps/yyb_mover/app.yaml; ps aux | grep '[m]ain.py'")
        archive = ROOT / 'dist/maix-yyb_mover-v3.9.88.zip'
        entries = []
        in_files = False
        for line in (ROOT / 'app.yaml').read_text(encoding='utf-8').splitlines():
            if line == 'files:':
                in_files = True
            elif in_files and line.strip().startswith('- '):
                entries.append(line.strip()[2:])
        archive.parent.mkdir(exist_ok=True)
        with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as bundle:
            for entry in entries:
                path = ROOT / entry
                for item in sorted(path.rglob('*')) if path.is_dir() else [path]:
                    if item.is_file():
                        bundle.write(item, item.relative_to(ROOT).as_posix())
        with zipfile.ZipFile(archive) as bundle:
            if bundle.testzip() is not None:
                raise RuntimeError('Invalid archive')
        stamp = time.strftime('%Y%m%d-%H%M%S')
        backup = '/root/yyb_mover-backup-' + stamp + '.tar.gz'
        run(client, f'tar -czf {backup} -C /maixapp/apps yyb_mover')
        print('Backup: ' + backup, flush=True)
        with client.open_sftp() as sftp:
            sftp.put(str(archive), '/root/' + archive.name)
        run(client, 'app_store_cli install /root/' + archive.name)
        run(client, "pkill -9 -f '[m]ain\\.py' || test $? = 1; pkill -9 -f '[l]auncher' || test $? = 1")
        time.sleep(3)
        run(client, "find /maixapp/apps/yyb_mover -type f -name '*.pyc' -delete; ulimit -c unlimited; printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &")
        time.sleep(8)
        run(client, "cat /maixapp/apps/yyb_mover/app.yaml; ps aux | grep '[m]ain.py'; tail -60 /maixapp/tmp/last_run.log")
    finally:
        client.close()


if __name__ == '__main__':
    main()
