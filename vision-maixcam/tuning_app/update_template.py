"""Upload the real-pixel target template, preserving the previous asset."""
import getpass
import hashlib
from pathlib import Path
import time
import paramiko

client = paramiko.SSHClient()
client.load_system_host_keys()
client.set_missing_host_key_policy(paramiko.RejectPolicy())
client.connect('192.168.3.51', username='root', password=getpass.getpass('Device password: '),
               look_for_keys=False, allow_agent=False, timeout=8)
try:
    path = '/maixapp/apps/yyb_mover/assets/target2.png'
    local = Path(__file__).resolve().parent.parent / 'maixcam_app_v2/assets/target2.png'
    with client.open_sftp() as sftp:
        with sftp.open(path, 'rb') as source:
            old = source.read()
        backup = path + '.backup-' + time.strftime('%Y%m%d-%H%M%S')
        with sftp.open(backup, 'wb') as target:
            target.write(old)
        sftp.put(str(local), path + '.new')
        sftp.posix_rename(path + '.new', path)
        with sftp.open(path, 'rb') as source:
            if hashlib.sha256(source.read()).digest() != hashlib.sha256(local.read_bytes()).digest():
                raise RuntimeError('Template hash mismatch')
        print('Uploaded and verified. Backup: ' + backup, flush=True)
    _, stdout, _ = client.exec_command("ps aux | grep '[p]ython3 /maixapp/apps/yyb_mover/main.py'", timeout=10)
    running = stdout.read().decode()
    if running.strip():
        _, stdout, stderr = client.exec_command("pkill -9 -f '[m]ain\\.py'; pkill -9 -f '[l]auncher'; sleep 3; ulimit -c unlimited; printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &", timeout=20)
        stdout.read()
        time.sleep(6)
        _, stdout, stderr = client.exec_command('tail -18 /maixapp/tmp/last_run.log', timeout=10)
        print(stdout.read().decode(), flush=True)
    else:
        print('YYB app not active; template will load on its next launch.', flush=True)
finally:
    client.close()
