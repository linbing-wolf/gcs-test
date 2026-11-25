"""Change only H's inside ratio on the device and reload the active app."""
import getpass
import hashlib
import re
import time
import paramiko

client = paramiko.SSHClient()
client.load_system_host_keys()
client.set_missing_host_key_policy(paramiko.RejectPolicy())
client.connect('192.168.3.51', username='root', password=getpass.getpass('Device password: '),
               look_for_keys=False, allow_agent=False, timeout=8)


def run(command):
    _, out, err = client.exec_command(command, timeout=25)
    result = out.read().decode(errors='replace')
    error = err.read().decode(errors='replace')
    if out.channel.recv_exit_status() != 0:
        raise RuntimeError(error or result)
    return result


try:
    path = '/maixapp/apps/yyb_mover/config.py'
    with client.open_sftp() as sftp:
        with sftp.open(path, 'rb') as source:
            old = source.read()
        text, count = re.subn(r'(?m)^PLATE_INSIDE_MIN_RATIO\s*=\s*[0-9.]+[^\r\n]*',
                             'PLATE_INSIDE_MIN_RATIO = 0.50   # New-field trial: 50 percent',
                             old.decode('utf-8'))
        if count != 1:
            raise RuntimeError('Expected exactly one H ratio setting')
        compile(text, path, 'exec')
        backup = path + '.backup-' + time.strftime('%Y%m%d-%H%M%S')
        with sftp.open(backup, 'wb') as target:
            target.write(old)
        data = text.encode('utf-8')
        with sftp.open(path + '.new', 'wb') as target:
            target.write(data)
        sftp.posix_rename(path + '.new', path)
        with sftp.open(path, 'rb') as source:
            if hashlib.sha256(source.read()).digest() != hashlib.sha256(data).digest():
                raise RuntimeError('Device config verification failed')
        print('Updated ratio=0.50. Backup: ' + backup, flush=True)
    active = run("ps aux | grep '[p]ython3 /maixapp/apps/yyb_mover/main.py' || true")
    if active.strip():
        run("pkill -9 -f '[m]ain\\.py' || true; pkill -9 -f '[l]auncher' || true")
        time.sleep(3)
        run("ulimit -c unlimited; printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &")
        time.sleep(7)
        print(run("ps aux | grep '[p]ython3 /maixapp/apps/yyb_mover/main.py'; tail -16 /maixapp/tmp/last_run.log"), flush=True)
    else:
        print('YYB not active; ratio takes effect on next YYB launch.', flush=True)
finally:
    client.close()
