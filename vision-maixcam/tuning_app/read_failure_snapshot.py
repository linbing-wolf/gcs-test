import getpass
from pathlib import Path
import paramiko

c = paramiko.SSHClient()
c.load_system_host_keys()
c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51', username='root', password=getpass.getpass('Device password: '), look_for_keys=False, allow_agent=False, timeout=20)
try:
    for cmd in ['uptime; cat /proc/sys/kernel/random/boot_id; ls -lt /maixapp/tmp | head -18', 'dmesg | tail -45', 'tail -8 /maixapp/tmp/last_run.log']:
        _, out, err = c.exec_command(cmd, timeout=20)
        print(out.read().decode(errors='replace'))
        print(err.read().decode(errors='replace'))
    dest = Path(__file__).resolve().parent.parent / '_from_device/soft_uart'
    with c.open_sftp() as s:
        files = sorted((f for f in s.listdir_attr('/root/yyb_mover_serial') if f.filename.startswith('serial_')), key=lambda f:f.st_mtime, reverse=True)[:4]
        for f in files:
            s.get('/root/yyb_mover_serial/'+f.filename, str(dest/f.filename))
            print('Saved', f.filename)
finally:
    c.close()
