import getpass
from pathlib import Path
import paramiko
import sys

c = paramiko.SSHClient()
c.load_system_host_keys()
c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect(sys.argv[1] if len(sys.argv)>1 else '192.168.3.51', username='root', password=getpass.getpass('Device password: '), look_for_keys=False, allow_agent=False, timeout=20, banner_timeout=20)
try:
    for cmd in ["date; ps aux | grep -E '[p]ython3 /maixapp/apps/yyb_mover/(main|soft_uart).py'; cat /proc/tty/driver/serial",
                "tail -150 /maixapp/tmp/last_run.log",
                "python3 -c \"from pathlib import Path; fs=sorted(Path('/root/yyb_mover_serial').glob('serial_*.log'),key=lambda p:p.stat().st_mtime,reverse=True)[:3]; [print(str(p)+'\\n'+p.read_text()[-6000:]) for p in fs]\""]:
        _, out, err = c.exec_command(cmd, timeout=20)
        print(out.read().decode(errors='replace'), flush=True)
        error = err.read().decode(errors='replace')
        if error:
            print(error, flush=True)
    dest = Path(__file__).resolve().parent.parent / '_from_device/soft_uart'
    dest.mkdir(parents=True, exist_ok=True)
    with c.open_sftp() as s:
        s.get('/maixapp/tmp/last_run.log', str(dest / 'last_run.log'))
finally:
    c.close()
