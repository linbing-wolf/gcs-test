import getpass
from pathlib import Path
import time
import paramiko
c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20)
def run(cmd):
    _,o,e=c.exec_command(cmd,timeout=30)
    a=o.read().decode(errors='replace');b=e.read().decode(errors='replace')
    print(a,b,flush=True)
    if o.channel.recv_exit_status():raise RuntimeError('Remote command failed')
try:
    dest=Path(__file__).resolve().parent.parent/'_from_device/disc_template_20261030/failure'
    dest.mkdir(exist_ok=True)
    with c.open_sftp() as s:
        parent='/root/yyb_mover_data/disc_debug'
        folders=sorted(s.listdir(parent))
        for folder in folders[-2:]:
            local=dest/folder;local.mkdir(exist_ok=True)
            for f in s.listdir_attr(parent+'/'+folder):
                if f.st_size<3000000:s.get(parent+'/'+folder+'/'+f.filename,str(local/f.filename))
        s.get('/maixapp/apps/yyb_mover/config.py',str(dest/'device_config.py'))
    run("pkill -9 -f '[m]ain\\.py' || true; pkill -9 -f '[l]auncher' || true")
    time.sleep(2)
    run("printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &")
    time.sleep(8)
    run("ps aux | grep '[p]ython3 /maixapp/apps/yyb_mover/main.py'; tail -14 /maixapp/tmp/last_run.log")
finally:c.close()
