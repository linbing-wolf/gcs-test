import getpass
import hashlib
from pathlib import Path
import time
import zipfile
import paramiko

archive = Path(__file__).resolve().parent.parent / 'maixcam_app_v2/dist/maix-yyb_mover-v3.9.89.zip'
with zipfile.ZipFile(archive) as z:
    if z.testzip() is not None:
        raise RuntimeError('Corrupt local archive')
    payloads = {n: z.read(n) for n in z.namelist() if not n.endswith('/')}
for name, data in payloads.items():
    if not data:
        raise RuntimeError('Empty source: ' + name)
    if name.endswith('.py'):
        compile(data, name, 'exec')
config = payloads['config.py'].decode('utf-8')
for expected in ['UART_DEV = "/dev/ttyS4"', 'PIN_TX = "A21"', 'PIN_RX = "A22"']:
    if expected not in config:
        raise RuntimeError('Unexpected source config')

c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20,banner_timeout=20)
def run(cmd):
 _,o,e=c.exec_command(cmd,timeout=40);a=o.read().decode(errors='replace');b=e.read().decode(errors='replace')
 if o.channel.recv_exit_status():raise RuntimeError(b or a)
 print(a,flush=True)
try:
 stamp=time.strftime('%Y%m%d-%H%M%S')
 stage='/maixapp/apps/yyb_mover-repair-'+stamp
 old='/maixapp/apps/yyb_mover-broken-'+stamp
 run('mkdir -p '+stage+'/assets')
 with c.open_sftp() as s:
  for name,data in payloads.items():
   with s.open(stage+'/'+name,'wb') as f:f.write(data)
   s.chmod(stage+'/'+name,0o755 if name in ('run.sh','main.py') else 0o644)
   with s.open(stage+'/'+name,'rb') as f:
    if hashlib.sha256(f.read()).digest()!=hashlib.sha256(data).digest():raise RuntimeError('Mismatch: '+name)
 manifest={n:hashlib.sha256(d).hexdigest() for n,d in payloads.items()}
 import json
 verify="import os,pathlib,hashlib,json; p=pathlib.Path("+repr(stage)+"); m=json.loads("+repr(json.dumps(manifest))+"); [(os.fsync(f.fileno()),f.close()) for f in [open(p/n,'rb') for n in m]]; assert all(hashlib.sha256((p/n).read_bytes()).hexdigest()==h for n,h in m.items()); os.sync(); print('All files fsynced and verified')"
 import shlex
 run('python3 -c '+shlex.quote(verify))
 run("pkill -9 -f '[m]ain\\.py' || true; pkill -9 -f '/yyb_mover/[s]oft_uart.py' || true; pkill -9 -f '[l]auncher' || true")
 time.sleep(2)
 run('mv /maixapp/apps/yyb_mover '+old+' && mv '+stage+' /maixapp/apps/yyb_mover && sync')
 run("printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &")
 time.sleep(8)
 run("ps aux | grep '[p]ython3 /maixapp/apps/yyb_mover/main.py'; tail -25 /maixapp/tmp/last_run.log; sync")
 time.sleep(15)
 run("ps aux | grep '[p]ython3 /maixapp/apps/yyb_mover/main.py'; find /maixapp/apps/yyb_mover -type f -size 0; tail -8 /maixapp/tmp/last_run.log")
 print('Preserved broken app: '+old,flush=True)
finally:c.close()
