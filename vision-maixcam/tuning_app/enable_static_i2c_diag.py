import getpass
from pathlib import Path
import re
import shlex
import time
import paramiko

c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20)
def run(cmd):
 _,o,e=c.exec_command(cmd,timeout=30);a=o.read().decode(errors='replace');b=e.read().decode(errors='replace')
 if o.channel.recv_exit_status():raise RuntimeError(b or a)
 print(a,flush=True)
try:
 base='/maixapp/apps/yyb_mover/'
 backup='/root/yyb_mover-before-static-diag-'+time.strftime('%Y%m%d-%H%M%S')+'.tar.gz'
 run('tar -czf '+backup+' -C /maixapp/apps yyb_mover; sync')
 with c.open_sftp() as s:
  with s.open(base+'config.py','rb') as f:config=f.read().decode('utf-8')
  config=re.sub(r'(?m)^I2C_STATIC_INPUT_DIAG\s*=[^\r\n]*','I2C_STATIC_INPUT_DIAG = True',config) if re.search(r'(?m)^I2C_STATIC_INPUT_DIAG\s*=',config) else config+'\nI2C_STATIC_INPUT_DIAG = True\n'
  files={'config.py':config.encode(),'i2c_transport.py':Path(__file__).with_name('i2c_transport.py').read_bytes()}
  for name,data in files.items():
   compile(data,name,'exec')
   with s.open(base+name+'.new','wb') as f:f.write(data)
  run("pkill -9 -f '[m]ain\\.py' || true; pkill -9 -f '[l]auncher' || true")
  time.sleep(2)
  for name,data in files.items():
   s.posix_rename(base+name+'.new',base+name)
   with s.open(base+name,'rb') as f:
    if f.read()!=data:raise RuntimeError('Readback mismatch '+name)
  code="import os; paths="+repr([base+n for n in files])+"; [(os.fsync(f.fileno()),f.close()) for f in [open(p,'rb') for p in paths]]; os.sync()"
  run('python3 -c '+shlex.quote(code))
 run("find /maixapp/apps/yyb_mover -type f -name '*.pyc' -delete; printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &")
 time.sleep(10)
 run("tail -15 /maixapp/tmp/last_run.log; cat /sys/kernel/debug/gpio")
 print('Backup: '+backup,flush=True)
finally:c.close()
