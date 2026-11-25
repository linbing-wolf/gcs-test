import getpass
from pathlib import Path
import time
import paramiko
c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20)
def run(cmd):
 _,o,e=c.exec_command(cmd,timeout=30);a=o.read().decode();b=e.read().decode()
 if o.channel.recv_exit_status():raise RuntimeError(b or a)
 print(a,flush=True)
try:
 path='/maixapp/apps/yyb_mover/i2c_transport.py'
 backup=path+'.before-update-'+time.strftime('%Y%m%d-%H%M%S')
 run('cp '+path+' '+backup)
 data=Path(__file__).with_name('i2c_transport.py').read_bytes();compile(data,path,'exec')
 with c.open_sftp() as s:
  with s.open(path+'.new','wb') as f:f.write(data)
  s.posix_rename(path+'.new',path)
  with s.open(path,'rb') as f:
   if f.read()!=data:raise RuntimeError('Readback mismatch')
 run("pkill -9 -f '[m]ain\\.py' || true; pkill -9 -f '[l]auncher' || true")
 time.sleep(2)
 run("find /maixapp/apps/yyb_mover -type f -name '*.pyc' -delete; printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &")
 time.sleep(8)
 run('tail -45 /maixapp/tmp/last_run.log')
 print('Backup: '+backup)
finally:c.close()
