import getpass
from pathlib import Path
import shlex
import time
import paramiko

c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20)
def run(cmd):
 _,o,e=c.exec_command(cmd,timeout=30);a=o.read().decode(errors='replace');b=e.read().decode(errors='replace')
 if o.channel.recv_exit_status():raise RuntimeError(b or a)
 print(a,flush=True)
 return a
def start():
 run("find /maixapp/apps/yyb_mover -type f -name '*.pyc' -delete; printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &")
 time.sleep(8)
try:
 path='/maixapp/apps/yyb_mover/gpio_i2c_master.py'
 data=Path(__file__).with_name('gpio_i2c_master.py').read_bytes();compile(data,path,'exec')
 backup=path+'.before-pullup-'+time.strftime('%Y%m%d-%H%M%S')
 run('cp '+path+' '+backup+'; sync')
 with c.open_sftp() as s:
  with s.open(path+'.new','wb') as f:f.write(data)
  run("pkill -9 -f '[m]ain\\.py' || true; pkill -9 -f '[l]auncher' || true")
  time.sleep(2)
  s.posix_rename(path+'.new',path)
  with s.open(path,'rb') as f:
   if f.read()!=data:raise RuntimeError('Readback mismatch')
 code="import os; f=open("+repr(path)+",'rb');os.fsync(f.fileno());f.close();os.sync()"
 run('python3 -c '+shlex.quote(code))
 start()
 log=run('tail -45 /maixapp/tmp/last_run.log')
 if '[I2C-GPIO] idle readback' not in log:
  run("pkill -9 -f '[m]ain\\.py' || true; pkill -9 -f '[l]auncher' || true")
  run('cp '+backup+' '+path+'; sync')
  start()
  raise RuntimeError('Pull-up startup not confirmed; previous backend restored')
 run("cat /sys/kernel/debug/gpio; ps aux | grep '[p]ython3 /maixapp/apps/yyb_mover/main.py'")
 print('Backup: '+backup,flush=True)
finally:c.close()
