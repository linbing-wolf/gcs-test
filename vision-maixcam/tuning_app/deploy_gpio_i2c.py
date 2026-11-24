import getpass
import hashlib
from pathlib import Path
import re
import shlex
import time
import paramiko

c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20)
def run(cmd):
 _,o,e=c.exec_command(cmd,timeout=35);a=o.read().decode(errors='replace');b=e.read().decode(errors='replace')
 if o.channel.recv_exit_status():raise RuntimeError(b or a)
 print(a,flush=True)
try:
 base='/maixapp/apps/yyb_mover/'
 backup='/root/yyb_mover-before-gpio-i2c-'+time.strftime('%Y%m%d-%H%M%S')+'.tar.gz'
 run('tar -czf '+backup+' -C /maixapp/apps yyb_mover; sync')
 with c.open_sftp() as s:
  def read(name):
   with s.open(base+name,'rb') as f:return f.read().decode('utf-8')
  config=read('config.py')
  for key,value in dict(UART_DEV='i2c-gpio-master:0x42',UART_FUNC_TX='GPIOA1',UART_FUNC_RX='GPIOA0',FRAME_TIMEOUT_MS=600,I2C_STATIC_INPUT_DIAG=False).items():
   config,n=re.subn(r'(?m)^'+key+r'\s*=[^\r\n]*',key+' = '+repr(value),config)
   if n==0 and key=='I2C_STATIC_INPUT_DIAG':
    config=config.rstrip()+'\n'+key+' = '+repr(value)+'\n'
    continue
   if n!=1:raise RuntimeError('Unexpected config '+key)
  yaml=read('app.yaml').replace('\r\r\n','\n').replace('\r\n','\n')
  if not any(v in yaml for v in ('version: 3.9.89-i2c-test','version: 3.9.89-i2c-gpio-test')):raise RuntimeError('Expected current I2C version')
  yaml=yaml.replace('version: 3.9.89-i2c-test','version: 3.9.89-i2c-gpio-test')
  if '  - gpio_i2c_master.py' not in yaml:yaml=yaml.rstrip()+'\n  - gpio_i2c_master.py\n'
  root=Path(__file__).parent
  files={'config.py':config.encode(),'app.yaml':yaml.encode(),
         'gpio_i2c_master.py':(root/'gpio_i2c_master.py').read_bytes(),
         'i2c_transport.py':(root/'i2c_transport.py').read_bytes()}
  for name,data in files.items():
   if name.endswith('.py'):compile(data,name,'exec')
   with s.open(base+name+'.new','wb') as f:f.write(data)
  run("pkill -9 -f '[m]ain\\.py' || true; pkill -9 -f '[l]auncher' || true")
  time.sleep(2)
  for name,data in files.items():
   s.posix_rename(base+name+'.new',base+name)
   with s.open(base+name,'rb') as f:
    if hashlib.sha256(f.read()).digest()!=hashlib.sha256(data).digest():raise RuntimeError('Mismatch '+name)
  code="import os; paths="+repr([base+n for n in files])+"; [(os.fsync(f.fileno()),f.close()) for f in [open(p,'rb') for p in paths]]; os.sync()"
  run('python3 -c '+shlex.quote(code))
 run("find /maixapp/apps/yyb_mover -type f -name '*.pyc' -delete; printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &")
 time.sleep(8)
 run("ps aux | grep '[p]ython3 /maixapp/apps/yyb_mover/main.py'; tail -38 /maixapp/tmp/last_run.log")
 time.sleep(6)
 run('tail -9 /maixapp/tmp/last_run.log')
 print('Backup: '+backup,flush=True)
finally:c.close()
