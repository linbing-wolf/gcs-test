import getpass
import hashlib
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
 run("python3 -c \"from maix import pinmap,uart; assert 'UART2_TX' in pinmap.get_pin_functions('B0'); assert 'UART2_RX' in pinmap.get_pin_functions('B1'); assert '/dev/ttyS2' in uart.list_devices(); print('UART2 B0/B1 supported')\"")
 path='/maixapp/apps/yyb_mover/config.py'
 stamp=time.strftime('%Y%m%d-%H%M%S')
 with c.open_sftp() as s:
  with s.open(path,'rb') as f:old=f.read()
  text=old.decode('utf-8')
  for key,value in dict(UART_DEV='/dev/ttyS2',PIN_TX='B0',PIN_RX='B1',UART_FUNC_TX='UART2_TX',UART_FUNC_RX='UART2_RX').items():
   text,n=re.subn(r'(?m)^'+key+r'\s*=[^\r\n]*',key+' = '+repr(value),text)
   if n!=1:raise RuntimeError('Unexpected setting: '+key)
  compile(text,path,'exec');data=text.encode('utf-8')
  backup=path+'.uart4-backup-'+stamp
  with s.open(backup,'wb') as f:f.write(old)
  with s.open(path+'.new','wb') as f:f.write(data)
  s.posix_rename(path+'.new',path)
  with s.open(path,'rb') as f:
   if hashlib.sha256(f.read()).digest()!=hashlib.sha256(data).digest():raise RuntimeError('Readback mismatch')
 run('python3 -c '+shlex.quote("import os; p="+repr(path)+"; f=open(p,'rb'); os.fsync(f.fileno()); f.close(); os.sync()"))
 run("pkill -9 -f '[m]ain\\.py' || true; pkill -9 -f '[l]auncher' || true")
 time.sleep(2)
 run("find /maixapp/apps/yyb_mover -type f -name '*.pyc' -delete; printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &")
 time.sleep(8)
 run("ps aux | grep '[p]ython3 /maixapp/apps/yyb_mover/main.py'; tail -24 /maixapp/tmp/last_run.log; cat /proc/tty/driver/serial; sync")
 print('Backup: '+backup,flush=True)
finally:c.close()
