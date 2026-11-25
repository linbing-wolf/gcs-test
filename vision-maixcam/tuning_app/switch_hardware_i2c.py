import getpass
from pathlib import Path
import re
import shlex
import time
import paramiko

root=Path(__file__).parent
transport=(root/'i2c_transport.py').read_text(encoding='utf-8')
transport=transport.replace('from gpio_i2c_master import GPIOMaster','from hardware_i2c_master import HardwareMaster')
transport=transport.replace('self.bus = GPIOMaster()', 'self.bus = HardwareMaster()')
transport=transport.replace("self.static_diag = bool(getattr(config, 'I2C_STATIC_INPUT_DIAG', False))",'self.static_diag = False')
transport=transport.replace("backend='gpio-open-drain'", "backend='linux-hardware-i2c6'")
transport=transport.replace('max_clock_hz=10000, min_half_period_us=50','requested_hz=10000')
transport=transport.replace('clock may be slower under Linux; no register prefix; hardware I2C6 is bypassed','requested frequency not verified; hardware SCL may retain driver default; no register prefix')
transport=transport.replace('GPIO master A1/A0 max_clock=10000Hz','hardware master I2C6 A1/A0 requested_clock=10000Hz (unverified)')
transport=transport.replace('GPIO backend samples individual ACK bits; check error text for NACK stage','hardware errno cannot distinguish address NACK from data NACK')
compile(transport,'i2c_transport.py','exec')
c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20)
def run(cmd):
 _,o,e=c.exec_command(cmd,timeout=30);a=o.read().decode(errors='replace');b=e.read().decode(errors='replace')
 if o.channel.recv_exit_status():raise RuntimeError(b or a)
 print(a,flush=True)
try:
 base='/maixapp/apps/yyb_mover/'
 backup='/root/yyb_mover-before-hardware-i2c-'+time.strftime('%Y%m%d-%H%M%S')+'.tar.gz'
 run('tar -czf '+backup+' -C /maixapp/apps yyb_mover; sync')
 with c.open_sftp() as s:
  def read(n):
   with s.open(base+n,'rb') as f:return f.read().decode('utf-8')
  config=read('config.py')
  for key,value in dict(UART_DEV='i2c6-master:0x42',UART_FUNC_TX='I2C6_SCL',UART_FUNC_RX='I2C6_SDA',I2C_STATIC_INPUT_DIAG=False).items():
   config,n=re.subn(r'(?m)^'+key+r'\s*=[^\r\n]*',key+' = '+repr(value),config)
   if n!=1:raise RuntimeError('Unexpected config '+key)
  yaml=read('app.yaml')
  yaml=re.sub(r'(?m)^version:[^\r\n]*','version: 3.9.89-i2c-test',yaml)
  yaml=yaml.replace('  - gpio_i2c_master.py','  - hardware_i2c_master.py')
  files={'i2c_transport.py':transport.encode(),'hardware_i2c_master.py':(root/'hardware_i2c_master.py').read_bytes(),'config.py':config.encode(),'app.yaml':yaml.encode()}
  for n,b in files.items():
   if n.endswith('.py'):compile(b,n,'exec')
   with s.open(base+n+'.new','wb') as f:f.write(b)
  run("pkill -9 -f '[m]ain\\.py' || true; pkill -9 -f '[l]auncher' || true")
  time.sleep(2)
  for n,b in files.items():
   s.posix_rename(base+n+'.new',base+n)
   with s.open(base+n,'rb') as f:
    if f.read()!=b:raise RuntimeError('Readback mismatch '+n)
  code="import os; paths="+repr([base+n for n in files])+"; [(os.fsync(f.fileno()),f.close()) for f in [open(p,'rb') for p in paths]]; os.sync()"
  run('python3 -c '+shlex.quote(code))
 run("find /maixapp/apps/yyb_mover -type f -name '*.pyc' -delete; printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &")
 time.sleep(8)
 run("ps aux | grep '[p]ython3 /maixapp/apps/yyb_mover/main.py'; tail -35 /maixapp/tmp/last_run.log")
 print('Backup: '+backup,flush=True)
finally:c.close()
