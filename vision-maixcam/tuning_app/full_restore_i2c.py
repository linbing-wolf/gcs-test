import getpass
from pathlib import Path
import shlex
import time
import paramiko

REMOTE_PREPARE = r'''
import ast, hashlib, os, pathlib, zipfile, time, json, re
source='/root/i2c-restore-base-389.zip'
stamp=time.strftime('%Y%m%d-%H%M%S')
stage=pathlib.Path('/maixapp/apps/yyb_mover-i2c-stage-'+stamp)
names=['app.yaml','main.py','config.py','protocol.py','vision.py','tasks.py',
'cam_light.py','disc_debug.py','serial_log.py','timing.py','touch_ui.py',
'screen_echo.py','precision.py','ai_model.py','run.sh','i2c_transport.py','gpio_i2c_master.py',
'assets/target2.png','assets/yyb_mover.png']
with zipfile.ZipFile(source) as archive:
 data={}
 overrides={'i2c_transport.py':'/root/i2c-restore-transport.py','gpio_i2c_master.py':'/root/i2c-restore-gpio.py'}
 for name in names:
  data[name]=pathlib.Path(overrides[name]).read_bytes() if name in overrides else archive.read(name)
  if not data[name]:raise RuntimeError('Empty '+name)
  if name.endswith('.py'):ast.parse(data[name],filename=name)
config=pathlib.Path('/maixapp/apps/yyb_mover/config.py').read_text()
for key,value in dict(UART_DEV='i2c-gpio-master:0x42',PIN_TX='A1',PIN_RX='A0',UART_FUNC_TX='GPIOA1',UART_FUNC_RX='GPIOA0',FRAME_TIMEOUT_MS=600).items():
 config,n=re.subn(r'(?m)^'+key+r'\s*=[^\r\n]*',key+' = '+repr(value),config)
 assert n==1,key
data['config.py']=config.encode()
protocol=data['protocol.py'].decode().replace('\r\r\n','\n').replace('\r\n','\n')
assert protocol.count('self.ser = uart.UART(dev, baud)')==2
protocol=protocol.replace('self.ser = uart.UART(dev, baud)','self.ser = I2CTransport(dev, baud)').replace('from maix import uart, pinmap','from maix import uart, pinmap\nfrom i2c_transport import I2CTransport')
data['protocol.py']=protocol.encode()
yaml=data['app.yaml'].decode().replace('\r\r\n','\n').replace('\r\n','\n')
yaml=re.sub(r'(?m)^version:.*$','version: 3.9.89-i2c-gpio-test',yaml).rstrip()+'\n  - i2c_transport.py\n  - gpio_i2c_master.py\n'
data['app.yaml']=yaml.encode()
assert 'version: 3.9.89-i2c-gpio-test' in data['app.yaml'].decode().splitlines()
assert 'i2c-gpio-master:0x42' in data['config.py'].decode()
assert 'from gpio_i2c_master import GPIOMaster' in data['i2c_transport.py'].decode()
assert 'self.ser = I2CTransport(dev, baud)' in data['protocol.py'].decode()
assert 'SoftUART' not in data['protocol.py'].decode()
assert 'uart_write_checked' not in data['protocol.py'].decode()
stage.mkdir()
for name,blob in data.items():
 p=stage/name;p.parent.mkdir(parents=True,exist_ok=True)
 if name.endswith('.py'):ast.parse(blob,filename=name)
 with p.open('wb') as f:f.write(blob);f.flush();os.fsync(f.fileno())
 p.chmod(0o755 if name in ('main.py','run.sh') else 0o644)
 assert hashlib.sha256(p.read_bytes()).digest()==hashlib.sha256(blob).digest()
os.sync()
print(json.dumps({'stage':str(stage),'files':len(data),'verified':'GPIO I2C master 0x42 files staged'},ensure_ascii=False))
'''

c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20)
def run(cmd):
 _,o,e=c.exec_command(cmd,timeout=40);a=o.read().decode(errors='replace');b=e.read().decode(errors='replace')
 if o.channel.recv_exit_status():raise RuntimeError(b or a)
 print(a,flush=True)
 return a
try:
 import json
 root=Path(__file__).resolve().parent
 with c.open_sftp() as s:
  s.put(str(root.parent/'maixcam_app_v2/dist/maix-yyb_mover-v3.9.89.zip'),'/root/i2c-restore-base-389.zip')
  s.put(str(root/'i2c_transport.py'),'/root/i2c-restore-transport.py')
  s.put(str(root/'gpio_i2c_master.py'),'/root/i2c-restore-gpio.py')
 result=json.loads(run('python3 -c '+shlex.quote(REMOTE_PREPARE)))
 stage=result['stage']
 if not stage.startswith('/maixapp/apps/yyb_mover-i2c-stage-'):raise RuntimeError('Unexpected stage')
 old='/maixapp/apps/yyb_mover-uart2-backup-'+time.strftime('%Y%m%d-%H%M%S')
 run("pkill -9 -f '[m]ain\\.py' || true; pkill -9 -f '/yyb_mover/[s]oft_uart.py' || true; pkill -9 -f '[l]auncher' || true")
 time.sleep(2)
 run('mv /maixapp/apps/yyb_mover '+old+' && mv '+stage+' /maixapp/apps/yyb_mover && sync')
 run("printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &")
 time.sleep(8)
 run("ps aux | grep -E '[p]ython3 /maixapp/apps/yyb_mover/(main|soft_uart).py'; tail -35 /maixapp/tmp/last_run.log; grep -E '^(UART_DEV|PIN_TX|PIN_RX|UART_FUNC_TX|UART_FUNC_RX) =' /maixapp/apps/yyb_mover/config.py; find /maixapp/apps/yyb_mover -type f -size 0; sync")
 time.sleep(5)
 run("tail -6 /maixapp/tmp/last_run.log; ps aux | grep '[p]ython3 /maixapp/apps/yyb_mover/main.py'")
 print('UART2 backup: '+old,flush=True)
finally:c.close()
