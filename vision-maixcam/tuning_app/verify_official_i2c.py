import getpass
import shlex
import time
import paramiko

PROBE = r'''
from maix import i2c, pinmap
import os,fcntl,json,time
for pin,func in [('A1','I2C6_SCL'),('A0','I2C6_SDA')]:
 pinmap.set_pin_function(pin,func)
 assert pinmap.get_pin_function(pin)==func
 print(json.dumps({'pin':pin,'function':func}),flush=True)
bus=i2c.I2C(6,i2c.Mode.MASTER)
print('Official default constructor; freq argument does not prove runtime clock',flush=True)
for index in range(3):
 started=time.monotonic()
 try:result=bus.scan(0x42)
 except Exception as e:result=str(e)
 print(json.dumps({'test':'official_scan_0x42','attempt':index+1,'result':result,'ms':(time.monotonic()-started)*1000}),flush=True)
 time.sleep(.1)
try:
 result=bus.readfrom(0x42,32)
 print(json.dumps({'test':'official_read32','result':None if result is None else bytes(result).hex(' ')}),flush=True)
except Exception as e:print('official_read32 exception: '+str(e),flush=True)
ready=bytes.fromhex('aa550552454144590c0d')
packet=bytes([len(ready)])+ready+bytes(31-len(ready))
try:
 result=bus.writeto(0x42,packet)
 print(json.dumps({'test':'official_write32_READY','result':result,'expected':32}),flush=True)
except Exception as e:print('official_write32 exception: '+str(e),flush=True)
fd=os.open('/dev/i2c-6',os.O_RDWR)
try:
 fcntl.ioctl(fd,0x0703,0x42)
 for name,action in [('linux_read1',lambda:os.read(fd,1)),('linux_write32_READY',lambda:os.write(fd,packet))]:
  try:
   result=action()
   print(json.dumps({'test':name,'result':result.hex() if isinstance(result,bytes) else result}),flush=True)
  except OSError as e:print(json.dumps({'test':name,'errno':e.errno,'error':str(e)}),flush=True)
finally:os.close(fd)
'''
c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20)
def run(cmd):
 _,o,e=c.exec_command(cmd,timeout=35);a=o.read().decode(errors='replace');b=e.read().decode(errors='replace')
 print(a,flush=True)
 if b:print(b,flush=True)
 if o.channel.recv_exit_status():raise RuntimeError('Remote command failed')
try:
 run("pkill -9 -f '[m]ain\\.py' || true; pkill -9 -f '[l]auncher' || true")
 time.sleep(2)
 try:
  run('python3 -c '+shlex.quote(PROBE))
 finally:
  run("printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &")
  time.sleep(8)
  run("ps aux | grep '[p]ython3 /maixapp/apps/yyb_mover/main.py'; tail -5 /maixapp/tmp/last_run.log")
finally:c.close()
