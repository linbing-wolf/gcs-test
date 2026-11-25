import getpass,paramiko,time
c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20)
try:
 cmd="""python3 - <<'PY'
from maix import pinmap
for p,f in [('A1','I2C6_SCL'),('A0','I2C6_SDA')]:
 try:print('BEFORE',p,pinmap.get_pin_function(p),flush=True)
 except Exception as e:print('BEFORE',p,str(e),flush=True)
 pinmap.set_pin_function(p,f)
 print('AFTER',p,pinmap.get_pin_function(p),flush=True)
PY"""
 _,o,e=c.exec_command(cmd,timeout=15);print(o.read().decode());print(e.read().decode())
 time.sleep(3)
 _,o,e=c.exec_command('tail -16 /maixapp/tmp/last_run.log',timeout=15);print(o.read().decode())
finally:c.close()
