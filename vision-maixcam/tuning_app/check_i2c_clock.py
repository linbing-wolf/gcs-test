import getpass
import paramiko

c = paramiko.SSHClient()
c.load_system_host_keys()
c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51', username='root', password=getpass.getpass('Device password: '), look_for_keys=False, allow_agent=False, timeout=20)
try:
    command = '''python3 - <<'PY'
from pathlib import Path
p=Path('/sys/class/i2c-adapter/i2c-6')
print('attrs:',[str(x) for x in p.resolve().parent.iterdir()])
print('boot:',[str(x) for x in Path('/boot').iterdir()])
print('adapter:', p.resolve())
print('name:', (p/'name').read_text())
for root in [p/'of_node',p/'device/of_node']:
 print('node:',root.resolve())
 if root.exists():
  for f in root.iterdir():
   if any(k in f.name for k in ['clock','freq','speed','timing']):
    b=f.read_bytes(); print(f.name,b.hex(), 'u32:', [int.from_bytes(b[i:i+4],'big') for i in range(0,len(b),4)])
PY'''
    _, out, err = c.exec_command(command, timeout=15)
    print(out.read().decode()); print(err.read().decode())
finally:
    c.close()
