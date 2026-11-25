import getpass
import paramiko
c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20)
try:
 command="""python3 - <<'PY'
import os,time,termios
def stats():
 print([x.strip() for x in open('/proc/tty/driver/serial') if x.startswith('4:')],flush=True)
stats()
fd=os.open('/dev/ttyS4',os.O_WRONLY|os.O_NOCTTY)
try:
 packet=bytes.fromhex('aa 55 05 52 45 41 44 59 0c 0d')
 for i in range(3):
  n=os.write(fd,packet)
  termios.tcdrain(fd)
  print('PROBE',i+1,'requested',len(packet),'written',n,'drained',packet.hex(' '),flush=True)
  time.sleep(1)
finally:os.close(fd)
stats()
PY"""
 _,o,e=c.exec_command(command,timeout=15);print(o.read().decode());print(e.read().decode())
finally:c.close()
