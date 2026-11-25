import getpass
import paramiko
c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20)
try:
 _,o,e=c.exec_command("dmesg | grep -Ei 'i2c|nack|abort|4856000' | tail -45; tail -8 /maixapp/tmp/last_run.log",timeout=20)
 print(o.read().decode());print(e.read().decode())
finally:c.close()
