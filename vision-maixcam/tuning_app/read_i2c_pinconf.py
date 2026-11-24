import getpass
import paramiko
c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20)
try:
 _,o,e=c.exec_command("ls /sys/kernel/debug/pinctrl; grep -H -E 'pin (0|1) \\(|A0|A1 ' /sys/kernel/debug/pinctrl/*/pin* 2>/dev/null | head -35",timeout=20)
 print(o.read().decode());print(e.read().decode())
finally:c.close()
