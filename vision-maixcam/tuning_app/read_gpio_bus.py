import getpass
import paramiko
import sys
c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect(sys.argv[1] if len(sys.argv)>1 else '192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20,banner_timeout=20)
try:
 _,o,e=c.exec_command("date; ps aux | grep '[p]ython3 /maixapp/apps/yyb_mover/main.py'; cat /sys/kernel/debug/gpio; tail -8 /maixapp/tmp/last_run.log",timeout=20)
 print(o.read().decode());print(e.read().decode())
finally:c.close()
