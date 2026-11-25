import getpass
import time
import paramiko
c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20)
def run(cmd):
 _,o,e=c.exec_command(cmd,timeout=25);a=o.read().decode(errors='replace');b=e.read().decode(errors='replace')
 if o.channel.recv_exit_status():raise RuntimeError(b or a)
 print(a,flush=True)
try:
 run("ls -l /proc/2889/fd 2>/dev/null; cat /proc/tty/driver/serial")
 run("pkill -9 -f '[m]ain\\.py' || true; pkill -9 -f '[l]auncher' || true")
 time.sleep(2)
 run("printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &")
 time.sleep(8)
 run("ps aux | grep '[p]ython3 /maixapp/apps/yyb_mover/main.py'; tail -23 /maixapp/tmp/last_run.log; cat /proc/tty/driver/serial")
 time.sleep(12)
 run("tail -10 /maixapp/tmp/last_run.log; cat /proc/tty/driver/serial")
finally:c.close()
