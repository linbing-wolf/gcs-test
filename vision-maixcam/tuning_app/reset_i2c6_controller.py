import getpass,time,paramiko
c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20)
def run(cmd):
 _,o,e=c.exec_command(cmd,timeout=30);a=o.read().decode(errors='replace');b=e.read().decode(errors='replace')
 print(a,flush=True)
 if o.channel.recv_exit_status():raise RuntimeError(b or a)
try:
 run("pkill -9 -f '[m]ain\\.py' || true; pkill -9 -f '[l]auncher' || true")
 time.sleep(3)
 try:
  run("printf '4856000.i2c' > /sys/bus/platform/drivers/i2c_designware/unbind")
 finally:
  run("if test ! -e /sys/devices/platform/soc/4856000.i2c/driver; then printf '4856000.i2c' > /sys/bus/platform/drivers/i2c_designware/bind; fi; cat /sys/class/i2c-adapter/i2c-6/name")
 run("printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &")
 time.sleep(15)
 run("ps aux | grep '[p]ython3 /maixapp/apps/yyb_mover/main.py'; tail -18 /maixapp/tmp/last_run.log")
finally:c.close()
