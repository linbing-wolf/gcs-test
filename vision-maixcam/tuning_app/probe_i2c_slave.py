import getpass
import paramiko
c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20)
try:
 for cmd in ["printf '0x1042' > /sys/class/i2c-adapter/i2c-6/delete_device; test ! -e /sys/bus/i2c/devices/6-1042 && echo 'Probe removed; master app remains stopped'"]:
  _,o,e=c.exec_command(cmd,timeout=20);print(o.read().decode(),flush=True);print(e.read().decode(),flush=True);print('exit',o.channel.recv_exit_status())
finally:c.close()
