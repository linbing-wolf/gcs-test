import getpass
import paramiko
c=paramiko.SSHClient(); c.load_system_host_keys(); c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20)
try:
    _,o,e=c.exec_command("python3 -c \"from maix import i2c; print(i2c.I2C.__init__.__doc__); b=i2c.I2C(6,i2c.Mode.MASTER,10000); print('OPENED 10000')\"",timeout=20)
    print(o.read().decode());print(e.read().decode())
finally:c.close()
