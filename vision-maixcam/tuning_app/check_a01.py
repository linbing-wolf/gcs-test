import getpass
import paramiko
import sys

c = paramiko.SSHClient()
c.load_system_host_keys()
c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect(sys.argv[1] if len(sys.argv) > 1 else '192.168.3.51', username='root', password=getpass.getpass('Device password: '), look_for_keys=False, allow_agent=False, timeout=20, banner_timeout=20, auth_timeout=20)
try:
    _, out, err = c.exec_command('python3 -c "from maix import pinmap; print([(p,pinmap.get_pin_functions(p)) for p in [\'A0\',\'A1\']])"', timeout=15)
    print(out.read().decode())
    print(err.read().decode())
finally:
    c.close()
