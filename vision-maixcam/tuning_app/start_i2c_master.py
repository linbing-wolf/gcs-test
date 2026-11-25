import getpass
import time
import paramiko

c = paramiko.SSHClient()
c.load_system_host_keys()
c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51', username='root', password=getpass.getpass('Device password: '), look_for_keys=False, allow_agent=False, timeout=20, banner_timeout=20)

def run(cmd):
    _, out, err = c.exec_command(cmd, timeout=25)
    result, error = out.read().decode(errors='replace'), err.read().decode(errors='replace')
    if out.channel.recv_exit_status():
        raise RuntimeError(error or result)
    print(result, flush=True)

try:
    run("python3 -c \"from pathlib import Path; p=Path('/maixapp/apps/yyb_mover'); assert 'version: 3.9.89-i2c-test' in (p/'app.yaml').read_text(); assert 'i2c.Mode.MASTER, 10000' in (p/'i2c_transport.py').read_text(); print('I2C master 10k configuration confirmed')\"")
    run("pkill -9 -f '[m]ain\\.py' || true; pkill -9 -f '[l]auncher' || true")
    time.sleep(2)
    run("printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &")
    time.sleep(8)
    run("ps aux | grep '[p]ython3 /maixapp/apps/yyb_mover/main.py'; tail -26 /maixapp/tmp/last_run.log")
finally:
    c.close()
