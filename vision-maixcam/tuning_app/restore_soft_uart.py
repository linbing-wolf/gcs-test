import getpass
import time
import paramiko

c = paramiko.SSHClient()
c.load_system_host_keys()
c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51', username='root', password=getpass.getpass('Device password: '), look_for_keys=False, allow_agent=False, timeout=20, banner_timeout=20)

def run(cmd):
    _, out, err = c.exec_command(cmd, timeout=45)
    result, error = out.read().decode(errors='replace'), err.read().decode(errors='replace')
    if out.channel.recv_exit_status():
        raise RuntimeError(error or result)
    print(result, flush=True)
    return result

try:
    source = '/root/yyb_mover-softuart-test-20261029-192246.tar.gz'
    version = run('tar -xOzf ' + source + ' yyb_mover/app.yaml')
    if 'version: 3.9.89-softuart-test' not in version.splitlines():
        raise RuntimeError('Unexpected backup version')
    backup = '/root/yyb_mover-hardware-before-switch-' + time.strftime('%Y%m%d-%H%M%S') + '.tar.gz'
    run('tar -czf ' + backup + ' -C /maixapp/apps yyb_mover')
    print('Hardware backup: ' + backup, flush=True)
    run("pkill -9 -f '[m]ain\\.py' || true; pkill -9 -f '/yyb_mover/[s]oft_uart.py' || true; pkill -9 -f '[l]auncher' || true")
    time.sleep(2)
    run('tar -xzf ' + source + ' -C /maixapp/apps')
    run("python3 -c \"import ast,pathlib; p=pathlib.Path('/maixapp/apps/yyb_mover'); [ast.parse((p/n).read_text()) for n in ['protocol.py','config.py','soft_uart.py']]; print('Syntax OK')\"")
    run("find /maixapp/apps/yyb_mover -type f -name '*.pyc' -delete; printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &")
    time.sleep(8)
    run("grep '^version:' /maixapp/apps/yyb_mover/app.yaml; grep -E '^(UART_DEV|UART_BAUDRATE|PIN_TX|PIN_RX) =' /maixapp/apps/yyb_mover/config.py; ps aux | grep -E '[p]ython3 /maixapp/apps/yyb_mover/(main|soft_uart).py'; tail -24 /maixapp/tmp/last_run.log")
finally:
    c.close()
