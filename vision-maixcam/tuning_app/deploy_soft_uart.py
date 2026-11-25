"""Install the experimental A1/A0 transport over the device's current app."""
import ast
import getpass
import hashlib
from pathlib import Path
import re
import time
import paramiko

c = paramiko.SSHClient()
c.load_system_host_keys()
c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51', username='root', password=getpass.getpass('Device password: '),
          look_for_keys=False, allow_agent=False, timeout=20, banner_timeout=20)

def run(cmd):
    _, out, err = c.exec_command(cmd, timeout=40)
    result, error = out.read().decode(errors='replace'), err.read().decode(errors='replace')
    if out.channel.recv_exit_status():
        raise RuntimeError(error or result)
    print(result, flush=True)
    return result

try:
    base = '/maixapp/apps/yyb_mover/'
    backup = '/root/yyb_mover-before-softuart-' + time.strftime('%Y%m%d-%H%M%S') + '.tar.gz'
    run('tar -czf ' + backup + ' -C /maixapp/apps yyb_mover')
    print('BACKUP=' + backup, flush=True)
    with c.open_sftp() as s:
        def read(name):
            with s.open(base + name, 'rb') as f:
                return f.read().decode('utf-8')
        protocol = read('protocol.py')
        config = read('config.py')
        yaml = read('app.yaml')
        if not re.search(r'(?m)^version:\s*3\.9\.89\s*$', yaml):
            raise RuntimeError('Expected deployed 3.9.89; refusing to mix versions')
        needle = 'self.ser = uart.UART(dev, baud)'
        if protocol.count(needle) != 2:
            raise RuntimeError('Unexpected UART constructor layout')
        protocol = protocol.replace(needle, 'self.ser = SoftUART(dev, baud)')
        protocol = protocol.replace('from maix import uart, pinmap',
                                    'from maix import uart, pinmap\nfrom soft_uart import SoftUART')
        values = dict(UART_DEV='soft:A1_TX/A0_RX', PIN_TX='A1', PIN_RX='A0',
                      UART_FUNC_TX='GPIOA1', UART_FUNC_RX='GPIOA0', UART_BAUDRATE=115200)
        for key, value in values.items():
            config, n = re.subn(r'(?m)^' + key + r'\s*=[^\r\n]*', key + ' = ' + repr(value), config)
            if n != 1:
                raise RuntimeError('Expected one setting: ' + key)
        yaml = re.sub(r'(?m)^version:.*$', 'version: 3.9.89-softuart-test', yaml)
        yaml += '\n' if not yaml.endswith('\n') else ''
        yaml += '  - soft_uart.py\n'
        payloads = {'protocol.py': protocol.encode(), 'config.py': config.encode(),
                    'app.yaml': yaml.encode(), 'soft_uart.py': Path(__file__).with_name('soft_uart.py').read_bytes()}
        for name, data in payloads.items():
            if name.endswith('.py'):
                ast.parse(data, filename=name)
            with s.open(base + name + '.new', 'wb') as f:
                f.write(data)
        run("pkill -9 -f '[m]ain\\.py' || true; pkill -9 -f '[l]auncher' || true")
        time.sleep(2)
        for name, data in payloads.items():
            s.posix_rename(base + name + '.new', base + name)
            with s.open(base + name, 'rb') as f:
                if hashlib.sha256(f.read()).digest() != hashlib.sha256(data).digest():
                    raise RuntimeError('Readback mismatch: ' + name)
            print('Verified ' + name, flush=True)
    run("find /maixapp/apps/yyb_mover -type f -name '*.pyc' -delete; printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &")
    time.sleep(8)
    run("ps aux | grep -E '[p]ython3 /maixapp/apps/yyb_mover/(main|soft_uart).py'; tail -35 /maixapp/tmp/last_run.log")
finally:
    c.close()
