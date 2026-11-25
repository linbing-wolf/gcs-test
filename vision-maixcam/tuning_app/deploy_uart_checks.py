"""Install UART diagnostics on legacy firmware with the original send/read code."""
import getpass
import hashlib
from pathlib import Path
import shlex
import time
import paramiko

c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20)
def run(cmd):
 _,o,e=c.exec_command(cmd,timeout=30);a=o.read().decode(errors='replace');b=e.read().decode(errors='replace')
 if o.channel.recv_exit_status():raise RuntimeError(b or a)
 print(a,flush=True)
try:
 base='/maixapp/apps/yyb_mover/'
 stamp=time.strftime('%Y%m%d-%H%M%S')
 backup='/root/yyb_mover-before-uart-checks-'+stamp+'.tar.gz'
 run('tar -czf '+backup+' -C /maixapp/apps yyb_mover; sync')
 with c.open_sftp() as s:
  def read(n):
   with s.open(base+n,'rb') as f:return f.read().decode('utf-8').replace('\r\r\n','\n').replace('\r\n','\n')
  code=read('protocol.py')
  if 'self.ser.write(pkt[sent:])' in code or 'from uart_write_checked import write_frame' in code:
   print('UART send checks already present')
   raise SystemExit(0)
  old='        self.ser.write(pkt)\n        self.slog.tx(payload, pkt)'
  new='''        try:
            written, calls, duration = write_frame(self.ser, pkt)
        except Exception as exc:
            self.slog.event("TX FAILED %s: %s" % (self.dev, exc))
            print("[UART-TX-ERROR] %s: %s" % (self.dev, exc))
            raise
        self.slog.event("TX WRITE OK dev=%s bytes=%d/%d calls=%d duration_ms=%.3f" %
                        (self.dev, written, len(pkt), calls, duration))
        print("[UART-TX] dev=%s written=%d/%d calls=%d duration_ms=%.3f" %
              (self.dev, written, len(pkt), calls, duration))
        self.slog.tx(payload, pkt)'''
  if code.count(old)!=1:raise RuntimeError('Unexpected send implementation')
  code=code.replace(old,new).replace('import serial_log','import serial_log\nfrom uart_write_checked import write_frame',1)
  old='''        except Exception:
            return None
        if data:'''
  new='''        except Exception as exc:
            now = ticks_ms()
            self._rx_errors = getattr(self, "_rx_errors", 0) + 1
            if self._rx_errors == 1 or now - getattr(self, "_rx_error_logged", 0) >= 5000:
                self._rx_error_logged = now
                message = "RX FAILED dev=%s count=%d error=%s" % (self.dev, self._rx_errors, exc)
                self.slog.event(message)
                print("[UART-RX-ERROR] " + message)
            return None
        if data:'''
  if code.count(old)!=1:raise RuntimeError('Unexpected receive implementation')
  code=code.replace(old,new)
  yaml=read('app.yaml')
  yaml=yaml.rstrip()+'\n  - uart_write_checked.py\n'
  payloads={'protocol.py':code.encode('utf-8'),'app.yaml':yaml.encode('utf-8'),
            'uart_write_checked.py':Path(__file__).with_name('uart_write_checked.py').read_bytes()}
  for name,data in payloads.items():
   if name.endswith('.py'):compile(data,name,'exec')
   with s.open(base+name+'.new','wb') as f:f.write(data)
  run("pkill -9 -f '[m]ain\\.py' || true; pkill -9 -f '[l]auncher' || true")
  time.sleep(2)
  for name,data in payloads.items():
   s.posix_rename(base+name+'.new',base+name)
   with s.open(base+name,'rb') as f:
    if hashlib.sha256(f.read()).digest()!=hashlib.sha256(data).digest():raise RuntimeError('Mismatch '+name)
  command="import os; paths="+repr([base+n for n in payloads])+"; [(os.fsync(f.fileno()),f.close()) for f in [open(p,'rb') for p in paths]]; os.sync()"
  run('python3 -c '+shlex.quote(command))
 run("find /maixapp/apps/yyb_mover -type f -name '*.pyc' -delete; printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &")
 time.sleep(8)
 run("tail -30 /maixapp/tmp/last_run.log; cat /proc/tty/driver/serial")
 print('Backup: '+backup,flush=True)
finally:c.close()
