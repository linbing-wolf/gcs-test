import getpass
import paramiko
c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20,banner_timeout=20)
try:
 for cmd in ["date; ps aux | grep -E '[m]ain.py|[l]auncher|[s]oft_uart.py'; tail -100 /maixapp/tmp/last_run.log; cat /proc/tty/driver/serial; stty -F /dev/ttyS4 -a",
 "cat /tmp/run_app.txt 2>/dev/null; cat /maixapp/apps/yyb_mover/run.sh; grep -E '^(UART_|PIN_|version:)' /maixapp/apps/yyb_mover/config.py /maixapp/apps/yyb_mover/app.yaml; ls -l /maixapp/apps/yyb_mover/main.py; tail -6 /root/yyb_mover_serial/sessions.jsonl"]:
  _,o,e=c.exec_command(cmd,timeout=20);print(o.read().decode(errors='replace'),flush=True);print(e.read().decode(errors='replace'),flush=True)
finally:c.close()
