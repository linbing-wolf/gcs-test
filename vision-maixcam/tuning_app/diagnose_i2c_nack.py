import getpass
import paramiko
c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20)
try:
 command="""python3 - <<'PY'
from pathlib import Path
import mmap,os,struct,time,subprocess
print(subprocess.getoutput("date; ps aux | grep '[p]ython3 /maixapp/apps/yyb_mover/main.py'"))
print(subprocess.getoutput("tail -10 /maixapp/tmp/last_run.log"))
control=Path('/sys/kernel/debug/dynamic_debug/control')
if control.exists():
 lines=[s for s in control.read_text().splitlines() if 'i2c_dw_handle_tx_abort' in s]
 print('DEBUG RULES:',lines,flush=True)
 enabled=any('=p' in s or '=_' not in s for s in lines)
 if lines and not enabled:
  try:
   control.write_text('func i2c_dw_handle_tx_abort +p')
   time.sleep(.6)
   result=subprocess.check_output(['dmesg'],text=True)
   print('\\n'.join([s for s in result.splitlines() if 'i2c_dw_handle_tx_abort' in s][-12:]))
  finally:control.write_text('func i2c_dw_handle_tx_abort -p')
else: print('Dynamic debug unavailable')
print(subprocess.getoutput("zcat /proc/config.gz | grep CONFIG_DYNAMIC_DEBUG"))
print(subprocess.getoutput("cat /sys/kernel/debug/clk/clk_summary | grep -iE 'i2c|apb'"))
fd=os.open('/dev/mem',os.O_RDONLY|os.O_SYNC)
try:
 with mmap.mmap(fd,4096,flags=mmap.MAP_SHARED,prot=mmap.PROT_READ,offset=0x4856000) as mem:
  for name,offset in [('IC_CON',0),('SS_HCNT',0x14),('SS_LCNT',0x18),('FS_HCNT',0x1c),('FS_LCNT',0x20),('ENABLE',0x6c),('STATUS',0x70)]:
   print(name,hex(struct.unpack_from('<I',mem,offset)[0]))
finally:os.close(fd)
PY"""
 _,o,e=c.exec_command(command,timeout=20);print(o.read().decode());print(e.read().decode())
finally:c.close()
