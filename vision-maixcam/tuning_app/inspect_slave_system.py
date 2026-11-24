import getpass
import paramiko
from pathlib import Path
c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20)
try:
 cmds=["uname -a; cat /proc/cmdline; ls -l /boot/board /boot/boards /boot/configs; cat /boot/board; ls /sys/kernel/config; readlink /sys/devices/platform/soc/4856000.i2c/driver; ls /usr/src /lib/modules",
 "python3 - <<'PY'\nfrom pathlib import Path\np=Path('/sys/firmware/devicetree/base/soc/i2c@4856000')\nfor f in p.rglob('*'):\n if f.is_file():\n  b=f.read_bytes();print(str(f.relative_to(p)), b.hex(),repr(b) if len(b)<180 else '')\nfor p in [Path('/boot/configs'),Path('/boot/board')]:\n print(str(p),p.read_text(errors='replace')[:10000] if p.is_file() else '')\nPY",
 "command -v dtc; command -v fdtoverlay; ls /sys/kernel/debug; zcat /proc/config.gz | grep -E 'CONFIG_(OF_OVERLAY|OF_DYNAMIC|CONFIGFS|I2C_DESIGNWARE|I2C_SLAVE|IKCONFIG)' ; ls -l /sys/firmware/fdt"]
 cmds=["gcc --version | head -1; command -v flex; command -v bison; command -v bc; command -v openssl; command -v perl; command -v insmod; command -v modinfo; test -f /usr/include/openssl/bio.h && echo OPENSSL_HEADERS; test -f /usr/include/libelf.h && echo ELF_HEADERS"]
 for cmd in cmds:
  _,o,e=c.exec_command(cmd,timeout=25);print(o.read().decode(errors='replace'),flush=True);print(e.read().decode(errors='replace'),flush=True)
 dest=Path(__file__).resolve().parent.parent/'_from_device/i2c_slave_system'
 dest.mkdir(parents=True,exist_ok=True)
 with c.open_sftp() as s:
  s.get('/sys/firmware/fdt',str(dest/'running.dtb'))
  s.get('/proc/config.gz',str(dest/'kernel.config.gz'))
 print('Saved running DTB and kernel config',flush=True)
finally:c.close()
