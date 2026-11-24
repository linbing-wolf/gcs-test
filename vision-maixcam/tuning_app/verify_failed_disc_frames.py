import getpass
from pathlib import Path
import paramiko

c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20)
code='''import sys,json
sys.path.insert(0,'/maixapp/apps/yyb_mover')
from maix import image
from disc_template_locator import locate
for folder in ['20261030-212845-D','20261030-213129-D']:
    for name in ['000_start.jpg','001_no_disc.jpg','002_final_none.jpg']:
        path='/root/yyb_mover_data/disc_debug/'+folder+'/'+name
        meta={}
        p=locate(image.load(path),meta)
        print(json.dumps(dict(file=path,position=p,meta=meta)),flush=True)
        expected=(443,99) if folder=='20261030-212845-D' else (431,150)
        if p is None or abs(p[0]-expected[0])>15 or abs(p[1]-expected[1])>15:raise RuntimeError('Failed-frame regression')
print('FAILED_FRAMES_PASS 6/6')
'''
try:
    with c.open_sftp() as s:
        with s.open('/tmp/test_failed_disc.py','wb') as f:f.write(code.encode())
    _,o,e=c.exec_command('python3 /tmp/test_failed_disc.py',timeout=60)
    text=o.read().decode(errors='replace');error=e.read().decode(errors='replace')
    print(text,error)
    folder=Path(__file__).resolve().parent.parent/'_from_device/disc_template_20261030'
    (folder/'failed_frames_validation.txt').write_text(text+error,encoding='utf-8')
    if o.channel.recv_exit_status():raise RuntimeError('Device failed-frame regression failed')
finally:c.close()
