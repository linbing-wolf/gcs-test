import getpass
import hashlib
import json
from pathlib import Path
import shlex
import time
import cv2
import paramiko
from build_disc_template import match

root=Path(__file__).resolve().parent.parent
folder=root/'_from_device/disc_template_20261030'
candidate=(folder/'target2_candidate.png').read_bytes()
manifest=json.loads((root/'_from_device/new_field_other/manifest.json').read_text())
tpl=cv2.imdecode(__import__('numpy').frombuffer(candidate,dtype='uint8'),0)
cases=[]
for i,item in enumerate(manifest):
    frame=cv2.imread(item['local'],0)
    _,box=match(frame,tpl,(280,30,280,240) if i<10 else None)
    cx,cy=box[0]+box[2]/2,box[1]+box[3]/2
    x=max(0,min(round(cx)-42,frame.shape[1]-84)); y=max(0,min(round(cy)-46,frame.shape[0]-92))
    cases.append((item['remote'],x,y))
code='''from maix import image
import json
base=image.load('/tmp/target2_candidate.png').to_format(image.Format.FMT_GRAYSCALE)
tpls=[base.resize(round(60*s),round(80*s)) for s in [.7,.8,.9,1.,1.1]]
cases=CASES
results=[]
for path,x,y in cases:
    frame=image.load(path).to_format(image.Format.FMT_GRAYSCALE)
    roi=frame.crop(x,y,84,92)
    hits=[list(r) for t in tpls if (r:=roi.find_template(t,.55,step=2))]
    results.append(dict(path=path,passed=bool(hits),hits=hits))
print('TEMPLATE_RESULT='+json.dumps(results))
if not all(r['passed'] for r in results):raise RuntimeError('Template regression failed')
'''.replace('CASES',repr(cases))
c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20)
def run(command,timeout=60):
    _,o,e=c.exec_command(command,timeout=timeout)
    text=o.read().decode(errors='replace'); error=e.read().decode(errors='replace')
    print(text,flush=True)
    if error: print(error,flush=True)
    if o.channel.recv_exit_status():raise RuntimeError('Remote command failed')
    return text
try:
    with c.open_sftp() as s:
        with s.open('/tmp/target2_candidate.png','wb') as f:f.write(candidate)
        with s.open('/tmp/verify_disc_template.py','wb') as f:f.write(code.encode())
    result=run('python3 /tmp/verify_disc_template.py',120)
    (folder/'device_validation.txt').write_text(result,encoding='utf-8')
    path='/maixapp/apps/yyb_mover/assets/target2.png'
    stamp=time.strftime('%Y%m%d-%H%M%S')
    backup=path+'.backup-'+stamp
    with c.open_sftp() as s:
        with s.open(path,'rb') as f:old=f.read()
        if not old:raise RuntimeError('Existing template empty')
        (folder/'device_previous.png').write_bytes(old)
        with s.open(backup,'wb') as f:f.write(old)
        with s.open(path+'.new','wb') as f:f.write(candidate)
        s.posix_rename(path+'.new',path)
        with s.open(path,'rb') as f:
            if hashlib.sha256(f.read()).digest()!=hashlib.sha256(candidate).digest():raise RuntimeError('Hash mismatch')
    flush="import os; paths="+repr([path,backup])+"; [(os.fsync(f.fileno()),f.close()) for f in [open(p,'rb') for p in paths]]; os.sync()"
    run('python3 -c '+shlex.quote(flush))
    (root/'maixcam_app_v2/assets/target2.png').write_bytes(candidate)
    run("pkill -9 -f '[m]ain\\.py' || true; pkill -9 -f '[l]auncher' || true; sleep 2; printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &")
    time.sleep(10)
    run("ps aux | grep '[p]ython3 /maixapp/apps/yyb_mover/main.py'; tail -12 /maixapp/tmp/last_run.log")
    print('Backup: '+backup)
finally:c.close()
