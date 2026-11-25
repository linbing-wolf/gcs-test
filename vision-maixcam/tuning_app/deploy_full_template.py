import getpass
import hashlib
import json
from pathlib import Path
import shlex
import time
import paramiko

root=Path(__file__).resolve().parent
evidence=root.parent/'_from_device/disc_template_20261030'
manifest=json.loads((root.parent/'_from_device/new_field_other/manifest.json').read_text())
cases=[(v['remote'],[430,150] if i<10 else None) for i,v in enumerate(manifest)]
remote_test='''import sys,time,json
sys.path.insert(0,'/maixapp/apps/yyb_mover')
from maix import image
from disc_template_locator import locate
cases=CASES
report=[]
for path,expected in cases:
    frame=image.load(path)
    meta={}
    p=locate(frame,meta)
    print(json.dumps(dict(path=path,position=p,meta=meta)),flush=True)
    if p is None:raise RuntimeError('Full-frame target missed: '+path)
    if expected and ((p[0]-expected[0])**2+(p[1]-expected[1])**2)**.5>15:raise RuntimeError('Wrong target center: '+path)
    report.append(meta['ms'])
    if expected:
        # Remove the entire 2 region: 1 on the left, partial 3 on the right,
        # and the lower gripper remain as independent negative images.
        for box in [(0,0,280,480),(550,0,90,480),(0,270,640,210)]:
            negative=frame.crop(*box)
            if locate(negative,{}) is not None:raise RuntimeError('False target in negative region: '+str(box))
print('FULL_TEMPLATE_PASS count=%d max_ms=%.2f mean_ms=%.2f'%(len(report),max(report),sum(report)/len(report)),flush=True)
'''.replace('CASES',repr(cases))
c=paramiko.SSHClient();c.load_system_host_keys();c.set_missing_host_key_policy(paramiko.RejectPolicy())
c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=20)
def run(cmd,timeout=120):
    _,o,e=c.exec_command(cmd,timeout=timeout)
    a=o.read().decode(errors='replace');b=e.read().decode(errors='replace')
    print(a,b,flush=True)
    if o.channel.recv_exit_status():raise RuntimeError('Remote command failed')
    return a
try:
    # Test new helper in /tmp before changing the active app.
    helper=(root/'disc_template_locator.py').read_bytes()
    with c.open_sftp() as s:
        with s.open('/tmp/disc_template_locator.py','wb') as f:f.write(helper)
        with s.open('/tmp/test_full_template.py','wb') as f:f.write(remote_test.encode())
    result=run('python3 /tmp/test_full_template.py',240)
    (evidence/'full_template_device_validation.txt').write_text(result,encoding='utf-8')
    base='/maixapp/apps/yyb_mover/'
    with c.open_sftp() as s:
        with s.open(base+'vision.py','rb') as f:original=f.read()
        text=original.decode().replace('\r\r\n','\n').replace('\r\n','\n')
        start=text.index('def find_disc_center(img,')
        point=text.index('    t0 = ticks_ms()',start)
        injection='''    # Search the complete image for the printed 2 before circle proposals.
    if bool(getattr(config, "DISC_TEMPLATE_ON", True)):
        from disc_template_locator import locate
        direct = locate(img, meta)
        if direct is not None:
            return direct

'''
        if 'from disc_template_locator import locate' in text:raise RuntimeError('Already patched; inspect before redeploy')
        text=text[:point]+injection+text[point:]
        compile(text,'vision.py','exec');compile(helper,'disc_template_locator.py','exec')
        stamp=time.strftime('%Y%m%d-%H%M%S')
        backup=base+'vision.py.before-full-template-'+stamp
        (evidence/'device_vision_before_full_template.py').write_bytes(original)
        with s.open(backup,'wb') as f:f.write(original)
        files={'vision.py':text.encode(),'disc_template_locator.py':helper}
        with s.open(base+'app.yaml','rb') as f:yaml=f.read().decode()
        if '  - disc_template_locator.py' not in yaml:yaml=yaml.rstrip()+'\n  - disc_template_locator.py\n'
        files['app.yaml']=yaml.encode()
        for name,data in files.items():
            with s.open(base+name+'.new','wb') as f:f.write(data)
        run("pkill -9 -f '[m]ain\\.py' || true; pkill -9 -f '[l]auncher' || true")
        time.sleep(2)
        for name,data in files.items():
            s.posix_rename(base+name+'.new',base+name)
            with s.open(base+name,'rb') as f:
                if hashlib.sha256(f.read()).digest()!=hashlib.sha256(data).digest():raise RuntimeError('Hash mismatch: '+name)
    code="import os; paths="+repr([base+n for n in files]+[backup])+"; [(os.fsync(f.fileno()),f.close()) for f in [open(p,'rb') for p in paths]]; os.sync()"
    run('python3 -c '+shlex.quote(code))
    run("find /maixapp/apps/yyb_mover -type f -name '*.pyc' -delete; printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &")
    time.sleep(10)
    run("ps aux | grep '[p]ython3 /maixapp/apps/yyb_mover/main.py'; tail -16 /maixapp/tmp/last_run.log")
    print('Backup: '+backup)
finally:c.close()
