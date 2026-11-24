import getpass,paramiko,time
c=paramiko.SSHClient(); c.load_system_host_keys(); c.set_missing_host_key_policy(paramiko.RejectPolicy()); c.connect('192.168.3.51',username='root',password=getpass.getpass('Device password: '),look_for_keys=False,allow_agent=False,timeout=8)
def run(cmd):
 _,o,e=c.exec_command(cmd,timeout=50); s=o.read().decode(errors='replace'); err=e.read().decode(errors='replace')
 if o.channel.recv_exit_status(): raise RuntimeError(err or s)
 print(s,flush=True); return s
try:
 source='/root/yyb_mover-before-h-20261028-204900.tar.gz'
 version=run('tar -xOzf '+source+' yyb_mover/app.yaml')
 if 'version: 3.9.89' not in version: raise RuntimeError('Backup version mismatch')
 backup='/root/yyb_mover-before-rollback-'+time.strftime('%Y%m%d-%H%M%S')+'.tar.gz'
 run('tar -czf '+backup+' -C /maixapp/apps yyb_mover'); print('Current app backup: '+backup,flush=True)
 run("pkill -9 -f '[m]ain\\.py' || true; pkill -9 -f '[l]auncher' || true")
 time.sleep(3)
 run('tar -xzf '+source+' -C /maixapp/apps')
 run("python3 -c \"import ast,pathlib; p=pathlib.Path('/maixapp/apps/yyb_mover'); [ast.parse((p/n).read_text()) for n in ['config.py','vision.py','tasks.py']]; print('Syntax OK')\"")
 run("find /maixapp/apps/yyb_mover -type f -name '*.pyc' -delete; ulimit -c unlimited; printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' > /tmp/run_app.txt; nohup /maixapp/apps/launcher/launcher_daemon >/tmp/launcher_daemon.log 2>&1 < /dev/null &")
 time.sleep(7)
 run("grep '^version:' /maixapp/apps/yyb_mover/app.yaml; grep -E '^PLATE_(INSIDE_MIN_RATIO|RIM_MIN_SIDE_RAYS)' /maixapp/apps/yyb_mover/config.py; ps aux | grep '[p]ython3 /maixapp/apps/yyb_mover/main.py'; tail -18 /maixapp/tmp/last_run.log")
finally: c.close()
