"""Read-only H incident collection."""
import getpass
from pathlib import Path
import paramiko

dest = Path(__file__).resolve().parent.parent / '_from_device/h_latest'
dest.mkdir(exist_ok=True)
client = paramiko.SSHClient()
client.load_system_host_keys()
client.set_missing_host_key_policy(paramiko.RejectPolicy())
client.connect('192.168.3.51', username='root', password=getpass.getpass('Device password: '),
               look_for_keys=False, allow_agent=False, timeout=8)
try:
    command = "find /root/yyb_mover_data/plate_debug -type f -printf '%T@ %s %p\\n' 2>/dev/null | sort -nr | head -12"
    _, stdout, stderr = client.exec_command(command, timeout=25)
    listing = stdout.read().decode()
    print(listing, flush=True)
    (dest / 'listing.txt').write_text(listing, encoding='utf-8')
    _, out, err = client.exec_command("date; grep -E '^PLATE_(INSIDE_MIN_RATIO|DEBUG_SAVE)' /maixapp/apps/yyb_mover/config.py; tail -8 /root/yyb_mover_serial/sessions.jsonl", timeout=10)
    print(out.read().decode(), flush=True)
    with client.open_sftp() as sftp:
        sftp.get('/maixapp/tmp/last_run.log', str(dest / 'last_run.log'))
        files = []
        for line in listing.splitlines():
            parts = line.split(' ', 2)
            if len(parts) == 3 and parts[2].lower().endswith(('.mp4', '.avi', '.h264', '.mkv')):
                files.append(parts[2])
        if files:
            path = files[0]
            sftp.get(path, str(dest / Path(path).name))
            print('Downloaded: ' + path, flush=True)
        for line in listing.splitlines()[:15]:
            parts = line.split(' ', 2)
            if len(parts) == 3 and '/plate_debug/' in parts[2]:
                path = parts[2]
                sftp.get(path, str(dest / (Path(path).parent.name + '_' + Path(path).name)))
finally:
    client.close()
