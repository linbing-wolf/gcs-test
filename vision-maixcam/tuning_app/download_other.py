"""Read-only SSH download of the newest other photos."""
import getpass
import json
from pathlib import Path
import paramiko

client = paramiko.SSHClient()
client.load_system_host_keys()
client.set_missing_host_key_policy(paramiko.RejectPolicy())
client.connect('192.168.3.51', username='root', password=getpass.getpass('Device password: '),
               look_for_keys=False, allow_agent=False, timeout=8)
try:
    _, output, error = client.exec_command("find /root /maixapp -type d -name other 2>/dev/null", timeout=20)
    directories = output.read().decode().splitlines()
    print(json.dumps(directories), flush=True)
    with client.open_sftp() as sftp:
        files = []
        for directory in directories:
            for item in sftp.listdir_attr(directory):
                if item.filename.lower().endswith(('.jpg', '.jpeg', '.png')):
                    files.append((item.st_mtime, directory, item.filename))
        selected = sorted(files, reverse=True)[:24]
        dest = Path(__file__).resolve().parent.parent / '_from_device/new_field_other'
        dest.mkdir(exist_ok=True)
        manifest = []
        for index, (mtime, directory, name) in enumerate(selected):
            target = dest / (str(index).zfill(2) + '_' + name)
            sftp.get(directory + '/' + name, str(target))
            manifest.append({'local': str(target), 'remote': directory + '/' + name, 'mtime': mtime})
        (dest / 'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
        print(json.dumps(manifest), flush=True)
finally:
    client.close()
