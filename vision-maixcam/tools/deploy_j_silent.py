"""Deploy the J runtime patch directly over SFTP with backup and rollback."""

import argparse
import getpass
import hashlib
import os
from pathlib import Path
import time

import paramiko


ROOT = Path(__file__).resolve().parent.parent
APP_ROOT = ROOT / "maixcam_app_v2"
REMOTE_APP = "/maixapp/apps/yyb_mover"
RUNTIME_FILES = (
    "config.py",
    "protocol.py",
    "tasks.py",
    "material_state.py",
    "main.py",
    "screen_echo.py",
    "cam_light.py",
    "i2c_transport.py",
    "gpio_i2c_master.py",
)


def validate_runtime_transport():
    """Reject a mixed upload before it replaces the board's working I2C link."""
    config_text = (APP_ROOT / "config.py").read_text(encoding="utf-8")
    protocol_text = (APP_ROOT / "protocol.py").read_text(encoding="utf-8")
    uses_gpio_i2c = 'UART_DEV = "i2c-gpio-master:0x42"' in config_text
    if uses_gpio_i2c and 'from i2c_transport import I2CTransport' not in protocol_text:
        raise RuntimeError(
            "config.py selects GPIO I²C but protocol.py has no I2CTransport support"
        )


def run(client, command, timeout=45):
    _, stdout, stderr = client.exec_command(command, timeout=timeout)
    output = stdout.read().decode(errors="replace")
    error = stderr.read().decode(errors="replace")
    status = stdout.channel.recv_exit_status()
    if status:
        raise RuntimeError("remote command failed (%d): %s\n%s" % (status, error, output))
    return output


def digest(data):
    return hashlib.sha256(data).hexdigest()


def upload_files(sftp, source_root, names):
    for name in names:
        data = (source_root / name).read_bytes()
        remote = REMOTE_APP + "/" + name
        with sftp.open(remote, "wb") as handle:
            handle.write(data)
        sftp.chmod(remote, 0o644)


def restart(client):
    run(client, "pkill -9 -f '[m]ain\\.py' || test $? = 1")
    run(client, "pkill -9 -f '[l]auncher' || test $? = 1")
    time.sleep(3)
    run(client, "find /maixapp/apps/yyb_mover -type f -name '*.pyc' -delete")
    run(
        client,
        "printf '/maixapp/apps/yyb_mover/main.py\\nyyb_mover\\nauto_start\\n' "
        "> /tmp/run_app.txt; "
        "ulimit -c unlimited; "
        "nohup /maixapp/apps/launcher/launcher_daemon "
        ">/tmp/launcher_daemon.log 2>&1 < /dev/null &",
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="100.118.92.10")
    parser.add_argument("--user", default="root")
    args = parser.parse_args()

    password = os.environ.get("MAIX_SSH_PASSWORD")
    if password is None:
        password = getpass.getpass("Device SSH password: ")

    validate_runtime_transport()

    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    client.connect(
        args.host,
        username=args.user,
        password=password,
        look_for_keys=False,
        allow_agent=False,
        timeout=12,
        auth_timeout=12,
        banner_timeout=12,
    )

    backup = "/root/yyb-before-j-silent-" + time.strftime("%Y%m%d-%H%M%S")
    rolled_back = False
    try:
        version = run(
            client,
            "awk '/^version:/{print $2; exit}' /maixapp/apps/yyb_mover/app.yaml",
        ).strip()
        run(client, "test -d %s" % REMOTE_APP)
        run(client, "test ! -e %s && cp -a %s %s" % (backup, REMOTE_APP, backup))
        print("backup=" + backup)
        print("remote_version_before=" + version)

        with client.open_sftp() as sftp:
            upload_files(sftp, APP_ROOT, RUNTIME_FILES)
            for name in RUNTIME_FILES:
                local = digest((APP_ROOT / name).read_bytes())
                with sftp.open(REMOTE_APP + "/" + name, "rb") as handle:
                    remote = digest(handle.read())
                if local != remote:
                    raise RuntimeError("hash mismatch after upload: " + name)
                print("%s=%s" % (name, local))

        restart(client)
        active = ""
        for _ in range(12):
            time.sleep(2)
            active = run(client, "ps aux | grep '[m]ain.py' | grep -v grep || true")
            if active.strip():
                break
        if not active.strip():
            raise RuntimeError("main.py did not stay active after restart")

        tail = run(client, "tail -100 /maixapp/tmp/last_run.log")
        errors = run(
            client,
            "grep -Ein 'traceback|syntaxerror|importerror|segmentation|fatal' "
            "/maixapp/tmp/last_run.log | tail -20 || true",
        )
        ready = "READY" in tail or "I2C" in tail
        print(active.strip())
        print(tail)
        if errors.strip() or not ready:
            raise RuntimeError("startup verification failed")
        print("startup_ready=yes")
        print("deployed_files=" + ",".join(RUNTIME_FILES))
        print("j_confirm_ms=" + run(
            client,
            "grep -E '^MATERIAL_STATE_CONFIRM_MS' "
            "/maixapp/apps/yyb_mover/config.py",
        ).strip())
    except Exception:
        if not rolled_back:
            print("deployment failed, restoring " + backup)
            try:
                for name in RUNTIME_FILES:
                    run(
                        client,
                        "cp -a %s/%s %s/%s"
                        % (backup, name, REMOTE_APP, name),
                    )
                restart(client)
                rolled_back = True
            except Exception as rollback_error:
                print("rollback failed: %r" % (rollback_error,))
        raise
    finally:
        password = None
        client.close()


if __name__ == "__main__":
    main()
