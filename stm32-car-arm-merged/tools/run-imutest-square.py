#!/usr/bin/env python3
"""Run the local imutest square sequence over the PB0/PB1 debug UART.

This helper is intentionally fail-stop: after any fault or timeout it sends
the firmware emergency-stop command and never starts the next segment.
"""

import argparse
import pathlib
import sys
import time

import serial


ACTIONS = (("f", "FORWARD"), ("r", "RIGHT"), ("b", "BACKWARD"), ("l", "LEFT"))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM9")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--distance", type=int, default=2000)
    parser.add_argument("--forward-mm", type=int)
    parser.add_argument("--right-mm", type=int)
    parser.add_argument("--backward-mm", type=int)
    parser.add_argument("--left-mm", type=int)
    parser.add_argument("--actions", default="frbl")
    parser.add_argument("--rpm", type=int, default=160)
    parser.add_argument("--kp", type=int, default=300)
    parser.add_argument("--ki", type=int, default=10)
    parser.add_argument("--kd", type=int, default=60)
    parser.add_argument("--segment-timeout", type=float, default=90.0)
    parser.add_argument("--log", required=True)
    args = parser.parse_args()

    log_path = pathlib.Path(args.log)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = log_path.open("w", encoding="utf-8", errors="replace")
    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    pending = ""

    def emit(text: str) -> None:
        log_file.write(text)
        log_file.flush()

    def read_until(marker: str, timeout: float) -> bool:
        nonlocal pending
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            chunk = ser.read(4096)
            if not chunk:
                continue
            text = chunk.decode("utf-8", "replace")
            emit(text)
            pending = (pending + text)[-8192:]
            if marker in pending:
                return True
        return False

    def command(text: str, timeout: float = 3.0) -> None:
        nonlocal pending
        pending = ""
        emit(f"\n> {text}\n")
        ser.write((text + "\r").encode("ascii"))
        ser.flush()
        if not read_until("[imutest", timeout):
            raise RuntimeError(f"no response to {text!r}")

    try:
        time.sleep(0.3)
        ser.reset_input_buffer()
        command("imutest clear", 4.0)
        command("imutest pid")
        segment_mm = {
            "f": args.forward_mm or args.distance,
            "r": args.right_mm or args.distance,
            "b": args.backward_mm or args.distance,
            "l": args.left_mm or args.distance,
        }
        command(f"imutest rpm {args.rpm}")
        command(f"imutest kp {args.kp}")
        command(f"imutest ki {args.ki}")
        command(f"imutest kd {args.kd}")
        command("imutest")

        selected = []
        for action in args.actions:
            matches = [item for item in ACTIONS if item[0] == action]
            if not matches:
                raise RuntimeError(f"unsupported action {action!r}")
            selected.append(matches[0])
        for action, name in selected:
            command(f"imutest dist {segment_mm[action]}")
            pending = ""
            emit(f"\n> imutest {action}\n")
            ser.write(f"imutest {action}\r".encode("ascii"))
            ser.flush()
            deadline = time.monotonic() + args.segment_timeout
            done = False
            while time.monotonic() < deadline:
                chunk = ser.read(4096)
                if not chunk:
                    continue
                text = chunk.decode("utf-8", "replace")
                emit(text)
                pending = (pending + text)[-8192:]
                if "[imutest rejected" in pending:
                    raise RuntimeError(f"{name} rejected: {pending.strip().splitlines()[-1]}")
                if "FAULT" in pending or "run fault" in pending:
                    raise RuntimeError(f"{name} fault")
                if f"DONE_BY_IMU_ENCODER travel={name}" in pending:
                    done = True
                    break
                if "STOPPED travel=" in pending and f"travel={name}" in pending:
                    done = True
                    break
            if not done:
                raise RuntimeError(f"{name} timed out")
            time.sleep(0.8)

        pending = ""
        emit("\n> imutest\n")
        ser.write(b"imutest\r")
        read_until("\n", 3.0)
        return 0
    except Exception as exc:
        emit(f"\n[host abort] {exc}\n")
        try:
            ser.write(b"!\r")
            ser.flush()
        except serial.SerialException:
            pass
        print(f"[host abort] {exc}", file=sys.stderr)
        return 1
    finally:
        ser.close()
        log_file.close()


if __name__ == "__main__":
    raise SystemExit(main())
