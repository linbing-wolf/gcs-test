#!/usr/bin/env python3
"""Rotate in place N times (default 4x90 deg left) to isolate turn slip.

No translation commands are issued, so any net displacement comes from the
turn itself.
"""

import argparse
import pathlib
import sys
import time

import serial


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM9")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--turns", type=int, default=4)
    parser.add_argument("--direction", choices=("turnl", "turnr"), default="turnl")
    parser.add_argument("--imu-baud", type=int, default=460800)
    parser.add_argument("--imu-rate", type=int, default=200)
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

    def read_until(markers, timeout: float) -> str:
        nonlocal pending
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            chunk = ser.read(4096)
            if not chunk:
                continue
            text = chunk.decode("utf-8", "replace")
            emit(text)
            pending = (pending + text)[-8192:]
            for marker in markers:
                if marker in pending:
                    return marker
        return ""

    def command(text: str, markers, timeout: float = 5.0) -> str:
        nonlocal pending
        pending = ""
        emit(f"\n> {text}\n")
        ser.write((text + "\r").encode("ascii"))
        ser.flush()
        return read_until(markers, timeout)

    try:
        time.sleep(0.3)
        ser.reset_input_buffer()
        if args.imu_baud and args.imu_baud != 115200:
            command(f"imu baud {args.imu_baud}", ["[imu]"], 5.0)
            time.sleep(0.3)
        if args.imu_rate:
            command(f"imu rate {args.imu_rate}", ["[imu]"], 4.0)
            time.sleep(0.3)
        command("imutest clear", ["[imutest"], 5.0)
        command("imutest pid", ["[imutest"], 4.0)
        command("imutest", ["accel_age_ms"], 4.0)

        for i in range(1, args.turns + 1):
            emit(f"\n=== TURN {i}/{args.turns} {args.direction} 90 ===\n")
            pending = ""
            emit(f"> imutest {args.direction}\n")
            ser.write(f"imutest {args.direction}\r".encode("ascii"))
            ser.flush()
            hit = read_until(["[imutest accepted", "[imutest rejected", "FAULT"], 6.0)
            if hit != "[imutest accepted":
                raise RuntimeError(f"turn {i} not accepted: {hit!r}")
            pending = ""
            hit = read_until(["[turn closed-loop] DONE_BY_IMU", "FAULT"], 60.0)
            if not hit.startswith("[turn closed-loop] DONE_BY_IMU"):
                raise RuntimeError(f"turn {i} failed: {hit!r}")
            time.sleep(1.0)

        emit("\n=== restore IMU 50Hz / 115200 ===\n")
        command("imu rate 50", ["[imu]"], 4.0)
        time.sleep(0.2)
        command("imu baud 115200", ["[imu]"], 5.0)
        command("imutest", ["accel_age_ms"], 4.0)
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
