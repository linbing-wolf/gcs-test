#!/usr/bin/env python3
"""Cross test on the PB0/PB1 debug UART: rotate then strafe, no forward drive.

Each leg strafes 1 m with imutest l, then turns left 90 deg. Four legs cover
four world directions and should return near the start. Fail-stop on any fault.
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
    parser.add_argument("--distance", type=int, default=1000)
    parser.add_argument("--rpm", type=int, default=160)
    parser.add_argument("--imu-baud", type=int, default=460800)
    parser.add_argument("--imu-rate", type=int, default=200)
    parser.add_argument("--legs", type=int, default=4)
    parser.add_argument("--no-extra", action="store_true",
                        help="disable accelerometer distance extension for a clean 1 m leg")
    parser.add_argument("--trim", default="",
                        help="flat fwd/left tenths per leg, e.g. '8 27 27 -8 ...'")
    parser.add_argument("--shift-timeout", type=float, default=40.0)
    parser.add_argument("--turn-timeout", type=float, default=60.0)
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

    def command(text: str, timeout: float = 4.0) -> bool:
        nonlocal pending
        pending = ""
        emit(f"\n> {text}\n")
        ser.write((text + "\r").encode("ascii"))
        ser.flush()
        return bool(read_until(["[imutest", "[imu]", "DONE", "FAULT"], timeout))

    try:
        time.sleep(0.3)
        ser.reset_input_buffer()
        if args.imu_baud and args.imu_baud != 115200:
            if not command(f"imu baud {args.imu_baud}", 5.0):
                raise RuntimeError("imu baud: no response")
            time.sleep(0.3)
        if args.imu_rate:
            if not command(f"imu rate {args.imu_rate}", 4.0):
                raise RuntimeError("imu rate: no response")
            time.sleep(0.3)
        if not command("imutest clear", 5.0):
            raise RuntimeError("imutest clear: no response")
        command("imutest pid", 4.0)
        command(f"imutest rpm {args.rpm}", 4.0)
        command(f"imutest dist {args.distance}", 4.0)
        if args.no_extra:
            command("imutest extra 0", 4.0)
        probe = command("imutest", 4.0)
        if not probe:
            raise RuntimeError("imutest status: no response")
        if "accel_age_ms" not in pending:
            raise RuntimeError(f"unexpected status: {pending.strip()[-160:]}")

        for leg in range(1, args.legs + 1):
            trims = [int(item) for item in args.trim.replace(",", " ").split()]
            if len(trims) >= 2:
                idx = ((leg - 1) * 2) % len(trims)
                command(f"imutest trim {trims[idx]} {trims[idx + 1]}", 4.0)
            emit(f"\n=== LEG {leg} lateral {args.distance}mm ===\n")
            pending = ""
            emit("> imutest l\n")
            ser.write(b"imutest l\r")
            ser.flush()
            hit = read_until(["[imutest accepted", "[imutest rejected", "FAULT"], 6.0)
            if hit != "[imutest accepted":
                if hit == "[imutest rejected":
                    raise RuntimeError(f"leg {leg} strafe rejected")
                if hit == "FAULT":
                    raise RuntimeError(f"leg {leg} strafe fault")
                raise RuntimeError(f"leg {leg} strafe did not start")
            pending = ""
            hit = read_until(
                ["DONE_BY_IMU_ENCODER travel=LEFT", "[imutest rejected", "FAULT"],
                args.shift_timeout)
            if hit != "DONE_BY_IMU_ENCODER travel=LEFT":
                raise RuntimeError(f"leg {leg} strafe failed/timed out: {hit!r}")
            time.sleep(0.8)

            emit(f"\n=== LEG {leg} turn left 90 ===\n")
            pending = ""
            emit("> imutest turnl\n")
            ser.write(b"imutest turnl\r")
            ser.flush()
            hit = read_until(["[imutest accepted", "[imutest rejected", "FAULT"], 6.0)
            if hit != "[imutest accepted":
                raise RuntimeError(f"leg {leg} turn not accepted: {hit!r}")
            pending = ""
            hit = read_until(
                ["[turn closed-loop] DONE_BY_IMU", "[imutest rejected", "FAULT"],
                args.turn_timeout)
            if not hit.startswith("[turn closed-loop] DONE_BY_IMU"):
                raise RuntimeError(f"leg {leg} turn failed/timed out: {hit!r}")
            time.sleep(0.8)

        emit("\n=== restore IMU 50Hz / 115200 ===\n")
        command("imutest trim 0 0", 4.0)
        command("imutest extra 1", 4.0)
        command("imu rate 50", 4.0)
        time.sleep(0.2)
        command("imu baud 115200", 5.0)
        command("imutest", 4.0)
        return 0
    except Exception as exc:
        emit(f"\n[host abort] {exc}\n")
        try:
            ser.write(b"!\r")
            ser.flush()
        except serial.SerialException:
            pass
        print(f"[host abort] {exc}", file=sys.stderr)
        try:
            time.sleep(0.3)
            ser.reset_input_buffer()
            command("imu rate 50", 4.0)
            command("imu baud 115200", 5.0)
        except Exception as restore_exc:
            print(f"[restore failed] {restore_exc}", file=sys.stderr)
        return 1
    finally:
        ser.close()
        log_file.close()


if __name__ == "__main__":
    raise SystemExit(main())
