# -*- coding: utf-8 -*-
"""
serial_log.py —— 全串口 RX/TX 日志

每次 APP 启动在 SERIAL_LOG_DIR 下新建一个文件：
    /root/yyb_mover_serial/serial_20261020-163012.log

每行：
    2026-10-20 16:30:12.345 [RX] D | AA 55 01 44 2E 0D
    2026-10-20 16:30:12.480 [TX] MOVE,-400 | AA 55 08 4D 4F 56 45 2C 2D 34 30 ...
    2026-10-20 16:30:12.480 [RX-RAW] 6 bytes | AA 55 01 44 2E 0D

目的：现场出问题时，可以把一次运行的全部串口收发（含原始帧）拉回来逐帧对。
"""

import os
import time


class SerialLog:
    def __init__(self, enabled=True, directory="/root/yyb_mover_serial", raw=True):
        self.enabled = bool(enabled)
        self.raw = bool(raw)
        self.dir = directory
        self.fp = None
        self.path = None
        if not self.enabled:
            return
        try:
            os.makedirs(self.dir, exist_ok=True)
            stamp = time.strftime("%Y%m%d-%H%M%S")
            self.path = os.path.join(self.dir, "serial_%s.log" % stamp)
            self.fp = open(self.path, "a", encoding="utf-8")
            self.event("APP START")
            print("[串口日志] %s" % self.path)
        except Exception as e:
            print("[串口日志] 初始化失败，关闭日志: %s" % e)
            self.enabled = False
            self.fp = None

    def _write(self, tag, text, raw=None):
        if not self.enabled or self.fp is None:
            return
        try:
            ts = time.strftime("%Y-%m-%d %H:%M:%S")
            ms = int((time.time() % 1) * 1000)
            line = "%s.%03d [%s] %s" % (ts, ms, tag, text)
            if self.raw and raw:
                if isinstance(raw, (bytes, bytearray)):
                    hx = bytes(raw).hex(" ")
                else:
                    hx = str(raw)
                if hx:
                    line += " | " + hx
            self.fp.write(line + "\n")
            self.fp.flush()
        except Exception:
            pass

    def event(self, msg):
        self._write("EVENT", msg)

    def rx_raw(self, data):
        self._write("RX-RAW", "%d bytes" % len(data), data)

    def rx(self, payload):
        self._write("RX", payload)

    def tx(self, payload, raw=None):
        self._write("TX", payload, raw)

    def close(self):
        if self.fp is not None:
            try:
                self.event("APP EXIT")
                self.fp.close()
            except Exception:
                pass
            self.fp = None
