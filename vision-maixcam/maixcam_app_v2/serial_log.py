# -*- coding: utf-8 -*-
"""
serial_log.py —— 全串口 RX/TX 日志（持久化在 APP 目录之外）

每次 APP 启动在 SERIAL_LOG_DIR 下新建一个文件：
    /root/yyb_mover_serial/serial_20261020-163012.log

每行：
    2026-10-20 16:30:12.345 [+00012.345s] [RX] D | AA 55 01 44 2E 0D
    2026-10-20 16:30:12.480 [+00012.480s] [TX] MOVE,-400 | AA 55 ...
    2026-10-20 16:30:12.480 [+00012.480s] [RX-RAW] 6 bytes | AA 55 ...

同时追加会话索引：
    /root/yyb_mover_serial/sessions.jsonl
    每次启动/退出各一行，记录版本、PID、启动时间、运行时长。

日志目录在 /root 下，不随 APP 安装/升级被覆盖。
"""

import json
import os
import time


def app_version():
    """读取同目录 app.yaml 的 version 字段；失败时返回 unknown。"""
    try:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "app.yaml")
        with open(path, "r", encoding="utf-8") as f:
            for line in f:
                if line.startswith("version:"):
                    return line.split(":", 1)[1].strip()
    except Exception:
        pass
    return "unknown"


class SerialLog:
    def __init__(self, enabled=True, directory="/root/yyb_mover_serial", raw=True):
        self.enabled = bool(enabled)
        self.raw = bool(raw)
        self.dir = directory
        self.fp = None
        self.path = None
        self.version = app_version()
        self.start_wall = time.time()
        self.t0 = time.monotonic()
        if not self.enabled:
            return
        try:
            os.makedirs(self.dir, exist_ok=True)
            stamp = time.strftime("%Y%m%d-%H%M%S")
            self.path = os.path.join(self.dir, "serial_%s.log" % stamp)
            self.fp = open(self.path, "a", encoding="utf-8")
            self.event("APP START version=%s pid=%d" % (self.version, os.getpid()))
            self._journal("start")
            print("[串口日志] %s (version=%s)" % (self.path, self.version))
        except Exception as e:
            print("[串口日志] 初始化失败，关闭日志: %s" % e)
            self.enabled = False
            self.fp = None

    def _uptime(self):
        return time.monotonic() - self.t0

    def _write(self, tag, text, raw=None):
        if not self.enabled or self.fp is None:
            return
        try:
            ts = time.strftime("%Y-%m-%d %H:%M:%S")
            ms = int((time.time() % 1) * 1000)
            line = "%s.%03d [+%09.3fs] [%s] %s" % (
                ts, ms, self._uptime(), tag, text)
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

    def _journal(self, event):
        """追加一条会话记录，便于统计每次运行时长/版本。"""
        if not self.enabled or not self.dir:
            return
        try:
            rec = {
                "event": str(event),
                "version": self.version,
                "pid": os.getpid(),
                "wall": time.strftime("%Y-%m-%d %H:%M:%S"),
                "uptime_s": round(self._uptime(), 3),
            }
            path = os.path.join(self.dir, "sessions.jsonl")
            with open(path, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
                f.flush()
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
                up = self._uptime()
                self.event("APP EXIT uptime=%.3fs version=%s"
                           % (up, self.version))
                self._journal("exit")
                self.fp.close()
            except Exception:
                pass
            self.fp = None
