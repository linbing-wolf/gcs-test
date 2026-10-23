# -*- coding: utf-8 -*-
"""
disc_debug.py —— D/E 圆盘对齐调试记录器

每次 D/E 任务在 DISC_DEBUG_DIR 下建一个目录，例如：

    /root/yyb_mover_data/disc_debug/20261019-153012-D/
        000_start.jpg
        001_iter0_before.jpg
        002_final_ok.jpg
        meta.jsonl          # 每帧检测数据（JSON Lines）
        summary.json        # 任务结果/参数

只保留最近 DISC_DEBUG_KEEP_TASKS 次任务，自动删旧目录。
存图失败/目录不可写时自动关闭抓图，不影响正常对齐。
"""

import os
import json
import time
import shutil

try:
    from timing import ticks_ms
except Exception:
    def ticks_ms():
        return int(time.time() * 1000)


class DiscDebug:
    def __init__(self, tag, base_dir, max_frames=24, quality=85,
                 keep_tasks=20, min_interval_ms=100, enabled=True):
        self.tag = tag
        self.base_dir = base_dir
        self.max_frames = max(1, int(max_frames))
        self.quality = max(1, min(100, int(quality)))
        # <=0 表示不自动清理，保留全部任务目录
        self.keep_tasks = int(keep_tasks)
        self.min_interval_ms = max(0, int(min_interval_ms))
        self.enabled = bool(enabled)
        self.dir = None
        self.frames = 0
        self.last_img_ms = -10 ** 9
        self.t0 = ticks_ms()
        self.meta_fp = None

    # ---------------- 生命周期 ----------------
    def start(self):
        if not self.enabled:
            return False
        try:
            os.makedirs(self.base_dir, exist_ok=True)
            stamp = time.strftime("%Y%m%d-%H%M%S")
            path = os.path.join(self.base_dir, "%s-%s" % (stamp, self.tag))
            n = 1
            while os.path.exists(path):
                n += 1
                path = os.path.join(self.base_dir, "%s-%s-%d" % (stamp, self.tag, n))
            os.makedirs(path, exist_ok=True)
            self.dir = path
            self.meta_fp = open(os.path.join(path, "meta.jsonl"), "w", encoding="utf-8")
            self._write({"type": "start", "tag": self.tag, "t": 0})
            self._prune()
            print("[调试] 抓图目录: %s" % path)
            return True
        except Exception as e:
            print("[调试] 初始化失败，关闭抓图: %s" % e)
            self.enabled = False
            self.dir = None
            return False

    def close(self, result="", extra=None):
        if not self.enabled or self.dir is None:
            return
        summary = {
            "type": "summary",
            "tag": self.tag,
            "result": str(result),
            "frames_saved": self.frames,
            "duration_ms": ticks_ms() - self.t0,
        }
        if extra:
            summary.update(extra)
        self._write(summary)
        try:
            with open(os.path.join(self.dir, "summary.json"), "w", encoding="utf-8") as f:
                json.dump(summary, f, ensure_ascii=False, indent=2)
        except Exception:
            pass
        try:
            if self.meta_fp is not None:
                self.meta_fp.close()
        except Exception:
            pass
        self.meta_fp = None

    def __del__(self):
        try:
            if getattr(self, "meta_fp", None) is not None:
                self.close("")
        except Exception:
            pass

    # ---------------- 记录 ----------------
    def data(self, fields, phase="frame"):
        """只记 JSON 数据，不存图（每帧都调，开销很小）"""
        if not self.enabled or self.dir is None:
            return
        self._write(self._payload(fields, phase, None))

    def snap(self, img, fields, phase="frame", force=False):
        """记 JSON + 存一张 JPEG（关键节点调；force=True 忽略限流/帧数上限）"""
        if not self.enabled or self.dir is None:
            return None
        now = ticks_ms()
        if not force:
            if self.frames >= self.max_frames:
                self._write(self._payload(fields, phase, None))
                return None
            if self.frames > 0 and (now - self.last_img_ms) < self.min_interval_ms:
                self._write(self._payload(fields, phase, None))
                return None

        name = "%03d_%s.jpg" % (self.frames, _safe_name(phase))
        path = os.path.join(self.dir, name)
        ok = False
        try:
            if hasattr(img, "save"):
                img.save(path, quality=self.quality)
                ok = os.path.exists(path) and os.path.getsize(path) > 0
        except Exception as e:
            print("[调试] 存图失败 %s: %s" % (path, e))
        if ok:
            self.frames += 1
            self.last_img_ms = now
        self._write(self._payload(fields, phase, name if ok else None))
        return name if ok else None

    # ---------------- 内部 ----------------
    def _payload(self, fields, phase, img_name):
        payload = {
            "type": "frame",
            "tag": self.tag,
            "phase": str(phase),
            "t": ticks_ms() - self.t0,
            "img": img_name,
        }
        if fields:
            for k, v in fields.items():
                if k not in payload:
                    payload[k] = v
        return payload

    def _write(self, obj):
        if self.meta_fp is None:
            return
        try:
            self.meta_fp.write(json.dumps(obj, ensure_ascii=False, default=str) + "\n")
            self.meta_fp.flush()
        except Exception:
            pass

    def _prune(self):
        if self.keep_tasks <= 0:
            return
        try:
            entries = []
            for name in os.listdir(self.base_dir):
                p = os.path.join(self.base_dir, name)
                if os.path.isdir(p):
                    entries.append((os.path.getmtime(p), p))
            entries.sort()
            while len(entries) > self.keep_tasks:
                _, old = entries.pop(0)
                shutil.rmtree(old, ignore_errors=True)
                print("[调试] 清理旧抓图: %s" % old)
        except Exception:
            pass


def _safe_name(s):
    out = []
    for ch in str(s):
        if ch.isalnum() or ch in ("-", "_", "."):
            out.append(ch)
        else:
            out.append("_")
    return "".join(out)[:48] or "frame"
