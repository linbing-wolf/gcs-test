# -*- coding: utf-8 -*-
"""tool_dataset.py —— 数据集采集（工具集内嵌）

⚠️ 保存的是【干净帧】：先 save 原始画面，再画预览叠加层（顶栏/类别按钮不进保存的图）。

顶栏：AUTO 键(MANUAL⇄AUTO ON) · `-`/`+` 调速(0.1~2.0s，默认1.0s) · BACK 回菜单
保存目录：优先 /maixapp/share/picture/dataset/<类别>，否则退回 /root/dataset/<类别>
"""
import os
import time

from maix import image, app

from ui import hit, topbar

MIN_IV, MAX_IV, STEP = 0.1, 2.0, 0.1
CLASSES = ["red", "yellow", "blue", "green", "background", "other"]


def out_dir(sub):
    for base in ("/maixapp/share/picture", "/maixapp/share", "/root"):
        p = os.path.join(base, sub)
        try:
            os.makedirs(p, exist_ok=True)
            return p
        except Exception:
            continue
    return "/root/" + sub


def _clamp(v):
    return round(max(MIN_IV, min(MAX_IV, v)), 1)


def _next_index(d, cls):
    """扫描已有 cls_*.jpg，返回下一个不会覆盖的编号"""
    n = 0
    try:
        for name in os.listdir(d):
            if not (name.startswith(cls + "_") and name.endswith(".jpg")):
                continue
            stem = name[len(cls) + 1:-4]
            if stem.isdigit():
                n = max(n, int(stem) + 1)
    except Exception:
        pass
    return n


def _count_existing(d, cls):
    """已有样本张数（用于界面显示，编号有空洞也不虚报）"""
    n = 0
    try:
        for name in os.listdir(d):
            if name.startswith(cls + "_") and name.endswith(".jpg"):
                n += 1
    except Exception:
        pass
    return n


def run(cam, disp, touch, w, h):
    root = out_dir("dataset")
    bw = w // len(CLASSES)
    bh = 52
    btns = {c: [i * bw, h - bh, bw - 2, bh] for i, c in enumerate(CLASSES)}
    y = 44
    exit_r = [w - 120, y, 108, 36]
    auto_r = [w - 240, y, 112, 36]
    plus_r = [w - 286, y, 40, 36]
    minus_r = [w - 330, y, 40, 36]
    counts = {c: _next_index(os.path.join(root, c), c) for c in CLASSES}
    saved = {c: _count_existing(os.path.join(root, c), c) for c in CLASSES}
    cur = CLASSES[0]
    auto = False
    interval = 1.0
    last = 0.0
    print("数据集采集 → %s（保存干净帧）" % root)

    while not app.need_exit():
        img = cam.read()

        # ---------- 先决定保存，存【干净帧】 ----------
        save = False
        ix, iy, clicked = touch.poll(disp)
        if clicked:
            if hit(ix, iy, exit_r):
                return
            elif hit(ix, iy, auto_r):
                auto = not auto
                last = 0.0
            elif hit(ix, iy, minus_r):
                interval = _clamp(interval - STEP)
            elif hit(ix, iy, plus_r):
                interval = _clamp(interval + STEP)
            else:
                switched = False
                for c, r in btns.items():
                    if hit(ix, iy, r):
                        cur = c
                        switched = True
                        break
                if (not switched and 44 <= iy < h - bh
                        and 0 <= ix < w):
                    save = True
        if auto and time.time() - last >= interval:
            save = True
        if save:
            d = os.path.join(root, cur)
            os.makedirs(d, exist_ok=True)
            n = counts[cur]
            path = os.path.join(d, "%s_%04d.jpg" % (cur, n))
            while os.path.exists(path):
                n += 1
                path = os.path.join(d, "%s_%04d.jpg" % (cur, n))
            try:
                img.save(path)   # ← 干净帧
                if os.path.exists(path) and os.path.getsize(path) > 0:
                    counts[cur] = n + 1
                    saved[cur] += 1
                    last = time.time()
                else:
                    print("保存失败（空文件）: %s" % path)
            except Exception as e:
                print("保存失败 %s: %s" % (path, e))

        # ---------- 预览叠加层（不进保存的图） ----------
        topbar(img, w, "%s  saved=%d" % (cur, saved[cur]))
        ac = image.COLOR_GREEN if auto else image.COLOR_GRAY
        img.draw_rect(auto_r[0], auto_r[1], auto_r[2], auto_r[3], ac, 2)
        img.draw_string(auto_r[0] + 8, auto_r[1] + 12,
                        ("AUTO %.1fs" % interval) if auto else "MANUAL", ac, 1)
        for r, txt in ((minus_r, "-"), (plus_r, "+")):
            img.draw_rect(r[0], r[1], r[2], r[3], image.COLOR_WHITE, 2)
            img.draw_string(r[0] + 15, r[1] + 8, txt, image.COLOR_WHITE, 2)
        img.draw_rect(exit_r[0], exit_r[1], exit_r[2], exit_r[3], image.COLOR_RED, 2)
        img.draw_string(exit_r[0] + 20, exit_r[1] + 8, "BACK", image.COLOR_RED, 2)
        for c, r in btns.items():
            color = image.COLOR_YELLOW if c == cur else image.COLOR_WHITE
            img.draw_rect(r[0] + 1, r[1] + 1, r[2], r[3], color, 2)
            img.draw_string(r[0] + 6, r[1] + 6, c[:6], color, 1)
            img.draw_string(r[0] + 6, r[1] + 24, str(saved[c]), color, 1)

        disp.show(img)