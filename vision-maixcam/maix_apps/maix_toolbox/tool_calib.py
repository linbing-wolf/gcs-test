# -*- coding: utf-8 -*-
"""tool_calib.py —— 标定拍图（工具集内嵌）

⚠️ 保存的是【干净帧】：先 save 原始画面，再画预览叠加层（九宫格/按钮不会进保存的图）。

顶栏：AUTO 键(MANUAL⇄AUTO ON) · `-`/`+` 调速(0.1~2.0s，默认1.0s) · BACK 回菜单
保存目录：优先 /maixapp/share/picture/calib，否则退回 /root/calib
"""
import os
import time

from maix import image, app

from ui import hit, topbar

MIN_IV, MAX_IV, STEP = 0.1, 2.0, 0.1


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


def run(cam, disp, touch, w, h):
    out = out_dir("calib")
    y = 44
    exit_r = [w - 120, y, 108, 36]
    auto_r = [w - 240, y, 112, 36]
    plus_r = [w - 286, y, 40, 36]
    minus_r = [w - 330, y, 40, 36]
    n = 0
    auto = False
    interval = 1.0
    last = 0.0
    print("标定拍图 → %s（保存干净帧）" % out)

    while not app.need_exit():
        img = cam.read()

        # ---------- 先决定保存，并在画任何东西之前存【干净帧】 ----------
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
            elif 0 <= ix < w and 0 <= iy < h:
                save = True
        if auto and time.time() - last >= interval:
            save = True
        if save:
            img.save(os.path.join(out, "calib_%04d.jpg" % n))   # ← 干净帧
            n += 1
            last = time.time()

        # ---------- 以下只画预览叠加层，不进保存的图 ----------
        topbar(img, w, "CALIB  saved=%d" % n)
        for k in (1, 2):
            img.draw_line(w * k // 3, 0, w * k // 3, h, image.COLOR_BLUE, 1)
            img.draw_line(0, h * k // 3, w, h * k // 3, image.COLOR_BLUE, 1)
        ac = image.COLOR_GREEN if auto else image.COLOR_GRAY
        img.draw_rect(auto_r[0], auto_r[1], auto_r[2], auto_r[3], ac, 2)
        img.draw_string(auto_r[0] + 8, auto_r[1] + 12,
                        ("AUTO %.1fs" % interval) if auto else "MANUAL", ac, 1)
        for r, txt in ((minus_r, "-"), (plus_r, "+")):
            img.draw_rect(r[0], r[1], r[2], r[3], image.COLOR_WHITE, 2)
            img.draw_string(r[0] + 15, r[1] + 8, txt, image.COLOR_WHITE, 2)
        img.draw_rect(exit_r[0], exit_r[1], exit_r[2], exit_r[3], image.COLOR_RED, 2)
        img.draw_string(exit_r[0] + 20, exit_r[1] + 8, "BACK", image.COLOR_RED, 2)

        disp.show(img)