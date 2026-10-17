# -*- coding: utf-8 -*-
"""capture_calib.py —— 标定拍图（板上 CLI 版，保存【干净帧】）

用法：python3 capture_calib.py --out /maixapp/share/picture/calib --interval_ms 0
  --interval_ms 0 = 点屏保存；>0 = 自动连拍
"""
import argparse
import os
import time

from maix import camera, display, image, app

try:
    from maix import touchscreen
except Exception:
    touchscreen = None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/maixapp/share/picture/calib")
    ap.add_argument("--w", type=int, default=640)
    ap.add_argument("--h", type=int, default=480)
    ap.add_argument("--interval_ms", type=int, default=0)
    ap.add_argument("--skip", type=int, default=10)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    cam = camera.Camera(args.w, args.h)
    cam.skip_frames(args.skip)
    disp = display.Display()

    ts = None
    if args.interval_ms == 0 and touchscreen is not None:
        try:
            ts = touchscreen.TouchScreen()
        except Exception as e:
            print("[!] 触屏不可用，退化为自动连拍:", e)

    n, last, prev = 0, 0.0, False
    print("保存干净帧 -> %s" % args.out)
    while not app.need_exit():
        img = cam.read()
        now = time.time() * 1000.0
        save = False
        if args.interval_ms > 0:
            if now - last >= args.interval_ms:
                save = True
        elif ts is not None:
            try:
                _x, _y, pressed = ts.read()
            except Exception:
                pressed = False
            if pressed and not prev:
                save = True
            prev = pressed
        if ts is None and args.interval_ms == 0 and now - last >= 1000:
            save = True
        if save:
            img.save(os.path.join(args.out, "calib_%04d.jpg" % n))   # ← 保存干净帧
            n += 1
            last = now
        # 预览叠加（不进图）
        for k in (1, 2):
            img.draw_line(args.w * k // 3, 0, args.w * k // 3, args.h, image.COLOR_BLUE, 1)
            img.draw_line(0, args.h * k // 3, args.w, args.h * k // 3, image.COLOR_BLUE, 1)
        img.draw_string(4, 4, "calib saved=%d" % n, image.COLOR_GREEN, 2)
        disp.show(img)
    print("完成:", n, "张")


if __name__ == "__main__":
    main()