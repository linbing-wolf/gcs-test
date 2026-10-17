# -*- coding: utf-8 -*-
"""capture_dataset.py —— 数据集采集（板上 CLI 版，保存【干净帧】）

用法：python3 capture_dataset.py --name red --interval_ms 0
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
    ap.add_argument("--out", default="/maixapp/share/picture/dataset")
    ap.add_argument("--name", default="")
    ap.add_argument("--w", type=int, default=640)
    ap.add_argument("--h", type=int, default=480)
    ap.add_argument("--interval_ms", type=int, default=0)
    ap.add_argument("--skip", type=int, default=10)
    args = ap.parse_args()

    name = args.name.strip() or input("类别名: ").strip()
    save_dir = os.path.join(args.out, name)
    os.makedirs(save_dir, exist_ok=True)
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
    print("保存干净帧 -> %s" % save_dir)
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
            img.save(os.path.join(save_dir, "%s_%04d.jpg" % (name, n)))   # ← 干净帧
            n += 1
            last = now
        img.draw_string(4, 4, "%s saved=%d" % (name, n), image.COLOR_GREEN, 2)
        disp.show(img)
    print("完成:", n, "张 ->", save_dir)


if __name__ == "__main__":
    main()