# -*- coding: utf-8 -*-
"""
calib_capture —— 标定拍图助手（MaixCAM 标准 APP）

九宫格参考线 + 保存计数；右上控件：
  - EXIT 退出
  - AUTO 键：MANUAL(点屏保存) ⇄ AUTO ON(自动连拍)
  - `-` / `+` 键：调自动连拍间隔（0.1~2.0s，步进 0.1）
"""
import os
import time

from maix import camera, display, image, app

try:
    from maix import touchscreen
except Exception:
    touchscreen = None

OUT = "/root/calib"
W, H = 640, 480
MIN_IV, MAX_IV, STEP = 0.1, 2.0, 0.1


def _clamp(v):
    return round(max(MIN_IV, min(MAX_IV, v)), 1)


class Touch:
    def __init__(self):
        self.ts = None
        self.prev = False
        if touchscreen is not None:
            try:
                self.ts = touchscreen.TouchScreen()
                print("[touch] ok")
            except Exception as e:
                print("[touch] 不可用:", e)

    def poll(self, disp):
        if self.ts is None:
            return None, None, False
        try:
            x, y, pressed = self.ts.read()
        except Exception:
            return None, None, False
        if not pressed:
            self.prev = False
            return None, None, False
        try:
            p = image.resize_map_pos(W, H, disp.width(), disp.height(),
                                     image.Fit.FIT_CONTAIN, x, y)
            ix, iy = int(p[0]), int(p[1])
        except Exception:
            ix, iy = int(x), int(y)
        clicked = not self.prev
        self.prev = True
        return ix, iy, clicked


def hit(px, py, r):
    return r[0] <= px < r[0] + r[2] and r[1] <= py < r[1] + r[3]


def main():
    os.makedirs(OUT, exist_ok=True)
    cam = camera.Camera(W, H)
    cam.skip_frames(10)
    disp = display.Display()
    ts = Touch()

    exit_r = [W - 120, 8, 108, 40]
    auto_r = [W - 240, 8, 112, 40]
    plus_r = [W - 286, 8, 40, 40]
    minus_r = [W - 330, 8, 40, 40]
    n = 0
    auto = False
    interval = 1.0
    last = 0.0
    print("保存到 %s；AUTO 连拍 / -+ 调速 / MANUAL 点屏；EXIT 退出。" % OUT)

    while not app.need_exit():
        img = cam.read()
        for k in (1, 2):
            img.draw_line(W * k // 3, 0, W * k // 3, H, image.COLOR_BLUE, 1)
            img.draw_line(0, H * k // 3, W, H * k // 3, image.COLOR_BLUE, 1)

        img.draw_string(6, 6, "calib saved=%d" % n, image.COLOR_GREEN, 2)
        ac = image.COLOR_GREEN if auto else image.COLOR_GRAY
        img.draw_rect(auto_r[0], auto_r[1], auto_r[2], auto_r[3], ac, 2)
        img.draw_string(auto_r[0] + 8, auto_r[1] + 14,
                        ("AUTO %.1fs" % interval) if auto else "MANUAL", ac, 1)
        for r, txt in ((minus_r, "-"), (plus_r, "+")):
            img.draw_rect(r[0], r[1], r[2], r[3], image.COLOR_WHITE, 2)
            img.draw_string(r[0] + 15, r[1] + 10, txt, image.COLOR_WHITE, 2)
        img.draw_rect(exit_r[0], exit_r[1], exit_r[2], exit_r[3], image.COLOR_RED, 2)
        img.draw_string(exit_r[0] + 22, exit_r[1] + 10, "EXIT", image.COLOR_RED, 2)

        save = False
        ix, iy, clicked = ts.poll(disp)
        if clicked:
            if hit(ix, iy, exit_r):
                app.set_exit_flag(True)
            elif hit(ix, iy, auto_r):
                auto = not auto
                last = 0.0
            elif hit(ix, iy, minus_r):
                interval = _clamp(interval - STEP)
            elif hit(ix, iy, plus_r):
                interval = _clamp(interval + STEP)
            elif 0 <= ix < W and 0 <= iy < H:
                save = True

        if auto and time.time() - last >= interval:
            save = True
        if save:
            img.save(os.path.join(OUT, "calib_%04d.jpg" % n))
            n += 1
            last = time.time()
            if n % 10 == 0:
                print("已保存 %d 张" % n)

        disp.show(img)

    print("退出，共 %d 张 -> %s" % (n, OUT))


if __name__ == "__main__":
    main()