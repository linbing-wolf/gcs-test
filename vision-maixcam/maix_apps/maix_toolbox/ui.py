# -*- coding: utf-8 -*-
"""ui.py —— 工具集共用的触摸 + 绘制辅助（MaixCAM 1:1 触屏坐标）"""
from maix import image

try:
    from maix import touchscreen
except Exception:
    touchscreen = None


class Touch:
    def __init__(self, w, h):
        self.w, self.h = w, h
        self.ts = None
        self.prev = False
        if touchscreen is not None:
            try:
                self.ts = touchscreen.TouchScreen()
                print("[touch] ok")
            except Exception as e:
                print("[touch] 不可用:", e)

    def poll(self, disp):
        """返回 (ix, iy, clicked)；clicked 只在按下第一帧为 True"""
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
            p = image.resize_map_pos(self.w, self.h, disp.width(), disp.height(),
                                     image.Fit.FIT_CONTAIN, x, y)
            ix, iy = int(p[0]), int(p[1])
        except Exception:
            ix, iy = int(x), int(y)
        clicked = not self.prev
        self.prev = True
        return ix, iy, clicked


def hit(px, py, r):
    return r[0] <= px < r[0] + r[2] and r[1] <= py < r[1] + r[3]


def button(img, r, label, color=image.COLOR_WHITE, scale=2):
    img.draw_rect(r[0], r[1], r[2], r[3], color, 2)
    img.draw_string(r[0] + 14, r[1] + (r[3] - 16 * scale) // 2, label, color, scale)


def topbar(img, w, title):
    img.draw_rect(0, 0, w, 40, image.COLOR_BLUE, -1)
    img.draw_string(10, 10, title, image.COLOR_WHITE, 2)
    img.draw_line(w // 3, 0, w // 3, img.height(), image.COLOR_GRAY, 1)
    img.draw_line(2 * w // 3, 0, 2 * w // 3, img.height(), image.COLOR_GRAY, 1)