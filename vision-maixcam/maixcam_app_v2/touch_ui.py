# -*- coding: utf-8 -*-
"""
touch_ui.py —— 触屏调试入口（临时，默认关）。点屏幕按钮直接触发任务，不用等 STM32。

【调试完记得把 config.TOUCH_UI 改回 False】——开着时不会读串口命令！
"""

from maix import touchscreen, image

# code -> (显示名, 命令载荷)。code 就是点下去要执行的命令字（含参数）。
DEFAULT_BUTTONS = [
    ("A",     "A START"),
    ("C,1",   "C RED"),
    ("C,2",   "C YEL"),
    ("C,3",   "C BLU"),
    ("C,4",   "C GRN"),
    ("D",     "D ROUGH"),
    ("E",     "E TEMP"),
    ("F",     "F STACK"),
]

JOBS = {code: (label, code) for code, label in DEFAULT_BUTTONS}


class TouchUI:
    def __init__(self, img_w, img_h, buttons, exit_label="EXIT"):
        self.img_w = img_w
        self.img_h = img_h
        self.buttons = buttons
        self.status = ""
        self._pressed = False
        self._last_enter = [0, 0]
        self.ts = None
        self.avail = False
        try:
            self.ts = touchscreen.TouchScreen()
            self.avail = True
            print("[触屏] TouchScreen 初始化成功")
        except Exception as e:
            print("[触屏] 初始化失败（不影响串口模式）: %s" % e)

    def map_touch(self, x, y, screen_w, screen_h, fit=None):
        if fit is None:
            fit = image.Fit.FIT_CONTAIN
        try:
            p = image.resize_map_pos(self.img_w, self.img_h, screen_w, screen_h, fit, x, y)
            return int(p[0]), int(p[1])
        except Exception:
            s = min(screen_w / float(self.img_w), screen_h / float(self.img_h))
            ox = (screen_w - self.img_w * s) / 2.0
            oy = (screen_h - self.img_h * s) / 2.0
            return int((x - ox) / s), int((y - oy) / s)

    def layout(self):
        out = {}
        n = len(self.buttons)
        if n == 0:
            return out
        bw, bh, gap = 118, 92, 6
        x0, y0 = 6, self.img_h - 2 * (bh + gap) - 6
        for i, (code, label) in enumerate(self.buttons):
            row, col = divmod(i, 4)
            out[code] = [x0 + col * (bw + gap), y0 + row * (bh + gap), bw, bh]
        out["__info__"] = [self.img_w - 172, 6, 166, 40]
        out["__exit__"] = [self.img_w - 172, 52, 166, 44]
        return out

    @staticmethod
    def hit(px, py, rect):
        x, y, w, h = rect
        return x <= px < x + w and y <= py < y + h

    def find_button(self, px, py, rects):
        for code, rect in rects.items():
            if self.hit(px, py, rect):
                return code
        return None

    def poll(self, screen_w, screen_h, rects):
        if not self.avail:
            return None, None, None
        try:
            x, y, pressed = self.ts.read()
        except Exception:
            return None, None, None
        if not pressed:
            self._pressed = False
            return None, None, None
        ix, iy = self.map_touch(x, y, screen_w, screen_h)
        self._last_enter = [ix, iy]
        clicked = None
        if not self._pressed:
            self._pressed = True
            clicked = self.find_button(ix, iy, rects)
        return ix, iy, clicked

    def draw(self, img, rects, active=None):
        for code, rect in rects.items():
            x, y, w, h = rect
            if code == "__exit__":
                color, label = image.COLOR_RED, "EXIT"
            elif code == "__info__":
                color, label = image.COLOR_GRAY, "INFO"
            else:
                color = image.COLOR_YELLOW if code == active else image.COLOR_GREEN
                label = None
                for c, lb in self.buttons:
                    if c == code:
                        label = lb
                        break
            if label is None:
                label = str(code)
            img.draw_rect(x, y, w, h, color, 2)
            parts = label.split(" ", 1)
            ty = y + 14 if len(parts) == 1 else y + 8
            img.draw_string(x + 8, ty, parts[0], color, 1)
            if len(parts) > 1:
                img.draw_string(x + 8, ty + 18, parts[1], color, 1)
        if self.status:
            img.draw_string(6, 6, self.status[:44], image.COLOR_WHITE, 1)