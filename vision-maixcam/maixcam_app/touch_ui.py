# -*- coding: utf-8 -*-
"""
touch_ui.py —— 临时调试入口：在 MaixCAM2 屏幕上点按钮直接触发任务

为什么需要它：
  正常流程是 STM32 发命令字 → MaixCAM2 执行。但调试时 STM32 那侧还没改好，
  所以加一个触屏入口，不用等 STM32 就能单独跑每个任务、对着画面调 LAB 阈值。

特点：
  1) 任务跑完后【不会自动退出】，会一直识别并把结果画在屏幕上 —— 方便调阈值；
     点同一个按钮或点 EXIT 键退出该任务。
  2) 不管有没有真的发串口，config.py 里 TOUCH_UI_SEND_UART 决定要不要把结果发给 STM32。
  3) 触摸坐标到图像坐标的换算统一走 image.resize_map_pos()，
     避免"图片大小和屏幕不一样，坐标对不上"这个经典坑。

布局（在 640x480 的图像上）：
  第 1 行： 2  |  a  |  3  |  4  |  d      ← 物块定标/两圈循环/边界
  第 2 行： 7  |  8  |  9  |  0  |  1      ← 色环粗精/码垛粗精/二维码(空)
  右上角： EXIT 按钮
"""

from maix import touchscreen, image, app


class TouchUI:
    """屏幕按钮面板：把触摸点映射成任务命令字"""

    def __init__(self, img_w, img_h, buttons, exit_label="EXIT"):
        self.img_w = img_w
        self.img_h = img_h
        self.buttons = buttons          # [(code, label), ...]
        self.ts = None
        self.avail = False
        self.last_code = None           # 各按钮最近一次的运行结果（用于显示）
        self.status = ""
        self._pressed = False           # 上一帧是否按下（用于检沿，避免一次点击触发多次）
        self._last_enter = [0, 0]       # 本帧按下的坐标（给框架画个点）
        try:
            self.ts = touchscreen.TouchScreen()
            self.avail = True
            print("[触屏] TouchScreen 初始化成功")
        except Exception as e:
            print("[触屏] 初始化失败，触屏入口不可用: %s" % e)
            print("       不影响串口模式，STM32 发命令照样能跑")

    # ------------------------------------------------------------------
    # 坐标换算：触摸坐标(屏幕) -> 图像坐标
    # 默认 disp.show(img) 用 FIT_CONTAIN：等比缩放 + 居中 + 黑边，
    # 所以必须做这个映射，否则按钮点不准。
    # ------------------------------------------------------------------
    def map_touch(self, x, y, screen_w, screen_h, fit=None):
        if fit is None:
            fit = image.Fit.FIT_CONTAIN
        try:
            p = image.resize_map_pos(self.img_w, self.img_h, screen_w, screen_h,
                                     fit, x, y)
            return int(p[0]), int(p[1])
        except Exception:
            # 兜底：等比缩放 + 居中（与 FIT_CONTAIN 一致）
            s = min(screen_w / float(self.img_w), screen_h / float(self.img_h))
            ox = (screen_w - self.img_w * s) / 2.0
            oy = (screen_h - self.img_h * s) / 2.0
            return int((x - ox) / s), int((y - oy) / s)

    # ------------------------------------------------------------------
    # 布局：把按钮排成两行 + 右下角大按钮
    # ------------------------------------------------------------------
    def layout(self):
        out = {}
        n = len(self.buttons)
        if n == 0:
            return out
        bw, bh, gap = 118, 92, 6
        x0, y0 = 6, self.img_h - 2 * (bh + gap) - 6
        for i, (code, label) in enumerate(self.buttons):
            row, col = divmod(i, 5)
            x = x0 + col * (bw + gap)
            y = y0 + row * (bh + gap)
            out[code] = [x, y, bw, bh]
        # 右下角：EXIT / INFO
        out["__info__"] = [self.img_w - 172, 6, 166, 40]
        out["__exit__"] = [self.img_w - 172, 52, 166, 44]
        return out

    @staticmethod
    def hit(px, py, rect):
        x, y, w, h = rect
        return x <= px < x + w and y <= py < y + h

    def find_button(self, px, py, rects):
        """返回被点中的按钮 code（含 __exit__ / __info__），没点中返回 None"""
        for code, rect in rects.items():
            if self.hit(px, py, rect):
                return code
        return None

    # ------------------------------------------------------------------
    # 读取一次触摸，返回 (image_x, image_y, clicked_code)
    # clicked_code 只在"按下"的那一帧返回一次（检沿），避免长按重复触发
    # ------------------------------------------------------------------
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
        if not self._pressed:               # 只在按下的第一帧算点击
            self._pressed = True
            clicked = self.find_button(ix, iy, rects)
        return ix, iy, clicked

    # ------------------------------------------------------------------
    # 绘制按钮面板
    # ------------------------------------------------------------------
    def draw(self, img, rects, active=None):
        for code, rect in rects.items():
            x, y, w, h = rect
            if code == "__exit__":
                color = image.COLOR_RED
                label = "EXIT"
            elif code == "__info__":
                color = image.COLOR_GRAY
                label = "INFO"
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
            # 标签分两行显示，避免太长溢出
            parts = label.split(" ", 1)
            ty = y + 14 if len(parts) == 1 else y + 8
            img.draw_string(x + 8, ty, parts[0], color, 1)
            if len(parts) > 1:
                img.draw_string(x + 8, ty + 18, parts[1], color, 1)

        # 显示最近一次结果
        if self.status:
            img.draw_string(6, 6, self.status[:44], image.COLOR_WHITE, 1)
        if self.avail and self._pressed:
            img.draw_circle(self._last_enter[0], self._last_enter[1], 6,
                            image.COLOR_PURPLE, 2)


# 默认按钮表：code 就是发给自己的命令字（和 STM32 用的一套）
# 标签一律用 ASCII 英文缩写 —— MaixPy 的 draw_string 默认字体只支持 ASCII，
# 写中文会显示成乱码（要中文得自己指定 TTF 字体文件，调试阶段没必要）。
# 缩写对照：BLK=物块 RING=色环 STACK=码垛 LOOP=两圈循环 C=粗(coarse) F=精(fine) EDGE=边界线
DEFAULT_BUTTONS = [
    ("2", "2 BLK-A3"),
    ("a", "a BLK-A3"),
    ("3", "3 LOOP-1"),
    ("4", "4 LOOP-2"),
    ("d", "d EDGE"),
    ("7", "7 RING-C"),
    ("8", "8 RING-F"),
    ("9", "9 STACK-C"),
    ("0", "0 STACK-F"),
    ("6", "6 READY"),
]
