# -*- coding: utf-8 -*-
"""
视觉标定取点工具

用法（在 tools 目录里打开 PowerShell）：
    python 像素取点.py                  # 弹出文件选择框，挑一张标定照片
    python 像素取点.py 照片.jpg          # 直接打开指定照片

操作：
    鼠标左键    取一个点，打印这个点在「原图」里的像素坐标 (x, y)
    D 键        切换 取点 / 测距 模式。测距模式下点两个点，
                打印像素距离，并弹框让你填这两点现实里相距多少毫米，
                自动算出 mm/像素
    R 键        清空所有点，重新来
    Q / Esc     退出

结果同时打印在屏幕上、并追加写入照片同目录下的「标定记录.txt」，
方便你攒够多个高度的数据去做线性拟合。

怎么用它对上参考工程的两个数：
    第 2 步（中心点 322/165）：把物块摆到「车已经到位、正好能抓」的位置，
        读物块中心的像素坐标，那就是中心点。
    第 3 步（mm/像素）：同一高度把物块平移一段已知距离（建议 200mm 以上），
        用 D 模式量出像素距离，算出来的 mm/像素 就是 bilichi。
        换几个高度各量一次，再拟合 k = a*高度 + b。

依赖：Pillow（能开 jpg、能自动缩放）。没装也能跑，但只能开 PNG/GIF：
    pip install pillow
"""

import math
import os
import sys

try:
    from PIL import Image, ImageTk
    HAS_PIL = True
except Exception:
    HAS_PIL = False

import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog

MAX_W = 1280
MAX_H = 860
MARK_COLOR = "#ff2020"   # 标记颜色（红色，深色图纸上也看得见）
FILE_TYPES = [("图片", "*.jpg *.jpeg *.png *.bmp *.gif"), ("所有文件", "*.*")]


def say(text):
    """往控制台打印；在没控制台的环境（pythonw）下安静跳过。"""
    try:
        print(text)
    except Exception:
        pass


class PointTool:
    def __init__(self, root, path):
        self.root = root
        self.path = path
        self.points = []          # 原图坐标（不是屏幕坐标）
        self.marks = []
        self.distance_mode = False
        self.logfile = os.path.join(os.path.dirname(path), "标定记录.txt")

        if HAS_PIL:
            self.image = Image.open(path).convert("RGB")
            self.ow, self.oh = self.image.size
            # 图太大就按比例缩小显示；记录坐标时再按比例换算回原图
            self.scale = min(1.0, MAX_W / float(self.ow), MAX_H / float(self.oh))
            dw = max(1, int(round(self.ow * self.scale)))
            dh = max(1, int(round(self.oh * self.scale)))
            self.photo = ImageTk.PhotoImage(self.image.resize((dw, dh)))
        else:
            self.photo = tk.PhotoImage(file=path)
            self.ow, self.oh = self.photo.width(), self.photo.height()
            self.scale = 1.0
            dw, dh = self.ow, self.oh

        root.title("像素取点 - %s  (%d x %d)" % (os.path.basename(path), self.ow, self.oh))
        self.canvas = tk.Canvas(root, width=dw, height=dh,
                                highlightthickness=0, cursor="crosshair")
        self.canvas.pack()
        self.canvas.create_image(0, 0, anchor="nw", image=self.photo)
        self.canvas.bind("<Button-1>", self.on_click)

        self.status = tk.Label(root, anchor="w", justify="left", font=("Consolas", 9))
        self.status.pack(fill="x")

        root.bind("r", self.reset)
        root.bind("R", self.reset)
        root.bind("d", self.toggle_distance)
        root.bind("D", self.toggle_distance)
        root.bind("q", lambda e: root.destroy())
        root.bind("Q", lambda e: root.destroy())
        root.bind("<Escape>", lambda e: root.destroy())

        self.log("=== 打开图片: %s  (%d x %d) ===" % (path, self.ow, self.oh))
        self.set_status()

    # ---------- 交互 ----------

    def on_click(self, event):
        ox = event.x / self.scale
        oy = event.y / self.scale
        self.points.append((ox, oy))
        idx = len(self.points)
        x, y = event.x, event.y

        self.marks.append(self.canvas.create_line(x - 8, y, x + 8, y, fill=MARK_COLOR))
        self.marks.append(self.canvas.create_line(x, y - 8, x, y + 8, fill=MARK_COLOR))
        self.marks.append(self.canvas.create_text(x + 7, y - 13, text=str(idx),
                                                  anchor="w", fill=MARK_COLOR))
        self.log("点%d: (%.1f, %.1f) px" % (idx, ox, oy))

        if self.distance_mode and idx % 2 == 0:
            self.measure()
        self.set_status()

    def measure(self):
        (x1, y1), (x2, y2) = self.points[-2], self.points[-1]
        dpx = math.hypot(x2 - x1, y2 - y1)
        self.marks.append(self.canvas.create_line(x1 * self.scale, y1 * self.scale,
                                                  x2 * self.scale, y2 * self.scale,
                                                  fill=MARK_COLOR, dash=(4, 2)))
        self.log("  像素距离 = %.2f px" % dpx)
        if dpx <= 0:
            return
        mm = simpledialog.askfloat(
            "实际距离",
            "这两点在现实中相距多少毫米？\n\n(像素距离 %.2f px)" % dpx,
            parent=self.root, minvalue=0.0001)
        if mm:
            self.log("  实际 %.1f mm  ->  mm/像素 = %.4f" % (mm, mm / dpx))

    def reset(self, event=None):
        for m in self.marks:
            self.canvas.delete(m)
        self.marks = []
        self.points = []
        self.set_status()

    def toggle_distance(self, event=None):
        self.distance_mode = not self.distance_mode
        self.set_status()

    def set_status(self):
        mode = "测距模式（点两个点）" if self.distance_mode else "取点模式"
        self.status.config(text=(
            "当前：%s    已取 %d 点\n"
            "左键=取点    D=切换取点/测距    R=清空    Q/Esc=退出\n"
            "结果另存到：%s" % (mode, len(self.points), self.logfile)))

    # ---------- 输出 ----------

    def log(self, text):
        say(text)
        try:
            with open(self.logfile, "a", encoding="utf-8") as f:
                f.write(text + "\n")
        except Exception as exc:
            say("(写记录失败: %s)" % exc)


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        path = filedialog.askopenfilename(title="选一张标定照片", filetypes=FILE_TYPES)
    if not path:
        return
    path = os.path.abspath(path)
    if not os.path.exists(path):
        say("找不到文件：%s" % path)
        return

    root = tk.Tk()
    try:
        PointTool(root, path)
    except Exception as exc:
        root.destroy()
        say("打不开这张图：%s" % exc)
        if not HAS_PIL:
            say("提示：没装 Pillow，只能打开 PNG / GIF。装一下就能开 jpg:")
            say("    pip install pillow")
        return
    root.mainloop()


if __name__ == "__main__":
    main()
