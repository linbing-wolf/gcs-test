# -*- coding: utf-8 -*-
"""
main.py —— MAIX TOOLBOX（工具集 APP 入口）

一个菜单，装到设备上就能干这四件事（不用回电脑）：
  - Calib Capture   标定拍图（棋盘格，九宫格参考线 + 点屏保存）
  - Dataset Capture  数据集采集（选类别 + 点屏存帧）
  - Disable Maix Comm 关闭系统通信协议，让出 UART4 (A21/A22)
  - Diagnostics      设备自检（MaixPy 版本 / 原生算子 / 串口 / 引脚）

点每个工具里的 BACK 回菜单；按设备 FUNC 键退出整个 APP。
"""
from maix import camera, display, image, app

import light
import ui
import tool_calib
import tool_color
import tool_dataset
import tool_comm
import tool_diag

W, H = 640, 480
MENU = [
    ("calib", "Calib Capture"),
    ("color", "Color Tune"),
    ("dataset", "Dataset Capture"),
    ("comm", "Disable Maix Comm"),
    ("diag", "Diagnostics"),
    ("exit", "EXIT"),
]


def main():
    cam = camera.Camera(W, H)
    cam.skip_frames(10)
    disp = display.Display()
    touch = ui.Touch(W, H)
    light.init()

    bw, bh, gap = 520, 50, 8
    x0 = (W - bw) // 2
    y0 = 64
    rects = {code: [x0, y0 + i * (bh + gap), bw, bh] for i, (code, _) in enumerate(MENU)}
    light_r = [W - 176, 4, 168, 38]

    while not app.need_exit():
        img = cam.read()
        img.draw_rect(0, 0, W, 46, image.COLOR_BLUE, -1)
        img.draw_string(12, 12, "MAIX TOOLBOX", image.COLOR_WHITE, 2)
        lc = image.COLOR_YELLOW if light.is_on() else image.COLOR_GRAY
        img.draw_rect(light_r[0], light_r[1], light_r[2], light_r[3], lc, 2)
        img.draw_string(light_r[0] + 8, light_r[1] + 12, light.label(), lc, 1)
        for code, label in MENU:
            r = rects[code]
            color = image.COLOR_RED if code == "exit" else image.COLOR_GREEN
            img.draw_rect(r[0], r[1], r[2], r[3], color, 2)
            img.draw_string(r[0] + 18, r[1] + (r[3] - 20) // 2, label, color, 2)

        ix, iy, clicked = touch.poll(disp)
        if clicked:
            code = None
            for c, _ in MENU:
                if ui.hit(ix, iy, rects[c]):
                    code = c
                    break
            if ui.hit(ix, iy, light_r):
                light.toggle()
            elif code == "exit":
                app.set_exit_flag(True)
            elif code == "calib":
                tool_calib.run(cam, disp, touch, W, H)
            elif code == "color":
                tool_color.run(cam, disp, touch, W, H)
            elif code == "dataset":
                tool_dataset.run(cam, disp, touch, W, H)
            elif code == "comm":
                tool_comm.run(cam, disp, touch, W, H)
            elif code == "diag":
                tool_diag.run(cam, disp, touch, W, H)

        disp.show(img)


if __name__ == "__main__":
    main()