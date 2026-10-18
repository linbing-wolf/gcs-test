# -*- coding: utf-8 -*-
"""
selftest_echo.py —— screen_echo（屏幕命令回显条）电脑端自检

跑法：
    python3 selftest_echo.py
不需要 MaixCAM2，也不需要 maix 包（screen_echo 对 maix 是可选的）。

覆盖：
  1. wrap(None) 安全；wrap 后 show() 会转发给真 display
  2. 回显条内容：RX / TX / 状态计数三行；行数可配 1..3
  3. 计数：set_rx/set_tx 自增、set_bad 生效
  4. 颜色：坏帧>0 变红、RUNNING 变蓝、IDLE 正常色
  5. 超长命令截断到 MAX_TEXT
  6. 图片没有 height()/width() 或它们抛异常时也不崩（退回 640x480）
  7. draw() 任何异常都不外抛
"""

import screen_echo as se

_checks = [0, 0]        # [总数, 失败数]


def check(name, cond, detail=""):
    _checks[0] += 1
    if not cond:
        _checks[1] += 1
        print("  [FAIL] %s  %s" % (name, detail))
    else:
        print("  [OK] %s" % name)


def group(name):
    print("== %s ==" % name)


# ----------------------------------------------------------------------
# 假对象
# ----------------------------------------------------------------------
class FakeImg:
    def __init__(self, h=480, w=640, h_mode="ok"):
        self.calls = []
        self._h, self._w, self._mode = h, w, h_mode

    def height(self):
        if self._mode == "raise":
            raise RuntimeError("no height")
        if self._mode == "missing":
            raise AttributeError("nope")
        return self._h

    def width(self):
        return self._w

    def draw_string(self, x, y, s, color, scale):
        self.calls.append(("str", x, y, s, color, scale))

    def draw_rect(self, x, y, w, h, color, th):
        self.calls.append(("rect", x, y, w, h, color, th))

    def texts(self):
        return [c[3] for c in self.calls if c[0] == "str"]

    def colors(self):
        return [c[4] for c in self.calls if c[0] == "str"]

    def rects(self):
        return [c for c in self.calls if c[0] == "rect"]


class FakeDisp:
    def __init__(self):
        self.shown = 0
        self.last = None

    def show(self, img):
        self.shown += 1
        self.last = img

    def width(self):
        return 480

    def height(self):
        return 640


def main():
    se.echo.reset()

    # ---------------------------------------------------------------- 1
    group("wrap / show 转发")
    check("wrap(None) 返回 None", se.wrap(None) is None)
    real = FakeDisp()
    d = se.wrap(real)
    img = FakeImg()
    d.show(img)
    check("show() 转发给真 display", real.shown == 1)
    check("show() 画了回显条", len(img.texts()) >= 1, str(img.texts()))
    check("width() 透传", d.width() == 480)
    check("height() 透传", d.height() == 640)

    # ---------------------------------------------------------------- 2
    group("三行内容")
    se.echo.reset()
    se.set_rx("C,1")
    se.set_tx("PICK,1,312,238")
    se.set_state("IDLE")
    lines = se._lines(n=3)
    check("行数=3", len(lines) == 3, str(lines))
    check("第1行是 RX", lines[0].startswith("RX: C,1"), lines[0])
    check("第2行是 TX", lines[1].startswith("TX: PICK,1,312,238"), lines[1])
    check("第3行含状态与计数",
          ("IDLE" in lines[2] and "rx=1" in lines[2] and "tx=1" in lines[2]
           and "bad=0" in lines[2]), lines[2])
    check("行数可配 1", len(se._lines(n=1)) == 1)
    check("行数可配 2", len(se._lines(n=2)) == 2)

    # R/T/Z 这类不读相机的命令也要有痕迹
    se.set_rx("R")
    check("R 也能回显", se._lines(n=1)[0] == "RX: R", se._lines(n=1)[0])
    se.set_rx("T,134+123+314+231")
    check("T 也能回显", se._lines(n=1)[0] == "RX: T,134+123+314+231", se._lines(n=1)[0])

    # ---------------------------------------------------------------- 3
    group("计数")
    se.echo.reset()
    for _ in range(3):
        se.set_rx("A")
    for _ in range(2):
        se.set_tx("START,1,320,240,120,80")
    se.set_bad(4)
    st = se.echo
    check("rx_count=3", st.rx_count == 3, str(st.rx_count))
    check("tx_count=2", st.tx_count == 2, str(st.tx_count))
    check("bad=4", st.bad == 4, str(st.bad))
    check("bad 写进第3行", "bad=4" in se._lines(n=3)[2], se._lines(n=3)[2])

    # ---------------------------------------------------------------- 4
    group("颜色")
    se.echo.reset()
    se.set_state("IDLE")
    img = FakeImg()
    se.draw(img, w=640, h=480, lines=3)
    cols = img.colors()
    check("IDLE 第3行=普通色", cols[2] == se.COLOR_FG, str(cols))
    se.set_state("RUNNING")
    img = FakeImg()
    se.draw(img, w=640, h=480, lines=3)
    check("RUNNING 第3行=蓝色", img.colors()[2] == se.COLOR_RUN, str(img.colors()))
    se.set_bad(1)
    img = FakeImg()
    se.draw(img, w=640, h=480, lines=3)
    check("坏帧>0 第3行=红色", img.colors()[2] == se.COLOR_BAD, str(img.colors()))
    se.set_bad(0)
    se.set_state("IDLE")

    # ---------------------------------------------------------------- 5
    group("截断与排版")
    se.echo.reset()
    se.set_rx("X" * 200)
    check("超长截断到 <=MAX_TEXT", len(se.echo.rx) <= se.MAX_TEXT, str(len(se.echo.rx)))
    check("截断有省略号", se.echo.rx.endswith("..."), se.echo.rx[-6:])
    se.set_rx("A\r\nB\tC")
    check("控制字符被压成空格", ("\r" not in se.echo.rx and "\n" not in se.echo.rx
                            and "\t" not in se.echo.rx), repr(se.echo.rx))

    img = FakeImg(h=480, w=640)
    se.draw(img, w=640, h=480, lines=3)
    rect = img.rects()[0]
    check("底色矩形铺满宽度", rect[1] == 0 and rect[3] == 640, str(rect))
    check("底色矩形贴底", rect[2] + rect[4] == 480, str(rect))
    ys = [c[2] for c in img.calls if c[0] == "str"]
    check("三行纵向排开且都在画面内",
          len(ys) == 3 and ys[0] < ys[1] < ys[2] and ys[2] < 480, str(ys))

    # ---------------------------------------------------------------- 6
    group("图片接口缺失/异常时也不崩")
    for mode in ("ok", "raise", "missing"):
        img = FakeImg(h=300, w=200, h_mode=mode)
        try:
            se.draw(img, lines=3)
            ok = True
            err = ""
        except Exception as e:
            ok = False
            err = str(e)
        check("height() 模式=%s 不抛异常" % mode, ok, err)
        if mode == "ok":
            check("按图片真实高度贴底", img.rects()[0][2] + img.rects()[0][4] == 300,
                  str(img.rects()))
        else:
            check("退回默认高度 480 贴底" % (), img.rects()[0][2] + img.rects()[0][4] == 480,
                  str(img.rects()))

    # ---------------------------------------------------------------- 7
    group("draw 的异常不外抛")
    class BadImg:
        def draw_rect(self, *a, **k):
            raise RuntimeError("no rect")

        def draw_string(self, *a, **k):
            raise RuntimeError("no string")

    try:
        se.draw(BadImg(), lines=3)
        ok = True
    except Exception as e:
        ok = False
        print(e)
    check("画不上去也不崩", ok)

    # ---------------------------------------------------------------- 汇总
    print("\n%d checks, %d failed" % (_checks[0], _checks[1]))
    print("[ALL PASS]" if _checks[1] == 0 else "[FAIL]")
    return 1 if _checks[1] else 0


if __name__ == "__main__":
    raise SystemExit(main())