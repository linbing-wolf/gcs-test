# -*- coding: utf-8 -*-
"""
screen_echo.py —— 屏幕命令回显条（给现场排障用）

为什么要有它：
  视觉端是"被动服务"——单片机不发命令，它什么都不做、也不刷屏。于是：
    · 空闲时屏幕停在上一次任务的最后一帧，看着像卡死；
    · `R` / `T` / `Z` 这类不读相机的命令，屏幕上一丝痕迹都没有；
    · 从屏幕上完全看不出"单片机到底发进来没有、发的是什么、回的是什么"。
  这个模块在画面底部常驻一条回显，把上面几件事都摆出来：

      RX: C,1                    ← 最近收到的命令（含 R/T/Z）
      TX: PICK,1,312,238         ← 最近发出的回复
      IDLE  rx=12 tx=12 bad=0    ← 状态 + 收发计数 + 坏帧数

两种用法（main.py 已经接好，别处一般不用管）：
  1. 把 display 包一层：disp = screen_echo.wrap(disp)
     这样任务里原有的 disp.show(img) 会自动带上回显条，不用改 tasks.py。
  2. 事件发生时更新状态：set_rx() / set_tx() / set_state() / set_bad()

本模块不 import maix 也能加载（颜色常量退化为占位值），方便电脑端自检。
"""

# ----------------------------------------------------------------------
# 颜色：拿不到 maix.image 时退化为占位整数，保证电脑端能 import
# ----------------------------------------------------------------------
try:
    from maix import image as _image
except Exception:                                  # pragma: no cover
    _image = None


def _col(name, fallback):
    if _image is not None:
        return getattr(_image, name, fallback)
    return fallback


COLOR_BG   = _col("COLOR_BLACK", 0x000000)
COLOR_FG   = _col("COLOR_WHITE", 0xFFFFFF)
COLOR_RX   = _col("COLOR_YELLOW", 0xFFFF00)
COLOR_TX   = _col("COLOR_GREEN", 0x00FF00)
COLOR_RUN  = _col("COLOR_BLUE", 0x0000FF)
COLOR_BAD  = _col("COLOR_RED", 0xFF0000)
COLOR_DIM  = _col("COLOR_GRAY", 0x808080)

MAX_TEXT = 56          # 单条命令/回复最大显示长度
LINE_H   = 18          # 行高（scale=1）
PAD      = 4


def _clip(s):
    s = "" if s is None else str(s)
    s = s.replace("\r", " ").replace("\n", " ").replace("\t", " ")
    if len(s) > MAX_TEXT:
        s = s[:MAX_TEXT - 3] + "..."
    return s


class EchoState:
    """回显条的内容（全局一个实例，模块级 echo）"""

    def __init__(self):
        self.rx = ""
        self.tx = ""
        self.state = "BOOT"      # BOOT / IDLE / RUNNING
        self.rx_count = 0
        self.tx_count = 0
        self.bad = 0             # 坏帧数（CRC/帧尾/半帧）
        self.note = ""           # 可选的额外提示（例如"相机不可用"）

    def reset(self):
        self.__init__()


echo = EchoState()


# ----------------------------------------------------------------------
# 事件接口：main.py / tasks.py 调用
# ----------------------------------------------------------------------
def set_rx(raw):
    echo.rx = _clip(raw)
    echo.rx_count += 1


def set_tx(payload):
    echo.tx = _clip(payload)
    echo.tx_count += 1


def set_state(state):
    echo.state = str(state)


def set_bad(n):
    try:
        echo.bad = int(n or 0)
    except Exception:
        echo.bad = 0


def set_note(text):
    echo.note = _clip(text)


# ----------------------------------------------------------------------
# 文本生成（单独抽出来，方便电脑端自检）
# ----------------------------------------------------------------------
def _lines(state=None, n=3):
    st = state or echo
    out = ["RX: %s" % (st.rx if st.rx else "-")]
    if n >= 2:
        out.append("TX: %s" % (st.tx if st.tx else "-"))
    if n >= 3:
        line = "%s  rx=%d tx=%d bad=%d" % (st.state, st.rx_count, st.tx_count, st.bad)
        if st.note:
            line += "  " + st.note
        out.append(line)
    return out[:max(1, min(3, int(n)))]


def _line_color(idx, st):
    if idx == 0:
        return COLOR_RX
    if idx == 1:
        return COLOR_TX
    if st.bad > 0:
        return COLOR_BAD
    if str(st.state).upper() == "RUNNING":
        return COLOR_RUN
    if str(st.state).upper() in ("BOOT", "ERR"):
        return COLOR_DIM
    return COLOR_FG


def _img_size(img, w_fallback, h_fallback):
    w, h = w_fallback, h_fallback
    for name, cur in (("width", w), ("height", h)):
        fn = getattr(img, name, None)
        if callable(fn):
            try:
                v = int(fn())
                if v > 0:
                    if name == "width":
                        w = v
                    else:
                        h = v
            except Exception:
                pass
    return w, h


def draw(img, state=None, w=None, h=None, lines=None):
    """把回显条画到 img 底部。任何异常都不往外抛（画不上最多是没回显，不能影响任务）"""
    st = state or echo
    w_def, h_def, n = 640, 480, 3
    try:
        import config as _cfg
        w_def = int(getattr(_cfg, "W", 640))
        h_def = int(getattr(_cfg, "H", 480))
        n = int(getattr(_cfg, "SCREEN_ECHO_LINES", 3))
    except Exception:
        pass
    if w is None:
        w = w_def
    if h is None:
        h = h_def
    if lines is not None:
        n = int(lines)

    w, h = _img_size(img, w, h)
    texts = _lines(st, n)
    band = LINE_H * len(texts) + PAD * 2
    y0 = max(0, h - band)

    try:
        img.draw_rect(0, y0, w, band, COLOR_BG, -1)      # 底色（thickness=-1 填充）
    except Exception:
        try:
            img.draw_rect(0, y0, w, band, COLOR_BG, 1)
        except Exception:
            pass

    for i, text in enumerate(texts):
        try:
            img.draw_string(PAD + 2, y0 + PAD + i * LINE_H, text, _line_color(i, st), 1)
        except Exception:
            pass
    return len(texts)


# ----------------------------------------------------------------------
# display 包装：任务里的 disp.show(img) 会自动带上回显条
# ----------------------------------------------------------------------
class EchoDisplay:
    def __init__(self, real):
        self._real = real

    def show(self, img):
        try:
            draw(img)
        except Exception:
            pass
        return self._real.show(img)

    # width()/height()/其它 display 方法原样转发（触屏那边要用来算坐标）
    def __getattr__(self, name):
        return getattr(self._real, name)


def wrap(real):
    """real 为 None 时原样返回 None，方便 main.py 一行接好"""
    if real is None:
        return None
    return EchoDisplay(real)