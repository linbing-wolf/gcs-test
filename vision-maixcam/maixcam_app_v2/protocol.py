# -*- coding: utf-8 -*-
"""
protocol.py —— 新流程串口协议（变长帧），MaixCAM2 ⇄ STM32

为什么改协议：
  旧工程用固定 9 字节帧（0xFF + 9 + 0xFE），但新任务码是 4 组三位数、共 15 字符，
  一帧装不下，所以改成变长帧。

帧格式（双向一致）：
  ┌──────┬──────┬─────┬───────────────┬──────┬──────┐
  │ 0xAA │ 0x55 │ LEN │ payload[LEN]  │ CRC8 │ 0x0D │
  └──────┴──────┴─────┴───────────────┴──────┴──────┘
  LEN     = payload 字节数（0~255）
  CRC8    = 对 LEN + payload 做的 CRC-8（poly 0x07），防半帧/粘连
  帧尾     = 0x0D

payload 文本约定：
  下位机→上位机：[命令]              或  [命令],[参数]      例如 "C,1" / "T,134+123+314+231"
  上位机→下位机：一串 ASCII，见文件末尾的 pack_* 构造函数。

命令字表（下位机→上位机）：
  A            识别启停区              → START,<区号>,x,y,w,h
  T,<任务码>    下发任务码(扫码模块给的) → TASK,<第一批颜色>,<第二批颜色>
  C,<颜色码>    原料区三选一：找该色物料 → PICK,color,x,y
  D            粗加工区圆盘中心/对齐     → DISC,x,y,dx,ok
  E            暂存区圆盘中心/对齐       → DISC,x,y,dx,ok
  F            码垛中心               → STACK,x,y     (高度由下位机自己算)
  G            前方障碍物              → OBST,x,y,w,h,dist / CLEAR
  R            复位/重新握手            → READY
  Z            结束                    → OK
失败统一回：NONE[,<tag>]    异常回：ERR,<msg>

视觉端→下位机的主动指令（仅圆盘对齐时使用）：
  MOVE,<dx>          水平微调：dx = 水平偏差像素（符号见 ALIGN_INVERT_D/E）；
  MOVEY,<dy>         垂直微调：dy = 垂直偏差像素（符号见 ALIGN_INVERT_DY_D/E）；
                     单片机执行后不用回帧，视觉端等 ALIGN_SETTLE_MS 再重新测量。
                     两轴同时超差时，视觉端会连发 MOVE 和 MOVEY。

一问一答模型（重要）：
  视觉端是**阻塞单线程**：跑一个任务期间不主动读命令（最长 TASK_TIMEOUT_MS）。
  为防下位机干等，任务循环里每帧会「看一眼」串口：
    · 收到 R  → 立即中断当前任务，回 READY
    · 收到其它命令 → 回一帧 BUSY（表示"正在忙，别重复发"），任务继续
  因此下位机的超时必须 > TASK_TIMEOUT_MS，并把 BUSY 当作"还在算"而不是失败。

握手约定：
  视觉端上电会发一次 READY 作为「我就绪了」的提示；但**正式握手由下位机发 R 触发**
  （视觉端可能比下位机先启动，先发的那次 READY 会丢）。
"""

from maix import uart, pinmap

from timing import ticks_ms, sleep_ms
import serial_log

# 串口参数唯一来源：config.py
try:
    import config as _cfg
    UART_DEV = _cfg.UART_DEV
    UART_BAUDRATE = _cfg.UART_BAUDRATE
    PIN_TX = _cfg.PIN_TX
    PIN_RX = _cfg.PIN_RX
    UART_FUNC_TX = _cfg.UART_FUNC_TX
    UART_FUNC_RX = _cfg.UART_FUNC_RX
except Exception as _e:
    print("[!] 读 config.py 失败(%s)，用内置默认 UART4(A21/A22)" % _e)
    UART_DEV, UART_BAUDRATE = "/dev/ttyS4", 115200
    PIN_TX, PIN_RX = "A21", "A22"
    UART_FUNC_TX, UART_FUNC_RX = "UART4_TX", "UART4_RX"

# ============================== 帧常量 ==============================
FRAME_H1 = 0xAA
FRAME_H2 = 0x55
FRAME_TAIL = 0x0D
MAX_PAYLOAD = 255

# 命令字
CMD_START = "A"
CMD_TASK = "T"
CMD_PICK = "C"
CMD_DISC_ROUGH = "D"
CMD_DISC_TEMP = "E"
CMD_STACK = "F"
CMD_FORWARD = "G"
CMD_RESET = "R"
CMD_END = "Z"


def crc8(data):
    """CRC-8，poly 0x07，init 0x00（纯 Python，无依赖）"""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def encode_frame(payload):
    """把字符串/bytes 打包成一帧"""
    if isinstance(payload, str):
        payload = payload.encode("utf-8")
    payload = bytes(payload)[:MAX_PAYLOAD]
    body = bytes([len(payload)]) + payload
    return bytes([FRAME_H1, FRAME_H2]) + body + bytes([crc8(body), FRAME_TAIL])


class FrameParser:
    """流式拆帧：喂任意字节，吐出完整 payload（bytes）列表

    半帧超时：如果缓冲里有残留、且超过 frame_timeout_ms 没再收到字节，
    就丢弃这段残留（否则断线/重启时的一条半帧会把后面的正常帧吞掉）。
    """

    def __init__(self, frame_timeout_ms=200):
        self.buf = bytearray()
        self.frame_timeout_ms = max(0, int(frame_timeout_ms or 0))
        self._last_ms = 0
        # 计数器：屏幕回显条 / 现场排障用（bad=坏帧总数）
        self.ok = 0
        self.bad = 0

    def feed(self, data):
        now = ticks_ms()
        # 半帧超时清理（只可能发生在两次 feed 之间）
        if (self.buf and self.frame_timeout_ms > 0
                and (now - self._last_ms) > self.frame_timeout_ms):
            print("[!] 半帧超时(%dms)，丢弃 %d 字节" %
                  (now - self._last_ms, len(self.buf)))
            self.buf = bytearray()
            self.bad += 1
        self._last_ms = now
        self.buf += bytes(data)
        out = []
        while True:
            i = self.buf.find(bytes([FRAME_H1, FRAME_H2]))
            if i < 0:
                # 没找到帧头：只留最后 1 字节（可能是半个 0xAA）
                if len(self.buf) > 1:
                    del self.buf[:-1]
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < 3:
                break
            ln = self.buf[2]
            total = 3 + ln + 2          # 头2 + LEN1 + payload + CRC1 + TAIL1
            if len(self.buf) < total:
                break
            payload = bytes(self.buf[3:3 + ln])
            crc = self.buf[3 + ln]
            tail = self.buf[3 + ln + 1]
            if tail == FRAME_TAIL and crc == crc8(bytes([ln]) + payload):
                out.append(payload)
                self.ok += 1
            else:
                print("[!] 坏帧丢弃: crc/tail 不符")
                self.bad += 1
            del self.buf[:total]
        return out


def _pin_now(pin):
    """读取引脚当前的复用功能名（读不到返回空串）"""
    try:
        return str(pinmap.get_pin_function(pin))
    except Exception:
        return ""


class VisionLink:
    """与 STM32 的串口链路：发变长帧 + 收命令帧"""

    def __init__(self, dev=UART_DEV, baud=UART_BAUDRATE, use_pinmap=True):
        if use_pinmap:
            for pin, func in ((PIN_TX, UART_FUNC_TX), (PIN_RX, UART_FUNC_RX)):
                try:
                    ret = pinmap.set_pin_function(pin, func)
                    code = ret if isinstance(ret, bool) else getattr(ret, "code", lambda: -1)()
                    ok = (code == 0) if not isinstance(ret, bool) else ret
                    after = _pin_now(pin)
                    # 判据：设置后引脚"当前功能"是不是目标功能。
                    # 很多板子对已配置过的引脚会返回非 0，但功能其实已生效。
                    if after and after == func:
                        print("[自检] pinmap %s -> %s OK（当前=%s%s）"
                              % (pin, func, after,
                                 "" if ok else "，返回码%s 但已生效" % code))
                    else:
                        print("[!] pinmap %s -> %s 失败！返回码=%s，当前=%s（UART 可能不通）"
                              % (pin, func, code, after if after else "读不到"))
                except Exception as e:
                    print("[!] pinmap %s -> %s 异常: %s（继续，但 UART 可能不通）" % (pin, func, e))
        try:
            self.ser = uart.UART(dev, baud)
        except Exception as e:
            # 启动重试：现场插拔/系统服务还没让出串口时很有用
            retries = 3
            delay = 1000
            try:
                import config as _c2
                retries = max(1, int(getattr(_c2, "STARTUP_RETRY", 3)))
                delay = max(0, int(getattr(_c2, "STARTUP_RETRY_DELAY_MS", 1000)))
            except Exception:
                pass
            self.ser = None
            for attempt in range(1, retries + 1):
                print("[!] 打开串口失败(%d/%d): %s @ %d -> %s"
                      % (attempt, retries, dev, baud, e))
                if attempt < retries and delay:
                    sleep_ms(delay)
                if attempt < retries:
                    try:
                        self.ser = uart.UART(dev, baud)
                        break
                    except Exception as e2:
                        e = e2
            if self.ser is None:
                print("[!] 串口仍不可用，进程退出（此时没有通道能通知下位机）")
                try:
                    from maix import uart as _u
                    print("[!] 可用串口: %s" % (_u.list_devices(),))
                except Exception:
                    pass
                raise e
        self.dev = dev
        self.baud = baud
        try:
            _fto = int(getattr(_cfg, "FRAME_TIMEOUT_MS", 200))
        except Exception:
            _fto = 200
        self.parser = FrameParser(_fto)
        self._rxq = []          # 已解出的命令，待消费
        try:
            import config as _lc
            _on = bool(getattr(_lc, "SERIAL_LOG_ON", True))
            _dir = getattr(_lc, "SERIAL_LOG_DIR", "/root/yyb_mover_serial")
            _raw = bool(getattr(_lc, "SERIAL_LOG_RAW", True))
        except Exception:
            _on, _dir, _raw = True, "/root/yyb_mover_serial", True
        self.slog = serial_log.SerialLog(_on, _dir, _raw)
        self.slog.event("LINK OPEN %s @ %d" % (dev, baud))

    # ---------------- 发送 ----------------
    def send_payload(self, payload):
        pkt = encode_frame(payload)
        self.ser.write(pkt)
        self.slog.tx(payload, pkt)
        if _verbose():
            print("[发送] %r  <- %s" % (pkt.hex(" "), payload))
        return pkt

    # ---------------- 接收 ----------------
    def poll_command(self):
        """非阻塞取一条命令字符串；没有则 None"""
        if self._rxq:
            return self._rxq.pop(0)
        try:
            data = self.ser.read()
        except Exception:
            return None
        if data:
            self.slog.rx_raw(data)
            for p in self.parser.feed(data):
                s = p.decode("utf-8", "ignore")
                self._rxq.append(s)
                self.slog.rx(s)
        if self._rxq:
            return self._rxq.pop(0)
        return None

    def close(self):
        self.slog.event("LINK CLOSE")
        self.slog.close()
        try:
            self.ser.close()
        except Exception:
            pass


def _verbose():
    try:
        import config as c
        return getattr(c, "VERBOSE", True)
    except Exception:
        return True


def preflight():
    """启动自检：串口设备 + 引脚功能"""
    print("-" * 60)
    print("[自检] 串口 %s @ %d | 引脚 %s=%s, %s=%s"
          % (UART_DEV, UART_BAUDRATE, PIN_TX, UART_FUNC_TX, PIN_RX, UART_FUNC_RX))
    try:
        from maix import uart as _u
        devs = _u.list_devices()
        print("[自检] 可用串口: %s" % (devs,))
        if UART_DEV not in devs:
            print("[!] 警告: %s 不在可用列表" % UART_DEV)
    except Exception as e:
        print("[自检] 列串口失败: %s" % e)
    try:
        from maix import pinmap as _p
        for pin, func in ((PIN_TX, UART_FUNC_TX), (PIN_RX, UART_FUNC_RX)):
            try:
                now = ""
                try:
                    now = _p.get_pin_function(pin)
                except Exception:
                    pass
                print("[自检] %s 支持: %s | 当前: %s"
                      % (pin, _p.get_pin_functions(pin), now or "?"))
            except Exception as e:
                print("[自检] 查询 %s 失败: %s" % (pin, e))
    except Exception as e:
        print("[自检] import pinmap 失败: %s" % e)
    print("-" * 60)


# ======================================================================
# 命令解析：把 payload 拆成 (命令, 参数)
# ======================================================================
def parse_command(payload):
    """'C,1' -> ('C', '1')；'D' -> ('D', '')"""
    payload = (payload or "").strip()
    if not payload:
        return "", ""
    if "," in payload:
        c, arg = payload.split(",", 1)
        return c.strip(), arg.strip()
    return payload[:1], payload[1:].strip()


# ======================================================================
# 回复帧构造器（上位机 → 下位机）
# ======================================================================
def pack_start(zone, x, y, w, h):
    """启停区：区号(1/2) + 蓝色启停区中心与尺寸（看不到时坐标全 0）"""
    return "START,%d,%d,%d,%d,%d" % (int(zone), int(x), int(y), int(w), int(h))


def pack_task(g1, g3):
    """任务码解析结果：第一批颜色顺序、第二批颜色顺序（G2/G4 是位置，无需回）"""
    return "TASK,%s,%s" % (g1, g3)


def pack_pick(color, x, y):
    return "PICK,%s,%d,%d" % (str(color), int(x), int(y))


def pack_disc(x, y, dx=0, ok=1):
    """圆盘中心 + 水平偏差(dx=x-W/2) + 是否已对齐(ok)"""
    return "DISC,%d,%d,%d,%d" % (int(x), int(y), int(dx), int(ok))


def pack_move(dx):
    """【视觉端 → 下位机】请求水平微调：dx 像素（目标在画面上的水平偏差）

    具体是前进/后退还是左右平移，由下位机自己映射（本车是前后动）。
    方向反了就把 config.ALIGN_INVERT_D/E 置 True。
    """
    return "MOVE,%d" % int(dx)


def pack_movey(dy):
    """【视觉端 → 下位机】请求垂直微调：dy 像素（目标在画面上的垂直偏差）

    方向反了就把 config.ALIGN_INVERT_DY_D/E 置 True。
    """
    return "MOVEY,%d" % int(dy)


def pack_stack(x, y):
    """码垛：只给中心。高度由 STM32 自己按物料/层数算，视觉不测高度"""
    return "STACK,%d,%d" % (int(x), int(y))


def pack_obstacle(x, y, w, h, dist=0):
    """前方有障碍物：中心 + 尺寸 + 估算距离(mm, 0=未知)"""
    return "OBST,%d,%d,%d,%d,%d" % (int(x), int(y), int(w), int(h), int(dist))


def pack_clear():
    """前方无障碍物"""
    return "CLEAR"


def pack_busy():
    """任务执行中又收到命令：表示"正在忙，别重复发"，任务会继续跑"""
    return "BUSY"


# 合法颜色码（红1 黄2 蓝3 绿4）
COLOR_CODES = ("1", "2", "3", "4")


def is_color_code(s):
    return str(s).strip() in COLOR_CODES


def pack_ready():
    return "READY"


def pack_ok():
    return "OK"


def pack_none(tag=""):
    return "NONE" + (("," + tag) if tag else "")


def pack_error(msg):
    return "ERR,%s" % str(msg)


# ======================================================================
# 任务码解析
# ======================================================================
def parse_task_code(code):
    """解析 15 字符任务码 'G1+G2+G3+G4'

    G1/G3 = 颜色+顺序（每位必须是 1~4），G2/G4 = 放置位置（编码规则由 STM32 定，这里不校验）。
    返回 dict：g1..g4（各 3 字符串）或 None（格式/颜色非法）。
    """
    code = (code or "").strip()
    parts = code.split("+")
    if len(parts) != 4:
        return None
    for p in parts:
        if len(p) != 3 or not p.isdigit():
            return None
    # 语义校验：G1/G3 的每一位都必须是合法颜色码（红1 黄2 蓝3 绿4）
    for p in (parts[0], parts[2]):
        if any(ch not in COLOR_CODES for ch in p):
            return None
    return {"g1": parts[0], "g2": parts[1], "g3": parts[2], "g4": parts[3],
            "raw": code}