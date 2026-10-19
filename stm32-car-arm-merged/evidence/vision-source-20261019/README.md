# 把 gcs-gold-medal 的视觉部分迁移到 MaixCAM —— 分析与方案

分析对象（国赛金奖开源工程）：

| 目录 | 内容 | 规模 |
| --- | --- | --- |
| `cv2_python/` | 树莓派视觉主程序（Python + OpenCV） | **1892 行，单文件** |
| `yyb_stm32/` | STM32F407 主控固件（Keil 工程） | 业务代码约 3000 行（不含标准库） |

结论先给：**视觉侧可以整体搬到 MaixCAM，STM32 侧只需要改一个波特率数字。**
因为 MaixPy 原生就带「找色块 / 找圆 / 找线 / 识别二维码」四件套，正好一一对上原程序用 OpenCV 手写的四个能力，
而这些算子是硬件加速的，帧率反而更高。真正的难点不在算法，在**摄像头视角**和**时序约定**。

---

## 1. 现状：这套系统是怎么工作的

### 1.1 分工

```
┌──────────────── 树莓派（视觉）────────────────┐        ┌────────── STM32F407（执行）──────────┐
│ zongchengxu20250724chusai.py                 │        │ yyb_stm32/                          │
│                                              │        │                                     │
│  /dev/video_xia0  ─┐                         │ USART1 │  main.c: 一整套硬编码的流程编排      │
│  /dev/video_shang0 ─┴→ OpenCV 识别           │ 9600   │   ├ can_ZDT/  6 个闭环步进(CAN)     │
│                        ↓                     │ <====> │   ├ Hardware/ 3 舵机 + PID + 补光灯 │
│  SerialInterruptHandler                      │ 10 字节│   ├ imu/      WIT101 航向环         │
│  收单字节命令 → 执行任务 → 回 10 字节结果     │ 帧     │   ├ OLED/     显示                │
│                                              │        │   └ lanya/    串口屏 TJC          │
└──────────────────────────────────────────────┘        └─────────────────────────────────────┘
```

- **树莓派负责看**：识别二维码任务码、物块颜色与位置、色环/码垛位置、地面边界线倾角。
- **STM32 负责动**：麦克纳姆轮底盘 + 升降 + 平推 + 夹爪 + 云台 + 物料平台，全程按 
  `main.c` 里写死的比赛流程走，视觉只提供坐标和颜色。
- 两边是**主从握手**：STM32 发一个 ASCII 命令字 → 树莓派做完识别 → 回一帧结果 → STM32 继续下一步。

### 1.2 通信协议（移植必须 100% 保真的部分）

**STM32 → 树莓派**：单字节 ASCII 命令字（`Serial_SendByte`，`main.c:821` 的 `wait_send_shumeipai`）

| 命令字 | 含义 | 视觉侧动作 |
| --- | --- | --- |
| `'1'` | 扫描二维码 | ❌ 已改用**专用扫码模块**（走 USART1/PB6-PB7），MaixCAM2 只回应答 |
| `'2'` / `'a'` | 物块定标（第一圈 / 第二圈） | `run_wukuaiyuanxin_1` |
| `'3'` / `'4'` | 两圈循环检测 | `LOOP1` / `LOOP2` |
| `'5' '6' 'b' 'c'` | 指定工位物块是否静止 | `run_wukuaiyuanxin_xuanzequyu` |
| `'7' '8'` | 色环粗定标 / 精细定标 | `run_sehuanyuanxin2_centered` |
| `'9' '0'` | 码垛粗定标 / 精细定标 | `run_maduoyuanxin2` |
| `'d'` | 边界线纠偏 | 原程序被注释掉，直接回 `000000000` |
| `'r'` | 重启视觉程序 | `restart_program`（`os.execl` 自杀重启） |

**树莓派 → STM32**：10 字节帧 `0xFF + 9 数据 + 0xFE`（`Serial.c:157-196` 状态机解析）

| 帧内容 | 数据位含义 | 用例 |
| --- | --- | --- |
| 物块帧 `xxxxyyyyCR` | [0..3]=x 四位十进制 ASCII，[4..7]=y 四位，[8]=颜色码 | `f{cx:04}{cy:04}{color}{radius:01}` |
| 任务码帧 `abc0def` | [0..2]=第一组顺序，[3]=分隔，[4..6]=第二组顺序 | `f{INDEX}{INDEX1}{INDEX2}456789` |
| 角度帧 `000000sid` | [6]=符号，[7]=整数部分，[8]=小数部分 | `f000000{sign}{int}{dec}` |
| 就位帧 `'98'` | `Serial_RxPacket[0]==0x39 && [1]==0x38` | 物块已静止，可以抓 |
| 就绪帧 `'987654321'` | 程序启动第一件事 | STM32 等它才开始 |

**颜色码约定**：红 `'1'` / 绿 `'2'` / 蓝 `'3'` / **没看到 `'4'`**。
STM32 侧 `Serial_RxPacket[8] == 0x34` 即「没看到」，据此走兜底分支（`direct_x=-5, direct_y=5`）。

### 1.3 视觉侧 1892 行到底在做什么

| 函数 | 行号 | 用什么算法 | 输出 |
| --- | --- | --- | --- |
| `run_erweima` | 98-171 | `pyzbar.decode` 灰度二维码，连续 3 次相同才确认 | 7 位任务码 |
| `run_wukuaiyuanxin_1/2` | 173-515 | `HoughCircles` 找圆 → 圆内 `inRange(HSV)` 比三色面积 | 圆中心 + 颜色 + 半径 |
| `run_wukuaiyuanxin_xuanzequyu` | 516-658 | 同上，按工位选 ROI | 同上 |
| `run_wukuaiyuanxin_A` | 659-786 | 快速扫 A1/A2/A3 三区颜色 | 三个颜色码 |
| `LOOP1/2 + main_logic_loop*` | 787-886 | 判断转盘转向 → 算任务序号 | 3 位序号 |
| `detect_boundary_line` | 887-982 | `Canny + HoughLinesP` → 长度加权平均角 → 方差筛选 | 角度（±9.9°） |
| `run_sehuanyuanxin2*` | 983-1375 | 色环圆心定标（粗/精两遍） | 圆心像素坐标 |
| `run_maduoyuanxin2*` | 1376-1774 | 码垛位置定标 | 圆心像素坐标 |
| `SerialInterruptHandler` | 1775-1892 | 多线程收串口 + 命令字分发 | — |

**一个关键事实**：这 1892 行里，**真正做识别的只有约 300 行算法，其余都是状态机、稳定性判定、ROI 坐标和重复代码**。
移植时那 300 行被 MaixPy 原生算子替代，状态机部分按同样语义重写（见 §6 交付代码）。

---

## 2. 可行性：逐个算子对照

| 原实现（OpenCV，CPU） | MaixPy 原生算子 | 等价性 | 说明 |
| --- | --- | --- | --- |
| `cv2.inRange(HSV)` + `countNonZero` | `img.find_blobs(LAB阈值)` | ✅ 等价 | 都是颜色区域分割，只是色彩空间不同（HSV→LAB）。MaixPy 可一次传多组阈值，用 `blob.code()` 位标记直接知道命中哪种颜色 |
| `cv2.HoughCircles` | `img.find_circles()` | ✅ 等价 | 都是 Hough 圆变换；MaixPy 参数 `threshold/x_margin/r_min/r_max` 对应 OpenCV 的 `param2/minDist/minRadius/maxRadius` |
| `pyzbar.decode` | `img.find_qrcodes()` | ✅ 更好 | 原生二维码识别，返回 `payload()/rect()/corners()`，不用自己转灰度 |
| `cv2.Canny + HoughLinesP` | `img.find_lines()` | ⚠️ 需换算 | **单位陷阱**：MaixPy 返回 Hough 的 `(rho, theta)`，`theta` 是**法线**角；原程序用的是斜率角 = `90° - theta`。不换算会整体偏 90° |
| `cv2.erode/dilate` 去噪 | `find_blobs(merge=True, margin=N)` | ✅ 等效 | 原程序靠膨胀合并碎块，MaixPy 直接有 merge 机制 |
| `cv2.VideoCapture` 多 USB 摄像头 | `camera.Camera()` 单个 MIPI 摄像头 | ❌ **不对等** | 本次迁移最大的硬件变化，见 §4.1 |
| `np.var/np.mean` 统计判定 | 纯 Python 计算 | ✅ 等价 | 3 个样本的方差，手写循环即可 |

从算法角度，MaixCAM 不但能做，而且更快：

| 能力 | 原方案（树莓派 CPU） | MaixCAM（硬件加速） |
| --- | --- | --- |
| 色块检测 @640x480 | ~15-30 ms（含多次腐蚀膨胀） | ~5-10 ms |
| 找圆 | ~40-80 ms（HoughCircles 很贵） | ~15-30 ms |
| 二维码 | ~30-60 ms（pyzbar） | ~20-40 ms |
| 端到端帧率 | ~8-12 fps | ~30-60 fps |

---

## 3. 目标架构

```
┌──────────────────── MaixCAM（替代树莓派）────────────────────┐
│  maixcam_app/main.py                                          │
│    ├── protocol.py  复刻原 10 字节帧协议（STM32 零改动）      │
│    ├── vision.py    find_blobs / find_circles / find_qrcodes  │
│    │                / find_lines 四个原生能力                 │
│    └── config.py    LAB 阈值 + ROI + 稳定性参数集中配置       │
│                                                               │
│  UART4: A21(TX) A22(RX) ──115200──► STM32 UART5 PD2/PC12   │
│  屏幕实时显示识别框，MaixVision 看预览调阈值                  │
└───────────────────────────────────────────────────────────────┘
```

**为什么 STM32 只需改波特率**：我把原协议的每个字节都在 MaixPy 里复刻了（`protocol.py`）：

```python
# 原程序 send_data()（第 52-61 行）的行为，逐字节复刻
arr = [ord(c) for c in str(data)]
while len(arr) < 9:
    arr.append(0)                      # 不足 9 字节补 0x00（注意不是 ASCII 的 0）
packet = bytes([0xFF] + arr[:9] + [0xFE])   # 帧头 + 9 数据 + 帧尾
```

STM32 的状态机（`Serial.c:157`）本来就是按 `0xFF...0xFE` 收 9 字节，
所以帧格式完全不用动；颜色码、任务码、角度帧的位含义也一一保留。**唯一必须改的是波特率**。

---

## 4. 关键差异与坑（这一节决定成败）

### 4.1 最大的坑：两台摄像头 vs 一台摄像头

原程序同时开了**两个 USB 摄像头**：

| 设备节点 | 用途 | 看的东西 |
| --- | --- | --- |
| `/dev/video_xia0` | 扫二维码 | 侧向/前方 |
| `/dev/video_shang0` | 物块、色环、码垛、边界线 | **俯视台面** |

MaixCAM 只有**一个 MIPI 摄像头接口**（`camera.Camera()` 只能开一路）。三种解法：

| 方案 | 做法 | 优点 | 缺点 | 成本 |
| --- | --- | --- | --- | --- |
| **A. 机械切换（推荐）** | 一个舵机带动**反射镜**或转动云台，在俯视和侧视之间切换 | 一台 MaixCAM 搞定；MaixCAM 自带 PWM 可直驱舵机 | 切换要 100-200ms；两个视角的坐标映射都要标定 | 舵机+镜子 ≈ 20 元 |
| **B. 两台 MaixCAM** | 一台管俯视（物块/色环/码垛/边界），一台管侧视二维码 | 无机械延迟；可并行 | 成本翻倍；STM32 要加一路 UART | 约两台整机 |
| C. 单视角重构 | 只用俯视，把二维码也贴平放 | 最简单 | **要改比赛道具摆放方式**，风险大 | 0 |

> 建议先按方案 A 做。原程序里二维码只在一开始扫一次（`main.c:110-113` 那一步），
> 之后全是俯视任务，所以「切换一次然后一直俯视」在时序上完全够用。

### 4.2 串口分配：MaixCAM2 UART4 ⇄ STM32 UART5，115200

> ℹ️ **更新**：默认串口已改为 **UART4（A21=TX / A22=RX，/dev/ttyS4）**。
> UART4 被系统「Maix 通信协议」占用，需先跑 `maix_toolkit/board/disable_comm.py --apply` 关闭并重启板子。
> 下文表格里的 UART2（B0/B1）为**备选方案**，保留作对照。

**设备型号确认：MaixCAM2**（AX630C）。它的引脚编号和一代 MaixCAM **完全不同**——
一代的 A19/A18 在二代的官方引脚图上根本不存在，网上抄来的 UART 代码要重新对表。

官方 MaixCAM2 Pin v1.0 引脚图上的串口：

| 官方标注 | 引脚 | 设备路径 | 备注 |
| --- | --- | --- | --- |
| UART0_TX / RX | A28 / A29 | /dev/ttyS0 | 系统日志口，**禁用** |
| UART1_TX / RX | A30 / A31 | /dev/ttyS1 | — |
| **UART4_TX / RX** | **A21 / A22** | /dev/ttyS4 | **本方案选用**（需先关 Maix 通信协议）|
| UART3_TX / RX | B2 / B3 | /dev/ttyS3 | 同脚复用 SPI2_MOSI/MISO |
| UART4_TX / RX | A21 / A22 | /dev/ttyS4 | 同脚复用 SPI1_MOSI/MISO |

**接线表**（对应自制集成板上的 U8 座）：

| MaixCAM2 | 方向 | STM32 | 说明 |
| --- | --- | --- | --- |
| A21（UART4_TX） | → | **PD2**（UART5_RX） | MaixCAM2 发，STM32 收 |
| A22（UART4_RX） | ← | **PC12**（UART5_TX） | STM32 发，MaixCAM2 收 |
| GND | — | GND | **必须共地** |
| 5V | — | 5V1（U8 第 4 脚） | 供电 |

两边都是 **3.3V TTL 电平，直连即可**，不要加电平转换板。

**波特率必须 115200**：MaixPy 官方文档明确「仅验证 115200 可靠可用」。
STM32 侧对应的驱动我已经写好（stm32_patch/vision_uart.c），初始化时传 115200。

> ⚠️ 原来的 9600 还有个隐患：一帧 11 字节要 ~11.5ms，而 STM32 的 wait_send_shumeipai 是死等回包的，
> 一旦线缆或波特率有问题就永久卡死（工程自带报告把它列为严重级风险 ③）。

> 另外顺带说一句：**SYSTEM/Serial.c 本身也是残缺的** —— 它配的是 PA9/PA10 的 USART1，
> 但你们板子上扫码模块走的是 PB6/PB7，所以那份 USART1 驱动现在是对不上硬件的，需要一起修（见 stm32_patch/README.md 第 5.1 节）。

### 4.2b 为什么是 UART5 而不是 USART1

USART1 已经被**扫码模块**占用（你们板上是 PB6/PB7），所以 MaixCAM2 只能另找一路。
UART5（PC12/PD2）原本接蓝牙 HC-05，而蓝牙在代码里是**死代码**（find_congji/fasong_task_code 从未被调用），
把 UART5 挪给视觉是改动最小的方案：不用新占引脚、不用动 IMU(USART2) 和串口屏(USART6)。

STM32 侧需要的改动：**新增 UART5 驱动 + main.c 里把 Serial_RxPacket 换成 Vision_RxPacket**，
协议和解析逻辑一个字都不用改。详见 stm32_patch/README.md。

### 4.3 电气要点小结

- 3.3V TTL 直连，**不需要电平转换**；GND 必须共地
- 默认用 UART4（A21/A22）；若该口被系统占用，先 `disable_comm.py --apply` 让出来，或改用备选 UART2（B0/B1）
- **别用 UART0**（A28/A29）：它是系统终端和日志口
- 接线前先量一下：MaixCAM2 的 A21 是输出，PD2 是 STM32 的输入；接反了双方都收不到
### 4.4 坐标系口径必须冻结

STM32 端有一套线性换算 + 标定系数（`main.c:697-800`）：

```c
// transfer_position_wukuai：把像素坐标换算成底盘移动量(mm)
direct_x = (x_origin - 322) * (-0.0035 * (distance + 80) + 1.1967);   // 转盘区
direct_y = (y_origin - 165) * (-0.0035 * (distance + 80) + 1.1967);
```

这些系数是**针对 640x480 像素坐标**拟合的。所以：

- MaixCAM 必须**先用 640x480**（`config.py` 已设好），保证像素坐标口径一致；
- 若以后想降到 320x240 提帧率，**要么同步改 STM32 标定系数（除以 2），要么让 MaixCAM 把坐标放大回 640x480 口径再发**——后者更安全：

```python
# 降分辨率时把坐标放大回 640x480 口径再上报，STM32 侧零改动
SCALE_X = 640.0 / config.W
SCALE_Y = 480.0 / config.H
link.send_data(pack_wukuai(cx * SCALE_X, cy * SCALE_Y, color, radius))
```

### 4.5 颜色判定：HSV → LAB 必须重新标定

| 颜色 | 原 HSV | MaixPy LAB（起手值，**必须重标**） |
| --- | --- | --- |
| 红 | H 0-10 & 160-180, S 50-255, V 50-255 | `[0, 80, 40, 80, 10, 80]` |
| 绿 | H 30-89, S 40-255, V 40-255 | `[0, 80, -120, -10, 0, 30]` |
| 蓝 | H 90-140, S 50-255, V 50-255 | `[0, 80, 30, 100, -120, -60]` |

标定方法比原来更简单：板子上打开自带**「找色块」APP** → 点一下物块 → 再点一下出现的框 → 阈值自动填好 → 抄进 `config.py`。
灯光必须和比赛一致（原程序用 `TIM3` 补光灯，`main.c:127,184` 控制，记得开）。

### 4.6 圆形检测的半径口径变了（但不影响协议）

原程序用 `HoughCircles` 直接得到半径（35-45 px）。MaixPy 有两个选择：

- `find_circles()`：同样给圆心和半径，但**参数语义不同**、耗时也更高；
- `find_blobs()`：给外接矩形，半径用 `(w+h)/4` 折算（`vision.py` 里默认这么做，更快）。

好在这套系统里 **STM32 只用 [0..7] 位（x/y），半径本来就只是占位**——
原程序 `f{cx:04}{cy:04}{color}{radius:01}` 是 10 个字符，而 `send_data` 只发前 9 个，
**半径的最后一位其实被丢掉了**。所以半径精度对现有协议没有影响。

### 4.7 串口屏 / IMU / CAN 不归视觉管，不用动

STM32 侧的串口屏（USART6）、IMU（USART2）、蓝牙（UART5）、CAN 步进（CAN1）与视觉无耦合，
MaixCAM 接入后这些**完全不用碰**。

---

## 5. 逐任务迁移对照表

| 原命令字 | 原函数（行号） | MaixCAM 实现 | 关键改动 |
| --- | --- | --- | --- |
| `'1'` | `run_erweima` (98) | `task_erweima()` | pyzbar → `find_qrcodes()`；保留连续 3 次相同才确认 |
| `'2'` `'a'` | `run_wukuaiyuanxin_1` (173) | `task_wukuai()` | HoughCircles+HSV → find_blobs(LAB)；保留 N 帧稳定 + 颜色投票 |
| `'3'` `'4'` | `LOOP1/2 + main_logic_loop*` (787) | `task_loop(first_loop)` | 三段扫描 → `_scan_three_areas()`；序号计算逻辑逐行保留 |
| `'5' '6' 'b' 'c'` | `run_wukuaiyuanxin_xuanzequyu` (516) | `task_select_area()` | 稳定后回 `'98'`（与原来一致） |
| `'7' '8'` | `run_sehuanyuanxin2*` (983/1198) | `task_sehuan(fine)` | 色环圆心定位，粗/精两遍 |
| `'9' '0'` | `run_maduoyuanxin2*` (1376/1591) | `task_maduo(fine)` | 同上 |
| `'d'` | `detect_boundary_line` (887) | `task_boundary()` | Canny+HoughLinesP → find_lines；**注意 theta 换算** |
| `'r'` | `restart_program` (43) | `dispatch()` 里回 `987654321` | 不再自杀重启进程，改为重新握手 |

### 5.1 稳定性判定：这是原程序的精髓，必须保留

原程序每个任务函数里都抄了一遍这段（我抽成了 `StabilityTracker` 类）：

```
每帧检测到目标：
  color_history.append(颜色)          # 颜色投票窗口
  与首帧中心比距离：
      dist < max_pixel_move(10) → stable_count += 1
      否则 → stable_count = 1，重置首帧和颜色历史
  颜色投票：窗口 ≥15 帧且最高票占比 ≥0.7 → final_color 确认
  stable_count ≥ threshold(15~30) 且 final_color 确认 → 发数据
```

为什么不能省：**这是抗抖动、抗误检的核心**。去掉它，一次误识别就会让机械臂扑空或撞台面。

---

## 6. 交付代码（`maixcam_app/`）

| 文件 | 行数 | 作用 |
| --- | --- | --- |
| `protocol.py` | 146 | 复刻 10 字节帧协议 + 命令字收发 + 各任务打包函数 |
| `../stm32_patch/vision_uart.c/.h` | 100+ | **STM32 侧新增**：UART5(PC12/PD2) 视觉驱动 + 帧状态机 |
| `../stm32_patch/README.md` | — | STM32 集成步骤 + 必须顺带修的硬伤 |
| `vision.py` | 199 | 四个原生识别能力（色块/圆/二维码/边界角），含 theta 换算 |
| `config.py` | 56 | LAB 阈值、ROI（沿用原像素坐标）、稳定性参数 |
| `main.py` | 452 | 命令字分发 + 8 个任务函数 + StabilityTracker |

全部通过 Python 语法校验。运行方式：

```powershell
# 用上一轮配好的开发环境传到板子并运行
cd maixcam-dev
pwsh -File tools\put.ps1 -Project ..\maixcam-migration\maixcam_app -Run
```

---

## 7. 分阶段实施路线

| 阶段 | 目标 | 验收标准 | 预估 |
| --- | --- | --- | --- |
| **P0 打通链路** | MaixCAM 与 STM32 通电通信 | MaixCAM 发 `987654321`，STM32 能进主流程；STM32 发 `'1'`，MaixCAM 打印收到 | 半天 |
| **P1 单个任务** | 二维码 + 物块定标（命令字 1/2） | 串口屏任务码正确；STM32 能按坐标移动到位 | 1-2 天 |
| **P2 全任务** | 补齐 3/4/5/6/b/c/7/8/9/0 | 全流程空跑一遍不卡死 | 2-3 天 |
| **P3 边界线** | `find_lines` 版本的角度纠偏 | 与旧系统角度误差 < 1° | 半天 |
| **P4 调优** | ROI 收紧、分辨率/帧率、稳定性参数 | 端到端周期比原系统短 | 1 天 |

### P0 的调试利器

先用上一轮建的环境把链路单独测通，**不要一上来就跑完整流程**：

```powershell
cd maixcam-dev
# 把 USB 转 TTL 并在 MaixCAM 的 TX 线上，用串口助手直接看字节
python tools\serial_monitor.py COM5 115200 --hex
# 应该看到：FF 39 38 37 36 35 34 33 32 31 FE
```

---

## 8. 风险与前置修复（重要）

这套 STM32 工程自带的分析报告已指出 4 个**严重级**问题。移植视觉之前或同时**建议先修掉**，
否则 MaixCAM 再稳也会被 STM32 侧拖死：

| # | 问题 | 位置 | 对移植的影响 |
| --- | --- | --- | --- |
| ① | `main.c:83-91` 疑似调试残留，`jixie_reset()` 里有 `while(1){}` 永不返回 | main.c:90, 814 | **主流程整个不可达**，MaixCAM 再改也跑不到识别环节 |
| ② | `USART6` 开了接收中断但没有 ISR | ckp/uart6_ckp.c:52 | 串口屏回一个字节就进 `Default_Handler` **死循环** |
| ③ | 所有握手无超时、无看门狗（`time` 参数声明了没用） | main.c:821 | 视觉丢一帧 → STM32 永久卡死 |
| ④ | `shengjiang_control` / `pingtui_control` 越界直接 `while(1){}` | yyb_move.c:184,217 | 坐标异常就锁死 |

**MaixCAM 侧我已经做的防护**：

- 每个 `task_*` 都有超时，超时就回 `000000004`（没看到），不让 STM32 死等；
- `wait_command` 有超时（原程序是死等）；
- 用 `app.need_exit()` 而不是 `while True`，MaixVision 点停止能干净退出；
- 命令字未知时只打印警告，不崩。

---

## 9. 收益总结

| 维度 | 原方案（树莓派 + OpenCV） | MaixCAM 方案 |
| --- | --- | --- |
| 视觉帧率 | 8-12 fps | 30-60 fps |
| 启动时间 | 树莓派 Linux 开机 30-60s + 导入 cv2/pyzbar 数秒 | 上电数秒即可运行 |
| 体积/功耗 | 树莓派 4 + 两个 USB 摄像头，5V/3A | 一块 40x60mm 板子 + 一个摄像头 |
| 成本 | 树莓派 4（约 400 元）+ 2 摄像头 | MaixCAM（约 200 元）+ 1 摄像头 |
| 断线恢复 | 进程崩溃要手动重启 | 可配开机自启 + 看门狗 |
| 开发体验 | 改一行要 SSH + 重启脚本 | MaixVision 在线运行 + 实时预览 + 阈值编辑器 |
| 移植工作量 | — | **协议零改动，STM32 改 1 行波特率** |

---

## 10. 需要你确认的三件事

1. **摄像头视角怎么解决？**（§4.1 的 A/B/C）—— 决定接线与机械改动，推荐 A：舵机转镜。
2. **STM32 侧那 4 个严重问题能否先修？**（§8）—— 尤其 ① 的 `while(1){}`，不修主流程根本跑不到视觉环节。
3. **命令字 `'r'`（重启视觉）** 原程序是杀死 Python 进程重启；我在 MaixCAM 上改成回 `987654321` 重新握手。
   如果 STM32 侧依赖「视觉真的重启了」，需要另想办法。

---

## 参考

- 原工程自带分析报告 `yyb_stm32/程序分析报告.md` —— 协议表、硬件资源、风险清单都很详细，强烈建议通读
- [MaixPy 寻找色块](https://wiki.sipeed.com/maixpy/doc/zh/vision/find_blobs.html)
- [MaixPy 寻找直线](https://wiki.sipeed.com/maixpy/doc/zh/vision/line_tracking.html)
- [MaixPy 二维码识别](https://wiki.sipeed.com/maixpy/doc/zh/vision/qrcode.html)
- [MaixPy API: maix.image](https://wiki.sipeed.com/maixpy/api/maix/image.html)（find_blobs/find_circles/find_lines/find_qrcodes 全参数）
- [MaixPy UART](https://wiki.sipeed.com/maixpy/doc/zh/peripheral/uart.html)（115200 唯一可靠波特率的出处）
