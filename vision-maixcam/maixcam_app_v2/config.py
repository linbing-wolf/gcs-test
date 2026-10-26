# -*- coding: utf-8 -*-
"""
config.py —— 新流程（智能搬运赛项）视觉端配置（唯一参数来源）

与旧工程的关键差异（务必注意）：
  1) 颜色码换了！官方新定义：红1 黄2 蓝3 绿4（旧工程是 红1 绿2 蓝3）。
  2) 任务码变成 4 组三位数："G1+G2+G3+G4"，共 15 字符（旧的是 2 组 7 字符）。
  3) 新增区域：启停区、原料区(转盘)、粗加工区、暂存区。

LAB 阈值下面几组是**起手值**，必须在比赛灯光下用板子自带的「找色块」APP 重新标定！
"""

# ============================== 分辨率 ==============================
W, H = 640, 480
FPS = 60

# ============================== 颜色码（官方定义）==============================
# 顺序即阈值数组顺序，find_blobs 的 code 位序依赖它，别乱动。
COLOR_NAMES = ["red", "yellow", "blue", "green"]
COLOR_CODES = {"red": "1", "yellow": "2", "blue": "3", "green": "4"}

# LAB 阈值 [L_min, L_max, A_min, A_max, B_min, B_max]
# 由 870 张样本集统计得出（tools_analyze_lab.py：取最饱和 8% 像素 + 剔除离群图 + p2/p98）
# 口径：MaixPy/OpenMV  L 0~100, A/B -128~127
TH_RED    = [15, 60,  35,  75,  18,  60]     # 红：A 明显偏正
TH_YELLOW = [45, 90, -16,   5,  45,  78]     # 黄：A 近 0、B 明显偏正
TH_BLUE   = [15, 60,   6,  38, -75, -35]     # 蓝：B 明显偏负
TH_GREEN  = [30, 78, -70, -32,  26,  64]     # 绿：A 明显偏负

THRESHOLDS = [TH_RED, TH_YELLOW, TH_BLUE, TH_GREEN]   # 顺序 = COLOR_NAMES

# ============================== 检测区域 ROI ==============================
# 格式 [x, y, w, h]（640x480 口径）。跑通后按现场收紧。
ROI_START      = []                      # 启停区蓝色块：全画面找最大蓝块
# 启停区编号判定：画面右侧有没有「灰色道路」→ 有=1号 无=2号
ROI_ROAD       = [384, 120, 256, 300]    # 右侧检测区（640x480 口径）
# ⚠️ 样本集里「灰色路面」和「白桌面」都是低饱和（路面 L≈63，桌面 L≈70），
#    LAB 只能靠亮度差分离，现场必须用真实画面复核这条阈值。
TH_ROAD        = [35, 66, -12, 12, -12, 12]  # LAB 灰色路面（低饱和 + 偏暗）
ROAD_MIN_RATIO = 0.35                    # 灰色像素占 ROI ≥ 35% → 判定「有道路」
ROAD_STABLE_FRAMES = 5                   # 连续 N 帧结论一致才确认
START_ZONE_HAS_ROAD    = 1               # 右侧有道路 → 几号启停区
START_ZONE_NO_ROAD     = 2               # 右侧没道路 → 几号启停区
ROAD_AI_LABEL          = "other"         # AI 复确认：判成这个类才算「有道路」
# AI 判成 non-color（background/other）且置信度够时，是否直接丢弃该候选
#   True  = 严格：不把非物料当物料（可能漏，AI 判错时）
#   False = 宽松：退回阈值判色（不漏，但可能误检）
AI_VETO_OTHER = True
ROI_QRCODE     = []                      # 二维码：全画面
ROI_MATERIAL   = [140, 100, 360, 300]    # 原料区转盘（放 3 个物料）
ROI_DISC_ROUGH = [160, 20, 320, 340]     # 粗加工区圆盘（上边界必须包住整个靶环，否则会拟合出偏下的大假圆）
ROI_DISC_TEMP  = [160, 20, 320, 340]     # 暂存区圆盘（同上）
ROI_STACK      = [80, 0, 480, 480]       # 码垛区：纵向全屏（x 80..560，y 0..480）

# ============================== 稳定性判定 ==============================
STABLE_FRAMES    = 15     # 连续多少帧中心不动才确认
MAX_PIXEL_MOVE   = 10     # 相邻帧中心位移超过它就清零
COLOR_WINDOW     = 15     # 颜色投票窗口长度
COLOR_CONFIDENCE = 0.7    # 颜色置信度阈值

# ============================== 识别阈值 ==============================
MIN_PIXELS       = 300      # 色块最小像素数
STACK_COLOR_VOTE = False    # F 码垛时是否也做颜色投票（False=只判中心稳定，更快更稳）

# ============================== 原料区：停止才回复（转盘）==============================
# 原料区是电动转盘（6~10 秒/圈，间歇停）。物料还在转时不能回坐标，必须等它停稳。
PICK_STOP_ONLY       = True   # False = 退回旧行为（只要 15 帧中心稳定就回复）
PICK_STOP_SPEED_PX_S = 8.0    # 判定"停稳"的残余速度阈值（像素/秒，640x480 口径）
PICK_STOP_HOLD_MS    = 300    # 需要持续静止多久才确认（ms）
# 码垛选底座：同色多个时取离 ROI 中心最近的；没有同色就回"空环中心"
STACK_PICK_NEAREST = True
STACK_RING_R_MIN   = 20     # 找空环时的半径范围（px，640x480 口径）
STACK_RING_R_MAX   = 140
STACK_RING_THRESHOLD = 1200 # 霍夫圆累加阈值（比圆盘低，因为环线很细）
STACK_RING_FALLBACK = True  # 连环都没找到时，退回 ROI 中心（False=回 NONE,F）
# ---- F 码垛双轴对齐：与 D/E 一样发 MOVE/MOVEY ----
STACK_ALIGN_ON       = True    # False = 只报告，不发 MOVE/MOVEY
STACK_ALIGN_TOL      = 5       # 水平容差(px)
STACK_ALIGN_DY_TOL   = 5       # 垂直容差(px)
STACK_ALIGN_DX_OFFSET = 5      # 参数本身=5；目标 x=320-5=315
STACK_ALIGN_DY_OFFSET = 45     # 参数本身=45；目标 y=240-45=195
STACK_INVERT_X       = False   # F 的 MOVE 方向反了置 True
STACK_INVERT_Y       = False   # F 的 MOVEY 方向反了置 True
STACK_START_DELAY_MS = 800     # 【最长等待】等画面稳定再识别；判稳即提前开始（见 SCENE_SETTLE_*）
STACK_DEBUG_TRIGGER_REAL_FILE = "/tmp/run_stack_move"  # 调试：直接跑一次真实 F
STACK_DEBUG_TRIGGER_COLOR = "3"  # ""=F 不带颜色；"1"~"4"=按颜色找底座
# ---- F 颜色中心：CV 找不到同色底座时，用 AI 分类器分块扫描兜底 ----
STACK_AI_LOCATE      = True    # True = 启用 AI 分块找颜色中心
STACK_AI_ONLY        = True    # True = F,<颜色> 只用 AI，不再走 CV/空环兜底
STACK_AI_TILE        = 96      # 扫描块边长(px，640x480 口径)
STACK_AI_STRIDE      = 96      # 扫描步长(px)：96=无重叠→25 块(快)；48=50% 重叠→81 块(准)
STACK_AI_MIN_PROB    = 0.75    # AI 置信度下限
STACK_AI_BLUE_MIN_PROB = 0.90   # 蓝色(3)单独提高阈值，压掉蓝色误判
STACK_AI_BLUE_MAX_TILES = 12    # 蓝色命中块上限；新模型对物料边缘块也会命中，实测约 9~10 块
STACK_AI_BLUE_MAX_CY    = 240   # 蓝色中心必须在画面上半部；下半部多为车体/电控
STACK_AI_MIN_TILES   = 2       # 至少多少个相邻块命中才认可
STACK_AI_DEBUG       = True    # True = 日志打印 AI 命中块/置信度/中心
# ---- AI 粗定位后，用经典 CV 在 AI 框内找物料顶部圆心（精定位）----
STACK_CV_FINE        = True    # True = AI 粗中心 + CV 精修
STACK_CV_FINE_MODE   = "color" # color=目标颜色最大色块质心；circle=旧霍夫圆
STACK_COLOR_MIN_PIXELS = 100   # 颜色质心最小色块面积(px)
STACK_COLOR_MAX_JUMP   = 100   # 质心离 AI 中心超过该距离就丢弃(px)
STACK_TOP_R_MIN      = 18      # circle 模式：物料顶部圆半径下限(px)
STACK_TOP_R_MAX      = 55      # circle 模式：物料顶部圆半径上限(px)
STACK_TOP_THRESHOLD  = 1500    # circle 模式：顶部圆霍夫阈值
STACK_CV_SEARCH_MARGIN = 24    # AI 框外扩后再精修(px)
# ---- F 码垛调试记录（JPEG + meta.jsonl，放在 APP 目录外）----
STACK_DEBUG_SAVE        = False   # 比赛提速：默认关；要抓 F 调试图改回 True
STACK_DEBUG_DIR         = "/root/yyb_mover_data/stack_debug"
STACK_DEBUG_MAX_FRAMES  = 80   # 每次 F 最多存 80 张，覆盖完整的动作前/动作后
STACK_DEBUG_QUALITY     = 85
STACK_DEBUG_KEEP_TASKS  = 0    # 0 = 不自动删除，保留每次 F 的校准照片
STACK_DEBUG_MIN_INTERVAL_MS = 100
MIN_PIXELS_START = 5000     # 启停区是大面积蓝，用大阈值避免和蓝色物料混淆
QR_REPEAT        = 3        # 二维码连续 N 次一致才确认
QR_REGEX         = r"^\d{3}\+\d{3}\+\d{3}\+\d{3}$"   # 15 字符任务码

# ============================== 圆盘 / 圆环 ==============================
DISC_R_MIN     = 40         # 圆盘半径范围（find_circles）
DISC_R_MAX     = 90         # 靶环外径约 60~70px，超过 90 基本都是假圆
DISC_THRESHOLD = 2000       # find_circles 累加器阈值
# 环距（像素，640x480 口径）。视觉只给圆盘中心，各环位置由 STM32 按此环距
# + 平行关系推算（官方：环为同心/等距，视觉看不到所有环）。
RING_SPACING_PX = 22

# ---- 圆盘识别增强（D/E 专用）----
DISC_REFINE_ON         = True   # 粗定位后用「径向边缘点 + 最小二乘」拟合亚像素圆心
DISC_REFINE_RAYS       = 72     # 径向采样条数（越大越准、越慢）
DISC_REFINE_BAND       = 10     # 在粗半径 ±band(px) 内找边缘（应小于环距的一半）
DISC_REFINE_STEP       = 2      # 径向采样步长(px)
DISC_REFINE_MIN_GRAD   = 6      # 边缘最小亮度梯度(0~255)，抗噪
DISC_REFINE_MIN_POINTS = 24     # 有效边缘点少于此值 → 放弃拟合，用粗结果
DISC_REFINE_MAX_SHIFT  = 6      # 拟合圆心相对粗圆心最大偏移(px)，超了视为不可信
DISC_REFINE_MAX_RMS    = 3.0    # 拟合残差 RMS 上限(px)，超了视为不可信
DISC_REFINE_TRY        = 2      # 粗候选最多尝试几个（第一个拟合失败时再试下一个）
DISC_CLUSTER_PX        = 15     # 圆心聚类半径(px)：同心环的圆心应落在这个范围内
DISC_CLUSTER_MAX_INPUT = 80     # 聚类前最多取多少个候选（按半径从大到小截断，防 O(n^2) 过慢）
DISC_RING_FALLBACK     = True   # find_circles 完全找不到时，用同心圆环兜底
DISC_RING_R_MIN        = 20     # 圆环兜底半径范围（px）
DISC_RING_R_MAX        = 140
DISC_RING_THRESHOLD    = 1200   # 圆环霍夫累加阈值（环线细，比圆盘低）
DISC_TIMEOUT_MS        = 8000   # D/E 专属兜底超时（比通用 TASK_TIMEOUT_MS 短）
DISC_START_DELAY_MS    = 600    # 【最长等待】等画面稳定再读第一帧；判稳即提前开始（见 SCENE_SETTLE_*）
DISC_LOST_FRAMES       = 2      # 连续 N 帧看不到圆 → 立即回 NONE（1=第一帧就回）
DISC_TRACK_MAX_JUMP    = 50     # 优先选离上一次圆心 ≤N px 的候选（0=不启用）

# ---- D/E 频率与跑飞保护 ----
DISC_MIN_INTERVAL_MS   = 100    # 两次 D/E 检测最小间隔；更快的重复 D 直接回 BUSY
DISC_RUNAWAY_GUARD     = True   # 连续同方向且 |dx| 持续增大 → 判定方向反了/跑飞
DISC_RUNAWAY_COUNT     = 4      # 连续 N 次“同方向 + |dx| 变大”触发保护（锁死直到 R）
DISC_RUNAWAY_GROW_PX   = 2      # |dx| 至少增大 2px 才算“变大”

# ---- D/E 同心环靶：黑色像素质心（比 Hough 稳，现场实测抖动 ~3px）----
DISC_BLACK_CENTER_ON   = True   # True=优先用黑色环像素质心定位（靶是黑环+白底）
DISC_BLACK_MIN_PIXELS  = 300    # 黑色像素少于此值 → 退回 Hough
DISC_BLACK_ROI         = [190, 45, 240, 195]  # 黑色质心专用子区域（640x480）：包住中间靶，避开左右靶和机械臂
DISC_BLACK_SUB_RATIO   = 0.65   # DISC_BLACK_ROI 为空时才用：取任务 ROI 上部 65%
DISC_BLACK_TOP_N       = 8      # 取最大的 N 个黑色 blob 做加权质心

# ---- D/E 最内圈小圆（靶心白块）----
# 同心环靶最内圈是一个亮色小圆盘；只认它，避免 Hough 在多层环之间跳。
DISC_CENTER_ON         = True
TH_CENTER_LIGHT        = [55, 100, -18, 18, -18, 18]  # 白色/亮色（LAB）
DISC_CENTER_MIN_D      = 25     # 中心白块直径范围(px)
DISC_CENTER_MAX_D      = 140
DISC_CENTER_MIN_PIXELS = 200
DISC_CENTER_PICK_RADIUS = 20    # 距黑色质心多少 px 内优先取“最小”的白块（最内圈）

# ---- D/E 内容校验：圆内必须能匹配到数字“2” ----
# 只有模板匹配通过才画绿圆/回 DISC；否则回 NONE。解决“没圆也产生绿圆”。
DISC_TEMPLATE_ON        = True
DISC_TEMPLATE_PATH      = "/maixapp/apps/yyb_mover/assets/target2.png"
DISC_TEMPLATE_THRESHOLD = 0.55   # NCC 阈值（0~1）；当前物料挡住数字下半部，实测 2 号环 ~0.59，1/3 号环 <0.30
DISC_TEMPLATE_SCALES    = [0.9, 1.0]  # 0.9 覆盖稍远距离；两个尺度仍然很快
DISC_TEMPLATE_ROI_W     = 72     # 在候选圆心周围搜“2”的窗口（环距仅 22px，窗口要小）
DISC_TEMPLATE_ROI_H     = 80
DISC_TEMPLATE_ROI_DX    = 0      # 窗口中心相对候选圆心的偏移
DISC_TEMPLATE_ROI_DY    = 0
# 匹配点必须离候选圆心足够近，否则当作没匹配（防止锁到旁边别的环上）
#   实际上限 = min(本值, max(12, 0.35×候选半径))
DISC_TEMPLATE_MAX_SHIFT = 22

# ---- D/E 调试抓图（现场分析用；正式比赛可关）----
DISC_DEBUG_SAVE          = False   # True=每次 D/E 保存关键帧 + 逐帧检测数据
DISC_DEBUG_DIR           = "/root/yyb_mover_data/disc_debug"  # 放在 APP 目录外，升级不丢
DISC_DEBUG_MAX_FRAMES    = 24     # 单次任务最多存多少张图（数据行不受限）
DISC_DEBUG_QUALITY       = 85     # JPEG 质量 1~100
DISC_DEBUG_KEEP_TASKS    = 20     # 只保留最近 N 次任务目录，自动删旧
DISC_DEBUG_MIN_INTERVAL_MS = 100  # 两张图之间至少间隔 ms（0=不限）
# 不接 STM32 也能抓一次：SSH 上 `touch /tmp/run_disc_once`，
# 空闲循环会跑一次 D（或 E）任务，只记录不发 MOVE。
DISC_DEBUG_TRIGGER_FILE  = "/tmp/run_disc_once"
DISC_DEBUG_TRIGGER_REAL_FILE = "/tmp/run_disc_move"  # 存在时直接跑一次真实 D（发 MOVE + DISC）
DISC_DEBUG_TRIGGER_CMD   = "D"    # "D"=粗加工区，"E"=暂存区
DISC_DEBUG_TRIGGER_MS    = 2000   # 触发抓图持续时长(ms)

# ============================== 圆盘对齐（P6：环2 到水平正中心）==============================
# 视觉端主动发 MOVE,<dx> / MOVEY,<dy> 驱动下位机微调，直到 |x-W/2|<=ALIGN_TOL 且 |y-H/2|<=ALIGN_DY_TOL
ALIGN_ON          = True    # False = 只报告，不主动发 MOVE（中心稳定就带偏差回复）
ALIGN_MODE        = "movexy"  # movexy=MOVEXY,dx,dy 双轴并行；split=MOVE + MOVEY
ALIGN_ONE_SHOT    = True    # True = 只发一轮 MOVEXY（或 MOVE/MOVEY）就回复
ALIGN_ONE_SHOT_OK = 1       # 兼容旧配置；当前 DISC 回复的 ok 固定为 1
# ---- 回复策略：回读已彻底移除 ----
#   现在 D/E、F、H 发完 MOVE/MOVEY/MOVEXY 立即返回「动作前」的 x/y/dx，绝不再回读。
#   理由：回读要等机构停稳（ONE_SHOT_VERIFY 0.6~2.5s），是为诊断而生的功能；
#   比赛/现场只要最快响应，移动是否正确由上位机自己按刚发的步长推算。
ALIGN_TOL         = 5       # 对齐容差（px）；按 2 环约 4mm 预算取 5px
ALIGN_GAIN        = 0.65    # 比例系数：按实测 k≈1.48 取 1/k；先走大部分，第二轮只补残差
ALIGN_STEP_MIN    = 2       # 单次 MOVE 最小步长(px)，防止小偏差发 0
ALIGN_STEP_MAX    = 0       # 单次 MOVE/MOVEY 最大步长（px）；0 = 不限制
MOVE_SCALE        = 1.0     # MOVE 数值系数：最终 MOVE = step × MOVE_SCALE（先改回 1x）
ALIGN_DX_OFFSET   = 2       # E 目标 x=318（应用 D 的 21:55 实测修正）
ALIGN_DX_OFFSET_D = 2       # D 专用：目标 x=318（按 21:55 实测 +3px 修正）
ALIGN_DX_OFFSET_E = None    # E 专用；None = 用 ALIGN_DX_OFFSET
# ---- 垂直方向（MOVEY）：与水平轴独立配置 ----
ALIGN_DY_ON       = True    # False = 不校正垂直偏差，只发 MOVE
ALIGN_DY_TOL      = 5       # 垂直对齐容差(px)
ALIGN_DY_OFFSET   = 40      # E 目标 y=200（应用 D 的 21:55 实测修正）
ALIGN_DY_OFFSET_D = 40      # D 专用：目标 y=200（按 21:55 实测 +10px 修正）
ALIGN_DY_OFFSET_E = None    # E 专用；None = 用 ALIGN_DY_OFFSET
MOVEY_SCALE       = 0.86    # E 实测过冲比 107/92 → 1/1.163 ≈ 0.86
ALIGN_INVERT_DY_D = False   # D：MOVEY 方向（反了就置 True）
ALIGN_INVERT_DY_E = False   # E：MOVEY 方向（反了就置 True）
MOVE_ECHO_HOLD_MS = 0       # 发完 MOVE 后屏幕回显停留时间(ms)；0=不停留（响应最快）
ALIGN_STABLE_FRAMES = 3     # 进入容差后还要连续 N 帧稳定才回 ok=1
ALIGN_SETTLE_MS   = 1200    # 等机构真正停稳再测第二次；800→1200
ALIGN_MAX_ITER    = 3       # 主修正 + 小补刀 + 最后一次收残差；2→3
ALIGN_INVERT_D    = False   # D：按要求先保持不反
ALIGN_INVERT_E    = False   # E：MOVE 方向保持（当前 E 是正确的）

# ============================== 码垛 ==============================
MATERIAL_HEIGHT_MM = 40     # 固定物料高度常量，随 STACK 帧下发（STM32 用于减去高度）

# ============================== 超时 ==============================
TASK_TIMEOUT_MS = 20000     # 每个任务的兜底超时（原程序是死等）

# ============================== 串口全量日志 ==============================
# 每次 APP 启动在 SERIAL_LOG_DIR 下新建一个文件，记录全部 RX/TX（含原始帧 hex）。
SERIAL_LOG_ON  = True
SERIAL_LOG_DIR = "/root/yyb_mover_serial"
SERIAL_LOG_RAW = True       # True=同时记录原始帧 hex

# ============================== 串口（MaixCAM2）==============================
# 唯一来源：protocol.py 会读这里。
#
#   ★ 默认采用 UART4（底部 6P 1.25mm 座）：
#       A21 = UART4_TX → STM32 PD2 (UART5_RX)
#       A22 = UART4_RX ← STM32 PC12(UART5_TX)
#       设备路径 /dev/ttyS4
#   ⚠️ UART4 被系统「Maix 通信协议」默认占用（负责和 MaixVision 通信）：
#       必须先跑 maix_toolkit/board/disable_comm.py --apply 关掉它并重启板子；
#       之后 MaixVision 的 UART 连接会失效，改用 网络(TCP) / SSH 连板子。
#
#   备选 UART2（左侧 2x6 座）：B0=TX / B1=RX / /dev/ttyS2（见文件末尾注释）
UART_DEV = "/dev/ttyS4"
PIN_TX = "A21"
PIN_RX = "A22"
UART_FUNC_TX = "UART4_TX"
UART_FUNC_RX = "UART4_RX"
UART_BAUDRATE = 115200      # STM32 端必须同步 115200

# ============================== 启动重试 ==============================
STARTUP_RETRY = 3           # 相机/串口 打开失败时重试次数
STARTUP_RETRY_DELAY_MS = 1000   # 每次重试间隔

# ============================== 串口拆帧 ==============================
# 半帧超过这个时间没有新字节，就丢弃残留（防止断线后吞掉后续正常帧）
FRAME_TIMEOUT_MS = 200

# 备选方案：UART2（左侧 2x6 座）—— 需要时把下面 5 行替换上面的 5 行
# UART_DEV = "/dev/ttyS2"
# PIN_TX = "B0"
# PIN_RX = "B1"
# UART_FUNC_TX = "UART2_TX"
# UART_FUNC_RX = "UART2_RX"

# ============================== 调试开关 ==============================
SHOW_PREVIEW = True         # 屏幕显示 + MaixVision 预览
# ---- 摄像头补光灯（MaixCAM2 板载照明 LED）----
#   B25 = 照明 LED（高电平点亮）。别用 PWM6 调亮度：PWM6 与 A30(UART1) 二选一，
#   会和 Maix 通信协议抢引脚。
#   ★ 现状：**只在 D/E、H（近距离对盘）期间点亮**，任务前后自动开关；
#     其它命令（A/T/C/F/G）、空闲时都不亮。
DISC_LIGHT_ON  = False     # False = D/E 不点灯（现场要求：D/E 区关灯）
PLATE_LIGHT_ON = True      # False = H 也不点灯
CAM_LIGHT_PIN  = "B25"
CAM_LIGHT_FUNC = "GPIOB25"
VERBOSE = True              # 打印识别过程
TOUCH_UI = False            # 【默认关】True=触屏调试模式（会旁路串口命令！）
TOUCH_UI_SEND_UART = False  # 触屏触发的任务是否也发给 STM32（默认否，避免野数据）

# ============================== 屏幕命令回显条 ==============================
# 在画面底部常驻一条回显：最近收到的命令 / 最近发出的回复 / 状态与收发计数。
# 为什么需要：以前 R/T/Z 这类不读相机的命令在屏幕上一丝痕迹都没有，空闲时屏幕
# 还停在上次任务的最后一帧（像卡死）。开了这个，不接电脑也能一眼看出
# "单片机到底发进来没有、发的是什么、回的是什么、有没有坏帧"。
# 注意：SHOW_PREVIEW=False 时没有屏幕，这一项自动无效。
SCREEN_ECHO = True          # False = 关掉回显条（屏幕与之前完全一样）
SCREEN_ECHO_IDLE_MS = 0     # 0=空闲不刷预览（响应最快，屏幕会停在上一次画面）；>0=每 N ms 刷一帧
# 空闲主循环的轮询间隔(ms)：越小则命令响应越快（每轮只 sleep 这么久再 poll）。
IDLE_POLL_MS        = 2
SCREEN_ECHO_LINES = 3       # 显示行数 1..3（1=只显示 RX，2=+TX，3=+状态计数）

# ============================== 精度：总开关 ==============================
# False = 完全退回旧行为（不缩放 / 不去畸变 / 不多帧平均），方便现场 A/B 对比
PRECISION_ON = True

# ============================== 精度：坐标系（高分辨率检测）==============================
# 检测分辨率可与对外口径(W,H)不同；由 precision.Pipeline 自动缩放 ROI 与回传坐标。
# 注意：改 DET 后 CAM_K 会按比例自动缩放，不需要重标。
DET_W, DET_H = 640, 480     # 想提精度就调大（如 1280, 960），记得关掉 MaixVision 预览

# ============================== 精度：镜头畸变（坐标层反畸变）==============================
# 由 maix_toolkit/pc/calib_distort.py 生成后粘过来（2026-10-17 实测，RMS 0.35px）
UNDISTORT_ON = True
CAM_K = [1233.82373356, 0.0, 329.05893682, 0.0, 1251.76780944, 260.49139204, 0.0, 0.0, 1.0]
CAM_DIST = [-1.62119437, 5.17504868, 0.00591734, 0.00596065, -13.65384756]  # k1,k2,p1,p2,k3
# 反畸变保护：位移超过该值(px)或跑出画面 → 该点退回原坐标（四角标定不可信时防呆）
UNDISTORT_MAX_SHIFT = 48

# ============================== 精度：手眼标定（像素→毫米）==============================
# 由 maix_toolkit/pc/calib_homography.py 生成后粘过来
HOMOGRAPHY_ON = False
HOMOGRAPHY = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]     # 3x3 行优先

# ============================== 精度：多帧平均 ==============================
CENTER_FILTER_N = 9         # 坐标多帧平均窗口（σ 降为 1/sqrt(N)）；0/1 = 关

# ============================== AI（可选，见 maixcam_app_v2/ai_model.py）==============================
# 融合方式：经典 CV 负责「定位」（LAB 阈值找色块），AI 负责「判色」（抗现场色差）。
# AI 不可用/低置信度时自动退回阈值判色，所以开了不会比纯 CV 更差。
USE_AI = True               # 最终版默认开；模型已实测 val 99.38%，失效会自动退回阈值判色
AI_KIND = "classify"        # classify=分类(抗色差) | detect=检测(形状/码垛)
AI_MODEL = "/root/models/cls11n_far.mud"     # 清洗+远景微调后的 yolo11n-cls（F 96px 分块）
AI_CONF = 0.5
AI_IOU = 0.45
# 分类类别顺序（ultralytics 按文件夹字母序，必须与训练一致）+ 类别→颜色码
AI_LABELS = ["background", "blue", "green", "other", "red", "yellow"]
AI_CLASS_TO_COLOR = {"red": "1", "yellow": "2", "blue": "3", "green": "4"}
AI_MIN_PROB = 0.60          # AI 判色置信度下限；低于它退回阈值判色
AI_LOCATE_MARGIN = 8        # 送进分类器前把色块外扩的像素（给模型一点上下文）
AI_PREFER_CV = False        # True=强制以阈值判色为准，AI 只记录（A/B 对比排查用）
AI_DEBUG_PROB = False       # True=每次 AI 判色都打印 (label, prob)，用来定 AI_MIN_PROB

# ============================== 障碍物（前进状态，命令字 'G'）==============================
# 官方赛道障碍物：黑色，φ50×100mm。黑色 = L 很低。
TH_BLACK = [0, 35, -25, 25, -25, 25]   # LAB 黑色阈值（起手值，现场重标）
ROI_FORWARD = []                       # 前方视野；可收紧成下方条带如 [0,180,640,300]
MIN_PIXELS_OBST = 400                  # 障碍物最小像素数（抗噪点/阴影）
OBST_MERGE_MARGIN = 8                  # 合并碎块
OBST_WIDTH_MM = 50.0                   # 障碍物物理宽度(mm)，用于估距
OBST_CLEAR_FRAMES = 3                  # 连续 N 帧没看到 → 判定"前方无障碍"(CLEAR)
OBST_STABLE_FRAMES = 5                 # 障碍物中心稳定帧数（行进中连续问，要快）
OBST_TIMEOUT_MS = 3000                 # 兜底超时，超时按无障碍回

# ============================== 物料盘下弧对齐（命令字 'H'）==============================
# 场景：物料盘（白色带孔圆盘）太大，画面只能拍到它的下弧，圆心落在画面外（y 为负）。
# 做法：从「粗圆心」沿下半弧发射射线找边缘点 → 最小二乘拟合圆 → 取「最低点」(cx, cy+r)
#       作为基准；再与目标最低点相减得到 dx/dy，像 D/E 一样发 MOVEXY,dx,dy 微调。
# 回复复用 D/E 的 DISC 格式：DISC,<最低点x>,<最低点y>,<dx>,<ok>
#   （dy 不放进回复帧，而是随 MOVEXY 下发，与 D/E 的做法一致；STM32 侧解析无需改动）
# ⚠️ 粗圆心/半径和目标最低点都要现场标定；先把盘移到目标位置，看日志里的 low=(x,y)。
PLATE_ON            = True
PLATE_INIT_CX       = 310.0    # 粗圆心 x（对外 640x480 口径）
# 粗圆心 y：先验「盘大致在哪」，决定第一轮搜索带扫哪里（带内 ±PLATE_SEARCH_BANDS[0]）。
# 落点目标调过之后，把先验收在「旧落点 ~ 现落点」中间，保证都在搜索带内
# （先验低点 = cy+r ≈ 298，±60 覆盖 238~358；现落点 313 在其中）。
PLATE_INIT_CY       = -32.0
PLATE_INIT_R        = 330.0    # 粗半径（像素）
PLATE_SEARCH_BANDS  = (36.0, 14.0)  # 迭代搜索带半径(px)：小→更小（提速；大偏移交给粗定位）
PLATE_RAYS          = 120      # 下半弧采样射线数（120≈3°一条，精度与耗时平衡）
PLATE_STEP          = 2.0      # 径向采样步长(px)
PLATE_MIN_GRAD      = 8.0      # 边缘最小亮度梯度（抗噪）
PLATE_ARC_DEG       = (-20.0, 200.0)  # 采样角度范围(度)，覆盖可见下弧
# ---- 跳过正下方扇区（现场盘最低点常被夹具/阴影污染）----
#   实测现场：夹具与阴影正好压在盘下缘，拟合椭圆被整体拉低 ~30px（rms 6~8）。
#   故主拟合失败时，再用「跳过正下方 ±ALT 度、只靠两侧弧外推」试一轮：
#   SKIP_BOTTOM_DEG = 主拟合跳过的角度（0=不跳）；
#   SKIP_BOTTOM_ALT = 备用尝试跳过的角度；SKIP_ALT_ON 控制是否启用备用尝试。
PLATE_ARC_SKIP_BOTTOM_DEG = 0.0
PLATE_ARC_SKIP_BOTTOM_ALT = 30.0
PLATE_ARC_SKIP_ALT_ON     = False   # 最简版：不做备用尝试（省一轮拟合）
# ---- 弧模型：盘是平面圆，镜头斜视 → 投影是「轴对齐椭圆」而不是圆 ----
#   实测 12 张真实下弧图：边缘半径随角度极差约 12px（底部最小、±45° 最大），
#   圆拟合残差 ~3~4px，最低点比真实低 ~3.5px，且绿圆在两侧明显翘离盘缘。
#   "ellipse" = 拉伸空间拟合圆（k 自动估计），最低点 = (cx, cy + b)；"circle" = 旧行为。
PLATE_ARC_MODEL     = "ellipse"
PLATE_ARC_ROBUST    = True     # 拟合后剔除径向残差大的边缘点（中间黑条/夹具/阴影）再拟合
PLATE_ARC_OUTLIER_PX = 6.0     # 离群剔除半径阈值(px)；实际用 max(它, 2×rms)
# ---- 粗定位（先验无关）：盘横向偏移很大时，配置先验会偏出搜索带 → 先全图粗找盘 ----
#   逐列扫「亮→暗」最强下降沿（盘缘暗线）→ RANSAC 圆拟合 → 当粗圆心，
#   再由椭圆模型在它附近精修。抗中间黑条/夹具/线缆/桌面边等离群。
PLATE_COARSE_LOCATE      = True
PLATE_COARSE_X_STEP      = 20    # 粗扫列间隔(px)
PLATE_COARSE_Y0          = 24    # 从 y 多少开始扫
PLATE_COARSE_Y_STEP      = 3     # 粗扫纵向步长(px)：越大越快、越可能漏
# ★ 粗定位是「贵」操作（板端 ~150~200ms，get_pixel 约 15µs/次），
#   只在「配置先验那轮不可信」时才跑（惰性），正常位姿零额外开销。
PLATE_COARSE_ITERS       = 120   # RANSAC 采样次数
PLATE_COARSE_TOL         = 8.0   # RANSAC 内点阈值(px)
PLATE_COARSE_MIN_INLIERS = 12    # 至少多少内点才算找到盘
PLATE_COARSE_R_MIN       = 150.0 # 粗圆半径绝对范围
PLATE_COARSE_R_MAX       = 560.0
PLATE_COARSE_R_RATIO_MIN = 0.55  # 又要在「配置先验半径 × 这个比例」范围内，防认错圆
PLATE_COARSE_R_RATIO_MAX = 1.60
# ---- 质量闸门：拟合不可信时怎么处理 ----
#   实测 35 张真实图：可信拟合 rms ≤ 3.0 / npts ≥ 44；假圆 rms ≥ 5.96。
#   PLATE_GATE_MODE:
#     "report" = 【默认·现场要求】只标记低置信（日志 + meta.low_conf），
#                仍按最佳猜测发 MOVEXY / 回 DISC —— 「不准确也要执行完」
#     "reject" = 严格：不可信就回 NONE,H（不动作）
PLATE_QUALITY_GATE  = True
PLATE_GATE_MODE     = "report"
PLATE_MAX_RMS       = 4.0      # 拟合残差上限(px)，超了算低置信
PLATE_MIN_PTS       = 30       # 有效边缘点下限，少了算低置信
# ---- 几何硬校验（与「准不准」无关，任何模式都生效）----
#   现场出现过 low_y=613（画面只有 480 高）+ MOVEXY dy=195 的离谱拟合，
#   这种「几何上不可能」的结果必须无条件作废，不能因为"要执行"就放行。
PLATE_GEOM_CHECK        = True
PLATE_LOW_Y_MIN_RATIO   = 0.25   # 最低点 y 必须 ≥ 0.25×画面高（且 < 画面高）
PLATE_LOW_X_EDGE_RATIO  = 0.05   # 最低点 x 必须在 [0.05W, 0.95W]
PLATE_GEOM_B_RATIO_MIN  = 0.40   # 竖直半轴 b 必须在「先验 r × 0.40~2.00」
PLATE_GEOM_B_RATIO_MAX  = 2.00
# ---- 拟合提速：抽稀拟合点（点越多越慢，40~48 点足够）+ 鲁棒重拟合轮数 ----
PLATE_FIT_MAX_PTS   = 0        # >0 时抽稀拟合点（实测会掉精度，默认 0=不抽稀）
PLATE_ROBUST_ROUNDS = 1        # 离群剔除后重拟合轮数（1 足够；0=不重拟合）

# 低置信时把单次步长限幅（0=不限）；防止一次错拟合把车甩太远。
PLATE_LOWCONF_STEP_MAX = 0
PLATE_ELLIPSE_K_MIN = 0.86     # 椭圆竖直/水平半轴比 k 的搜索范围与步长
PLATE_ELLIPSE_K_MAX = 1.05
PLATE_ELLIPSE_K_STEP = 0.02
PLATE_TARGET_X      = 316.0    # 目标最低点 x（对外口径，现场标定）
# 现场反馈「定位后车往右偏」→ 目标 x 左移；之后又要求「车往左」→ 调回并加大：
# 311 → 291 → 316（有效目标 x = 316 − 2 = 314）。
# 注意方向：目标 x 往左 → 车最终停更靠右；要「车往左」应把 x 调大。
# 目标最低点 y：原 267 是「圆模型」标定的，椭圆模型的最低点整体上移，
# 故下调同步修正（实测 12 张真实图 圆low−椭low 平均 4.2px，范围 0.3~7.4px）。
# 换回 PLATE_ARC_MODEL="circle" 时应改回 267。
# 现场要求「落点大幅下移」：263 → 333（再下移 70px）；
# 之后又反馈「车不那么靠前一点」→ 落点抬高 20px：333 → 313；
# 再反馈「车可以再往后一点」→ 再抬高 10px：313 → 303；
# 最后现场实测定稿（车往左 + 往下）：x 291 → 316（有效 314）、y 303 → 278（有效 278）。
# 有效目标 y = PLATE_TARGET_Y − PLATE_DY_OFFSET；y 越大=落点越低=车越靠前。
PLATE_TARGET_Y      = 278.0
# 固定偏差修正（与 D/E 的 ALIGN_DX_OFFSET_D / ALIGN_DY_OFFSET_D 同款）
#   dx = 最低点x − PLATE_TARGET_X + PLATE_DX_OFFSET
#   dy = 最低点y − PLATE_TARGET_Y + PLATE_DY_OFFSET
# ★ 有效目标 = (TARGET_X − DX_OFFSET, TARGET_Y − DY_OFFSET)：dy=0 时最低点就落在这里。
#   DY_OFFSET=40 是把「对齐后落点」整体抬高 40px；现场反馈希望落点更靠下，
#   故 40→0：有效目标 y 由 223 变成 263，净下移 40px（想只下移 35px 就填 5）。
PLATE_DX_OFFSET     = 2
PLATE_DY_OFFSET     = 0
# ---- 运行时可调偏差 / 落点（现场调，免重编包）----
# 文件存在时逐行 KEY=VALUE 覆盖下面的项（只写要改的那几行即可，其余仍用 config）：
#   PLATE_TARGET_X / PLATE_TARGET_Y / PLATE_DX_OFFSET / PLATE_DY_OFFSET
#   ssh root@<板子> "printf 'PLATE_DY_OFFSET=5\n' > /tmp/plate_bias"   # 生效
#   ssh root@<板子> "rm -f /tmp/plate_bias"                            # 恢复默认
# 支持 # 注释、空行；数值不合法的行忽略。H 启动日志会打印实际生效值与来源。
PLATE_BIAS_FILE     = "/tmp/plate_bias"
PLATE_BIAS_CACHE_MS = 500     # 文件检查缓存(ms)，避免逐帧 stat
PLATE_BIAS_KEYS     = ("PLATE_TARGET_X", "PLATE_TARGET_Y",
                       "PLATE_DX_OFFSET", "PLATE_DY_OFFSET")
PLATE_TOL_X         = 6        # 水平容差(px)
PLATE_TOL_Y         = 6        # 垂直容差(px)
PLATE_STABLE_FRAMES = 3        # 进入容差后连续 N 帧稳定才回 ok=1
PLATE_LOST_FRAMES   = 2        # 连续 N 帧看不到下弧 → 立即回 NONE
PLATE_TIMEOUT_MS    = 8000     # 专属兜底超时（比通用 TASK_TIMEOUT_MS 短）
PLATE_START_DELAY_MS = 600     # 【最长等待】等画面稳定再读第一帧；判稳即提前开始（见 SCENE_SETTLE_*）
# 一次/闭环模式跟随 D/E 的 ALIGN_ONE_SHOT（True=只发一轮 MOVEXY，回读后即回复）
PLATE_GAIN          = 0.65     # 步长增益（比例系数）
PLATE_MAX_ITER      = 3        # 非 one-shot 时的最大微调轮数
PLATE_INVERT_X      = False    # 水平方向反了置 True（不要改 STM32 符号）
PLATE_INVERT_Y      = False    # 垂直方向反了置 True
# ---- H 调试抓图（关键帧 + meta.jsonl，放在 APP 目录外，升级不丢）----
PLATE_DEBUG_SAVE    = False     # True=每次 H 存关键帧（含画的椭圆/最低点）到 PLATE_DEBUG_DIR
PLATE_DEBUG_DIR     = "/root/yyb_mover_data/plate_debug"
PLATE_DEBUG_MAX_FRAMES = 16    # 单次 H 最多存多少张图
PLATE_DEBUG_QUALITY = 85
PLATE_DEBUG_KEEP_TASKS = 10    # 只保留最近 N 次 H 的目录
PLATE_DEBUG_MIN_INTERVAL_MS = 100

# ============================== 启动延时：改为「画面稳定」判定 ==============================
# 收到 D/E、F、H 后不再死等固定毫秒，而是等画面连续稳定后再读第一帧：
#   · 机构（机械臂/转盘/车体）还在动 → 画面变化大 → 继续等
#   · 停下来了 → 立即开始识别（通常比固定值更快、更可靠）
# 各任务原来的 DISC_/STACK_/PLATE_START_DELAY_MS 现在作为「最长等待上限」。
# SCENE_SETTLE_ON=False 时退回旧的固定等待（行为与 3.9.46 一致）。
SCENE_SETTLE_ON     = True
SCENE_SETTLE_FRAMES = 2        # 连续 N 帧画面变化 < 阈值 → 判定已稳定（3→2，提速）
SCENE_SETTLE_DIFF   = 3.0      # 降采样(32x24)灰度均差阈值；越小越严、越大越松
SCENE_SETTLE_MIN_MS = 20       # 至少等这么久，防止刚收到命令就误判稳定（100→40→20）