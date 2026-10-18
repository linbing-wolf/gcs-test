/* ============================================================================
 *  STM32 机械臂 + 爪子 合并测试版（CAN 两轴 + 舵机）—— 串口按键版
 *
 *  【这是什么】
 *    把两份程序合成一份：
 *      ① main_arm.c  的机械臂两轴：升降(addr 5) + 平推(addr 7)，CAN 位置模式
 *      ② main_claw.c 的爪子舵机：PWM 脉冲，无地址、不走 CAN。
 *         默认 PA2 = TIM2_CH3；现场按 1 / 2 / 3 可以切到 PA1 / PA2 / PA3 任意一路。
 *      ③ 云台舵机（源码 yuantai）：PA3 = TIM2_CH4，PWM 脉冲，无地址、不走 CAN
 *    **不包含四个轮子**（轮子的走位程序还是用 main_move.c / main_lap.c）。
 *
 *  【跟参考工程（原项目）的关系】★ 10/14 改版
 *    动作层不再"我们自己重写一遍"，而是**直接调用参考工程那几份文件的函数** ——
 *    它们已经原样拷进这个 Keil 工程（Hardware/PWM.c、Hardware/servo.c、
 *    can_ZDT/yyb_move.c，和原工程逐字节相同）：
 *      · 爪子 / 云台 / 载物台的转角 = PWM.c 的 set_zhuashou_Angle() /
 *        set_yuantai_Angle() / set_wukuaipingtai_Angle()，插值在 TIM2 中断里做
 *        （20ms 一步，S 曲线 6t^5-15t^4+10t^3），和原项目一样"非阻塞、几路能一起动"。
 *      · 三轴联动 = 照抄 PWM.c 的 move_all()：云台先下目标（时间按原公式分段算），
 *        两轴 CAN 同时发，最后 delay 三者里最长的那个时间等大家都到位。
 *      · 三段抓取流程 = 照抄 USER/main.c 的 yuantai_na()，角度/行程数字一个没改。
 *    仍然不同的地方（有意保留）：
 *      · 两轴 CAN 地址：升降 5（同源码）、**平推 7**（源码是 6，我们机上实测是 7）。
 *      · 爪子默认脚 PA1 = CH2（和源码 servo.c 的 Servo_SetAngle2_zhuashou 一致），
 *        另外保留 1/2/3 现场换脚（换脚就是换 servo 结构体里的函数指针，仍走中断插值）。
 *      · 多出来的这套串口按键菜单是调试用的，原项目没有；原项目是视觉+主流程自动跑。
 *
 *  【和原来两份的关系 / 哪些键变了】
 *    机械臂那几个键原样保留：u/j、f/r、s、+/-、[/]、0、e/d。
 *    爪子有三个键**必须改**，因为原来和机械臂撞车了，这里定成这样：
 *      claw 松力   ：原来是 s   →  现在 **x**（s 留给机械臂急停，更安全）
 *      claw 微调角度：原来是 +/- →  现在 **,** / **.**（+/- 留给机械臂步长）
 *      claw 打印参数：原来是 p   →  现在并进 **p**（p 一次把臂+爪都打出来）
 *    其余爪子键不变：o 开、c 合、b 咬中心、a 输角度、t 自动来回。
 *
 *  【接线】
 *    机械臂两台电机：并在 CAN 总线（H-H、L-L），和 VP230 绿端子接一起，共地。
 *    爪子舵机三根：红 +5V ── 电源模块**绿色座 5V**（★ 蓝色座 12V 会烧舵机）
 *                  棕/黑 GND ── 电源模块 GND，且要和核心板 GND 共地
 *                  橙/黄/白 信号 ── 核心板 PA1 / PA2 / PA3 之一（就看插在集成板哪个 3 针座）
 *    云台舵机三根：信号 ── 核心板 PA3（TIM2_CH4，= 集成板上丝印"云台"那个 3 针座的
 *                         **方形焊盘那一脚**）；电源和地跟爪子一样走 5V + 共地。
 *                  ★ 那个座子的 2 脚丝印是 8.4V，别把舵机整根三针插上去，先万用表量一下。
 *    ★ PA3 只能给一个舵机用：爪子要是哪天改接 PA3，云台就得换别的引脚。
 *    串口：板子右边 USB UART 口（PA9/PA10，115200），或无线调试器。
 *    注意：机械臂两轴要 12V/24V 电机电源；爪子舵机要 5V。两路都要共地。
 *
 *  【按键表】★ 串口打印全英文，避免乱码 ★
 *    ---- 机械臂（CAN，5=升降 7=平推）----
 *    u / j     升降 升 / 降（一个步长，默认 5mm）
 *    f / r     平推 伸出 / 收回（一个步长；账面 0 = 齿条收到底顶住后端，数字只往前加）
 *    w         平推直接回到初始位（账面 0 = 齿条收到底）。只是照账本走，不是回零/找限位：
 *              账本本身就错位时不救急，那种情况要按 d 失能、手推到后端、按 0 清零、按 e 使能。
 *    s         两轴立即停（急停保险键）
 *    + / -     臂步长 ±5mm（5 ~ 50）
 *    [ / ]     臂速度 -100 / +100 rpm（100 ~ 2000）
 *    0         臂账本归零（只清软件记账，不动电机）
 *              ★ 上电前：平推推到最后端顶住（账面 0）、升降抬到最上端顶住（账面 0）。
 *                忘了摆就先按 d 失能、手推到位，再按 0 清零、按 e 使能。
 *    e / d     两轴 使能 / 失能
 *    ---- 爪子（舵机，默认 PA1，可用 1/2/3 现场切脚）----
 *    ★ 上电/默认姿势：爪子 45 度（CLAW_HOME_DEG = CLAW_OPEN_DEG，张开姿），
 *      不用再手动按 b；要夹就按 c（合到 0 度 = 夹取角）。
 *    b         咬住中心角 90 度
 *    o / c     开到 CLAW_OPEN_DEG / 合到 CLAW_CLOSE_DEG
 *    , / .     当前角度 -/+ 一个步长（2 度，微调用）
 *    a135#     直接走到 135 度：按 a 开始输入，打数字，最后 # 结束
 *    l         量程实测：慢速往上探 5 度（0~270），每格都打印角度和脉宽
 *    z         往回退 5 度（探到顶、听到嗡嗡声立刻按这个）
 *    q         一次退回 30 度（卡住了用）
 *    t         自动来回开合 SWEEP_TIMES 趟
 *    x         松力（不发脉冲，可以用手掰）
 *    1 / 2 / 3 把爪子输出切到 PA1 / PA2 / PA3 —— 切完再按 b/o/c，哪个座的舵机动，
 *              爪子的线就在哪个座（集成板丝印：PA2 = 物块平台，PA3 = 云台）
 *    ---- 云台（舵机，PA3）----
 *    ★ 2026-10-15：我们车云台的方向跟参考车相反，且尺子已重新定零 ——
 *      初始位那一档 = 0 度（2026-10-17 重新定零，脉宽 1975us，舵机位置没动），
 *      朝物块转 = 角度往【正】走。参考工程的 278/295/261/135 在我们车上没有意义，
 *      这四个角都要自己量。复核认脉宽（按 p），别认角度。
 *    y         转到初始位 YUNTAI_INIT_DEG（★ = 0.0 度，脉宽 1975us）
 *    n         转到抓取角 YUNTAI_GRAB_DEG（★ 实测 126.0 度 = 1275us）
 *    g / k     当前角度 +/- 一个步长（2 度）；g 朝物块（变大），k 往回（变小）
 *              ★ 到 YUNTAI_MAX_DEG / YUNTAI_MIN_DEG 就停住并提示行程到头：
 *                这两头是同一颗单圈电位器的两端（500us / 2500us），
 *                软件再往外走不是"继续转"，舵机会自己反甩将近一整圈。
 *    i40#      直接走到 +40 度：按 i 开始输入，可带负号（i-30#），最后 # 结束
 *              ★ 任意角按 360 取模：i200# = -160、i360# = 0（回初始位）。
 *                0~360 每个位置都到得了；但 500/2500us 是单圈电位器的接缝，
 *                从一头"接着往同一方向"跨过去，实物要扫完整整一圈（约 358°），
 *                不是连续旋转舵机那种能一直转。
 *    v         松力（不发脉冲，可以用手掰）
 *    /         量程实测：往物块探 5 度（一路走到 500us 那头的极限）
 *    \         量程实测：反方向退 5 度（一路走到 2500us 那头的极限）
 *    速度       所有云台动作都乘 YUNTAI_SPEED_DIV（现在 2.0 = 源码一半速），
 *              手动键和三轴联动一起吃这一个数，嫌快就往上加。
 *    ---- 整套抓取流程（照抄原项目 yuantai_na()；行程照抄，云台角是占位）----
 *    4 / 5 / 6 抓红 / 绿 / 蓝：开爪 → 云台+平推+升降三轴同时动 → 合爪 → 退回来
 *              → 云台回 YUNTAI_BACK_DEG → 开爪（放料）。三个抓取角和退料角都还没标。
 *    ---- 载物台（舵机，PA2 = TIM2_CH3，源码 wukuaipingtai）----
 *    7 / 8 / 9  转到源码的 1 / 2 / 3 号位（20 / 139 / 256 度）
 *    ---- 一键夹取（2026-10-17 现场实测的那套数）----
 *    !         从默认位一次做完：开爪 -> 云台先转到 126 度（收着的时候转，摆幅最小）
 *              -> 高位把齿条伸到 65mm -> 降到 110mm -> 合爪并保持夹住。
 *              分几步走是因为从最高处直接斜着往 (35,115) 走会从物块上方掠过，
 *              在高处先伸出去、再垂直下来，每一步都在空处。松爪按 o。
 *    @         一键整轮：回默认位 -> 夹取（和 ! 完全同一套动作）-> 放上载物台。
 *              放下段（push 30 / lift 110 / yuntai 254，实测）：
 *              升到最高 135mm -> 齿条收回 0 -> 转到 254 度 -> 高位伸出 30mm
 *              -> 降到 110mm -> 开爪放下。顺序就是"先升高、再转、最后放下"。
 *    $         只做"放"这一段（手里已经夹着物块时用，不会重新去夹）。
 *    W         一键回默认位：升降升到最高 135mm -> 齿条收到底 0mm -> 云台回 0 度
 *              -> 爪子收回夹取角 0 度。先升后缩，爪子会先垂直抬离物块再横着退，
 *              不会拖着物块走（但爪子最后会松开，夹着东西别按）。
 *              （w 小写只回平推，W 大写连升降和云台一起回）
 *    ---- 通用 ----
 *    p         打印全部状态（臂 + 爪 + 云台）
 *    h 或 ?    重新打印按键表
 *
 *  【第一次上板怎么试】
 *    a) 先按 u 一下，看哪台动、往哪边走 → 认出升降；再按 f 一下认平推。
 *       认反了就把参数区 LIFT / PUSH 两块（地址+ppm+行程）整体对调。
 *       方向反了把 LIFT_DIR_INV / PUSH_DIR_INV 改成 1。
 *    b) 爪子先按 x 松力，用手掰到中间摸行程，再按 b 咬中心，用 , / . 试方向。
 *    没把握马上按 s（臂急停）或 x（爪松力），再断电。
 * ========================================================================== */

#include "stm32f4xx.h"
#include "Emm_V5.h"
#include "PWM.h"
#include "servo.h"
#include "delay.h"
#include "math.h"
#include "stddef.h"


/* 参考工程 PWM.c 里的三个舵机状态对象（非 static，可以直接 extern 进来用）。
 * 用法和原项目一样：给它下目标角，剩下的插值交给 TIM2 中断。
 *   zhuashou.current_angle      当前角（已经减过 1.0 修正的那把尺子）
 *   yuantai.current_angle       当前角（已经减过 7.9 修正的那把尺子）
 *   wukuaipingtai.current_angle 载物台当前角
 *   *.moving_flag               0 = 停稳，1 = 还在插值走 */
extern Servo_t yuantai;
extern Servo_t zhuashou;
extern Servo_t wukuaipingtai;


/* ==========================================================================
 *  ① 参数区
 * ========================================================================== */

/* ------------------------ 机械臂：升降（上下） ------------------------ */
#define ADDR_LIFT        5          /* CAN 地址（实车实测：5 = 升降） */
#define PPM_LIFT         80.0f      /* 源码 shengjiang_control：80 pulse/mm */
#define LIFT_MIN_MM      0.0f       /* 源码量程 0 ~ 135mm */
#define LIFT_MAX_MM      135.0f

/* ★ 2026-10-17：用户指定"机械臂默认位置 = 最高处"。
 *   程序上电时直接把升降账本设成这一档，而不是 0 ——
 *   实物平时就停在最高处，账本跟着对齐，按 j 立刻能往下走。
 *   （以前账本上电 = 0 表示"已经到底"，实际却停在顶上，一按 j 就被下限挡死，
 *     串口只会打 lift 0.0mm -> 0.0mm。）
 *   ★ 这只是"假设上电时在最高处"，没有编码器兜底：
 *     如果哪次开机机械臂不在顶上，按 d 失能 → 手动摆到想要的位置 → 按 0 认零点。 */
#define LIFT_HOME_MM     135.0f     /* 默认/上电位置：最高处（= 量程顶端） */
/* 10/14 实测：u/j 物理方向反了 —— 按 u 是往下、按 j 是往上。
 * 翻成 1，现在 u = 往上、j = 往下（和源码"数字越大越往下"一致）。 */
#define LIFT_DIR_INV     1          /* 填 1 就把 u/j 的物理方向对调 */

/* ------------------------ 机械臂：平推（前后伸出） -------------------- */
#define ADDR_PUSH        7          /* CAN 地址（实车实测：7 = 平推） */
#define PPM_PUSH         25.465f    /* 源码 pingtui_control：25.465 pulse/mm */
#define PUSH_MIN_MM      0.0f       /* ★ 0 = 齿条收到底、后端顶住（不再是源码的 -122） */
#define PUSH_MAX_MM      187.0f     /* 理论可用 ~187mm（源码量程 -122~+65 正好 187mm），★ 没实测。
                                     * 必须 ≥ PUSH_SRC(63)=185，否则 Grab_Flow 5/6 的目标
                                     * 会被 Axis_MoveTo() 静默截断到 180，少伸 5mm 还不报错。
                                     * 量准了再改；按 f 一步步试时手边留着急停 s。 */
/* ★ 2026-10-17 用户实测：按 f（账面往大走 = 该往外伸）齿条是往里缩的，
 *   也就是平推轴物理方向跟账本相反 —— 翻成 1。
 *   翻完：f = 往外伸（mm 变大），r = 往里缩，w = 回 0（收到底）。
 *   如果哪天又反了（换了电机/重装了齿条），把它改回 0 就行。 */
#define PUSH_DIR_INV     1

/* ★ 平推原点是 10/14 按用户要求改的：账面 0 = 齿条完全收到底、后端顶住。
 *
 *   程序是开环记账（不读编码器、也没有回零动作），上电那一刻齿条在哪，哪就是账面 0。
 *   所以"0 = 后端"靠的是操作流程，不是代码自己找的：
 *       断电（或按 d 失能）→ 手把齿条推到最后端顶住 → 上电（或按 0 清零再按 e）。
 *
 *   源码那台车的 0 位不在后端，在"后端往前 122mm"处（量程 -122 ~ +65）。
 *   我们的 0 位比它靠后 122mm，所以源码数字要 +PUSH_SRC_DELTA 才是我们账面上的值；
 *   Grab_Flow() 里就写成 PUSH_SRC(-100.0f) 这样，源码原值一眼还能看出来。 */
#define PUSH_SRC_DELTA   122.0f     /* 我们 0 位比源码 0 位靠后这么多：我们 = 源码 + 122 */
#define PUSH_SRC(v)      ((v) + PUSH_SRC_DELTA)

/* ------------------------ 机械臂：运动参数 ---------------------------- */
#define MOVE_SPEED       500        /* 起步速度，先慢，确认机构不卡滞再往上加 */
#define MOVE_SPEED_MIN   100
#define MOVE_SPEED_MAX   2000
#define MOVE_SPEED_STEP  100
#define MOVE_ACC         200        /* 0~255，越大起停越柔和 */

#define STEP_DEFAULT     5          /* 一次按键走多少毫米 */
#define STEP_STEP        5
#define STEP_MIN         5
#define STEP_MAX         50

/* 源码 PWM.c 的 move_all() 里两轴用的速度/加速度（源码写在函数体里，抄到这）：
 *   平推 speed 1000 / accel 230      升降 speed 2000 / accel 240 */
#define SRV_SPEED_PUSH   1000
#define SRV_SPEED_LIFT   2000
#define SRV_ACC_PUSH     230
#define SRV_ACC_LIFT     240

/* ------------------------ 爪子（舵机） -------------------------------- */
/* 爪子信号线接在 TIM2 的哪个通道：2 = PA1（10/14 实测：就是这路），3 = PA2，4 = PA3 */
#define CLAW_CH             2

/* 爪子到底接在哪个引脚，运行时可以现场改：串口按 1 / 2 / 3 就把爪子输出
 * 切到 PA1 / PA2 / PA3。集成板上两个舵机座（看丝印）：
 *   PA2 = "物块平台"（2 脚 5V）      PA3 = "云台"（2 脚 8.4V）
 * 按 1 / 2 / 3，哪个座上的舵机跟着爪子键动，那根就是爪子的线 —— 不用改代码重下载。 */
static uint8_t g_claw_ch = CLAW_CH;     /* 2 = PA1(CH2)，3 = PA2(CH3)，4 = PA3(CH4) */

/* 舵机总量程：只决定"角度 -> 脉宽"的比例。跟参考工程对齐用 270
 * （源码 servo.c:18 就是 Angle / 270 * 2000 + 500）。 */
#define CLAW_TRAVEL_DEG     270.0f

/* 脉宽换算：0 度 -> 500us，CLAW_TRAVEL_DEG 度 -> 2500us（50Hz，1 格 = 1us）
 * 按 270 算（再减 CLAW_OFFSET_DEG 1 度）：6 度 = 537us，45 度 = 826us。 */
#define CLAW_PULSE_MIN      500.0f
#define CLAW_PULSE_SPAN     2000.0f

/* 爪子也有零点修正：源码 set_zhuashou_Angle() 第一句就是 target -= 1.0f，
 * 抄过来（差 1 度，脉宽约 7us，影响很小，但保持一致好对照）。 */
#define CLAW_OFFSET_DEG     1.0f

/* 机械允许角度上下限。★ 下限必须 ≤ CLAW_CLOSE_DEG（合爪走 0 度）
 * ★ 2026-10-17：上限 180 -> 270，为了实测舵机到底能转到几度。
 *   这条只是软件限位，跟脉宽公式无关：6 度还是 537us、45 度还是 826us，
 *   已标定的开爪/合爪位置一点都不变，只是允许往上继续走。 */
#define CLAW_MIN_DEG        0.0f
#define CLAW_MAX_DEG        270.0f

/* 量程实测专用：l 一次往上探 5 度（走得慢，好观察），z 一次退回 5 度。
 * 探到"角度还在涨、舵机却有嗡嗡声或干脆不转了"就按 z 退回来。 */
#define CLAW_PROBE_STEP_DEG   5.0f
#define CLAW_PROBE_TIME_MS    700u   /* 探一格用多久，越大越慢、越好听声音 */
#define CLAW_BACKOFF_DEG      5.0f

/* 三个常用位置：合爪/夹取 0.0f，开爪/初始 45.0f；中心 90 是试出来的 */
#define CLAW_CENTER_DEG     90.0f
#define CLAW_OPEN_DEG       45.0f
#define CLAW_CLOSE_DEG      0.0f

/* ★ 2026-10-17（晚）：用户定下爪子的两个姿势 ——
 *   初始/默认角 = 45 度（= CLAW_OPEN_DEG，脉宽 = 500 + (45-1)/270*2000 ≈ 826us），
 *   夹取角     = 0 度（= CLAW_CLOSE_DEG，脉宽 ≈ 493us，舵机硬底那一档）。
 *   配合升降 135mm 最上、齿条 0mm 收到底、云台 0 度初始位。
 *   上电时爪子自己走到 45 度；按 W 回默认位也回到 45 度；要夹物块按 c 合到 0 度。 */
#define CLAW_HOME_DEG       CLAW_OPEN_DEG

#define CLAW_STEP_DEG       2.0f     /* , / . 一次走多少度 */
#define MOVE_TIME_MS        400u     /* 一次开合用多久（越小越快越猛） */
#define SWEEP_TIME_MS       600u     /* 自动来回时每一步用多久 */
#define STEP_MS             20u      /* 中间插值的节拍：20ms 一步 */
/* ★ 2026-10-17 加：等舵机走完的上限。正常插值一定会自己收尾，
 *   万一 TIM2 中断没跑到（或某次插值没收尾），到点就强制收尾 ——
 *   绝不能让主循环死在 while 里，那会表现成"按键没反应、舵机也不动"。 */
#define SERVO_WAIT_MAX_MS   5000u      /* 整圈最多约 2.8s，留够余量再判超时 */
#define SWEEP_TIMES         3u       /* 自动来回几趟 */

/* ------------------------ 云台（舵机） -------------------------------- */
/* 云台 = 让整个爪子"转头"的那一路（源码里叫 yuantai，接 PA3 / TIM2_CH4）。
 * 集成板上丝印"云台"的 3 针座，方形焊盘那一脚（1 脚）就是 PA3，所以这里写 4。 */
#define YUNTAI_CH           4

/* ★ 云台这把"尺子"跟爪子不一样！源码 servo.c 三路各一套：
 *     Servo_SetAngle2_zhuashou    : Angle / 270 * 2000 + 500   （爪子）
 *     Servo_SetAngle3_wukuaipingtai: Angle / 270 * 2000 + 500  （物块平台）
 *     Servo_SetAngle4_yuntai      : Angle / 360 * 2000 + 500   ★ 云台是 360
 * 早前这份代码把云台也按 270 算，脉宽算出来偏大（77.4 度多打 143us、278 度顶到 2500us），
 * 所以 y / n 停的位置都不对 —— 10/14 更正。
 * 另外源码 set_yuantai_Angle() 里还有两道修正：
 *     target = target - 7.9f;                    -- 零点偏 7.9 度（★ 我们照抄，见 YUNTAI_OFFSET_DEG）
 *     if (target <= 25.8f) target = 25.8f;       -- 机械下限（★ 我们不要，改成整圈取模）
 */
#define YUNTAI_TRAVEL_DEG   360.0f
#define YUNTAI_PULSE_MIN    500.0f
#define YUNTAI_PULSE_SPAN   2000.0f
#define YUNTAI_OFFSET_DEG   7.9f
/* ★ 2026-10-17（晚）：云台是"单圈 360° 位置舵机"，不是连续旋转舵机。
 *   500us 和 2500us 是内部电位器的两端：物理上相差约 360°（同一个朝向），
 *   但控制器把它们当成"另一头"。所以：
 *     · 0~360 的每个角度都到得了，任意输入按 360 取模；
 *     · 从 194 再往 + 走会落到 -164，舵机必须从 500us 那头一路扫到 2500us
 *       那头，实物表现为"反向甩将近一整圈"——这就是用户 194 度看到的现象；
 *     · 想在接缝处"接着同方向转 2°"物理上做不到，只能整圈扫过去或者退回来。
 *   源码 PWM.c 里 target <= 25.8 那道机械下限我们仍然不要（会锁死往 500us 的行程）。 */

/* ★★ 2026-10-15（晚）：把"初始位"那一档重新定成 0 度 —— 只是换标签，不动舵机。
 *   ★ 方向也定了：朝物块转 = 角度往【正】走（读数取了负号，见下面的 YUNTAI_DIR）。
 *     物理行程一点没变，只是数字不再带负号。
 *   新尺子：  deg = YUNTAI_ZERO_DEG - 旧尺子读数
 *   ⇒ 2026-10-17 又改了一次零点：用户把云台停在新姿势（旧读数 -118.0，脉宽 1586us）
 *     当初始位，所以 ZERO 从 85.4 抬到 203.4 —— 这一档现在读 0.0，舵机一个齿都没动。
 *     （上一版是 ZERO=85.4、那一档读 0、脉宽 931us。两边差 118.0 度，纯换标签。）
 *   ⇒ 2026-10-17 21:2x 再改一次：用户在旧尺子 -70.0 度（脉宽 1975us）这一档要当初始位，
 *     于是 ZERO 203.4 → 273.4，整把尺子平移 +70：这一档读 0.0，抓取位 126 → 196，
 *     退料位 68.4 → 138.4，窗口 -164.5..195.5 → -94.5..265.5。**脉宽全部不变**
 *     （初始仍是 1975us、退料仍是 1206us、两个端点仍是 500/2500us；抓取角 21:3x 另行复标）
 *   换算永远是：  内部量 = deg * YUNTAI_DIR + YUNTAI_ZERO_DEG - YUNTAI_OFFSET_DEG
 *                  脉宽 = 500 + 内部量 / 360 * 2000
 *       deg =    0.0  → 1975us（初始位，2026-10-17 21:2x 标定）
 *       deg =  126.0  → 1275us（抓取位，2026-10-17 21:3x 复标）
 *       deg =  265.5  → 501us（朝物块方向的软限位；舵机硬底 500us）
 *       deg =  138.4  → 1206us（退料角占位）
 *       deg =  -94.5  → 2500us（另一侧软限位）
 *   要改"哪个姿势算 0"只动 YUNTAI_ZERO_DEG；要把方向再翻过来只动 YUNTAI_DIR。 */
#define YUNTAI_ZERO_DEG     273.4f

/* -1 = 朝物块是正角度（现在这样）；+1 = 朝物块是负角度（上一版那样）。
 * 它只影响"数字怎么算"，脉宽和舵机位置不受影响 —— 同一个物理姿势，脉宽永远一样。 */
#define YUNTAI_DIR          (-1.0f)

/* ★★ 2026-10-15 实测发现：我们车的云台方向跟参考车相反 ——
 *   参考工程的抓取角 278/295/261 都在"角度大"那一头；我们车的舵机要往脉冲小的
 *   那一头转才是朝物块。实测旧尺子 85.4（旧初始位）一路走到 33.7，爪子是朝物块靠的。
 *   ⇒ 参考车那四个角（278/295/261/135）在我们车上没有意义，四个都要自己量。
 *   ⇒ 尺子（servo.c 的 Angle/360*2000+500）一个字没改，只在外面套了一层"新读数"。
 *
 *   还有一条：脉宽才是"真身"。同一个物理姿势，不管尺子怎么换，脉宽都不该变 ——
 *   标定时认脉冲 us 最保险（串口 p 每次都打）。 */

/* 整圈窗口（新尺子：初始位 = 0，朝物块为正）：
 *     MAX = 265.5 → 500us
 *     MIN = -94.5 → 2500us
 *   这两个数把一整圈 360° 刚好铺满，但两头是同一颗单圈电位器的两端：
 *   物理朝向相同、控制器却把它们当成"另一头"。实测从 194 再按 g，
 *   舵机会反向扫将近 358° 才到 -164 —— 不是软件限位，是接缝本身。
 *   ★ 所以 g / k / / / \ 到端点就停（Yuntai_Nudge），绝不取模跨过去；
 *     i 输入仍可显式要求跨到另一头，但那会是一整圈扫过去（会先打印 WARN）。
 *   ★ 输入按 360 取模：i200# → -160 度（200-360），i360# → 0 度（回初始位）。
 *   ★ 程序只保证不发 500us 以下 / 2500us 以上。 */
#define YUNTAI_MIN_DEG      (-94.5f)
#define YUNTAI_MAX_DEG      265.5f

/* 云台量程实测专用（只为了量行程，平时抓取不用）：/ 往物块探 5 度，\ 退回 5 度 */
#define YUNTAI_PROBE_STEP_DEG   5.0f
#define YUNTAI_PROBE_MS         700.0f   /* 探一格用多久，慢一点好判断舵机到底转没转 */

/* 云台常用位置（新尺子：初始位 = 0）。
 * ★ YUNTAI_INIT_DEG = 0 已在我们车上标定：2026-10-17 21:2x 用户指定的新初始姿势，
 *   脉宽 1975us（重设零点前的读数 -70.0）。
 *   按下 y 之后串口应该打印 pulse = 1975us —— 拿这个数复核"初始位没跑偏"。
 *   ★ 上电时软件默认认为自己就在这个姿势；实际要按一次 y 才是真的到位。 */
#define YUNTAI_INIT_DEG     0.0f

/* ★ 抓取角：2026-10-17 21:3x 现场复标 = 126.0 度（从初始位 0 往物块那边转）。
 *   对应串口 [yuntai] now = 126.0 deg, pulse = 1275us。
 *   （更早一版是 196.0 度 / 886us，那个姿势不是现在要抓的位置，已作废。）
 *   n 键、4/5/6 原项目流程、一键夹取都用这一个角；换场地摆放变了要重新量。
 *   三个颜色真要是三个角，再拆成 YUNTAI_GRAB1/2/3_DEG。 */
#define YUNTAI_GRAB_DEG     126.0f

/* ★ 退料/放料角：源码 135 度（旧尺子），我们车也没标。占位 138.4（脉宽 1206us），
 *   实测再改。 */
#define YUNTAI_BACK_DEG     138.4f

#define YUNTAI_STEP_DEG     2.0f     /* g / k 一次走多少度 */

/* ---- 一键夹取用的实测姿势（2026-10-17 用户现场标定的一组）----
 *   平推 65mm、升降 110mm、云台 126 度（从初始位 0 往物块那边转 126 度）。
 *   爪子闭合 0 度（CLAW_CLOSE_DEG = 夹取角），初始/张开 45 度（CLAW_OPEN_DEG）。
 *   对应串口：arm now: lift 110.0mm push 65.0mm / [claw] now = 6.0 deg /
 *            [yuntai] now = 126.0 deg。
 *   ★ 这几个数是"我们这台车"的，换场地摆放位置变了要重新量。 */
#define QUICK_GRAB_PUSH_MM    65.0f
#define QUICK_GRAB_LIFT_MM    110.0f
#define QUICK_GRAB_YUNTAI_DEG  126.0f

/* ---- 一键"放上载物台"用的实测姿势（2026-10-17 用户现场标定）----
 *   载物台那一档：平推 30mm、升降 110mm、云台 254 度（脉宽 564us）。
 *   动作顺序按用户要求：夹着物块先升到最高点，再转到载物台角度，最后下放、开爪。 */
#define QUICK_PLACE_PUSH_MM    30.0f
#define QUICK_PLACE_LIFT_MM    110.0f
#define QUICK_PLACE_YUNTAI_DEG 254.0f

/* ★ 云台速度总开关：所有云台动作的时间都乘这个数。
 *   1.0 = 源码原速；2.0 = 慢一倍（现在这个值）；越大越慢，越小越快越猛。
 *   手动键（y / n / g / k / i）和三轴联动（4/5/6 抓取、m 手输）都吃这一个数 ——
 *   觉得还是快就往上加 0.5，觉得磨叽就往下减。（改完要重新编译下载） */
#define YUNTAI_SPEED_DIV    2.0f
#define YUNTAI_MIN_MOVE_MS  150u     /* 最短动作时间：转一两度也至少走这么久，防抖 */

#define UART_BAUD           115200


/* ==========================================================================
 *  ② 串口（USART3：PB10 = TX，PB11 = RX）
 *  ★ 2026-10-14 改正：这版原来是 USART1（PA9/PA10），但集成板上无线调试器
 *    实际接的是 U4 的 2/3 脚 = PB11/PB10，也就是 USART3。跑得通的 main_lap.c
 *    用的就是 USART3 —— 这就是"下载完串口一声不吭"的根因。
 * ========================================================================== */
static void MyUART_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

    USART_InitStructure.USART_BaudRate            = UART_BAUD;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART3, &USART_InitStructure);

    USART_Cmd(USART3, ENABLE);
}

static void UART_SendByte(uint8_t b)
{
    USART_SendData(USART3, b);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

static void USART_SendString(char *s)
{
    while (*s != '\0') { UART_SendByte((uint8_t)*s); s++; }
}

static void UART_SendInt(int32_t n)
{
    char     buf[12];
    uint8_t  i = 0;
    uint32_t u;

    if (n < 0) { UART_SendByte('-'); u = (uint32_t)(-n); }
    else       { u = (uint32_t)n; }

    if (u == 0) { UART_SendByte('0'); return; }

    while (u > 0) { buf[i++] = (char)('0' + (u % 10)); u = u / 10; }
    while (i > 0) { UART_SendByte((uint8_t)buf[--i]); }
}

/* 打印 1 位小数，带负号（例如 92.0 / -12.5） */
static void UART_SendFix1(float v)
{
    int32_t x;

    if (v < 0.0f) { UART_SendByte('-'); v = -v; }
    x = (int32_t)(v * 10.0f + 0.5f);
    UART_SendInt(x / 10);
    UART_SendByte('.');
    UART_SendByte((uint8_t)('0' + (x % 10)));
}

/* 打印 mm，保留 1 位小数 */
static void UART_SendMm(float mm)
{
    UART_SendFix1(mm);
    USART_SendString("mm");
}

static uint8_t UART_GetByte(void)
{
    if (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET)
        return (uint8_t)USART_ReceiveData(USART3);
    return 0;
}


/* ==========================================================================
 *  ③ 机械臂：位置模式 + 软件账本
 * ========================================================================== */

static float    g_lift_mm = LIFT_HOME_MM;      /* 软件账本：上电默认在最高处（mm） */
static float    g_push_mm = 0.0f;              /* 软件账本：平推现在在哪（mm） */
static uint16_t g_speed   = MOVE_SPEED;
static uint16_t g_step    = STEP_DEFAULT;

static void Show_ArmState(void)
{
    USART_SendString("   arm now: lift ");
    UART_SendMm(g_lift_mm);
    USART_SendString(" (addr ");
    UART_SendInt(ADDR_LIFT);
    USART_SendString(")   push ");
    UART_SendMm(g_push_mm);
    USART_SendString(" (addr ");
    UART_SendInt(ADDR_PUSH);
    USART_SendString(")   step ");
    UART_SendMm((float)g_step);
    USART_SendString("   speed ");
    UART_SendInt(g_speed);
    USART_SendString(" rpm\r\n");
}

/* 源码 yyb_move.c 的 shengjiang_move_delay()：只算"走这么多脉冲要多久"，
 * 不真 delay（源码里那句 delay_ms1 是注释掉的，这里照抄算法）。 */
static float Axis_MoveTimeMs(uint32_t pulses, uint16_t accel, uint16_t speed)
{
    double a = (accel == 0u) ? 256.0 : (double)accel;
    double quanshu = (double)pulses;
    double fenjie = 320.0 * (double)speed * (double)speed / (a * 6.0);
    double time_fenjie = 2000.0 * (double)speed / a;
    double t;

    if (fenjie >= quanshu)
        t = time_fenjie * sqrt(quanshu / fenjie);
    else
        t = 1000.0 * (quanshu - fenjie) / (320.0 * (double)speed / 6.0) + time_fenjie;

    t *= 0.5;
    if (t <= 200.0) t = 200.0;      /* 源码注释：最少也要等 200ms */
    return (float)t;
}

/* 走到绝对位置 target_mm（正数 = 名义正方向），超程 clamp。
 * 发完 Emm_V5_Pos_Control 就返回（电机自己走，非阻塞），软件记账。
 * out_ms 非 NULL 时回传"预计走多久"（给 move_all 那种联动等时间用）。
 * 账本没变（本来就在目标上）时不动电机，*out_ms 给 0。 */
static void Axis_MoveTo(uint8_t addr, float ppm, float *cur,
                        float lo, float hi, uint8_t dir_inv,
                        float target_mm, uint16_t accel, uint16_t speed,
                        float *out_ms)
{
    float    d;
    float    ad;
    uint32_t pulses;
    uint8_t  dir;

    if (out_ms != NULL) { *out_ms = 0.0f; }

    if (target_mm < lo) { target_mm = lo; }
    if (target_mm > hi) { target_mm = hi; }
    if (target_mm == *cur) { return; }

    d  = target_mm - *cur;
    ad = (d < 0.0f) ? -d : d;

    pulses = (uint32_t)(ad * ppm + 0.5f);
    if (pulses == 0u) { *cur = target_mm; return; }

    dir = (d > 0.0f) ? 0 : 1;
    if (dir_inv) { dir = (uint8_t)(dir ? 0 : 1); }

    Emm_V5_Pos_Control(addr, dir, speed, accel, pulses, false, false);

    *cur = target_mm;   /* 软件记账 —— 和源码一样，不是读编码器回来的 */

    if (out_ms != NULL) { *out_ms = Axis_MoveTimeMs(pulses, accel, speed); }
}

/* 走 delta 毫米（按键点动用）：返回实际走的毫米数 */
static float Axis_Move(uint8_t addr, float ppm, float *cur, float lo, float hi,
                       uint8_t dir_inv, float delta)
{
    float from = *cur;
    float target = *cur + delta;

    if (target < lo) { target = lo; }
    if (target > hi) { target = hi; }

    Axis_MoveTo(addr, ppm, cur, lo, hi, dir_inv, target, MOVE_ACC, g_speed, NULL);
    return target - from;
}

static void Do_Lift(int8_t sign)
{
    float moved = Axis_Move(ADDR_LIFT, PPM_LIFT, &g_lift_mm,
                            LIFT_MIN_MM, LIFT_MAX_MM, LIFT_DIR_INV,
                            (float)sign * (float)g_step);

    USART_SendString("lift  ");
    UART_SendMm(moved);
    USART_SendString(" -> ");
    UART_SendMm(g_lift_mm);
    USART_SendString("\r\n");
    Show_ArmState();
}

static void Do_Push(int8_t sign)
{
    float moved = Axis_Move(ADDR_PUSH, PPM_PUSH, &g_push_mm,
                            PUSH_MIN_MM, PUSH_MAX_MM, PUSH_DIR_INV,
                            (float)sign * (float)g_step);

    USART_SendString("push  ");
    UART_SendMm(moved);
    USART_SendString(" -> ");
    UART_SendMm(g_push_mm);
    USART_SendString("\r\n");
    Show_ArmState();
}

/* 平推一键回初始位：照账本走到 0（= 齿条收到底、后端顶住）。
 *
 * ★ 这不是"回零/找限位"，只是把账本上的数字走到 0：
 *   账本本身错位（比如按过 RST 但齿条没摆）时，按这个键什么也不会发生 —— 因为账面已经是 0，
 *   Axis_MoveTo() 看目标等于当前位置就直接 return。那种情况要：
 *       按 d 失能 → 手把齿条推到最后端顶住 → 按 0 清零 → 按 e 使能。
 *   速度跟手动点动同一个数（按键 [ / ] 调），不是源码 move_all 那个 1000rpm。 */
static void Do_PushHome(void)
{
    USART_SendString("push -> home (0mm = rack fully back)\r\n");

    Axis_MoveTo(ADDR_PUSH, PPM_PUSH, &g_push_mm,
                PUSH_MIN_MM, PUSH_MAX_MM, PUSH_DIR_INV,
                PUSH_MIN_MM, MOVE_ACC, g_speed, NULL);

    Show_ArmState();
}

static void Arm_StepChange(int16_t delta)
{
    int16_t s = (int16_t)g_step + delta;

    if (s < STEP_MIN) { s = STEP_MIN; }
    if (s > STEP_MAX) { s = STEP_MAX; }
    g_step = (uint16_t)s;

    USART_SendString("arm step = ");
    UART_SendMm((float)g_step);
    USART_SendString("\r\n");
}

static void Arm_SpeedChange(int16_t delta)
{
    int16_t s = (int16_t)g_speed + delta;

    if (s < MOVE_SPEED_MIN) { s = MOVE_SPEED_MIN; }
    if (s > MOVE_SPEED_MAX) { s = MOVE_SPEED_MAX; }
    g_speed = (uint16_t)s;

    USART_SendString("arm speed = ");
    UART_SendInt(g_speed);
    USART_SendString(" rpm (next move uses it)\r\n");
}

static void Arm_StopBoth(void)
{
    Emm_V5_Stop_Now(ADDR_LIFT, false);
    Emm_V5_Stop_Now(ADDR_PUSH, false);
    USART_SendString("STOP arm both\r\n");
    Show_ArmState();
}

static void Arm_EnableBoth(bool state)
{
    Emm_V5_En_Control(ADDR_LIFT, state, false);
    Emm_V5_En_Control(ADDR_PUSH, state, false);

    USART_SendString(state ? "arm enable" : "arm disable");
    USART_SendString(" both\r\n");
    Show_ArmState();
}

static void Arm_ZeroBook(void)
{
    /* 注意：这会把"当前高度"认成 0 = 量程最低点。
     * 只是对零用；对完再按 u 往上，j 才有下行的余地。 */
    g_lift_mm = 0.0f;
    g_push_mm = 0.0f;
    USART_SendString("arm book zeroed -> now = LOWEST point (motors NOT moved)\r\n");
    Show_ArmState();
}


/* ==========================================================================
 *  ④ 爪子：角度 -> 脉宽 -> CCR
 * ========================================================================== */

/* ★ 10/14 起：爪子"当前角"不再自己记账，直接读源码 PWM.c 的 Servo_t 结构体。
 * 为什么：真正写给舵机的值是 zhuashou.current_angle（TIM2 中断里的 servo_update()
 * 每 20ms 更新一次，然后调 zhuashou.SetServoAngle 写 CCR）。我们再记一份，
 * 就一定会有"两份对不上"的时候。
 * 两把尺子的换算（源码 set_zhuashou_Angle 第一句就是 target -= 1.0f）：
 *      结构体内部角 = 我们外面这把尺子的角 - CLAW_OFFSET_DEG
 * 例：外面 45 度（开爪）-> 内部 44.0 -> CCR = 44/270*2000+500 = 826us。 */
static float Claw_CurrentDeg(void)
{
    return zhuashou.current_angle + CLAW_OFFSET_DEG;
}

/* 把结构体直接摆到某个"外面的角"（只对齐账本，不产生任何动作）。
 * 上电初始化用；别拿它当"移动"用 —— 移动走 Claw_MoveTo()。 */
static void Claw_SetDegState(float deg)
{
    float inner = deg - CLAW_OFFSET_DEG;

    zhuashou.current_angle = inner;
    zhuashou.start_angle   = inner;
    zhuashou.target_angle  = inner;
}

/* 通道号 -> 引脚名（打印用；2 = PA1，3 = PA2，4 = PA3） */
static char *Pwm_PinName(uint8_t ch)
{
    if (ch == 2u) return "PA1";
    if (ch == 3u) return "PA2";
    if (ch == 4u) return "PA3";
    return "PA?";
}

/* 往任意一路写 CCR；写 0 = 整周期低电平 = 不发脉冲 = 舵机松力 */
static void Pwm_WriteCh(uint8_t ch, uint16_t ccr)
{
    if      (ch == 2u) PWM_SetCompare2(ccr);
    else if (ch == 3u) PWM_SetCompare3(ccr);
    else if (ch == 4u) PWM_SetCompare4(ccr);
}

/* 把 CCR 值写到爪子这一路（哪一路由 g_claw_ch 决定，可以用 1/2/3 现场切） */
static void Claw_WriteCompare(uint16_t ccr)
{
    Pwm_WriteCh(g_claw_ch, ccr);
}

static uint16_t Claw_PulseOf(float deg)
{
    float p;

    if (deg < CLAW_MIN_DEG) deg = CLAW_MIN_DEG;
    if (deg > CLAW_MAX_DEG) deg = CLAW_MAX_DEG;

    p = CLAW_PULSE_MIN + (deg - CLAW_OFFSET_DEG) / CLAW_TRAVEL_DEG * CLAW_PULSE_SPAN;
    if (p < 1.0f)     p = 1.0f;
    if (p > 19999.0f) p = 19999.0f;

    return (uint16_t)(p + 0.5f);
}

/* 按"该通道自己的尺子"往这一路写一个内部角：直接借源码 servo.c 的三个换算函数。
 *   ch2 -> Servo_SetAngle2_zhuashou      （/270*2000+500）
 *   ch3 -> Servo_SetAngle3_wukuaipingtai （/270*2000+500）
 *   ch4 -> Servo_SetAngle4_yuntai        （/360*2000+500，云台是 360）
 * 用了这个函数，爪子/载物台/云台三路就永远是"同一套换算"，不会各算各的。 */
static void Ch_WriteInner(uint8_t ch, float inner_deg)
{
    if      (ch == 2u) Servo_SetAngle2_zhuashou(inner_deg);
    else if (ch == 3u) Servo_SetAngle3_wukuaipingtai(inner_deg);
    else if (ch == 4u) Servo_SetAngle4_yuntai(inner_deg);
}

static void Claw_Release(void)
{
    zhuashou.moving_flag = 0;       /* 让 TIM2 中断别再插值（不然马上又被写回去） */
    zhuashou.start_angle = zhuashou.current_angle;   /* 半路松的力：下次从"停下来的角"起算 */
    Claw_WriteCompare(0u);          /* CCR = 0：这一路不发脉冲，舵机松力，可以用手掰 */
}

static void Claw_Report(char *tag, float deg)
{
    USART_SendString("[claw] ");
    USART_SendString(tag);
    USART_SendString(" deg=");
    UART_SendFix1(deg);
    USART_SendString(" pulse=");
    UART_SendInt((int32_t)Claw_PulseOf(deg));
    USART_SendString("us\r\n");
}

/* 下目标角、不等待（等价于源码 set_zhuashou_Angle 去掉里面那个死等）。
 * 零点修正照抄：结构体内部角 = 外面的角 - CLAW_OFFSET_DEG(1 度)。
 * S 曲线插值仍然交给 TIM2 中断里的 servo_update()，和源码一模一样。 */
static void Claw_Goto(float target, float time_ms)
{
    if (target < CLAW_MIN_DEG) target = CLAW_MIN_DEG;
    if (target > CLAW_MAX_DEG) target = CLAW_MAX_DEG;
    if (time_ms < (float)STEP_MS) time_ms = (float)STEP_MS;

    zhuashou.target_angle = target - CLAW_OFFSET_DEG;
    zhuashou.time         = (int)time_ms;
    zhuashou.moving_flag  = 1;
}

/* 等插值收尾，但最多等 SERVO_WAIT_MAX_MS。
 * ★ 2026-10-17：源码/我们自己以前都是 while(moving_flag){} 死等，
 *   一旦中断没把 flag 清掉，整块板子就哑了（串口不回、键也不认）。
 *   现在超时就把 flag 强制清零，把控制权还给主循环。 */
static uint8_t Claw_WaitDone(void)
{
    uint32_t waited = 0u;

    while (zhuashou.moving_flag != 0)
    {
        delay_ms1(STEP_MS);
        waited += STEP_MS;
        if (waited >= SERVO_WAIT_MAX_MS)
        {
            zhuashou.moving_flag = 0;
            zhuashou.start_angle = zhuashou.current_angle;
            USART_SendString("[claw] wait timeout, force stop\r\n");
            return 0u;
        }
    }
    return 1u;
}

/* 移动爪子并等它停稳（和原来行为一致，只是死等换成了有上限的等）。 */
static void Claw_MoveTo(float target, uint32_t ms)
{
    if (ms < STEP_MS) ms = STEP_MS;

    Claw_Goto(target, (float)ms);
    Claw_WaitDone();
    delay_ms1(30);      /* 源码 set_zhuashou_Angle 结尾就有这句，照抄 */
}

/* ★ 2026-10-17：开/合爪必须走我们自己的尺子。
 *   Hardware/PWM.c 里的 set_zhuashou_he() / set_zhuashou_kai() 把角度写死成
 *   6.0f / 45.0f（改 CLAW_CLOSE_DEG 对它们无效），而且里面是无上限的
 *   while(moving_flag){} 死等 —— 这两点都不再用，统一从这儿走。 */
static void Claw_CloseHold(void)
{
    Claw_MoveTo(CLAW_CLOSE_DEG, MOVE_TIME_MS);
}

static void Claw_OpenWide(void)
{
    Claw_MoveTo(CLAW_OPEN_DEG, MOVE_TIME_MS);
}

static void Claw_ShowParams(void)
{
    float cur = Claw_CurrentDeg();

    USART_SendString("[claw] channel = TIM2_CH");
    UART_SendInt((int32_t)g_claw_ch);
    USART_SendString(" (");
    USART_SendString(Pwm_PinName(g_claw_ch));
    USART_SendString(")");
    if (g_claw_ch == YUNTAI_CH) USART_SendString("  <-- !! same pin as yuntai");
    USART_SendString("\r\n");

    USART_SendString("[claw] travel = ");
    UART_SendFix1(CLAW_TRAVEL_DEG);
    USART_SendString(" deg -> pulse ");
    UART_SendInt((int32_t)CLAW_PULSE_MIN);
    USART_SendString("..");
    UART_SendInt((int32_t)(CLAW_PULSE_MIN + CLAW_PULSE_SPAN));
    USART_SendString("us\r\n");

    USART_SendString("[claw] limits = ");
    UART_SendFix1(CLAW_MIN_DEG);
    USART_SendString(" .. ");
    UART_SendFix1(CLAW_MAX_DEG);
    USART_SendString(" deg  (edit CLAW_MIN_DEG / CLAW_MAX_DEG)\r\n");

    USART_SendString("[claw] open/close/center = ");
    UART_SendFix1(CLAW_OPEN_DEG);
    USART_SendString(" / ");
    UART_SendFix1(CLAW_CLOSE_DEG);
    USART_SendString(" / ");
    UART_SendFix1(CLAW_CENTER_DEG);
    USART_SendString("\r\n");

    USART_SendString("[claw] now = ");
    UART_SendFix1(cur);
    USART_SendString(" deg, pulse = ");
    UART_SendInt((int32_t)Claw_PulseOf(cur));
    USART_SendString("us\r\n");
}


/* ==========================================================================
 *  ④b 云台：角度 -> 脉宽 -> CCR4（PA3）
 * ========================================================================== */

/* ★ 2026-10-17（晚）：把任意角度折进整圈窗口，不再截断。
 *   MIN..MAX 正好是 360°，所以：
 *     i200#  → -160 度（200-360），i360# → 0 度，i-200# → 160 度；
 *     0..360 每个输入都能落到一个真实位置，360/720 与 0 等价。
 *   注意：这是"一圈取模"，不是连续多圈 —— 普通位置舵机只有一个 360° 周期。 */
static float Yuntai_WrapDeg(float deg)
{
    float rel;
    int32_t turns;

    if (deg != deg) return YUNTAI_INIT_DEG;                 /* NaN 兜底 */
    if (deg >= YUNTAI_MIN_DEG && deg <= YUNTAI_MAX_DEG)     /* 含两个硬底端点 */
        return deg;

    rel   = (deg - YUNTAI_MIN_DEG) / YUNTAI_TRAVEL_DEG;
    turns = (int32_t)rel;                                   /* C 向 0 取整 */
    if (rel < 0.0f) turns -= 1;                             /* 负数改成向下取整 */

    deg -= (float)turns * YUNTAI_TRAVEL_DEG;
    if (deg < YUNTAI_MIN_DEG) deg += YUNTAI_TRAVEL_DEG;     /* 浮点兜底 */
    if (deg > YUNTAI_MAX_DEG) deg -= YUNTAI_TRAVEL_DEG;
    return deg;
}

/* 和爪子一样：云台当前角也直接读源码结构体（yuantai.current_angle），
 * 它已经被 set_yuantai_Angle 减过 7.9 的零点修正，所以对外要加回来；
 * 再减掉 YUNTAI_ZERO_DEG，才是"以初始位为 0"的新尺子。
 *      结构体内部角 = 旧尺子读数 - YUNTAI_OFFSET_DEG
 *      新尺子读数   = (结构体内部角 + YUNTAI_OFFSET_DEG - YUNTAI_ZERO_DEG) * YUNTAI_DIR */
static float Yuntai_CurrentDeg(void)
{
    return (yuantai.current_angle + YUNTAI_OFFSET_DEG - YUNTAI_ZERO_DEG) * YUNTAI_DIR;
}

/* 只对齐账本 / 初始化用：把结构体摆到某个"外面的角"，不产生动作。 */
static void Yuntai_SetDegState(float deg)
{
    float inner;

    deg   = Yuntai_WrapDeg(deg);
    inner = deg * YUNTAI_DIR + YUNTAI_ZERO_DEG - YUNTAI_OFFSET_DEG;
    if (inner < 0.0f)              inner = 0.0f;
    if (inner > YUNTAI_TRAVEL_DEG) inner = YUNTAI_TRAVEL_DEG;

    yuantai.current_angle = inner;
    yuantai.start_angle   = inner;
    yuantai.target_angle  = inner;
}

/* 写 0 = 整周期低电平 = 不发脉冲 = 舵机松力 */
static void Yuntai_WriteCompare(uint16_t ccr)
{
    PWM_SetCompare4(ccr);
}

static uint16_t Yuntai_PulseOf(float deg)
{
    float cmd;
    float p;

    deg = Yuntai_WrapDeg(deg);     /* ★ 任意角先折进整圈，不截断 */

    /* 换算回源码那套内部量：cmd = 旧尺子角 - 7.9 = 新尺子角 * DIR + YUNTAI_ZERO_DEG - 7.9 */
    cmd = deg * YUNTAI_DIR + YUNTAI_ZERO_DEG - YUNTAI_OFFSET_DEG;
    if (cmd < 0.0f)               cmd = 0.0f;                 /* 舵机硬底：500us */
    if (cmd > YUNTAI_TRAVEL_DEG)  cmd = YUNTAI_TRAVEL_DEG;    /* 舵机硬顶：2500us */

    p = YUNTAI_PULSE_MIN + cmd / YUNTAI_TRAVEL_DEG * YUNTAI_PULSE_SPAN;
    if (p < YUNTAI_PULSE_MIN) p = YUNTAI_PULSE_MIN;
    if (p > YUNTAI_PULSE_MIN + YUNTAI_PULSE_SPAN)
        p = YUNTAI_PULSE_MIN + YUNTAI_PULSE_SPAN;

    return (uint16_t)(p + 0.5f);
}

static void Yuntai_Release(void)
{
    yuantai.moving_flag = 0;        /* 让 TIM2 中断别再插值 */
    yuantai.start_angle = yuantai.current_angle;
    Yuntai_WriteCompare(0u);
}

/* 云台"要转这么多度用多久"：照抄源码 PWM.c 的 move_all() 那套分段公式，
 * 13 = 源码里传进去的云台转速档（speed_of_yuntai）。 */
#define YUNTAI_RATE   13.0f

static float Yuntai_MoveTime(float delta_deg)
{
    float d = (delta_deg < 0.0f) ? -delta_deg : delta_deg;

    if (d <= 28.0f)  return 120.0f * d / YUNTAI_RATE;
    if (d <= 50.0f)  return 100.0f * d / YUNTAI_RATE;
    if (d <= 100.0f) return 85.0f  * d / YUNTAI_RATE;
    if (d <= 150.0f) return 70.0f  * d / YUNTAI_RATE;
    if (d <= 250.0f) return 60.0f  * d / YUNTAI_RATE;
    return 50.0f * d / YUNTAI_RATE;
}

/* 云台"实际"要走多久 = 源码那套分段公式 × 慢速倍数，再垫一个最短时间。
 * 手动键（y/n/g/k/i）和 move_all（4/5/6 抓取、m 手输）都走这里，
 * 改 YUNTAI_SPEED_DIV 一处就能整体调速。 */
static float Yuntai_SlowTime(float delta_deg)
{
    float t = Yuntai_MoveTime(delta_deg) * YUNTAI_SPEED_DIV;

    if (t < (float)YUNTAI_MIN_MOVE_MS) { t = (float)YUNTAI_MIN_MOVE_MS; }
    return t;
}

/* 给云台下目标角（不等待）：直接调源码 PWM.c 的 set_yuantai_Angle()。
 * ★ 2026-10-15 改：不再走那个函数，理由见下。 */
static void Yuntai_Goto(float target, float time_ms)
{
    target = Yuntai_WrapDeg(target);   /* ★ 不再截断，整圈取模 */
    if (time_ms < 100.0f) time_ms = 100.0f;

    /* ★ 不走 PWM.c 的 set_yuantai_Angle()：那里面有参考车的一道机械下限
     *   （target -= 7.9f; if (target <= 25.8f) target = 25.8f;  → 脉宽永远 ≥ 643us），
     *   会把我们往物块方向的行程锁死。这里改成直接填结构体，把为零点修正自己减掉；
     *   插值 / S 曲线照样由 TIM2 中断里的 servo_update() 来做，机制和源码完全一样，
     *   只是没有那道闸。 */
    yuantai.target_angle = target * YUNTAI_DIR + YUNTAI_ZERO_DEG - YUNTAI_OFFSET_DEG;
    yuantai.time         = (int)time_ms;
    yuantai.moving_flag  = 1;
    /* 这里故意不写 yuantai.current_angle：和源码一样，让中断里的 servo_update()
     * 一步步把它插值过去，这样 Yuntai_CurrentDeg() 读到的才是"现在真正在哪"。 */
}

static void Yuntai_Report(char *tag, float deg)
{
    USART_SendString("[yuntai] ");
    USART_SendString(tag);
    USART_SendString(" deg=");
    UART_SendFix1(deg);
    USART_SendString(" pulse=");
    UART_SendInt((int32_t)Yuntai_PulseOf(deg));
    USART_SendString("us\r\n");
}

/* 转云台并等它停稳（串口按 y / n / g / k / i 手动用）。
 * 时间按"要转多少度"算（Yuntai_SlowTime），不是固定值 —— 转得远就慢慢转，
 * 微调一两度就快进快出，不会每次都磨 400ms。
 * 动作流程里用 Yuntai_Goto()，不等 —— 要和平推/升降一起动。 */
/* 等云台插值收尾，同样带超时上限（理由见 Claw_WaitDone）。 */
static uint8_t Yuntai_WaitDone(void)
{
    uint32_t waited = 0u;

    while (yuantai.moving_flag != 0)
    {
        delay_ms1(STEP_MS);
        waited += STEP_MS;
        if (waited >= SERVO_WAIT_MAX_MS)
        {
            yuantai.moving_flag  = 0;
            yuantai.start_angle  = yuantai.current_angle;
            USART_SendString("[yuntai] wait timeout, force stop\r\n");
            return 0u;
        }
    }
    return 1u;
}

/* 云台移动：target 先按 360 取模折进 MIN..MAX，然后按窗口内的实际差值走。
 * ★ 2026-10-17（晚）撤掉"接缝跳变"：实测 194 → -164 时舵机反向甩了将近一圈，
 *   说明 500us / 2500us 不是可以无缝跨过的同一点，而是单圈电位器的两端。
 *   这里绝不再直接改账本/跳脉宽，只做一段连续的插值。
 * time_ms <= 0 → 按距离自动算时间；time_ms > 0 → 用这个固定时间。 */
static void Yuntai_MoveToT(float target, float time_ms)
{
    float cur   = Yuntai_CurrentDeg();
    float tgt   = Yuntai_WrapDeg(target);
    float dist  = tgt - cur;
    if (dist < 0.0f) dist = -dist;

    if (dist < 0.05f) return;   /* 已经到目标 */

    /* 只有显式的 i195.5# / i-164.5# 这种"同一朝向的另一头"才会走满一圈。
     * 先大声提示，避免用户以为它坏了；这一段是连续插值，不是跳变。 */
    if (dist > (YUNTAI_TRAVEL_DEG - 5.0f))
    {
        USART_SendString("[yuntai] WARN other end of single-turn pot: ");
        UART_SendFix1(dist);
        USART_SendString(" deg full sweep, keep hands clear\r\n");
    }

    Yuntai_Goto(tgt, (time_ms > 0.0f) ? time_ms : Yuntai_SlowTime(dist));
    Yuntai_WaitDone();
}

static void Yuntai_MoveTo(float target)
{
    Yuntai_MoveToT(target, -1.0f);
}

/* g / k / / / \ 这种"在当前角上加减一小步"的移动。
 * ★ 关键：先加步长再判断是否超出 MIN..MAX，超了就停在端点，绝不取模。
 *   取模会把 196 变成 -164，舵机就得从 500us 那头扫到 2500us 那头，
 *   实物就是用户看到的"反向转 360°"。 */
static void Yuntai_Nudge(float delta, float time_ms)
{
    float cur = Yuntai_CurrentDeg();
    float tgt = cur + delta;
    uint8_t at_max = 0u;
    uint8_t at_min = 0u;

    if (tgt >= YUNTAI_MAX_DEG)
    {
        tgt    = YUNTAI_MAX_DEG;
        at_max = 1u;
    }
    else if (tgt <= YUNTAI_MIN_DEG)
    {
        tgt    = YUNTAI_MIN_DEG;
        at_min = 1u;
    }

    if (tgt > cur + 0.05f || tgt < cur - 0.05f)
        Yuntai_MoveToT(tgt, time_ms);

    if (at_max)
        USART_SendString("[yuntai] at 500us end; press k to go back, or * to test which servo type this is\r\n");
    else if (at_min)
        USART_SendString("[yuntai] at 2500us end; press g to go back, or * to test which servo type this is\r\n");
}

static void Yuntai_ShowParams(void)
{
    float cur = Yuntai_CurrentDeg();

    USART_SendString("[yuntai] channel = TIM2_CH");
    UART_SendInt((int32_t)YUNTAI_CH);
    USART_SendString(" (PA3, board silk = yuntai header pin1)\r\n");

    USART_SendString("[yuntai] circle = full 360 deg (500..2500us); any input angle wraps into ");
    UART_SendFix1(YUNTAI_MIN_DEG);
    USART_SendString(" .. ");
    UART_SendFix1(YUNTAI_MAX_DEG);
    USART_SendString(" deg\r\n");
    USART_SendString("[yuntai] NOTE single-turn pot: 500/2500us are its two ends; g/k stop there, ");
    USART_SendString("crossing them means a ~360 deg sweep\r\n");

    USART_SendString("[yuntai] init/grab/back = ");
    UART_SendFix1(YUNTAI_INIT_DEG);
    USART_SendString(" / ");
    UART_SendFix1(YUNTAI_GRAB_DEG);
    USART_SendString(" / ");
    UART_SendFix1(YUNTAI_BACK_DEG);
    USART_SendString(" deg (new ruler: init = 0, towards-block = +, zero@");
    UART_SendFix1(YUNTAI_ZERO_DEG);
    USART_SendString(" old-deg, dir = ");
    UART_SendFix1(YUNTAI_DIR);
    USART_SendString(", offset -");
    UART_SendFix1(YUNTAI_OFFSET_DEG);
    USART_SendString(")\r\n");

    /* 脉宽才是"真身"：换代码 / 改尺子都不该改变同一个物理姿势的脉宽。 */
    USART_SendString("[yuntai] pulse: init = ");
    UART_SendInt((int32_t)Yuntai_PulseOf(YUNTAI_INIT_DEG));
    USART_SendString("us, back = ");
    UART_SendInt((int32_t)Yuntai_PulseOf(YUNTAI_BACK_DEG));
    USART_SendString("us, ");

    USART_SendString("now = ");
    UART_SendFix1(cur);
    USART_SendString(" deg, pulse = ");
    UART_SendInt((int32_t)Yuntai_PulseOf(cur));
    USART_SendString("us\r\n");
}


/* ==========================================================================
 *  ④c 源码搬运：move_all() 三轴联动 + yuantai_na() 抓取流程
 * ========================================================================== */

/* ★ 2026-10-17 新增：舵机"硬测试"（串口按 ` 或 ~）。
 *   这条路径完全不碰账本、完全不等 TIM2 中断，直接往 CCR 写固定脉宽 ——
 *   PWM 波形是定时器硬件出的，只要板子有电、线没掉，舵机就一定会动。
 *   用途：分清楚"程序/插值卡住"还是"舵机没电 / 线松了"。
 *     有动作 = 供电、信号、舵机都没问题，问题在插值那一套（已修）。
 *     不动   = 查绿色 5V、共地、信号线插头。 */
static void Servo_HardTest(void)
{
    uint16_t pc = Claw_PulseOf(CLAW_HOME_DEG);        /* 45 度 ≈ 826us */
    uint16_t py = Yuntai_PulseOf(YUNTAI_INIT_DEG);    /* 1975us */

    zhuashou.moving_flag = 0;       /* 别让中断插值抢着写同一个 CCR */
    yuantai.moving_flag  = 0;

    USART_SendString("[hard] claw ");
    USART_SendString(Pwm_PinName(g_claw_ch));
    USART_SendString(" -> ");
    UART_SendInt((int32_t)pc);
    USART_SendString("us\r\n");
    Claw_WriteCompare(pc);
    delay_ms1(500u);

    USART_SendString("[hard] claw -> +150us\r\n");
    Claw_WriteCompare((uint16_t)(pc + 150u));
    delay_ms1(500u);

    USART_SendString("[hard] yuntai PA3 -> ");
    UART_SendInt((int32_t)py);
    USART_SendString("us\r\n");
    Yuntai_WriteCompare(py);
    delay_ms1(500u);

    USART_SendString("[hard] yuntai PA3 -> +100us\r\n");
    Yuntai_WriteCompare((uint16_t)(py + 100u));
    delay_ms1(500u);

    /* 回默认姿势，顺便把账本对齐（不然下一次 S 曲线会从旧值起算） */
    Claw_SetDegState(CLAW_HOME_DEG);
    Yuntai_SetDegState(YUNTAI_INIT_DEG);
    Claw_WriteCompare(pc);
    Yuntai_WriteCompare(py);

    USART_SendString("[hard] back to default. no movement => check 5V / GND / signal\r\n");
}

/* ★ 2026-10-17（晚）：云台舵机类型判定（串口按 0）。
 *   "195.5 卡住" 到底是软件限位还是舵机本身到头，必须先分清这颗是哪种舵机：
 *     单圈 360° 位置舵机：500us 停在一头、1500us 停在中间、2500us 停在另一头；
 *     360° 连续旋转舵机：1500us 附近停住，500us 一直同方向转，2500us 一直反转。
 *   这里全部用固定脉宽直写 CCR，绕过插值、账本和所有限位，最多约 6 秒。
 *   结束时停在 1500us（新尺子 15.5 度），再把账本对齐到这个位置。 */
static void Servo_TypeTest(void)
{
    yuantai.moving_flag = 0;        /* 别让 TIM2 中断插值抢着写同一个 CCR */

    USART_SendString("\r\n[type] clear the yuntai area! fixed pulse, no interpolation\r\n");

    USART_SendString("[type] 1/3 1500us 1.2s -> POSITION: holds an angle | CONTINUOUS: stands still\r\n");
    Yuntai_WriteCompare(1500u);
    delay_ms1(1200u);

    USART_SendString("[type] 2/3  500us 2.0s -> POSITION: runs to one end and STOPS | CONTINUOUS: keeps spinning\r\n");
    Yuntai_WriteCompare(500u);
    delay_ms1(2000u);

    USART_SendString("[type]     1500us 0.8s -> stop\r\n");
    Yuntai_WriteCompare(1500u);
    delay_ms1(800u);

    USART_SendString("[type] 3/3 2500us 2.0s -> POSITION: runs to the other end and STOPS | CONTINUOUS: spins the other way\r\n");
    Yuntai_WriteCompare(2500u);
    delay_ms1(2000u);

    USART_SendString("[type]     1500us -> neutral, test done\r\n");
    Yuntai_WriteCompare(1500u);
    delay_ms1(400u);

    /* 1500us 正好是 500..2500 的中点 ⇒ 新尺子 85.5 度（265.5 - 180） */
    Yuntai_SetDegState(85.5f);
    Yuntai_Report("type-test-end", Yuntai_CurrentDeg());

    USART_SendString("[type] VERDICT A: stopped at both ends = POSITION servo (265.5 is its real end).\r\n");
    USART_SendString("[type] VERDICT B: kept turning at 500/2500 = CONTINUOUS servo (angle control must be replaced).\r\n");
    USART_SendString("[type] tell me A or B; press y to go back to INIT ");
    UART_SendFix1(YUNTAI_INIT_DEG);
    USART_SendString(" deg.\r\n");
}

/* 三轴一起动：平推 / 升降 / 云台「同时」走，最后等最慢的那个。
 * 照抄 Hardware/PWM.c 的 move_all()，只把两轴那两句换成我们带地址（5 / 7）的版本：
 *   ① 云台先下目标角，时间按源码那套分段公式算（走 TIM2 中断，异步）；
 *   ② 两轴 CAN 位置模式立刻发（发完就返回，电机自己走）；
 *   ③ delay 三者里最长的时间，等大家都到位。
 * 源码里 accel/speed1 是在函数体写死的（accel=230、accel1=240、speed1=2000），
 * 只有平推速度用传进来的值，这里照抄成 SRV_* 那组宏。
 * 参数顺序也和源码一致：Move_All(平推 mm, 升降 mm, 云台角)。 */
static void Move_All(float push_mm, float lift_mm, float yuntai_deg)
{
    float t_push = 0.0f;
    float t_lift = 0.0f;
    float t_yt;
    float t_max;
    float yt_tgt = yuntai_deg;

    /* 先按整圈折一次目标角，再算"要转多少度" ——
     * ★ 源码那句 fabs(target - yuantai.current_angle) 是"外面角 - 结构体内部角"
     *   （两边差一个 7.9 的修正量），这里改成两边都用外面这把尺子，算出来的时间才准。
     * ★ 2026-10-17 起不再夹 MIN/MAX：任意角都折进整圈窗口（见 Yuntai_WrapDeg）。 */
    yt_tgt = Yuntai_WrapDeg(yt_tgt);

    t_yt = Yuntai_SlowTime(yt_tgt - Yuntai_CurrentDeg());
    Yuntai_Goto(yt_tgt, t_yt);

    Axis_MoveTo(ADDR_PUSH, PPM_PUSH, &g_push_mm, PUSH_MIN_MM, PUSH_MAX_MM,
                PUSH_DIR_INV, push_mm, SRV_ACC_PUSH, SRV_SPEED_PUSH, &t_push);
    Axis_MoveTo(ADDR_LIFT, PPM_LIFT, &g_lift_mm, LIFT_MIN_MM, LIFT_MAX_MM,
                LIFT_DIR_INV, lift_mm, SRV_ACC_LIFT, SRV_SPEED_LIFT, &t_lift);

    t_max = t_yt;
    if (t_push > t_max) { t_max = t_push; }
    if (t_lift > t_max) { t_max = t_lift; }
    if (t_max < 200.0f) { t_max = 200.0f; }

    delay_ms1((uint32_t)t_max + STEP_MS);   /* 等最慢的那个走完 */
    Yuntai_WaitDone();                      /* 云台插值也收尾，免得下一句就合爪 */

    USART_SendString("[move_all] push=");
    UART_SendFix1(g_push_mm);
    USART_SendString(" lift=");
    UART_SendFix1(g_lift_mm);
    USART_SendString(" yuntai=");
    UART_SendFix1(yt_tgt);
    USART_SendString(" (wait ");
    UART_SendInt((int32_t)t_max);
    USART_SendString("ms)\r\n");
}

/* 载物台（源码 wukuaipingtai，PA2 / TIM2_CH3）：
 *   1 号位 = 20 度，2 号位 = 20+119 = 139 度，3 号位 = 20+119+117 = 256 度
 *   —— 数字照抄 PWM.c 的 set_wukuaipingtai_weizhi()。 */
static void Wukuai_Goto(uint8_t pos)
{
    if (pos < 1u || pos > 3u)
    {
        USART_SendString("[wukuai] pos must be 1 / 2 / 3\r\n");
        return;
    }
    if (g_claw_ch == 3u)
    {
        USART_SendString("[warn] claw is driving PA2 too; press 1 to move claw back to PA1\r\n");
    }

    set_wukuaipingtai_weizhi((int)pos);     /* 源码函数：TIM2 中断插值，非阻塞 */
    {
        uint32_t waited = 0u;

        while (wukuaipingtai.moving_flag != 0)     /* 同样带超时上限，别卡死主循环 */
        {
            delay_ms1(STEP_MS);
            waited += STEP_MS;
            if (waited >= SERVO_WAIT_MAX_MS)
            {
                wukuaipingtai.moving_flag = 0;
                wukuaipingtai.start_angle = wukuaipingtai.current_angle;
                USART_SendString("[wukuai] wait timeout, force stop\r\n");
                break;
            }
        }
    }

    USART_SendString("[wukuai] pos ");
    UART_SendInt((int32_t)pos);
    USART_SendString(" deg = ");
    UART_SendFix1(wukuaipingtai.current_angle);
    USART_SendString("\r\n");
}

/* 三段抓取流程：照抄 USER/main.c 的 yuantai_na()。平推/升降的行程照抄原值，
 * 云台角全部先指向 YUNTAI_GRAB_DEG / YUNTAI_BACK_DEG 这两个占位（源码的 278/295/261/135
 * 都在"角度大"那一头，跟我们车要走的反方向 + 没标定，留着会误导）。
 * ★ 标定完如果三个颜色确实要三个不同的云台角，就把这里拆成
 *   YUNTAI_GRAB1/2/3_DEG 三个宏再逐个填。 */
static void Grab_Flow(uint8_t task)
{
    if (task < 1u || task > 3u)
    {
        USART_SendString("[grab] task must be 1(red) / 2(green) / 3(blue)\r\n");
        return;
    }

    USART_SendString("[grab] task ");
    UART_SendInt((int32_t)task);
    USART_SendString(" start\r\n");

    if (task == 1u)                 /* 红：源码 yuantai_na(0x01) */
    {
        Claw_OpenWide();                            /* CLAW_OPEN_DEG = 45 度 */
        Move_All(PUSH_SRC(-100.0f), 50.0f, YUNTAI_GRAB_DEG); /* 伸过去，云台转到抓取位 */
        Claw_CloseHold();                           /* CLAW_CLOSE_DEG = 0 度，夹住 */
        Move_All(PUSH_SRC(-48.0f), 25.0f, YUNTAI_BACK_DEG);  /* 退回来 */
        Move_All(PUSH_SRC(-48.0f), 33.0f, YUNTAI_BACK_DEG);  /* 抬高一点，再退 */
        Claw_OpenWide();                            /* 松爪放料 */
    }
    else if (task == 2u)            /* 绿：源码 yuantai_na(0x02) */
    {
        Claw_OpenWide();
        Move_All(PUSH_SRC(63.0f), 48.0f, YUNTAI_GRAB_DEG);
        Claw_CloseHold();
        Move_All(PUSH_SRC(20.0f), 25.0f, YUNTAI_GRAB_DEG);
        Move_All(PUSH_SRC(-20.0f), 25.0f, YUNTAI_BACK_DEG);
        Move_All(PUSH_SRC(-48.0f), 25.0f, YUNTAI_BACK_DEG);
        Move_All(PUSH_SRC(-48.0f), 33.0f, YUNTAI_BACK_DEG);
        Claw_OpenWide();
    }
    else                            /* 蓝：源码 yuantai_na(0x03) */
    {
        Claw_OpenWide();
        Move_All(PUSH_SRC(63.0f), 48.0f, YUNTAI_GRAB_DEG);
        Claw_CloseHold();
        Move_All(PUSH_SRC(-48.0f), 25.0f, YUNTAI_BACK_DEG);
        Move_All(PUSH_SRC(-48.0f), 33.0f, YUNTAI_BACK_DEG);
        Claw_OpenWide();
    }

    /* 最后一步是 Claw_OpenWide()（Claw_MoveTo 会等到停稳并带超时），
     * 所以账本此时已经是 CLAW_OPEN_DEG；不用再对齐。 */
    USART_SendString("[grab] task ");
    UART_SendInt((int32_t)task);
    USART_SendString(" done\r\n");
}

/* 一键夹取：从默认位（升降最高、齿条收到底）直接走到实测的夹取姿势，合爪并保持。
 *   ① 开爪 45 度 —— 下降时手指要够宽，别先撞到物块；
 *   ② 云台先转到 126 度 —— 这一步在齿条还收着的时候做，摆动半径最小；
 *   ③ 升降留在最高处，把齿条伸到 65mm —— 高处走横向，路上没有任何东西；
 *   ④ 平推不动，升降降到 110mm；
 *   ⑤ 合爪到 0 度，夹住不松。
 *   想放下就按 o（开爪）；想微调就再按 j/f 之类，账本已经跟上了。 */
static void Quick_Grab(void)
{
    USART_SendString("[qgrab] 1/5 open claw\r\n");
    Claw_OpenWide();

    /* 云台先在"收着"的姿势转到位：此时伸出去的长度是 0，摆动半径最小，
     * 不会甩着伸长的齿条去扫台面上的东西。 */
    USART_SendString("[qgrab] 2/5 yuntai -> ");
    UART_SendFix1(QUICK_GRAB_YUNTAI_DEG);
    USART_SendString(" deg while retracted (push=");
    UART_SendFix1(g_push_mm);
    USART_SendString(" mm, lift=");
    UART_SendFix1(g_lift_mm);
    USART_SendString(" mm)\r\n");
    Move_All(g_push_mm, g_lift_mm, QUICK_GRAB_YUNTAI_DEG);

    USART_SendString("[qgrab] 3/5 extend at TOP: push=");
    UART_SendFix1(QUICK_GRAB_PUSH_MM);
    USART_SendString(" mm, lift stays ");
    UART_SendFix1(LIFT_HOME_MM);
    USART_SendString(" mm\r\n");
    Move_All(QUICK_GRAB_PUSH_MM, LIFT_HOME_MM, QUICK_GRAB_YUNTAI_DEG);

    USART_SendString("[qgrab] 4/5 lower into place: lift=");
    UART_SendFix1(QUICK_GRAB_LIFT_MM);
    USART_SendString(" mm\r\n");
    Move_All(QUICK_GRAB_PUSH_MM, QUICK_GRAB_LIFT_MM, QUICK_GRAB_YUNTAI_DEG);

    USART_SendString("[qgrab] 5/5 close claw (hold)\r\n");
    Claw_CloseHold();

    Show_ArmState();
    Claw_Report("grab", Claw_CurrentDeg());
    Yuntai_Report("grab", Yuntai_CurrentDeg());
    USART_SendString("[qgrab] done, block held.  o = open / release\r\n");
}

/* 一键放上载物台（@）：夹着物块从夹取位走到载物台姿势并松爪。
 *   ★ 顺序是用户 2026-10-17 指定的：先升到最高，再转角度，最后下放。
 *   ① 升降先升到最高 —— 物块先离开台面，横向摆动不会刮到东西；
 *   ② 高处把齿条收回 0 —— 收着转，摆动半径最小（和抓取流程同一个道理）；
 *   ③ 云台转到载物台角 254 度（脉宽 564us）；
 *   ④ 高处把齿条伸到 30mm；
 *   ⑤ 升降降到 110mm —— 物块落到载物台上；
 *   ⑥ 开爪 45 度，放完就走。
 *   参数：push 30 / lift 110 / yuntai 254（我们这台车实测）。 */
static void Quick_PlaceOnPlatform(void)
{
    USART_SendString("[qplace] 1/6 lift up to ");
    UART_SendFix1(LIFT_HOME_MM);
    USART_SendString(" mm first (block leaves the table)\r\n");
    Move_All(g_push_mm, LIFT_HOME_MM, Yuntai_CurrentDeg());

    USART_SendString("[qplace] 2/6 retract push -> 0.0 mm at TOP\r\n");
    Move_All(0.0f, LIFT_HOME_MM, Yuntai_CurrentDeg());

    USART_SendString("[qplace] 3/6 yuntai -> ");
    UART_SendFix1(QUICK_PLACE_YUNTAI_DEG);
    USART_SendString(" deg while retracted\r\n");
    Move_All(0.0f, LIFT_HOME_MM, QUICK_PLACE_YUNTAI_DEG);

    USART_SendString("[qplace] 4/6 extend at TOP: push=");
    UART_SendFix1(QUICK_PLACE_PUSH_MM);
    USART_SendString(" mm\r\n");
    Move_All(QUICK_PLACE_PUSH_MM, LIFT_HOME_MM, QUICK_PLACE_YUNTAI_DEG);

    USART_SendString("[qplace] 5/6 lower onto the platform: lift=");
    UART_SendFix1(QUICK_PLACE_LIFT_MM);
    USART_SendString(" mm\r\n");
    Move_All(QUICK_PLACE_PUSH_MM, QUICK_PLACE_LIFT_MM, QUICK_PLACE_YUNTAI_DEG);

    USART_SendString("[qplace] 6/6 open claw (release)\r\n");
    Claw_OpenWide();

    Show_ArmState();
    Claw_Report("place", Claw_CurrentDeg());
    Yuntai_Report("place", Yuntai_CurrentDeg());
    USART_SendString("[qplace] done on platform (push 30 / lift 110 / yuntai 254)\r\n");
}

/* 一键回默认位（上电时程序假设的那个姿势）：
 *   升降 = LIFT_HOME_MM(135，最高)，平推 = 0（齿条收到底），云台 = YUNTAI_INIT_DEG(0)。
 *   ① 先垂直升到最高 —— 爪子先离开物块，别横着把它拖走；
 *   ② 再在高处把齿条收到底，云台回初始角；
 *   ③ 最后爪子回到默认位 CLAW_HOME_DEG(45 度)。
 *   ★ 爪子这一步会让夹着的东西掉下来，要留着东西就别按 W。 */
static void Arm_Home(void)
{
    USART_SendString("[home] 1/2 lift up to ");
    UART_SendFix1(LIFT_HOME_MM);
    USART_SendString(" mm (push stays at ");
    UART_SendFix1(g_push_mm);
    USART_SendString(" mm)\r\n");
    Move_All(g_push_mm, LIFT_HOME_MM, Yuntai_CurrentDeg());

    USART_SendString("[home] 2/2 push -> 0.0 mm, yuntai -> ");
    UART_SendFix1(YUNTAI_INIT_DEG);
    USART_SendString(" deg\r\n");
    Move_All(0.0f, LIFT_HOME_MM, YUNTAI_INIT_DEG);

    USART_SendString("[home] 3/3 claw -> ");
    UART_SendFix1(CLAW_HOME_DEG);
    USART_SendString(" deg (default pose)\r\n");
    Claw_MoveTo(CLAW_HOME_DEG, MOVE_TIME_MS);

    Show_ArmState();
    Claw_Report("home", Claw_CurrentDeg());
    Yuntai_Report("home", Yuntai_CurrentDeg());
    USART_SendString("[home] done (default pose: lift 135 / push 0 / yuntai 0 / claw 45)\r\n");
}

/* 一键"夹起来再放到载物台"（@）：一整轮，从默认位开始。
 *   ① Arm_Home()        —— 先回默认位，保证每次起点一样
 *                          （爪会张开：手里还夹着东西时别按 @，用 $ 单独放）
 *   ② Quick_Grab()      —— 原来的夹取动作，一字没改：开爪 -> 转 126° -> 高位伸出 65mm
 *                          -> 降到 110mm -> 合爪夹住
 *   ③ Quick_PlaceOnPlatform() —— 夹着物块升到最高 135mm -> 收齿条 -> 转 254°
 *                          -> 高位伸出 30mm -> 降到 110mm -> 开爪放到载物台
 *   注意：放完停在载物台姿势（push 30 / lift 110 / yuntai 254、爪张开），
 *   再按一次 @ 会先回默认位，不会在半路斜着扫回来。 */
static void Quick_GrabAndPlace(void)
{
    USART_SendString("[cycle] ==== home -> grab -> place on platform ====\r\n");

    Arm_Home();

    USART_SendString("[cycle] ---- grab ----\r\n");
    Quick_Grab();

    USART_SendString("[cycle] ---- carry to platform ----\r\n");
    Quick_PlaceOnPlatform();

    USART_SendString("[cycle] done: block is on the platform\r\n");
}


/* ==========================================================================
 *  ⑤ 菜单 / 状态
 * ========================================================================== */

static void Show_Menu(void)
{
    USART_SendString("\r\n==== ARM + CLAW test ====\r\n");
    USART_SendString("-- arm (CAN: lift=5, push=7) --\r\n");
    USART_SendString("u / j : lift  up / down      (one step)\r\n");
    USART_SendString("        power-on default = TOP (LIFT_HOME 135mm), so j works right away\r\n");
    USART_SendString("f / r : push  out / retract  (one step; 0 = rack pushed fully back)\r\n");
    USART_SendString("w     : push -> HOME 0mm (rack fully back; follows the book, not a homing switch)\r\n");
    USART_SendString("W     : ARM HOME: lift -> 135mm first, then push -> 0mm, yuntai -> 0deg,\r\n");
    USART_SendString("        claw -> 45deg = default pose (a held block will drop)\r\n");
    USART_SendString("s     : STOP arm both now\r\n");
    USART_SendString("+ / - : arm step +-5mm   [ / ] : arm speed -+100 rpm\r\n");
    USART_SendString("0     : zero arm book (no move)   e / d : enable / disable arm\r\n");
    USART_SendString("zero pos: push = rack fully BACK (hard stop)   lift = lifted to the TOP\r\n");
    USART_SendString("-- claw (servo) --  now driving ");
    USART_SendString(Pwm_PinName(g_claw_ch));
    USART_SendString(" (TIM2_CH");
    UART_SendInt((int32_t)g_claw_ch);
    USART_SendString(")\r\n");
    USART_SendString("b     : claw -> CENTER 90deg   (power-on default is HOME 45deg)\r\n");
    USART_SendString("o / c : open / close\r\n");
    USART_SendString(", / . : nudge claw -/+ step deg\r\n");
    USART_SendString("a135# : claw go to 135 deg (a -> digits -> #)\r\n");
    USART_SendString("l     : RANGE PROBE, +5 deg slowly (for finding the real max travel)\r\n");
    USART_SendString("z     : back off 5 deg   (press the moment it buzzes or stops turning)\r\n");
    USART_SendString("q     : back off 30 deg at once (if it jammed)\r\n");
    USART_SendString("t     : claw auto sweep   x : claw release (soft)\r\n");
    USART_SendString("` (or ~) : HARD TEST both servos, bypass all logic (check 5V / wiring)\r\n");
    USART_SendString("1 / 2 / 3 : move claw output to PA1 / PA2 / PA3 (find which header the claw is on)\r\n");
    USART_SendString("       PA2 = board silk \"wu kuai ping tai\",  PA3 = board silk \"yun tai\"\r\n");
    USART_SendString("       press 1/2/3, then press b/o/c -- the servo that moves is the claw\r\n");
    USART_SendString("       we already found it: claw = PA1 (CH2). 2/3 = that pin belongs to\r\n");
    USART_SendString("       wu kuai ping tai / yuntai, so only use them to re-check wiring.\r\n");
    USART_SendString("-- yuntai (servo, PA3 / TIM2_CH4) --\r\n");
    USART_SendString("y     : yuntai -> INIT ");
    UART_SendFix1(YUNTAI_INIT_DEG);
    USART_SendString(" deg (calibrated 2026-10-17, pulse 1975us)\r\n");
    USART_SendString("n     : yuntai -> GRAB ");
    UART_SendFix1(YUNTAI_GRAB_DEG);
    USART_SendString(" deg (PLACEHOLDER, not calibrated yet)\r\n");
    USART_SendString("g / k : yuntai nudge -/+ step deg   (g = towards the block on OUR car)\r\n");
    USART_SendString("i40#  : yuntai go to +40 deg = towards the block (i -> digits -> #)\r\n");
    USART_SendString("i-30# : ...or with '-' toggled: go to -30 deg = backwards\r\n");
    USART_SendString("v     : yuntai release (soft, turn by hand)\r\n");
    USART_SendString("/     : RANGE PROBE, +5 deg towards block (500us end)\r\n");
    USART_SendString("\\     : RANGE PROBE, -5 deg away from block  (2500us end)\r\n");
    USART_SendString("*     : SERVO TYPE TEST - 1500 / 500 / 2500us, 2s each, auto stop (POSITION vs CONTINUOUS)\r\n");
    USART_SendString("speed : all yuntai moves x YUNTAI_SPEED_DIV = ");
    UART_SendFix1(YUNTAI_SPEED_DIV);
    USART_SendString(" slower than source (bigger = slower)\r\n");
    USART_SendString("-- copy of the source project (gcs-gold-medal) --\r\n");
    USART_SendString("4 / 5 / 6 : FULL grab flow, source yuantai_na(1/2/3) = red / green / blue\r\n");
    USART_SendString("            claw open -> move_all(push,lift,yuntai) x several -> claw close -> open\r\n");
    USART_SendString("            numbers are the source car's; ours NOT calibrated yet.\r\n");
    USART_SendString("7 / 8 / 9 : wu kuai ping tai (PA2 / CH3) -> source pos 1 / 2 / 3 = 20 / 139 / 256 deg\r\n");
    USART_SendString("m         : type one move_all by hand,\r\n");
    USART_SendString("            m -> \"push lift yuntai #\"  e.g.  22 50 33#   (integers, '-' allowed)\r\n");
    USART_SendString("!         : QUICK GRAB (measured 2026-10-17): open claw -> yuntai 126deg\r\n");
    USART_SendString("            -> push 65mm at TOP -> lift 110mm -> close claw, HOLD the block\r\n");
    USART_SendString("            (two steps on purpose: no diagonal sweep over the block)\r\n");
    USART_SendString("@         : FULL CYCLE: home -> GRAB (same as !) -> carry to the on-car platform:\r\n");
    USART_SendString("            lift 135mm TOP -> retract -> yuntai 254deg -> push 30mm -> lift 110mm -> open\r\n");
    USART_SendString("$         : PLACE ONLY (already holding a block): same carry steps, no re-grab\r\n");
    USART_SendString("-- both --\r\n");
    USART_SendString("p     : print state (arm + claw)   h : this menu\r\n");
    Show_ArmState();
    Claw_ShowParams();
    Yuntai_ShowParams();
}


/* ==========================================================================
 *  ⑥ 按键处理
 * ========================================================================== */

/* 现场切换"爪子输出在哪个引脚"：1 = PA1(TIM2_CH2)、2 = PA2(TIM2_CH3)、3 = PA3(TIM2_CH4)。
 * ★ 10/14 改版后这活儿要"两件事一起做"，少一件就会看起来像没生效：
 *   ① 换通道：zhuashou.SetServoAngle 这个函数指针要指向对应通道的换算函数
 *      （servo.c 的 Servo_SetAngle2/3/4_xxx 各自写死 CH2/CH3/CH4）——
 *      不然 TIM2 中断还在按老通道写 CCR，按 1/2/3 等于白按；
 *   ② 换完立刻按"那一路自己的当前角"补一发脉冲，让座上的舵机马上跟过来。
 * 注意 2 / 3 是 PA2 / PA3：那两路上本来插着载物台和云台的舵机（我们板上实测），
 * 切过去就是"爪子的键去指挥那台舵机"，只在查线时用，查完按 1 切回 PA1。 */
static void Claw_SelectCh(uint8_t ch)
{
    if (ch < 2u || ch > 4u)
    {
        USART_SendString("[claw] channel must be 2 / 3 / 4\r\n");
        return;
    }

    if (ch == g_claw_ch)
    {
        USART_SendString("[claw] unchanged, still driving ");
        USART_SendString(Pwm_PinName(ch));
        USART_SendString("\r\n");
        return;
    }

    /* 先把爪子的插值停住，别让中断继续往老通道写 */
    zhuashou.moving_flag = 0;
    zhuashou.start_angle = zhuashou.current_angle;

    USART_SendString("[claw] old ");
    USART_SendString(Pwm_PinName(g_claw_ch));
    USART_SendString(" -> off (that servo goes soft)\r\n");

    Claw_WriteCompare(0u);                  /* 先关老的那一路（CCR = 0 = 不发脉冲） */
    g_claw_ch = ch;                         /* 再换到新的一路 */

    /* ① 换函数指针：让 TIM2 中断里的插值写到新通道去 */
    if (ch == 2u)      zhuashou.SetServoAngle = Servo_SetAngle2_zhuashou;
    else if (ch == 3u) zhuashou.SetServoAngle = Servo_SetAngle3_wukuaipingtai;
    else               zhuashou.SetServoAngle = Servo_SetAngle4_yuntai;

    /* ② 新的一路立刻按"该通道自己的内部角"发脉冲（每路尺子不同，所以读各自的结构体） */
    if (ch == 2u)      Ch_WriteInner(2u, zhuashou.current_angle);
    else if (ch == 3u) Ch_WriteInner(3u, wukuaipingtai.current_angle);
    else               Ch_WriteInner(4u, yuantai.current_angle);

    USART_SendString("[claw] now driving ");
    USART_SendString(Pwm_PinName(g_claw_ch));
    USART_SendString("  (which servo moves now?)\r\n");

    if (g_claw_ch == 3u)
        USART_SendString("[warn] PA2 is wu-kuai-ping-tai's pin; the claw keys now drive that servo\r\n");
    if (g_claw_ch == YUNTAI_CH)
        USART_SendString("[warn] PA3 is yuntai's pin; y / n and the claw keys now fight over it\r\n");
}

/* 0 = 无，1 = 正在输入爪子角度，2 = 正在输入云台角度，3 = 正在输入 move_all 三元组 */
static uint8_t  g_input_mode = 0;
static uint16_t g_input_val  = 0;
static uint8_t  g_skip_lf    = 0;
static uint8_t  g_input_neg  = 0;               /* mode 3：当前这个数是不是负的 */
static uint8_t  g_mv_idx     = 0;               /* mode 3：已经收进来几个数（0~3） */
static float    g_mv_v[3]    = {0.0f, 0.0f, 0.0f};
static uint8_t  g_probe_hint = 0;               /* 量程实测：过了 180 度只提醒一次 */

static void Claw_DoInputDigit(char c)
{
    g_input_val = (uint16_t)(g_input_val * 10u + (uint16_t)(c - '0'));
    if (g_input_val > 999u) g_input_val = 999u;
    UART_SendByte((uint8_t)c);          /* 回显数字 */
}

static void Mv_InputReset(void)
{
    g_input_mode = 0u;
    g_input_val  = 0u;
    g_input_neg  = 0u;
    g_mv_idx     = 0u;
    g_mv_v[0]    = 0.0f;
    g_mv_v[1]    = 0.0f;
    g_mv_v[2]    = 0.0f;
}

/* mode 3：手输一组 move_all 参数 —— "平推 升降 云台角#"，三个整数，空格隔开，可带负号。
 * 例：22 50 33#  就等于源码 yuantai_na(0x01) 的第一步（源码 -100 → 我们 0 位 = 22，
 *     云台角用我们车上量的数字）。
 *     ★ 平推账面 0 = 齿条顶住后端，只往前加；负号现在是给云台角留的。
 * 用途：不用改代码重编译，就能一组一组试参数，方便在我们车上标定那三个角。 */
static void Handle_MoveAllInput(char c)
{
    if (c >= '0' && c <= '9') { Claw_DoInputDigit(c); return; }

    if (c == '-')
    {
        g_input_neg = (uint8_t)(g_input_neg ? 0u : 1u);
        UART_SendByte((uint8_t)(g_input_neg ? '-' : '+'));
        return;
    }

    if (c == ' ' || c == ',' || c == '\t')
    {
        if (g_mv_idx < 3u)
        {
            g_mv_v[g_mv_idx] = (float)g_input_val * (g_input_neg ? -1.0f : 1.0f);
            g_mv_idx++;
        }
        g_input_val = 0u;
        g_input_neg = 0u;
        UART_SendByte((uint8_t)' ');
        return;
    }

    if (c == '#' || c == '\r' || c == '\n')
    {
        if (g_mv_idx < 3u)
        {
            g_mv_v[g_mv_idx] = (float)g_input_val * (g_input_neg ? -1.0f : 1.0f);
            g_mv_idx++;
        }
        if (c == '\r') g_skip_lf = 1u;
        USART_SendString("\r\n");

        if (g_mv_idx != 3u)
        {
            USART_SendString("[move_all] need 3 numbers (push lift yuntai), got ");
            UART_SendInt((int32_t)g_mv_idx);
            USART_SendString(" -> cancelled\r\n");
        }
        else
        {
            Move_All(g_mv_v[0], g_mv_v[1], g_mv_v[2]);
            Yuntai_Report("now", Yuntai_CurrentDeg());
        }
        Mv_InputReset();
        return;
    }

    Mv_InputReset();
    USART_SendString("\r\n[cancel] bad char in move_all input\r\n");
}

static void Handle_Key(char c)
{
    /* --- 输入模式：1 = 爪子角度，2 = 云台角度，3 = move_all 三元组 --- */
    if (g_input_mode != 0u)
    {
        uint8_t mode = g_input_mode;

        if (mode == 3u) { Handle_MoveAllInput(c); return; }

        if (c >= '0' && c <= '9') { Claw_DoInputDigit(c); return; }
        /* mode 2（云台角）允许负号：新尺子下初始位是 0，往物块那边是正角，
         * 往回/退料那边才是负角，所以要能打 "i-30#" 这种。按一下 '-' 取负，再按一下取消。 */
        if (c == '-' && mode == 2u)
        {
            g_input_neg = (uint8_t)(g_input_neg ? 0u : 1u);
            UART_SendByte((uint8_t)(g_input_neg ? '-' : '+'));
            return;
        }
        /* ★ 2026-10-17：'\n' 也算确认。有些串口工具按回车只发 LF，
         *   以前会被当成非法字符直接 [cancel]，白打一串数字。 */
        if (c == '#' || c == '\r' || c == '\n')
        {
            float tgt = (float)g_input_val * (g_input_neg ? -1.0f : 1.0f);

            g_input_mode = 0;
            g_input_neg  = 0u;
            if (c == '\r') g_skip_lf = 1u;
            USART_SendString("\r\n");
            if (mode == 1u)
            {
                if (tgt < CLAW_MIN_DEG || tgt > CLAW_MAX_DEG)
                    USART_SendString("[warn] outside limits, clamped\r\n");
                Claw_MoveTo(tgt, MOVE_TIME_MS);
                Claw_Report("goto", Claw_CurrentDeg());
            }
            else
            {
                float wrapped = Yuntai_WrapDeg(tgt);

                if (wrapped < tgt - 0.05f || wrapped > tgt + 0.05f)
                {
                    USART_SendString("[yuntai] 360 wrap: ");
                    UART_SendFix1(tgt);
                    USART_SendString(" -> ");
                    UART_SendFix1(wrapped);
                    USART_SendString(" deg (full circle)\r\n");
                }
                Yuntai_MoveTo(wrapped);
                Yuntai_Report("goto", Yuntai_CurrentDeg());
            }
            return;
        }
        g_input_mode = 0;
        g_input_neg  = 0u;
        USART_SendString("\r\n[cancel] not a digit\r\n");
        return;
    }

    /* 回车/换行不当命令，但 '\r\n' 只吞一次 */
    if (c == '\r' || c == '\n')
    {
        if (g_skip_lf == 1u && c == '\n') { g_skip_lf = 0u; return; }
        g_skip_lf = 0u;
        return;
    }
    g_skip_lf = 0u;

    switch (c)
    {
        /* ---- 机械臂 ---- */
        case 'u': case 'U': Do_Lift(+1);                        break;
        case 'j': case 'J': Do_Lift(-1);                        break;
        case 'f': case 'F': Do_Push(+1);                        break;
        case 'r': case 'R': Do_Push(-1);                        break;
        case 'w':           Do_PushHome();                      break;  /* 只回平推 */
        case 'W':           Arm_Home();                         break;  /* 三轴一起回默认位 */

        case 's': case 'S': Arm_StopBoth();                     break;

        case '+': case '=': Arm_StepChange(+STEP_STEP);         break;
        case '-': case '_': Arm_StepChange(-STEP_STEP);         break;

        case '[': case '{': Arm_SpeedChange(-MOVE_SPEED_STEP);  break;
        case ']': case '}': Arm_SpeedChange(+MOVE_SPEED_STEP);  break;

        case '0':           Arm_ZeroBook();                     break;
        case 'e': case 'E': Arm_EnableBoth(true);               break;
        case 'd': case 'D': Arm_EnableBoth(false);              break;

        /* ---- 爪子 ---- */
        case 'b': case 'B':
            Claw_MoveTo(CLAW_CENTER_DEG, MOVE_TIME_MS);
            Claw_Report("center", Claw_CurrentDeg());
            break;

        case 'o': case 'O':
            Claw_MoveTo(CLAW_OPEN_DEG, MOVE_TIME_MS);
            Claw_Report("open", Claw_CurrentDeg());
            break;

        case 'c': case 'C':
            Claw_MoveTo(CLAW_CLOSE_DEG, MOVE_TIME_MS);
            Claw_Report("close", Claw_CurrentDeg());
            break;

        case ',':
        case '<':
        {
            float cur = Claw_CurrentDeg() - CLAW_STEP_DEG;

            if (cur < CLAW_MIN_DEG) { cur = CLAW_MIN_DEG; USART_SendString("[claw] at MIN\r\n"); }
            Claw_MoveTo(cur, MOVE_TIME_MS);
            Claw_Report("nudge-", Claw_CurrentDeg());
            break;
        }

        case '.':
        case '>':
        {
            float cur = Claw_CurrentDeg() + CLAW_STEP_DEG;

            if (cur > CLAW_MAX_DEG) { cur = CLAW_MAX_DEG; USART_SendString("[claw] at MAX\r\n"); }
            Claw_MoveTo(cur, MOVE_TIME_MS);
            Claw_Report("nudge+", Claw_CurrentDeg());
            break;
        }

        /* ---- 舵机量程实测（只为量"最多能转到几度"，平时抓取不用这几个键）----
         * l：慢速往上探 5 度，一格格走，走完打印角度和脉宽
         * z：往回退 5 度（听到憋响就按这个）
         * q：一把退回 30 度，卡死了用 */
        case 'l': case 'L':
        {
            float cur = Claw_CurrentDeg() + CLAW_PROBE_STEP_DEG;

            if (cur > CLAW_MAX_DEG)
            {
                cur = CLAW_MAX_DEG;
                USART_SendString("[claw] at 270 (program limit)\r\n");
            }
            Claw_MoveTo(cur, CLAW_PROBE_TIME_MS);
            Claw_Report("probe+", Claw_CurrentDeg());

            if (Claw_CurrentDeg() >= 180.0f && g_probe_hint == 0u)
            {
                g_probe_hint = 1u;
                USART_SendString("[probe] past 180 now. If the horn stops while deg still rises,\r\n"
                                 "[probe] or you hear a steady buzz: press z, do NOT hold it there.\r\n");
            }
            break;
        }

        case 'z': case 'Z':
        {
            float cur = Claw_CurrentDeg() - CLAW_BACKOFF_DEG;

            if (cur < CLAW_MIN_DEG) cur = CLAW_MIN_DEG;
            Claw_MoveTo(cur, CLAW_PROBE_TIME_MS);
            Claw_Report("backoff", Claw_CurrentDeg());
            if (Claw_CurrentDeg() < 180.0f) g_probe_hint = 0u;
            break;
        }

        case 'q': case 'Q':
        {
            float cur = Claw_CurrentDeg() - 30.0f;

            if (cur < CLAW_MIN_DEG) cur = CLAW_MIN_DEG;
            Claw_MoveTo(cur, 1000u);
            Claw_Report("retreat", Claw_CurrentDeg());
            if (Claw_CurrentDeg() < 180.0f) g_probe_hint = 0u;
            break;
        }

        case 'a': case 'A':
            g_input_mode = 1;
            g_input_val  = 0u;
            g_input_neg  = 0u;
            USART_SendString("claw angle input: digits then '#'\r\n[angle] ");
            break;

        case 't': case 'T':
        {
            uint8_t k;

            USART_SendString("[claw] auto sweep x");
            UART_SendInt((int32_t)SWEEP_TIMES);
            USART_SendString("\r\n");
            for (k = 0u; k < SWEEP_TIMES; k++)
            {
                Claw_MoveTo(CLAW_OPEN_DEG,  SWEEP_TIME_MS);
                Claw_Report("sweep open",  Claw_CurrentDeg());
                Claw_MoveTo(CLAW_CLOSE_DEG, SWEEP_TIME_MS);
                Claw_Report("sweep close", Claw_CurrentDeg());
            }
            USART_SendString("[claw] sweep done\r\n");
            break;
        }

        case 'x': case 'X':
            Claw_Release();
            USART_SendString("[claw] released (no pulses, soft)\r\n");
            break;

        /* ---- 云台（PA3 / TIM2_CH4） ---- */
        case 'y': case 'Y':
            Yuntai_MoveTo(YUNTAI_INIT_DEG);
            Yuntai_Report("init", Yuntai_CurrentDeg());
            break;

        case 'n': case 'N':
            Yuntai_MoveTo(YUNTAI_GRAB_DEG);
            Yuntai_Report("grab", Yuntai_CurrentDeg());
            break;

        /* g 永远是"朝物块"。新尺子下朝物块 = 角度变大，所以这里比 k 多了个 +。 */
        case 'g': case 'G':
            /* 到 YUNTAI_MAX_DEG 就停：再往外是单圈电位器接缝，不能取模跨过去 */
            Yuntai_Nudge(+YUNTAI_STEP_DEG, -1.0f);
            Yuntai_Report("towards-block", Yuntai_CurrentDeg());
            break;

        case 'k': case 'K':
            Yuntai_Nudge(-YUNTAI_STEP_DEG, -1.0f);
            Yuntai_Report("away-block", Yuntai_CurrentDeg());
            break;

        /* ---- 云台量程实测：/ 往物块探 5 度，\ 退回 5 度（和爪子的 l / z 对应）---- */
        case '/':
            Yuntai_Nudge(+YUNTAI_PROBE_STEP_DEG, YUNTAI_PROBE_MS);
            Yuntai_Report("probe+", Yuntai_CurrentDeg());
            break;

        case '\\':
            Yuntai_Nudge(-YUNTAI_PROBE_STEP_DEG, YUNTAI_PROBE_MS);
            Yuntai_Report("probe-", Yuntai_CurrentDeg());
            break;

        case 'i': case 'I':
            g_input_mode = 2;
            g_input_val  = 0u;
            g_input_neg  = 0u;
            USART_SendString("yuntai angle input: any angle, '#' to send (+ = towards block; 360 wraps; e.g. i40# i-30# i200#->-160 i360#->0)\r\n[angle] ");
            break;

        case 'v': case 'V':
            Yuntai_Release();
            USART_SendString("[yuntai] released (no pulses, soft)\r\n");
            break;

        /* ` 或 ~ ：舵机硬测试，绕过账本和插值，直接写脉宽（排查供电/接线用） */
        case '`': case '~':
            Servo_HardTest();
            break;

        /* *：云台舵机类型判定（位置舵机 vs 连续旋转舵机），自动跑完回中 */
        case '*':
            Servo_TypeTest();
            break;

        /* ---- 通用 ---- */
        /* 1 / 2 / 3：现场把"爪子输出"切到 PA1 / PA2 / PA3 三路中的任意一路。
         * 哪个座上的舵机跟着爪子键动，爪子的信号线就插在那个座上。 */
        case '1': Claw_SelectCh(2u); break;     /* PA1 / TIM2_CH2 */
        case '2': Claw_SelectCh(3u); break;     /* PA2 / TIM2_CH3，集成板丝印"物块平台" */
        case '3': Claw_SelectCh(4u); break;     /* PA3 / TIM2_CH4，集成板丝印"云台" */

        /* ---- 照抄源码的整套动作 ---- */
        /* 4 / 5 / 6：完整抓取流程 = 源码 USER/main.c 的 yuantai_na(1/2/3)（红/绿/蓝）。
         * 里面那几组数字是原项目那台车标定出来的，我们车的云台还没标，先照抄跑。 */
        case '4': Grab_Flow(1u);  break;
        case '5': Grab_Flow(2u);  break;
        case '6': Grab_Flow(3u);  break;

        /* !：一键夹取 —— 从默认位直接走到实测的夹取姿势并合爪保持 */
        case '!': Quick_Grab();   break;

        /* @：一键整轮：回默认位 -> 原来那套夹取 -> 升到最高 -> 转载物台 -> 放下 */
        case '@': Quick_GrabAndPlace();    break;

        /* $：只做"放"—— 手里已经夹着东西时用（不重新夹） */
        case '$': Quick_PlaceOnPlatform(); break;

        /* 7 / 8 / 9：载物台（源码 wukuaipingtai，PA2 / TIM2_CH3）转到 1/2/3 号位 */
        case '7': Wukuai_Goto(1u); break;
        case '8': Wukuai_Goto(2u); break;
        case '9': Wukuai_Goto(3u); break;

        /* m：手输一组 move_all（平推 升降 云台角），标定用 */
        case 'm': case 'M':
            g_input_mode = 3u;
            g_input_val  = 0u;
            g_input_neg  = 0u;
            g_mv_idx     = 0u;
            USART_SendString("move_all input: \"push lift yuntai #\"  e.g. 22 50 33#  (push 0 = rack fully back)\r\n[mv] ");
            break;

        case 'p': case 'P':
            USART_SendString("print:\r\n");
            Show_ArmState();
            Claw_ShowParams();
            Yuntai_ShowParams();
            break;

        case 'h': case 'H': case '?': Show_Menu();              break;

        default:
            USART_SendString("?  (h = help)\r\n");
            break;
    }
}


/* ==========================================================================
 *  ⑦ 主程序
 * ========================================================================== */
int main(void)
{
    delay_init(168);        /* 软件延时（主频 168MHz） */
    Timer6_Init();          /* delay_ms1() 用的是 TIM6，必须先初始化 */
    MyUART_Init();          /* 串口：PB10/PB11（USART3），无线调试器接的就是这路 */

    PWM_Init();             /* TIM2 四路 PWM，50Hz；里面会把 CCR2/3/4 预置成 520/648/838 */

    /* ★ 2026-10-17：先把"我活着"打出来。以前这句在自动摆位之后，
     *   一旦摆位那两步卡住，串口一句都不吭 —— 分不清是程序死了还是舵机没电。
     *   现在只要看到这行，就说明 CPU、串口、定时器都活过来了。 */
    USART_SendString("\r\n[boot] alive: PWM + UART ok, setting default pose\r\n");

    /* 一上电先把非爪子那两路关掉：CCR = 0 = 不发脉冲。
     * 这样上电瞬间别的舵机不会突然跑到参考工程那个初始角上去。 */
#if (CLAW_CH != 2)
    PWM_SetCompare2(0u);
#endif
#if (CLAW_CH != 3)
    PWM_SetCompare3(0u);
#endif
#if (CLAW_CH != 4)
    PWM_SetCompare4(0u);
#endif
    /* ★ 顺序很重要：先把两个 Servo_t 结构体摆到我们想认的"上电姿势"
     *   （爪子 45 度、云台初始 YUNTAI_INIT_DEG = 0.0 度 = 脉宽 1975us），再松力。
     *   因为 Claw_CurrentDeg() / Yuntai_CurrentDeg() 现在直接读结构体，
     *   结构体的静态初值（爪子 2.7、云台 60.84）是源码那台车的上电姿势，
     *   不先覆盖掉，菜单和后面每次 S 曲线的起点都会差一截。 */
    Claw_SetDegState(CLAW_HOME_DEG);        /* 结构体内部角 = 45 - 1.0 = 44.0 */
    Yuntai_SetDegState(YUNTAI_INIT_DEG);    /* 结构体内部角 = 0.0*DIR + ZERO(273.4) - 7.9 = 265.5 */

    /* ★ 2026-10-17：上电直接摆到用户指定的默认姿势（爪子 45 度、云台 0 度），
     *   不再"先松着等按键"。好处是上电后账面和实物一定对得上 ——
     *   以前上电时云台停在 132 度、账面却记 0，按 ! 就不会再转，落点全错。
     *
     * ★★ 这里刻意不走 Claw_MoveTo()/Yuntai_MoveTo()（它们要等 TIM2 插值走完）。
     *   上电时只要插值没跑起来，那两个 while 就会把整个开机流程锁死。
     *   直接写一组固定脉宽最稳：PWM 是 TIM2 硬件产生的，跟中断无关。 */
    zhuashou.moving_flag = 0;
    yuantai.moving_flag  = 0;
    Claw_WriteCompare(Claw_PulseOf(CLAW_HOME_DEG));       /* 爪子 45 度 ≈ 826us */
    Yuntai_WriteCompare(Yuntai_PulseOf(YUNTAI_INIT_DEG)); /* 云台 0 度 = 1975us */
    Claw_Report("power-on", Claw_CurrentDeg());
    Yuntai_Report("power-on", Yuntai_CurrentDeg());

    /* 注意：这里故意不调用 Servo_Init()（它会再 PWM_Init 一遍）；
     * 也不给三个 Servo_t 设 moving_flag = 1 —— 中断里的 servo_update() 只在
     * moving_flag 为 1 时才插值、才写 CCR，所以现在 TIM2 中断等于空转，
     * 上电瞬间三个舵机谁都不会突然跑到源码那个初始角上去。
     * 载物台（wukuaipingtai，CH3）我们没动它的结构体：源码初值 20 度正好是
     * 1 号位，按 7/8/9 从它出发插值就行。 */

    board_init();           /* 初始化 CAN1：PA11 / PA12，500K，扩展帧 */

    USART_SendString("\r\n== ARM + CLAW test (CAN lift/push + servo claw) ==\r\n");
    USART_SendString("addr: lift = ");
    UART_SendInt(ADDR_LIFT);
    USART_SendString(", push = ");
    UART_SendInt(ADDR_PUSH);
    USART_SendString("   claw signal: ");
    USART_SendString(Pwm_PinName(g_claw_ch));
    USART_SendString(" (press 1/2/3 to switch)   yuntai signal: PA3 (TIM2_CH4)");
    USART_SendString("\r\n");
    USART_SendString("key change vs old programs: claw release = x, claw nudge = , / .\r\n");
    USART_SendString("ZERO POS: push rack fully BACK (hard stop) + lift to the TOP before power-up.\r\n");

    /* 上电就使能机械臂两轴：闭环步进使能后保持力矩，机械臂不会自己往下掉。
     * 爪子故意松着（上电时舵机是软的，按 b 才咬住）—— 跟 main_claw.c 一致。 */
    Arm_EnableBoth(true);
    Show_Menu();

    while (1)
    {
        uint8_t c = UART_GetByte();
        if (c != 0) { Handle_Key((char)c); }
    }
}
