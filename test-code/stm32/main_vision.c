/* ============================================================================
 *  main_vision.c  ---  上位机（MaixCAM2 视觉端）联调测试台
 *
 *  换程序 / 下载：
 *      .\tools\switch-main.ps1 vision -Build -Download
 *
 *  ---------------------------------------------------------------------------
 *  这个程序干什么
 *  ---------------------------------------------------------------------------
 *  专门用来**测视觉端**，不碰机械臂、不碰整条路线。就是一台"串口试验台"：
 *  你按一个键，它按 AI_HANDOFF_STM32.md 的 v3.3.0 协议发一条命令，
 *  把视觉端回的帧解析、打印、判定，并把耗时/超时/BUSY/MOVE/CRC 错误都记账。
 *
 *  协议详见 vision_proto.c/h，帧格式：
 *      AA 55 | LEN | payload | CRC8 | 0D
 *  CRC8 只覆盖 LEN+payload（poly 0x07 / init 0x00）。
 *
 *  ---------------------------------------------------------------------------
 *  【接线】集成板 3.1（与 stm32_patch/vision_uart.c 一致）
 *      MaixCAM2  TX  ->  PD2  (STM32 UART5_RX)
 *      MaixCAM2  RX  <-  PC12 (STM32 UART5_TX)
 *      GND 共地，3.3V TTL 直连，115200 8N1
 *      调试打印/按键走 USART3：PB10 = TX，PB11 = RX，115200（接 U4 座）
 *
 *  ★ 视觉端那边的 UART 是它自己的 /dev/ttyS4(UART4)，A21=TX / A22=RX。
 *    两边都是 3.3V，直连即可；**不要**接 5V。
 *
 *  ★ 中断函数冲突（必读，**只做一次**）：
 *    Keil 工程里已有 vendor/stm32/lanya/usart5.c，它定义了 UART5_IRQHandler
 *    （蓝牙 HC-05 死代码，主流程从不调用）。编译前二选一：
 *      ① 推荐：Keil 工程树里展开 lanya 分组 → 右键 usart5.c → Remove File from
 *         Group（磁盘文件还在，只是不参与编译）。**这样就行了，不用加任何宏**：
 *         vision_proto.c 默认就提供 UART5_IRQHandler。
 *         —— 这也是 vision-maixcam/stm32_patch/README.md 既定的接法。
 *      ② 一定要保留 usart5.c：在工程预定义宏里加 VISION_NO_UART5_IRQ，
 *         并在它的 UART5_IRQHandler 里加一行
 *         Vision_IsrByte((uint8_t)USART_ReceiveData(UART5));
 *    漏做的话链接期会报 "UART5_IRQHandler multiply defined"，不会静默收不到数据。
 *
 *  ---------------------------------------------------------------------------
 *  【按键表】（在 USART3 里按，普通命令以换行结束；单字符键不用回车）
 *      ? / h   打印菜单
 *      v       自检：CRC 向量 + 拆帧边界 + 回复解析（**不接视觉端**也能跑，先跑这个）
 *      l       回环自测：PC12 与 PD2 短接后，发的帧应原样收回（验 UART 通路）
 *      r       握手  R（1s 超时 ×3 次重发）
 *      a       A   启停区        → START,区号,x,y,w,h
 *      t       T   任务码        → TASK,G1,G3 / NONE,T
 *      c       C,<颜色码>        → PICK,色,x,y / NONE,C
 *      d       D   粗加工圆盘    → (MOVE,dx)*  → DISC,x,y,dx,ok
 *      e       E   暂存圆盘      → 同上
 *      f       F,<颜色码>        → STACK,x,y / NONE,F
 *      g       G   障碍物        → OBST,x,y,w,h,mm / CLEAR
 *      z       Z   结束          → OK
 *      p       跑一遍推荐全流程（交接文档 §7），逐步判定 PASS/FAIL
 *      s       连测：当前命令自动重复 5 次，统计成功率与耗时
 *      m       压力测：连续 20 次 G，统计 OBST/CLEAR 与响应时间
 *      k       打印统计（帧数/CRC 错/超时/BUSY/MOVE/未知帧 + 每个会话耗时）
 *      o       显示上一次收到的帧（原文 + 整帧 hex）
 *      x       中止当前会话（先发 R 打断视觉端）
 *      b       开关"每帧打印整帧 hex"
 *      1..4    设置颜色码（红1 黄2 蓝3 绿4）
 *      0       演示 MOVE 的像素→毫米换算（不真动电机）
 *
 *  ---------------------------------------------------------------------------
 *  【怎么算通过】
 *      1. 先按 v：自检全 PASS（证明帧/CRC/解析都对）
 *      2. 按 l（需短接 PC12-PD2）：回环 PASS（证明 UART 硬通道通）
 *      3. 接上视觉端，按 r：应 1s 内回 READY
 *      4. 按 p：11 步里至少 R/A/T/C/D/E/F/G/Z 都拿到对应回复；
 *         屏幕上同时会打 [move]/[busy]/[none] 的细节
 *      5. 按 k 看 bad_crc=0；若 bad_crc 一直涨 = 波特率/接线/电平问题
 * ========================================================================== */

#include "stm32f4xx.h"
#include "delay.h"
#include "vision_proto.h"

/* --------------------------------------------------------------------------
 *  可调项
 * -------------------------------------------------------------------------- */
#define DEFAULT_TASK_CODE   "134+123+314+231"   /* 任务码 15 字符 */
#define SOAK_REPEAT              5u             /* s 连测次数 */
#define STRESS_G_REPEAT         20u             /* m 压力测次数 */
#define LOOPBACK_WAIT_MS       300u             /* l 回环等待 */

/* 是否真的驱动底盘执行 MOVE（默认 0：只打印换算结果）*/
#define VISION_BENCH_DRIVE_MOVE  0

/* ==========================================================================
 *  ① 调试串口 USART3（PB10 = TX / PB11 = RX）115200
 * ========================================================================== */
#define DBG_BAUD  115200u

static void Dbg_Init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

    usart.USART_BaudRate            = DBG_BAUD;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART3, &usart);
    USART_Cmd(USART3, ENABLE);
}

static void Dbg_PutC(uint8_t b)
{
    USART_SendData(USART3, b);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

static void Dbg_Str(const char *s)
{
    while (*s)
    {
        Dbg_PutC((uint8_t)*s);
        s++;
    }
}

static void Dbg_Int(int32_t n)
{
    char     buf[12];
    uint8_t  i = 0;
    uint32_t u;

    if (n < 0) { Dbg_PutC('-'); u = (uint32_t)(-n); }
    else       { u = (uint32_t)n; }
    if (u == 0u) { Dbg_PutC('0'); return; }
    while (u > 0u) { buf[i++] = (char)('0' + (u % 10u)); u /= 10u; }
    while (i > 0u) { Dbg_PutC((uint8_t)buf[--i]); }
}

static void Dbg_UInt(uint32_t u) { Dbg_Int((int32_t)u); }

/* ASCII 字符串比较（不引 libc，避免 AC5 下 locale 出怪）*/
static int strcmp_ascii(const char *a, const char *b);

static uint8_t Dbg_GetC(void)
{
    if (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET)
        return (uint8_t)USART_ReceiveData(USART3);
    return 0u;
}

/* ==========================================================================
 *  ② 毫秒节拍 TIM7（1ms 自由跑）
 * ========================================================================== */
static volatile uint32_t g_ms = 0u;

void TIM7_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
        g_ms++;
    }
}

static void Tick_Init(void)
{
    TIM_TimeBaseInitTypeDef tb;
    NVIC_InitTypeDef        nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    tb.TIM_Prescaler         = 8400 - 1;      /* 84MHz/8400 = 10kHz，再 /10 = 1kHz */
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_Period            = 10 - 1;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM7, &tb);

    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel                   = TIM7_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    TIM_Cmd(TIM7, ENABLE);
}

static uint32_t Now(void) { return g_ms; }

/* ==========================================================================
 *  ③ 测试台状态
 * ========================================================================== */
static VisionParser  g_parser;
static VisionSession g_sess;

static uint8_t  g_color      = 1u;      /* 当前颜色码 1..4 */
static uint8_t  g_show_hex   = 1u;      /* 每帧是否打印整帧 hex */

static uint32_t g_sess_start_ms = 0u;   /* 本次会话开始时刻（算耗时）*/
static char     g_last_text[VISION_TEXT_MAX];
static char     g_last_cmd[VISION_TEXT_MAX];   /* 上一条发出去的命令（s 连测用）*/
static uint32_t g_last_gap_ms = 0u;
static int32_t  g_last_dx = 0;

/* 记账 */
static uint32_t g_timeouts = 0u;
static uint32_t g_unknown  = 0u;
static uint32_t g_none     = 0u;
static uint32_t g_errors   = 0u;

/* 任务码 */
static char g_task_code[VISION_TEXT_MAX] = DEFAULT_TASK_CODE;

/* 颜色码名字（红1 黄2 蓝3 绿4）*/
static const char *ColorName(uint8_t c)
{
    switch (c)
    {
    case 1u: return "红";
    case 2u: return "黄";
    case 3u: return "蓝";
    case 4u: return "绿";
    default: return "?";
    }
}

/* 拼命令字符串：buf_color("C") → "C,3"；buf_task() → "T,134+..." */
static char *StrCopy(char *dst, const char *src, uint8_t cap)
{
    uint8_t i = 0u;
    while (src[i] && i < (uint8_t)(cap - 1u))
    {
        dst[i] = src[i];
        i++;
    }
    dst[i] = 0;
    return dst;
}

static const char *buf_color(const char *prefix)
{
    static char buf[VISION_TEXT_MAX];
    buf[0] = prefix[0];
    buf[1] = ',';
    buf[2] = (char)('0' + g_color);
    buf[3] = 0;
    return buf;
}

static const char *buf_task(void)
{
    static char buf[VISION_TEXT_MAX];
    buf[0] = 'T';
    buf[1] = ',';
    StrCopy(&buf[2], g_task_code, (uint8_t)(VISION_TEXT_MAX - 2u));
    return buf;
}

/* ==========================================================================
 *  ④ 打印与判定
 * ========================================================================== */
static void PrintFrameHex(const char *payload)
{
    char hex[VISION_FRAME_MAX * 3u + 4u];
    Vision_HexFrame(payload, hex, sizeof(hex));
    Dbg_Str("      帧: ");
    Dbg_Str(hex);
    Dbg_Str("\r\n");
}

/* 期待回复的"是否算通过"判定 */
static const char *Judge(VisionMsgId id)
{
    switch (id)
    {
    case VISION_MSG_READY:  return "PASS";
    case VISION_MSG_START:  return "PASS";
    case VISION_MSG_TASK:   return "PASS";
    case VISION_MSG_PICK:   return "PASS";
    case VISION_MSG_DISC:   return "PASS";
    case VISION_MSG_STACK:  return "PASS";
    case VISION_MSG_OBST:   return "PASS";
    case VISION_MSG_CLEAR:  return "PASS";
    case VISION_MSG_OK:     return "PASS";
    case VISION_MSG_NONE:   return "FAIL(NONE)";
    case VISION_MSG_ERR:    return "FAIL(ERR)";
    case VISION_MSG_BUSY:   return "WAIT(BUSY)";
    case VISION_MSG_MOVE:   return "WAIT(MOVE)";
    default:                return "FAIL(未知帧)";
    }
}

/* 按消息类型打印细节（现场一眼能看出对不对）*/
static void PrintReply(const VisionMsg *m, uint32_t elapsed_ms)
{
    Dbg_Str("[<-] ");
    Dbg_Str(m->text);
    Dbg_Str("   ");
    Dbg_UInt(elapsed_ms);
    Dbg_Str("ms  ");
    Dbg_Str(Judge(m->id));
    Dbg_Str("\r\n");

    switch (m->id)
    {
    case VISION_MSG_START:
        Dbg_Str("      启停区=");
        Dbg_Int(m->num[0]);
        Dbg_Str("号（1=右侧有灰路 / 2=没有）  蓝区中心(");
        Dbg_Int(m->num[1]); Dbg_Str(","); Dbg_Int(m->num[2]);
        Dbg_Str(") 尺寸 ");
        Dbg_Int(m->num[3]); Dbg_Str("x"); Dbg_Int(m->num[4]);
        if (m->num[3] == 0 && m->num[4] == 0)
            Dbg_Str("  ← 坐标全 0：没看到蓝色启停区，区号仍然有效");
        Dbg_Str("\r\n");
        break;

    case VISION_MSG_TASK:
        Dbg_Str("      第一批颜色=");
        Dbg_Str(m->arg[0]);
        Dbg_Str("  第二批颜色=");
        Dbg_Str(m->arg[1]);
        Dbg_Str("（每位 1红 2黄 3蓝 4绿）\r\n");
        break;

    case VISION_MSG_PICK:
        Dbg_Str("      颜色码=");
        Dbg_Str(m->arg[0]);
        Dbg_Str(" 物料中心(");
        Dbg_Int(m->num[1]); Dbg_Str(","); Dbg_Int(m->num[2]);
        Dbg_Str(")  ← 640x480 口径，已去畸变\r\n");
        break;

    case VISION_MSG_DISC:
        g_last_dx = m->num[2];
        Dbg_Str("      圆中心(");
        Dbg_Int(m->num[0]); Dbg_Str(","); Dbg_Int(m->num[1]);
        Dbg_Str(") dx=");
        Dbg_Int(m->num[2]);
        Dbg_Str(" ok=");
        Dbg_Int(m->num[3]);
        if (m->num[3] == 1)
            Dbg_Str("  ← 已对中(|dx|<=5)。ok=0 表示超次数没对中，要自己补");
        Dbg_Str("\r\n");
        break;

    case VISION_MSG_STACK:
        Dbg_Str("      底座中心(");
        Dbg_Int(m->num[0]); Dbg_Str(","); Dbg_Int(m->num[1]);
        Dbg_Str(")  ← 只有中心，没有高度\r\n");
        break;

    case VISION_MSG_OBST:
        Dbg_Str("      障碍物(");
        Dbg_Int(m->num[0]); Dbg_Str(","); Dbg_Int(m->num[1]);
        Dbg_Str(") 尺寸 ");
        Dbg_Int(m->num[2]); Dbg_Str("x"); Dbg_Int(m->num[3]);
        Dbg_Str(" 估算距离=");
        Dbg_Int(m->num[4]);
        Dbg_Str("mm（0=未标定）\r\n");
        break;

    case VISION_MSG_NONE:
        g_none++;
        Dbg_Str("      任务失败，tag=");
        Dbg_Str(m->arg[0]);
        Dbg_Str("  ← tag 是失败的那条命令\r\n");
        break;

    case VISION_MSG_ERR:
        g_errors++;
        Dbg_Str("      视觉端异常: ");
        Dbg_Str(m->arg[0]);
        Dbg_Str("  ← camera 表示相机不可用，视觉端随后会退出\r\n");
        break;

    default:
        break;
    }

    if (g_show_hex)
        PrintFrameHex(m->text);
}

/* MOVE：视觉端请求微调（仅圆盘对齐时出现）*/
static void Move_Handle(int32_t dx)
{
    int32_t mm;
    int32_t adx = (dx < 0) ? -dx : dx;

    mm = (int32_t)((float)adx * VISION_MOVE_MM_PER_PX + 0.5f);

    Dbg_Str("[move] dx=");
    Dbg_Int(dx);
    Dbg_Str(" → 应");
    Dbg_Str(dx > 0 ? "前进" : "后退");
    Dbg_Str(" ");
    Dbg_Int(mm);
    Dbg_Str(" mm");
#if VISION_BENCH_DRIVE_MOVE
    Dbg_Str("（已按换算驱动）");
#else
    Dbg_Str("（只打印不驱动；毫米数是占位值，须现场标定 VISION_MOVE_MM_PER_PX）");
#endif
    Dbg_Str("\r\n");

#if VISION_BENCH_DRIVE_MOVE
    /* 接整车时在这里调用你的走位函数，例如：
     *   MotionParameters p; p.axis = BODY_FORWARD; p.mm = 前进?mm:-mm;
     *   p.rpm = 40; p.acceleration = CAR_MOTION_ACCELERATION;
     *   motion_start(&drive, p, Now());
     * 注意：方向反了要改视觉端的 config.ALIGN_INVERT，不要在这里翻符号。
     */
#endif
}

static void PrintStats(void)
{
    Dbg_Str("\r\n==== 统计 ====\r\n");
    Dbg_Str("帧: 好=");   Dbg_UInt(g_parser.frames_ok);
    Dbg_Str(" 坏CRC=");   Dbg_UInt(g_parser.bad_crc);
    Dbg_Str(" 坏帧尾=");  Dbg_UInt(g_parser.bad_tail);
    Dbg_Str(" LEN超限="); Dbg_UInt(g_parser.bad_len);
    Dbg_Str(" 噪声字节="); Dbg_UInt(g_parser.noise);
    Dbg_Str(" 半帧丢弃="); Dbg_UInt(g_parser.half_frame_drops);
    Dbg_Str("\r\n");
    Dbg_Str("会话: 回复=");   Dbg_UInt(g_sess.replies);
    Dbg_Str(" BUSY=");        Dbg_UInt(g_sess.busy);
    Dbg_Str(" MOVE=");        Dbg_UInt(g_sess.moves);
    Dbg_Str(" 超时=");        Dbg_UInt(g_sess.timeouts);
    Dbg_Str(" 重发=");        Dbg_UInt(g_sess.resends);
    Dbg_Str("\r\n");
    Dbg_Str("其它: NONE=");   Dbg_UInt(g_none);
    Dbg_Str(" ERR=");         Dbg_UInt(g_errors);
    Dbg_Str(" 未知帧=");      Dbg_UInt(g_unknown);
    Dbg_Str(" 超时会话=");    Dbg_UInt(g_timeouts);
    Dbg_Str("\r\n");
    if (g_parser.bad_crc || g_parser.bad_tail)
        Dbg_Str("  ★ 有坏帧：先查波特率(115200)、共地、TX/RX 是否接反\r\n");
    if (g_parser.noise)
        Dbg_Str("  ★ 有噪声字节：多为波特率不匹配或线上干扰\r\n");
    Dbg_Str("上次收到: ");
    Dbg_Str(g_last_text[0] ? g_last_text : "(还没有)");
    Dbg_Str("\r\n");
    Dbg_Str("上次间隔: "); Dbg_UInt(g_last_gap_ms); Dbg_Str("ms\r\n");
}

/* ==========================================================================
 *  ⑤ 会话发起 / 驱动
 * ========================================================================== */
/* 计划：单次 / 脚本 / 连测 / 压力测 */
enum { PLAN_NONE = 0, PLAN_SCRIPT, PLAN_SOAK, PLAN_STRESS };
static uint8_t  g_plan = PLAN_NONE;
static uint8_t  g_plan_index = 0;
static uint8_t  g_plan_total = 0;
static uint16_t g_plan_pass = 0;
static uint16_t g_plan_fail = 0;
static uint32_t g_plan_ms_sum = 0, g_plan_ms_max = 0;
static uint32_t g_obst_cnt = 0, g_clear_cnt = 0;
static char     g_soak_cmd[VISION_TEXT_MAX] = "";

/* 推荐全流程（交接文档 §7）。optional=1 表示两种回复都算过。*/
static const struct
{
    const char *cmd;
    VisionMsgId expect;
    VisionMsgId alt;            /* optional=1 时的第二种合法回复 */
    uint8_t     optional;
    const char *note;
} kScript[] =
{
    { "R", VISION_MSG_READY, VISION_MSG_EMPTY, 0u, "握手" },
    { "A", VISION_MSG_START, VISION_MSG_EMPTY, 0u, "启停区" },
    { "T," DEFAULT_TASK_CODE, VISION_MSG_TASK, VISION_MSG_EMPTY, 0u, "任务码" },
    { "C,1", VISION_MSG_PICK, VISION_MSG_EMPTY, 0u, "原料区 红" },
    { "C,3", VISION_MSG_PICK, VISION_MSG_EMPTY, 0u, "原料区 蓝" },
    { "C,4", VISION_MSG_PICK, VISION_MSG_EMPTY, 0u, "原料区 绿" },
    { "D", VISION_MSG_DISC, VISION_MSG_EMPTY, 0u, "粗加工圆盘（含 MOVE 闭环）" },
    { "E", VISION_MSG_DISC, VISION_MSG_EMPTY, 0u, "暂存圆盘" },
    { "F,2", VISION_MSG_STACK, VISION_MSG_EMPTY, 0u, "码垛 黄" },
    { "G", VISION_MSG_OBST, VISION_MSG_CLEAR, 1u, "障碍物（也可能 CLEAR）" },
    { "Z", VISION_MSG_OK, VISION_MSG_EMPTY, 0u, "结束" }
};
#define SCRIPT_N  (uint8_t)(sizeof(kScript) / sizeof(kScript[0]))

static uint32_t TimeoutOf(const char *cmd)
{
    return (cmd[0] == 'G' && cmd[1] == 0) ? VISION_TIMEOUT_G_MS : VISION_TIMEOUT_MS;
}

static void Bench_Start(const char *cmd, const char *note)
{
    char hex[VISION_FRAME_MAX * 3u + 4u];

    if (Vision_SessionActive(&g_sess))
    {
        Dbg_Str("[!] 上一个会话还没结束（按 x 可中止）\r\n");
        return;
    }
    StrCopy(g_last_cmd, cmd, (uint8_t)sizeof(g_last_cmd));
    g_sess_start_ms = Now();
    Dbg_Str("[->] ");
    if (note && note[0])
    {
        Dbg_Str(note);
        Dbg_Str("  ");
    }
    Dbg_Str("发: ");
    Dbg_Str(cmd);
    Dbg_Str("\r\n");
    if (g_show_hex)
    {
        Vision_HexFrame(cmd, hex, sizeof(hex));
        Dbg_Str("      帧: ");
        Dbg_Str(hex);
        Dbg_Str("\r\n");
    }
    Vision_SessionStart(&g_sess, cmd, Now(), TimeoutOf(cmd),
                        (cmd[0] == 'R') ? VISION_HANDSHAKE_RETRY : VISION_CMD_RETRY);
}

static void PrintPlanSummary(void)
{
    if (g_plan == PLAN_SCRIPT)
        Dbg_Str("\r\n==== 全流程结果 ====\r\n");
    else if (g_plan == PLAN_SOAK)
        Dbg_Str("\r\n==== 连测结果 ====\r\n");
    else
        Dbg_Str("\r\n==== 压力测结果 ====\r\n");

    Dbg_Str("通过 "); Dbg_UInt(g_plan_pass);
    Dbg_Str(" / 失败 "); Dbg_UInt(g_plan_fail);
    Dbg_Str(" / 共 "); Dbg_UInt(g_plan_total);
    Dbg_Str("   平均 "); Dbg_UInt(g_plan_total ? (g_plan_ms_sum / g_plan_total) : 0u);
    Dbg_Str("ms  最慢 "); Dbg_UInt(g_plan_ms_max); Dbg_Str("ms\r\n");
    if (g_obst_cnt || g_clear_cnt)
    {
        Dbg_Str("G: OBST×"); Dbg_UInt(g_obst_cnt);
        Dbg_Str("  CLEAR×"); Dbg_UInt(g_clear_cnt); Dbg_Str("\r\n");
    }
    if (g_plan_fail == 0u && g_plan_total > 0u)
        Dbg_Str("[PLAN PASS]\r\n");
    else
        Dbg_Str("[PLAN FAIL]\r\n");
    Dbg_Str("按 k 看统计，按 ? 看菜单\r\n");
}

/* 会话结束（1=拿到回复 / 0=超时失败）后统一处理计划推进 */
static void Plan_Advance(uint8_t ok)
{
    if (g_plan == PLAN_NONE)
        return;

    g_plan_index++;
    if (ok) g_plan_pass++;
    else    g_plan_fail++;

    if (g_plan_index >= g_plan_total)
    {
        PrintPlanSummary();
        g_plan = PLAN_NONE;
        return;
    }

    if (g_plan == PLAN_SCRIPT)
    {
        const char *cmd = kScript[g_plan_index].cmd;
        Dbg_Str("\r\n");
        Bench_Start(cmd, kScript[g_plan_index].note);
    }
    else
    {
        Bench_Start(g_soak_cmd, (g_plan == PLAN_STRESS) ? "压力测" : "连测");
    }
}

/* 返回 1 = 会话本步有结果（脚本/连测要推进）*/
static uint8_t Bench_SessionStep(uint32_t now)
{
    VisionMsg m;
    int       rc;
    uint32_t  elapsed;

    rc = Vision_SessionPoll(&g_sess, now, &m);
    if (rc == 2)
    {
        Move_Handle(m.num[0]);
        return 0u;
    }
    if (rc == 0)
        return 0u;

    elapsed = (uint32_t)(now - g_sess_start_ms);

    if (rc == 1)
    {
        PrintReply(&m, elapsed);
        g_last_text[0] = 0;
        {
            uint8_t i = 0u;
            while (m.text[i] && i < (uint8_t)(VISION_TEXT_MAX - 1u))
            {
                g_last_text[i] = m.text[i];
                i++;
            }
            g_last_text[i] = 0;
        }
        g_last_gap_ms = elapsed;
        if (m.id == VISION_MSG_OBST)  g_obst_cnt++;
        if (m.id == VISION_MSG_CLEAR) g_clear_cnt++;
        if (m.id == VISION_MSG_UNKNOWN) g_unknown++;

        /* 计划判定 */
        if (g_plan == PLAN_SCRIPT)
        {
            uint8_t ok = (uint8_t)((m.id == kScript[g_plan_index].expect)
                                   || (kScript[g_plan_index].optional
                                       && m.id == kScript[g_plan_index].alt));
            if (!ok)
            {
                Dbg_Str("      [步骤 FAIL] 期待 ");
                Dbg_Str(Vision_MsgName(kScript[g_plan_index].expect));
                if (kScript[g_plan_index].optional)
                {
                    Dbg_Str(" 或 ");
                    Dbg_Str(Vision_MsgName(kScript[g_plan_index].alt));
                }
                Dbg_Str("\r\n");
            }
            Plan_Advance(ok);
        }
        else if (g_plan != PLAN_NONE)
        {
            uint8_t ok = (uint8_t)(m.id != VISION_MSG_NONE && m.id != VISION_MSG_ERR
                                   && m.id != VISION_MSG_UNKNOWN);
            Plan_Advance(ok);
        }
        else
        {
            Dbg_Str("按 ? 看菜单\r\n");
        }
        return 1u;
    }

    /* rc == -1：超时 */
    g_timeouts++;
    Dbg_Str("[<-] 超时（已重发用尽）  ");
    Dbg_UInt(elapsed);
    Dbg_Str("ms\r\n");
    if (g_plan == PLAN_SCRIPT)
        Dbg_Str("      [步骤 FAIL] 无回复\r\n");
    Plan_Advance(0u);
    return 1u;
}

/* 不在会话里时，收到的主动帧（如视觉端上电自己发的 READY）*/
static void Bench_DrainUnsolicited(void)
{
    VisionMsg m;
    while (Vision_TakeMsg(&m))
    {
        Dbg_Str("[<-] (主动) ");
        Dbg_Str(m.text);
        Dbg_Str("  ");
        Dbg_Str(Judge(m.id));
        Dbg_Str("\r\n");
        g_last_text[0] = 0;
        {
            uint8_t i = 0u;
            while (m.text[i] && i < (uint8_t)(VISION_TEXT_MAX - 1u))
            {
                g_last_text[i] = m.text[i];
                i++;
            }
            g_last_text[i] = 0;
        }
        if (g_show_hex)
            PrintFrameHex(m.text);
    }
}

/* ==========================================================================
 *  ⑥ 自检（不需要接视觉端）
 * ========================================================================== */
static int g_st_pass = 0, g_st_total = 0;

static void ST(const char *name, uint8_t ok)
{
    g_st_total++;
    if (ok) { g_st_pass++; Dbg_Str("  [OK] "); }
    else    { Dbg_Str("  [FAIL] "); }
    Dbg_Str(name);
    Dbg_Str("\r\n");
}

static void SelfTest(void)
{
    static const struct { const char *payload; const char *hex; } kVec[] =
    {
        { "D",                 "AA 55 01 44 CE 0D" },
        { "R",                 "AA 55 01 52 AC 0D" },
        { "Z",                 "AA 55 01 5A 94 0D" },
        { "C,1",               "AA 55 03 43 2C 31 C4 0D" },
        { "T,134+123+314+231", "AA 55 11 54 2C 31 33 34 2B 31 32 33 2B 33 31 34 2B 32 33 31 B1 0D" }
    };
    VisionParser p;
    VisionMsg    m;
    char         hex[VISION_FRAME_MAX * 3u + 4u];
    unsigned     i;

    g_st_pass = g_st_total = 0;
    Dbg_Str("\r\n==== 自检（不需要接视觉端）====\r\n");

    /* 1) CRC / 组帧：与交接文档 §2.1 逐字节比 */
    Dbg_Str("-- CRC / 组帧 --\r\n");
    for (i = 0u; i < (unsigned)(sizeof(kVec) / sizeof(kVec[0])); i++)
    {
        Vision_HexFrame(kVec[i].payload, hex, sizeof(hex));
        ST(kVec[i].payload, (uint8_t)(strcmp_ascii(hex, kVec[i].hex) == 0));
    }

    /* 2) 拆帧 */
    Dbg_Str("-- 拆帧 --\r\n");
    Vision_ParserReset(&p, 0u);
    {
        static const char *kGood = "DISC,318,242,-2,1";
        uint8_t  frame[VISION_FRAME_MAX];
        uint16_t n = Vision_BuildFrame(kGood, frame, sizeof(frame));
        uint16_t k;
        uint8_t  hits = 0u;
        for (k = 0u; k < n; k++)
            if (Vision_ParserFeed(&p, frame[k], 0u))
                hits++;
        ST("完整帧解析", (uint8_t)(hits == 1u && strcmp_ascii((const char *)p.payload, kGood) == 0));
    }
    Vision_ParserReset(&p, 0u);
    {
        uint8_t  frame[VISION_FRAME_MAX];
        uint16_t n = Vision_BuildFrame("READY", frame, sizeof(frame));
        uint16_t k;
        uint8_t  hits = 0u;
        frame[n - 2u] ^= 0xFFu;
        for (k = 0u; k < n; k++)
            if (Vision_ParserFeed(&p, frame[k], 0u))
                hits++;
        ST("坏 CRC 必须丢弃", (uint8_t)(hits == 0u && p.bad_crc == 1u));
    }
    Vision_ParserReset(&p, 0u);
    {
        uint8_t  frame[VISION_FRAME_MAX];
        uint16_t n = Vision_BuildFrame("TASK,134,314", frame, sizeof(frame));
        uint16_t k;
        uint8_t  hits = 0u;
        for (k = 0u; k < n; k++)
        {
            Vision_ParserTick(&p, (uint32_t)k * 10u);      /* 分片但 <200ms */
            if (Vision_ParserFeed(&p, frame[k], (uint32_t)k * 10u))
                hits++;
        }
        ST("逐字节分片", (uint8_t)(hits == 1u));
    }
    Vision_ParserReset(&p, 0u);
    {
        uint8_t frame[8];
        uint16_t n = Vision_BuildFrame("OK", frame, sizeof(frame));
        uint16_t k;
        for (k = 0u; k < 3u && k < n; k++)
            Vision_ParserFeed(&p, frame[k], 0u);
        Vision_ParserTick(&p, VISION_HALF_FRAME_MS);
        ST("半帧 200ms 超时丢弃", (uint8_t)(p.half_frame_drops == 1u));
    }

    /* 3) 回复解析（含负数）*/
    Dbg_Str("-- 回复解析 --\r\n");
    Vision_ParseReply("DISC,318,242,-2,1", 5u, &m);
    ST("DISC 解析", (uint8_t)(m.id == VISION_MSG_DISC && m.num[0] == 318
                              && m.num[2] == -2 && m.num[3] == 1 && m.argc == 4u));
    Vision_ParseReply("NONE,T", 5u, &m);
    ST("NONE,tag 解析", (uint8_t)(m.id == VISION_MSG_NONE && strcmp_ascii(m.arg[0], "T") == 0));
    Vision_ParseReply("MOVE,-8", 5u, &m);
    ST("MOVE 负数 dx", (uint8_t)(m.id == VISION_MSG_MOVE && m.num[0] == -8));
    Vision_ParseReply("STACK,340,180", 5u, &m);
    ST("STACK 只有两个数", (uint8_t)(m.id == VISION_MSG_STACK && m.argc == 2u));

    Dbg_Str("自检 ");
    Dbg_Int(g_st_pass);
    Dbg_Str("/");
    Dbg_Int(g_st_total);
    Dbg_Str(g_st_pass == g_st_total ? "  [SELFTEST PASS]\r\n" : "  [SELFTEST FAIL]\r\n");
}

/* ==========================================================================
 *  ⑦ 回环自测（PC12 与 PD2 短接）
 * ========================================================================== */
static void LoopbackTest(void)
{
    uint8_t  frame[VISION_FRAME_MAX];
    uint16_t n, got = 0u, same = 1u;
    int      b;
    uint32_t t0;
    char     hex[VISION_FRAME_MAX * 3u + 4u];

    if (Vision_SessionActive(&g_sess))
    {
        Dbg_Str("[!] 先等会话结束（或按 x）\r\n");
        return;
    }
    Dbg_Str("\r\n==== 回环自测（需把 PC12 与 PD2 短接）====\r\n");

    while (Vision_RxPop() >= 0) { }                  /* 清空 */
    Vision_ParserReset(&g_parser, Now());

    n = Vision_BuildFrame("D", frame, sizeof(frame));
    Vision_TxBytes(frame, n);                        /* 直接发，不建会话 */

    Dbg_Str("已发 ");
    Dbg_UInt(n);
    Dbg_Str(" 字节: ");
    Vision_HexFrame("D", hex, sizeof(hex));
    Dbg_Str(hex);
    Dbg_Str("\r\n");

    t0 = Now();
    while ((uint32_t)(Now() - t0) < LOOPBACK_WAIT_MS)
    {
        while ((b = Vision_RxPop()) >= 0)
        {
            if (got < n && (uint8_t)b == frame[got])
                got++;
            else
                same = 0u;
        }
        if (got >= n)
            break;
    }

    Dbg_Str("收回 ");
    Dbg_UInt(got);
    Dbg_Str("/");
    Dbg_UInt(n);
    Dbg_Str(" 字节  ");
    if (got == n && same)
        Dbg_Str("[LOOPBACK PASS]\r\n");
    else
    {
        Dbg_Str("[LOOPBACK FAIL]\r\n");
        if (got == 0u)
        {
            Dbg_Str("  收不到任何字节：PC12/PD2 没短接，或中断没接（见文件头 ★ 中断冲突）\r\n");
            Dbg_Str("  还有一种可能：UART5 的时钟/引脚被别的模块抢占\r\n");
        }
        else
            Dbg_Str("  字节对不上：多半是波特率或分频不对\r\n");
    }
}

/* ==========================================================================
 *  ⑧ ASCII 字符串比较（不引 libc，避免 AC5 下因 locale 出怪）
 * ========================================================================== */
static int strcmp_ascii(const char *a, const char *b)
{
    while (*a && *a == *b) { a++; b++; }
    return (int)((uint8_t)*a) - (int)((uint8_t)*b);
}

/* ==========================================================================
 *  ⑨ 菜单与按键
 * ========================================================================== */
static void Menu(void)
{
    Dbg_Str("\r\n==== VISION 测试台 菜单 ====\r\n");
    Dbg_Str(" ?/h 菜单          v 自检(不接视觉端)   l 回环自测(短接PC12-PD2)\r\n");
    Dbg_Str(" r 握手R           a 启停区A            t 任务码T\r\n");
    Dbg_Str(" c C,<颜色码>      d 圆盘D(MOVE闭环)    e 圆盘E\r\n");
    Dbg_Str(" f F,<颜色码>      g 障碍物G            z 结束Z\r\n");
    Dbg_Str(" p 跑全流程         s 连测5次            m 压力测G×20\r\n");
    Dbg_Str(" k 统计            o 上次的帧           x 中止会话       b 帧hex开关\r\n");
    Dbg_Str(" 1..4 设颜色码(红1黄2蓝3绿4)             0 演示MOVE换算\r\n");
    Dbg_Str("当前颜色码=");
    Dbg_Int(g_color);
    Dbg_Str("(");
    Dbg_Str(ColorName(g_color));
    Dbg_Str(")  任务码=");
    Dbg_Str(g_task_code);
    Dbg_Str("\r\n");
}

static void StartPlan(uint8_t plan, uint8_t total, const char *cmd)
{
    if (Vision_SessionActive(&g_sess))
    {
        Dbg_Str("[!] 忙：先等会话结束或按 x\r\n");
        return;
    }
    g_plan       = plan;
    g_plan_index = 0u;
    g_plan_total = total;
    g_plan_pass  = 0u;
    g_plan_fail  = 0u;
    g_plan_ms_sum = 0u;
    g_plan_ms_max = 0u;
    g_obst_cnt   = 0u;
    g_clear_cnt  = 0u;

    if (cmd)
    {
        uint8_t i = 0u;
        while (cmd[i] && i < (uint8_t)(VISION_TEXT_MAX - 1u))
        {
            g_soak_cmd[i] = cmd[i];
            i++;
        }
        g_soak_cmd[i] = 0;
    }

    if (plan == PLAN_SCRIPT)
        Bench_Start(kScript[0].cmd, kScript[0].note);
    else
        Bench_Start(g_soak_cmd, (plan == PLAN_STRESS) ? "压力测" : "连测");
}

static void Key(char c)
{
    /* 会话进行中只允许 中止 / 统计 / 菜单 */
    if (Vision_SessionActive(&g_sess) && c != 'x' && c != 'k' && c != '?' && c != 'h')
    {
        if (c != 0)
            Dbg_Str("[!] 会话进行中（等回复或按 x 中止）\r\n");
        return;
    }

    switch (c)
    {
    case '?':
    case 'h': Menu(); break;

    case 'v': SelfTest(); break;
    case 'l': LoopbackTest(); break;

    case 'r': g_plan = PLAN_NONE; Bench_Start("R", "握手"); break;
    case 'a': Bench_Start("A", "启停区"); break;
    case 't': Bench_Start(buf_task(), "任务码"); break;
    case 'c': Bench_Start(buf_color("C"), "原料区"); break;
    case 'd': Bench_Start("D", "粗加工圆盘"); break;
    case 'e': Bench_Start("E", "暂存圆盘"); break;
    case 'f': Bench_Start(buf_color("F"), "码垛"); break;
    case 'g': Bench_Start("G", "障碍物"); break;
    case 'z': Bench_Start("Z", "结束"); break;

    case 'p': StartPlan(PLAN_SCRIPT, SCRIPT_N, 0); break;
    case 's':
        if (!g_last_cmd[0])
            Dbg_Str("[!] 还没有发过命令；先按 a/c/d/g… 再按 s\r\n");
        else
            StartPlan(PLAN_SOAK, SOAK_REPEAT, g_last_cmd);
        break;
    case 'm': StartPlan(PLAN_STRESS, STRESS_G_REPEAT, "G"); break;

    case 'k': PrintStats(); break;
    case 'o':
        Dbg_Str("上次收到: ");
        Dbg_Str(g_last_text[0] ? g_last_text : "(还没有)");
        Dbg_Str("  间隔 ");
        Dbg_UInt(g_last_gap_ms);
        Dbg_Str("ms\r\n");
        if (g_last_text[0] && g_show_hex)
            PrintFrameHex(g_last_text);
        break;

    case 'x':
        if (Vision_SessionActive(&g_sess))
        {
            Vision_SessionCancel(&g_sess);
            g_plan = PLAN_NONE;
            Dbg_Str("[!] 已中止并将发 R 打断视觉端\r\n");
            Vision_TxPayload("R");
        }
        break;

    case 'b':
        g_show_hex = (uint8_t)(g_show_hex ? 0u : 1u);
        Dbg_Str(g_show_hex ? "帧 hex 打印: 开\r\n" : "帧 hex 打印: 关\r\n");
        break;

    case '0':
        Dbg_Str("MOVE 换算演示（VISION_MOVE_MM_PER_PX=");
        Dbg_Int((int32_t)(VISION_MOVE_MM_PER_PX * 100.0f));
        Dbg_Str("/100 mm/px）\r\n");
        Move_Handle(40);
        Move_Handle(-20);
        Dbg_Str("  ★ 现场标定：按 d，记下每次 MOVE 的 dx 和车实际前后走的毫米数，\r\n");
        Dbg_Str("    用 毫米/像素 反推后改 VISION_MOVE_MM_PER_PX。方向反了改视觉端 ALIGN_INVERT\r\n");
        break;

    case '1': case '2': case '3': case '4':
        g_color = (uint8_t)(c - '0');
        Dbg_Str("颜色码=");
        Dbg_Int(g_color);
        Dbg_Str("(");
        Dbg_Str(ColorName(g_color));
        Dbg_Str(")  红1 黄2 蓝3 绿4\r\n");
        break;

    default:
        break;
    }
}

/* ==========================================================================
 *  ⑩ main
 * ========================================================================== */
int main(void)
{
    delay_init(168);
    Tick_Init();
    Dbg_Init();
    Vision_Init(115200u);
    Vision_ParserReset(&g_parser, Now());
    Vision_ResetMailbox();

    Dbg_Str("\r\n== VISION bench boot (protocol v3.3.0) ==\r\n");
    Dbg_Str("wiring: Maix TX->PD2(UART5_RX), Maix RX<-PC12(UART5_TX), GND, 115200 8N1\r\n");
    Dbg_Str("debug : USART3 PB10=TX PB11=RX 115200\r\n");
    Dbg_Str("先按 v 自检；按 l 前先把 PC12 与 PD2 短接\r\n");
    Menu();

    for (;;)
    {
        uint32_t now = Now();
        uint8_t  c;

        /* 1) 收字节 → 拆帧 → 投信箱 */
        (void)Vision_Pump(&g_parser, now);

        /* 2) 按键 */
        c = Dbg_GetC();
        if (c != 0u)
            Key((char)c);

        /* 3) 会话/计划推进；没会话时打印主动帧 */
        if (Vision_SessionActive(&g_sess))
            (void)Bench_SessionStep(now);
        else
            Bench_DrainUnsolicited();
    }
}