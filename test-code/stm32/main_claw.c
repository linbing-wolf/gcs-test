/* ============================================================================
 *  STM32 单独测试"爪子"（机械抓手 / 舵机）—— 串口按键版
 *
 *  【这是什么】
 *    只测爪子：不碰 CAN 电机、不碰走位程序。烧进去以后板子只干一件事 ——
 *    把 PWM 脉冲发到爪子那根信号线上，让你按键试角度、找机械行程。
 *
 *  【为什么不需要电机地址】
 *    爪子是舵机：靠"一根信号线上的脉冲宽度"决定角度，插在哪个脚就是哪一路通道，
 *    没有地址、没有 ID、不需要 CAN。参考工程 move_all() 里那个"两个电机"
 *    （平推平台 + 升降）才是 CAN 步进电机，那才要 CAN 地址；它们跟爪子不是一回事。
 *    所以本程序完全不初始化 CAN，也不给任何电机发命令 —— 电机插着也不动。
 *
 *  【接线：只需要三根】
 *    舵机 红    +5V  ── 电源模块 绿色座 5V   ★ 千万别接蓝色座 12V，会烧舵机
 *    舵机 棕/黑 GND  ── 电源模块 GND，并且要和核心板 GND 共地
 *    舵机 橙/黄/白 信号 ── 核心板 PA2 （= TIM2_CH3）
 *    核心板用 USB 供电，串口插板子右边的 USB UART 口（115200）。
 *    VP230 / 四个轮子的电机 都可以不管，也可以留着不接。
 *
 *  【按键表】★ 串口打印全用英文，是为了不出现乱码，含义看这里 ★
 *    h 或 ?     重新打印按键表
 *    b          先按这个：抓住中心角（上电时舵机是松的，按 b 才通电咬住）
 *    o / c      开到 CLAW_OPEN_DEG / 合到 CLAW_CLOSE_DEG
 *    + / -      当前角度 ±CLAW_STEP_DEG（微调、试方向用）
 *    a135#      直接走到 135 度：按 a 开始输入，打数字，最后打 # 结束
 *    t          自动来回开合 SWEEP_TIMES 趟（确认机械不顶死之后再按）
 *    s          松力（停止发脉冲，爪子变松，可以用手掰）
 *    p          打印当前参数和状态
 *
 *  【第一次上电的推荐顺序】
 *    1) 先按 s 松力，用手把爪子掰到中间，试着摸清机械行程大概多少（别硬顶）
 *    2) 按 b 对中，看到它咬住不动（这一步它会一下子跑到 90 度，手拿开）
 *    3) 按 + / - 每次 2 度，确认方向：+ 是往哪边转
 *    4) 拿 + / - 试出两个机械极限，把下面 CLAW_MIN_DEG / CLAW_MAX_DEG
 *       改成"极限再退 5 度"，重新编译下载，这样以后就不会顶死
 *    5) 最后才试 o / c / t
 * ========================================================================== */

#include "stm32f4xx.h"
#include "PWM.h"
#include "delay.h"


/* ==========================================================================
 *  ① 参数区（要改就改这里）
 * ========================================================================== */

/* 爪子信号线接在 TIM2 的哪个通道：2 = PA1，3 = PA2（我们车上这根），4 = PA3 */
#define CLAW_CH             3

/* 舵机总量程：这个数只决定"角度 -> 脉宽"的比例。
 * 改成 270 是为了跟参考工程对齐 —— 源码 servo.c:18 就是 Angle / 270 * 2000 + 500。
 * 两边都用 270，下面填的 6 / 45 才和源码是同一个脉宽、同一个物理位置；
 * 如果这里写 180，同样填 45，发出去的脉宽会变成 1000us（源码是 833us），开合幅度就不同了。 */
#define CLAW_TRAVEL_DEG     270.0f

/* 脉宽换算：0 度 -> 500us，CLAW_TRAVEL_DEG 度 -> 2500us（50Hz，1 格 = 1us）
 * 按 270 算：6 度 = 544us，45 度 = 833us。
 * （源码 PWM.c:162 还额外减了 1 度，约 7us，可以忽略，这里不加。） */
#define CLAW_PULSE_MIN      500.0f
#define CLAW_PULSE_SPAN     2000.0f

/* 机械允许的角度上下限：先用小范围试，找到极限后再改这里。
 * ★ 下限必须小于 CLAW_CLOSE_DEG：源码合爪要走到 6 度，
 *   原来写 20 会把 6 钳成 20（Claw_PulseOf 里 clamp），看起来"改了没用"。 */
#define CLAW_MIN_DEG        0.0f
#define CLAW_MAX_DEG        180.0f

/* 三个常用位置（都是"名义角度"，必须按我们的齿轮组重新标定）
 * 开 / 合 照抄参考工程 PWM.c：set_zhuashou_he() = 6.0f（合爪），
 *                              set_zhuashou_kai() = 45.0f（开爪）。
 * 中心 90 是我们自己加的手动测试位，源码里没有这个值。 */
#define CLAW_CENTER_DEG     90.0f
#define CLAW_OPEN_DEG       45.0f
#define CLAW_CLOSE_DEG      6.0f

/* 动作参数 */
#define CLAW_STEP_DEG       2.0f     /* + / - 一次走多少度 */
#define MOVE_TIME_MS        400u     /* 一次开合用多久（越小越快越猛） */
#define SWEEP_TIME_MS       600u     /* 自动来回时每一步用多久 */
#define STEP_MS             20u      /* 中间插值的节拍：20ms 一步 */
#define SWEEP_TIMES         3u       /* 自动来回几趟 */

#define UART_BAUD           115200


/* ==========================================================================
 *  ② 串口（USART1：PA9 = TX，PA10 = RX）
 * ========================================================================== */

static void MyUART_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    USART_InitStructure.USART_BaudRate            = UART_BAUD;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

static void UART_SendByte(uint8_t b)
{
    USART_SendData(USART1, b);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

static void UART_SendString(char *s)
{
    while (*s != '\0') { UART_SendByte((uint8_t)*s); s++; }
}

static void UART_SendInt(uint32_t n)
{
    char    buf[12];
    uint8_t i = 0;

    if (n == 0u) { UART_SendByte('0'); return; }

    while (n > 0u) { buf[i++] = (char)('0' + (n % 10u)); n = n / 10u; }
    while (i > 0u) { UART_SendByte((uint8_t)buf[--i]); }
}

/* 打印一位小数，例如 92.0 */
static void UART_SendFixed1(float v)
{
    int32_t x;

    if (v < 0.0f) { UART_SendByte('-'); v = -v; }
    x = (int32_t)(v * 10.0f + 0.5f);
    UART_SendInt((uint32_t)(x / 10));
    UART_SendByte('.');
    UART_SendByte((uint8_t)('0' + (x % 10)));
}


/* ==========================================================================
 *  ③ 爪子输出：角度 -> 脉宽 -> CCR
 * ========================================================================== */

/* 当前角度，存成"度 x10"的整数，省掉浮点打印的麻烦 */
static uint16_t g_deg_x10 = (uint16_t)(CLAW_CENTER_DEG * 10.0f + 0.5f);

static float Claw_CurrentDeg(void)
{
    return (float)g_deg_x10 / 10.0f;
}

static void Claw_SetDegState(float deg)
{
    g_deg_x10 = (uint16_t)(deg * 10.0f + 0.5f);
}

/* 把 CCR 值写到爪子这一路；写 0 = 整周期低电平 = 不发脉冲 = 舵机松力 */
static void Claw_WriteCompare(uint16_t ccr)
{
#if   (CLAW_CH == 2)
    PWM_SetCompare2(ccr);
#elif (CLAW_CH == 3)
    PWM_SetCompare3(ccr);
#elif (CLAW_CH == 4)
    PWM_SetCompare4(ccr);
#else
#error "CLAW_CH must be 2 (PA1), 3 (PA2) or 4 (PA3)"
#endif
}

static uint16_t Claw_PulseOf(float deg)
{
    float p;

    if (deg < CLAW_MIN_DEG) deg = CLAW_MIN_DEG;
    if (deg > CLAW_MAX_DEG) deg = CLAW_MAX_DEG;

    p = CLAW_PULSE_MIN + deg / CLAW_TRAVEL_DEG * CLAW_PULSE_SPAN;
    if (p < 1.0f)     p = 1.0f;
    if (p > 19999.0f) p = 19999.0f;

    return (uint16_t)(p + 0.5f);
}

static void Claw_WriteAngle(float deg)
{
    Claw_WriteCompare(Claw_PulseOf(deg));
}

static void Claw_Release(void)
{
    Claw_WriteCompare(0u);
}

static void Claw_Report(char *tag, float deg)
{
    UART_SendString("[claw] ");
    UART_SendString(tag);
    UART_SendString(" deg=");
    UART_SendFixed1(deg);
    UART_SendString(" pulse=");
    UART_SendInt((uint32_t)Claw_PulseOf(deg));
    UART_SendString("us\r\n");
}


/* ==========================================================================
 *  ④ 动作：S 曲线插值移动（6t^5-15t^4+10t^3，起步/停下都软，不甩齿轮）
 * ========================================================================== */

static void Claw_MoveTo(float target, uint32_t ms)
{
    float    start = Claw_CurrentDeg();
    uint32_t steps;
    uint32_t i;

    if (target < CLAW_MIN_DEG) target = CLAW_MIN_DEG;
    if (target > CLAW_MAX_DEG) target = CLAW_MAX_DEG;
    if (start  < CLAW_MIN_DEG) start  = CLAW_MIN_DEG;
    if (start  > CLAW_MAX_DEG) start  = CLAW_MAX_DEG;

    steps = ms / STEP_MS;
    if (steps < 1u) steps = 1u;

    for (i = 1u; i <= steps; i++)
    {
        float tau = (float)i / (float)steps;
        float s   = 6.0f * tau * tau * tau * tau * tau
                  - 15.0f * tau * tau * tau * tau
                  + 10.0f * tau * tau * tau;
        float d   = start + (target - start) * s;

        Claw_SetDegState(d);
        Claw_WriteAngle(d);
        delay_ms1(STEP_MS);
    }

    Claw_SetDegState(target);
    Claw_WriteAngle(target);
}


/* ==========================================================================
 *  ⑤ 按键处理
 * ========================================================================== */

static uint8_t  g_input_mode = 0;   /* 1 = 正在输入角度 */
static uint16_t g_input_val  = 0;
static uint8_t  g_skip_lf    = 0;

static void ShowHelp(void)
{
    UART_SendString("\r\n");
    UART_SendString("==== CLAW test (servo only, no CAN) ====\r\n");
    UART_SendString(" h / ?   this help\r\n");
    UART_SendString(" b       hold CENTER   (do this first after power-up)\r\n");
    UART_SendString(" o       open\r\n");
    UART_SendString(" c       close\r\n");
    UART_SendString(" + / -   nudge +/- step deg\r\n");
    UART_SendString(" a135#   go to 135 deg  (a -> digits -> # )\r\n");
    UART_SendString(" t       auto sweep open/close\r\n");
    UART_SendString(" s       release (no pulses, claw goes soft)\r\n");
    UART_SendString(" p       print params / state\r\n");
    UART_SendString("---------------------------------------\r\n");
}

static void ShowParams(void)
{
    float cur = Claw_CurrentDeg();

    UART_SendString("[param] channel = TIM2_CH");
    UART_SendInt((uint32_t)CLAW_CH);
    UART_SendString(" (PA");
    UART_SendInt((uint32_t)(CLAW_CH - 1));
    UART_SendString(")\r\n");

    UART_SendString("[param] travel = ");
    UART_SendFixed1(CLAW_TRAVEL_DEG);
    UART_SendString(" deg -> pulse ");
    UART_SendInt((uint32_t)CLAW_PULSE_MIN);
    UART_SendString("..");
    UART_SendInt((uint32_t)(CLAW_PULSE_MIN + CLAW_PULSE_SPAN));
    UART_SendString("us\r\n");

    UART_SendString("[param] limits = ");
    UART_SendFixed1(CLAW_MIN_DEG);
    UART_SendString(" .. ");
    UART_SendFixed1(CLAW_MAX_DEG);
    UART_SendString(" deg  (edit CLAW_MIN_DEG / CLAW_MAX_DEG)\r\n");

    UART_SendString("[param] open/close/center = ");
    UART_SendFixed1(CLAW_OPEN_DEG);
    UART_SendString(" / ");
    UART_SendFixed1(CLAW_CLOSE_DEG);
    UART_SendString(" / ");
    UART_SendFixed1(CLAW_CENTER_DEG);
    UART_SendString("\r\n");

    UART_SendString("[param] step = ");
    UART_SendFixed1(CLAW_STEP_DEG);
    UART_SendString(" deg, move = ");
    UART_SendInt(MOVE_TIME_MS);
    UART_SendString(" ms\r\n");

    UART_SendString("[state] now = ");
    UART_SendFixed1(cur);
    UART_SendString(" deg, pulse = ");
    UART_SendInt((uint32_t)Claw_PulseOf(cur));
    UART_SendString("us\r\n");
}

static void HandleKey(char c)
{
    float cur;

    /* --- 角度输入模式 --- */
    if (g_input_mode == 1)
    {
        if (c >= '0' && c <= '9')
        {
            g_input_val = (uint16_t)(g_input_val * 10u + (uint16_t)(c - '0'));
            if (g_input_val > 999u) g_input_val = 999u;
            UART_SendByte((uint8_t)c);          /* 回显数字 */
            return;
        }
        if (c == '#' || c == '\r')
        {
            float tgt = (float)g_input_val;

            g_input_mode = 0;
            if (c == '\r') g_skip_lf = 1u;
            UART_SendString("\r\n");
            if (tgt < CLAW_MIN_DEG || tgt > CLAW_MAX_DEG)
            {
                UART_SendString("[warn] outside CLAW_MIN_DEG..CLAW_MAX_DEG, clamped\r\n");
            }
            Claw_MoveTo(tgt, MOVE_TIME_MS);
            Claw_Report("goto", Claw_CurrentDeg());
            return;
        }
        g_input_mode = 0;
        UART_SendString("\r\n[cancel] not a digit\r\n");
        return;
    }

    if (c == '\r' || c == '\n')
    {
        if (g_skip_lf == 1u && c == '\n') { g_skip_lf = 0u; return; }
        g_skip_lf = 0u;
        return;                                 /* 单个回车不当命令 */
    }
    g_skip_lf = 0u;

    UART_SendString("[cmd] '");
    UART_SendByte((uint8_t)c);
    UART_SendString("' -> ");

    switch (c)
    {
    case 'h':
    case '?':
        UART_SendString("help\r\n");
        ShowHelp();
        break;

    case 'b':
        UART_SendString("hold CENTER\r\n");
        Claw_MoveTo(CLAW_CENTER_DEG, MOVE_TIME_MS);
        Claw_Report("center", Claw_CurrentDeg());
        break;

    case 'o':
        UART_SendString("open\r\n");
        Claw_MoveTo(CLAW_OPEN_DEG, MOVE_TIME_MS);
        Claw_Report("open", Claw_CurrentDeg());
        break;

    case 'c':
        UART_SendString("close\r\n");
        Claw_MoveTo(CLAW_CLOSE_DEG, MOVE_TIME_MS);
        Claw_Report("close", Claw_CurrentDeg());
        break;

    case '+':
    case '=':
        cur = Claw_CurrentDeg() + CLAW_STEP_DEG;
        if (cur > CLAW_MAX_DEG) { cur = CLAW_MAX_DEG; UART_SendString("at MAX\r\n"); }
        else                    { UART_SendString("step up\r\n"); }
        Claw_MoveTo(cur, MOVE_TIME_MS);
        Claw_Report("nudge+", Claw_CurrentDeg());
        break;

    case '-':
    case '_':
        cur = Claw_CurrentDeg() - CLAW_STEP_DEG;
        if (cur < CLAW_MIN_DEG) { cur = CLAW_MIN_DEG; UART_SendString("at MIN\r\n"); }
        else                    { UART_SendString("step down\r\n"); }
        Claw_MoveTo(cur, MOVE_TIME_MS);
        Claw_Report("nudge-", Claw_CurrentDeg());
        break;

    case 'a':
        g_input_mode = 1;
        g_input_val  = 0u;
        UART_SendString("angle input: type digits then '#'\r\n[angle] ");
        break;

    case 't':
    {
        uint8_t k;

        UART_SendString("auto sweep x");
        UART_SendInt(SWEEP_TIMES);
        UART_SendString("\r\n");
        for (k = 0u; k < SWEEP_TIMES; k++)
        {
            Claw_MoveTo(CLAW_OPEN_DEG,  SWEEP_TIME_MS);
            Claw_Report("sweep open",  Claw_CurrentDeg());
            Claw_MoveTo(CLAW_CLOSE_DEG, SWEEP_TIME_MS);
            Claw_Report("sweep close", Claw_CurrentDeg());
        }
        UART_SendString("[sweep] done\r\n");
        break;
    }

    case 's':
        Claw_Release();
        UART_SendString("released (no pulses, claw soft)\r\n");
        break;

    case 'p':
        UART_SendString("params\r\n");
        ShowParams();
        break;

    default:
        UART_SendString("unknown key, press h\r\n");
        break;
    }
}


/* ==========================================================================
 *  ⑥ main
 * ========================================================================== */

int main(void)
{
    delay_init(168);        /* 软件延时（主频 168MHz） */
    Timer6_Init();          /* delay_ms1() 用的是 TIM6，必须先初始化 */
    MyUART_Init();          /* 串口：PA9/PA10 */

    PWM_Init();             /* TIM2 四路 PWM，50Hz；里面会把 CCR2/3/4 预置成 520/648/838 */

    /* 一上电先把三路都关掉：CCR = 0 = 不发脉冲 = 舵机松力。
     * 这样上电瞬间爪子不会突然跑到参考工程那个 2.7 度的初始角上去。 */
#if (CLAW_CH != 2)
    PWM_SetCompare2(0u);
#endif
#if (CLAW_CH != 3)
    PWM_SetCompare3(0u);
#endif
#if (CLAW_CH != 4)
    PWM_SetCompare4(0u);
#endif
    Claw_Release();

    /* 注意：这里故意不调用 Servo_Init()（它会再 PWM_Init 一遍）；
     * 也不给 yuantai / zhuashou / wukuaipingtai 设目标角 ——
     * 它们的 moving_flag 一直是 0，所以 TIM2 中断只空转，不会动别的舵机。 */

    Claw_SetDegState(CLAW_CENTER_DEG);

    UART_SendString("\r\n=== STM32 CLAW test (servo only) ===\r\n");
    UART_SendString("signal: PA");
    UART_SendInt((uint32_t)(CLAW_CH - 1));
    UART_SendString("  (TIM2_CH");
    UART_SendInt((uint32_t)CLAW_CH);
    UART_SendString(")\r\n");
    UART_SendString("servo is RELEASED now (soft). press 'b' to hold center.\r\n");
    ShowHelp();

    while (1)
    {
        if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)
        {
            char c = (char)(USART_ReceiveData(USART1) & 0xFF);
            HandleKey(c);
        }
    }
}
