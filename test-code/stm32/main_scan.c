/* ============================================================================
 *  STM32 扫码模块测试版（独立程序，不动机械臂 / 舵机 / CAN）
 *
 *  【接线】（依据集成板原理图，见 进度记录.md）
 *    扫码模块  TX  -> 核心板 PB7（STM32 的 USART1_RX）
 *    扫码模块  RX  -> 核心板 PB6（STM32 的 USART1_TX）
 *    扫码模块 GND  -> 核心板 GND（必须共地）
 *    扫码模块 VCC  -> 按模块要求（一般 3.3V 或 5V，先看模块丝印）
 *    触发脚：PC0，平时高电平，拉低约 80ms 触发一次
 *
 *    ★ 调试打印仍走 USART3：PB10 = TX、PB11 = RX，115200，接传感器预留 U4
 *      （U4: 1=5V1, 2=PB11=MCU RX, 3=PB10=MCU TX, 4=GND）
 *    ★ 板子右边那个 USB UART 口在这块集成板上不是 USART1，别插那里看打印。
 *
 *  【这个程序干什么】
 *    只做一件事：把扫码模块吐出来的东西**原样**打出来。
 *    不解析任务码、不猜格式 —— 先看清模块到底发 ASCII 还是二进制、
 *    有没有前缀后缀、波特率是多少，再写正式解析。
 *
 *  【按键表】（在 USART3 那个串口里按）
 *    h        打印菜单
 *    r        触发一次扫码并接收（当前波特率）
 *    b        循环切换波特率（9600 → 115200 → 57600 → 38400 → 19200 → 4800 → 230400）
 *    1 ~ 7    直接选第 N 个波特率
 *    q        自动扫一遍所有波特率（每个只等 600ms，用来快速找对波特率）
 *    p        再打印一次上一次收到的内容
 *    t        开/关连续触发（每 2 秒一次），方便对着码反复扫
 *
 *  【打印格式】
 *    [scan] trigger, baud=9600
 *    [scan] raw="134+123+314+231" len=15
 *    [scan] hex=31 33 34 2B ...
 *    [scan] timeout, nothing received
 *    非可打印字符在 raw 里显示成 '.'，hex 行能看到真实字节。
 *
 *  【怎么算通过】
 *    按 r 后能稳定打印出二维码里的字符串，长度固定、内容可重复；
 *    再把 baud / raw / hex 三行原样发出来，就能确定模块参数和任务码格式。
 * ========================================================================== */

#include "stm32f4xx.h"
#include "delay.h"


/* ==========================================================================
 *  ① 调试串口 USART3（PB10 = TX，PB11 = RX），115200
 * ========================================================================== */
#define DBG_BAUD            115200

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
    while (*s) { Dbg_PutC((uint8_t)*s); s++; }
}

static void Dbg_Int(int32_t n)
{
    char     buf[12];
    uint8_t  i = 0;
    uint32_t u;

    if (n < 0) { Dbg_PutC('-'); u = (uint32_t)(-n); }
    else       { u = (uint32_t)n; }

    if (u == 0) { Dbg_PutC('0'); return; }
    while (u > 0) { buf[i++] = (char)('0' + (u % 10)); u /= 10; }
    while (i > 0) { Dbg_PutC((uint8_t)buf[--i]); }
}

static uint8_t Dbg_GetC(void)
{
    if (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET)
        return (uint8_t)USART_ReceiveData(USART3);
    return 0;
}


/* ==========================================================================
 *  ② 毫秒节拍：TIM7 自由跑 1ms 中断
 *     （delay_ms1 是阻塞的，不能拿来做接收超时判断）
 * ========================================================================== */
static volatile uint32_t g_ms = 0;

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

    /* 84MHz 定时器时钟 / 8400 = 10kHz，再 /10 = 1kHz → 1ms */
    tb.TIM_Prescaler         = 8400 - 1;
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
 *  ③ 扫码串口 USART1（PB6 = TX，PB7 = RX） + PC0 触发
 * ========================================================================== */
static uint32_t g_scan_baud = 9600;

static const uint32_t kBauds[] =
{
    9600, 115200, 57600, 38400, 19200, 4800, 230400
};
#define BAUD_N  (sizeof(kBauds) / sizeof(kBauds[0]))

static void Scan_ApplyBaud(uint32_t baud)
{
    USART_InitTypeDef usart;

    g_scan_baud = baud;

    USART_Cmd(USART1, DISABLE);
    usart.USART_BaudRate            = baud;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &usart);
    USART_Cmd(USART1, ENABLE);

    /* 清掉换波特率过程中可能留下的错位数据 */
    (void)USART1->SR;
    (void)USART1->DR;
}

static void Scan_Init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

    /* PB6/PB7 复用成 USART1（F4 用 AF 编号，不是 F1 的重映射） */
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    /* 触发脚 PC0：推挽输出，平时高，触发时拉低 */
    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_Pin   = GPIO_Pin_0;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &gpio);
    GPIO_SetBits(GPIOC, GPIO_Pin_0);

    /* P1 的 6/7 脚（PC4/PC5）先当上拉输入，用来判断模块是不是插偏了一格 */
    gpio.GPIO_Mode  = GPIO_Mode_IN;
    gpio.GPIO_Pin   = GPIO_Pin_4 | GPIO_Pin_5;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpio);

    USART_Cmd(USART1, DISABLE);
    usart.USART_BaudRate            = g_scan_baud;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &usart);
    USART_Cmd(USART1, ENABLE);
}

/* 触发参数：现场可改。默认低电平有效、80ms 脉冲 */
static uint8_t  g_trig_low  = 1;      /* 1 = 低电平触发；0 = 高电平触发 */
static uint32_t g_trig_ms   = 80;

static void Scan_TrigAssert(void)
{
    if (g_trig_low) GPIO_ResetBits(GPIOC, GPIO_Pin_0);
    else            GPIO_SetBits(GPIOC, GPIO_Pin_0);
}

static void Scan_TrigRelease(void)
{
    if (g_trig_low) GPIO_SetBits(GPIOC, GPIO_Pin_0);
    else            GPIO_ResetBits(GPIOC, GPIO_Pin_0);
}

static void Scan_Trigger(void)
{
    Scan_TrigAssert();
    delay_ms1(g_trig_ms);
    Scan_TrigRelease();
}


/* ==========================================================================
 *  ④ 接收一帧
 *    结束条件：收到 '\n'，或收到首字节后空闲 100ms；最多等 2000ms。
 *    缓冲区保留原始字节，不裁剪、不解析。
 * ========================================================================== */
#define SCAN_BUF_SIZE   128

typedef struct
{
    uint8_t  buf[SCAN_BUF_SIZE];
    uint16_t len;        /* 实际存进 buf 的字节数（最多 127） */
    uint32_t total;      /* 模块实际发来的字节数（可能超过 127） */
    uint8_t  overflow;   /* 1 = 超过缓冲区，后面的丢掉了 */
    uint32_t start_ms;
    uint32_t end_ms;
    uint8_t  valid;      /* 1 = 至少收到过 1 个字节 */
} ScanFrame;

static ScanFrame g_last;

/* 收 1 个字节进 frame；收到返回 1，没有返回 0 */
static uint8_t Scan_PollByte(ScanFrame *f)
{
    uint8_t b;

    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != SET) return 0;

    b = (uint8_t)USART_ReceiveData(USART1);
    if (f->len < (SCAN_BUF_SIZE - 1))
        f->buf[f->len++] = b;
    else
        f->overflow = 1;

    f->total++;
    f->valid = 1;
    return 1;
}

static void Scan_Collect(ScanFrame *f, uint32_t idle_ms, uint32_t timeout_ms)
{
    uint32_t t0, last;

    f->len      = 0;
    f->total    = 0;
    f->overflow = 0;
    f->valid    = 0;
    f->start_ms = Now();
    f->end_ms   = f->start_ms;
    last        = f->start_ms;
    t0          = f->start_ms;

    while ((Now() - t0) < timeout_ms)
    {
        if (Scan_PollByte(f))
        {
            last = Now();

            if (f->buf[f->len - 1] == '\n') break;   /* 一行结束 */
        }
        else if (f->valid && (Now() - last) >= idle_ms)
        {
            break;                      /* 发完就没动静了 */
        }
    }

    f->end_ms = Now();
}

/* 不管有没有数据，死等满 ms（抓"应答帧之后才来"的解码结果用） */
static void Scan_CollectFull(ScanFrame *f, uint32_t ms)
{
    uint32_t t0;

    f->len      = 0;
    f->total    = 0;
    f->overflow = 0;
    f->valid    = 0;
    f->start_ms = Now();

    t0 = f->start_ms;
    while ((Now() - t0) < ms)
        Scan_PollByte(f);

    f->end_ms = Now();
}

/* 长按触发：先保持有效电平 hold_ms（边保持边收），松开后再收 after_ms */
static void Scan_HoldTrigger(uint32_t hold_ms, uint32_t after_ms)
{
    ScanFrame *f = &g_last;
    uint32_t   t0, tr, last;
    uint8_t    done = 0;

    f->len      = 0;
    f->total    = 0;
    f->overflow = 0;
    f->valid    = 0;
    f->start_ms = Now();
    last        = f->start_ms;
    t0          = f->start_ms;

    Scan_TrigAssert();
    while ((Now() - t0) < hold_ms)
    {
        if (Scan_PollByte(f)) { last = Now(); done = 1; }
    }
    Scan_TrigRelease();

    tr = Now();
    while ((Now() - tr) < after_ms)
    {
        if (Scan_PollByte(f)) { last = Now(); done = 1; }
        else if (done && (Now() - last) >= 100) break;
    }

    f->end_ms = Now();
}

/* 打印时去掉尾部 CR/LF，其他字节原样（不可打印显示 '.'） */
static void Scan_PrintFrame(const ScanFrame *f)
{
    uint16_t i;
    uint16_t n = f->len;

    if (!f->valid)
    {
        Dbg_Str("[scan] timeout, nothing received\r\n");
        return;
    }

    while (n > 0 && (f->buf[n - 1] == '\r' || f->buf[n - 1] == '\n')) n--;

    Dbg_Str("[scan] raw=\"");
    for (i = 0; i < n; i++)
    {
        uint8_t b = f->buf[i];
        Dbg_PutC((b >= 0x20 && b <= 0x7E) ? b : '.');
    }
    Dbg_Str("\"  len=");
    Dbg_Int((int32_t)f->total);
    if (f->overflow) Dbg_Str(" (truncated)");
    Dbg_Str("  time=");
    Dbg_Int((int32_t)(f->end_ms - f->start_ms));
    Dbg_Str("ms\r\n");

    Dbg_Str("[scan] hex=");
    for (i = 0; i < f->len; i++)
    {
        static const char hex[] = "0123456789ABCDEF";
        Dbg_PutC((uint8_t)hex[(f->buf[i] >> 4) & 0x0F]);
        Dbg_PutC((uint8_t)hex[f->buf[i] & 0x0F]);
        Dbg_PutC(' ');
    }
    if (f->overflow) Dbg_Str("...");
    Dbg_Str("\r\n");
}

static void Scan_Once(uint32_t idle_ms, uint32_t timeout_ms)
{
    Dbg_Str("[scan] trigger ");
    Dbg_Str(g_trig_low ? "low " : "high ");
    Dbg_Int((int32_t)g_trig_ms);
    Dbg_Str("ms, baud=");
    Dbg_Int((int32_t)g_scan_baud);
    Dbg_Str("\r\n");

    Scan_Trigger();
    Scan_Collect(&g_last, idle_ms, timeout_ms);
    Scan_PrintFrame(&g_last);
}


/* ---------- 诊断工具 ---------- */
static void Dbg_Hex32(uint32_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    int8_t i;

    Dbg_Str("0x");
    for (i = 28; i >= 0; i -= 4) Dbg_PutC((uint8_t)hex[(v >> i) & 0x0F]);
}

/* 看引脚和 USART1 状态：PC0 输出电平、PB7 实际电平、波特率、触发参数 */
static void Scan_PinReport(void)
{
    uint32_t sr = USART1->SR;

    Dbg_Str("[pin] PC0(out)=");
    Dbg_Int(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_0) ? 1 : 0);
    Dbg_Str("  PB7(RX)=");
    Dbg_Int(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) ? 1 : 0);
    Dbg_Str("  PB6=");
    Dbg_Int(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) ? 1 : 0);
    Dbg_Str("  PC4=");
    Dbg_Int(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4) ? 1 : 0);
    Dbg_Str("  PC5=");
    Dbg_Int(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5) ? 1 : 0);
    Dbg_Str("  baud=");
    Dbg_Int((int32_t)g_scan_baud);
    Dbg_Str("  trig=");
    Dbg_Str(g_trig_low ? "low/" : "high/");
    Dbg_Int((int32_t)g_trig_ms);
    Dbg_Str("ms\r\n");

    Dbg_Str("[uart1] SR=");
    Dbg_Hex32(sr);
    Dbg_Str("  ORE=");
    Dbg_Int((sr & USART_FLAG_ORE) ? 1 : 0);
    Dbg_Str(" FE=");
    Dbg_Int((sr & USART_FLAG_FE) ? 1 : 0);
    Dbg_Str(" NE=");
    Dbg_Int((sr & USART_FLAG_NE) ? 1 : 0);
    Dbg_Str(" PE=");
    Dbg_Int((sr & USART_FLAG_PE) ? 1 : 0);
    Dbg_Str(" RXNE=");
    Dbg_Int((sr & USART_FLAG_RXNE) ? 1 : 0);
    Dbg_Str("\r\n");
}

/* 只听不发：模块如果自己在连续扫描，这里应该能收到东西 */
static void Scan_Listen(uint32_t ms)
{
    Dbg_Str("[scan] listening only ");
    Dbg_Int((int32_t)ms);
    Dbg_Str("ms (no trigger)...\r\n");
    Scan_Collect(&g_last, 150, ms);
    Scan_PrintFrame(&g_last);
}

/*
 * 看 PB7 / PB6 上有没有电平跳变。
 * 这个判断和波特率无关：只要模块真的在往外发字节，TX 线上就一定有边沿。
 *   mode 0 = 不触发，只听
 *   mode 1 = 先打一个触发脉冲，再听
 *   mode 2 = 触发脚一直保持有效电平，边保持边听
 * 同时统计 PB6（MCU 的 TX）：如果 PB6 有边沿而 PB7 没有，
 * 说明模块的 TX/RX 接反了（数据打到了 MCU 的发送脚上）。
 */
static void Scan_WatchEdges(uint8_t mode, uint32_t ms)
{
    ScanFrame *f = &g_last;
    uint32_t   t0;
    uint32_t   n7 = 0, n6 = 0;
    uint8_t    last7, last6, cur7, cur6;

    f->len      = 0;
    f->total    = 0;
    f->overflow = 0;
    f->valid    = 0;
    f->start_ms = Now();
    f->end_ms   = f->start_ms;

    if (mode == 1)
    {
        Scan_TrigAssert();
        delay_ms1(g_trig_ms);
        Scan_TrigRelease();
    }
    else if (mode == 2)
    {
        Scan_TrigAssert();
    }

    last7 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) ? 1 : 0;
    last6 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) ? 1 : 0;
    t0    = Now();

    while ((Now() - t0) < ms)
    {
        cur7 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) ? 1 : 0;
        cur6 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) ? 1 : 0;
        if (cur7 != last7) { n7++; last7 = cur7; }
        if (cur6 != last6) { n6++; last6 = cur6; }
        Scan_PollByte(f);
    }

    if (mode == 2) Scan_TrigRelease();
    f->end_ms = Now();

    /* 清掉观察期间可能积下的 ORE/RXNE，免得影响下一次接收 */
    (void)USART1->SR;
    (void)USART1->DR;

    Dbg_Str("[watch] mode=");
    Dbg_Str(mode == 0 ? "listen" : (mode == 1 ? "pulse" : "hold"));
    Dbg_Str(" ");
    Dbg_Int((int32_t)ms);
    Dbg_Str("ms  edges PB7=");
    Dbg_Int((int32_t)n7);
    Dbg_Str(" PB6=");
    Dbg_Int((int32_t)n6);
    Dbg_Str("  now PB7=");
    Dbg_Int(last7);
    Dbg_Str(" PB6=");
    Dbg_Int(last6);
    Dbg_Str("\r\n");

    Scan_PrintFrame(f);
}

/*
 * 一次盯住 P1 的 4 根信号线：PB6(3脚) PB7(4脚) PC4(6脚) PC5(7脚)。
 * 目的：模块如果整体插偏了一个脚位（比如第1脚插到了第2脚），
 * 它的 TX 就可能落在 PB7 以外的脚上。这里哪根线有边沿一目了然。
 */
static void Scan_WatchP1(uint32_t ms)
{
    uint32_t t0, e6 = 0, e7 = 0, e4 = 0, e5 = 0;
    uint16_t b0, c0, b1, c1;

    b0 = GPIOB->IDR;
    c0 = GPIOC->IDR;
    t0 = Now();

    while ((Now() - t0) < ms)
    {
        b1 = GPIOB->IDR;
        c1 = GPIOC->IDR;

        if ((b1 ^ b0) & GPIO_Pin_6) e6++;
        if ((b1 ^ b0) & GPIO_Pin_7) e7++;
        if ((c1 ^ c0) & GPIO_Pin_4) e4++;
        if ((c1 ^ c0) & GPIO_Pin_5) e5++;
        b0 = b1;
        c0 = c1;
    }

    Dbg_Str("[p1] ");
    Dbg_Int((int32_t)ms);
    Dbg_Str("ms edges PB6=");
    Dbg_Int((int32_t)e6);
    Dbg_Str(" PB7=");
    Dbg_Int((int32_t)e7);
    Dbg_Str(" PC4=");
    Dbg_Int((int32_t)e4);
    Dbg_Str(" PC5=");
    Dbg_Int((int32_t)e5);
    Dbg_Str("  levels PB6=");
    Dbg_Int((GPIOB->IDR & GPIO_Pin_6) ? 1 : 0);
    Dbg_Str(" PB7=");
    Dbg_Int((GPIOB->IDR & GPIO_Pin_7) ? 1 : 0);
    Dbg_Str(" PC4=");
    Dbg_Int((GPIOC->IDR & GPIO_Pin_4) ? 1 : 0);
    Dbg_Str(" PC5=");
    Dbg_Int((GPIOC->IDR & GPIO_Pin_5) ? 1 : 0);
    Dbg_Str("\r\n");
}

/* 自环测试：需要把 PB6 和 PB7 用一根杜邦线短接 */
static void Scan_Loopback(void)
{
    const char *msg = "SCAN-LOOP-1234\r\n";
    const char *p   = msg;

    Dbg_Str("[loop] short PB6<->PB7, sending...\r\n");
    while (*p)
    {
        USART_SendData(USART1, (uint8_t)*p);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        p++;
    }
    Scan_Collect(&g_last, 100, 500);
    Scan_PrintFrame(&g_last);
}

/* 手动把 PC0 翻一次，方便万用表量电平 */
static void Scan_TogglePC0(void)
{
    if (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_0))
        GPIO_ResetBits(GPIOC, GPIO_Pin_0);
    else
        GPIO_SetBits(GPIOC, GPIO_Pin_0);

    Dbg_Str("[pin] PC0 -> ");
    Dbg_Int(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_0) ? 1 : 0);
    Dbg_Str("  (measure it with a meter)\r\n");
}

/* 常见扫码引擎的"开始扫描"指令。有些模块没有触发脚，只能用串口命令触发。 */
typedef struct
{
    uint8_t len;
    uint8_t b[9];
} TrigCmd;

static const TrigCmd kTrigCmds[] =
{
    { 9, { 0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD } }, /* 新大陆/兼容引擎: Start */
    { 3, { 0x1B, 0x31, 0x0D } },                                     /* ESC '1' */
    { 2, { 0x7E, 0x00 } },                                           /* 帧头探测 */
    { 1, { 0x04 } },                                                 /* EOT */
    { 1, { 0x12 } },                                                 /* DC2 */
    { 1, { 0x54 } },                                                 /* 'T' */
    { 2, { 0x31, 0x0D } },                                           /* "1\r" */
    { 1, { 0x0D } }                                                  /* CR */
};

static void Scan_SendBytes(const uint8_t *p, uint8_t n)
{
    while (n--)
    {
        USART_SendData(USART1, *p++);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
}

/*
 * PB6 方波：把 MCU 的 TX 脚（P1 第 3 脚）当普通 GPIO 输出，1Hz 翻转 10 秒。
 * 用万用表量模块的 RX 脚：应该看到 0 / 3.3V 交替跳。
 *   - 会跳：MCU 到模块 RX 这根线是通的
 *   - 一点不动：这根线断了 / 插错脚 / 没接
 * 测完恢复 USART1。
 */
static void Scan_TxSquare(uint32_t ms)
{
    GPIO_InitTypeDef gpio;
    uint32_t t0;
    uint8_t  lvl = 1;

    USART_Cmd(USART1, DISABLE);

    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_Pin   = GPIO_Pin_6;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &gpio);

    Dbg_Str("[tx] PB6 square wave 1Hz for ");
    Dbg_Int((int32_t)ms);
    Dbg_Str("ms - meter between module RX and GND\r\n");

    t0 = Now();
    while ((Now() - t0) < ms)
    {
        if (lvl) GPIO_SetBits(GPIOB, GPIO_Pin_6);
        else     GPIO_ResetBits(GPIOB, GPIO_Pin_6);
        lvl = !lvl;
        delay_ms1(500);
    }

    /* 恢复 USART1 的 TX/RX */
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
    USART_Cmd(USART1, ENABLE);

    Dbg_Str("[tx] done, USART1 restored\r\n");
}

/*
 * 串口命令扫触发：每个波特率把上面几种"开始扫描"指令轮流发一遍，
 * 每条发完听 250ms，统计模块回了多少字节。
 * 用途：模块没有触发脚（或触发脚没接上）时，看它认不认串口命令。
 *   -> 某个波特率下 bytes > 0：找到能唤醒它的指令/波特率
 *   -> 全是 0：模块不接受这些指令，或者根本没识别到码
 */
static void Scan_CommandSweep(void)
{
    uint8_t  bi, ci;
    uint32_t old = g_scan_baud;

    Dbg_Str("[cmd] sweeping trigger commands (aim at a code!)...\r\n");
    for (bi = 0; bi < BAUD_N; bi++)
    {
        uint32_t got = 0;

        Scan_ApplyBaud(kBauds[bi]);
        Dbg_Str("  baud ");
        Dbg_Int((int32_t)g_scan_baud);
        Dbg_Str(": ");

        for (ci = 0; ci < (uint8_t)(sizeof(kTrigCmds) / sizeof(kTrigCmds[0])); ci++)
        {
            uint32_t t0;

            Scan_SendBytes(kTrigCmds[ci].b, kTrigCmds[ci].len);

            t0 = Now();
            while ((Now() - t0) < 250)
            {
                if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)
                {
                    (void)USART_ReceiveData(USART1);
                    got++;
                }
            }
        }

        Dbg_Int((int32_t)got);
        Dbg_Str(" bytes\r\n");
    }

    Scan_ApplyBaud(old);
    Dbg_Str("[cmd] restored baud=");
    Dbg_Int((int32_t)g_scan_baud);
    Dbg_Str("\r\n");
}

/*
 * 单条指令探测：在当前波特率下，把每条候选指令单独发一次，
 * 每条后面听 800ms，把回了什么原样打出来（hex + 可打印字符）。
 * 用途：`c` 扫到某个波特率有字节后，用这个确定到底是哪条指令
 * 唤醒了模块、它回的是什么（应答帧 / 任务码 / 错误码）。
 */
static void Scan_CmdProbe(void)
{
    uint8_t ci, n = (uint8_t)(sizeof(kTrigCmds) / sizeof(kTrigCmds[0]));

    Dbg_Str("[probe] one command at a time @");
    Dbg_Int((int32_t)g_scan_baud);
    Dbg_Str(" (aim at a code)\r\n");

    for (ci = 0; ci < n; ci++)
    {
        uint16_t i;

        /* 丢掉之前残留的字节 */
        while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
            (void)USART_ReceiveData(USART1);

        Dbg_Str("[probe] #");
        Dbg_Int((int32_t)(ci + 1));
        Dbg_Str(" send=");
        for (i = 0; i < kTrigCmds[ci].len; i++)
        {
            static const char hex[] = "0123456789ABCDEF";
            Dbg_PutC((uint8_t)hex[(kTrigCmds[ci].b[i] >> 4) & 0x0F]);
            Dbg_PutC((uint8_t)hex[kTrigCmds[ci].b[i] & 0x0F]);
            Dbg_PutC(' ');
        }
        Dbg_Str("\r\n");

        Scan_SendBytes(kTrigCmds[ci].b, kTrigCmds[ci].len);
        Scan_CollectFull(&g_last, 2000);
        Scan_PrintFrame(&g_last);
    }

    Dbg_Str("[probe] done\r\n");
}

/*
 * 发"开始扫描"指令后长时间监听。
 * `n` 每条只听 800ms，引擎往往要 1~3 秒才解出码，数据会落在窗口外。
 * 这个按键发完指令听 8 秒，专门抓解码结果。
 */
static void Scan_StartAndListen(uint32_t ms)
{
    static const uint8_t start_cmd[9] =
        { 0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD };
    ScanFrame *f = &g_last;
    uint32_t   t0;

    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
        (void)USART_ReceiveData(USART1);

    Dbg_Str("[start] send 7E 00 08 01 00 02 01 AB CD, listen ");
    Dbg_Int((int32_t)ms);
    Dbg_Str("ms (aim at a code)\r\n");

    f->len      = 0;
    f->total    = 0;
    f->overflow = 0;
    f->valid    = 0;
    f->start_ms = Now();

    Scan_SendBytes(start_cmd, sizeof(start_cmd));

    /* 这里必须听满整个窗口：应答帧之后模块要 1~3 秒才解出码，
       Scan_Collect 的"空闲 300ms 就结束"会提前收工。 */
    t0 = Now();
    while ((Now() - t0) < ms)
        Scan_PollByte(f);

    f->end_ms = Now();
    Scan_PrintFrame(&g_last);
}

/* 只认那帧 7 字节指令应答，连续模式下不打它，免得刷屏 */
static uint8_t Scan_IsAck(const ScanFrame *f)
{
    return (f->len == 7 &&
            f->buf[0] == 0x02 && f->buf[1] == 0x00 && f->buf[2] == 0x00 &&
            f->buf[3] == 0x01 && f->buf[4] == 0x00);
}

/*
 * 连续扫 ms 毫秒：每 1.5 秒补发一次"开始解码"，中间一直收。
 * 用途：拿码从 2cm 慢慢拉到 20cm，看模块在哪个距离才出数据。
 * 这类 OEM 引擎都是定焦的，离太近或太远都会"灯亮着但解不出来"。
 * 命中时会打印 [cont] hit at t=xxxxms + raw/hex。
 */
static void Scan_Continuous(uint32_t ms)
{
    static const uint8_t start_cmd[9] =
        { 0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD };
    ScanFrame *f = &g_last;
    uint32_t   t0, next_cmd, last;

    Dbg_Str("[cont] ");
    Dbg_Int((int32_t)(ms / 1000));
    Dbg_Str("s running, move the code slowly 2cm -> 20cm\r\n");

    f->len      = 0;
    f->total    = 0;
    f->overflow = 0;
    f->valid    = 0;
    f->start_ms = Now();
    t0          = f->start_ms;
    last        = t0;
    next_cmd    = t0 - 1;          /* 立刻发第一条 */

    while ((Now() - t0) < ms)
    {
        if ((int32_t)(Now() - next_cmd) >= 0)
        {
            Scan_SendBytes(start_cmd, sizeof(start_cmd));
            next_cmd = Now() + 1500;
        }

        if (Scan_PollByte(f)) { last = Now(); continue; }

        /* 一帧收完（停顿 60ms）就判断：是应答就丢掉，是数据就打出来 */
        if (f->valid && (Now() - last) >= 60)
        {
            f->end_ms = Now();
            if (!Scan_IsAck(f))
            {
                Dbg_Str("[cont] hit at t=");
                Dbg_Int((int32_t)(f->end_ms - t0));
                Dbg_Str("ms\r\n");
                Scan_PrintFrame(f);
            }
            f->len      = 0;
            f->total    = 0;
            f->overflow = 0;
            f->valid    = 0;
            f->start_ms = Now();
            last        = f->start_ms;
        }
    }

    f->end_ms = Now();
    Dbg_Str("[cont] done\r\n");
}

/*
 * XM1601 真实协议（见《XM1601 用户产品手册 v2.0》）：
 *   读标志位: 7E 00 | 07 | 01 | addr_hi addr_lo | count | AB CD
 *   写标志位: 7E 00 | 08 | 01 | addr_hi addr_lo | data   | AB CD
 *   存 Flash : 7E 00 | 09 | 01 | 00 00        | 00     | DE C8
 *   应答    : 02 00 | 00 | len | data... | crc
 * CRC 写 AB CD 表示免校验。
 *
 * 关键：7E 00 08 01 00 02 01 AB CD 只是把 0x0002 bit0（命令触发标志）写 1，
 * **只有在 0x0000 的低两位 = 01（命令触发模式）时才真的开始扫**。
 * 出厂默认 00 = 手动模式，必须靠 PIN12(TRIG) 拉低才成像。
 */
static void Scan_FlushRx(void)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
        (void)USART_ReceiveData(USART1);
}

/* 读 n 个标志位到 f，成功返回 1 */
static uint8_t Scan_ReadFlags(uint16_t addr, uint8_t n, ScanFrame *f)
{
    uint8_t cmd[9];

    cmd[0] = 0x7E; cmd[1] = 0x00;
    cmd[2] = 0x07; cmd[3] = 0x01;
    cmd[4] = (uint8_t)(addr >> 8);
    cmd[5] = (uint8_t)(addr & 0xFF);
    cmd[6] = n;
    cmd[7] = 0xAB; cmd[8] = 0xCD;

    Scan_FlushRx();
    Scan_SendBytes(cmd, sizeof(cmd));
    Scan_CollectFull(f, 800);

    return (uint8_t)(f->len >= 6 && f->buf[0] == 0x02 && f->buf[1] == 0x00 &&
                     f->buf[2] == 0x00);
}

static void Scan_PutHex8(uint8_t b)
{
    static const char hex[] = "0123456789ABCDEF";
    Dbg_PutC((uint8_t)hex[(b >> 4) & 0x0F]);
    Dbg_PutC((uint8_t)hex[b & 0x0F]);
}

/* 写一个标志位字节，成功返回 1（应答 02 00 00 ...） */
static uint8_t Scan_WriteFlag(uint16_t addr, uint8_t val, ScanFrame *f)
{
    uint8_t wr[9];

    wr[0] = 0x7E; wr[1] = 0x00;
    wr[2] = 0x08; wr[3] = 0x01;
    wr[4] = (uint8_t)(addr >> 8);
    wr[5] = (uint8_t)(addr & 0xFF);
    wr[6] = val;
    wr[7] = 0xAB; wr[8] = 0xCD;

    Scan_FlushRx();
    Scan_SendBytes(wr, sizeof(wr));
    Scan_CollectFull(f, 600);

    return (uint8_t)(f->len >= 5 && f->buf[0] == 0x02 && f->buf[2] == 0x00);
}

/* 7E 00 09 01 00 00 00 DE C8 —— 把当前设置存进模块 Flash */
static uint8_t Scan_SaveFlash(ScanFrame *f)
{
    static const uint8_t save_cmd[9] =
        { 0x7E, 0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0xDE, 0xC8 };

    Scan_FlushRx();
    Scan_SendBytes(save_cmd, sizeof(save_cmd));
    Scan_CollectFull(f, 600);

    return (uint8_t)(f->len >= 5 && f->buf[0] == 0x02 && f->buf[2] == 0x00);
}

/*
 * a：查"输出端口"和"QR 码识读开关"，不对就顺手改好并存 Flash。
 *
 * 手册标志位 0x000D：
 *   bit1-0 输出端口模式  00 串口输出 / 01 仅 USB-HID / 10 串口&HID / 11 USB 虚拟串口
 *   bit3-2 输出数据编码  00 GBK / 01 UNICODE / 10 原数据 / 11 UTF8
 *   bit6   虚拟键盘使能   bit7 开票模式
 * 手册标志位 0x003F：
 *   bit0 QR 码识读开关  0 禁止 / 1 允许
 *   bit5 QR 模式1 开关
 *
 * ★ 出厂默认是 USB-HID 输出（手册 2.1 节）。若 bit1-0 = 01，
 *   解码数据只往 USB 送，UART 只负责收配置命令 —— 表现就是
 *   "命令有应答、永远收不到码"，和我们现在的情况完全一致。
 */
static void Scan_CheckOutput(void)
{
    ScanFrame *f = &g_last;
    uint8_t    i, n, v0d = 0, v3f = 0, port, enc;
    uint8_t    have0d = 0, have3f = 0, fix = 0;

    Dbg_Str("[cfg] read 0x0000..0x000F\r\n");
    if (!Scan_ReadFlags(0x0000, 16, f))
    {
        Scan_PrintFrame(f);
        Dbg_Str("[cfg] read failed\r\n");
        return;
    }
    n = f->buf[3];
    if (n > (uint8_t)(f->len - 6)) n = (uint8_t)(f->len - 6);

    Dbg_Str("[cfg] flags=");
    for (i = 0; i < n; i++)
    {
        Scan_PutHex8(f->buf[4 + i]);
        Dbg_PutC(' ');
    }
    Dbg_Str("\r\n");

    if (n > 0x0D)
    {
        v0d   = f->buf[4 + 0x0D];
        have0d = 1;
    }

    if (have0d)
    {
        port = (uint8_t)(v0d & 0x03);
        enc  = (uint8_t)((v0d >> 2) & 0x03);

        Dbg_Str("[cfg] 0x000D  = ");
        Scan_PutHex8(v0d);
        Dbg_Str("  output port = ");
        switch (port)
        {
        case 0:  Dbg_Str("UART serial  (OK)\r\n");            break;
        case 1:  Dbg_Str("USB-HID ONLY <-- data never reaches UART!\r\n"); fix = 1; break;
        case 2:  Dbg_Str("UART + HID\r\n");                   break;
        default: Dbg_Str("USB virtual COM <-- no UART output!\r\n"); fix = 1; break;
        }

        Dbg_Str("[cfg]            data encoding = ");
        switch (enc)
        {
        case 0:  Dbg_Str("GBK\r\n");       break;
        case 1:  Dbg_Str("UNICODE\r\n");   break;
        case 2:  Dbg_Str("raw\r\n");       break;
        default: Dbg_Str("UTF8\r\n");      break;
        }
    }

    Dbg_Str("[cfg] read 0x003F (QR enable)\r\n");
    if (Scan_ReadFlags(0x003F, 1, f) && f->len >= 6)
    {
        v3f    = f->buf[4];
        have3f = 1;

        Dbg_Str("[cfg] 0x003F  = ");
        Scan_PutHex8(v3f);
        Dbg_Str("  QR decode = ");
        if (v3f & 0x01) Dbg_Str("ENABLED\r\n");
        else            { Dbg_Str("DISABLED <-- this alone explains no reads!\r\n"); fix = 1; }
    }
    else
    {
        Scan_PrintFrame(f);
        Dbg_Str("[cfg] 0x003F read failed\r\n");
    }

    if (!fix)
    {
        Dbg_Str("[cfg] both settings look correct, nothing changed\r\n");
        return;
    }

    /* ---- 修：输出强制改回串口，QR 打开 ---- */
    if (have0d)
    {
        uint8_t nv = (uint8_t)(v0d & 0xFC);   /* bit1-0 = 00 串口输出 */

        if (nv != v0d)
        {
            Dbg_Str("[cfg] write 0x000D: ");
            Scan_PutHex8(v0d);
            Dbg_Str(" -> ");
            Scan_PutHex8(nv);
            Dbg_Str(" ... ");
            if (Scan_WriteFlag(0x000D, nv, f)) Dbg_Str("ok\r\n");
            else { Dbg_Str("FAILED\r\n"); Scan_PrintFrame(f); return; }
        }
    }

    if (have3f && !(v3f & 0x01))
    {
        Dbg_Str("[cfg] write 0x003F: ");
        Scan_PutHex8(v3f);
        Dbg_Str(" -> ");
        Scan_PutHex8((uint8_t)(v3f | 0x01));
        Dbg_Str(" ... ");
        if (Scan_WriteFlag(0x003F, (uint8_t)(v3f | 0x01), f)) Dbg_Str("ok\r\n");
        else { Dbg_Str("FAILED\r\n"); Scan_PrintFrame(f); return; }
    }

    if (Scan_SaveFlash(f))
        Dbg_Str("[cfg] saved to flash -- now re-check with 'a' then test with 's'\r\n");
    else
    {
        Scan_PrintFrame(f);
        Dbg_Str("[cfg] save failed (will revert on power cycle)\r\n");
    }
}

/* d：把 0x0000~0x0007 读回来并翻译成人话 */
static void Scan_DumpFlags(void)
{
    ScanFrame *f = &g_last;
    uint8_t    i, n, v0;

    Dbg_Str("[cfg] read 0x0000..0x0007\r\n");
    if (!Scan_ReadFlags(0x0000, 8, f))
    {
        Scan_PrintFrame(f);
        Dbg_Str("[cfg] no valid answer\r\n");
        return;
    }

    n = (f->buf[3] == 0) ? 0 : f->buf[3];
    if (n > (uint8_t)(f->len - 6)) n = (uint8_t)(f->len - 6);

    Dbg_Str("[cfg] flags=");
    for (i = 0; i < n; i++)
    {
        Scan_PutHex8(f->buf[4 + i]);
        Dbg_PutC(' ');
    }
    Dbg_Str("\r\n");

    if (n < 1) return;
    v0 = f->buf[4];

    Dbg_Str("[cfg] mode      = ");
    switch (v0 & 0x03)
    {
    case 0: Dbg_Str("MANUAL (trigger pin, default)\r\n"); break;
    case 1: Dbg_Str("CMD-TRIGGER (serial cmd can start scan)\r\n"); break;
    case 2: Dbg_Str("CONTINUOUS (scans all the time)\r\n"); break;
    default: Dbg_Str("SENSOR (light-change wake)\r\n"); break;
    }

    Dbg_Str("[cfg] aimer     = ");
    switch ((v0 >> 4) & 0x03)
    {
    case 0: Dbg_Str("off\r\n"); break;
    case 1: Dbg_Str("on while imaging\r\n"); break;
    case 2: Dbg_Str("always on\r\n"); break;
    default: Dbg_Str("on while imaging (stay on)\r\n"); break;
    }

    Dbg_Str("[cfg] fill light= ");
    switch ((v0 >> 2) & 0x03)
    {
    case 0: Dbg_Str("off\r\n"); break;
    case 1: Dbg_Str("on while imaging\r\n"); break;
    default: Dbg_Str("always on\r\n"); break;
    }
}

/*
 * u：改成连续模式 + 定位灯/补光灯"拍照时点亮" + 存 Flash。
 * 连续模式下模块自己一直在扫，主机不用触发、不用发指令，
 * 码一进视野就把内容从 TX 吐出来 —— 比赛用这个最省事。
 */
static void Scan_SetContinuous(void)
{
    static const uint8_t save_cmd[9] =
        { 0x7E, 0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0xDE, 0xC8 };
    ScanFrame *f = &g_last;
    uint8_t    wr[9];
    uint8_t    v, old;

    Dbg_Str("[cfg] read 0x0000 ...\r\n");
    if (!Scan_ReadFlags(0x0000, 1, f) || f->len < 6)
    {
        Scan_PrintFrame(f);
        Dbg_Str("[cfg] read failed, nothing changed\r\n");
        return;
    }

    old = f->buf[4];
    v   = old;
    v   = (uint8_t)((v & 0xFC) | 0x02);   /* bit1-0 = 10 连续模式 */
    v   = (uint8_t)((v & 0xCF) | 0x10);   /* bit5-4 = 01 定位灯拍照时点亮 */
    v   = (uint8_t)((v & 0xF3) | 0x04);   /* bit3-2 = 01 补光灯拍照时点亮 */

    Dbg_Str("[cfg] 0x0000: ");
    Scan_PutHex8(old);
    Dbg_Str(" -> ");
    Scan_PutHex8(v);
    Dbg_Str("\r\n");

    wr[0] = 0x7E; wr[1] = 0x00;
    wr[2] = 0x08; wr[3] = 0x01;
    wr[4] = 0x00; wr[5] = 0x00;
    wr[6] = v;
    wr[7] = 0xAB; wr[8] = 0xCD;

    Scan_FlushRx();
    Scan_SendBytes(wr, sizeof(wr));
    Scan_CollectFull(f, 600);
    if (!(f->len >= 5 && f->buf[0] == 0x02 && f->buf[2] == 0x00))
    {
        Scan_PrintFrame(f);
        Dbg_Str("[cfg] write failed (RAM only, not saved)\r\n");
        return;
    }
    Dbg_Str("[cfg] write ok\r\n");

    Scan_FlushRx();
    Scan_SendBytes(save_cmd, sizeof(save_cmd));
    Scan_CollectFull(f, 600);
    if (f->len >= 5 && f->buf[0] == 0x02 && f->buf[2] == 0x00)
        Dbg_Str("[cfg] saved to flash -- module now scans continuously\r\n");
    else
    {
        Scan_PrintFrame(f);
        Dbg_Str("[cfg] save failed (will revert on power cycle)\r\n");
    }
}

/*
 * PB7 悬空测试（关键诊断）。
 *
 * 把 PB7 从 USART1 复用功能收回成"什么上下拉都不加"的普通输入：
 *   - 如果 P1 第 4 脚真的连着模块的 TX，而模块 TX 正在推挽输出空闲高电平，
 *     PB7 就会被模块硬驱动到 1，读到几乎 100% 的高电平；
 *   - 如果线没接通（插偏、线断、座子虚焊、模块 TX 是高阻），
 *     引脚就是悬空的，通常读 0 或乱跳。
 * 这一条和波特率、触发方式都无关，只判断"线到底通不通"。
 * 测完恢复 USART1 接收。
 */
static void Scan_FloatTest(uint32_t ms)
{
    GPIO_InitTypeDef gpio;
    uint32_t t0, n = 0, ones = 0, edges = 0;
    uint8_t  last, cur;

    USART_Cmd(USART1, DISABLE);

    gpio.GPIO_Mode  = GPIO_Mode_IN;
    gpio.GPIO_Pin   = GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &gpio);

    /* 给引脚一点时间稳定，再开始统计 */
    delay_ms1(5);
    last = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) ? 1 : 0;

    t0 = Now();
    while ((Now() - t0) < ms)
    {
        cur = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) ? 1 : 0;
        if (cur) ones++;
        if (cur != last) { edges++; last = cur; }
        n++;
        delay_ms1(1);
    }

    Dbg_Str("[float] PB7 no-pull ");
    Dbg_Int((int32_t)ms);
    Dbg_Str("ms  samples=");
    Dbg_Int((int32_t)n);
    Dbg_Str(" high=");
    Dbg_Int((int32_t)ones);
    Dbg_Str(" edges=");
    Dbg_Int((int32_t)edges);
    if (n && ones == n)
        Dbg_Str("  -> driven HIGH (wire looks connected)");
    else if (ones == 0)
        Dbg_Str("  -> floating LOW (pin 4 not driven by module)");
    else
        Dbg_Str("  -> unstable (bad contact / noise)");
    Dbg_Str("\r\n");

    /* 恢复 USART1 的 RX 脚 */
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
    USART_Cmd(USART1, ENABLE);
}


/* ==========================================================================
 *  ⑤ 菜单 / 按键
 * ========================================================================== */
static void Show_Menu(void)
{
    uint8_t i;

    Dbg_Str("\r\n== SCAN module test (USART1: PB6=TX PB7=RX, PC0 trigger) ==\r\n");
    Dbg_Str("   h : help\r\n");
    Dbg_Str("   r : trigger one scan\r\n");
    Dbg_Str("   e : hold trigger 1.5s (level-trigger modules)\r\n");
    Dbg_Str("   s : listen only 5s (auto/continuous mode)\r\n");
    Dbg_Str("   w : trigger + watch PB7 edges 5s (baud independent)\r\n");
    Dbg_Str("   x : watch PB7 edges 5s, no trigger\r\n");
    Dbg_Str("   m : hold trigger + watch PB7 edges 5s\r\n");
    Dbg_Str("   y : watch P1 pins PB6/PB7/PC4/PC5 10s\r\n");
    Dbg_Str("   z : PB7 floating test 3s (is pin4 driven by module?)\r\n");
    Dbg_Str("   c : sweep serial trigger commands on all bauds\r\n");
    Dbg_Str("   j : PB6 1Hz square 10s (meter check MCU->module RX wire)\r\n");
    Dbg_Str("   n : probe each trigger command once, print reply hex\r\n");
    Dbg_Str("   f : send start-scan cmd, listen 8s for decoded data\r\n");
    Dbg_Str("   l : hold TRIG low 6s + listen (manual/level-trigger mode)\r\n");
    Dbg_Str("   d : read flags 0x0000..0x0007 (show mode / lights)\r\n");
    Dbg_Str("   u : set CONTINUOUS mode + save to module flash\r\n");
    Dbg_Str("   a : check output port / QR enable, auto-fix + save if wrong\r\n");
    Dbg_Str("   g : continuous 30s (sweep code 2cm -> 20cm)\r\n");
    Dbg_Str("   i : invert trigger polarity\r\n");
    Dbg_Str("   + / - : trigger pulse +/-50ms\r\n");
    Dbg_Str("   o : manually toggle PC0\r\n");
    Dbg_Str("   v : show pin / UART status\r\n");
    Dbg_Str("   k : loopback test (short PB6-PB7)\r\n");
    Dbg_Str("   b : cycle baud\r\n");
    Dbg_Str("   1-7 : pick baud\r\n");
    Dbg_Str("   q : try all bauds quickly\r\n");
    Dbg_Str("   p : print last result\r\n");
    Dbg_Str("   t : auto trigger on/off (2s)\r\n");
    Dbg_Str("baud list:");
    for (i = 0; i < BAUD_N; i++)
    {
        Dbg_Str(" ");
        Dbg_Int((int32_t)(i + 1));
        Dbg_Str("=");
        Dbg_Int((int32_t)kBauds[i]);
    }
    Dbg_Str("\r\ncurrent baud = ");
    Dbg_Int((int32_t)g_scan_baud);
    Dbg_Str("\r\n\r\n");
}

static void Select_Baud(uint8_t idx)
{
    if (idx >= BAUD_N) return;
    Scan_ApplyBaud(kBauds[idx]);
    Dbg_Str("[scan] baud -> ");
    Dbg_Int((int32_t)g_scan_baud);
    Dbg_Str("\r\n");
}

static void Try_All_Bauds(void)
{
    uint8_t  i;
    uint32_t old = g_scan_baud;

    Dbg_Str("[scan] trying all bauds, 1200ms each...\r\n");
    for (i = 0; i < BAUD_N; i++)
    {
        Scan_ApplyBaud(kBauds[i]);
        Dbg_Str("---- baud ");
        Dbg_Int((int32_t)g_scan_baud);
        Dbg_Str(" ----\r\n");
        Scan_Trigger();
        Scan_Collect(&g_last, 100, 1200);
        Scan_PrintFrame(&g_last);
        delay_ms1(150);
    }
    Scan_ApplyBaud(old);
    Dbg_Str("[scan] restored baud=");
    Dbg_Int((int32_t)g_scan_baud);
    Dbg_Str("\r\n");
}

static uint8_t  g_auto_on   = 0;
static uint32_t g_last_auto = 0;

static void Handle_Key(char c)
{
    if (c >= '1' && c <= '9')
    {
        Select_Baud((uint8_t)(c - '1'));
        return;
    }

    switch (c)
    {
    case 'h': case 'H': Show_Menu();             break;
    case 'r': case 'R': Scan_Once(100, 2000);    break;
    case 'e': case 'E':
        Dbg_Str("[scan] hold trigger 1500ms...\r\n");
        Scan_HoldTrigger(1500, 1000);
        Scan_PrintFrame(&g_last);
        break;
    case 's': case 'S': Scan_Listen(5000);       break;
    case 'w': case 'W': Scan_WatchEdges(1, 5000); break;
    case 'x': case 'X': Scan_WatchEdges(0, 5000); break;
    case 'm': case 'M': Scan_WatchEdges(2, 5000); break;
    case 'y': case 'Y': Scan_WatchP1(10000);      break;
    case 'z': case 'Z': Scan_FloatTest(3000);     break;
    case 'c': case 'C': Scan_CommandSweep();      break;
    case 'j': case 'J': Scan_TxSquare(10000);     break;
    case 'n': case 'N': Scan_CmdProbe();          break;
    case 'f': case 'F': Scan_StartAndListen(8000); break;
    case 'l': case 'L':
        Dbg_Str("[scan] hold TRIG 6000ms + listen (level trigger)...\r\n");
        Scan_HoldTrigger(6000, 1500);
        Scan_PrintFrame(&g_last);
        break;
    case 'd': case 'D': Scan_DumpFlags();          break;
    case 'u': case 'U': Scan_SetContinuous();      break;
    case 'a': case 'A': Scan_CheckOutput();        break;
    case 'g': case 'G': Scan_Continuous(30000);    break;
    case 'i': case 'I':
        g_trig_low = !g_trig_low;
        Dbg_Str("[scan] trigger polarity -> ");
        Dbg_Str(g_trig_low ? "LOW active\r\n" : "HIGH active\r\n");
        break;
    case 'o': case 'O': Scan_TogglePC0();        break;
    case 'v': case 'V': Scan_PinReport();        break;
    case 'k': case 'K': Scan_Loopback();         break;
    case '+':
        if (g_trig_ms < 2000) g_trig_ms += 50;
        Dbg_Str("[scan] pulse -> ");
        Dbg_Int((int32_t)g_trig_ms);
        Dbg_Str("ms\r\n");
        break;
    case '-':
        if (g_trig_ms > 20) g_trig_ms -= 50;
        Dbg_Str("[scan] pulse -> ");
        Dbg_Int((int32_t)g_trig_ms);
        Dbg_Str("ms\r\n");
        break;
    case 'p': case 'P': Scan_PrintFrame(&g_last); break;
    case 'q': case 'Q': Try_All_Bauds();         break;
    case 'b': case 'B':
        {
            uint8_t i;
            for (i = 0; i < BAUD_N; i++)
                if (kBauds[i] == g_scan_baud) break;
            Select_Baud((uint8_t)((i + 1) % BAUD_N));
        }
        break;
    case 't': case 'T':
        g_auto_on = !g_auto_on;
        g_last_auto = Now();
        Dbg_Str(g_auto_on ? "[scan] auto ON (2s)\r\n" : "[scan] auto OFF\r\n");
        break;
    default:
        break;
    }
}


/* ==========================================================================
 *  ⑥ 主程序
 * ========================================================================== */
int main(void)
{
    delay_init(168);        /* 软件延时 */
    Timer6_Init();          /* delay_ms1 用 TIM6 */
    Tick_Init();            /* TIM7：1ms 节拍，用于接收超时判断 */
    Dbg_Init();             /* USART3：调试打印 + 按键 */
    Scan_Init();            /* USART1：扫码模块 + PC0 触发 */

    Dbg_Str("\r\n== SCAN test boot ==\r\n");
    Dbg_Str("wiring: scan TX->PB7, scan RX->PB6, GND common, trigger=PC0\r\n");
    Show_Menu();

    while (1)
    {
        uint8_t c = Dbg_GetC();

        if (c != 0) Handle_Key((char)c);

        if (g_auto_on && (Now() - g_last_auto) >= 2000)
        {
            g_last_auto = Now();
            Scan_Once(100, 2000);
        }
    }
}
