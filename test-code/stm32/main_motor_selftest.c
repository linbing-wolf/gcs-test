/* ============================================================================
 *  STM32 驱动 X42S 电机（CAN）—— 精细自检版
 *  ---------------------------------------------------------------------------
 *  和上一版（main_motor.c）的区别，都是为了让"电机不动"这件事有据可查：
 *
 *   1) 上电先做 CAN 回环自测（不需要电机、不需要外部接线）：
 *      单片机自己发一帧、自己收一帧。
 *        收不到  ->  问题在单片机这边（CAN 初始化 / 时钟），跟电机、接线无关
 *        收得到  ->  CAN 外设和帧格式没问题，嫌疑范围立刻缩到"线"和"电机"上
 *
 *   2) 每条指令都把"真正要发到总线上的原始帧"打印出来
 *      （扩展帧 ID / DLC / 每一个字节），可以直接和手算结果、抓包对照。
 *
 *   3) 发完一帧后检查发送错误计数 TEC：
 *        TEC 涨了 -> 总线上没有第二个设备回 ACK
 *                    （对方没上电 / CAN_H、CAN_L 断了或接反 / 波特率不符）
 *        TEC 没动 -> 有人应答了，物理链路是通的
 *      这一条直接把"线的问题"和"代码的问题"分开。
 *
 *   4) 收到的回帧全部打印（ID / DLC / 数据），电机的回帧会标出来源。
 *
 *   5) CAN_ABOM = ENABLE：总线出错后自动恢复，不会一次错误就锁死。
 *      （工程原来的 can_ZDT\board.c 里是 DISABLE）
 *
 *   6) 判"有没有人应答"不再只看 TEC，改成看"发送邮箱有没有腾空"。
 *      原因：没人应答时 TEC 会很快冲到 128，节点进入 error-passive，之后
 *      TEC 就卡住不动了 —— 用"TEC 有没有涨"判断，后面每一步都会误报 [OK]。
 *
 *   7) 加了总线扫描：按 's' 键，把 1M/500K/250K/125K/100K/50K 逐个试一遍，
 *      再扫一遍电机地址 0~8，直接告诉你到底是波特率还是地址对不上。
 *      （只发"读版本"指令，电机不会转）
 *
 *  ---------------------------------------------------------------------------
 *  接线（和上一版完全一样）：
 *      核心板 3.3V  ->  VP230 VCC
 *      核心板 GND   ->  VP230 GND
 *      核心板 PA12  ->  VP230 TX        <-- 交叉，不是 TX 对 TX
 *      核心板 PA11  ->  VP230 RX        <--
 *      VP230 H      ->  电机 CAN_H
 *      VP230 L      ->  电机 CAN_L
 *      12V 正       ->  电机 V+
 *      12V 负       ->  电机 GND，并且必须和核心板 GND 连通（共地！）
 *
 *  串口：USART1（PA9 = TX，PA10 = RX），115200，8N1
 *
 *  ---------------------------------------------------------------------------
 *  怎么用：
 *    1) 先只插板子的 USB 串口线，打开串口助手（COM9 / 115200），看自检那几行。
 *       如果是 [BAD]，先别接电机，那是单片机自己的问题。
 *    2) 再按上面接线接上 VP230 和电机，重新上电。
 *    3) 看每条指令下面跟的是 [OK]（有人应答）还是 [BAD]（没人应答）。
 *
 *  自动序列会一直循环。任何时候都可以敲键盘：
 *      '0' 停止      '1' ~ '9' 手动转速 = n * 100 RPM     'a' 回到自动序列
 *      'e' 使能      'v' 读固件版本      's' 扫描总线（波特率 + 地址）
 *
 *  【注意】所有串口输出都写成 ASCII，不要改成中文 ——
 *          工程里配的串口助手是按 ASCII 解码的，中文会变成乱码。
 * ========================================================================== */

#include "stm32f4xx.h"
#include "Emm_V5.h"


/* ==========================================================================
 *  ① 参数区
 * ========================================================================== */

#define MOTOR_ADDR      1        /* 电机 CAN 地址（出厂默认一般是 1） */
#define TEST_ACC        200      /* 加速度 0 ~ 255，0 = 直接启动（很猛） */
#define UART_BAUD       115200   /* 串口波特率 */

#define LOOP_MS         10       /* 主循环节拍（毫秒） */
#define ACK_WAIT_MS     30       /* 发完一帧等多久，用来判断有没有人应答 */
#define LOOPBACK_TEST   1        /* 1 = 上电先做回环自检 */

/* 自动演示序列：每一步做什么、用什么数、持续多久 */
typedef struct
{
    uint8_t  kind;               /* 1=读版本 2=使能 3=正转 4=反转 5=停止 */
    uint16_t val;                /* 速度，单位 RPM */
    uint16_t ms;                 /* 这一步持续多少毫秒 */
} Step_t;

static const Step_t STEP_TABLE[] =
{
    {1,   0,  800},              /* 读固件版本（只读，电机不会动） */
    {2,   0,  300},              /* 使能 */
    {3, 100, 1500},              /* 正转 100 RPM（先慢，确认方向和反应） */
    {3, 300, 1500},              /* 正转 300 RPM */
    {3, 600, 1500},              /* 正转 600 RPM */
    {5,   0,  800},              /* 停 */
    {4, 300, 1500},              /* 反转 300 RPM */
    {5,   0, 1200}               /* 停 */
};

#define STEP_COUNT      (sizeof(STEP_TABLE) / sizeof(STEP_TABLE[0]))


/* ==========================================================================
 *  ② 串口（USART1：PA9 = TX，PA10 = RX）+ 打印小工具
 * ========================================================================== */

static void UART_Init(void)
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

static void P_Ch(char c)
{
    USART_SendData(USART1, (uint16_t)c);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) { }
}

static void P_Str(const char *s)
{
    while (*s != '\0') { P_Ch(*s); s++; }
}

/* 打印十进制无符号整数 */
static void P_U32(uint32_t v)
{
    char    buf[11];
    uint8_t i = 0U;

    if (v == 0U) { P_Ch('0'); return; }

    while (v > 0U && i < 11U) { buf[i] = (char)('0' + (v % 10U)); v /= 10U; i++; }
    while (i > 0U) { i--; P_Ch(buf[i]); }
}

/* 打印一个字节的十六进制，固定两位，比如 0x0F -> "0F" */
static void P_Hex2(uint8_t b)
{
    const char *hex = "0123456789ABCDEF";
    P_Ch(hex[(b >> 4) & 0x0FU]);
    P_Ch(hex[b & 0x0FU]);
}

/* 打印 29 位扩展帧 ID，固定 8 位，比如 0x100 -> "00000100" */
static void P_Hex8(uint32_t v)
{
    P_Hex2((uint8_t)(v >> 24));
    P_Hex2((uint8_t)(v >> 16));
    P_Hex2((uint8_t)(v >> 8));
    P_Hex2((uint8_t)v);
}

/* 读一个串口字节；没有数据返回 0 */
static uint8_t UART_GetByte(void)
{
    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
    {
        return (uint8_t)USART_ReceiveData(USART1);
    }
    return 0U;
}


/* ==========================================================================
 *  ③ 毫秒延时
 *      工程里 ..\SYSTEM\delay\delay.c 的 delay_ms() 必须先调 delay_init() 才能用，
 *      这里干脆自己用 SysTick 写一个，不受别的初始化影响。
 * ========================================================================== */

static void Wait_ms(uint32_t ms)
{
    uint32_t i;
    uint32_t reload = SystemCoreClock / 8000U;      /* SysTick 走 HCLK/8，这是 1ms */

    if (reload == 0U)         { reload = 168000U / 8U; }
    if (reload > 0x00FFFFFFU) { reload = 0x00FFFFFFU; }

    SysTick->LOAD = reload - 1U;
    SysTick->VAL  = 0U;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;        /* CLKSOURCE = 0 -> HCLK/8 */

    for (i = 0U; i < ms; i++)
    {
        SysTick->VAL = 0U;
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0U) { }
    }

    SysTick->CTRL = 0U;
}


/* ==========================================================================
 *  ③b 波特率表（'s' 扫描时用）
 *      波特率 = 42MHz / 分频 / (1 + BS1 + BS2)
 *      张大头电机常用的就这几档，扫描时从高往低试。
 * ========================================================================== */

typedef struct
{
    uint8_t  bs1;
    uint8_t  bs2;
    uint16_t prescaler;
    uint16_t kbps;
} Baud_t;

static const Baud_t BAUDS[] =
{
    { CAN_BS1_10tq, CAN_BS2_3tq,  3U, 1000U },      /* 42M / 3  / 14 = 1.000M */
    { CAN_BS1_9tq,  CAN_BS2_2tq,  7U,  500U },      /* 42M / 7  / 12 = 0.500M */
    { CAN_BS1_9tq,  CAN_BS2_2tq, 14U,  250U },      /* 42M / 14 / 12 = 0.250M */
    { CAN_BS1_9tq,  CAN_BS2_2tq, 28U,  125U },
    { CAN_BS1_9tq,  CAN_BS2_2tq, 35U,  100U },
    { CAN_BS1_9tq,  CAN_BS2_2tq, 70U,   50U }
};

#define BAUD_COUNT      (sizeof(BAUDS) / sizeof(BAUDS[0]))


/* ==========================================================================
 *  ④ CAN 初始化
 *      和 can_ZDT\board.c 里的 can_init() 相比，只多/改了两件事：
 *        - 模式可以传进来（回环 / 正常），方便做自检
 *        - CAN_ABOM = ENABLE，总线出错后自动恢复
 *        - 波特率的分频 / BS1 / BS2 也传进来，方便扫描各种波特率
 *      波特率、帧格式、过滤器都和原来完全一致（500K / 扩展帧 / 全通）。
 * ========================================================================== */

static void CAN_InitEx(uint8_t mode, uint16_t prescaler, uint8_t bs1, uint8_t bs2)
{
    GPIO_InitTypeDef      GPIO_InitStructure;
    CAN_InitTypeDef       CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,  ENABLE);

    /* PA12 = CAN1_TX */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA11 = CAN1_RX */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);

    /* 换模式之前先复位，免得上一次的配置残留 */
    CAN_DeInit(CAN1);

    CAN_StructInit(&CAN_InitStructure);
    CAN_InitStructure.CAN_TTCM = DISABLE;           /* 关闭时间触发通信 */
    CAN_InitStructure.CAN_ABOM = ENABLE;            /* ★ 自动离线恢复 */
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;           /* 允许自动重发 */
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = mode;
    CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1  = bs1;
    CAN_InitStructure.CAN_BS2  = bs2;
    CAN_InitStructure.CAN_Prescaler = prescaler;
    CAN_Init(CAN1, &CAN_InitStructure);

    /* 过滤器：只收扩展帧，ID 全通（收到什么都进 FIFO0） */
    CAN_FilterInitStructure.CAN_FilterNumber         = 1;
    CAN_FilterInitStructure.CAN_FilterMode           = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale          = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh         = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow          = CAN_Id_Extended;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh     = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow      = CAN_Id_Extended;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation     = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    /* 收帧中断（中断处理函数在 can_ZDT\can.c 里） */
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}


/* ==========================================================================
 *  ⑤ 打印 + 发送
 * ========================================================================== */

/* 把一条指令"翻译"成它真正会出现在总线上的帧。规则和 can_ZDT\can.c 的
 * can_SendCmd() 完全一样：ID = (地址 << 8) | 包序号，数据第 0 字节是功能码，
 * 一条装不下就拆成多帧。 */
static void Print_Frames(const uint8_t *cmd, uint8_t len)
{
    uint8_t i = 0U;
    uint8_t j = (uint8_t)(len - 2U);        /* 去掉地址和功能码后，要发的字节数 */
    uint8_t pack = 0U;
    uint8_t k, l;

    while (i < j)
    {
        k = (uint8_t)(j - i);               /* 还剩几个字节没发 */

        P_Str("      frame");
        P_U32(pack);
        P_Str("  ID=0x");
        P_Hex8(((uint32_t)cmd[0] << 8) | pack);

        if (k < 8U)
        {
            P_Str("  DLC=");
            P_U32((uint32_t)(k + 1U));
            P_Str("  data:");
            P_Str(" ");
            P_Hex2(cmd[1]);                 /* 第一个数据字节永远是功能码 */
            for (l = 0U; l < k; l++, i++)
            {
                P_Str(" ");
                P_Hex2(cmd[i + 2U]);
            }
        }
        else
        {
            P_Str("  DLC=8  data:");
            P_Str(" ");
            P_Hex2(cmd[1]);
            for (l = 0U; l < 7U; l++, i++)
            {
                P_Str(" ");
                P_Hex2(cmd[i + 2U]);
            }
        }

        P_Str("\r\n");
        pack++;
    }
}

/* 数一数还有几个发送邮箱是空的。
 * 3 = 三个都空（帧都发完了）；小于 3 = 还有帧卡在邮箱里等 ACK。
 * 这是"到底有没有人应答"最可靠的判据（TEC 会卡住，这个不会）。*/
static uint8_t Count_TXfree(void)
{
    uint8_t  free = 0U;
    uint32_t tsr  = CAN1->TSR;

    if ((tsr & CAN_TSR_TME0) != 0U) { free++; }
    if ((tsr & CAN_TSR_TME1) != 0U) { free++; }
    if ((tsr & CAN_TSR_TME2) != 0U) { free++; }

    return free;
}

/* 打印当前总线状态：错误计数 + 空闲邮箱 + 错误等级 */
static void Print_Bus(void)
{
    uint8_t  tec  = (uint8_t)((CAN1->ESR & CAN_ESR_TEC) >> 16);
    uint8_t  rec  = (uint8_t)((CAN1->ESR & CAN_ESR_REC) >> 24);

    P_Str("      bus: TEC=");
    P_U32(tec);
    P_Str(" REC=");
    P_U32(rec);
    P_Str("  TXfree=");
    P_U32((uint32_t)Count_TXfree());
    P_Str("/3");

    if ((CAN1->ESR & CAN_ESR_BOFF) != 0U)      { P_Str("  [BUS-OFF]"); }
    else if ((CAN1->ESR & CAN_ESR_EPVF) != 0U) { P_Str("  [error-passive]"); }
    else if ((CAN1->ESR & CAN_ESR_EWGF) != 0U) { P_Str("  [error-warning]"); }
    else                                       { P_Str("  [ok]"); }

    P_Str("\r\n");
}

/* 把最后一个错误码翻译成人话 */
static void Print_Lec(uint8_t lec)
{
    P_Str("      LEC=");
    P_U32(lec);

    switch (lec)
    {
        case 0U:  P_Str(" (no error)\r\n"); break;
        case 1U:  P_Str(" (stuff error)\r\n"); break;
        case 2U:  P_Str(" (form error)\r\n"); break;
        case 3U:  P_Str(" (ACK error: nobody answered) <-- most common\r\n"); break;
        case 4U:  P_Str(" (bit recessive error)\r\n"); break;
        case 5U:  P_Str(" (bit dominant error: check CAN_H / CAN_L)\r\n"); break;
        case 6U:  P_Str(" (CRC error)\r\n"); break;
        default:  P_Str(" (software error)\r\n"); break;
    }
}

/* 把卡在邮箱里等 ACK 的帧全部丢掉，让下一次发送是干净的 */
static void Flush_Tx(void)
{
    CAN_CancelTransmit(CAN1, 0U);       /* 邮箱 0 */
    CAN_CancelTransmit(CAN1, 1U);       /* 邮箱 1 */
    CAN_CancelTransmit(CAN1, 2U);       /* 邮箱 2 */
    Wait_ms(1U);
}

/* 发送一条指令，并且报告结果
 * 返回 1 = 帧真的发出去了（有人应答），0 = 没人应答
 *
 * 【判据】看"发送邮箱有没有腾空"，不只看 TEC。
 *   帧被别的节点 ACK 之后，邮箱会在 0.2ms 内自动腾空（500K 下一帧才 ~130us）；
 *   没人应答时帧会一直重发，邮箱就一直占着。
 *   为什么不只看 TEC：一旦没人应答，TEC 会很快涨到 128，节点进入 error-passive，
 *   之后 TEC 就卡住不动了 —— 用"TEC 有没有涨"判断会得到假的 [OK]。 */
static uint8_t Send_Cmd(const char *tag, uint8_t *cmd, uint8_t len)
{
    uint8_t  tec_after, lec;

    P_Str(tag);
    P_Str("\r\n");
    Print_Frames(cmd, len);

    /* 先把上一次卡住（等不到 ACK）的帧丢掉，保证这一条是干净的 */
    Flush_Tx();

    if ((CAN1->TSR & CAN_TSR_TME) == 0U)
    {
        P_Str("      [BAD] all 3 TX mailboxes are still busy -> nobody answered.\r\n");
        Print_Bus();
        return 0U;
    }

    can_SendCmd((__IO uint8_t *)cmd, len);          /* 真正发送，走工程原来的驱动 */

    Wait_ms(ACK_WAIT_MS);

    tec_after = (uint8_t)((CAN1->ESR & CAN_ESR_TEC) >> 16);
    lec       = (uint8_t)((CAN1->ESR & CAN_ESR_LEC) >> 4);

    if ((CAN1->ESR & CAN_ESR_BOFF) != 0U)
    {
        P_Str("      [BAD] bus-off (too many errors). ABOM will restart CAN1.\r\n");
        Print_Bus();
        return 0U;
    }

    if (Count_TXfree() == 3U)
    {
        P_Str("      [OK] the frame left the mailbox -> another device ACKed it.\r\n");
        Print_Bus();
        return 1U;
    }

    P_Str("      [BAD] nobody answered: ");
    P_U32((uint32_t)(3U - Count_TXfree()));
    P_Str(" frame(s) still waiting for an ACK.\r\n");
    Print_Lec(lec);
    P_Str("            => the frame went out but no device sent ACK:\r\n");
    P_Str("               motor power off / CAN_H-CAN_L open, swapped, or not\r\n");
    P_Str("               reaching the motor / baudrate or address mismatch.\r\n");
    if (tec_after >= 128U)
    {
        P_Str("            (node is error-passive from earlier errors; press 's'\r\n");
        P_Str("             to scan baudrate and motor address)\r\n");
    }
    Print_Bus();
    return 0U;
}

/* 静默发一条指令：不打印帧，只回 1 = 有人应答 */
static uint8_t Try_Send(uint8_t *cmd, uint8_t len)
{
    Flush_Tx();

    if ((CAN1->TSR & CAN_TSR_TME) == 0U) { return 0U; }

    can_SendCmd((__IO uint8_t *)cmd, len);
    Wait_ms(ACK_WAIT_MS);

    return (Count_TXfree() == 3U) ? 1U : 0U;
}

/* 总线扫描：先扫波特率，再扫电机地址。只发"读版本"，电机不会动。 */
static void Scan_Bus(void)
{
    uint8_t i;
    uint8_t r;
    uint8_t found;
    uint8_t cmd[3];

    found = 0xFFU;

    P_Str("\r\n=== scan 1/2: which baudrate does the motor answer at? ===\r\n");
    P_Str("    (only 'read version' is sent -- the motor will NOT move)\r\n");

    for (i = 0U; i < (uint8_t)BAUD_COUNT; i++)
    {
        CAN_InitEx(CAN_Mode_Normal, BAUDS[i].prescaler, BAUDS[i].bs1, BAUDS[i].bs2);

        cmd[0] = MOTOR_ADDR;
        cmd[1] = 0x1F;                  /* 读固件版本 */
        cmd[2] = 0x6B;
        r = Try_Send(cmd, 3U);

        P_Str("    ");
        P_U32((uint32_t)BAUDS[i].kbps);
        P_Str("K : ");
        if (r != 0U)
        {
            P_Str("<<< ANSWERED\r\n");
            if (found == 0xFFU) { found = i; }
        }
        else
        {
            P_Str("no answer\r\n");
        }
    }

    if (found == 0xFFU)
    {
        CAN_InitEx(CAN_Mode_Normal, 7U, CAN_BS1_9tq, CAN_BS2_2tq);   /* 回到 500K */
        P_Str("    -> NO baudrate was answered. The motor is not on the bus:\r\n");
        P_Str("       not powered / CAN_H-CAN_L not reaching it / address wrong.\r\n");
        P_Str("=== scan done (kept 500K) ===\r\n");
        return;
    }

    CAN_InitEx(CAN_Mode_Normal, BAUDS[found].prescaler, BAUDS[found].bs1, BAUDS[found].bs2);
    P_Str("    -> staying at ");
    P_U32((uint32_t)BAUDS[found].kbps);
    P_Str("K for the address scan\r\n");

    P_Str("\r\n=== scan 2/2: which motor address answers? ===\r\n");
    P_Str("    (only 'read version' is sent -- the motor will NOT move)\r\n");

    for (i = 0U; i <= 8U; i++)
    {
        cmd[0] = i;
        cmd[1] = 0x1F;
        cmd[2] = 0x6B;
        r = Try_Send(cmd, 3U);

        P_Str("    addr ");
        P_U32((uint32_t)i);
        if (i == 0U) { P_Str(" (broadcast)"); }
        P_Str(" : ");
        if (r != 0U) { P_Str("<<< ANSWERED\r\n"); }
        else         { P_Str("no answer\r\n"); }
    }

    P_Str("=== scan done ===\r\n");
}

/* 打印一帧收到的数据 */
static void Print_RxMsg(__IO CanRxMsg *m)
{
    uint8_t i;

    P_Str("  ID=0x");
    P_Hex8(m->ExtId);
    P_Str("  DLC=");
    P_U32((uint32_t)m->DLC);
    P_Str("  data:");
    for (i = 0U; i < m->DLC; i++)
    {
        P_Str(" ");
        P_Hex2(m->Data[i]);
    }
}

/* 主循环里轮询：有回帧就打印出来 */
static void Print_Rx(void)
{
    if (!can.rxFrameFlag) { return; }
    can.rxFrameFlag = false;

    P_Str("  <- RX frame ");
    Print_RxMsg(&can.CAN_RxMsg);
    P_Str("\r\n");

    if (((can.CAN_RxMsg.ExtId >> 8) & 0xFFU) == (uint32_t)MOTOR_ADDR)
    {
        P_Str("     (from motor ");
        P_U32(MOTOR_ADDR);
        P_Str(", function code 0x");
        P_Hex2(can.CAN_RxMsg.Data[0]);
        P_Str(")\r\n");
    }
}


/* ==========================================================================
 *  ⑥ 回环自检：不接电机、不接线，先证明单片机自己能发能收
 * ========================================================================== */

static uint8_t SelfTest_LoopBack(void)
{
    uint8_t  cmd[8];
    uint16_t t;

    P_Str("\r\n--- [1] CAN loopback self-test (no motor, no wiring) ---\r\n");

    CAN_InitEx(CAN_Mode_LoopBack, 7U, CAN_BS1_9tq, CAN_BS2_2tq);
    can.rxFrameFlag = false;

    cmd[0] = MOTOR_ADDR;
    cmd[1] = 0xF6;                  /* 速度控制 */
    cmd[2] = 0x00;                  /* 正转 */
    cmd[3] = 0x00;
    cmd[4] = 0x64;                  /* 100 RPM */
    cmd[5] = TEST_ACC;
    cmd[6] = 0x00;                  /* 不用同步 */
    cmd[7] = 0x6B;                  /* 校验字节 */

    Print_Frames(cmd, 8U);
    can_SendCmd(cmd, 8U);

    for (t = 0U; t < 200U; t++)
    {
        if (can.rxFrameFlag)
        {
            can.rxFrameFlag = false;
            P_Str("  [OK] loopback passed, got my own frame back:\r\n");
            P_Str("  <- ");
            Print_RxMsg(&can.CAN_RxMsg);
            P_Str("\r\n");
            P_Str("       => CAN1 peripheral and frame format are fine.\r\n");
            return 1U;
        }
        Wait_ms(1U);
    }

    P_Str("  [BAD] loopback failed: even my own frame did not come back.\r\n");
    P_Str("        => this is a STM32-side problem (CAN init / clock).\r\n");
    P_Str("           Do NOT blame the motor or the wiring for this one.\r\n");
    return 0U;
}


/* ==========================================================================
 *  ⑦ 电机指令
 *      这几个函数填的字节和 Emm_V5.c 里的 Emm_V5_En_Control() /
 *      Emm_V5_Vel_Control() 完全一样，只是自己填一遍，方便把帧打印出来。
 * ========================================================================== */

/* 读固件版本：地址 + 0x1F + 0x6B，一帧，电机不会动 */
static void Motor_ReadVersion(void)
{
    uint8_t cmd[3];

    cmd[0] = MOTOR_ADDR;
    cmd[1] = 0x1F;
    cmd[2] = 0x6B;

    Send_Cmd("    cmd 0x1F  read firmware version", cmd, 3U);
}

/* 使能电机：地址 + 0xF3 + 0xAB + 使能状态 + 同步标志 + 0x6B */
static void Motor_Enable(void)
{
    uint8_t cmd[6];

    cmd[0] = MOTOR_ADDR;
    cmd[1] = 0xF3;
    cmd[2] = 0xAB;
    cmd[3] = 0x01;                  /* 01 = 使能，00 = 失能 */
    cmd[4] = 0x00;                  /* 不用同步 */
    cmd[5] = 0x6B;

    Send_Cmd("    cmd 0xF3  enable motor", cmd, 6U);
}

/* 速度模式：地址 + 0xF6 + 方向 + 速度高 + 速度低 + 加速度 + 同步 + 0x6B */
static void Motor_Vel(uint8_t dir, uint16_t rpm)
{
    uint8_t cmd[8];

    cmd[0] = MOTOR_ADDR;
    cmd[1] = 0xF6;
    cmd[2] = dir;                                   /* 0 = 正转 CW，非 0 = 反转 CCW */
    cmd[3] = (uint8_t)(rpm >> 8);
    cmd[4] = (uint8_t)(rpm & 0x00FFU);
    cmd[5] = TEST_ACC;
    cmd[6] = 0x00;
    cmd[7] = 0x6B;

    if (dir == 0U) { Send_Cmd("    cmd 0xF6  run CW",  cmd, 8U); }
    else           { Send_Cmd("    cmd 0xF6  run CCW", cmd, 8U); }
}

/* 停：速度给 0，按加速度平滑减速（不是急停） */
static void Motor_Stop(void)
{
    uint8_t cmd[8];

    cmd[0] = MOTOR_ADDR;
    cmd[1] = 0xF6;
    cmd[2] = 0x00;
    cmd[3] = 0x00;
    cmd[4] = 0x00;
    cmd[5] = TEST_ACC;
    cmd[6] = 0x00;
    cmd[7] = 0x6B;

    Send_Cmd("    cmd 0xF6  stop (speed = 0)", cmd, 8U);
}


/* ==========================================================================
 *  ⑧ 自动序列 + 键盘
 * ========================================================================== */

static uint8_t g_auto = 1U;         /* 1 = 正在跑自动序列 */

static void Run_Step(uint8_t kind, uint16_t val)
{
    switch (kind)
    {
        case 1U:
            P_Str("\r\n>>> step: read version (motor will NOT move)\r\n");
            Motor_ReadVersion();
            break;

        case 2U:
            P_Str("\r\n>>> step: enable motor\r\n");
            Motor_Enable();
            break;

        case 3U:
            P_Str("\r\n>>> step: run CW  ");
            P_U32((uint32_t)val);
            P_Str(" rpm\r\n");
            Motor_Vel(0U, val);
            break;

        case 4U:
            P_Str("\r\n>>> step: run CCW ");
            P_U32((uint32_t)val);
            P_Str(" rpm\r\n");
            Motor_Vel(1U, val);
            break;

        default:
            P_Str("\r\n>>> step: stop\r\n");
            Motor_Stop();
            break;
    }
}

static void Handle_Key(uint8_t c)
{
    P_Str("\r\n[key] ");

    if (c == '0')                                   /* 停止 */
    {
        P_Str("manual stop\r\n");
        g_auto = 0U;
        Motor_Stop();
    }
    else if (c >= '1' && c <= '9')                  /* 手动转速 = n * 100 RPM */
    {
        uint16_t rpm = (uint16_t)(((uint16_t)(c - '0')) * 100U);

        P_Str("manual run CW ");
        P_U32((uint32_t)rpm);
        P_Str(" rpm\r\n");
        g_auto = 0U;
        Motor_Enable();
        Motor_Vel(0U, rpm);
    }
    else if (c == 'e' || c == 'E')
    {
        P_Str("enable motor\r\n");
        g_auto = 0U;
        Motor_Enable();
    }
    else if (c == 'v' || c == 'V')
    {
        P_Str("read firmware version\r\n");
        g_auto = 0U;
        Motor_ReadVersion();
    }
    else if (c == 's' || c == 'S')
    {
        P_Str("scan the bus (baudrate + motor address)\r\n");
        g_auto = 0U;
        Scan_Bus();
    }
    else if (c == 'a' || c == 'A')
    {
        P_Str("back to auto sequence\r\n");
        g_auto = 1U;
    }
    else
    {
        P_Str("got '");
        P_Ch((char)c);
        P_Str("' (no action)\r\n");
    }
}


/* ==========================================================================
 *  ⑨ 主程序
 * ========================================================================== */

int main(void)
{
    uint8_t  idx;
    uint16_t elapsed;
    uint8_t  c;

    UART_Init();
    Wait_ms(200U);                          /* 等串口稳定，别把开头几个字丢了 */

    P_Str("\r\n\r\n");
    P_Str("========================================\r\n");
    P_Str(" X42S motor CAN test  (self-check ver)\r\n");
    P_Str("========================================\r\n");
    P_Str("SystemCoreClock = ");
    P_U32(SystemCoreClock);
    P_Str(" Hz\r\n");
    P_Str("motor addr = ");
    P_U32(MOTOR_ADDR);
    P_Str("   acc = ");
    P_U32(TEST_ACC);
    P_Str("\r\n");

    nvic_init();                            /* CAN 收帧中断（can_ZDT\board.c） */

#if LOOPBACK_TEST
    SelfTest_LoopBack();
#endif

    CAN_InitEx(CAN_Mode_Normal, 7U, CAN_BS1_9tq, CAN_BS2_2tq);
    P_Str("\r\n--- [2] normal mode: 500K, extended frame ---\r\n");
    P_Str("keys: 0=stop  1..9=manual speed (n*100 rpm)  a=auto  e=enable\r\n");
    P_Str("      v=version   s=scan bus (baudrate + motor address)\r\n");

    idx     = 0U;
    elapsed = 0U;
    Run_Step(STEP_TABLE[idx].kind, STEP_TABLE[idx].val);

    while (1)
    {
        c = UART_GetByte();
        if (c != 0U) { Handle_Key(c); }

        Print_Rx();

        if (g_auto)
        {
            elapsed = (uint16_t)(elapsed + LOOP_MS);
            if (elapsed >= STEP_TABLE[idx].ms)
            {
                elapsed = 0U;
                idx = (uint8_t)((idx + 1U) % STEP_COUNT);
                Run_Step(STEP_TABLE[idx].kind, STEP_TABLE[idx].val);
            }
        }

        Wait_ms(LOOP_MS);
    }
}
