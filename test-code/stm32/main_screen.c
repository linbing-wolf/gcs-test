/* ============================================================================
 *  STM32 第 9 课 · 串口屏：让核心板自己往屏上发字
 *  ---------------------------------------------------------------------------
 *  【这一课解决什么问题】
 *    10/7 晚我们是用"电脑 + USB 转 TTL"把字发到屏上的（t0.txt="..." + 三个 FF）。
 *    但比赛的时候电脑不在车上 —— 发指令的必须是核心板。
 *    这一课把同一句话改成用核心板的 USART6（PC6/PC7，9600）发出去。
 *
 *  【接线】—— 关键动作：把 USB 转 TTL 从屏上摘掉！
 *    屏 RX   <- 核心板 PC6（USART6_TX）      ← 就这一根是"发指令"用的
 *    屏 TX   -> 核心板 PC7（USART6_RX）      ← 可选，只用来收屏的应答
 *    屏 5V   -> 电源模块【绿色座 5V】
 *    屏 GND  -> 电源模块【绿色座 GND】，再和核心板 GND 连一根（共地，本来就该有）
 *    ⚠ USB 转 TTL 的 TXD 如果还插在屏 RX 上，就和 PC6 顶上了（两个输出抢一个输入），
 *      表现是字乱跳 / 干脆不动 —— 所以测这一课前，先把 TTL 那两根信号线拔掉。
 *    ⚠ 电源模块蓝色座是 12V，接屏会烧；红色座 3.3V 带不动。
 *
 *  【怎么算通过】—— 三条
 *    1) 上电后屏上的数字每 0.5 秒 +1（0、1、2、3……）。
 *       数字在跳 = PC6 → 屏 这条线通了，这是本课的核心判据。
 *    2) 在串口助手（插板子右边的 USB UART 口，COM7 / 115200）里打字再回车，
 *       屏上就显示你打的这行字。
 *    3) 敲 'p' 停/继续自动计数，敲 'c' 清空，敲 'z' 显示中文"你好"。
 *       （打字回车、或按 'z' 时会自动先停掉自动计数 —— 不停的话数字 500ms 就把它盖回去了；
 *         想再看到数字跳，敲一下 'p'。）
 *    4) **打中文**：串口助手那个输入框里打汉字再回车，屏上就显示这句中文。
 *       原理：汉字在 GB2312 里是 2 个 >= 0x80 的字节，程序原样转发给屏，屏按 GB2312 解码。
 *       ⚠ 前提是**串口助手按 GB2312 发**（tools\serial-gui.ps1 里已经设好了）。
 *         用别的串口助手（比如 XCOM）要自己把编码/字符集选成 GB2312，不然发出去是 '?'。
 *
 *  【注意】串口打印全部写成 ASCII，别改成中文 —— 串口助手按 ASCII 解码。
 * ========================================================================== */

#include "stm32f4xx.h"


/* ==========================================================================
 *  ① 参数区 —— 平时要改的东西全在这里
 * ========================================================================== */

#define DEBUG_BAUD       115200U   /* 调试串口（接电脑那个）波特率 */
#define HMI_BAUD         9600U     /* 串口屏的波特率，10/7 实测就是 9600 */

#define SEND_EVERY_MS    500U      /* 自动计数时，多久往屏上发一次 */
#define PRINT_EVERY_MS   1000U     /* 串口打印的间隔 */
#define BLINK_MS         500U      /* 心跳灯翻转间隔 */

/* 1ms 心跳：84MHz / 8400 = 10kHz，再数 10 格 = 1kHz */
#define TIM_PSC          8400U
#define TIM_ARR          10U

#define LINE_MAX         64U       /* 串口打字的缓冲长度（中文一个字占 2 字节，够写 30 来个汉字） */


/* ==========================================================================
 *  ② 全局变量
 * ========================================================================== */

static volatile uint32_t g_tick_ms = 0U;   /* 开机以来过了多少毫秒 */

static uint8_t  g_auto    = 1U;            /* 1 = 自动每 500ms 发一个递增数字 */
static uint32_t g_counter = 0U;            /* 自动计数用的数字 */

static char     g_line[LINE_MAX];          /* 串口打字的缓冲 */
static uint8_t  g_line_len = 0U;


/* ==========================================================================
 *  ③ 调试串口 USART1（PA9/PA10 → 板载 USB UART 口 → 电脑 COM7）
 * ========================================================================== */

static void Debug_UART_Init(void)
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

    USART_InitStructure.USART_BaudRate            = DEBUG_BAUD;
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
    USART_SendData(USART1, (uint16_t)((uint8_t)c));
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) { }
}

static void P_Str(const char *s)
{
    while (*s != '\0')
    {
        P_Ch(*s);
        s++;
    }
}

static void P_U32(uint32_t v)
{
    char buf[12];
    int  i = 0;
    int  j;
    char t;

    if (v == 0U) { buf[i++] = '0'; }
    while (v > 0U) { buf[i++] = (char)('0' + (v % 10U)); v /= 10U; }
    for (j = 0; j < i / 2; j++) { t = buf[j]; buf[j] = buf[i - 1 - j]; buf[i - 1 - j] = t; }
    buf[i] = '\0';

    P_Str(buf);
}

/* 没有新数据就返回 0 */
static uint8_t Debug_GetByte(void)
{
    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
    {
        return 0U;
    }
    return (uint8_t)(USART_ReceiveData(USART1) & 0x00FFU);
}


/* ==========================================================================
 *  ④ 屏串口 USART6（PC6 = TX，PC7 = RX，9600）
 *
 *  参考工程里这套接口叫 uart6_init / HMISends / HMISendb（在 ckp\uart6_ckp.c），
 *  这里为了让 main.c 自己就能独立编译，重写了一份等价的。
 *
 *  淘晶驰屏的指令格式：<正文 ASCII/GB2312> + 三个 0xFF
 *  例：t0.txt="123" <FF FF FF>   ->  把 t0 这个文字控件的内容改成 123
 * ========================================================================== */

static void HMI_UART_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

    USART_InitStructure.USART_BaudRate            = HMI_BAUD;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART6, &USART_InitStructure);

    USART_Cmd(USART6, ENABLE);
}

/* 发一个字节。参数用 uint8_t，最前面补 0 就落到 DR 的低 8 位，不会碰到校验位 */
static void HMI_Ch(uint8_t c)
{
    USART_SendData(USART6, (uint16_t)c);
    while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET) { }
}

static void HMI_Str(const char *s)
{
    while (*s != '\0')
    {
        HMI_Ch((uint8_t)(*s));
        s++;
    }
}

/* 每条指令的结束符：三个 0xFF，少一个屏都不理你 */
static void HMI_End(void)
{
    HMI_Ch(0xFFU);
    HMI_Ch(0xFFU);
    HMI_Ch(0xFFU);
}

/* 把一行字显示到屏上的 t0 控件 */
static void HMI_ShowText(const char *s)
{
    HMI_Str("t0.txt=\"");
    HMI_Str(s);
    HMI_Str("\"");
    HMI_End();
}

/* 把数字显示到屏上的 t0 控件 */
static void HMI_ShowNumber(uint32_t v)
{
    char buf[12];
    int  i = 0;
    int  j;
    char t;

    if (v == 0U) { buf[i++] = '0'; }
    while (v > 0U) { buf[i++] = (char)('0' + (v % 10U)); v /= 10U; }
    for (j = 0; j < i / 2; j++) { t = buf[j]; buf[j] = buf[i - 1 - j]; buf[i - 1 - j] = t; }
    buf[i] = '\0';

    HMI_ShowText(buf);
}

/* 显示中文"你好"：直接发 GB2312 的 4 个字节 C4 E3 BA C3，
 * 这样和源文件本身是什么编码没关系。 */
static void HMI_ShowHelloCN(void)
{
    HMI_Str("t0.txt=\"");
    HMI_Ch(0xC4U);
    HMI_Ch(0xE3U);
    HMI_Ch(0xBAU);
    HMI_Ch(0xC3U);
    HMI_Str("\"");
    HMI_End();
}


/* ==========================================================================
 *  ⑤ 1ms 心跳（TIM3 中断）
 * ========================================================================== */

static void Tick_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Prescaler         = TIM_PSC - 1U;
    TIM_TimeBaseInitStructure.TIM_Period            = TIM_ARR - 1U;
    TIM_TimeBaseInitStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1U;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1U;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM3, ENABLE);
}

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        g_tick_ms++;
    }
}


/* ==========================================================================
 *  ⑥ 心跳灯 PA0
 * ========================================================================== */

static void Led_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOA, GPIO_Pin_0);
}


/* ==========================================================================
 *  ⑦ 按键处理
 * ========================================================================== */

static void Handle_Byte(uint8_t c)
{
    if ((c == (uint8_t)'\r') || (c == (uint8_t)'\n'))
    {
        P_Str("\r\n");
        if (g_line_len > 0U)
        {
            g_line[g_line_len] = '\0';

            /* 要显示你打的字，就得先把自动计数停掉 —— 否则 500ms 后又被数字盖回去 */
            g_auto = 0U;

            HMI_ShowText(g_line);          /* 把这一行发到屏上 */
            P_Str("-> sent to screen: ");
            P_Str(g_line);
            P_Str("   (auto OFF; press p to resume)\r\n");
            g_line_len = 0U;
        }
        return;
    }

    if ((c == 0x08U) || (c == 0x7FU))      /* 退格 */
    {
        if (g_line_len > 0U)
        {
            g_line_len--;
            P_Str("\b \b");
        }
        return;
    }

    /* 只有"这一行还是空的"时候，p / c / z 才当命令用。
       不然你打 "power" 这种词，第一个字母 p 就把自动计数给切了。 */
    if ((g_line_len == 0U) && (c == (uint8_t)'p'))
    {
        g_auto = (g_auto == 0U) ? 1U : 0U;
        P_Str(g_auto != 0U ? "\r\n[auto ON]\r\n" : "\r\n[auto OFF]\r\n");
        return;
    }

    if ((g_line_len == 0U) && (c == (uint8_t)'c'))
    {
        g_counter  = 0U;
        g_line_len = 0U;
        HMI_ShowText("");
        P_Str("\r\n[clear]\r\n");
        return;
    }

    if ((g_line_len == 0U) && (c == (uint8_t)'z'))
    {
        g_auto = 0U;                       /* 同上：先停自动计数，中文才留得住 */
        HMI_ShowHelloCN();
        P_Str("\r\n[hello GB2312]  (auto OFF; press p to resume)\r\n");
        return;
    }

    /* 其它字符：收进缓冲，等回车一起发。
       注意这里**只挡住 < 0x20 的控制字符**，0x80 以上照收 ——
       一个汉字在 GB2312 里是 2 个 >= 0x80 的字节，挡了就发不出中文了。 */
    if ((c >= 0x20U) && (g_line_len < (LINE_MAX - 1U)))
    {
        g_line[g_line_len] = (char)c;
        g_line_len++;
        P_Ch((char)c);                     /* 回显，让你看得见自己打了什么 */
    }
}


/* ==========================================================================
 *  ⑧ 主程序
 * ========================================================================== */

int main(void)
{
    uint32_t next_send_ms  = 0U;
    uint32_t next_print_ms = 0U;
    uint32_t next_blink_ms = 0U;
    uint8_t  led_on        = 0U;

    Debug_UART_Init();
    Led_Init();
    HMI_UART_Init();
    Tick_Init();

    P_Str("\r\n== Lesson 9: STM32 -> serial screen (USART6 PC6/PC7 @9600) ==\r\n");
    P_Str("type text + Enter = show it on screen (Chinese ok, GB2312)\r\n");
    P_Str("keys (only when input is empty): p=auto on/off  c=clear  z=hello\r\n");

    HMI_ShowText("");                      /* 上电先清屏上的字 */
    HMI_ShowNumber(0U);                    /* 先放一个 0，证明通没通 */

    while (1)
    {
        uint32_t now = g_tick_ms;
        uint8_t  c;

        /* ---------- 心跳灯 ---------- */
        if (now >= next_blink_ms)
        {
            next_blink_ms = now + BLINK_MS;

            if (led_on == 0U)
            {
                led_on = 1U;
                GPIO_SetBits(GPIOA, GPIO_Pin_0);
            }
            else
            {
                led_on = 0U;
                GPIO_ResetBits(GPIOA, GPIO_Pin_0);
            }
        }

        /* ---------- 自动往屏上发递增数字 ---------- */
        if ((g_auto != 0U) && (now >= next_send_ms))
        {
            next_send_ms = now + SEND_EVERY_MS;
            HMI_ShowNumber(g_counter);
            g_counter++;
        }

        /* ---------- 串口收键盘 ---------- */
        c = Debug_GetByte();
        if (c != 0U)
        {
            Handle_Byte(c);
        }

        /* ---------- 串口打印（调试用） ---------- */
        if (now >= next_print_ms)
        {
            next_print_ms = now + PRINT_EVERY_MS;

            P_Str("t=");
            P_U32(now / 1000U);
            P_Str("s  auto=");
            P_U32((uint32_t)g_auto);
            P_Str("  count=");
            P_U32(g_counter);
            P_Str("\r\n");
        }
    }
}
