/* ============================================================================
 *  STM32 第 7 课 · 定时器中断（TIM3）+ 状态机
 *  ---------------------------------------------------------------------------
 *  【这一课解决什么问题】
 *    第 4 课的主循环是"查一下有没有数据，没有就再查"——这叫「忙等」。
 *    忙等的毛病：想知道"过了多少毫秒"只能傻等（Wait_ms），等的时候什么都干不了；
 *    任务一多就互相拖累（盯着串口的时候，按键就丢了）。
 *
 *    真正的机器人程序是「定时器在后台数拍子，主循环按拍子干活」。
 *    这一课就是把那个"后台数拍子"做出来，再用它驱动一个状态机。
 *
 *  【接线】
 *    一根线都不用接！只插板子右边的 USB UART 口看打印就行。
 *    （PA0 每 500ms 翻转一次，想用示波器看心跳可以接 PA0，不接也不影响。）
 *
 *  【怎么算通过】—— 四条，全中才算过
 *    1) 上电后串口每 200ms 打印一行 tick=... state=...，tick 一直在涨；
 *    2) 不碰它，状态会自己走 IDLE -> RUN -> BRAKE -> IDLE，一圈约 4.5 秒；
 *    3) 敲 't' 看开机秒数，和手机秒表对 60 秒，误差应该 < 0.1 秒；
 *    4) 敲 '1' / '2' / '3' 能把状态强行拨过去，'f' 进 FAULT，'r' 复位。
 *
 *  串口：USART1（PA9 = TX，PA10 = RX），115200，8N1。
 *  【注意】串口输出全部写成 ASCII，不要改成中文——串口助手按 ASCII 解码。
 * ========================================================================== */

#include "stm32f4xx.h"


/* ==========================================================================
 *  ① 参数区 —— 平时要改的东西全在这里
 * ========================================================================== */

#define UART_BAUD        115200U   /* 串口波特率 */

/* 定时器中断频率。1000 = 每 1ms 中断一次，这个节拍叫「系统节拍」 */
#define TICK_HZ          1000U

/* 主循环每多少毫秒打印一行（只影响看着舒服，不影响状态机的节奏） */
#define PRINT_EVERY_MS   200U

/* 心跳灯：PA0，第 2 课用过的那个脚。每 BLINK_MS 翻转一次 */
#define BLINK_MS         500U

/* 状态机每一步自动停留多久（毫秒）。想改节奏，就改这三个数 */
#define IDLE_HOLD_MS     1000U
#define RUN_HOLD_MS      3000U
#define BRAKE_HOLD_MS    500U

/* --------------------------------------------------------------------------
 *  定时器参数（第 3 步会算，这里先摆出来）
 *      TIM3 挂在 APB1 上。APB1 本身是 42MHz，但【定时器的时钟要 ×2 = 84MHz】，
 *      这是 STM32F4 的经典坑，很多人在这儿把时间算成一半。
 *
 *      中断频率 = TIMxCLK / ((PSC + 1) * (ARR + 1))
 *               = 84 000 000 / (8400 * 10)
 *               = 1000 Hz  →  每 1ms 中断一次
 * ------------------------------------------------------------------------ */
#define TIMER_CLK_HZ     84000000U  /* TIM3 的时钟（不是 42MHz！） */
#define TIM_PSC          8400U      /* 预分频：84MHz / 8400 = 10kHz，每格 0.1ms */
#define TIM_ARR          10U        /* 数 10 格溢出一次 → 1kHz */


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
    while (*s != '\0')
    {
        P_Ch(*s);
        s++;
    }
}

/* 打印一个 0 ~ 4294967295 的无符号整数（不补前导 0） */
static void P_U32(uint32_t v)
{
    char buf[11];
    int  i = 0;

    if (v == 0U)
    {
        P_Ch('0');
        return;
    }

    while (v > 0U && i < 10)
    {
        buf[i] = (char)('0' + (int)(v % 10U));
        v /= 10U;
        i++;
    }

    while (i > 0)
    {
        i--;
        P_Ch(buf[i]);
    }
}

/* 打印毫秒数，固定三位小数：12345 -> "12.345" */
static void P_Uptime(uint32_t ms_total)
{
    uint32_t ms3 = ms_total % 1000U;

    P_U32(ms_total / 1000U);
    P_Str(".");
    P_Ch((char)('0' + (int)(ms3 / 100U)));
    P_Ch((char)('0' + (int)((ms3 / 10U) % 10U)));
    P_Ch((char)('0' + (int)(ms3 % 10U)));
}

/* 读一个字节，没数据就返回 0 */
static uint8_t UART_GetByte(void)
{
    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
    {
        return (uint8_t)USART_ReceiveData(USART1);
    }
    return 0U;
}


/* ==========================================================================
 *  ③ 定时器中断：整个系统的心跳
 * ========================================================================== */

/* ★ 这个变量被"中断"和"主循环"同时读写，必须加 volatile。
 *   不加的话，编译器会觉得"主循环里没人改它"，把读操作优化掉，
 *   于是 tick 永远停在 0，你会以为中断没进——这是本课最常见的坑。 */
static volatile uint32_t g_tick_ms = 0U;

static void Tick_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;

    /* TIM3 挂在 APB1，RCC_APB1Periph_TIM3 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Prescaler         = TIM_PSC - 1U;
    TIM_TimeBaseInitStructure.TIM_Period            = TIM_ARR - 1U;
    TIM_TimeBaseInitStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    /* 让 TIM3 的"数到头了"这件事能产生中断 */
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    /* 在 NVIC（中断总闸）里把 TIM3 这条路打开 */
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1U;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1U;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 最后才让定时器开始跑 */
    TIM_Cmd(TIM3, ENABLE);
}

/* --------------------------------------------------------------------------
 *  中断服务函数（ISR）
 *
 *  ★ 三条铁律：
 *    1) 名字必须和启动文件里的向量表完全一致（startup_stm32f4xx.s 里叫
 *       TIM3_IRQHandler），拼错一个字，中断就会跑进默认的死循环。
 *    2) 进来先清中断标志，不然它会一直重复触发，主循环再也回不去。
 *    3) 里面只干"最少的事"。这里就一句计数。千万别在里面 Wait_ms、
 *       打印一大串、或者做浮点运算——那会把整个系统拖卡。
 * ------------------------------------------------------------------------ */
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

        g_tick_ms++;                    /* 唯一的工作：拍子 +1 */
    }
}


/* ==========================================================================
 *  ④ 心跳灯（PA0）
 *      有了 1ms 的拍子，让灯每 500ms 翻一次，就是一句话的事。
 *      想确认"中断真的在跑"，把示波器接 PA0，应该看到 1Hz 的方波。
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
 *  ⑤ 状态机
 *      状态机的三要素：① 现在在哪个状态 ② 这个状态进来多久了 ③ 满足什么条件走
 * ========================================================================== */

typedef enum
{
    ST_IDLE = 0,        /* 待机：等一会儿自己去 RUN */
    ST_RUN,             /* 运行：真项目里这里就是"读陀螺仪 -> 算偏差 -> 给电机发指令" */
    ST_BRAKE,           /* 刹车：停一下，再回 IDLE */
    ST_FAULT            /* 故障：自己不出来，必须按 'r' 复位 */
} State_t;

static State_t  g_state    = ST_IDLE;
static uint32_t g_state_ms = 0U;    /* 进入当前状态的那一刻的 tick 值 */

static const char *State_Name(State_t s)
{
    switch (s)
    {
    case ST_IDLE:  return "IDLE";
    case ST_RUN:   return "RUN";
    case ST_BRAKE: return "BRAKE";
    case ST_FAULT: return "FAULT";
    default:       return "?";
    }
}

static void State_Enter(State_t s, uint32_t now)
{
    g_state    = s;
    g_state_ms = now;               /* 记下"进来的时刻"，后面的计时全靠它 */
}

/* --------------------------------------------------------------------------
 *  按键：这一版故意用"同一个状态机能被按键拨动"来演示——
 *  主循环里没有任何 Wait_ms，所以按键随时都灵。
 * ------------------------------------------------------------------------ */
static void Handle_Key(uint8_t c, uint32_t now)
{
    switch (c)
    {
    case '1':
        State_Enter(ST_IDLE, now);
        P_Str("[key] -> IDLE\r\n");
        break;

    case '2':
        State_Enter(ST_RUN, now);
        P_Str("[key] -> RUN\r\n");
        break;

    case '3':
        State_Enter(ST_BRAKE, now);
        P_Str("[key] -> BRAKE\r\n");
        break;

    case 'f':
    case 'F':
        State_Enter(ST_FAULT, now);
        P_Str("[key] -> FAULT (press r)\r\n");
        break;

    case 'r':
    case 'R':
        State_Enter(ST_IDLE, now);
        P_Str("[key] reset -> IDLE\r\n");
        break;

    case 't':
    case 'T':
        P_Str("uptime = ");
        P_Uptime(now);
        P_Str(" s\r\n");
        break;

    default:
        break;
    }
}


/* ==========================================================================
 *  ⑥ main
 * ========================================================================== */

int main(void)
{
    uint32_t next_print_ms = 0U;
    uint32_t next_blink_ms = 0U;
    uint8_t  led_on        = 0U;

    UART_Init();
    Led_Init();
    Tick_Init();

    P_Str("\r\n== Lesson 7: TIM3 tick + state machine ==\r\n");
    P_Str("tick = 1ms. keys: 1=IDLE 2=RUN 3=BRAKE f=FAULT r=reset t=uptime\r\n");

    State_Enter(ST_IDLE, g_tick_ms);

    while (1)
    {
        uint32_t now;
        uint32_t elapsed;
        uint8_t  c;

        /* 一轮循环只读一次"现在几点"，后面全用这个 now，
         * 免得同一轮里前后不一致（这是个好习惯）。 */
        now     = g_tick_ms;
        elapsed = now - g_state_ms;

        /* ---------- 心跳：每 500ms 翻转 PA0 ---------- */
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

        /* ---------- 状态机：条件到了就换状态 ---------- */
        switch (g_state)
        {
        case ST_IDLE:
            if (elapsed >= IDLE_HOLD_MS)
            {
                State_Enter(ST_RUN, now);
            }
            break;

        case ST_RUN:
            /* 真项目里，这一格将来要干的活是：
             *   读陀螺仪 yaw -> 和"目标航向"相减 -> PID -> 给电机发速度 */
            if (elapsed >= RUN_HOLD_MS)
            {
                State_Enter(ST_BRAKE, now);
            }
            break;

        case ST_BRAKE:
            if (elapsed >= BRAKE_HOLD_MS)
            {
                State_Enter(ST_IDLE, now);
            }
            break;

        case ST_FAULT:
            /* 故障态不自己退出，只能按 'r'。这就是"状态机比 if 堆砌好"的地方。 */
            break;

        default:
            State_Enter(ST_FAULT, now);
            break;
        }

        /* ---------- 打印 ---------- */
        if (now >= next_print_ms)
        {
            next_print_ms = now + PRINT_EVERY_MS;

            P_Str("tick=");
            P_U32(now);
            P_Str(" state=");
            P_Str(State_Name(g_state));
            P_Str(" (+");
            P_U32(elapsed);
            P_Str("ms)\r\n");
        }

        /* ---------- 收键盘 ---------- */
        c = UART_GetByte();
        if (c != 0U)
        {
            Handle_Key(c, now);
        }
    }
}

