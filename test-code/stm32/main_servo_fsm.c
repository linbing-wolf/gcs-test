/* ============================================================================
 *  STM32 第 8 课 · 舵机 + 状态机
 *  ---------------------------------------------------------------------------
 *  【这一课解决什么问题】
 *    第 3 课的舵机是"流水账"写法：
 *        摆到 20 度 -> 傻等 1 秒 -> 摆到 135 度 -> 傻等 1 秒 -> ...
 *    傻等着的那 1 秒里，串口按键是收不到的（Wait_ms 把 CPU 占死了）。
 *    而且真实任务是"抓取 -> 抬起 -> 放下"，中途随时可能要求停、要求急停。
 *
 *    第 7 课已经做出了"后台每 1ms 数一拍"的心跳。这一课把舵机挂到这拍子上，
 *    改成【状态机】：每个状态记着"我要摆到几度、摆到以后停多久、下一个状态是谁"，
 *    主循环一个 Wait_ms 都不用，所以任何时候按键都能立刻生效。
 *
 *  【接线】—— 和第 3 课一模一样，接 3 根线
 *    舵机 橙/黄/白（信号） -> 核心板 PA2
 *    舵机 红（+5V）        -> 电源模块【绿色座 5V】
 *    舵机 棕/黑（GND）     -> 电源模块【绿色座 GND】，再从同一个 GND 引一根到核心板 GND
 *    ⚠ 电源模块【蓝色座是 12V】，接上去会烧舵机；红色座 3.3V 带不动。
 *    ⚠ 必须共地：电源模块的 GND 和核心板的 GND 之间要有一根线。
 *    串口看打印：插板子右边的 USB UART 口（COM7 / 115200）。
 *
 *  【怎么算通过】—— 四条，全中才算过
 *    1) 上电后串口每 200ms 打一行 t=... state=... servo=当前/目标，t 一直在涨；
 *       上电后舵机自己在原位（20 度）不动，状态是 IDLE；
 *    2) 敲 'g'：状态自动走 STEP1 -> STEP2 -> STEP3 -> RETURN -> IDLE，一圈约 9 秒，
 *       舵机是【慢慢扫过去】的（不是"嗖"一下跳过去）；
 *    3) 走到一半敲 's'：立刻回 IDLE 并扫回 20 度；敲 'f' 进 FAULT，舵机 PWM 撤掉（松力）；
 *    4) 敲 '1'/'2'/'3' 能把舵机单独拨到三个位置（装机械臂的时候用这个对位置）。
 *
 *  【注意】串口输出全部写成 ASCII，不要改成中文——串口助手按 ASCII 解码。
 * ========================================================================== */

#include "stm32f4xx.h"


/* ==========================================================================
 *  ① 参数区 —— 平时要改的东西全在这里
 * ========================================================================== */

#define UART_BAUD        115200U   /* 串口波特率 */

/* 主循环每多少毫秒打印一行（只影响看着舒服） */
#define PRINT_EVERY_MS   200U

/* 心跳灯：PA0，每 BLINK_MS 翻转一次 */
#define BLINK_MS         500U

/* --------------------------------------------------------------------------
 *  舵机：型号和三个位置
 * ------------------------------------------------------------------------ */
#define SERVO_TRAVEL_DEG 270       /* 180 / 270 / 360，按你的舵机改 */
#define SERVO_CH         3         /* 2 -> PA1   3 -> PA2(现在用这个)   4 -> PA3 */

/* ★ 三个位置，单位是度。想改动作，只改这三个数。建议留 15~255 的余量，
 *   免得舵机顶到机械极限一直"嗡嗡"响。 */
#define DEG_HOME         20        /* 待机/原位 */
#define DEG_STEP1        30        /* 第 1 步 */
#define DEG_STEP2       150        /* 第 2 步 */
#define DEG_STEP3       250        /* 第 3 步 */

/* 舵机扫动的速度：每 SERVO_TICK_MS 毫秒走 SERVO_STEP_DEG 度。
 * 2 度 / 20ms = 100 度每秒。想让它快一点就把这个数加大。
 * ★ 这行就是"状态机"比"流水账"强的地方：扫动这件事交给主循环慢慢做，
 *   而不是用 Wait_ms 把 CPU 堵死。 */
#define SERVO_TICK_MS    20U
#define SERVO_STEP_DEG   2U

/* 每一步"扫到位以后"还停多久（毫秒） */
#define STEP1_HOLD_MS    1000U
#define STEP2_HOLD_MS    1500U
#define STEP3_HOLD_MS    1500U
#define RETURN_HOLD_MS   1000U

/* --------------------------------------------------------------------------
 *  定时器参数（和第 7 课一样）
 *      TIM3 挂在 APB1，但【定时器的时钟要 ×2 = 84MHz】，别按 42MHz 算。
 *      中断频率 = 84MHz / ((8400) * (10)) = 1000Hz -> 每 1ms 一拍
 * ------------------------------------------------------------------------ */
#define TIM_PSC          8400U
#define TIM_ARR          10U


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

/* 打印一个无符号整数（不补前导 0） */
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

/* 打印秒数，固定三位小数：12345ms -> "12.345" */
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
 *  ③ 定时器中断：整个系统的心跳（第 7 课的内容，原样搬过来）
 * ========================================================================== */

/* ★ 被"中断"和"主循环"同时读写的变量，必须加 volatile，
 *   不然编译器会把主循环里的读操作优化掉，tick 永远停在 0。 */
static volatile uint32_t g_tick_ms = 0U;

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

/* 中断服务函数：名字必须和 startup_stm32f4xx.s 里一致，进来先清标志，只干最少的事 */
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

        g_tick_ms++;
    }
}


/* ==========================================================================
 *  ④ 心跳灯（PA0）
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
 *  ⑤ 舵机 PWM（第 3 课的内容）
 *      TIM2 的 CH2 / CH3 / CH4 -> PA1 / PA2 / PA3
 *      频率 50Hz（周期 20ms），定时器的一格 = 1 微秒
 * ========================================================================== */

static void Servo_PWM_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);

    /* 84MHz ÷ 84 = 1MHz（一格 1 微秒），数 20000 格 = 20ms = 50Hz */
    TIM_InternalClockConfig(TIM2);

    TIM_TimeBaseInitStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Prescaler         = 84 - 1;
    TIM_TimeBaseInitStructure.TIM_Period            = 20000 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);

    TIM_Cmd(TIM2, ENABLE);
}

/* 把"脉宽"写进定时器。根据 SERVO_CH 自动选通道，换脚位只改上面的宏 */
static void Servo_WritePulse(uint16_t pulse)
{
#if   SERVO_CH == 2
    TIM_SetCompare2(TIM2, pulse);   /* 通道 2 输出在 PA1 */
#elif SERVO_CH == 3
    TIM_SetCompare3(TIM2, pulse);   /* 通道 3 输出在 PA2 */
#else
    TIM_SetCompare4(TIM2, pulse);   /* 通道 4 输出在 PA3 */
#endif
}

/* 把"角度"换算成脉宽：0° -> 0.5ms，满行程(270°) -> 2.5ms，中间线性插值。
 * 定时器一格正好 1 微秒，所以算出来的数直接就是脉宽的微秒数。
 * ★ 写 0 不是"转到 0 度"，是【完全不输出信号】——舵机松力（用在急停上）。 */
static void Servo_SetAngle(uint16_t angle)
{
    uint32_t pulse;

    if (angle > SERVO_TRAVEL_DEG) { angle = SERVO_TRAVEL_DEG; }

    pulse = 500U + ((uint32_t)angle * 2000U) / SERVO_TRAVEL_DEG;
    Servo_WritePulse((uint16_t)pulse);
}


/* ==========================================================================
 *  ⑥ 状态机 —— 本课的重点
 *
 *  三要素：① 现在在哪个状态  ② 这个状态进来多久了  ③ 满足什么条件换下一个
 *
 *  ★ 关键改动（相对第 3 课）：
 *    第 3 课是"先摆舵机、再 Wait_ms 傻等"。
 *    这里是"进入状态时只登记目标角度，扫过去和计时都交给主循环按拍子做"，
 *    所以主循环里一个 Wait_ms 都没有，按键随时都灵。
 * ========================================================================== */

typedef enum
{
    ST_IDLE = 0,        /* 待机：舵机停在原位，等 'g' 才开始（不自己走） */
    ST_STEP1,           /* 动作第 1 步 */
    ST_STEP2,           /* 动作第 2 步 */
    ST_STEP3,           /* 动作第 3 步 */
    ST_RETURN,          /* 回原位 */
    ST_FAULT,           /* 故障：PWM 撤掉、舵机松力，只能按 'r' 出来 */
    ST_COUNT
} State_t;

/* 动作表：把"每一步摆到几度、停多久、下一步是谁"写成一张表。
 * 想改动作顺序，只改这张表，主循环一个字都不用动 —— 这就是状态机的价值。 */
typedef struct
{
    const char *name;
    uint16_t    angle;      /* 这一步的目标角度 */
    uint32_t    hold_ms;    /* 扫到位以后再停多久 */
    State_t     next;       /* 下一步 */
} Step_t;

static const Step_t kPlan[6] = {
    /* name        angle       hold_ms          next        */
    { "IDLE",      DEG_HOME,   0U,              ST_IDLE     },
    { "STEP1",     DEG_STEP1,  STEP1_HOLD_MS,   ST_STEP2    },
    { "STEP2",     DEG_STEP2,  STEP2_HOLD_MS,   ST_STEP3    },
    { "STEP3",     DEG_STEP3,  STEP3_HOLD_MS,   ST_RETURN   },
    { "RETURN",    DEG_HOME,   RETURN_HOLD_MS,  ST_IDLE     },
    { "FAULT",     DEG_HOME,   0U,              ST_FAULT    }
};

static State_t  g_state      = ST_IDLE;
static uint32_t g_state_ms   = 0U;      /* 进入当前状态的时刻 */
static uint16_t g_servo_deg  = DEG_HOME;/* 舵机"现在"在几度 */
static uint16_t g_target_deg = DEG_HOME;/* 舵机"要去"几度 */
static uint8_t  g_servo_on   = 1U;      /* 0 = 撤掉 PWM（舵机松力） */
static uint8_t  g_arrived    = 0U;      /* 目标角度是否已经走到了 */
static uint32_t g_arrived_ms = 0U;      /* 走到的时刻，停多久从这儿算 */
static uint32_t g_sweep_ms   = 0U;      /* 下一次扫动的时刻 */

static const char *State_Name(State_t s)
{
    if ((int)s >= 0 && (int)s < (int)ST_COUNT)
    {
        return kPlan[s].name;
    }
    return "?";
}

/* --------------------------------------------------------------------------
 *  进状态：登记"目标角度"和"下一个是谁"。真正的扫动交给 Servo_Task()。
 * ------------------------------------------------------------------------ */
static void State_Enter(State_t s, uint32_t now)
{
    if ((int)s < 0 || (int)s >= (int)ST_COUNT)
    {
        s = ST_FAULT;
    }

    g_state      = s;
    g_state_ms   = now;
    g_arrived    = 0U;
    g_arrived_ms = 0U;
    g_target_deg = kPlan[s].angle;

    if (s == ST_FAULT)
    {
        /* 急停：干脆不输出信号，舵机不受力（比"硬停在某个角度"安全） */
        g_servo_on = 0U;
        Servo_WritePulse(0U);
    }
    else
    {
        g_servo_on = 1U;
        /* 如果目标角度和当前一样，立刻补一次输出，保证 PWM 一定是开着的 */
        if (g_servo_deg == g_target_deg)
        {
            Servo_SetAngle(g_servo_deg);
        }
    }
}

/* --------------------------------------------------------------------------
 *  舵机任务：每 SERVO_TICK_MS 把"现在角度"朝"目标角度"挪一小步。
 *  这个函数在主循环里被反复调用，每次只做一点点事，不会堵住别人。
 * ------------------------------------------------------------------------ */
static void Servo_Task(uint32_t now)
{
    if (g_servo_on == 0U)
    {
        return;                     /* FAULT 态：不给信号 */
    }

    if (g_servo_deg == g_target_deg)
    {
        if (g_arrived == 0U)
        {
            g_arrived    = 1U;
            g_arrived_ms = now;     /* 刚到位：从这一刻开始算"停多久" */
        }
        return;
    }

    if (now < g_sweep_ms)
    {
        return;                     /* 还没到下一小步的时间 */
    }
    g_sweep_ms = now + SERVO_TICK_MS;

    if (g_servo_deg < g_target_deg)
    {
        g_servo_deg = (uint16_t)(g_servo_deg + SERVO_STEP_DEG);
        if (g_servo_deg > g_target_deg) { g_servo_deg = g_target_deg; }
    }
    else
    {
        if (g_servo_deg >= SERVO_STEP_DEG)
        {
            g_servo_deg = (uint16_t)(g_servo_deg - SERVO_STEP_DEG);
        }
        else
        {
            g_servo_deg = 0U;
        }
        if (g_servo_deg < g_target_deg) { g_servo_deg = g_target_deg; }
    }

    Servo_SetAngle(g_servo_deg);
}

/* --------------------------------------------------------------------------
 *  按键：主循环里没有 Wait_ms，所以这些键在任何状态下都立刻生效。
 * ------------------------------------------------------------------------ */
static void Handle_Key(uint8_t c, uint32_t now)
{
    switch (c)
    {
    case 'g':
    case 'G':
        State_Enter(ST_STEP1, now);
        P_Str("[key] g -> start sequence\r\n");
        break;

    case 's':
    case 'S':
        State_Enter(ST_IDLE, now);
        P_Str("[key] s -> stop, back to IDLE\r\n");
        break;

    case 'f':
    case 'F':
        State_Enter(ST_FAULT, now);
        P_Str("[key] f -> FAULT (servo released, press r)\r\n");
        break;

    case 'r':
    case 'R':
        State_Enter(ST_IDLE, now);
        P_Str("[key] r -> reset to IDLE\r\n");
        break;

    case '1':
        State_Enter(ST_STEP1, now);
        P_Str("[key] 1 -> STEP1\r\n");
        break;

    case '2':
        State_Enter(ST_STEP2, now);
        P_Str("[key] 2 -> STEP2\r\n");
        break;

    case '3':
        State_Enter(ST_STEP3, now);
        P_Str("[key] 3 -> STEP3\r\n");
        break;

    case 't':
    case 'T':
        P_Str("uptime = ");
        P_Uptime(now);
        P_Str(" s\r\n");
        break;

    case 'h':
    case 'H':
    case '?':
        P_Str("keys: g=run  s=stop  f=fault  r=reset  1/2/3=goto  t=uptime  h=help\r\n");
        break;

    default:
        break;
    }
}


/* ==========================================================================
 *  ⑦ main
 * ========================================================================== */

int main(void)
{
    uint32_t next_print_ms = 0U;
    uint32_t next_blink_ms = 0U;
    uint8_t  led_on        = 0U;

    UART_Init();
    Led_Init();
    Servo_PWM_Init();
    Tick_Init();

    P_Str("\r\n== Lesson 8: servo + state machine (TIM2 PWM on PA2) ==\r\n");
    P_Str("keys: g=run  s=stop  f=fault  r=reset  1/2/3=goto  t=uptime  h=help\r\n");

    State_Enter(ST_IDLE, g_tick_ms);

    while (1)
    {
        uint32_t now;
        uint32_t elapsed;
        uint8_t  c;

        now     = g_tick_ms;            /* 一轮只读一次"现在几点" */
        elapsed = now - g_state_ms;

        /* ---------- 心跳灯：每 500ms 翻转 PA0 ---------- */
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

        /* ---------- 舵机扫动：每次只挪一小步 ---------- */
        Servo_Task(now);

        /* ---------- 状态机：① 扫到位了 ② 停够了 才换下一个 ---------- */
        switch (g_state)
        {
        case ST_IDLE:
            /* 待机不自己走，等用户按 'g'。真项目里这里等的是"任务码来了"。 */
            break;

        case ST_STEP1:
        case ST_STEP2:
        case ST_STEP3:
        case ST_RETURN:
            if ((g_arrived != 0U) && ((now - g_arrived_ms) >= kPlan[g_state].hold_ms))
            {
                State_Enter(kPlan[g_state].next, now);
            }
            break;

        case ST_FAULT:
            /* 故障态不自己退出，只能按 'r' */
            break;

        default:
            State_Enter(ST_FAULT, now);
            break;
        }

        /* ---------- 打印 ---------- */
        if (now >= next_print_ms)
        {
            next_print_ms = now + PRINT_EVERY_MS;

            P_Str("t=");
            P_Uptime(now);
            P_Str("  state=");
            P_Str(State_Name(g_state));
            P_Str("  servo=");
            P_U32(g_servo_deg);
            P_Ch('/');
            P_U32(g_target_deg);
            if (g_servo_on == 0U)
            {
                P_Str("(off)");
            }
            P_Str("  (+");
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
