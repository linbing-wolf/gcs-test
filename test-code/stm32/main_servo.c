/* ============================================================================
 *  STM32 舵机 + 串口 测试程序（第 3 课 + 第 4 课）
 *
 *  【怎么用】
 *    1. F7 编译，F8 下载
 *    2. 舵机会按"① 参数区"里那三个角度来回摆，每个停一会儿
 *    3. 想改角度 → 只改"① 参数区"里的那三个数，别的代码一行都不用动
 *
 *  【接线】
 *    舵机 橙（信号）→ 核心板 PA2
 *    舵机 红（电源）→ 电源模块 5V（绿色座）
 *    舵机 棕（地）  → 电源模块 GND，另外再引一根线到核心板 GND（共地！）
 * ========================================================================== */

#include "stm32f4xx.h"


/* ==========================================================================
 *  ① 参数区 —— 你平时要改的东西全在这里
 * ========================================================================== */

/* 舵机的总行程角度：
 *     180  → 180° 舵机
 *     270  → 270° 舵机（常见的大扭矩舵机多半是这个）
 *     360  → 360° 连续旋转舵机（那种不会停在角度上，会一直转） */
#define SERVO_TRAVEL_DEG        270

/* 信号线插在哪个脚上（三个脚都已经初始化好了，改这里就行）：
 *     2 → PA1     3 → PA2  ← 我们现在用这个     4 → PA3 */
#define SERVO_CH                3

/* ★★★ 三个测试位置，单位是"度" ★★★
 * 想改角度，就改下面这三个数字。别的什么都不用动。
 * 建议范围 15 ~ 255：两端留点余量，免得舵机顶到机械极限"嗡嗡"响。 */
static const uint16_t ANGLE_TABLE[3] = {
     20,     /* 第 1 个位置 */
    135,     /* 第 2 个位置 */
    250,     /* 第 3 个位置 */
};

/* 每个位置停多久。数字越大停得越久（大约每 500 万 ≈ 1 秒） */
#define HOLD_TICKS              5000000

/* 串口波特率（要和电脑那头一致） */
#define UART_BAUD               115200


/* ==========================================================================
 *  ② 驱动代码 —— 正常情况下你不需要动下面这些
 * ========================================================================== */

/* --------------------------------------------------------------------------
 * 2.1  舵机 PWM 初始化
 *      TIM2 的 CH2 / CH3 / CH4 → PA1 / PA2 / PA3
 *      频率 50Hz（周期 20ms），定时器的一格 = 1 微秒
 * ------------------------------------------------------------------------ */
static void Servo_PWM_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;

    /* 开时钟：TIM2 在 APB1，GPIOA 在 AHB1 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* PA1 / PA2 / PA3 配成复用模式 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 复用给 TIM2 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);

    /* 时基：84MHz ÷ 84 = 1MHz（一格 1 微秒），数 20000 格 = 20ms = 50Hz */
    TIM_InternalClockConfig(TIM2);

    TIM_TimeBaseInitStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Prescaler         = 84 - 1;
    TIM_TimeBaseInitStructure.TIM_Period            = 20000 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    /* 三个输出比较通道，PWM 模式 1 */
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

/* --------------------------------------------------------------------------
 * 2.2  把"脉宽"写进定时器
 *      根据 SERVO_CH 自动选通道，所以你换脚位只要改上面那个宏
 * ------------------------------------------------------------------------ */
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

/* --------------------------------------------------------------------------
 * 2.3  把"角度"换算成脉宽
 *      规则：0° → 0.5ms，满行程 → 2.5ms，中间线性插值。
 *      因为定时器一格正好 1 微秒，所以算出来的数直接就是脉宽的微秒数。
 * ------------------------------------------------------------------------ */
static void Servo_SetAngle(uint16_t angle)
{
    uint32_t pulse;

    if (angle > SERVO_TRAVEL_DEG) { angle = SERVO_TRAVEL_DEG; }

    pulse = 500 + ((uint32_t)angle * 2000) / SERVO_TRAVEL_DEG;

    Servo_WritePulse((uint16_t)pulse);
}

/* --------------------------------------------------------------------------
 * 2.4  串口（USART1：PA9 = TX，PA10 = RX）
 * ------------------------------------------------------------------------ */
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

/* 把一个整数以十进制打印出来（0~4294967295） */
static void UART_SendInt(uint32_t n)
{
    char    buf[12];
    uint8_t i = 0;

    if (n == 0) { UART_SendByte('0'); return; }

    while (n > 0) { buf[i++] = (char)('0' + (n % 10)); n = n / 10; }
    while (i > 0) { UART_SendByte((uint8_t)buf[--i]); }
}

/* 收一个字节：有就返回它，没有就返回 0 */
static uint8_t UART_GetByte(void)
{
    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
        return (uint8_t)USART_ReceiveData(USART1);
    return 0;
}

/* --------------------------------------------------------------------------
 * 2.5  往串口报告一次"现在转到几度"
 * ------------------------------------------------------------------------ */
static void ReportAngle(uint16_t angle)
{
    UART_SendString("servo(PA");
    UART_SendInt(SERVO_CH - 1);          /* 通道 3 就是 PA2 */
    UART_SendString(") -> ");
    UART_SendInt(angle);
    UART_SendString(" deg\r\n");
}


/* ==========================================================================
 *  ③ 主程序
 * ========================================================================== */
int main(void)
{
    uint8_t  idx  = 0;      /* 现在在第几个位置 */
    uint32_t tick = 0;      /* 计时用的圈数 */

    Servo_PWM_Init();
    MyUART_Init();

    UART_SendString("\r\n== servo + uart ready ==\r\n");

    /* 先摆到第一个位置 */
    Servo_SetAngle(ANGLE_TABLE[0]);
    ReportAngle(ANGLE_TABLE[0]);

    while (1)
    {
        /* ---------- 串口：收到什么就回什么 ---------- */
        uint8_t c = UART_GetByte();
        if (c != 0)
        {
            UART_SendString("you typed: ");
            UART_SendByte(c);
            UART_SendString("\r\n");
        }

        /* ---------- 数圈计时，到点就换下一个角度 ---------- */
        tick++;
        if (tick >= HOLD_TICKS)
        {
            tick = 0;
            idx  = (uint8_t)((idx + 1) % 3);

            Servo_SetAngle(ANGLE_TABLE[idx]);
            ReportAngle(ANGLE_TABLE[idx]);
        }
    }
}
