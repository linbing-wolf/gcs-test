/* ============================================================================
 *  STM32 测试程序：舵机 PWM + 串口 + CAN 电机
 *
 *  【怎么用】
 *    1. 看下面"① 参数区"，把 TEST_MODE 改成你想测的东西
 *    2. F7 编译 → F8 下载
 *    3. 打开串口助手（COM9 / 115200），屏幕上有每一步在干什么
 *
 *  【⚠️ 前提：STM32 发 CAN 需要外接一个 CAN 收发器】
 *    STM32 芯片里只有 CAN 控制器，出来的是 TTL 电平，
 *    必须外加一颗收发器（TJA1050 / SN65HVD230 之类）才能变成 CAN_H / CAN_L。
 *    集成板上有这个电路，但板子还没打样出来。
 *    → TEST_MODE=2 的代码现在能编译，但要等有收发器才能真的发出去。
 *    → 在那之前，"让电机转"请走 USB 转 CAN 模块 + 张大头上位机那条路。
 *
 *  【接线】
 *    舵机：橙 → 核心板 PA2；红 → 电源模块 5V；棕 → 电源模块 GND（并引一根到核心板 GND）
 *    CAN ：PA11 = CAN_RX，PA12 = CAN_TX → CAN 收发器 → CAN_H / CAN_L → 电机
 * ========================================================================== */

#include "stm32f4xx.h"
#include "Emm_V5.h"     /* 张大头电机协议 —— 工程里 can_ZDT 目录已经带了 */


/* ==========================================================================
 *  ① 参数区 —— 你平时要改的东西全在这里
 * ========================================================================== */

/* ★ 开机跑哪个测试：
 *       1 = 只跑舵机（现在用这个）
 *       2 = 只跑电机（要有 CAN 收发器）
 *       3 = 两个一起跑                                            */
#define TEST_MODE               1


/* ------------------------- 舵机相关 ------------------------- */

/* 舵机的总行程角度：180 → 180° 舵机；270 → 270° 舵机 */
#define SERVO_TRAVEL_DEG        270

/* 信号线插在哪个脚：2 → PA1；3 → PA2；4 → PA3 */
#define SERVO_CH                3

/* ★★★ 三个测试位置，单位是"度"。想改角度就改这三个数 ★★★
 * 建议范围 15 ~ 255：两端留点余量，免得舵机顶到机械极限"嗡嗡"响。 */
static const uint16_t ANGLE_TABLE[3] = {
     20,     /* 第 1 个位置 */
    135,     /* 第 2 个位置 */
    250,     /* 第 3 个位置 */
};


/* ------------------------- 电机相关 ------------------------- */

/* ★ 要测的电机地址。出厂默认一般是 1，用上位机可以改 ★
 * 先把一个电机测通，再扩到四个（四个的地址不能重复）。 */
#define MOTOR_ADDR              1

/* 速度，单位是 RPM（0 ~ 5000）。
 * ★ 第一次试先给 30 ~ 50，确认方向对了再往上加 ★ */
#define TEST_SPEED              100

/* 加速度，0 ~ 255。0 表示直接启动（很猛），给大一点的数更柔和 */
#define TEST_ACC                200


/* --------------------------- 公共 --------------------------- */

/* 每个动作停多久（大约每 500 万 ≈ 1 秒） */
#define STEP_TICKS              8000000

/* 串口波特率 */
#define UART_BAUD               115200


/* ==========================================================================
 *  ② 驱动代码 —— 正常情况下你不需要动下面这些
 * ========================================================================== */

/* --------------------------------------------------------------------------
 * 2.1  串口（USART1：PA9 = TX，PA10 = RX）
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

/* 把整数以十进制打印出来 */
static void UART_SendInt(uint32_t n)
{
    char    buf[12];
    uint8_t i = 0;

    if (n == 0) { UART_SendByte('0'); return; }

    while (n > 0) { buf[i++] = (char)('0' + (n % 10)); n = n / 10; }
    while (i > 0) { UART_SendByte((uint8_t)buf[--i]); }
}

static uint8_t UART_GetByte(void)
{
    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
        return (uint8_t)USART_ReceiveData(USART1);
    return 0;
}


/* --------------------------------------------------------------------------
 * 2.2  舵机 PWM：TIM2 的 CH2/CH3/CH4 → PA1/PA2/PA3，50Hz
 *      下面这段只在 TEST_MODE 是 1 或 3 时才编译进去
 *      （这样不用到的东西不会产生编译警告）
 * ------------------------------------------------------------------------ */
#if (TEST_MODE == 1) || (TEST_MODE == 3)

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

/* 根据 SERVO_CH 把脉宽写进对应通道 */
static void Servo_WritePulse(uint16_t pulse)
{
#if   SERVO_CH == 2
    TIM_SetCompare2(TIM2, pulse);   /* 通道 2 → PA1 */
#elif SERVO_CH == 3
    TIM_SetCompare3(TIM2, pulse);   /* 通道 3 → PA2 */
#else
    TIM_SetCompare4(TIM2, pulse);   /* 通道 4 → PA3 */
#endif
}

/* 角度 → 脉宽：0° 对应 0.5ms，满行程对应 2.5ms，中间线性插值。
   定时器一格正好 1 微秒，所以算出来的数直接就是脉宽微秒数。 */
static void Servo_SetAngle(uint16_t angle)
{
    uint32_t pulse;

    if (angle > SERVO_TRAVEL_DEG) { angle = SERVO_TRAVEL_DEG; }

    pulse = 500 + ((uint32_t)angle * 2000) / SERVO_TRAVEL_DEG;

    Servo_WritePulse((uint16_t)pulse);
}

/* 每 STEP_TICKS 圈换下一个角度 */
static void Servo_Task(void)
{
    static uint8_t  idx  = 0;
    static uint32_t tick = 0;

    tick++;
    if (tick < STEP_TICKS) { return; }
    tick = 0;

    idx = (uint8_t)((idx + 1) % 3);
    Servo_SetAngle(ANGLE_TABLE[idx]);

    UART_SendString("servo(PA");
    UART_SendInt(SERVO_CH - 1);
    UART_SendString(") -> ");
    UART_SendInt(ANGLE_TABLE[idx]);
    UART_SendString(" deg\r\n");
}

#endif  /* 舵机段结束 */


/* --------------------------------------------------------------------------
 * 2.3  CAN 电机：直接用工程里 can_ZDT 目录现成的张大头驱动
 *
 *      board_init() 会初始化 CAN1：PA11 = CAN_RX，PA12 = CAN_TX，
 *      500K 波特率，扩展帧。
 *      下面这段只在 TEST_MODE 是 2 或 3 时才编译进去。
 * ------------------------------------------------------------------------ */
#if (TEST_MODE == 2) || (TEST_MODE == 3)

/* 使能电机（有些控制器不发这一步就不动） */
static void Motor_Enable(uint8_t addr)
{
    Emm_V5_En_Control(addr, true, false);

    UART_SendString("motor ");
    UART_SendInt(addr);
    UART_SendString("  enable\r\n");
}

/* 让电机按某个方向和速度转。dir：0 = 正转，1 = 反转 */
static void Motor_Run(uint8_t addr, uint8_t dir, uint16_t rpm)
{
    Emm_V5_Vel_Control(addr, dir, rpm, TEST_ACC, false);

    UART_SendString("motor ");
    UART_SendInt(addr);
    UART_SendString(dir ? "  CCW " : "  CW  ");
    UART_SendInt(rpm);
    UART_SendString(" rpm\r\n");
}

/* 让电机停下来（速度给 0，会按加速度平滑减速，不是急停） */
static void Motor_Stop(uint8_t addr)
{
    Emm_V5_Vel_Control(addr, 0, 0, TEST_ACC, false);

    UART_SendString("motor ");
    UART_SendInt(addr);
    UART_SendString("  stop\r\n");
}

/* 每 STEP_TICKS 圈推进一步：正转 → 停 → 反转 → 停 → 循环 */
static void Motor_Task(void)
{
    static uint8_t  step = 0;
    static uint32_t tick = 0;

    tick++;
    if (tick < STEP_TICKS) { return; }
    tick = 0;

    switch (step)
    {
        case 0:  Motor_Run(MOTOR_ADDR, 0, TEST_SPEED);  break;   /* 正转 */
        case 1:  Motor_Stop(MOTOR_ADDR);                break;
        case 2:  Motor_Run(MOTOR_ADDR, 1, TEST_SPEED);  break;   /* 反转 */
        default: Motor_Stop(MOTOR_ADDR);                break;
    }

    step = (uint8_t)((step + 1) % 4);
}

#endif  /* 电机段结束 */


/* ==========================================================================
 *  ③ 主程序
 * ========================================================================== */
int main(void)
{
    MyUART_Init();

#if (TEST_MODE == 2) || (TEST_MODE == 3)
    board_init();       /* 初始化 CAN1（PA11/PA12，500K，扩展帧） */
#endif

#if (TEST_MODE == 1) || (TEST_MODE == 3)
    Servo_PWM_Init();
#endif

    UART_SendString("\r\n== ready ==\r\n");
    UART_SendString("TEST_MODE = ");
    UART_SendInt(TEST_MODE);
    UART_SendString("\r\n");

#if (TEST_MODE == 1) || (TEST_MODE == 3)
    Servo_SetAngle(ANGLE_TABLE[0]);     /* 先摆到第一个位置 */
#endif

#if (TEST_MODE == 2) || (TEST_MODE == 3)
    Motor_Enable(MOTOR_ADDR);
#endif

    while (1)
    {
        /* 串口：收到什么就打回什么 */
        uint8_t c = UART_GetByte();
        if (c != 0)
        {
            UART_SendString("you typed: ");
            UART_SendByte(c);
            UART_SendString("\r\n");
        }

#if (TEST_MODE == 1) || (TEST_MODE == 3)
        Servo_Task();
#endif

#if (TEST_MODE == 2) || (TEST_MODE == 3)
        Motor_Task();
#endif
    }
}
