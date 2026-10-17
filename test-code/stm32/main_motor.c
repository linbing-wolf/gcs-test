/* ============================================================================
 *  STM32 驱动单个 X42S 电机（CAN）—— 测试程序
 *
 *  【怎么用】
 *    1. 改下面"① 参数区"里的电机地址和速度
 *    2. F7 编译 → F8 下载
 *    3. 打开串口助手（COM9 / 115200），屏幕上会打印每一步在干什么
 *
 *  【接线】
 *    核心板 3.3V  ────  VP230 模块 VCC
 *    核心板 GND   ────  VP230 模块 GND
 *    核心板 PA12  ────  VP230 模块 TX      (CAN_TX)
 *    核心板 PA11  ────  VP230 模块 RX      (CAN_RX)
 *    VP230 的 H   ────  电机 绿线 R/AH     (CAN_H)
 *    VP230 的 L   ────  电机 蓝线 T/B/L    (CAN_L)
 *    12V 正极     ────  电机 红线 V+
 *    12V 负极     ────  电机 灰线 Gnd，并且这根还要接到核心板 GND（共地！）
 *
 *  【⚠️ 两个不能错的地方】
 *    1. VP230 是 3.3V 器件，VCC 只能接 3.3V，接 5V 会烧
 *    2. 12V 绝对不能碰到 CAN 线上
 *
 *  【如果电机不动，按这个顺序查】
 *    1. 串口有没有在打印？没有 → 程序没跑起来
 *    2. 模块 VCC 和 GND 之间是不是 3.3V？（有没有插反）
 *    3. 12V 电源的负极有没有和核心板 GND 连通？（共地）
 *    4. CAN_H / CAN_L 有没有接反？（接反不会烧，对调一下再试）
 *    5. 电机地址对不对？出厂默认一般是 1
 * ========================================================================== */

#include "stm32f4xx.h"
#include "Emm_V5.h"     /* 张大头电机协议 —— 工程里 can_ZDT 目录已经带了 */


/* ==========================================================================
 *  ① 参数区 —— 平时要改的就这几个数
 * ========================================================================== */

/* 电机的 CAN 地址。出厂默认一般是 1，可以用张大头上位机改 */
#define MOTOR_ADDR              1

/* 速度，单位是 RPM（0 ~ 5000）。
 * ★ 第一次先给 30 ~ 50，确认方向和反应，再往上加 ★ */
#define TEST_SPEED              50

/* 加速度，0 ~ 255。0 表示直接启动（很猛），给大一点的数更柔和 */
#define TEST_ACC                200

/* 每个动作停多久（大约每 500 万 ≈ 1 秒） */
#define STEP_TICKS              8000000

/* 串口波特率 */
#define UART_BAUD               115200


/* ==========================================================================
 *  ② 驱动代码 —— 正常情况下不需要动
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

/* 打印一个整数（十进制） */
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
 * 2.2  CAN 电机控制
 *      真正干活的是工程里 can_ZDT 目录的张大头驱动：
 *        board_init()              → 初始化 CAN1（PA11/PA12，500K，扩展帧）
 *        Emm_V5_En_Control()       → 使能电机
 *        Emm_V5_Vel_Control()      → 按速度转
 * ------------------------------------------------------------------------ */

/* 使能电机（有些控制器不发这一步就不动） */
static void Motor_Enable(uint8_t addr)
{
    Emm_V5_En_Control(addr, true, false);

    UART_SendString("motor ");
    UART_SendInt(addr);
    UART_SendString("  enable\r\n");
}

/* 让电机转。dir：0 = 正转（CW），1 = 反转（CCW） */
static void Motor_Run(uint8_t addr, uint8_t dir, uint16_t rpm)
{
    Emm_V5_Vel_Control(addr, dir, rpm, TEST_ACC, false);

    UART_SendString("motor ");
    UART_SendInt(addr);
    UART_SendString(dir ? "  CCW " : "  CW  ");
    UART_SendInt(rpm);
    UART_SendString(" rpm\r\n");
}

/* 让电机停（速度给 0，会按加速度平滑减速，不是急停） */
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


/* ==========================================================================
 *  ③ 主程序
 * ========================================================================== */
int main(void)
{
    MyUART_Init();
    board_init();       /* 初始化 CAN1：PA11 / PA12，500K，扩展帧 */

    UART_SendString("\r\n== motor test ==\r\n");
    UART_SendString("addr = ");
    UART_SendInt(MOTOR_ADDR);
    UART_SendString("   speed = ");
    UART_SendInt(TEST_SPEED);
    UART_SendString(" rpm\r\n");

    Motor_Enable(MOTOR_ADDR);

    while (1)
    {
        /* 串口：收到什么就打回什么（顺便当作"程序还活着"的证据） */
        uint8_t c = UART_GetByte();
        if (c != 0)
        {
            UART_SendString("you typed: ");
            UART_SendByte(c);
            UART_SendString("\r\n");
        }

        Motor_Task();
    }
}
