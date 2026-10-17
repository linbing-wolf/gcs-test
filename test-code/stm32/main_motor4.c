/* ============================================================================
 *  STM32 同时驱动 4 个 X42S 电机（CAN）—— 测试程序
 *
 *  【和单电机版（motor）的区别，就一处】
 *    单电机版只跟 1 个地址说话；这版按下面"地址表"给 1 / 2 / 3 / 4 各发一遍，
 *    所以两种情况下都能一起转：
 *      · 四台地址都还是出厂默认 1  → 收到"1 号"那条指令，四台一起动
 *      · 已经用上位机改成 1/2/3/4  → 每人收到自己那条，四台一起动
 *    （**只能"一起动"，还不能各自单独转**。要独立控制，必须先把地址改成 1~4。）
 *
 *  【接线：四台是"并联 + 共地"，绝对不是串联！】
 *
 *    VP230 绿端子 H ──┬── 电机1 H ── 电机2 H ── 电机3 H ── 电机4 H
 *    VP230 绿端子 L ──┴── 电机1 L ── 电机2 L ── 电机3 L ── 电机4 L
 *
 *    12V 正极 ── 四台的 V+ 全接在一起
 *    12V 负极 ── 四台的 Gnd 全接在一起，并且还要接到核心板 GND（共地！）
 *
 *  【⚠ 三个坑】
 *    1. CAN_H 和 CAN_L 绝不能插进面包板的同一个"竖条"（那是短路）；
 *       H 接 H、L 接 L，两条线各自单独一条竖条
 *    2. 四台电机一起跑，电流可能 3~6A —— 别让这么大的电流走面包板
 *       （面包板触点约 1A，会发热、压降，电机就抖/丢步/复位）
 *    3. 12V 绝对不能碰到 CAN 线；VP230 的 VCC 只能是 3.3V
 *
 *  【怎么用】
 *    1. 接好线（先接 2 台试也行），再上电
 *    2. .\tools\switch-main.ps1 motor4 -Build -Download
 *    3. 看串口：每 1 秒左右打印一行，四台应该一起 正转 → 停 → 反转 → 停
 * ========================================================================== */

#include "stm32f4xx.h"
#include "Emm_V5.h"     /* 张大头电机协议 —— 工程里 can_ZDT 目录已经带了 */


/* ==========================================================================
 *  ① 参数区 —— 平时要改的就这几个数
 * ========================================================================== */

/* 四台电机的 CAN 地址。出厂默认都是 1，要独立控制就用上位机改成 1/2/3/4 */
static const uint8_t g_addr[4] = { 1, 2, 3, 4 };
#define MOTOR_NUM               4

/* 速度，单位是 RPM（0 ~ 5000）。
 * ★ 四台一起上，先用 30 ~ 50 试，确认电源扛得住、方向一致，再加 ★ */
#define TEST_SPEED              50

/* 加速度，0 ~ 255。0 表示直接启动（很猛），给大一点的数更柔和 */
#define TEST_ACC                200

/* 每个动作停多久（大约每 500 万 ≈ 1 秒） */
#define STEP_TICKS              8000000

/* 串口波特率 */
#define UART_BAUD               115200

/* 0 = 直接发指令（最简单，先跑通这个）
 * 1 = 同步帧模式：四台先各自收好指令，等一条广播再"同时起步"，
 *     比赛时四轮要同一瞬间启动就用这个 */
#define USE_SYNC                0


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

/* 打印这次指令发给了哪些地址，例：  "addr 1 2 3 4" */
static void UART_SendAddrList(void)
{
    uint8_t i;

    UART_SendString("addr");
    for (i = 0; i < MOTOR_NUM; i++)
    {
        UART_SendByte(' ');
        UART_SendInt(g_addr[i]);
    }
}


/* --------------------------------------------------------------------------
 * 2.2  CAN 电机控制
 *      真正干活的是工程里 can_ZDT 目录的张大头驱动：
 *        board_init()              → 初始化 CAN1（PA11/PA12，500K，扩展帧）
 *        Emm_V5_En_Control()       → 使能电机
 *        Emm_V5_Vel_Control()      → 按速度转
 *        Emm_V5_Synchronous_motion() → 广播"同时起步"（地址 0 = 所有电机）
 * ------------------------------------------------------------------------ */

/* 使能全部电机（有些控制器不发这一步就不动） */
static void Motor_Enable_All(void)
{
    uint8_t i;

    for (i = 0; i < MOTOR_NUM; i++)
        Emm_V5_En_Control(g_addr[i], true, false);

    UART_SendString("enable  ");
    UART_SendAddrList();
    UART_SendString("\r\n");
}

/* 让全部电机按速度转。dir：0 = 正转（CW），1 = 反转（CCW） */
static void Motor_Run_All(uint8_t dir, uint16_t rpm)
{
    uint8_t i;

    for (i = 0; i < MOTOR_NUM; i++)
        Emm_V5_Vel_Control(g_addr[i], dir, rpm, TEST_ACC,
                           (USE_SYNC != 0) ? true : false);

#if (USE_SYNC != 0)
    Emm_V5_Synchronous_motion(0);       /* 广播：四台同时起步 */
#endif

    UART_SendString(dir ? "CCW " : "CW  ");
    UART_SendString(" ");
    UART_SendInt(rpm);
    UART_SendString(" rpm  ");
    UART_SendAddrList();
    UART_SendString("\r\n");
}

/* 让全部电机停（速度给 0，会按加速度平滑减速，不是急停） */
static void Motor_Stop_All(void)
{
    uint8_t i;

    for (i = 0; i < MOTOR_NUM; i++)
        Emm_V5_Vel_Control(g_addr[i], 0, 0, TEST_ACC,
                           (USE_SYNC != 0) ? true : false);

#if (USE_SYNC != 0)
    Emm_V5_Synchronous_motion(0);
#endif

    UART_SendString("stop      ");
    UART_SendAddrList();
    UART_SendString("\r\n");
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
        case 0:  Motor_Run_All(0, TEST_SPEED);  break;   /* 四台一起正转 */
        case 1:  Motor_Stop_All();              break;
        case 2:  Motor_Run_All(1, TEST_SPEED);  break;   /* 四台一起反转 */
        default: Motor_Stop_All();              break;
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

    UART_SendString("\r\n== motor4 test ==\r\n");
    UART_SendString("motors = ");
    UART_SendInt(MOTOR_NUM);
    UART_SendString("   speed = ");
    UART_SendInt(TEST_SPEED);
    UART_SendString(" rpm   sync = ");
    UART_SendInt(USE_SYNC);
    UART_SendString("\r\n");
    UART_SendAddrList();
    UART_SendString("\r\n");

    Motor_Enable_All();

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
