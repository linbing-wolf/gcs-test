/* ============================================================================
 *  STM32 单独控制 4 个 X42S 电机（CAN）—— 串口按键版
 *
 *  【和 motor4 的区别】
 *    motor4    = 四台永远一起动（只能一起）
 *    motor4ctl = 先"选中"几号电机，再单独命令它 —— 四台各转各的
 *                （各自记住自己的方向和速度，所以可以让 1 号正转、2 号反转同时跑）
 *
 *  【怎么用】
 *    1. 接线和 motor4 一样（四台并联在一条 CAN 总线上，共地）
 *    2. .\tools\switch-main.ps1 motor4ctl -Build -Download
 *    3. 电脑上开串口工具连 COM7（115200），在"发送"框里打一个字符就执行，不用回车
 *
 *  【按键表】  ★ 串口打印用英文，是为了不在串口里变成乱码，含义看这里 ★
 *    1 2 3 4   选中 1~4 号电机（只是选中，不动）
 *    w         选中的那台 正转（CW）
 *    x         选中的那台 反转（CCW）
 *    s         选中的那台 停
 *    + / -     速度 ±10 rpm（10 ~ 300），正在转的电机会立刻跟着变速
 *    e / d     使能 / 失能 选中的那台
 *    z         四台全部停（保险键）
 *    h 或 ?    重新打印按键表
 *
 *  【接线（和 motor4 一样）】
 *    VP230 绿端子 H ── 四台的 H 全接一起
 *    VP230 绿端子 L ── 四台的 L 全接一起
 *    12V 正 ── 四台 V+ 全接一起
 *    12V 负 ── 四台 Gnd 全接一起，并且要接回核心板 GND（共地）
 * ========================================================================== */

#include "stm32f4xx.h"
#include "Emm_V5.h"


/* ==========================================================================
 *  ① 参数区
 * ========================================================================== */

/* 四台电机的 CAN 地址，顺序就是串口上按 1 2 3 4 对应的电机 */
static const uint8_t g_addr[4] = { 1, 2, 3, 4 };
#define MOTOR_NUM               4

/* 上电默认速度，单位 RPM */
#define SPEED_DEFAULT           50
#define SPEED_STEP              10
#define SPEED_MIN               10
#define SPEED_MAX               300

/* 加速度，0 ~ 255。0 表示直接启动（很猛），给大一点的数更柔和 */
#define TEST_ACC                200

#define UART_BAUD               115200

/* 0xFF = 这台现在是停的；0 = 正转；1 = 反转 */
#define DIR_STOP                0xFF


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

static void USART_SendString(char *s)
{
    while (*s != '\0') { UART_SendByte((uint8_t)*s); s++; }
}

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


/* ==========================================================================
 *  ③ 电机状态与动作
 * ========================================================================== */

static uint8_t  g_cur   = 1;                    /* 当前选中的电机地址 */
static uint16_t g_speed = SPEED_DEFAULT;        /* 当前速度 rpm */
static uint8_t  g_run[MOTOR_NUM];               /* 每台现在在干嘛（DIR_STOP / 0 / 1） */

/* 打印 "   now: motor 2  CW 50 rpm" —— 每次动作后都打一次，方便对账 */
static void Show_State(void)
{
    USART_SendString("      now: motor ");
    UART_SendInt(g_cur);
    USART_SendString("   speed ");
    UART_SendInt(g_speed);
    USART_SendString(" rpm   [1]");
    UART_SendInt(g_run[0] == DIR_STOP ? 0 : (g_run[0] == 0 ? 1 : 2));
    USART_SendString(" [2]");
    UART_SendInt(g_run[1] == DIR_STOP ? 0 : (g_run[1] == 0 ? 1 : 2));
    USART_SendString(" [3]");
    UART_SendInt(g_run[2] == DIR_STOP ? 0 : (g_run[2] == 0 ? 1 : 2));
    USART_SendString(" [4]");
    UART_SendInt(g_run[3] == DIR_STOP ? 0 : (g_run[3] == 0 ? 1 : 2));
    USART_SendString("\r\n");
}

static void Show_Menu(void)
{
    USART_SendString("\r\n--- keys ---\r\n");
    USART_SendString("1 2 3 4 : select motor (now ");
    UART_SendInt(g_cur);
    USART_SendString(")\r\n");
    USART_SendString("w = CW   x = CCW   s = stop\r\n");
    USART_SendString("+ / -   : speed +-10 (now ");
    UART_SendInt(g_speed);
    USART_SendString(" rpm)\r\n");
    USART_SendString("e / d   : enable / disable selected\r\n");
    USART_SendString("z = stop ALL     h = this menu\r\n");
    Show_State();
}

/* 第 [1] 0 是"1 号现在停"，1 = 正转，2 = 反转，只是给串口看的记号 */
static void Motor_Run(uint8_t addr, uint8_t dir)
{
    if (addr < 1 || addr > MOTOR_NUM) { return; }

    Emm_V5_Vel_Control(addr, dir, g_speed, TEST_ACC, false);
    g_run[addr - 1] = dir;

    USART_SendString("motor ");
    UART_SendInt(addr);
    USART_SendString(dir ? "  CCW " : "  CW  ");
    UART_SendInt(g_speed);
    USART_SendString(" rpm\r\n");
    Show_State();
}

static void Motor_Stop(uint8_t addr)
{
    if (addr < 1 || addr > MOTOR_NUM) { return; }

    Emm_V5_Vel_Control(addr, 0, 0, TEST_ACC, false);
    g_run[addr - 1] = DIR_STOP;

    USART_SendString("motor ");
    UART_SendInt(addr);
    USART_SendString("  stop\r\n");
    Show_State();
}

static void Motor_Stop_All(void)
{
    uint8_t i;

    for (i = 0; i < MOTOR_NUM; i++)
    {
        Emm_V5_Vel_Control(g_addr[i], 0, 0, TEST_ACC, false);
        g_run[i] = DIR_STOP;
    }

    USART_SendString("ALL stop\r\n");
    Show_State();
}

/* 改速度：正在转的电机立刻跟着变 */
static void Speed_Change(int16_t delta)
{
    uint8_t i;
    int16_t s = (int16_t)g_speed + delta;

    if (s < SPEED_MIN) { s = SPEED_MIN; }
    if (s > SPEED_MAX) { s = SPEED_MAX; }
    g_speed = (uint16_t)s;

    for (i = 0; i < MOTOR_NUM; i++)
        if (g_run[i] != DIR_STOP)
            Emm_V5_Vel_Control(g_addr[i], g_run[i], g_speed, TEST_ACC, false);

    USART_SendString("speed = ");
    UART_SendInt(g_speed);
    USART_SendString(" rpm\r\n");
    Show_State();
}


/* ==========================================================================
 *  ④ 按键处理
 * ========================================================================== */
static void Do_Command(uint8_t c)
{
    switch (c)
    {
        case '1': case '2': case '3': case '4':
            g_cur = (uint8_t)(c - '0');
            USART_SendString("select motor ");
            UART_SendInt(g_cur);
            USART_SendString("\r\n");
            Show_State();
            break;

        case 'w': case 'W': Motor_Run(g_cur, 0);           break;
        case 'x': case 'X': Motor_Run(g_cur, 1);           break;
        case 's': case 'S': Motor_Stop(g_cur);             break;

        case '+': case '=': Speed_Change(+SPEED_STEP);     break;
        case '-': case '_': Speed_Change(-SPEED_STEP);     break;

        case 'e': case 'E':
            Emm_V5_En_Control(g_cur, true, false);
            USART_SendString("enable  motor ");
            UART_SendInt(g_cur);
            USART_SendString("\r\n");
            Show_State();
            break;

        case 'd': case 'D':
            Emm_V5_En_Control(g_cur, false, false);
            g_run[g_cur - 1] = DIR_STOP;
            USART_SendString("disable motor ");
            UART_SendInt(g_cur);
            USART_SendString("\r\n");
            Show_State();
            break;

        case 'z': case 'Z': Motor_Stop_All();              break;
        case 'h': case 'H': case '?': Show_Menu();         break;

        /* 回车/换行直接忽略：有些串口助手会自动追加 \r\n，不用它报错 */
        case '\r': case '\n':                              break;

        default:
            USART_SendString("?  (h = help)\r\n");
            break;
    }
}


/* ==========================================================================
 *  ⑤ 主程序
 * ========================================================================== */
int main(void)
{
    uint8_t i;

    MyUART_Init();
    board_init();       /* 初始化 CAN1：PA11 / PA12，500K，扩展帧 */

    for (i = 0; i < MOTOR_NUM; i++) { g_run[i] = DIR_STOP; }

    USART_SendString("\r\n== motor4ctl ==\r\n");
    USART_SendString("send one char, no Enter needed\r\n");

    /* 上电先把四台都使能，这样按 w/x 就能动 */
    for (i = 0; i < MOTOR_NUM; i++)
        Emm_V5_En_Control(g_addr[i], true, false);

    Show_Menu();

    while (1)
    {
        uint8_t c = UART_GetByte();
        if (c != 0) { Do_Command(c); }
    }
}
