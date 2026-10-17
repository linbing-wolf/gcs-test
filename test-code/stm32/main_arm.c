/* ============================================================================
 *  STM32 机械臂两轴测试（升降 + 平推，CAN 位置模式）—— 串口按键版
 *
 *  【这是什么】
 *    参考工程 yyb_stm32 的"机械臂"是三轴联动：平推(6号) + 升降(5号) + 云台(舵机)。
 *    我们车上这两台型号跟轮子一样（Emm42/X42S 一体驱动），但 CAN 地址是
 *    **5 和 7**（6 号是空的，别照抄源码里的 6）。
 *    这一版只做**位置模式单独测试**：按一下走一小段固定的毫米数，用来
 *      ① 认出哪台是升降、哪台是平推；
 *      ② 标定每毫米多少脉冲（pulse/mm）和方向。
 *    不做云台、不做爪子、不联动 —— 就为了把这两个轴的参数摸出来。
 *
 *  【怎么用】
 *    1. 两台电机并在和轮子同一条 CAN 总线上（H-H、L-L），共地，电机供上电
 *    2. .\tools\switch-main.ps1 arm -Build -Download
 *    3. 串口连 115200（板载 USB UART 口，或用无线调试器），打一个字符就执行，不用回车
 *
 *  【按键表】 ★ 串口打印用英文，避免乱码 ★
 *    u / j     升降  升 / 降   一个步长（默认 5mm）
 *    f / r     平推  伸出 / 收回  一个步长
 *    s         两台立即停（Stop now，保险键）
 *    + / -     步长 ±5mm（5 ~ 50）
 *    [ / ]     速度 -100 / +100 rpm（100 ~ 2000）
 *    0         账本归零（只清软件记账，**不动电机**）
 *    p         打印当前账本（两轴各在哪、步长、速度）
 *    e / d     两台使能 / 失能
 *    h 或 ?    重新打印按键表
 *
 *  【第一次上板怎么试（重要）】
 *    下面参数区里的"哪台是哪个轴""方向""pulse/mm"都是**照源码猜的**，
 *    第一次一定不准，所以每一步都给得很小（默认 5mm），发现不对马上按 s：
 *      a) 先按 u 一下  -> 看哪台动、往哪个方向动、大概走了多远
 *      b) 再按 f 一下  -> 看另一台
 *      c) 升降/平推认反了：把参数区 LIFT 和 PUSH 两块（地址+ppm+行程）整体对调
 *      d) 方向反了：把 LIFT_DIR_INV / PUSH_DIR_INV 改成 1（别的不用动）
 *      e) 走了多远对不上：量实际值，改 PPM_xxx（新值 = 旧值 × 名义mm ÷ 实测mm）
 *    没把握就先按 s，再断电 —— 撞限位最容易烧的是电机堵转发热。
 *
 *  【接线（跟轮子同一套）】
 *    VP230 绿端子 H ── 轮子四台 + 机械臂两台的 H 全接一起
 *    VP230 绿端子 L ── 同上，L 全接一起
 *    电机电源正/负 ── 和轮子共用那一路，**负极要接回核心板 GND（共地）**
 * ========================================================================== */

#include "stm32f4xx.h"
#include "Emm_V5.h"


/* ==========================================================================
 *  ① 参数区
 * ========================================================================== */

/* 【地址映射以 10/14 用户现场确认为准】机上 5 号 = 升降，7 号 = 平推。
 *   （早前看照片记的是反的：5=水平齿条、7=竖直同步带，已作废。）
 *   若实测 u/j 走的是水平方向，说明又反了：把下面两组的 地址 + ppm + 行程
 *   一起换过去，别只换地址。 */

/* ---- 升降（上下） ---- */
#define ADDR_LIFT        5          /* CAN 地址（实车确认：5 = 升降） */
#define PPM_LIFT         80.0f      /* 源码 shengjiang_control：80 pulse/mm */
#define LIFT_MIN_MM      0.0f       /* 源码量程 0 ~ 135mm */
#define LIFT_MAX_MM      135.0f
#define LIFT_DIR_INV     0          /* 填 1 就把 u/j 的物理方向对调 */

/* ---- 平推（前后伸出） ---- */
#define ADDR_PUSH        7          /* CAN 地址（实车确认：7 = 平推） */
#define PPM_PUSH         25.465f    /* 源码 pingtui_control：25.465 pulse/mm */
#define PUSH_MIN_MM      (-122.0f)  /* 源码量程 -122 ~ +65mm */
#define PUSH_MAX_MM      65.0f
#define PUSH_DIR_INV     0

/* ---- 运动参数 ---- */
#define MOVE_SPEED       500        /* 起步速度，先慢，确认机构不卡滞再往上加 */
#define MOVE_SPEED_MIN   100
#define MOVE_SPEED_MAX   2000
#define MOVE_SPEED_STEP  100
#define MOVE_ACC         200        /* 0~255，越大起停越柔和 */

#define STEP_DEFAULT     5          /* 一次按键走多少毫米 */
#define STEP_STEP        5
#define STEP_MIN         5
#define STEP_MAX         50

#define UART_BAUD        115200


/* ==========================================================================
 *  ② 串口（USART3：PB10 = TX，PB11 = RX）
 *  ★ 2026-10-14 改正：原来是 USART1（PA9/PA10），但无线调试器实际接在集成板
 *    U4 的 2/3 脚 = PB11/PB10 = USART3，所以老版本会"下载完串口一声不吭"。
 *    跟 main_lap.c / main_armclaw.c 保持一致。
 * ========================================================================== */
static void MyUART_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

    USART_InitStructure.USART_BaudRate            = UART_BAUD;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART3, &USART_InitStructure);

    USART_Cmd(USART3, ENABLE);
}

static void UART_SendByte(uint8_t b)
{
    USART_SendData(USART3, b);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

static void USART_SendString(char *s)
{
    while (*s != '\0') { UART_SendByte((uint8_t)*s); s++; }
}

static void UART_SendInt(int32_t n)
{
    char     buf[12];
    uint8_t  i = 0;
    uint32_t u;

    if (n < 0) { UART_SendByte('-'); u = (uint32_t)(-n); }
    else       { u = (uint32_t)n; }

    if (u == 0) { UART_SendByte('0'); return; }

    while (u > 0) { buf[i++] = (char)('0' + (u % 10)); u = u / 10; }
    while (i > 0) { UART_SendByte((uint8_t)buf[--i]); }
}

/* 打印 mm，保留 1 位小数（放大 10 倍算，不依赖浮点打印库） */
static void UART_SendMm(float mm)
{
    int32_t x10;

    if (mm < 0.0f) { x10 = (int32_t)(mm * 10.0f - 0.5f); }
    else           { x10 = (int32_t)(mm * 10.0f + 0.5f); }

    UART_SendInt(x10 / 10);
    UART_SendByte('.');
    UART_SendInt((x10 % 10 < 0) ? -(x10 % 10) : (x10 % 10));
    USART_SendString("mm");
}

static uint8_t UART_GetByte(void)
{
    if (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET)
        return (uint8_t)USART_ReceiveData(USART3);
    return 0;
}


/* ==========================================================================
 *  ③ 两轴状态 + 动作
 * ========================================================================== */

static float    g_lift_mm = 0.0f;              /* 软件账本：升降现在在哪（mm） */
static float    g_push_mm = 0.0f;              /* 软件账本：平推现在在哪（mm） */
static uint16_t g_speed   = MOVE_SPEED;
static uint16_t g_step    = STEP_DEFAULT;

static void Show_State(void)
{
    USART_SendString("   now: lift ");
    UART_SendMm(g_lift_mm);
    USART_SendString(" (addr ");
    UART_SendInt(ADDR_LIFT);
    USART_SendString(")   push ");
    UART_SendMm(g_push_mm);
    USART_SendString(" (addr ");
    UART_SendInt(ADDR_PUSH);
    USART_SendString(")   step ");
    UART_SendMm((float)g_step);
    USART_SendString("   speed ");
    UART_SendInt(g_speed);
    USART_SendString(" rpm\r\n");
}

static void Show_Menu(void)
{
    USART_SendString("\r\n--- arm test: keys ---\r\n");
    USART_SendString("u / j : lift  up / down      (one step)\r\n");
    USART_SendString("f / r : push  out / retract  (one step)\r\n");
    USART_SendString("s     : stop BOTH now\r\n");
    USART_SendString("+ / - : step +-5mm     [ / ] : speed -+100 rpm\r\n");
    USART_SendString("0     : zero the book (no move)   p : print now\r\n");
    USART_SendString("e / d : enable / disable both     h : menu\r\n");
    Show_State();
}

/* 走 delta 毫米：正数 = 名义正方向。超程会被 clamp。返回实际走的毫米数 */
static float Axis_Move(uint8_t addr, float ppm, float *cur, float lo, float hi,
                       uint8_t dir_inv, float delta)
{
    float    target = *cur + delta;
    float    d;
    float    ad;
    uint32_t pulses;
    uint8_t  dir;

    if (target < lo) { target = lo; }
    if (target > hi) { target = hi; }
    if (target == *cur) { return 0.0f; }

    d  = target - *cur;
    ad = (d < 0.0f) ? -d : d;

    pulses = (uint32_t)(ad * ppm + 0.5f);
    if (pulses == 0) { return 0.0f; }

    dir = (d > 0.0f) ? 0 : 1;
    if (dir_inv) { dir = (uint8_t)(dir ? 0 : 1); }

    Emm_V5_Pos_Control(addr, dir, g_speed, MOVE_ACC, pulses, false, false);

    *cur = target;      /* 软件记账 —— 和源码一样，不是读编码器回来的 */
    return d;
}

static void Do_Lift(int8_t sign)
{
    float moved = Axis_Move(ADDR_LIFT, PPM_LIFT, &g_lift_mm,
                            LIFT_MIN_MM, LIFT_MAX_MM, LIFT_DIR_INV,
                            (float)sign * (float)g_step);

    USART_SendString("lift  ");
    UART_SendMm(moved);
    USART_SendString(" -> ");
    UART_SendMm(g_lift_mm);
    USART_SendString("  (addr ");
    UART_SendInt(ADDR_LIFT);
    USART_SendString(")\r\n");
    Show_State();
}

static void Do_Push(int8_t sign)
{
    float moved = Axis_Move(ADDR_PUSH, PPM_PUSH, &g_push_mm,
                            PUSH_MIN_MM, PUSH_MAX_MM, PUSH_DIR_INV,
                            (float)sign * (float)g_step);

    USART_SendString("push  ");
    UART_SendMm(moved);
    USART_SendString(" -> ");
    UART_SendMm(g_push_mm);
    USART_SendString("  (addr ");
    UART_SendInt(ADDR_PUSH);
    USART_SendString(")\r\n");
    Show_State();
}

static void Step_Change(int16_t delta)
{
    int16_t s = (int16_t)g_step + delta;

    if (s < STEP_MIN) { s = STEP_MIN; }
    if (s > STEP_MAX) { s = STEP_MAX; }
    g_step = (uint16_t)s;

    USART_SendString("step = ");
    UART_SendMm((float)g_step);
    USART_SendString("\r\n");
    Show_State();
}

static void Speed_Change(int16_t delta)
{
    int16_t s = (int16_t)g_speed + delta;

    if (s < MOVE_SPEED_MIN) { s = MOVE_SPEED_MIN; }
    if (s > MOVE_SPEED_MAX) { s = MOVE_SPEED_MAX; }
    g_speed = (uint16_t)s;

    USART_SendString("speed = ");
    UART_SendInt(g_speed);
    USART_SendString(" rpm (next move uses it)\r\n");
    Show_State();
}

static void Stop_Both(void)
{
    Emm_V5_Stop_Now(ADDR_LIFT, false);
    Emm_V5_Stop_Now(ADDR_PUSH, false);
    USART_SendString("STOP both\r\n");
    Show_State();
}

static void En_Both(bool state)
{
    Emm_V5_En_Control(ADDR_LIFT, state, false);
    Emm_V5_En_Control(ADDR_PUSH, state, false);

    USART_SendString(state ? "enable" : "disable");
    USART_SendString(" both\r\n");
    Show_State();
}

static void Zero_Book(void)
{
    g_lift_mm = 0.0f;
    g_push_mm = 0.0f;
    USART_SendString("book zeroed (motors NOT moved)\r\n");
    Show_State();
}


/* ==========================================================================
 *  ④ 按键处理
 * ========================================================================== */
static void Do_Command(uint8_t c)
{
    switch (c)
    {
        case 'u': case 'U': Do_Lift(+1);                     break;
        case 'j': case 'J': Do_Lift(-1);                     break;
        case 'f': case 'F': Do_Push(+1);                     break;
        case 'r': case 'R': Do_Push(-1);                     break;

        case 's': case 'S': Stop_Both();                     break;

        case '+': case '=': Step_Change(+STEP_STEP);         break;
        case '-': case '_': Step_Change(-STEP_STEP);         break;

        case '[': case '{': Speed_Change(-MOVE_SPEED_STEP);  break;
        case ']': case '}': Speed_Change(+MOVE_SPEED_STEP);  break;

        case '0':           Zero_Book();                     break;
        case 'p': case 'P': USART_SendString("print:\r\n"); Show_State(); break;

        case 'e': case 'E': En_Both(true);                   break;
        case 'd': case 'D': En_Both(false);                  break;

        case 'h': case 'H': case '?': Show_Menu();           break;

        /* 回车/换行直接忽略：有些串口助手会自动追加 \r\n */
        case '\r': case '\n':                                break;

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
    MyUART_Init();
    board_init();       /* 初始化 CAN1：PA11 / PA12，500K，扩展帧 */

    USART_SendString("\r\n== arm (lift + push, CAN pos mode) ==\r\n");
    USART_SendString("addr: lift = ");
    UART_SendInt(ADDR_LIFT);
    USART_SendString(", push = ");
    UART_SendInt(ADDR_PUSH);
    USART_SendString("\r\n");
    USART_SendString("first time? press 'u' then 'f', watch WHICH axis moves\r\n");

    /* 上电就使能两台：闭环步进使能后保持力矩，机械臂不会自己往下掉 */
    En_Both(true);
    Show_Menu();

    while (1)
    {
        uint8_t c = UART_GetByte();
        if (c != 0) { Do_Command(c); }
    }
}
