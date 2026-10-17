/* ============================================================================
 *  STM32 走位层（麦轮底盘 / 4 个 X42S 闭环步进）—— 开机自动"前进一小段就停"版
 *
 *  换上去：.\tools\switch-main.ps1 step -Build -Download
 *
 *  【和 move 版的区别，只有这一处】
 *    上电/复位后不用碰串口，程序自己跑一遍：
 *      使能四轮 -> 等 0.5s -> 前进 STEP_MM 毫米 -> 停住（电机保持使能，锁在原位）
 *    跑完之后串口按键全部照旧（f/b/l/r、1 2 3 4、s 急停、p 参数…），
 *    另外多一个 g 键 = "再走一小段"，方便反复试。
 *    ⚠ 第一次先把车架起来（轮子离地）确认方向，方向对了再落地。
 *
 *  【这个程序干什么】
 *    把原项目 can_ZDT\yyb_move.c 的麦轮走位算法搬过来，但改成能在串口上现场调的：
 *      一趟走多少毫米、走多快  —— 串口上现场调
 *      脉冲/毫米 两个标定系数   —— 串口上现场调（我们自己的车必须重新标）
 *      哪个方向不对，只改文件顶上 4 个 MAP_* 宏，算法一行都不用动
 *
 *  【麦轮逆运动学 —— 和原项目 car_move() 一行不差，只是写成了表格】
 *      speed[1] = +v_x + v_y - w   -> 电机 2，速度为正时 dir = 0
 *      speed[2] = -v_x + v_y + w   -> 电机 1，速度为正时 dir = 1
 *      speed[3] = -v_x + v_y - w   -> 电机 3，速度为正时 dir = 0
 *      speed[4] = +v_x + v_y + w   -> 电机 4，速度为正时 dir = 1
 *
 *    v_y 四项全同号（下面叫"Y 家族"）-> 一个方向的平移
 *    v_x 对角两组反号（"X 家族"）-> 另一个方向的平移
 *    w   两项反号 -> 原地自转（今晚不测）
 *    地址 1、4 的 dir 和 2、3 相反（电机镜像装）—— 抄代码最容易错的就是这里。
 *
 *  【位置模式怎么走】
 *    Emm_V5_Pos_Control(addr, dir, 速度rpm, 加速度, clk=脉冲数, false=相对, true=等同步)
 *    四轮都发完，最后 Emm_V5_Synchronous_motion(0x00) 广播一声，四轮同时起步；
 *    驱动器自己数脉冲、自己停 —— 所以"走 200mm" = 200 x 脉冲/毫米。
 *    发完指令主程序用梯形速度曲线的解析解估一个时间在等（和原项目
 *    car_move_delay() 是同一个公式），等的时候按 s 能立刻停。
 *
 *  【按键表】串口打印全用英文，是为了不在串口里变乱码，含义看这里
 *    f / b     前进 / 后退   g_dist 毫米（位置模式，走到自己停）
 *    l / r     左移 / 右移   g_dist 毫米
 *    w / x     一直前进 / 一直后退（速度模式，按 s 才停）
 *    s / z     立刻全停（走的过程中也能按，保险键）
 *    + / -     距离 +-50 mm（50 ~ 2000）
 *    , / .     速度 -20 / +20 rpm（20 ~ 300）—— 正在动的话立刻跟着变
 *    [ / ]     X 家族脉冲/mm -0.1 / +0.1
 *    { / }     Y 家族脉冲/mm -0.1 / +0.1
 *    1 2 3 4   单轮测试：只让 1~4 号电机走 g_dist mm（查地址、查轮子转向）
 *    e / d     四轮使能 / 失能
 *    p         打印现在所有参数（含"这一趟要发多少脉冲、大概等多久"）
 *    h / ?     重新打印按键表
 *
 *  【命令回传】每按一个键，串口先把收到的那个字符原样打出来，再接这条命令的结果：
 *    [cmd] 'f' -> [move] FWD  ...
 *    看到 [cmd] 那行就说明"这个键被认出来了"；按了没反应，先看有没有 [cmd]。
 *    （回车、换行不回传，免得串口助手自动追加的 \r\n 刷屏）
 *
 *  【接线（和 motor4 一样）】
 *    VP230 绿端子 H -- 四台的 H 接一起
 *    VP230 绿端子 L -- 四台的 L 接一起
 *    12V 正 -- 四台 V+ 接一起；12V 负 -- 四台 Gnd 接一起，并接回核心板 GND
 * ========================================================================== */

#include "stm32f4xx.h"
#include "Emm_V5.h"     /* 张大头协议；里面已经 include 了 board.h / can.h */
#include "delay.h"      /* delay_init() / delay_ms() —— SysTick 软件延时 */
#include <math.h>       /* sqrtf()，估等待时间用 */


/* ==========================================================================
 *  1 参数区
 * ========================================================================== */

#define MOTOR_NUM        4

/* 四台电机的 CAN 地址，顺序 = 串口上按 1 2 3 4 对应的那台 */
static const uint8_t g_addr[MOTOR_NUM] = { 1, 2, 3, 4 };

/* ---- 一趟走多少、走多快（串口上现场可调） ---- */
#define DIST_DEFAULT     200        /* 一次走多少毫米 */
#define DIST_STEP        50
#define DIST_MIN         50
#define DIST_MAX         2000

/* ---- 本版本专有：开机会自动走的那"一小段" ---- */
#define STEP_MM          50         /* 开机自动前进多少毫米（想更小就改这个数） */
#define STEP_RPM         60         /* 这一小段用多快（比默认 100 慢，更安全） */
#define STEP_SETTLE_MS   500        /* 使能后先等一会儿再起步，让驱动器准备好 */

#define SPEED_DEFAULT    100        /* 位置模式速度，单位是电机转速 RPM */
#define SPEED_STEP       20
#define SPEED_MIN        20
#define SPEED_MAX        300

/* 加速度档位 0~255（0 = 不经过加减速直接全速，很猛）。
 * 这个数既发给驱动器，也拿去估"这一趟要走多久" */
#define ACC_DEFAULT      150

#define WAIT_MIN_MS      200        /* 估出来的等待时间最少给 200ms */

#define UART_BAUD        115200

/* ---- 脉冲/毫米：我们车上最重要的两个标定数 ----
 * 原项目的车是 x 方向 10.5、y 方向 10.3（y 方向打滑多所以略小）。
 * 我们的轮子直径/减速比不一定一样 —— 第一次上车必须自己量：
 *   把 g_dist 调到 1000 -> 走一趟 -> 量实际走了多少毫米
 *   新系数 = 老系数 x 1000 / 实测量
 * 串口上用 [ ] 调 X 家族、用 { } 调 Y 家族，一次 0.1。
 */
static float g_ppm_x = 10.5f;       /* X 家族（对角两组反号） */
static float g_ppm_y = 10.3f;       /* Y 家族（四轮同号） */
#define PPM_STEP         0.1f

/* ---- 方向映射：第一次试的时候哪个键方向不对，只改这 4 个数 ----
 *   AXIS：0 = 走 X 家族，1 = 走 Y 家族
 *   SIGN：+1 / -1
 * 标准装法的麦轮，"四轮同号"那一族才是前进 —— 所以先按 AXIS=1 / SIGN=+1 试。
 * 按 f 车往后走 -> 把 MAP_FWD_SIGN 改成 (-1)；
 * 按 f 车横着走 -> 说明两个家族反了，把 AXIS 的 0 / 1 对调。
 */
#define MAP_FWD_AXIS     1
#define MAP_FWD_SIGN     (-1)       /* 10/7 实测：原来是 (+1)，上电后整车往后走 -> 取反 */
#define MAP_LEFT_AXIS    0
#define MAP_LEFT_SIGN    (+1)

/* 连续转（速度模式）的状态：0 = 没在连续转，1 = 一直前进，2 = 一直后退 */
#define JOG_STOP         0
#define JOG_FWD          1
#define JOG_BACK         2


/* ==========================================================================
 *  2 调试串口（USART1：PA9 = TX，PA10 = RX）
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

static void USART_SendString(const char *s)
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

/* 打印一位小数：10.5 -> "10.5"（不引 printf，省得链一大堆库） */
static void UART_SendFix1(float v)
{
    uint32_t x;

    if (v < 0.0f) { UART_SendByte('-'); v = -v; }
    x = (uint32_t)(v * 10.0f + 0.5f);
    UART_SendInt(x / 10);
    UART_SendByte('.');
    UART_SendByte((uint8_t)('0' + (x % 10)));
}

static uint8_t UART_GetByte(void)
{
    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
        return (uint8_t)USART_ReceiveData(USART1);
    return 0;
}


/* ==========================================================================
 *  3 轮子表 + 走位算法
 * ========================================================================== */

/* 每一台电机在麦轮公式里的三个系数，以及"正速度对应哪个 dir 位" */
typedef struct {
    uint8_t addr;       /* CAN 地址 */
    int8_t  sx;         /* v_x 的系数 */
    int8_t  sy;         /* v_y 的系数 */
    int8_t  sw;         /* w   的系数 */
    uint8_t dir_pos;    /* 速度为正时发给驱动器的 dir（0 或 1） */
} Wheel_t;

/* 表的顺序跟 g_addr[] 一样：{1, 2, 3, 4}。值直接从 yyb_move.c 的 car_move() 抄的
 *   speed[2] = -vx + vy + w  -> 地址 1，正速度 dir = 1
 *   speed[1] = +vx + vy - w  -> 地址 2，正速度 dir = 0
 *   speed[3] = -vx + vy - w  -> 地址 3，正速度 dir = 0
 *   speed[4] = +vx + vy + w  -> 地址 4，正速度 dir = 1
 */
static const Wheel_t g_wheel[MOTOR_NUM] = {
    { 1, -1, +1, +1, 1 },
    { 2, +1, +1, -1, 0 },
    { 3, -1, +1, -1, 0 },
    { 4, +1, +1, +1, 1 },
};

static uint16_t g_dist  = DIST_DEFAULT;     /* 一趟走多少毫米 */
static uint16_t g_speed = SPEED_DEFAULT;    /* 速度 rpm */
static uint8_t  g_jog   = JOG_STOP;         /* 是不是在连续转 */

/* 一台电机：给个转速 s（可为负），翻成 dir + 绝对值发出去 */
static void Wheel_Vel(const Wheel_t *w, int s)
{
    uint8_t  dir;
    uint16_t vel;

    if (s >= 0) { dir = w->dir_pos;         vel = (uint16_t)s; }
    else        { dir = (w->dir_pos ? 0 : 1); vel = (uint16_t)(-s); }

    if (vel > SPEED_MAX) { vel = SPEED_MAX; }
    Emm_V5_Vel_Control(w->addr, dir, vel, ACC_DEFAULT, false);
}

/* 速度模式：把 (v_x, v_y, w) 变成四轮转速。抄的是 car_move() */
static void Mv_Vel(int v_x, int v_y, int w)
{
    uint8_t i;

    for (i = 0; i < MOTOR_NUM; i++)
    {
        int s = g_wheel[i].sx * v_x + g_wheel[i].sy * v_y + g_wheel[i].sw * w;
        Wheel_Vel(&g_wheel[i], s);
        delay_ms(1);        /* CAN 只有 3 个发送邮箱，连发会丢帧 */
    }
}

/* 速度模式，只沿一族走：family 0 = X 家族，1 = Y 家族 */
static void Mv_Vel_Axis(uint8_t family, int8_t sign, uint16_t speed)
{
    if (family == 0) { Mv_Vel(sign * (int)speed, 0, 0); }
    else             { Mv_Vel(0, sign * (int)speed, 0); }
}

/* 估计"这一趟大概要走多久"，单位 ms。和原项目 car_move_delay() 一个公式：梯形速度曲线
 *   加减速段走掉的脉冲 = 53.33 x v^2 / a   （1 RPM 约 53.33 脉冲/秒）
 *   加减速段花的时间   = 2000 x v / a      （单位 ms）
 *   路近（脉冲不够加减速段）-> 三角形 t = t_acc * sqrt(q / q_acc)
 *   路远                    -> 三角形 + 匀速段
 */
static uint32_t Mv_EstimateMs(uint32_t pulses, uint16_t acc, uint16_t speed_rpm)
{
    float q, q_acc, t_acc, t;

    if (acc == 0)       { acc = 256; }      /* 0 档最猛，按 256 估 */
    if (speed_rpm == 0) { speed_rpm = 1; }

    q     = (float)pulses;
    q_acc = 53.3333f * (float)speed_rpm * (float)speed_rpm / (float)acc;
    t_acc = 2000.0f * (float)speed_rpm / (float)acc;

    if (q_acc >= q) { t = t_acc * sqrtf(q / q_acc); }
    else            { t = 1000.0f * (q - q_acc) / (53.3333f * (float)speed_rpm) + t_acc; }

    if (t < (float)WAIT_MIN_MS) { t = (float)WAIT_MIN_MS; }
    return (uint32_t)t;
}

/* 位置模式：沿 family 那一族走 distance_mm 毫米（正负 = 方向）。返回发出去的脉冲数 */
static uint32_t Mv_Pos(uint8_t family, float distance_mm)
{
    float    ppm = (family == 0) ? g_ppm_x : g_ppm_y;
    float    pf  = ppm * ((distance_mm < 0.0f) ? -distance_mm : distance_mm);
    int      sign;
    uint32_t pulses;
    uint8_t  i;

    sign = (distance_mm < 0.0f) ? -1 : +1;
    if (pf < 1.0f) { pf = 1.0f; }
    pulses = (uint32_t)(pf + 0.5f);

    for (i = 0; i < MOTOR_NUM; i++)
    {
        int     k   = (family == 0) ? g_wheel[i].sx : g_wheel[i].sy;
        uint8_t dir;

        k *= sign;
        dir = (k >= 0) ? g_wheel[i].dir_pos : (g_wheel[i].dir_pos ? 0 : 1);

        /* false = 相对运动（走这段就停），true = 先憋着等同步广播 */
        Emm_V5_Pos_Control(g_wheel[i].addr, dir, g_speed, ACC_DEFAULT, pulses, false, true);
        delay_ms(1);
    }
    Emm_V5_Synchronous_motion(0x00);    /* 广播：四轮同时起步 */

    return pulses;
}

static void Mv_Stop_All(void)
{
    uint8_t i;

    for (i = 0; i < MOTOR_NUM; i++)
    {
        Emm_V5_Stop_Now(g_wheel[i].addr, false);        /* 立刻停 */
        Emm_V5_Vel_Control(g_wheel[i].addr, 0, 0, ACC_DEFAULT, false);
        delay_ms(1);
    }
    g_jog = JOG_STOP;
}

/* 等这一趟走完。等的过程中每 10ms 摸一下串口，按 s / z 立刻停
 * 返回 1 = 正常走完，0 = 被 s 打断 */
static uint8_t Mv_Wait(uint32_t ms)
{
    uint32_t t = 0;

    while (t < ms)
    {
        uint8_t c;

        delay_ms(10);
        t += 10;
        if ((t % 500) == 0) { UART_SendByte('.'); }     /* 每 0.5s 一个点，看得到进度 */

        c = UART_GetByte();
        if (c == 's' || c == 'S' || c == 'z' || c == 'Z')
        {
            Mv_Stop_All();
            return 0;
        }
    }
    return 1;
}

/* 走一趟：算脉冲 -> 发指令 -> 估时间 -> 等 -> 报告结果 */
static void Mv_MoveDir(uint8_t family, int8_t sign, const char *name)
{
    uint32_t pulses = Mv_Pos(family, (float)g_dist * (float)sign);
    uint32_t ms     = Mv_EstimateMs(pulses, ACC_DEFAULT, g_speed);

    USART_SendString("[move] ");
    USART_SendString(name);
    USART_SendString("  ");
    UART_SendInt(g_dist);
    USART_SendString(" mm  ");
    USART_SendString(family ? "Y-family" : "X-family");
    USART_SendString((sign > 0) ? " +  " : " -  ");
    USART_SendString("pulses=");
    UART_SendInt(pulses);
    USART_SendString("  speed=");
    UART_SendInt(g_speed);
    USART_SendString("rpm  wait~");
    UART_SendInt(ms);
    USART_SendString("ms\r\n");

    if (Mv_Wait(ms)) { USART_SendString(" done\r\n"); }
    else             { USART_SendString(" interrupted\r\n"); }
}

/* 单轮测试：只让第 idx 台（0~3）走一趟。用来查地址、查轮子转向 */
static void Mv_OneWheel(uint8_t idx)
{
    uint32_t pulses = (uint32_t)(g_ppm_y * (float)g_dist + 0.5f);
    uint32_t ms     = Mv_EstimateMs(pulses, ACC_DEFAULT, g_speed);

    USART_SendString("[single] motor ");
    UART_SendInt(g_wheel[idx].addr);
    USART_SendString("  dir_pos=");
    UART_SendInt(g_wheel[idx].dir_pos);
    USART_SendString("  pulses=");
    UART_SendInt(pulses);
    USART_SendString("  wait~");
    UART_SendInt(ms);
    USART_SendString("ms\r\n");
    USART_SendString("  (if 4 wheels move together -> all motors still on addr 1)\r\n");

    Emm_V5_Pos_Control(g_wheel[idx].addr, g_wheel[idx].dir_pos, g_speed,
                       ACC_DEFAULT, pulses, false, true);
    Emm_V5_Synchronous_motion(0x00);

    if (Mv_Wait(ms)) { USART_SendString(" done\r\n"); }
    else             { USART_SendString(" interrupted\r\n"); }
}

/* 连续转：速度模式，一直转到按 s */
static void Jog_Apply(void)
{
    if (g_jog == JOG_FWD)       { Mv_Vel_Axis(MAP_FWD_AXIS, (int8_t)(MAP_FWD_SIGN), g_speed); }
    else if (g_jog == JOG_BACK) { Mv_Vel_Axis(MAP_FWD_AXIS, (int8_t)(-MAP_FWD_SIGN), g_speed); }
}


/* ==========================================================================
 *  4 菜单 / 参数打印
 * ========================================================================== */

static void Show_Menu(void)
{
    USART_SendString("\r\n--- keys (send one char, no Enter needed) ---\r\n");
    USART_SendString("f / b   : forward / backward   ");
    UART_SendInt(g_dist);
    USART_SendString(" mm\r\n");
    USART_SendString("l / r   : left / right         ");
    UART_SendInt(g_dist);
    USART_SendString(" mm\r\n");
    USART_SendString("w / x   : keep going fwd / back (velocity mode, stop with s)\r\n");
    USART_SendString("s / z   : STOP ALL now  (works in the middle of a move)\r\n");
    USART_SendString("+ / -   : dist +-50 mm       , / . : speed -20/+20 rpm\r\n");
    USART_SendString("[ / ]   : ppm_x -0.1/+0.1   { / } : ppm_y -0.1/+0.1\r\n");
    USART_SendString("1 2 3 4 : single motor test (addr check)\r\n");
    USART_SendString("g       : go one more small step (step build only)\r\n");
    USART_SendString("e / d   : enable / disable all    p : params    h : menu\r\n");
    USART_SendString("every key is echoed first: [cmd] 'x' ->  ...\r\n");
}

static void Show_Params(void)
{
    uint32_t pulses = (uint32_t)(g_ppm_y * (float)g_dist + 0.5f);

    USART_SendString("\r\n--- params ---\r\n");
    USART_SendString("dist  = ");
    UART_SendInt(g_dist);
    USART_SendString(" mm\r\n");
    USART_SendString("speed = ");
    UART_SendInt(g_speed);
    USART_SendString(" rpm\r\n");
    USART_SendString("acc   = ");
    UART_SendInt(ACC_DEFAULT);
    USART_SendString("  (position mode, 0-255)\r\n");
    USART_SendString("ppm_x = ");
    UART_SendFix1(g_ppm_x);
    USART_SendString("  ppm_y = ");
    UART_SendFix1(g_ppm_y);
    USART_SendString("\r\n");
    USART_SendString("fwd   = ");
    USART_SendString(MAP_FWD_AXIS ? "Y-family" : "X-family");
    USART_SendString((MAP_FWD_SIGN > 0) ? " +" : " -");
    USART_SendString("   left = ");
    USART_SendString(MAP_LEFT_AXIS ? "Y-family" : "X-family");
    USART_SendString((MAP_LEFT_SIGN > 0) ? " +\r\n" : " -\r\n");
    USART_SendString("now   : ");
    UART_SendInt(g_dist);
    USART_SendString(" mm -> ");
    UART_SendInt(pulses);
    USART_SendString(" pulses (Y-family), wait ~");
    UART_SendInt(Mv_EstimateMs(pulses, ACC_DEFAULT, g_speed));
    USART_SendString(" ms\r\n");
    USART_SendString("calib : ppm_new = ppm_old * dist / real_dist\r\n");
    USART_SendString("jog   : ");
    if (g_jog == JOG_STOP)      { USART_SendString("stopped\r\n"); }
    else if (g_jog == JOG_FWD)  { USART_SendString("forward\r\n"); }
    else                        { USART_SendString("backward\r\n"); }
}


/* ==========================================================================
 *  5 按键处理
 * ========================================================================== */

static void Do_Command(uint8_t c)
{
    /* 回车换行：既不回传也不处理（有些串口助手发字符时会自动追加 \r\n，回传会刷屏） */
    if (c == '\r' || c == '\n') { return; }

    /* 命令回传：先把收到的字符原样打出来，再执行。
     * 这样"我发出去的键到底被认出没有"一眼就能核对：
     * 打了字却连 [cmd] 都没出现 -> 串口没通 / 波特率不对（不是按键问题） */
    USART_SendString("[cmd] '");
    UART_SendByte(c);
    USART_SendString("' -> ");

    switch (c)
    {
        case 'f': case 'F': Mv_MoveDir(MAP_FWD_AXIS,  MAP_FWD_SIGN, "FWD");  break;
        case 'b': case 'B': Mv_MoveDir(MAP_FWD_AXIS,  (int8_t)(-MAP_FWD_SIGN), "BACK"); break;
        case 'l': case 'L': Mv_MoveDir(MAP_LEFT_AXIS, MAP_LEFT_SIGN, "LEFT"); break;
        case 'r': case 'R': Mv_MoveDir(MAP_LEFT_AXIS, (int8_t)(-MAP_LEFT_SIGN), "RIGHT"); break;

        case 'w': case 'W':
            g_jog = JOG_FWD;
            Jog_Apply();
            USART_SendString("jog: forward ");
            UART_SendInt(g_speed);
            USART_SendString(" rpm  (press s to stop)\r\n");
            break;

        case 'x': case 'X':
            g_jog = JOG_BACK;
            Jog_Apply();
            USART_SendString("jog: backward ");
            UART_SendInt(g_speed);
            USART_SendString(" rpm  (press s to stop)\r\n");
            break;

        case 's': case 'S': case 'z': case 'Z':
            Mv_Stop_All();
            USART_SendString("ALL stop (Stop_Now + vel 0)\r\n");
            break;

        case '+': case '=':
            if ((int)g_dist + DIST_STEP > DIST_MAX) { g_dist = DIST_MAX; }
            else { g_dist = (uint16_t)(g_dist + DIST_STEP); }
            USART_SendString("dist = ");
            UART_SendInt(g_dist);
            USART_SendString(" mm\r\n");
            break;

        case '-': case '_':
            if ((int)g_dist - DIST_STEP < DIST_MIN) { g_dist = DIST_MIN; }
            else { g_dist = (uint16_t)(g_dist - DIST_STEP); }
            USART_SendString("dist = ");
            UART_SendInt(g_dist);
            USART_SendString(" mm\r\n");
            break;

        case ',': case '<':
            if ((int)g_speed - SPEED_STEP < SPEED_MIN) { g_speed = SPEED_MIN; }
            else { g_speed = (uint16_t)(g_speed - SPEED_STEP); }
            USART_SendString("speed = ");
            UART_SendInt(g_speed);
            USART_SendString(" rpm\r\n");
            if (g_jog != JOG_STOP) { Jog_Apply(); }     /* 正在连续转就立刻变速 */
            break;

        case '.': case '>':
            if ((int)g_speed + SPEED_STEP > SPEED_MAX) { g_speed = SPEED_MAX; }
            else { g_speed = (uint16_t)(g_speed + SPEED_STEP); }
            USART_SendString("speed = ");
            UART_SendInt(g_speed);
            USART_SendString(" rpm\r\n");
            if (g_jog != JOG_STOP) { Jog_Apply(); }
            break;

        case '[':
            g_ppm_x -= PPM_STEP;
            if (g_ppm_x < 1.0f) { g_ppm_x = 1.0f; }
            USART_SendString("ppm_x = ");
            UART_SendFix1(g_ppm_x);
            USART_SendString("\r\n");
            break;

        case ']':
            g_ppm_x += PPM_STEP;
            USART_SendString("ppm_x = ");
            UART_SendFix1(g_ppm_x);
            USART_SendString("\r\n");
            break;

        case '{':
            g_ppm_y -= PPM_STEP;
            if (g_ppm_y < 1.0f) { g_ppm_y = 1.0f; }
            USART_SendString("ppm_y = ");
            UART_SendFix1(g_ppm_y);
            USART_SendString("\r\n");
            break;

        case '}':
            g_ppm_y += PPM_STEP;
            USART_SendString("ppm_y = ");
            UART_SendFix1(g_ppm_y);
            USART_SendString("\r\n");
            break;

        case '1': case '2': case '3': case '4':
            Mv_OneWheel((uint8_t)(c - '1'));
            break;

        case 'e': case 'E':
        {
            uint8_t i;
            for (i = 0; i < MOTOR_NUM; i++)
            {
                Emm_V5_En_Control(g_wheel[i].addr, true, false);
                delay_ms(1);
            }
            USART_SendString("all motors ENABLED\r\n");
            break;
        }

        case 'd': case 'D':
        {
            uint8_t i;
            Mv_Stop_All();
            for (i = 0; i < MOTOR_NUM; i++)
            {
                Emm_V5_En_Control(g_wheel[i].addr, false, false);
                delay_ms(1);
            }
            USART_SendString("all motors DISABLED\r\n");
            break;
        }

        case 'p': case 'P': Show_Params(); break;
        case 'h': case 'H': case '?': Show_Menu(); break;

        /* 本版本专有：再走一小段（走完立刻停，电机仍使能锁着） */
        case 'g': case 'G':
            Mv_MoveDir(MAP_FWD_AXIS, MAP_FWD_SIGN, "STEP");
            Mv_Stop_All();
            USART_SendString("stopped (motors still ENABLED)\r\n");
            break;

        default:
            USART_SendString("?  (h = help)\r\n");
            break;
    }
}


/* ==========================================================================
 *  6 主程序
 * ========================================================================== */

int main(void)
{
    uint8_t i;

    delay_init(168);    /* 延迟函数必须先初始化（主频 168MHz） */
    MyUART_Init();
    board_init();       /* 初始化 CAN1：PA11 / PA12，500K，扩展帧 */

    USART_SendString("\r\n== step (auto: forward a little, then stop) ==\r\n");
    USART_SendString("LIFT THE CAR (wheels off the ground) for the first test!\r\n");

    /* 本版本：开机就用这个很小的距离和较慢的速度，不接串口也能跑 */
    g_dist  = STEP_MM;
    g_speed = STEP_RPM;

    /* 上电先把四台都使能，这样按 f 就能动 */
    for (i = 0; i < MOTOR_NUM; i++)
    {
        Emm_V5_En_Control(g_addr[i], true, false);
        delay_ms(1);
    }

    g_jog = JOG_STOP;
    Show_Menu();
    Show_Params();

    /* ---- 开机自动跑的那一小段 ---- */
    USART_SendString("\r\nauto step: forward ");
    UART_SendInt(STEP_MM);
    USART_SendString(" mm, then stop\r\n");
    delay_ms(STEP_SETTLE_MS);
    Mv_MoveDir(MAP_FWD_AXIS, MAP_FWD_SIGN, "FWD");
    Mv_Stop_All();      /* 走完再明确停一下，保证一点残余速度都不留 */
    USART_SendString("stopped. motors are still ENABLED (holding in place).\r\n");
    USART_SendString("press g = one more ");
    UART_SendInt(STEP_MM);
    USART_SendString(" mm,  s = stop now,  h = menu\r\n");

    while (1)
    {
        uint8_t c = UART_GetByte();
        if (c != 0) { Do_Command(c); }
    }
}
