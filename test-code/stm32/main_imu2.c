/* ============================================================================
 *  STM32 读 HWT101 陀螺仪（串口）—— 第 5 课 · 进阶版 imu2
 *  ---------------------------------------------------------------------------
 *  目标：把陀螺仪从"能看到数字"变成"比赛里能用的角度计"。
 *
 *  imu 版只负责把角度打出来，有两个毛病比赛里没法用：
 *    1) 模块自己的 yaw 会慢慢漂（实测约 0.19 度/秒，一分钟漂十几度）；
 *    2) 因为会漂，没法判定"车头正前方到底是几度"。
 *
 *  这一版用**纯软件**解决，不给模块发任何指令：
 *    [a] 开机先静止 3 秒，量出这台模块自己的漂移速度 drift（度/秒）；
 *    [b] 把量完那一刻的朝向当成"车头正前方 = 0 度"，之后一直减掉 drift 造成的漂移；
 *    [c] 串口打印 rel（相对车头方向的角度）；按 z 键随时把当前朝向重新定为 0。
 *
 *  ---------------------------------------------------------------------------
 *  接线（核心板 -> 陀螺仪模块）：
 *      核心板 3.3V  ->  模块 VCC
 *      核心板 GND   ->  模块 GND
 *      核心板 PD6   ->  模块 TX     <-- 交叉！模块的 TX 进单片机
 *      核心板 PD5   ->  模块 RX     <-- 本版不用（不给模块发指令）
 *
 *  【注意】模块上有两排丝印，别接错：
 *      VCC / SCL / SDA / GND  = I2C 接口，本课**不用**
 *      VCC / RX  / TX  / GND  = 串口接口，用这排
 *
 *  调试串口仍是 USART1（PA9 = TX，PA10 = RX），115200，8N1。
 *
 *  【重要】改完陀螺仪的接线，**必须按一次板上的 RST**。
 *  因为 imu_init() 只在开机时扫描波特率，开机时线没接好就会锁错波特率，
 *  之后不管线怎么接，raw gx gy gz 永远是 0（10/7 踩过这个坑）。
 *
 *  ---------------------------------------------------------------------------
 *  怎么算通过（判据）：
 *    1) 开机那几行都打出来了，最后一行 drift= 是"每秒漂多少度"；
 *    2) 开机后**别碰模块**，盯 1 分钟：rel 的最大最小值差 < 3 度（说明漂移被压掉了）；
 *    3) 把模块原地转 90 度，rel 应该显示 90 左右（±5 度内）；转回来又回到 0 附近；
 *    4) 串口发一个 z，能看到 `ZERO`，并且 rel 立刻变成 0 左右。
 *
 *  【注意】所有串口输出都写成 ASCII，不要改成中文 —— 串口助手按 ASCII 解码。
 *  发送键：一个字符就是一条命令，不用回车。
 * ========================================================================== */

#include "stm32f4xx.h"
#include "imu.h"          /* imu_init(), imu_scan() */
#include "delay.h"        /* delay_init(), delay_ms() */
#include "wit_c_sdk.h"    /* sReg[] */
#include "REG.h"          /* GX / GY / GZ 这些寄存器编号 */


/* ==========================================================================
 *  ① 参数区
 * ========================================================================== */

#define UART_BAUD       115200  /* 调试串口波特率 */
#define SYS_CLOCK_MHZ   168     /* 核心板主频，延迟函数要用 */
#define PRINT_MS        200     /* 每 200 毫秒打印一行（约 5Hz） */
#define TICK_MS         10      /* 主循环的时间片，顺便用来收串口按键 */
#define SETTLE_MS       500     /* 测漂移前先等一会儿，让第一帧数据进来 */
#define DRIFT_WINDOW_MS 2000    /* 测漂移的窗口长度 */
#define DRIFT_LIMIT     30.0f   /* 窗口内 yaw 变化超过这个度数，就当"手在动"，不采信 */

/* imu.c 里的全局变量：扫描停在哪个波特率（没找到模块时会停在 921600） */
extern float realbaud;


/* ==========================================================================
 *  ② 状态（按键处理函数和主循环都要用，所以放成文件级全局）
 * ========================================================================== */

static float    s_yawOffset = 0.0f;     /* 车头正前方对应的模块 yaw */
static float    s_driftRate = 0.0f;     /* 模块自身漂移速度，度/秒 */
static uint32_t s_elapsedMs = 0;        /* 从"定零点"那一刻起走过的毫秒数 */
static float    s_lastYaw   = 0.0f;     /* 最近一次读到的模块 yaw */


/* ==========================================================================
 *  ③ 调试串口（USART1：PA9 = TX，PA10 = RX）+ 打印小工具
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
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

static void P_Char(char c)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) { }
    USART_SendData(USART1, (uint16_t)(uint8_t)c);
}

static void P_Str(const char *s)
{
    while (*s != '\0') { P_Char(*s); s++; }
}

/* 打印整数（不依赖 printf / microlib，省得踩库的坑） */
static void P_Int(int32_t v)
{
    char    buf[12];
    int32_t i = 0;

    if (v < 0) { P_Char('-'); v = -v; }

    if (v == 0) { buf[i] = '0'; i++; }
    while (v > 0)
    {
        buf[i] = (char)('0' + (v % 10));
        i++;
        v = v / 10;
    }
    while (i > 0) { i--; P_Char(buf[i]); }
}

/* 打印浮点，保留两位小数，比如 -12.34 */
static void P_Fix2(float f)
{
    int32_t scaled;

    if (f < 0.0f) { P_Char('-'); f = -f; }

    scaled = (int32_t)((f * 100.0f) + 0.5f);
    P_Int(scaled / 100);
    P_Char('.');
    if ((scaled % 100) < 10) { P_Char('0'); }
    P_Int(scaled % 100);
}


/* ==========================================================================
 *  ④ 角度小工具
 * ========================================================================== */

/* 把角度折到 -180 ~ +180，免得 yaw 从 179 跳到 -179 时算出个几百度的假变化 */
static float Wrap180(float a)
{
    while (a >  180.0f) { a -= 360.0f; }
    while (a < -180.0f) { a += 360.0f; }
    return a;
}

/* 当前"相对车头方向"的角度 = 模块 yaw - 零点 - 漂移补偿 */
static float RelAngle(float yaw)
{
    float sec = (float)s_elapsedMs / 1000.0f;
    return Wrap180(yaw - s_yawOffset - (s_driftRate * sec));
}


/* ==========================================================================
 *  ⑤ 串口按键（一个字符一条命令，不用回车）
 * ========================================================================== */

static void PollKey(void)
{
    char c;

    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET) { return; }

    c = (char)(USART_ReceiveData(USART1) & 0xFF);

    if (c == 'z' || c == 'Z')
    {
        /* 把"现在朝哪儿"定为新的 0 度 */
        s_yawOffset = s_lastYaw - (s_driftRate * ((float)s_elapsedMs / 1000.0f));
        P_Str("\r\nZERO: this direction is now 0 deg\r\n");
    }
    else if (c == 'h' || c == '?')
    {
        P_Str("\r\n--- keys ---\r\n");
        P_Str("  z : set the current direction as 0 deg\r\n");
        P_Str("  h : this help\r\n");
    }
    else if (c == '\r' || c == '\n')
    {
        /* 串口助手常自带"发送加换行"，忽略掉 */
    }
    else
    {
        P_Str("\r\n?  (h = help)\r\n");
    }
}


/* ==========================================================================
 *  ⑥ 主函数
 * ========================================================================== */

int main(void)
{
    float    fAcc[3];
    float    fGyro[3];
    float    fAngle[3];
    float    yawA, yawB, delta;
    uint32_t n = 0;
    int      i;

    UART_Init();

    P_Str("\r\n\r\n=== lesson5b: HWT101 gyro -> relative heading (PD6 = module TX) ===\r\n");
    P_Str("    debug uart: USART1 PA9/PA10, 115200 8N1\r\n\r\n");

    P_Str("[1] delay_init() ... ");
    delay_init(SYS_CLOCK_MHZ);
    P_Str("ok\r\n");

    P_Str("[2] imu_init(): looking for the module at 4800..921600 bps ...\r\n");
    imu_init();
    P_Str("    scanner stopped at ");
    P_Int((int32_t)realbaud);
    P_Str(" bps  (921600 = it never answered)\r\n");

    /* ---- [3] 量漂移：这 2.5 秒别碰模块 ---- */
    P_Str("[3] DO NOT MOVE the module for 3 seconds, measuring drift ...\r\n");
    delay_ms(SETTLE_MS);

    imu_scan(fAcc, fGyro, fAngle);
    yawA = fAngle[2];

    delay_ms(DRIFT_WINDOW_MS);
    imu_scan(fAcc, fGyro, fAngle);
    yawB = fAngle[2];

    delta = Wrap180(yawB - yawA);
    if ((delta > DRIFT_LIMIT) || (delta < -DRIFT_LIMIT))
    {
        /* 手在动，这数据不能用来算漂移 */
        s_driftRate = 0.0f;
        P_Str("    module was moving - drift NOT measured, using 0\r\n");
    }
    else
    {
        s_driftRate = delta / ((float)DRIFT_WINDOW_MS / 1000.0f);
    }

    P_Str("    drift = ");
    P_Fix2(s_driftRate);
    P_Str(" deg/s   (yaw ");
    P_Fix2(yawA);
    P_Str(" -> ");
    P_Fix2(yawB);
    P_Str(")\r\n");

    /* 量完这一刻的朝向 = 车头正前方 */
    s_yawOffset = yawB;
    s_elapsedMs = 0;
    s_lastYaw   = yawB;

    P_Str("[4] printing every ");
    P_Int(PRINT_MS);
    P_Str("ms.  rel = angle relative to the direction you just calibrated.\r\n");
    P_Str("    Heartbeat = 'raw': at rest these are tiny (+-5), never all 0.\r\n");
    P_Str("    All three stuck at 0 forever = not one byte got decoded\r\n");
    P_Str("    -> check module TX -> PD6, GND, 3.3V, then press RST.\r\n\r\n");

    /* ---- [5] 主循环 ---- */
    while (1)
    {
        /* 把 200ms 切成 20 个小片，每片收一次串口按键，顺便刷新陀螺仪数据 */
        for (i = 0; i < (PRINT_MS / TICK_MS); i++)
        {
            imu_scan(fAcc, fGyro, fAngle);
            delay_ms(TICK_MS);
            s_elapsedMs += TICK_MS;
            PollKey();
        }

        imu_scan(fAcc, fGyro, fAngle);   /* 打印前取最新的一帧 */
        s_lastYaw = fAngle[2];

        P_Char('[');
        P_Int((int32_t)n);
        P_Str("] rel=");
        P_Fix2(RelAngle(s_lastYaw));
        P_Str("  yaw=");
        P_Fix2(s_lastYaw);
        P_Str("  gyroZ=");
        P_Fix2(fGyro[2]);
        P_Str("  raw=");
        P_Int((int32_t)(int16_t)sReg[GX]);
        P_Char(' ');
        P_Int((int32_t)(int16_t)sReg[GY]);
        P_Char(' ');
        P_Int((int32_t)(int16_t)sReg[GZ]);
        P_Str("\r\n");

        n++;
    }
}
