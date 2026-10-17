/* ============================================================================
 *  STM32 读 HWT101 陀螺仪（串口）—— 第 5 课
 *  ---------------------------------------------------------------------------
 *  目标（总计划表 10/10 那条判据）：
 *      陀螺仪插上、程序跑起来，串口能打印出航向角（yaw），手转模块时数值跟着变。
 *
 *  这一版刻意做到"最小"：只干两件事 —— 读陀螺仪、往串口打印。
 *  不碰电机、不碰舵机、不碰 CAN。
 *
 *  ---------------------------------------------------------------------------
 *  接线（核心板 -> 陀螺仪模块）：
 *      核心板 3.3V  ->  模块 VCC
 *      核心板 GND   ->  模块 GND
 *      核心板 PD6   ->  模块 TX     <-- 交叉！模块的 TX 进单片机
 *      核心板 PD5   ->  模块 RX     <-- 只读角度的话这根可以先不接
 *
 *  【注意】模块上有两排丝印，别接错：
 *      VCC / SCL / SDA / GND  = I2C 接口，本课**不用**
 *      VCC / RX  / TX  / GND  = 串口接口，用这排
 *
 *  调试串口仍是 USART1（PA9 = TX，PA10 = RX），115200，8N1。
 *
 *  ---------------------------------------------------------------------------
 *  波特率不用猜：imu\imu.c 里的 imu_init() 会调 AutoScanSensor()，
 *  把 4800 ~ 921600 挨个试一遍，哪个有回应就停在哪个（存进 realbaud）。
 *
 *  【10/6 实测】这块模块原来就是 115200、8N1、被动不了（一直主动上报），
 *  而且上报内容只有两类帧：0x52（角速度）和 0x53（角度），**没有 0x51（加速度）**。
 *  所以别拿 AZ（加速度）当心跳，用 raw gx/gy/gz 那几个整数当心跳。
 *
 *  怎么算通过：
 *    1) 串口打印的 [1][2][3] 三行都出来了，没卡死；
 *    2) 每行的 `raw gx gy gz` 不是三个 0（静止时也该有 ±几的噪声），说明帧解出来了；
 *    3) 用手水平转动模块，yaw= 的数值跟着变，停下就不动。
 *
 *  【注意】所有串口输出都写成 ASCII，不要改成中文 —— 串口助手按 ASCII 解码。
 * ========================================================================== */

#include "stm32f4xx.h"
#include "imu.h"          /* imu_init(), imu_scan() */
#include "delay.h"        /* delay_init(), delay_ms() —— imu 的扫描靠它计时 */
#include "wit_c_sdk.h"    /* sReg[]、WIT_PROTOCOL_NORMAL */
#include "REG.h"          /* AZ / Roll / Pitch / Yaw 这些寄存器编号 */


/* ==========================================================================
 *  ① 参数区
 * ========================================================================== */

#define UART_BAUD     115200   /* 调试串口波特率 */
#define PRINT_MS      200      /* 每 200 毫秒打印一行（约 5Hz，够看又不刷屏） */
#define SYS_CLOCK_MHZ 168      /* 核心板主频，延迟函数要用 */

/* imu.c 里的全局变量：扫描停在哪个波特率（没找到模块时会停在 921600） */
extern float realbaud;


/* ==========================================================================
 *  ② 调试串口（USART1：PA9 = TX，PA10 = RX）+ 打印小工具
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
 *  ③ 主函数
 * ========================================================================== */

int main(void)
{
    float    fAcc[3];
    float    fGyro[3];
    float    fAngle[3];
    uint32_t n = 0;

    UART_Init();

    P_Str("\r\n\r\n=== lesson5: HWT101 gyro on USART2 (PD5=TX, PD6=RX) ===\r\n");
    P_Str("    debug uart: USART1 PA9/PA10, 115200 8N1\r\n\r\n");

    P_Str("[1] delay_init() ... ");
    delay_init(SYS_CLOCK_MHZ);
    P_Str("ok\r\n");

    P_Str("[2] imu_init(): looking for the module at 4800..921600 bps ...\r\n");
    imu_init();
    P_Str("    scanner stopped at ");
    P_Int((int32_t)realbaud);
    P_Str(" bps  (921600 = it never answered)\r\n");

    P_Str("[3] printing every ");
    P_Int(PRINT_MS);
    P_Str("ms. Turn the module by hand: yaw must follow.\r\n");
    P_Str("    Heartbeat = 'raw gx gy gz': at rest these are tiny (+-5), never all 0.\r\n");
    P_Str("    All three stuck at 0 forever = not one byte got decoded.\r\n");
    P_Str("    (This module sends only 0x52 gyro + 0x53 angle frames, no 0x51 acc.)\r\n\r\n");

    while (1)
    {
        imu_scan(fAcc, fGyro, fAngle);

        P_Char('[');
        P_Int((int32_t)n);
        P_Str("] roll=");
        P_Fix2(fAngle[0]);
        P_Str(" pitch=");
        P_Fix2(fAngle[1]);
        P_Str(" yaw=");
        P_Fix2(fAngle[2]);
        P_Str("  gyroZ=");
        P_Fix2(fGyro[2]);
        P_Str("  raw gx gy gz=");
        P_Int((int32_t)(int16_t)sReg[GX]);
        P_Char(' ');
        P_Int((int32_t)(int16_t)sReg[GY]);
        P_Char(' ');
        P_Int((int32_t)(int16_t)sReg[GZ]);
        P_Str("\r\n");

        n++;
        delay_ms(PRINT_MS);
    }
}
