/* ============================================================================
 *  main_lap.c  ---  STM32 绕场地跑一整圈（路线 + 起点直接抄自导航源码）
 *
 *  换程序 / 下载：
 *      .\tools\switch-main.ps1 lap -Build -Download
 *  （tools 真身在 C:\Users\Kobe\Desktop\tools\）
 *
 *  ---------------------------------------------------------------------------
 *  这是什么
 *  ---------------------------------------------------------------------------
 *  场地 2400x2400、灰道宽 400mm。开源导航项目的场地模型把场地划成 9 个节点
 *  （A~I）+ 12 条有向路（road0~11），其中外围 8 条路正好闭合成一整圈：
 *
 *        A -> B -> C -> D -> I -> H -> G -> F -> A
 *
 *  10/13 改版：原来那张表逐字抄自 route-study\mcu-navigation\structure.h 的
 *  road_motion_table，13 段里夹着 +60/-60/-170/-30 这些 30~170mm 的短段。
 *  根源是源码把 A、C 两个拐角标记的 x 摆成 2250，比行走线中线 x=2190 靠外 60mm
 *  （源码 README 自己也标注"30/60mm 短段尤其需要实测检查"）。
 *  现在干脆不并线，直接沿四条行走线的中线跑：20~170mm 的短段全部砍掉，
 *  整圈变成 8 段 >=860mm 的长直线，麦轮打滑的相对误差小得多。
 *  整圈全部用平移实现：forward / left 两个方向，车身全程不旋转
 *  —— 麦轮横移过去就行，不用转弯，这是这条路线最省事的地方。
 *
 *  ---------------------------------------------------------------------------
 *  起步条件（必须一致，否则整圈全歪）
 *  ---------------------------------------------------------------------------
 *  四条行走线的中线：右 x=2190、下 y=200、左 x=340、上 y=2060、中 x=1200/y=1200。
 *  围成一个 1850 x 1860 的中线矩形，四个角都是真实交点（节点名沿用 A~I，位置已改）：
 *      A(2190, 200) B(2190,1200) C(2190,2060) D(1200,2060)
 *      E(1200,1200 场地正中，外圈不含)
 *      I( 340,2060) H( 340,1200) G( 340, 200) F(1200, 200)
 *  默认起点 = A：把车放在中线矩形的"右下角"(2190,200)，车头朝场地 +Y。
 *  注意：这个角比源码老 A 标记(2250,170)靠左 60mm、靠后 30mm，
 *  现场要按新坐标重新量起点。停车位置和车头方向每次都要一样。
 *  想换个角当起点：串口按 k（A-B-C-D-I-H-G-F 循环，跳过场地正中的 E），
 *  然后把车摆到那个角上，车头照样朝场地 +Y。
 *
 *  ---------------------------------------------------------------------------
 *  8 段、合计 7420 mm
 *  ---------------------------------------------------------------------------
 *     1  FWD  +1000     2  FWD   +860     3  LEFT  +990     4  LEFT  +860
 *     5  FWD   -860     6  FWD  -1000     7  LEFT  -860     8  LEFT  -990
 *
 *  每段的场地位移 dx/dy 加起来正好是 (0,0) —— 走完回到起点，闭环。
 *  符号（车头朝 +Y）：FWD + = 往场地 +Y（上），FWD - = 往下；
 *                    LEFT + = 往 -x（场地左侧），LEFT - = 往 +x（场地右侧）。
 *
 *  ---------------------------------------------------------------------------
 *  怎么算通过
 *  ---------------------------------------------------------------------------
 *  1) 串口 115200 按 RST，应打出 "== lap (field perimeter loop...)" +
 *     按键表 + 段表信息 + "pos from A = (0, 0) mm"。
 *  2) 先缩放试：按 y 键把 scale 降到 0.1（本来 1.0，按 9 次），
 *     然后按回车一段一段走，量每段实际走了多少 —— 对得上再按 u 加回 1.0。
 *     缩放只是把每段的 mm 乘一个系数，段数和方向不变。
 *  3) 按 g 自动跑整圈：串口每段打 "[seg k/13] FWD/LEFT xxx mm ... pos=(x,y)"，
 *     走完打 "LAP DONE"。实测终点应目视回到 A 点（+-50mm 以内算好）。
 *  4) 中途任何时候按 s 立刻停（Stop_Now + 速度 0），自动跑圈也会中断。
 *
 *  ---------------------------------------------------------------------------
 *  已知短板（不是程序的 bug，是这几块还没做）
 *  ---------------------------------------------------------------------------
 *  - ppm_x 还是借来的 10.5（横移长度会偏），ppm_y 10.36 是实测过 995/1000 的；
 *    横移走不准会直接拖累整圈闭环精度 —— 这是最该先标定的一个数。
 *  - 没有陀螺仪纠偏：车头靠轮子自己保持一致，打滑会让它慢慢歪，圈越大越明显。
 *  - 没有避障：场地真摆上 φ50x100 障碍柱之前，别用 g 自动跑。
 *  - 跑之前把机械臂收回到最低/缩回位（重心越低越不容易翻）。
 * ========================================================================= */

#include "Emm_V5.h"     /* 张大头协议；里面已经 include 了 board.h / can.h */
#include "delay.h"      /* delay_init() / delay_ms() */
#include <math.h>       /* sqrtf()，估等待时间用 */
#include "imu.h"        /* imu_init() / imu_scan()：WT 陀螺仪走 USART2 (PD5/PD6) */


/* ==========================================================================
 *  1 参数区
 * ========================================================================== */

#define MOTOR_NUM        4

/* 四台电机的 CAN 地址，顺序 = 串口上按 1 2 3 4 对应的那台 */
static const uint8_t g_addr[MOTOR_NUM] = { 1, 2, 3, 4 };

/* 机械臂两根轴：跑圈时上电使能一下，免得臂在车上晃。不想让它使能就改成 0 */
#define ARM_HOLD         1
#define ADDR_ARM_PUSH    5          /* 平推 */
#define ADDR_ARM_LIFT    7          /* 升降 */

/* ---- 手动单次走多少 / 走多快（f b l r 用来查方向、做标定） ---- */
#define DIST_DEFAULT     1000
#define DIST_STEP        50
#define DIST_MIN         50
#define DIST_MAX         2000

/* 速度 rpm。跑圈默认 60（比 move 版的 100 慢一点，第一次上真场地稳一点）；
 * 想快就按 . 键一档一档加 */
#define SPEED_DEFAULT    60
#define SPEED_STEP       10
#define SPEED_MIN        20
#define SPEED_MAX        200

/* 加速度档位 0~255（0 = 不经过加减速直接全速，很猛）。
 * 这个数既发给驱动器，也拿去估"这一趟要走多久" */
#define ACC_DEFAULT      150
#define WAIT_MIN_MS      200

#define UART_BAUD        115200

/* ---- 脉冲/毫米：全车最重要的两个标定数 ----
 * ppm_y 10/8 实测过：命令 1000mm 实走 995mm -> 10.3 x 1000/995 = 10.35 -> 微调 10.36。
 * ppm_x 10/13 实测过：命令 1000mm 实走 950mm -> 10.5 x 1000/950 = 11.05。
 *     新系数 = 老系数 x 命令毫米 / 实走毫米
 * 串口上 [ ] 调 ppm_x、{ } 调 ppm_y，一次 0.05（走 1 米约合 5mm）。 */
static float g_ppm_x = 11.05f;
static float g_ppm_y = 10.36f;

/* ---- 横移交叉耦合补偿 ----
 * 10/13 实测：左移 1000mm，车除了横移还整体往后挪了约 25mm（左轮 30 / 右轮 20），
 * 车是斜着退的。麦轮横移时四轮同向、线速度常数不同，这个前后分量是结构性的，
 * 跟场地无关。改法：横移的时候让车心顺带往前挪 comp_ratio x 横移距离，抵消掉。
 * 左移配"往前补"、右移配"往后补"（正负由横移方向自动定）。
 * 实测校准：左移 1000mm 量出往前还差多少（mm），comp_ratio = 差的毫米 / 1000。 */
static float g_comp_ratio = 0.025f;

/* ---- 原地旋转（用来试"用旋转代替横移"）----
 * 旋转时对角两组轮子反向转：{1,4} 一组、{2,3} 一组，车心不动只转朝向。
 * 没有陀螺仪，转多少度全靠轮子走的弧长，所以 PPD 是实测标定出来的：
 *   先跑一次转 360 度，量车实际转了多少（度），
 *   新 PPD = 老 PPD x 360 / 实转角度。
 * 初值 18.0 是估的（轮距约 190mm），基本肯定要改，按实测调。 */
static const int8_t g_pivot_spin[MOTOR_NUM] = { +1, -1, -1, +1 };   /* 旋转时每轮方向系数 */
static const uint8_t g_pivot_dirpos[MOTOR_NUM] = { 1, 0, 0, 1 };    /* 旋转时每轮 dir_pos */

static float   g_ppd       = 18.0f;     /* pulses per degree（旋转标定） */
static float   g_pivot_ang = 0.0f;      /* 累计车头角度（度，左转为正），只做参考 */

/* 10/13 晚：横移改成"旋转方案"的开关与方向修正
 * g_piv_strafe = 1 : 横移段走 pivot+直行+pivot（默认，用户要求"用旋转的方式跑全程"）
 *               0 : 退回原来的麦轮横移
 * g_pivot_inv  = +1 : 正角度往左转。实车转反了按 m 键取反，不用重编。 */
static uint8_t g_piv_strafe = 1;
static int8_t  g_pivot_inv  = 1;

/* 起点"并线"（10/13 晚加回）：源项目里场地 A 标记在 (2250,170)，而我们的中线起点
 * 在 (2190,200) —— 差 60mm（往左）+ 30mm（往前）。比赛时车是摆在 A 标记上的，
 * 所以起步先做"左移 60 + 前进 30"并到中线上；跑完最后一段再反向做一次"右移 60 +
 * 后退 30"回到标记上，整圈起点终点都落在场地标记。按 a 键开关（默认开）。 */
#define MARK_LAT_MM      60
#define MARK_FWD_MM      30
static uint8_t g_align_mark = 1;
#define PPM_STEP         0.05f

/* ---- 方向映射：哪个键方向不对，只改这 4 个数 ----
 *   AXIS：0 = 走 X 家族，1 = 走 Y 家族      SIGN：+1 / -1 */
#define MAP_FWD_AXIS     1
#define MAP_FWD_SIGN     (-1)       /* 10/7 实测：上电整车往后走 -> 取反 */
#define MAP_LEFT_AXIS    0
#define MAP_LEFT_SIGN    (+1)


/* ==========================================================================
 *  2 场地路线表（抄自 structure.h 的 road_motion_table）
 *     axis：AXIS_FWD = 前进/后退，AXIS_LEFT = 左移/右移
 *     mm  ：正 = 前进 / 左移，负 = 后退 / 右移。车头朝场地 +Y。
 * ========================================================================== */

#define AXIS_LEFT        0
#define AXIS_FWD         1

typedef struct {
    uint8_t axis;
    int16_t mm;
} LapSeg;

/* 10/13 改版：纯走中线，8 段闭环（原 A->B(60) 等短段已并入相邻长段）
 * A->B(road0) / B->C(road1) / C->D(road2) / D->I(road3) /
 * I->H(road4) / H->G(road5) / G->F(road6) / F->A(road7) */
static const LapSeg g_lap[] = {
    { AXIS_FWD,   1000 },   /*  1  A->B  右中线下段 200->1200 */
    { AXIS_FWD,    860 },   /*  2  B->C  右中线上段 1200->2060 */
    { AXIS_LEFT,  990 },    /*  3  C->D  上中线右段 2190->1200 */
    { AXIS_LEFT,  860 },    /*  4  D->I  上中线左段 1200->340 */
    { AXIS_FWD,   -860 },   /*  5  I->H  左中线上段 2060->1200 */
    { AXIS_FWD,  -1000 },   /*  6  H->G  左中线下段 1200->200 */
    { AXIS_LEFT,  -860 },   /*  7  G->F  下中线左段 340->1200 */
    { AXIS_LEFT,  -990 },   /*  8  F->A  下中线右段 1200->2190 */
};
#define LAP_N   ((uint8_t)(sizeof(g_lap) / sizeof(g_lap[0])))


/* ==========================================================================
 *  2b 场地节点表 + 起点（逐字抄自 route-study\mcu-navigation\road_map.c）
 *      nodes[] 的顺序 = structure.h 里的 RoadNode：A B C D E F G H I
 *      seg = 从该节点出发时，g_lap[] 里接着要走第几段（0 基）
 *      E 是场地正中，不在外圈上，跑整圈用不到 -> seg 记 0xFF
 * ========================================================================== */

typedef struct {
    char    name;
    int16_t x, y;
    uint8_t seg;
} LapNode;

/* 10/13 改版：坐标改成中线矩形的角（原源码 A(2250,170)/C(2250,2230) 已挪到中线上） */
static const LapNode g_node[9] = {
    { 'A', 2190,  200, 0x00 },   /* 右中线 × 下中线 交点 */
    { 'B', 2190, 1200, 0x01 },   /* 右中线 × 中横线 交点 */
    { 'C', 2190, 2060, 0x02 },   /* 右中线 × 上中线 交点 */
    { 'D', 1200, 2060, 0x03 },   /* 上中线 × 中竖线 交点 */
    { 'E', 1200, 1200, 0xFF },   /* 场地正中，外圈不含 */
    { 'F', 1200,  200, 0x06 },   /* 下中线 × 中竖线 交点 */
    { 'G',  340,  200, 0x05 },   /* 左中线 × 下中线 交点 */
    { 'H',  340, 1200, 0x04 },   /* 左中线 × 中横线 交点 */
    { 'I',  340, 2060, 0x04 }    /* 左中线 × 上中线 交点 */
};

/* 默认起点：0 = A（中线矩形右下角 2190,200） */
#define LAP_START_NODE   0
#define NODE_E_INDEX     4       /* 场地正中，换起点时跳过 */

static uint8_t g_start = LAP_START_NODE;                       /* 当前起点节点 0~8 */
static LapSeg  g_run[sizeof(g_lap) / sizeof(g_lap[0])];        /* 按起点旋转过的整圈表 */

/* 把 8 段整圈表旋转成「从节点 g_start 出发」的顺序，填进 g_run[] */
static void Lap_BuildRun(void)
{
    uint8_t base = g_node[g_start].seg;
    uint8_t i;

    if (base == 0xFF) { base = 0; }
    for (i = 0; i < LAP_N; i++)
    {
        g_run[i] = g_lap[(uint8_t)((base + i) % LAP_N)];
    }
}


/* ==========================================================================
 *  3 调试串口（USART3：PB10 = TX，PB11 = RX）
 *    配套硬件：集成板「传感器预留 U4」座
 *      1 = 5V1   2 = PB11(MCU RX)   3 = PB10(MCU TX)   4 = GND
 *    无线调试器接法（交叉）：调试器 RX <- U4-3(TX)、调试器 TX -> U4-2(RX)、GND <-> U4-4
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

/* 带符号整数（坐标要打负数） */
static void UART_SendIntS(int32_t n)
{
    if (n < 0) { UART_SendByte('-'); n = -n; }
    UART_SendInt((uint32_t)n);
}

/* 打印一位小数：0.1 -> "0.1"（不引 printf，省得链一大堆库） */
static void UART_SendFix1(float v)
{
    uint32_t x;

    if (v < 0.0f) { UART_SendByte('-'); v = -v; }
    x = (uint32_t)(v * 10.0f + 0.5f);
    UART_SendInt(x / 10);
    UART_SendByte('.');
    UART_SendByte((uint8_t)('0' + (x % 10)));
}

/* 打印两位小数：10.35 -> "10.35"（ppm 标定要这个精度） */
static void UART_SendFix2(float v)
{
    uint32_t x;

    if (v < 0.0f) { UART_SendByte('-'); v = -v; }
    x = (uint32_t)(v * 100.0f + 0.5f);
    UART_SendInt(x / 100);
    UART_SendByte('.');
    UART_SendByte((uint8_t)('0' + ((x / 10) % 10)));
    UART_SendByte((uint8_t)('0' + (x % 10)));
}

static uint8_t UART_GetByte(void)
{
    if (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET)
        return (uint8_t)USART_ReceiveData(USART3);
    return 0;
}


/* ==========================================================================
 *  4 轮子表 + 走位算法（和 main_move.c 一模一样，改过就跑偏）
 * ========================================================================== */

typedef struct {
    uint8_t addr;       /* CAN 地址 */
    int8_t  sx;         /* v_x 的系数 */
    int8_t  sy;         /* v_y 的系数 */
    int8_t  sw;         /* w   的系数 */
    uint8_t dir_pos;    /* 速度为正时发给驱动器的 dir（0 或 1） */
} Wheel_t;

/* 表的顺序跟 g_addr[] 一样：{1, 2, 3, 4}。值从 yyb_move.c 的 car_move() 抄的 */
static const Wheel_t g_wheel[MOTOR_NUM] = {
    { 1, -1, +1, +1, 1 },
    { 2, +1, +1, -1, 0 },
    { 3, -1, +1, -1, 0 },
    { 4, +1, +1, +1, 1 },
};

static uint16_t g_dist  = DIST_DEFAULT;     /* 手动一次走多少毫米 */
static uint16_t g_speed = SPEED_DEFAULT;    /* 速度 rpm */

/* 估计"这一趟大概要走多久"，单位 ms。和原项目 car_move_delay() 一个公式：
 *   加减速段走掉的脉冲 = 53.33 x v^2 / a （1 RPM 约 53.33 脉冲/秒）
 *   加减速段花的时间   = 2000 x v / a      （单位 ms） */
static uint32_t Mv_EstimateMs(uint32_t pulses, uint16_t acc, uint16_t speed_rpm)
{
    float q, q_acc, t_acc, t;

    if (acc == 0)       { acc = 256; }
    if (speed_rpm == 0) { speed_rpm = 1; }

    q     = (float)pulses;
    q_acc = 53.3333f * (float)speed_rpm * (float)speed_rpm / (float)acc;
    t_acc = 2000.0f * (float)speed_rpm / (float)acc;

    if (q_acc >= q) { t = t_acc * sqrtf(q / q_acc); }
    else            { t = 1000.0f * (q - q_acc) / (53.3333f * (float)speed_rpm) + t_acc; }

    if (t < (float)WAIT_MIN_MS) { t = (float)WAIT_MIN_MS; }
    return (uint32_t)t;
}

/* 位置模式：沿 family 那一族走 distance_mm 毫米（正负 = 方向）。
 * comp_mm != 0 时，只给"另一族"的两个轮子加减脉冲，让车心在横移的同时前后挪一点，
 * 抵消麦轮横移的前后漂移（见 g_comp_ratio）。返回这一族的主脉冲数。 */
static uint32_t Mv_Pos(uint8_t family, float distance_mm, float comp_mm)
{
    float    ppm = (family == 0) ? g_ppm_x : g_ppm_y;
    float    pf  = ppm * ((distance_mm < 0.0f) ? -distance_mm : distance_mm);
    int      sign;
    uint32_t pulses;
    int32_t  comp_pulses = 0;
    uint8_t  i;

    sign = (distance_mm < 0.0f) ? -1 : +1;
    if (pf < 1.0f) { pf = 1.0f; }
    pulses = (uint32_t)(pf + 0.5f);

    if (comp_mm != 0.0f)
    {
        float cp = g_ppm_y * ((comp_mm < 0.0f) ? -comp_mm : comp_mm);
        comp_pulses = (int32_t)(cp + 0.5f);
        if (comp_mm < 0.0f) { comp_pulses = -comp_pulses; }
    }

    for (i = 0; i < MOTOR_NUM; i++)
    {
        int     k   = (family == 0) ? g_wheel[i].sx : g_wheel[i].sy;
        int     dk  = (family == 0) ? g_wheel[i].sy : g_wheel[i].sx;   /* 另一族系数 */
        int32_t raw;
        uint8_t dir;

        k *= sign;
        raw = (int32_t)k * (int32_t)pulses;
        if (comp_pulses != 0) { raw -= (int32_t)dk * comp_pulses; }
        if (raw == 0) { continue; }            /* 0 脉冲别发，驱动器会报错 */

        dir = (raw > 0) ? g_wheel[i].dir_pos : (g_wheel[i].dir_pos ? 0 : 1);

        /* false = 相对运动（走这段就停），true = 先憋着等同步广播 */
        Emm_V5_Pos_Control(g_wheel[i].addr, dir, g_speed, ACC_DEFAULT,
                           (uint32_t)((raw > 0) ? raw : -raw), false, true);
        delay_ms(1);
    }
    Emm_V5_Synchronous_motion(0x00);    /* 广播：四轮同时起步 */

    return pulses;
}

/* 前向声明：下面的陀螺仪闭环和等待循环都要用 */
static void    Mv_Stop_All(void);
static uint8_t Mv_Wait(uint32_t ms);
static uint8_t Mv_DriveFwdRaw(float mm_fwd, const char *name);


/* ==========================================================================
 *  6.5 陀螺仪（WT 系列：USART2，PD5 = TX / PD6 = RX，模块 TX -> 核心板 PD6）
 *       —— 转向闭环 + 直行纠偏
 *
 *  为什么要有它：以前转 90 度 = "发 ppd x 90 个脉冲，然后死等那么久"，轮子一
 *  打滑角度就不准，一段一段攒下来整圈会歪。现在改成：速度模式一直转，同时盯
 *  着陀螺仪的 yaw，误差进了死区再停 —— 打滑多少它自己补回来，而且不再需要
 *  标定 ppd，也不用管 g_pivot_inv 正不正（闭环自己会收敛）。
 *
 *  开机流程 Gyro_Init()：找模块 -> 静止 2 秒量出模块自身漂移 -> 把开机朝向定为
 *  0 度。所以开机这几秒别动车。坑：imu_init() 只在开机扫一次波特率，开机时线
 *  没接好会锁死在错的波特率 -> 拔插/接线之后按一下 RST。
 *
 *  e 键关掉闭环就退回老办法；陀螺仪没检测到也会自动关掉，不会卡死。
 * ========================================================================== */

/* ---- 可调参数 ---- */
static uint8_t g_closed   = 1;      /* e 键：1 = 转向闭环，0 = 退回数脉冲的老办法 */
static float   g_yaw_sign = 1.0f;   /* ; 键：偏航方向取反（转了反方向才用） */
static float   g_turn_db  = 2.0f;   /* 旋转到位死区（度），误差小于它就算到位 */
static float   g_hold_db  = 2.5f;   /* 直行纠偏死区（度），歪超过它才纠 */

#define TURN_VEL_DIV      3         /* 闭环旋转速度 = g_speed / 3（默认 60 -> 20 rpm） */
#define TURN_VEL_MIN      10        /* 再慢也不低于这个 rpm */
#define SLOW_NEAR_DEG     8.0f      /* 离目标还剩这些度就减半速，防冲过头来回摆 */
#define TURN_TIMEOUT_MS   8000U     /* 单次旋转最多等 8 秒（陀螺仪掉线的保护） */
#define GYRO_DRIFT_MS     2000U     /* 量漂移的窗口长度 */
#define GYRO_DRIFT_LIMIT  30.0f     /* 窗口里 yaw 变这么多 -> 当手在动，不采信 */
#define HOLD_CHUNK_MM     200.0f    /* 直行每走这么多毫米停下来看一眼车头 */
#define HOLD_SETTLE_MS    250U      /* 停稳多久再读 yaw（太短读数还在晃） */

/* ---- 状态 ---- */
static float    s_yawOffset  = 0.0f;    /* 开机朝向对应的模块 yaw */
static float    s_driftRate  = 0.0f;    /* 模块自身漂移速度，度/秒 */
static uint32_t s_elapsedMs  = 0;       /* 从"定零点"起走过的毫秒 */
static float    s_lastYaw    = 0.0f;    /* 最近一帧的模块 yaw */
static uint8_t  s_yawValid   = 0;       /* 1 = 检测到陀螺仪在出数 */
static float    g_heading_cmd = 0.0f;   /* 车头"应该"朝哪（度，开机朝向 = 0，左转为正） */


static float Wrap180(float a)
{
    while (a >  180.0f) { a -= 360.0f; }
    while (a < -180.0f) { a += 360.0f; }
    return a;
}

/* 当前车头相对"开机朝向"的角度（已经减掉模块自身漂移） */
static float RelAngle(void)
{
    float raw = s_lastYaw - s_yawOffset
                - (s_driftRate * ((float)s_elapsedMs / 1000.0f));
    return Wrap180(g_yaw_sign * raw);
}

/* 取一帧 + 时间前进 10ms。非阻塞：模块没出新数据就沿用上一次的值。
 * 凡是 delay_ms(10) 的等待循环里都调一下，s_elapsedMs 才不会停。 */
static void Gyro_Tick(void)
{
    float fAcc[3];
    float fGyro[3];
    float fAngle[3];

    fAcc[0] = fAcc[1] = fAcc[2] = 0.0f;
    fAngle[0] = fAngle[1] = fAngle[2] = 0.0f;
    imu_scan(fAcc, fGyro, fAngle);

    s_lastYaw = fAngle[2];
    s_elapsedMs += 10;
}

/* 探一下模块在不在：静止时加速度模长应该在 1g 附近，而且读数得有"变化"
 * （线没接的时候 sReg 是死的，读多少次都是一个值 -> distinct 会很小）。 */
static uint8_t Gyro_Probe(void)
{
    float   fAcc[3], fGyro[3], fAngle[3];
    float   m[10];
    uint8_t i, j, ok = 0, distinct = 0;

    for (i = 0; i < 10; i++)
    {
        fAcc[0] = fAcc[1] = fAcc[2] = 0.0f;
        imu_scan(fAcc, fGyro, fAngle);
        m[i] = sqrtf(fAcc[0] * fAcc[0] + fAcc[1] * fAcc[1] + fAcc[2] * fAcc[2]);
        if ((m[i] > 0.7f) && (m[i] < 1.3f)) { ok++; }
        delay_ms(20);
    }

    for (i = 0; i < 10; i++)
    {
        uint8_t seen = 0;
        for (j = 0; j < i; j++)
        {
            if (fabsf(m[j] - m[i]) < 0.005f) { seen = 1; break; }
        }
        if (!seen) { distinct++; }
    }

    return (uint8_t)((ok >= 8) && (distinct >= 3));
}

/* 把"现在朝哪"定为 0 度（开机自动做一次，按 x 也能随时重定） */
static void Gyro_Zero(void)
{
    s_yawOffset   = s_lastYaw - (s_driftRate * ((float)s_elapsedMs / 1000.0f));
    s_elapsedMs   = 0;
    g_heading_cmd = 0.0f;
    USART_SendString("[gyro] heading zeroed here\r\n");
}

/* 开机标定。返回 1 = 陀螺仪可用，0 = 没找到（自动关掉闭环，退回老办法） */
static uint8_t Gyro_Init(void)
{
    float   fAcc[3], fGyro[3], fAngle[3];
    float   yawA, yawB, delta;

    USART_SendString("[gyro] init (USART2: PD5=TX PD6=RX), scanning baud ...\r\n");
    imu_init();

    if (!Gyro_Probe())
    {
        USART_SendString("[gyro] NO DATA -- check 5V / GND / module TX -> PD6, then press RST.\r\n");
        USART_SendString("[gyro] closed loop DISABLED (fall back to ppd pulses).\r\n");
        s_yawValid = 0;
        g_closed   = 0;
        return 0;
    }

    USART_SendString("[gyro] found. DO NOT MOVE the car for 2s, measuring drift ...\r\n");
    delay_ms(300);

    fAcc[0] = fAcc[1] = fAcc[2] = 0.0f;
    imu_scan(fAcc, fGyro, fAngle);
    yawA = fAngle[2];

    delay_ms(GYRO_DRIFT_MS);

    imu_scan(fAcc, fGyro, fAngle);
    yawB  = fAngle[2];
    delta = Wrap180(yawB - yawA);

    if ((delta > GYRO_DRIFT_LIMIT) || (delta < -GYRO_DRIFT_LIMIT))
    {
        s_driftRate = 0.0f;
        USART_SendString("[gyro] module was moving - drift not measured (using 0)\r\n");
    }
    else
    {
        s_driftRate = delta / ((float)GYRO_DRIFT_MS / 1000.0f);
        USART_SendString("[gyro] drift = ");
        UART_SendFix2(s_driftRate);
        USART_SendString(" deg/s\r\n");
    }

    s_lastYaw     = yawB;
    s_yawOffset   = yawB;
    s_elapsedMs   = 0;
    s_yawValid    = 1;
    g_heading_cmd = 0.0f;

    USART_SendString("[gyro] this direction is now 0 deg. heading = 0.00\r\n");
    return 1;
}


/* 让车绕自己中心转：四轮反着来。dir_sign = +1 左转 / -1 右转。
 * 方向系数用的是 g_wheel[].sw（和原项目的 w 系数一套），dir_pos 复用位置模式的。 */
static void Mv_Vel_Spin(int dir_sign, uint16_t rpm)
{
    uint8_t i;

    for (i = 0; i < MOTOR_NUM; i++)
    {
        int     k   = (int)g_wheel[i].sw * dir_sign;
        uint8_t dir = (k > 0) ? g_wheel[i].dir_pos : (g_wheel[i].dir_pos ? 0 : 1);

        Emm_V5_Vel_Control(g_wheel[i].addr, dir, rpm, ACC_DEFAULT, true);
        delay_ms(1);
    }
    Emm_V5_Synchronous_motion(0x00);    /* 广播：四轮同时起步 */
}

/* 闭环旋转：速度模式一直转，盯 yaw，误差进死区就停。
 * ang 就是"车头该转多少度"（正 = 左转），target = 当前 yaw + ang。
 * 返回 1 = 到位，0 = 被 s 打断 / 超时。 */
static uint8_t Pivot_RotateClosed(float ang, const char *name)
{
    float    nb    = Wrap180(RelAngle() + ang);     /* 转完之后车头该在的 yaw */
    float    err;
    float    tgt   = nb;
    int      spin;
    uint16_t vel   = (uint16_t)(g_speed / TURN_VEL_DIV);
    uint32_t t     = 0;
    uint8_t  slow  = 0;
    uint8_t  i;

    if (vel < TURN_VEL_MIN) { vel = TURN_VEL_MIN; }

    err  = Wrap180(tgt - RelAngle());
    spin = (err < 0.0f) ? -1 : +1;

    USART_SendString("[pivot/GYRO] ");
    USART_SendString(name);
    USART_SendString("  cmd ");
    UART_SendFix1(ang);
    USART_SendString(" deg  now ");
    UART_SendFix1(RelAngle());
    USART_SendString(" -> target ");
    UART_SendFix1(tgt);
    USART_SendString("  vel ");
    UART_SendInt(vel);
    USART_SendString(" rpm  db ");
    UART_SendFix1(g_turn_db);
    USART_SendString("\r\n");

    Mv_Vel_Spin(spin, vel);

    while (t < TURN_TIMEOUT_MS)
    {
        uint8_t c;

        delay_ms(10);
        t += 10;
        Gyro_Tick();

        err = Wrap180(tgt - RelAngle());

        if (fabsf(err) <= g_turn_db) { break; }     /* 进死区了 */

        /* 快到了降半速，免得冲着死区冲过去再摆回来 */
        if ((!slow) && (fabsf(err) < SLOW_NEAR_DEG))
        {
            slow = 1;
            Mv_Vel_Spin(spin, (uint16_t)(vel / 2));
        }

        if ((t % 500) == 0) { UART_SendByte('.'); }

        c = UART_GetByte();
        if ((c == 's') || (c == 'S') || (c == 'z') || (c == 'Z'))
        {
            Mv_Stop_All();
            USART_SendString("  INTERRUPTED\r\n");
            return 0;
        }
    }

    Mv_Stop_All();      /* 立刻停：Stop_Now + 速度给 0 */

    if (t >= TURN_TIMEOUT_MS)
    {
        USART_SendString("  TIMEOUT after ");
        UART_SendInt(t);
        USART_SendString("ms  residual err = ");
        UART_SendFix1(err);
        USART_SendString(" deg -> STOP. (gyro ok? try x to re-zero)\r\n");
        return 0;
    }

    /* 停稳之后再读一次，把最终误差打出来（现场看闭环效果） */
    delay_ms(200);
    Gyro_Tick();
    err = Wrap180(tgt - RelAngle());

    g_heading_cmd = tgt;
    g_pivot_ang   = Wrap180(tgt);       /* 显示用：车头现在朝多少度 */

    USART_SendString("  done  yaw = ");
    UART_SendFix1(RelAngle());
    USART_SendString("  residual err = ");
    UART_SendFix1(err);
    USART_SendString(" deg\r\n");
    return 1;
}

/* 直行中途纠偏：车头歪得超过 g_hold_db 就原地纠回来。
 * 返回 1 = 不用纠或纠好了，0 = 被 s 打断 / 超时。 */
static uint8_t Head_Correct(void)
{
    float err;

    Gyro_Tick();
    err = Wrap180(g_heading_cmd - RelAngle());

    if (fabsf(err) <= g_hold_db) { return 1; }

    USART_SendString("\r\n  [hold] drifted ");
    UART_SendFix1(err);
    USART_SendString(" deg -> fix\r\n");

    return Pivot_RotateClosed(err, "heading fix");
}


/* 原地旋转 angle_deg 度：正 = 左转（逆时针），负 = 右转。
 * 陀螺仪在线 + 闭环开着 -> 走 Pivot_RotateClosed（推荐的）；否则退回老办法：
 * 发 ppd x 角度 个脉冲 + 定时死等（g_pivot_inv = -1 时整体取反，按 m 键改）。
 * 返回 1 = 到位，0 = 中途被 s 打断。 */
static uint8_t Pivot_Rotate(float angle_deg, const char *name)
{
    float    ang;
    float    f;
    int      sign;
    uint32_t pulses, ms;
    uint8_t  i;

    /* ---- 闭环：陀螺仪说了算 ---- */
    if (g_closed && s_yawValid) { return Pivot_RotateClosed(angle_deg, name); }

    /* ---- 开环：数脉冲 ---- */
    ang  = angle_deg * (float)g_pivot_inv;
    f    = g_ppd * ((ang < 0.0f) ? -ang : ang);
    sign = (ang < 0.0f) ? -1 : +1;

    pulses = (uint32_t)(f + 0.5f);
    if (pulses == 0) { pulses = 1; }
    ms = Mv_EstimateMs(pulses, ACC_DEFAULT, g_speed);
    if (ms < 400) { ms = 400; }

    USART_SendString("[pivot/OPEN] ");
    USART_SendString(name);
    USART_SendString("  ");
    UART_SendFix1(ang);
    USART_SendString(" deg  (ppd=");
    UART_SendFix2(g_ppd);
    USART_SendString(")  pulses=");
    UART_SendInt(pulses);
    USART_SendString("  wait~");
    UART_SendInt(ms);
    USART_SendString("ms\r\n");

    for (i = 0; i < MOTOR_NUM; i++)
    {
        int     k   = g_pivot_spin[i] * sign;
        uint8_t dir = (k > 0) ? g_pivot_dirpos[i] : (g_pivot_dirpos[i] ? 0 : 1);

        Emm_V5_Pos_Control(g_wheel[i].addr, dir, g_speed, ACC_DEFAULT, pulses, true, true);
        delay_ms(1);
    }
    Emm_V5_Synchronous_motion(0x00);

    if (!Mv_Wait(ms)) { USART_SendString("  INTERRUPTED\r\n"); return 0; }
    USART_SendString("  done\r\n");

    g_pivot_ang   += ang;
    g_heading_cmd  = Wrap180(g_heading_cmd + ang);   /* 账本跟着走，切回闭环也不会错 */
    return 1;
}
/* 沿车头方向直行 mm_fwd 毫米（正 = 朝车头方向往前，负 = 倒退）。
 * 用 FWD 那一族的映射，不加 comp（这一族本来就没有横移漂移）。
 * 返回 1 = 走完，0 = 被 s 打断。 */
static uint8_t Mv_DriveFwdRaw(float mm_fwd, const char *name)
{
    uint32_t pulses = Mv_Pos(MAP_FWD_AXIS, (float)MAP_FWD_SIGN * mm_fwd, 0.0f);
    uint32_t ms     = Mv_EstimateMs(pulses, ACC_DEFAULT, g_speed);

    USART_SendString("[drive] ");
    USART_SendString(name);
    USART_SendString("  ");
    UART_SendFix1(mm_fwd);
    USART_SendString(" mm  pulses=");
    UART_SendInt(pulses);
    USART_SendString("  wait~");
    UART_SendInt(ms);
    USART_SendString("ms");

    if (!Mv_Wait(ms)) { USART_SendString("  INTERRUPTED\r\n"); return 0; }
    USART_SendString("  done\r\n");
    return 1;
}

/* 直行 + 陀螺仪纠偏。距离还是靠位置模式数脉冲（那个已经标定准了，别动），
 * 只是把它切成 HOLD_CHUNK_MM 一段一段走，每段之间停下来看一眼车头：
 * 歪了先原地纠回来再接着走 —— 这样整段路线才是一条直线，不会越走越斜。
 * 闭环关掉 / 陀螺仪没接 时就是原来的 Mv_DriveFwdRaw，行为完全不变。 */
static uint8_t Mv_DriveFwd(float mm_fwd, const char *name)
{
    float   left;
    uint8_t n = 0;

    if ((!g_closed) || (!s_yawValid) || (fabsf(mm_fwd) <= HOLD_CHUNK_MM))
    {
        return Mv_DriveFwdRaw(mm_fwd, name);
    }

    left = mm_fwd;
    while (fabsf(left) > 0.5f)
    {
        float step = (left > 0.0f) ? HOLD_CHUNK_MM : -HOLD_CHUNK_MM;

        if (fabsf(left) < HOLD_CHUNK_MM) { step = left; }

        if (!Mv_DriveFwdRaw(step, name)) { return 0; }

        left -= step;
        n++;

        if (fabsf(left) > 0.5f)
        {
            delay_ms(HOLD_SETTLE_MS);       /* 等车停稳再读 yaw */
            if (!Head_Correct()) { return 0; }
        }
    }

    USART_SendString("  [hold] drove ");
    UART_SendInt(n);
    USART_SendString(" chunks with heading hold\r\n");
    return 1;
}

/* 直接用麦轮横移 dmm 毫米（正 = 左移），带前后漂移补偿。
 * 就是 j 切到 raw 之后走的那条路。返回 1 = 走完，0 = 被 s 打断。 */
static uint8_t Mv_StrafeRaw(float dmm, const char *name)
{
    float    comp   = dmm * g_comp_ratio;
    uint32_t pulses = Mv_Pos(MAP_LEFT_AXIS, dmm, comp);
    uint32_t ms     = Mv_EstimateMs(pulses, ACC_DEFAULT, g_speed);

    USART_SendString("[strafe-raw] ");
    USART_SendString(name);
    USART_SendString("  ");
    UART_SendFix1(dmm);
    USART_SendString(" mm  comp=");
    UART_SendFix1(comp);
    USART_SendString(" mm  pulses=");
    UART_SendInt(pulses);
    USART_SendString("  wait~");
    UART_SendInt(ms);
    USART_SendString("ms");

    if (!Mv_Wait(ms)) { USART_SendString("  INTERRUPTED\r\n"); return 0; }
    USART_SendString("  done\r\n");
    return 1;
}

/* 横移的"旋转方案"：先原地转 90 度把车头对准要去的方向，再沿车头方向直行
 * mag 毫米，最后反向转 90 度转回原朝向。
 *   dmm > 0 : 左移（先左转，车头朝场地 -x，直行 = 往 -x 走）
 *   dmm < 0 : 右移（先右转，车头朝场地 +x，直行 = 往 +x 走）
 * 好处：横移的那段距离完全交给"直行"完成，不再吃麦轮横移的前后漂移；
 *       每次转完都转回来，跑完整圈车头方向和出发时一样（还是 +Y）。
 * 返回 1 = 正常，0 = 被 s 打断。 */
static uint8_t Mv_StrafeByTurn(float dmm, const char *name)
{
    float    a   = (dmm >= 0.0f) ? 90.0f : -90.0f;
    float    mag = (dmm < 0.0f) ? -dmm : dmm;

    USART_SendString("[strafe-turn] ");
    USART_SendString(name);
    USART_SendString("  ");
    UART_SendFix1(dmm);
    USART_SendString(" mm  =  pivot ");
    UART_SendFix1(a);
    USART_SendString(" -> drive ");
    UART_SendFix1(mag);
    USART_SendString(" -> pivot ");
    UART_SendFix1(-a);
    USART_SendString(" deg\r\n");

    if (!Pivot_Rotate(a, "strafe t1")) { return 0; }

    /* 车头已经对着目标方向，纯前进 mag 毫米（用 FWD 那一族的映射，别加 comp） */
    if (!Mv_DriveFwd(mag, "strafe run")) { return 0; }

    if (!Pivot_Rotate(-a, "strafe t2")) { return 0; }
    return 1;
}

/* 起点/终点"并线"：
 *   dir = +1 : 从场地 A 标记走到中线起点（左移 60 + 前进 30）
 *   dir = -1 : 从走的终点回到场地 A 标记（右移 60 + 后退 30）
 * 横移那 60mm 跟着 j 的模式走（PIVOT 或 raw）。返回 1 = 正常，0 = 被 s 打断。 */
static uint8_t Lap_MarkShift(int8_t dir, const char *name)
{
    float lat = (float)MARK_LAT_MM * (float)dir;    /* 正 = 左移 */
    float fwd = (float)MARK_FWD_MM * (float)dir;    /* 正 = 前进 */

    USART_SendString("[align] ");
    USART_SendString(name);
    USART_SendString("  lat ");
    UART_SendFix1(lat);
    USART_SendString(" mm + fwd ");
    UART_SendFix1(fwd);
    USART_SendString(" mm   (strafe mode = ");
    USART_SendString(g_piv_strafe ? "PIVOT" : "raw");
    USART_SendString(")\r\n");

    if (g_piv_strafe) { if (!Mv_StrafeByTurn(lat, name)) { return 0; } }
    else              { if (!Mv_StrafeRaw(lat, name))     { return 0; } }

    if (!Mv_DriveFwd(fwd, name)) { return 0; }
    return 1;
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
        Gyro_Tick();        /* 顺手收一帧陀螺仪：时间基准不断、yaw 一直是最新的 */
        if ((t % 500) == 0) { UART_SendByte('.'); }     /* 每 0.5s 一个点 */

        c = UART_GetByte();
        if (c == 's' || c == 'S' || c == 'z' || c == 'Z')
        {
            Mv_Stop_All();
            return 0;
        }
    }
    return 1;
}

/* 走一趟：算脉冲 -> 发指令 -> 估时间 -> 等 -> 报告结果（手动 f b l r 用） */
static void Mv_MoveDir(uint8_t family, int8_t sign, const char *name)
{
    float    mm     = (float)g_dist * (float)sign;
    float    comp   = (family == MAP_LEFT_AXIS) ? (mm * g_comp_ratio) : 0.0f;
    uint32_t pulses = Mv_Pos(family, mm, comp);
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
    USART_SendString("  (if 4 wheels move together -> all motors on same addr)\r\n");

    Emm_V5_Pos_Control(g_wheel[idx].addr, g_wheel[idx].dir_pos, g_speed,
                       ACC_DEFAULT, pulses, false, true);
    Emm_V5_Synchronous_motion(0x00);

    if (Mv_Wait(ms)) { USART_SendString(" done\r\n"); }
    else             { USART_SendString(" interrupted\r\n"); }
}


/* ==========================================================================
 *  5 绕圈状态机
 *     g_seg = 下一段要走第几段（0 基）。== LAP_N 就是整圈走完了。
 *     g_px / g_py = 理论场地坐标（相对起步的 A 点，单位 mm）。
 *       车头朝场地 +Y：forward 记 +y，left 记 -x。
 * ========================================================================== */

static uint8_t  g_seg   = 0;
static float    g_scale = 1.0f;         /* 1.0 = 真场地全尺寸；先降到 0.1 试 */
static float    g_px    = 0.0f;
static float    g_py    = 0.0f;
#define SCALE_MIN       0.1f
#define SCALE_MAX       1.0f
#define SCALE_STEP      0.1f

static void Lap_ShowPos(void)
{
    USART_SendString("  pos from start = (");
    UART_SendIntS((int32_t)g_px);
    USART_SendString(", ");
    UART_SendIntS((int32_t)g_py);
    USART_SendString(") mm\r\n");
}

static void Lap_Reset(void)
{
    g_seg = 0;
    g_px  = 0.0f;
    g_py  = 0.0f;
    USART_SendString("lap reset -> standing at start node ");
    UART_SendByte((uint8_t)g_node[g_start].name);
    USART_SendString(", segment 1\r\n");
    Lap_ShowPos();
}

static void Lap_Show(void)
{
    uint8_t i;
    float   rest = 0.0f;

    USART_SendString("\r\n--- lap progress ---\r\n");
    USART_SendString("scale = ");
    UART_SendFix1(g_scale);
    USART_SendString("  speed = ");
    UART_SendInt(g_speed);
    USART_SendString(" rpm\r\n");

    if (g_seg >= LAP_N)
    {
        USART_SendString("status: LAP DONE (");
        UART_SendInt(LAP_N);
        USART_SendString(" segments)\r\n");
    }
    else
    {
        USART_SendString("next   : segment ");
        UART_SendInt((uint32_t)g_seg + 1);
        USART_SendString("/");
        UART_SendInt(LAP_N);
        USART_SendString("  ");
        USART_SendString(g_run[g_seg].axis == AXIS_FWD ? "FWD " : "LEFT ");
        UART_SendIntS(g_run[g_seg].mm);
        USART_SendString(" mm\r\n");
    }

    for (i = g_seg; i < LAP_N; i++)     /* 剩下还要走多少毫米（缩放后） */
    {
        float m = (float)g_run[i].mm;
        if (m < 0.0f) { m = -m; }
        rest += m;
    }
    rest *= g_scale;

    USART_SendString("remain: ");
    UART_SendInt((uint32_t)(rest + 0.5f));
    USART_SendString(" mm\r\n");
    Lap_ShowPos();
}

/* 走第 idx 段。返回 1 = 正常走完，0 = 被 s 打断 */
static uint8_t Lap_DoSeg(uint8_t idx)
{
    const LapSeg *s   = &g_run[idx];
    float    mm       = (float)s->mm * g_scale;
    int8_t   sign     = (mm < 0.0f) ? -1 : +1;      /* 只记"往哪边"，正负映射后面再乘 */
    uint8_t  use_turn = (uint8_t)((s->axis == AXIS_LEFT) && g_piv_strafe);
    uint8_t  family   = MAP_FWD_AXIS;
    uint32_t pulses   = 0;
    uint32_t ms       = 0;
    uint8_t  ok;

    if (mm < 0.0f) { mm = -mm; }
    if (mm < 1.0f) { mm = 1.0f; }       /* 缩放太小也别发 0 脉冲 */

    USART_SendString("[seg ");
    UART_SendInt((uint32_t)idx + 1);
    USART_SendString("/");
    UART_SendInt(LAP_N);
    USART_SendString("] ");
    USART_SendString(s->axis == AXIS_FWD ? "FWD  " : (use_turn ? "LTURN" : "LEFT "));
    UART_SendIntS(s->mm);
    USART_SendString(" mm x");
    UART_SendFix1(g_scale);
    USART_SendString(" -> ");
    UART_SendInt((uint32_t)(mm + 0.5f));
    USART_SendString(" mm");

    if (use_turn)
    {
        /* 旋转方案：转 90 -> 沿车头直行 -> 转回 90（mm 全由直行完成，不做 comp） */
        USART_SendString("  via pivot+drive\r\n");
        ok = Mv_StrafeByTurn((float)sign * mm, "lap strafe");
    }
    else
    {
        if (s->axis == AXIS_FWD) { family = MAP_FWD_AXIS;  sign = (int8_t)(sign * MAP_FWD_SIGN); }
        else                     { family = MAP_LEFT_AXIS; sign = (int8_t)(sign * MAP_LEFT_SIGN); }

        /* 横移段顺带补偿前后漂移：左移配往前、右移配往后（正负跟着横移方向走） */
        {
            float dmm  = (float)sign * mm;
            float comp = (family == MAP_LEFT_AXIS) ? (dmm * g_comp_ratio) : 0.0f;
            pulses = Mv_Pos(family, dmm, comp);
        }
        ms = Mv_EstimateMs(pulses, ACC_DEFAULT, g_speed);

        USART_SendString("  pulses=");
        UART_SendInt(pulses);
        USART_SendString("  wait~");
        UART_SendInt(ms);
        USART_SendString("ms");
        ok = Mv_Wait(ms);
    }

    if (!ok) { USART_SendString("  INTERRUPTED\r\n"); return 0; }
    USART_SendString("  done\r\n");

    if (s->axis == AXIS_FWD) { g_py += (float)s->mm * g_scale; }   /* 记理论坐标 */
    else                     { g_px -= (float)s->mm * g_scale; }

    g_seg = (uint8_t)(idx + 1);
    Lap_ShowPos();
    return 1;
}

/* 自动跑完整圈 */
static void Lap_RunAll(void)
{
    USART_SendString("\r\n=== LAP START (press s any time to abort) ===\r\n");

    /* 起点并线：车摆在场地 A 标记上时，先并到中线起点（左移 60 + 前进 30）。
     * 车本来就停在中线上（a 关掉）就直接从第一段开始。 */
    if (g_align_mark)
    {
        USART_SendString("-- start align (A mark -> midline) --\r\n");
        if (!Lap_MarkShift(+1, "start->midline"))
        {
            USART_SendString("!! LAP ABORTED during start align\r\n");
            return;
        }
    }

    while (g_seg < LAP_N)
    {
        uint8_t c;

        if (!Lap_DoSeg(g_seg))
        {
            USART_SendString("!! LAP ABORTED at segment ");
            UART_SendInt((uint32_t)g_seg + 1);
            USART_SendString("\r\n");
            return;
        }

        delay_ms(300);                  /* 段间停一下，让机构站稳 */
        c = UART_GetByte();
        if (c == 's' || c == 'S' || c == 'z' || c == 'Z')
        {
            Mv_Stop_All();
            USART_SendString("!! LAP ABORTED by stop key\r\n");
            return;                     /* 被 s 打断：不做终点并线，直接停在原地 */
        }
    }

    /* 终点并线：8 段走完车停在中线起点，再反向做一次回到场地 A 标记 */
    if (g_align_mark)
    {
        USART_SendString("-- end align (midline -> A mark) --\r\n");
        if (!Lap_MarkShift(-1, "midline->A mark"))
        {
            USART_SendString("!! LAP ABORTED during end align\r\n");
            return;
        }
    }

    USART_SendString("\r\n=== LAP DONE: ");
    UART_SendInt(LAP_N);
    USART_SendString(" segments, back to start node ");
    UART_SendByte((uint8_t)g_node[g_start].name);
    USART_SendString(" ===\r\n");
    Lap_ShowPos();
}


/* ==========================================================================
 *  6 菜单 / 参数
 * ========================================================================== */

static void Show_Menu(void)
{
    USART_SendString("\r\n--- keys (send one char, no Enter needed) ---\r\n");
    USART_SendString("g       : run the WHOLE lap (8 segments, auto)\r\n");
    USART_SendString("a       : start/end align (A mark 60+30) = ");
    USART_SendString(g_align_mark ? "ON\r\n" : "OFF\r\n");
    USART_SendString("Enter/n : do the NEXT segment only (step by step)\r\n");
    USART_SendString("i       : show progress / position   c : reset to seg 1\r\n");
    USART_SendString("k       : pick START node (A B C D I H G F) + reset\r\n");
    USART_SendString("u / y   : scale +-0.1 (1.0 = full field, 0.1 = shrunk)\r\n");
    USART_SendString("s / z   : STOP ALL now  (works in the middle of a move)\r\n");
    USART_SendString("f / b   : manual forward / backward   ");
    UART_SendInt(g_dist);
    USART_SendString(" mm\r\n");
    USART_SendString("l / r   : manual left / right (per j) ");
    UART_SendInt(g_dist);
    USART_SendString(" mm\r\n");
    USART_SendString("j       : strafe mode = ");
    USART_SendString(g_piv_strafe ? "PIVOT (turn90 + drive + turn90)\r\n"
                                  : "raw mecanum strafe\r\n");
    USART_SendString("q / w   : pivot turn L / R 90 deg (in place)  m : flip OPEN-loop pivot dir (now ");
    UART_SendIntS((int32_t)g_pivot_inv);
    USART_SendString(")\r\n");
    USART_SendString("t / o   : spin 360 L / R (only needed when gyro is OFF)  ");
    USART_SendString("ppd = ");
    UART_SendFix2(g_ppd);
    USART_SendString("  accumulated heading = ");
    UART_SendFix1(g_pivot_ang);
    USART_SendString(" deg\r\n");
    USART_SendString("e       : gyro closed loop = ");
    USART_SendString(((g_closed && s_yawValid) ? "ON  (turns: gyro stops them)\r\n"
                                               : "OFF (turns: ppd pulses + timed wait)\r\n"));
    USART_SendString("x       : gyro zero here    ; : flip yaw sign   (yaw sign now ");
    UART_SendFix1(g_yaw_sign);
    USART_SendString(")\r\n");
    USART_SendString("5 / 6   : turn deadband -/+ 0.5 deg   (now ");
    UART_SendFix1(g_turn_db);
    USART_SendString(" deg; 太大 = 转不准，太小 = 到位后来回摆)\r\n");
    USART_SendString("+ / -   : manual dist +-50 mm    , / . : speed -/+10 rpm\r\n");
    USART_SendString("[ / ]   : ppm_x -0.05/+0.05   { / } : ppm_y -0.05/+0.05\r\n");
    USART_SendString("1 2 3 4 : single motor test (addr check)\r\n");
    USART_SendString("v / d   : enable / disable all    p : params    h : menu\r\n");
}

static void Show_Params(void)
{
    uint32_t pulses = (uint32_t)(g_ppm_y * (float)g_dist + 0.5f);

    USART_SendString("\r\n--- params ---\r\n");
    USART_SendString("dist  = ");
    UART_SendInt(g_dist);
    USART_SendString(" mm   speed = ");
    UART_SendInt(g_speed);
    USART_SendString(" rpm   acc = ");
    UART_SendInt(ACC_DEFAULT);
    USART_SendString("\r\nppm_x = ");
    UART_SendFix2(g_ppm_x);
    USART_SendString("  ppm_y = ");
    UART_SendFix2(g_ppm_y);
    USART_SendString("\r\nppd   = ");
    UART_SendFix2(g_ppd);
    USART_SendString("  comp  = ");
    UART_SendFix2(g_comp_ratio);
    USART_SendString("\r\nstrafe= ");
    USART_SendString(g_piv_strafe ? "PIVOT(turn90+drive+turn90)" : "raw strafe");
    USART_SendString("  pivot_dir = ");
    UART_SendIntS((int32_t)g_pivot_inv);
    USART_SendString("  heading = ");
    UART_SendFix1(g_pivot_ang);
    USART_SendString(" deg");
    USART_SendString("\r\ngyro  = ");
    if (!s_yawValid)
    {
        USART_SendString("NOT DETECTED -> closed loop off (turns use ppd pulses)\r\n");
    }
    else
    {
        USART_SendString(g_closed ? "closed loop ON " : "closed loop OFF");
        USART_SendString("  yaw = ");
        UART_SendFix1(RelAngle());
        USART_SendString("  target = ");
        UART_SendFix1(g_heading_cmd);
        USART_SendString("  turn_db = ");
        UART_SendFix1(g_turn_db);
        USART_SendString("  hold_db = ");
        UART_SendFix1(g_hold_db);
        USART_SendString("  yaw_sign = ");
        UART_SendFix1(g_yaw_sign);
        USART_SendString("\r\n");
    }
    USART_SendString("   scale = ");
    UART_SendFix1(g_scale);
    USART_SendString("\r\nalign = ");
    USART_SendString(g_align_mark ? "ON" : "OFF");
    USART_SendString("  (start: L");
    UART_SendInt(MARK_LAT_MM);
    USART_SendString(" + F");
    UART_SendInt(MARK_FWD_MM);
    USART_SendString(" mm, end: reversed)\r\n");
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
    USART_SendString("if the car veers/turns: no gyro hold yet, that is expected\r\n");
}


/* ==========================================================================
 *  7 按键处理
 * ========================================================================== */

static void Do_Command(uint8_t c)
{
    if (c == '\r' || c == '\n')
    {
        USART_SendString("\r\n[Enter] -> ");       /* 回车 = 走下一段（单段调试） */
        if (g_seg < LAP_N) { Lap_DoSeg(g_seg); delay_ms(300); }
        else               { USART_SendString("lap already finished (c = reset)\r\n"); }
        return;
    }

    /* 命令回传：先把收到的字符原样打出来，再执行。
     * 打了字却连 [cmd] 都没出现 -> 串口没通 / 波特率不对（不是按键问题） */
    USART_SendString("[cmd] '");
    UART_SendByte(c);
    USART_SendString("' -> ");

    switch (c)
    {
        case 'g': case 'G': Lap_RunAll();     break;

        case 'n': case 'N':
            if (g_seg < LAP_N) { Lap_DoSeg(g_seg); delay_ms(300); }
            else               { USART_SendString("lap already finished (c = reset)\r\n"); }
            break;

        case 'i': case 'I':
            USART_SendString("\r\n");
            Lap_Show();
            break;

        case 'c': case 'C': Lap_Reset();      break;

        case 'k': case 'K':                 /* 换起点：A-B-C-D-I-H-G-F 循环 */
        {
            Mv_Stop_All();
            do { g_start = (uint8_t)((g_start + 1) % 9); } while (g_start == NODE_E_INDEX);
            Lap_BuildRun();
            g_seg = 0;
            g_px  = 0.0f;
            g_py  = 0.0f;
            USART_SendString("\r\nstart node = ");
            UART_SendByte((uint8_t)g_node[g_start].name);
            USART_SendString("   field (");
            UART_SendIntS((int32_t)g_node[g_start].x);
            USART_SendString(", ");
            UART_SendIntS((int32_t)g_node[g_start].y);
            USART_SendString(")  -> g_lap[");
            UART_SendInt((uint32_t)g_node[g_start].seg);
            USART_SendString("]\r\n");
            USART_SendString("put the car AT this node, nose to field +Y, then g / Enter\r\n");
            Lap_ShowPos();
            break;
        }

        case 'u': case 'U':
            g_scale += SCALE_STEP;
            if (g_scale > SCALE_MAX) { g_scale = SCALE_MAX; }
            USART_SendString("scale = ");
            UART_SendFix1(g_scale);
            USART_SendString("\r\n");
            break;

        case 'y': case 'Y':
            g_scale -= SCALE_STEP;
            if (g_scale < SCALE_MIN) { g_scale = SCALE_MIN; }
            USART_SendString("scale = ");
            UART_SendFix1(g_scale);
            USART_SendString("\r\n");
            break;

        case 'f': case 'F': Mv_MoveDir(MAP_FWD_AXIS,  (int8_t)MAP_FWD_SIGN, "FWD");  break;
        case 'b': case 'B': Mv_MoveDir(MAP_FWD_AXIS,  (int8_t)(-MAP_FWD_SIGN), "BACK"); break;

        /* 左右：按 j 的模式走。PIVOT 模式 = 转90 + 直行 + 转回90 */
        case 'l': case 'L':
            if (g_piv_strafe) { Mv_StrafeByTurn((float)g_dist * (float)MAP_LEFT_SIGN, "left"); }
            else              { Mv_MoveDir(MAP_LEFT_AXIS, (int8_t)MAP_LEFT_SIGN, "LEFT"); }
            break;
        case 'r': case 'R':
            if (g_piv_strafe) { Mv_StrafeByTurn(-(float)g_dist * (float)MAP_LEFT_SIGN, "right"); }
            else              { Mv_MoveDir(MAP_LEFT_AXIS, (int8_t)(-MAP_LEFT_SIGN), "RIGHT"); }
            break;

        case 'j': case 'J':
            g_piv_strafe = (uint8_t)(g_piv_strafe ? 0 : 1);
            USART_SendString("strafe mode = ");
            USART_SendString(g_piv_strafe ? "PIVOT (turn90 + drive + turn90)\r\n"
                                          : "raw mecanum strafe\r\n");
            break;

        /* 起点/终点并线开关：默认开（车摆在 A 标记上 -> 左移60+前进30 并到中线） */
        case 'a': case 'A':
            g_align_mark = (uint8_t)(g_align_mark ? 0 : 1);
            USART_SendString("start/end align (A mark 60+30) = ");
            USART_SendString(g_align_mark ? "ON\r\n" : "OFF\r\n");
            break;

        case 'm': case 'M':
            g_pivot_inv = (int8_t)(-g_pivot_inv);
            USART_SendString("pivot direction flipped, g_pivot_inv = ");
            UART_SendIntS((int32_t)g_pivot_inv);
            USART_SendString("\r\n");
            break;

        /* ---- 陀螺仪闭环 ---- */
        case 'e': case 'E':
            if (g_closed)
            {
                g_closed = 0;
                USART_SendString("closed loop = OFF (turns use ppd pulses + timed wait)\r\n");
            }
            else if (!s_yawValid)
            {
                USART_SendString("gyro NOT detected at boot - closed loop stays OFF.\r\n");
                USART_SendString("check 5V/GND and module TX -> PD6, then press RST\r\n");
            }
            else
            {
                g_closed = 1;
                USART_SendString("closed loop = ON (gyro stops the turns, yaw = ");
                UART_SendFix1(RelAngle());
                USART_SendString(" deg)\r\n");
            }
            break;

        case 'x': case 'X':
            Gyro_Zero();
            break;

        case ';': case ':':
            g_yaw_sign = -g_yaw_sign;
            USART_SendString("yaw sign = ");
            UART_SendFix1(g_yaw_sign);
            USART_SendString("  (now yaw reads ");
            UART_SendFix1(RelAngle());
            USART_SendString(" deg)\r\n");
            break;

        case '5':
            g_turn_db -= 0.5f;
            if (g_turn_db < 0.5f) { g_turn_db = 0.5f; }
            USART_SendString("turn deadband = ");
            UART_SendFix1(g_turn_db);
            USART_SendString(" deg (tighter)\r\n");
            break;

        case '6':
            g_turn_db += 0.5f;
            if (g_turn_db > 10.0f) { g_turn_db = 10.0f; }
            USART_SendString("turn deadband = ");
            UART_SendFix1(g_turn_db);
            USART_SendString(" deg (looser)\r\n");
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
            break;

        case '.': case '>':
            if ((int)g_speed + SPEED_STEP > SPEED_MAX) { g_speed = SPEED_MAX; }
            else { g_speed = (uint16_t)(g_speed + SPEED_STEP); }
            USART_SendString("speed = ");
            UART_SendInt(g_speed);
            USART_SendString(" rpm\r\n");
            break;

        case '[':
            g_ppm_x -= PPM_STEP;
            if (g_ppm_x < 1.0f) { g_ppm_x = 1.0f; }
            USART_SendString("ppm_x = ");
            UART_SendFix2(g_ppm_x);
            USART_SendString("\r\n");
            break;

        case ']':
            g_ppm_x += PPM_STEP;
            USART_SendString("ppm_x = ");
            UART_SendFix2(g_ppm_x);
            USART_SendString("\r\n");
            break;

        case '{':
            g_ppm_y -= PPM_STEP;
            if (g_ppm_y < 1.0f) { g_ppm_y = 1.0f; }
            USART_SendString("ppm_y = ");
            UART_SendFix2(g_ppm_y);
            USART_SendString("\r\n");
            break;

        case '}':
            g_ppm_y += PPM_STEP;
            USART_SendString("ppm_y = ");
            UART_SendFix2(g_ppm_y);
            USART_SendString("\r\n");
            break;

        case '1': case '2': case '3': case '4':
            Mv_OneWheel((uint8_t)(c - '1'));
            break;

        case 'q': case 'Q': Pivot_Rotate( 90.0f, "PIVOT L90");  break;
        case 'w': case 'W': Pivot_Rotate(-90.0f, "PIVOT R90");  break;
        case 't': case 'T': Pivot_Rotate( 360.0f, "SPIN L360 (calib)"); break;
        case 'o': case 'O': Pivot_Rotate(-360.0f, "SPIN R360 (calib)"); break;

        case 'v': case 'V':
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

        default:
            USART_SendString("?  (h = help)\r\n");
            break;
    }
}


/* ==========================================================================
 *  8 主程序
 * ========================================================================== */

int main(void)
{
    uint8_t i;

    delay_init(168);    /* 延迟函数必须先初始化（主频 168MHz） */
    MyUART_Init();
    Gyro_Init();        /* 陀螺仪 + 转向闭环：找模块、量漂移、把开机朝向定为 0 度
                         * 这几秒（约 2.5s）别动车，否则漂移标定会不准 */
    board_init();       /* 初始化 CAN1：PA11 / PA12，500K，扩展帧 */
    Lap_BuildRun();     /* 按起点节点旋转整圈表 -> g_run[] */

    USART_SendString("\r\n== lap loop: FWD edges + strafe-by-pivot (j = switch) ==\r\n");
    USART_SendString("align: start = LEFT 60 + FWD 30 (A mark -> midline), end = reversed (a = on/off)\r\n");
    USART_SendString("START node = ");
    UART_SendByte((uint8_t)g_node[g_start].name);
    USART_SendString("  field=(");
    UART_SendIntS((int32_t)g_node[g_start].x);
    USART_SendString(", ");
    UART_SendIntS((int32_t)g_node[g_start].y);
    USART_SendString(")  HEADING = field +Y   (k = change start node)\r\n");
    USART_SendString("nodes (midline corners): A(2190,200) B(2190,1200) C(2190,2060) D(1200,2060)\r\n");
    USART_SendString("                         E(1200,1200 center) I(340,2060) H(340,1200) G(340,200) F(1200,200)\r\n");
    USART_SendString("A-B-C-D-I-H-G-F-A, 8 segments, 7420 mm total\r\n");
    USART_SendString("strafes now run as: pivot L90 -> drive -> pivot R90 (nose back to +Y)\r\n");
    USART_SendString("TURNS: gyro closed loop = ");
    USART_SendString(((g_closed && s_yawValid) ? "ON  (ppd no longer matters)\r\n"
                                               : "OFF (ppd = 18.0 is still a guess!)\r\n"));
    USART_SendString("STRAIGHT: heading hold every ");
    UART_SendInt((uint32_t)HOLD_CHUNK_MM);
    USART_SendString(" mm, corrects when off by > ");
    UART_SendFix1(g_hold_db);
    USART_SendString(" deg\r\n");
    USART_SendString("first run: press y x9 (scale 0.1), then Enter per segment\r\n");

    /* 上电先把底盘四台使能，这样按 f 就能动 */
    for (i = 0; i < MOTOR_NUM; i++)
    {
        Emm_V5_En_Control(g_addr[i], true, false);
        delay_ms(1);
    }

    /* 机械臂两根轴也上电使能，跑的时候不会晃 */
#if ARM_HOLD
    Emm_V5_En_Control(ADDR_ARM_PUSH, true, false);
    delay_ms(1);
    Emm_V5_En_Control(ADDR_ARM_LIFT, true, false);
    delay_ms(1);
    USART_SendString("arm axes 5 / 7 enabled (hold)\r\n");
#endif

    Show_Menu();
    Show_Params();
    USART_SendString("\r\nready. standing at start node ");
    UART_SendByte((uint8_t)g_node[g_start].name);
    USART_SendString(", segment 1, scale = ");
    UART_SendFix1(g_scale);
    USART_SendString("\r\n");
    Lap_ShowPos();

    while (1)
    {
        uint8_t c = UART_GetByte();
        if (c != 0) { Do_Command(c); }
    }
}
