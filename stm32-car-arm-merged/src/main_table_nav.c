/* 小车控制入口：硬件初始化、串口命令、单段标定、完整路线调度。
 * 日常调参：car_config.h；道路动作：structure.h；全程顺序：full_route.h。
 * 主循环入口：firmware_step()；全程状态推进：run_update()。
 * runimu发车短段横移，其余按航向行驶；估时不确认实际到位或车身角度。
 */
#ifdef NAV_ENTRY_HOST_TEST
#include "../tests/entry_host_io.h"
#else
#include "stm32f4xx.h"
#include "Emm_V5.h"
#include "delay.h"
#endif
#include "navigation.h"
#include "motion.h"
#include "full_route.h"
#include "car_config.h"
#include "heading_sensor.h"
#include "heading_control.h"
#include "arm_control.h"
#include "arm_config.h"
#include "station_job.h"
#include "station_config.h"
#include "vision_link.h"
#include "hmi_status.h"
#include "can_tx.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#define RX_SIZE 128u
#define LINE_SIZE 64u

/* 模式数值保留0/1/2，test_mode仍为uint8_t，便于沿用Keil观察和现有测试。 */
#define MODE_NAVIGATION        0u
#define MODE_TRANSLATION_TEST  1u
#define MODE_ROTATION_TEST     2u

/* Emm V5 CAN回复协议；属于固定解析规则，不是日常可调参数。 */
#define MOTOR_COUNT                 4u
#define ALL_MOTORS_MASK          0x0fu
#define CAN_EXTENDED_FRAME_IDE       4u
#define CAN_PACKET_NUMBER_MASK   0xffu
#define CAN_MOTOR_ADDRESS_SHIFT      8u
#define CAN_REPLY_BYTES              3u
#define CAN_REPLY_MAX_BYTES          7u
#define CAN_BUS_OFF_FLAG             4u
#define MOTOR_POSITION_COMMAND   0xfdu
#define MOTOR_STATUS_COMMAND     0x3au
#define MOTOR_SPEED_COMMAND      0x35u
#define MOTOR_ERROR_COMMAND      0x37u
#define MOTOR_CURRENT_POSITION_COMMAND 0x36u
#define MOTOR_VELOCITY_COMMAND    0xf6u
#define MOTOR_REPLY_CHECK_BYTE   0x6bu
#define MOTOR_POSITION_ACK           2u
#define MOTOR_ARRIVAL_NOTICE     0x9fu
#define MOTOR_FLAG_ENABLED           1u
#define MOTOR_FLAG_ARRIVED           2u
#define MOTOR_FLAG_STALLED           4u
#define MOTOR_FLAG_PROTECTED         8u
/* ===== 1. 全局状态：Keil Watch 窗口可观察这些变量 ===== */
static struct {uint32_t start_ms,elapsed_ms;uint8_t running,started;} screen_clock;
static volatile uint32_t clock_ms;       /* 系统毫秒计时，TIM5 中断更新。 */
#define SCANNER_RX_SIZE 128u
typedef enum {SCANNER_OFF, SCANNER_BOOT, SCANNER_SET_ENDING, SCANNER_SET_CONTINUOUS,
              SCANNER_READY, SCANNER_FAULT} ScannerState;
static struct {
    ScannerState state;
    uint32_t baud, phase_ms, rx_bytes, results;
    uint32_t uart_errors, overflows;
    uint8_t tx_offset, ack_offset, length, discard;
    char result[128];
} scanner;
static volatile uint8_t scanner_rx[SCANNER_RX_SIZE], scanner_head, scanner_tail;
static volatile uint8_t scanner_rx_error, scanner_rx_overflow;
static struct {
    uint8_t candidate, stable;
    uint32_t changed_ms;
} start_button;
static volatile uint8_t rx_head;         /* 串口环形缓冲的写入位置。 */
static volatile uint8_t rx_tail;         /* 串口环形缓冲的读取位置。 */
static volatile uint8_t stop_pending;    /* 收到 ! 或输入溢出后，主循环优先停车。 */
static volatile uint8_t rx_overflow;
static volatile uint8_t rx_buffer[RX_SIZE];
static char line[LINE_SIZE];
static uint8_t line_length;
static uint8_t pose_set;                 /* 人工导航的真实起点是否已由 pose 确认。 */
static uint16_t speed_rpm;               /* 人工导航速度；0 表示使用道路表速度。 */
static uint8_t test_mode;                /* 0：导航/全程；1：平移标定；2：转向标定。 */
static uint16_t test_distance = CAR_TEST_DISTANCE_MM;
static uint16_t test_rpm = CAR_TEST_RPM;
static uint16_t turn_rpm = CAR_TURN_RPM;
static Motion drive;                    /* 当前单段运动状态、脉冲、估时、命令代号。 */
static Navigation nav;                  /* 人工查表导航的节点、道路、错误状态。 */
static NavState reported_nav = NAV_FAULT;
static MotionState reported_motion = MOTION_FAULT;
typedef enum
{
    RUN_IDLE,       /* 未启用全程模式。 */
    RUN_PREFLIGHT,  /* 查询四个驱动器，检查使能和堵转状态。 */
    RUN_MOVING,     /* IMU航向与编码器距离闭环平移，持续检查四轮反馈。 */
    RUN_DONE,       /* 全部指令按电机反馈完成，真实车身位置仍需现场检查。 */
    RUN_STOPPED,    /* 操作者停止，不自动续跑。 */
    RUN_FAULT,     /* 通信/执行异常，不自动重试。 */
    RUN_ALIGN,     /* runimu：停车后用新角度对齐航向。 */
    RUN_OBSTACLE_STOP,
    RUN_WAIT_OBSTACLE,
    RUN_STATION
} RunState;
typedef enum {
    VELOCITY_TURN, VELOCITY_BASELINE, VELOCITY_FORWARD, VELOCITY_STOP_FORWARD
} VelocityPhase;

typedef struct {
    BodyAxis axis;
    RoadHeading heading;
    int8_t sign;
    uint16_t mm;
} RunLeg;
typedef struct
{
    RunState state;
    uint8_t plan;             /* 0：A 起点路线；1：C 起点路线。 */
    uint8_t edge;             /* full_route_nodes 中当前道路起点的下标。 */
    uint8_t segment;          /* 当前道路内部的运动段下标，从 0 开始。 */
    uint8_t job;              /* 已经过的任务点数量，0～8。 */
    uint16_t steps;            /* 逻辑平移段及转向计数；直线穿越不增加一次启停。 */
    RoadHeading heading;     /* 已通过IMU对齐的地图朝向；地面实际姿态仍需检查。 */
    RoadHeading target_heading;
    RoadHeading initial_heading; /* runimu：A右上朝扫码为+Y，C右下朝扫码为-Y。 */
    uint8_t turning;         /* 当前动作是否为旋转；完成前不改变heading。 */
    uint8_t acks;             /* 电机1～4对应bit0～3，15表示四轮均回复速度ACK。 */
    uint8_t ready;            /* 四轮新鲜反馈掩码；15不等于地面实际到位。 */
    uint8_t pending;          /* 正在等待回复的电机地址；0 表示无等待。 */
    uint32_t warning_ms;
    uint8_t warning_seen, alignment_waiting;
    uint32_t query_ms;        /* 本次状态请求的发出时间。 */
    uint32_t last_query_ms;
    uint32_t segment_ms;      /* 当前段开始时间，用于 ACK 超时判断。 */
    uint16_t distance_mm;     /* 已提交的理论距离总和，不是实测行程。 */
    uint16_t completed_roads; /* 本轮已按反馈完整走完的无向道路；部分路段不计入。 */
    uint8_t imu_mode;         /* 已建立闭环会话，供诊断和中断采样故障检查。 */
    uint8_t turn_only, translation_only;
    uint8_t station_enabled, station_phase, scan_turn_stage, vision_debug;
    uint8_t station_departing, station_speed_logged; /* 工位离站预检范围及每轮日志去重。 */
    uint8_t departure_overlap; /* 末尾回正与离站转弯/平移重叠；两轴已实际135/0。 */
    uint8_t station_shift, station_relock; /* 粗加工子动作及完成后的四轮互锁交接。 */
    uint8_t transition_ready; /* 已完成道路或工位持续确认的航向/停车状态，只向下一段传递一次。 */
    BodyAxis travel_axis;     /* 连续平移轴：默认前后；仅授权段/手动l/r使用横移。 */
    uint16_t lateral_rpm;
    uint8_t vector_move;      /* 仅MOVEXY工位子动作：前后/横移目标独立闭环，同一批四轮速度。 */
    int32_t vector_target_mm[2], vector_progress_mm[2]; /* 0前后、1向左，均为有符号毫米。 */
    int32_t vector_mrpm[2];
    uint32_t vector_profile_ms[2];
    int8_t vector_profile_sign[2], vector_wheel_sign[4];
    uint8_t station_align_pending; /* 工位已走到，但尚未完成右侧朝向和停稳确认。 */
    int8_t travel_sign;       /* 平移方向：+1前/左，-1后/右；行程仍为正数。 */
    uint16_t remaining_mm, action_mm;
    int16_t yaw_zero_cdeg;
    uint16_t turn_rpm;
    HeadingControl heading_control;
    VelocityPhase velocity_phase;
    uint8_t velocity_query_index, velocity_waiting, velocity_sent;
    uint32_t velocity_ms, control_ms, phase_ms, zero_ms, stopped_ms, log_ms;
    uint32_t baseline_after_ms; /* 编码器基线必须采样于已确认的停稳区间内。 */
    uint32_t wheel_start_ms[4];
    int16_t wheel_rpm[4];
    int64_t encoder_start[4];
    int8_t encoder_sign[4];
    int64_t encoder_previous_delta[4];
    uint32_t encoder_sample_ms[4];
    uint8_t encoder_ready;    /* 四轮方向确认掩码；同一直行串连续通过节点时保留。 */
    int32_t progress_mm;
    int32_t encoder_offset_mm; /* 同一直行串内已通过的段长；不中断编码器基线。 */
    uint16_t continuation_mm, blocked_roads;
    RunLeg road_path[ROAD_MAX_SEGMENTS]; /* 当前道路已完成的车身动作，供反序退回。 */
    uint8_t road_path_count, return_index, obstacle_phase, obstacle_leg_active, obstacle_node, obstacle_preflight;
    int32_t forward_mrpm;      /* 平移速度斜坡，0.001RPM；仅在可提交速度批次时推进。 */
    int8_t profile_sign;      /* 平移斜坡当前方向；横移过冲先减到零再反向。 */
    uint32_t forward_profile_ms, control_dt_ms;
    int16_t forward_ref_rpm, forward_cmd_rpm, turn_ref_rpm, turn_cmd_rpm;
    struct {
        uint32_t sequence, samples, min_ms, max_ms;
        int32_t min_cdeg, max_cdeg, min_mm, max_mm;
        uint8_t seen;
    } heading_window;
    uint8_t pending_command;
    RoadPlan road;            /* structure.h 中取出的当前有向道路参数。 */
} FullRouteState;
static FullRouteState run;
static FullRouteState station_parent; /* 索引子动作完成后恢复原路线现场，不推进节点/任务。 */
static HeadingSensor heading_sensor;
static volatile uint8_t can_rx_fault, imu_uart_fault;
typedef struct {
    uint32_t replies, status_replies, action_status_replies, arrival_notices, last_ms, status_ms;
    uint8_t last_command, last_value, status_flags;
    uint32_t error_counts, error_ms, speed_ms;
    int32_t speed_rpm;
    uint8_t error_valid, error_negative, speed_valid;
    int64_t position;
    uint32_t position_ms, position_changed_ms;
    uint8_t position_valid;
} MotorFeedback;
static MotorFeedback motor_feedback[MOTOR_COUNT];
static uint32_t invalid_can_replies;
static volatile uint32_t rejected_can_frames;
static struct {
    uint32_t done_ms;
    int16_t done_yaw_cdeg;
    uint8_t active, next;
} turn_postcheck;

static HeadingSample heading_snapshot(void)
{
    HeadingSample sample;
    __disable_irq();
    sample = heading_sensor.sample;
    __enable_irq();
    sample.yaw_cdeg = heading_wrap_cdeg((int32_t)sample.yaw_cdeg * CAR_IMU_YAW_SIGN);
    return sample;
}


#define RUN_RX_SIZE 16u
typedef struct
{
    uint32_t id, received_ms;
    uint8_t length, data[CAN_REPLY_MAX_BYTES];
} RunCanFrame;
static volatile RunCanFrame run_rx[RUN_RX_SIZE];
static volatile uint8_t run_active, run_rx_head, run_rx_tail, run_rx_overflow;
/* 1等待四轮停稳，2机械机构独占，3故障锁定；仅显式命令恢复。 */
static struct {
    volatile uint8_t state;
    uint8_t query, requested[4], seen[4], settling;
    uint32_t started_ms, poll_ms, stable_ms, query_ms[4][2];
} arm_lock;
void route_test_can_error(uint8_t flags)
{
    arm_can_error(flags);
    if (run_active || arm_lock.state) can_rx_fault |= flags;
}
/* USART2中断只提交字节；更换型号时可在适配器中发布真实的新航向采样。 */
int route_test_imu_rx(uint8_t byte, uint8_t uart_error)
{
#if CAR_IMU_UART_ENABLED
    if (uart_error) {
        heading_sensor_invalidate(&heading_sensor);
        if (run_active && run.imu_mode) imu_uart_fault = 1;
        return 1;
    }
    heading_sensor_byte(&heading_sensor, byte, clock_ms);
    return 1;
#else
    (void)byte;
    (void)uart_error;
    return 0;
#endif
}

static void run_status(void);
static void run_velocity_update(void);
static void run_begin_alignment(void);
static int run_velocity_stopped(void);
static void arm_lock_begin(void);
static void run_resume_after_station(void);
static void run_station_update(void);
/* ===== 2. CAN 接收队列：中断只收报文，全程主循环负责解析 ===== */
/* CAN IRQ只复制短报文；主循环处理应答，不在IRQ内调用运动函数。 */
void route_test_can_rx(uint32_t id, uint8_t ide, uint8_t rtr, uint8_t length, const uint8_t *data)
{
    uint8_t i, next;
    arm_can_rx(id, ide, rtr, length, data, clock_ms);
    if ((id >> CAN_MOTOR_ADDRESS_SHIFT) == ARM_LIFT_ADDRESS
        || (id >> CAN_MOTOR_ADDRESS_SHIFT) == ARM_SLIDE_ADDRESS) return;
    if (!run_active && !arm_lock.state)
        return;
    if (!data || ide != CAN_EXTENDED_FRAME_IDE || rtr != 0
        || !(length == CAN_REPLY_BYTES
             || (length == 5u && data[0] == MOTOR_SPEED_COMMAND)
             || (length == 7u && (data[0] == MOTOR_ERROR_COMMAND
                                 || data[0] == MOTOR_CURRENT_POSITION_COMMAND)))
        || (id & CAN_PACKET_NUMBER_MASK)
        || (id >> CAN_MOTOR_ADDRESS_SHIFT) < 1 || (id >> CAN_MOTOR_ADDRESS_SHIFT) > MOTOR_COUNT) {
        rejected_can_frames++;
        return;
    }
    next = (uint8_t)((run_rx_head + 1u) % RUN_RX_SIZE);
    if (next == run_rx_tail)
    {
        run_rx_overflow = 1;
        return;
    }
    run_rx[run_rx_head].id = id;
    run_rx[run_rx_head].received_ms = clock_ms;
    run_rx[run_rx_head].length = length;
    for (i = 0; i < length; i++)
        run_rx[run_rx_head].data[i] = data[i];
    run_rx_head = next;
}

static int run_read_frame(RunCanFrame *frame)
{
    uint8_t i;
    if (run_rx_head == run_rx_tail)
        return 0;
    frame->id = run_rx[run_rx_tail].id;
    frame->received_ms = run_rx[run_rx_tail].received_ms;
    frame->length = run_rx[run_rx_tail].length;
    for (i = 0; i < frame->length; i++)
        frame->data[i] = run_rx[run_rx_tail].data[i];
    run_rx_tail = (uint8_t)((run_rx_tail + 1u) % RUN_RX_SIZE);
    return 1;
}

static void run_clear_rx(void)
{
    __disable_irq();
    run_rx_tail = run_rx_head;
    /* 活动动作的丢帧标志必须保留到故障处理，不能被下一动作清队列吞掉。 */
    if (!run_active) {
        run_rx_overflow = 0;
        can_rx_fault = 0;
    }
    __enable_irq();
}

/* 运行中仅清接收溢出；保留发送失败和bus-off标志，不能吞掉真实硬故障。 */
static void run_recover_receive(void)
{
    __disable_irq();
    run_rx_tail=run_rx_head;run_rx_overflow=0;can_rx_fault&=(uint8_t)~1u;
    __enable_irq();
    run.pending=0;
}

/* ===== 3. 串口收发与毫秒计时 ===== */
static void send_byte(uint8_t b)
{
#ifdef NAV_ENTRY_HOST_TEST
    entry_host_send_byte(b);
#else
    USART_SendData(USART3, b);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
    {
    }
#endif
}

static void text(const char *s)
{
    while (*s)
        send_byte((uint8_t) * s++);
}
void arm_output(const char *message) { text(message); }

static uint8_t start_button_pressed(void)
{
#ifdef NAV_ENTRY_HOST_TEST
    return entry_host_start_button_pressed() != 0;
#else
    return GPIO_ReadInputDataBit(CAR_START_BUTTON_GPIO, CAR_START_BUTTON_PIN) != 0;
#endif
}

static void start_button_init(void)
{
#ifndef NAV_ENTRY_HOST_TEST
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(CAR_START_BUTTON_GPIO_CLOCK, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = CAR_START_BUTTON_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(CAR_START_BUTTON_GPIO, &gpio);
#endif
    /* 上电时已按住不算新按下；松开再按才请求启动，避免上电自动发车。 */
    start_button.candidate = start_button.stable = start_button_pressed();
    start_button.changed_ms = clock_ms;
}

static uint8_t start_button_update(void)
{
    uint8_t pressed = start_button_pressed();
    if (pressed != start_button.candidate) {
        start_button.candidate = pressed;
        start_button.changed_ms = clock_ms;
    }
    if (pressed != start_button.stable
        && (uint32_t)(clock_ms - start_button.changed_ms) >= CAR_START_BUTTON_DEBOUNCE_MS) {
        start_button.stable = pressed;
        if (pressed) {text("[button] START pressed\r\n");return 1;}
    }
    return 0;
}

static void number(uint32_t value)
{
    char buf[11];
    uint8_t count = 0;
    do
    {
        buf[count++] = (char)('0' + value % 10u);
        value /= 10u;
    }
    while (value);
    while (count)
        send_byte((uint8_t) buf[-- count]);
}

static void signed_number(int32_t value)
{
    if (value < 0)
        text("-");
    number(value < 0 ? 0u - (uint32_t)value : (uint32_t)value);
}

/* 工位轮速只作诊断；道路/手动会话仍使用原停稳检查。 */
static int station_speed_log_only(void)
{
    return run.station_enabled && (run.state==RUN_STATION || run.station_shift || run.station_departing);
}
static void station_log_wheel_speed(uint8_t wheel, const char *stage)
{
    uint8_t bit=(uint8_t)(1u<<wheel);
    int32_t rpm=motor_feedback[wheel].speed_rpm;
    if (!rpm) {run.station_speed_logged &= (uint8_t)~bit;return;}
    if (run.station_speed_logged & bit) return;
    run.station_speed_logged |= bit;
    text("[station wheel] stage=");text(stage);text(" wheel=");number(wheel+1u);
    text(" speed_rpm=");signed_number(rpm);text("; logged only\r\n");
}

/* XM1601按照片中的指令设置CRLF，再切连续模式；不保存Flash或恢复出厂。 */
static const uint8_t scanner_commands[2][9] = {
    {0x7e,0x00,0x08,0x01,0x00,0x60,0x21,0x4b,0xf0},
    {0x7e,0x00,0x08,0x01,0x00,0x00,0xd6,0xdf,0x22}
};
static const uint8_t scanner_ack[7] = {0x02,0x00,0x00,0x01,0x00,0x33,0x31};

/* USART1中断仅入队，字符串处理与输出留在主循环。 */
int route_test_scanner_rx(uint8_t byte, uint8_t uart_error)
{
    uint8_t next = (uint8_t)((scanner_head + 1u) % SCANNER_RX_SIZE);
    if (uart_error) scanner_rx_error = 1;
    else if (next == scanner_tail) scanner_rx_overflow = 1;
    else {
        scanner_rx[scanner_head] = byte;
        scanner_head = next;
    }
    return 1;
}

static void scanner_init(uint32_t baud)
{
#ifndef NAV_ENTRY_HOST_TEST
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef irq;
    /* PC0是低有效触发输入，先置高再切开漏输出，避免初始化产生低脉冲。 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_SetBits(GPIOC, GPIO_Pin_0);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_0; gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_OD; gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpio);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    USART_Cmd(USART1, DISABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF; gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
    USART_StructInit(&uart);
    uart.USART_BaudRate = baud;
    USART_Init(USART1, &uart);
    (void)USART1->SR; (void)USART_ReceiveData(USART1);
#else
    entry_host_scanner_init(baud);
#endif
    __disable_irq();
    memset(&scanner, 0, sizeof(scanner));
    scanner_head = scanner_tail = scanner_rx_error = scanner_rx_overflow = 0;
    scanner.state = SCANNER_BOOT; scanner.baud = baud; scanner.phase_ms = clock_ms;
    __enable_irq();
#ifndef NAV_ENTRY_HOST_TEST
    irq.NVIC_IRQChannel = USART1_IRQn;
    irq.NVIC_IRQChannelPreemptionPriority = 1;
    irq.NVIC_IRQChannelSubPriority = 1;
    irq.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&irq);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
#endif
    text("[scanner] BOOT continuous requested; baud="); number(baud); text("\r\n");
}

static int scanner_tx_ready(void)
{
#ifdef NAV_ENTRY_HOST_TEST
    return entry_host_scanner_tx_ready();
#else
    return USART_GetFlagStatus(USART1, USART_FLAG_TXE) != RESET;
#endif
}

static void scanner_send_byte(uint8_t byte)
{
#ifdef NAV_ENTRY_HOST_TEST
    entry_host_scanner_send_byte(byte);
#else
    USART_SendData(USART1, byte);
#endif
}

static void scanner_status(void)
{
    static const char *names[] = {"OFF","BOOT","SET_ENDING","SET_CONTINUOUS","READY","FAULT"};
    text("[scanner] state="); text(names[scanner.state]);
    text(" baud="); number(scanner.baud);
    text(" rx_bytes="); number(scanner.rx_bytes);
    text(" results="); number(scanner.results);
    text(" uart_errors="); number(scanner.uart_errors);
    text(" overflows="); number(scanner.overflows); text("\r\n");
}

static void scanner_update(void)
{
    uint8_t budget = 32;
    if (scanner.state == SCANNER_OFF) return;
    if (scanner_rx_error || scanner_rx_overflow) {
        __disable_irq();
        scanner.uart_errors += scanner_rx_error != 0;
        scanner.overflows += scanner_rx_overflow != 0;
        scanner_rx_error = scanner_rx_overflow = 0; scanner_tail = scanner_head;
        __enable_irq();
        scanner.length = scanner.ack_offset = 0; scanner.discard = 1;
        if (scanner.state != SCANNER_READY) scanner.state = SCANNER_FAULT;
        text("[scanner] RX error/overflow; partial data discarded\r\n");
    }
    while (budget-- && scanner_tail != scanner_head) {
        uint8_t byte = scanner_rx[scanner_tail];
        scanner_tail = (uint8_t)((scanner_tail + 1u) % SCANNER_RX_SIZE);
        scanner.rx_bytes++;
        if ((scanner.state == SCANNER_SET_ENDING || scanner.state == SCANNER_SET_CONTINUOUS)
            && scanner.tx_offset == 9u) {
            if (byte == scanner_ack[scanner.ack_offset]) scanner.ack_offset++;
            else scanner.ack_offset = byte == scanner_ack[0] ? 1u : 0u;
            if (scanner.ack_offset == sizeof(scanner_ack)) {
                scanner.ack_offset = scanner.tx_offset = 0;
                scanner.phase_ms = clock_ms;
                if (scanner.state == SCANNER_SET_ENDING) {
                    scanner.state = SCANNER_SET_CONTINUOUS;
                    text("[scanner] CRLF acknowledged\r\n");
                } else {
                    scanner.state = SCANNER_READY;
                    text("[scanner] READY continuous acknowledged\r\n");
                }
            }
        } else if (scanner.state == SCANNER_READY) {
            if (byte == '\n') {
                if (!scanner.discard && scanner.length && scanner.result[scanner.length - 1u] == '\r') {
                    scanner.result[--scanner.length] = 0;
                    if (scanner.length) {
                        text("[scan] "); text(scanner.result); text("\r\n"); scanner.results++;
                        /* 首次抵达B即接码，包含转向过程；首个有效任务锁定，闲置/回程不重装。 */
                        if (run.station_enabled && run.job==0u && run.edge==1u
                            && ((run_active && run.scan_turn_stage==1u) || (!run_active && run.state==RUN_STATION))
                            && !stop_pending && !station_task_ready()) {
                            char task_line[sizeof(scanner.result) + 6u];
                            snprintf(task_line,sizeof(task_line),"task %s",scanner.result);
                            (void)station_command(task_line);
                        }
                    }
                }
                scanner.length = scanner.discard = 0;
            } else if (!scanner.discard) {
                if (scanner.length >= sizeof(scanner.result) - 1u || (byte < 0x20u && byte != '\r')) {
                    scanner.discard = 1;
                    text("[scanner] invalid/overlong result discarded\r\n");
                } else scanner.result[scanner.length++] = (char)byte;
            }
        }
    }
    if (scanner.state == SCANNER_BOOT && (uint32_t)(clock_ms - scanner.phase_ms) >= CAR_SCANNER_BOOT_MS) {
        scanner.state = SCANNER_SET_ENDING; scanner.phase_ms = clock_ms;
    }
    if (scanner.state == SCANNER_SET_ENDING || scanner.state == SCANNER_SET_CONTINUOUS) {
        if ((uint32_t)(clock_ms - scanner.phase_ms) >= CAR_SCANNER_ACK_MS) {
            scanner.state = SCANNER_FAULT;
            text("[scanner] configuration timeout; check baud/wiring; qr for status\r\n");
        } else if (scanner.tx_offset < 9u && scanner_tx_ready())
            scanner_send_byte(scanner_commands[scanner.state == SCANNER_SET_CONTINUOUS][scanner.tx_offset++]);
    }
}

static void queue_byte(uint8_t value)
{
    uint8_t next = (uint8_t)((rx_head + 1u) % RX_SIZE);
    if (value == '!')
    {
        stop_pending = 1;
        return;
    }
    if (next == rx_tail)
    {
        rx_overflow = 1;
        stop_pending = 1;
        return;
    }
    rx_buffer[rx_head] = value;
    rx_head = next;
}

#ifndef NAV_ENTRY_HOST_TEST
static void uart_init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef irq;
    /* 当前载板串口：PB10=USART3_TX，PB11=USART3_RX，USART3挂在APB1。 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
    USART_StructInit(&uart);
    uart.USART_BaudRate = CAR_DEBUG_UART_BAUD;
    USART_Init(USART3, &uart);
    irq.NVIC_IRQChannel = USART3_IRQn;
    irq.NVIC_IRQChannelPreemptionPriority = 1;
    irq.NVIC_IRQChannelSubPriority = 0;
    irq.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&irq);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART3, ENABLE);
}

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        queue_byte((uint8_t) USART_ReceiveData(USART3));
    }
    if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
    {
        (void) USART_ReceiveData(USART3);
        rx_overflow = 1;
        stop_pending = 1;
    }
}

#endif
static int read_byte(uint8_t *value)
{
    if (rx_tail == rx_head)
        return 0;
    * value = rx_buffer[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1u) % RX_SIZE);
    return 1;
}

#ifndef NAV_ENTRY_HOST_TEST
static void clock_init_ms(void)
{
    RCC_ClocksTypeDef clocks;
    TIM_TimeBaseInitTypeDef timer;
    NVIC_InitTypeDef irq;
    uint32_t timer_hz;
    RCC_GetClocksFreq(&clocks);
    timer_hz = clocks.PCLK1_Frequency;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0)
        timer_hz *= 2u;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = (uint16_t)(timer_hz / 1000000u - 1u);
    timer.TIM_Period = 999;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &timer);
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    irq.NVIC_IRQChannel = TIM5_IRQn;
    irq.NVIC_IRQChannelPreemptionPriority = 2;
    irq.NVIC_IRQChannelSubPriority = 0;
    irq.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&irq);
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
}

void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
    {
        clock_ms++;
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    }
}

#endif
/* ===== 4. 电机驱动接口：提交位置命令、同步启动、停车 ===== */
static int motor_position(void *user, uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc, uint32_t pulses)
{
    (void) user;
    if (stop_pending || can_rx_fault || (run_active && run_rx_overflow))
        return 0;
    /* 原驱动为void接口，提交命令不等于收到CAN应答或电机到位。 */
    Emm_V5_Pos_Control(addr, dir, rpm, acc, pulses, false, true);
    delay_ms(1);
    return !can_rx_fault && !(run_active && run_rx_overflow);
}

static int motor_sync(void *user)
{
    (void) user;
    if (stop_pending || can_rx_fault || (run_active && run_rx_overflow))
        return 0;
    Emm_V5_Synchronous_motion(0);
    return !can_rx_fault && !(run_active && run_rx_overflow);
}

static void motor_stop(void *user, uint8_t addr)
{
    (void) user;
    can_tx_cancel(addr);
    Emm_V5_Stop_Now(addr, false);
    delay_ms(1);
    Emm_V5_Vel_Control(addr, 0, 0, CAR_STOP_ACCELERATION, false);
    delay_ms(1);
}

static int port_start(void *user, MotionParameters parameters, uint32_t now_ms)
{
    Motion *motion = (Motion *) user;
    int result = motion_start(motion, parameters, now_ms);
    text("[segment] ");
    text(parameters.axis == BODY_FORWARD ? "forward " : "left ");
    if (parameters.mm < 0)
        text("-");
    number((uint32_t)(parameters.mm < 0 ? -parameters.mm : parameters.mm));
    text(" mm, rpm=");
    number(parameters.rpm);
    text(" acc=");
    number(parameters.acceleration);
    text(" id=");
    number(motion->generation);
    text(" estimate=");
    number(motion->estimate_ms);
    text("ms trim_forward=");
    signed_number(motion->forward_trim_mm);
    text(" mm (feedforward, not arrival)\r\n");
    return result;
}

static NavMotionEvent port_poll(void *user, uint32_t now_ms)
{
    MotionState state = motion_update((Motion *) user, now_ms);
    if (state == MOTION_DONE)
        return NAV_MOTION_DONE;
    if (state == MOTION_FAULT)
        return NAV_MOTION_FAULT;
    return NAV_MOTION_BUSY;
}

static void port_stop(void *user)
{
    motion_stop((Motion *) user);
}

/* ===== 5. 完整路线：runimu A/C，IMU与四轮反馈推进 ===== */
static void run_status(void)
{
    static const char *names[] =
    {
        "IDLE", "PREFLIGHT", "MOVING", "DONE_BY_MOTOR", "STOPPED", "FAULT", "ALIGN",
        "OBSTACLE_STOP", "WAIT_OBSTACLE", "STATION"
    };
    if (run.translation_only) {
        text("[shift closed-loop] ");
        text(run.state == RUN_DONE ? "DONE_BY_IMU_ENCODER" : names[run.state]);
        text(run.travel_sign > 0 ? " travel=LEFT" : " travel=RIGHT");
        text(" distance_mm="); signed_number(run.progress_mm);
        text(" target_mm="); number(run.remaining_mm ? run.remaining_mm : run.action_mm);
        text(" error_cdeg="); signed_number(run.heading_control.error_cdeg);
        text(" rpm_limit="); number(run.lateral_rpm);
        text(" ack="); number(run.acks);
        text(" ready="); number(run.ready);
        text("; physical_path=unconfirmed\r\n");
        return;
    }
    if (run.turn_only) {
        text("[turn closed-loop] ");
        text(run.state == RUN_DONE ? "DONE_BY_IMU" : names[run.state]);
        text(" target_cdeg="); signed_number(run.heading_control.target_cdeg);
        text(" measured_cdeg="); signed_number(run.heading_control.progress_cdeg);
        text(" error_cdeg="); signed_number(run.heading_control.error_cdeg);
        text(" rpm="); number(run.turn_rpm);
        text(" ack="); number(run.acks);
        text(" ready="); number(run.ready);
        text(" pending="); number(run.pending);
        if (run.imu_mode) text(" control=continuous_velocity");
        text("\r\n");
        return;
    }
    text("[run] ");
    text(names[run.state]);
    text(" step=");
    number(run.steps);
    text(" job=");
    number(run.job);
    text("/8 node_est=");
    send_byte((uint8_t)('A' + full_route_nodes[run.plan][run.edge]));
    text(" ack=");
    number(run.acks);
    text(" ready=");
    number(run.ready);
    text(" pending=");
    number(run.pending);
    text(" heading_est="); number((uint32_t)run.heading);
    text(run.turning ? " action=turn" : run.travel_axis == BODY_LEFT ? " action=lateral" : " action=forward");
    text(" physical_pose=unconfirmed");
    if (run.imu_mode) {
        text(" heading_control=IMU control=continuous_velocity");
        text(" target_yaw_cdeg="); signed_number(heading_wrap_cdeg((int32_t)run.heading_control.last_yaw_cdeg + run.heading_control.error_cdeg));
        text(" yaw_cdeg="); signed_number(run.heading_control.last_yaw_cdeg);
        text(" error_cdeg="); signed_number(run.heading_control.error_cdeg);
        text(" distance_mm="); signed_number(run.progress_mm);
        text(" station_align_pending="); number(run.station_align_pending);
        text(" completed_roads_mask="); number(run.completed_roads);
        text(" returning="); number(run.obstacle_phase == 2);
        text(" blocked_roads_mask="); number(run.blocked_roads);
    }
    text("\r\n");
}

/* 仅记录邮箱背压；ESR/TSR记录遇到积压时的硬件状态。 */
static void can_tx_status(void)
{
    CanTxSnapshot s;can_tx_snapshot(&s);
    text("[can tx] queued=");number(s.queued);text(" high_water=");number(s.high_water);
    text(" mailbox_waits=");number(s.mailbox_waits);text(" queue_overflows=");number(s.queue_overflows);
    text(" cancelled=");number(s.cancelled);text(" blocked=");number(s.blocked);
    text(" addr=");number(s.head_id>>8);text(" packet=");number(s.head_id&255u);
    text(" command_dec=");number(s.head_command);text(" ESR_dec=");number(s.esr);text(" TSR_dec=");number(s.tsr);
    text(" rejected_addr=");number(s.rejected_id>>8);text(" rejected_command_dec=");number(s.rejected_command);
    text(" rejected_ESR_dec=");number(s.rejected_esr);text(" rejected_TSR_dec=");number(s.rejected_tsr);text("\r\n");
}
static void can_tx_update(void)
{
    static uint32_t warned_ms;
    static uint8_t warned,waiting_logged;
    CanTxSnapshot s;
    can_tx_service();can_tx_snapshot(&s);
    if(s.blocked) {
        if(!warned || (uint32_t)(clock_ms-warned_ms)>=1000u) {
            warned=waiting_logged=1;warned_ms=clock_ms;
            text("[can tx waiting] mailboxes busy; commands retained\r\n");can_tx_status();
        }
    } else if(waiting_logged) {
        waiting_logged=0;text("[can tx resumed] queued frames submitted; motor feedback still required\r\n");
    }
}

static void feedback_status(void)
{
    uint8_t i;
    text("[feedback] invalid="); number(invalid_can_replies);
    text(" rejected_shape="); number(rejected_can_frames);
    text(" can_rx_fault="); number(can_rx_fault); text("\r\n");
    can_tx_status();
    for (i = 0; i < MOTOR_COUNT; i++) {
        MotorFeedback *feedback = &motor_feedback[i];
        text("[motor] addr="); number(i + 1u);
        text(" replies="); number(feedback->replies);
        text(" status_replies="); number(feedback->status_replies);
        text(" action_status_replies="); number(feedback->action_status_replies);
        text(" command_dec="); number(feedback->last_command);
        text(" value_dec="); number(feedback->last_value);
        text(" flags_dec="); number(feedback->status_flags);
        text(" flags_valid="); number(feedback->status_replies != 0);
        text(" enabled="); number(!!(feedback->status_flags & MOTOR_FLAG_ENABLED));
        text(" arrived="); number(!!(feedback->status_flags & MOTOR_FLAG_ARRIVED));
        text(" stalled="); number(!!(feedback->status_flags & MOTOR_FLAG_STALLED));
        text(" protected="); number(!!(feedback->status_flags & MOTOR_FLAG_PROTECTED));
        text(" arrival_notices="); number(feedback->arrival_notices);
        text(" age_ms="); number(clock_ms - feedback->last_ms);
        text(" status_age_ms="); number(clock_ms - feedback->status_ms);
        text(" error_valid="); number(feedback->error_valid);
        text(" error_counts=");
        if (feedback->error_negative) send_byte('-');
        number(feedback->error_counts);
        text(" error_age_ms="); number(clock_ms - feedback->error_ms);
        text(" speed_valid="); number(feedback->speed_valid);
        text(" speed_rpm="); signed_number(feedback->speed_rpm);
        text(" speed_age_ms="); number(clock_ms - feedback->speed_ms);
        text(" position_valid="); number(feedback->position_valid);
        text(" position_counts=");
        if (feedback->position < 0) send_byte('-');
        number((uint32_t)(feedback->position < 0 ? -feedback->position : feedback->position));
        text(" position_age_ms="); number(clock_ms - feedback->position_ms);
        text("\r\n");
    }
}

static void imu_status(void)
{
    HeadingSample sample = heading_snapshot();
    text("[imu] uart_enabled="); number(CAR_IMU_UART_ENABLED);
    text(" fresh="); number(heading_sample_fresh(&sample, clock_ms, CAR_IMU_MAX_AGE_MS));
    text(" yaw_cdeg="); signed_number(sample.yaw_cdeg);
    text(" age_ms="); number(clock_ms - sample.received_ms);
    text(" samples="); number(sample.sequence);
    text(" checksum_errors="); number(heading_sensor.checksum_errors);
    text("\r\n");
}

static void run_wait_feedback(const char *reason);

static void run_warning(const char *reason)
{
    if (run.warning_seen && (uint32_t)(clock_ms-run.warning_ms)<1000u) return;
    run.warning_seen=1;run.warning_ms=clock_ms;
    text("[run warning] ");text(reason);
    text(" addr=");number(run.pending);text(" command_dec=");number(run.pending_command);
    text(" wait_ms=");number(clock_ms-run.query_ms);text("; task retained\r\n");
}

static void run_fault(const char *reason)
{
    can_tx_cancel(0);if(run.vision_debug)run.job=0;run.vision_debug=0;
    if (run_rx_overflow) reason = "CAN reply queue overflow";
    else if (can_rx_fault & 2u) reason = "CAN transmit queue full; command not queued";
    else if (can_rx_fault & 1u) reason = "CAN hardware FIFO overrun; feedback lost";
    run_active = 0;
    run.departure_overlap = 0;
    if (run.station_enabled) {station_abort();vision_cancel_pick();if (arm_enabled()) {arm_stop();arm_lock.state=3;}}
    else if (!run.turn_only && !run.translation_only) station_clear_task();
    run.state = RUN_FAULT;
    nav_stop(&nav);
    pose_set = 0;
    nav.position_confirmed = 0;
    text("[run fault] ");
    text(reason);
    text("; stopped; no automatic retry\r\n");
    run_status();
    feedback_status();
    if (run.imu_mode) imu_status();
}

/* 统一计算实际车身动作；预看下一段不能改动当前道路和完成记录。 */
static int run_resolve_leg(const RoadPlan *road, uint8_t segment, uint8_t edge,
                           RoadHeading heading, uint16_t completed, RunLeg *leg)
{
    MotionParameters p, body;
    uint8_t done;
    if (segment >= road->count || road->count > ROAD_MAX_SEGMENTS) return 0;
    p = road->segments[segment];
    if ((unsigned)p.axis > BODY_LEFT || !p.mm) return 0;
    leg->mm = (uint16_t)(p.mm < 0 ? -p.mm : p.mm);
    /* 第二次暂存作业结束后，H->E->B->A/C保持离站航向，只换算车身平移方向。 */
    if (CAR_RUN_RETURN_LATERAL_ENABLED && run.job == FULL_ROUTE_JOBS - 1u
        && edge >= FULL_ROUTE_NODES - 4u) {
        if (!road_motion_parameters(p, heading, &body)) return 0;
        leg->axis = body.axis;leg->sign = body.mm > 0 ? 1 : -1;leg->heading = heading;
        return 1;
    }
    leg->heading = p.axis == BODY_FORWARD ? (p.mm > 0 ? HEAD_Y_POS : HEAD_Y_NEG)
                                           : (p.mm > 0 ? HEAD_X_NEG : HEAD_X_POS);
    leg->axis = BODY_FORWARD;leg->sign = 1;
    if (CAR_RUN_START_LATERAL_ENABLED && !edge && !segment && road->to == NODE_B
        && (road->from == NODE_A || road->from == NODE_C)) {
        if (road_motion_parameters(p, heading, &body) && body.axis == BODY_LEFT) {
            leg->axis = BODY_LEFT;leg->sign = body.mm > 0 ? 1 : -1;leg->heading = heading;
            return 1;
        }
    }
    done = (completed & (uint16_t)(1u << road->road)) != 0;
    if (CAR_RUN_REVISIT_LATERAL_ENABLED && done
        && road_motion_parameters(p, heading, &body) && body.axis == BODY_LEFT) {
        leg->axis = body.axis;leg->sign = body.mm > 0 ? 1 : -1;leg->heading = heading;
        return 1;
    }
    if (CAR_RUN_REVERSE_ENABLED && done
        && ((unsigned)leg->heading + 4u - (unsigned)heading) % 4u == 2u) {
        leg->heading = heading;leg->sign = -1;
    }
    return 1;
}

static int run_next_straight(RoadPlan *road, uint8_t *edge, uint8_t *segment,
                              uint16_t *completed, RunLeg current, RunLeg *next)
{
    if (++*segment >= road->count) {
        uint8_t node = road->to;
        if (node == full_route_jobs[run.plan][run.job] || *edge + 2u >= FULL_ROUTE_NODES) return 0;
        *completed |= (uint16_t)(1u << road->road);
        (*edge)++;*segment = 0;
        if (!road_plan(full_route_nodes[run.plan][*edge],full_route_nodes[run.plan][*edge + 1u],road)) return 0;
    }
    if ((run.blocked_roads & (uint16_t)(1u << road->road))
        || !run_resolve_leg(road,*segment,*edge,current.heading,*completed,next)) return 0;
    return next->axis == current.axis && next->sign == current.sign && next->heading == current.heading;
}

static uint16_t run_straight_remaining(RunLeg current)
{
    RoadPlan road = run.road;
    uint8_t edge = run.edge, segment = run.segment, count;
    uint16_t completed = run.completed_roads, total = 0;
    RunLeg next;
    for (count = 0;count < FULL_ROUTE_NODES * ROAD_MAX_SEGMENTS;count++) {
        if (!run_next_straight(&road,&edge,&segment,&completed,current,&next)) break;
        total += next.mm;current = next;
    }
    return total;
}

static void run_segment_log(uint16_t mm)
{
    text("[route segment] from="); send_byte((uint8_t)('A' + run.road.from));
    text(" to="); send_byte((uint8_t)('A' + run.road.to));
    text(" part="); number(run.segment + 1u);
    if (run.travel_axis == BODY_LEFT) text(run.travel_sign < 0 ? " travel=RIGHT" : " travel=LEFT");
    else text(run.travel_sign < 0 ? " travel=BACKWARD" : " travel=FORWARD");
    text(" distance_mm="); number(mm);
    text(" target_heading_dec="); number(run.target_heading);
    text(" road_completed="); number((run.completed_roads & (uint16_t)(1u << run.road.road)) != 0);
    text(" continue_mm="); number(run.continuation_mm);text("\r\n");
}

/* 已完成道路可前后/左右平移，首次道路与工位朝向保持既有策略。 */
static int run_segment(void)
{
    RunLeg leg;
    if (!run_resolve_leg(&run.road,run.segment,run.edge,run.heading,run.completed_roads,&leg)) return 0;
    run.travel_axis = leg.axis;run.travel_sign = leg.sign;run.target_heading = leg.heading;
    run.remaining_mm = leg.mm;run.continuation_mm = run_straight_remaining(leg);
    run_segment_log(leg.mm);
    run_begin_alignment();
    return 1;
}

static void run_done(void)
{
    run_active = 0;
    if (run.station_shift) {
#if CAR_ARM_TRANSLATION_OVERLAP_ENABLED
        /* 子动作已真实完成停车，保留其最新航向/停车证明供后续同向衔接。 */
        station_parent.heading_control=run.heading_control;
        station_parent.zero_ms=run.zero_ms;station_parent.stopped_ms=run.stopped_ms;
#endif
        run = station_parent;
        run.state = RUN_STATION;
        run.station_relock = 1;
        arm_lock_begin();
        text("[station index done] encoder+IMU complete; zero commands confirmed; reacquiring fresh wheel status\r\n");
        return;
    }
    run.state = RUN_DONE;
    nav_stop(&nav);
    nav.position_confirmed = 0;
    pose_set = 0;
    if (run.translation_only) {
        text("[shift done by encoder + IMU heading + four-wheel stop; no automatic next action]\r\n");
        run_status();
        return;
    }
    if (run.turn_only) {
        turn_postcheck.done_ms = clock_ms;
        turn_postcheck.done_yaw_cdeg = heading_snapshot().yaw_cdeg;
        turn_postcheck.next = 0;
        turn_postcheck.active = 1;
        text("[turn done by motor feedback + IMU heading; no automatic next action]\r\n");
        run_status();
        return;
    }
    station_clear_task();
    text("[task expired] full run ended; scan or enter a new task for next run\r\n");
    text("[run done by motor feedback + IMU heading; inspect actual return position]\r\n");
    run_status();
}

/* 从 structure.h 取出当前有向道路，再启动道路的第一段。 */
static int run_road(void)
{
    const uint8_t *nodes = full_route_nodes[run.plan];
    if (!road_plan(nodes[run.edge], nodes[run.edge + 1u], &run.road))
        return 0;
    run.segment = 0;
    return run_segment();
}

/* runimu入口：清除人工位置确认，进入IMU和四轮反馈自检。 */
static int start_run(uint8_t initial_node)
{
    uint8_t i;
    HeadingSample sample = heading_snapshot();
    if (run_active || stop_pending || drive.state == MOTION_RUNNING || run.state == RUN_WAIT_OBSTACLE || run.state == RUN_STATION)
        return 0;
    if (!heading_sample_fresh(&sample, clock_ms, CAR_IMU_MAX_AGE_MS)) {
        text("[runimu rejected: no fresh heading sample]\r\n");
        return 0;
    }
    if (station_auto_enabled() && (!arm_ready() || !arm_parked() || arm_lock.state != 2 || arm_platform_slot() != 1)) {
        text("[runimu rejected: automatic work needs arm on, declared pose/platform slot 1 and parked arm]\r\n");return 0;
    }
    nav_stop(&nav);
    turn_postcheck.active = 0;
    pose_set = 0;
    nav.position_confirmed = 0;
    test_mode = MODE_NAVIGATION;
    memset(&run, 0, sizeof(run));
    run.plan = initial_node == NODE_C;
    run.imu_mode = 1;run.station_enabled=(uint8_t)station_auto_enabled();
    run.turn_rpm = CAR_TURN_CLOSED_LOOP_RPM;
    run.initial_heading = initial_node == NODE_C ? HEAD_Y_NEG : HEAD_Y_POS;
    run.yaw_zero_cdeg = heading_wrap_cdeg((int32_t)sample.yaw_cdeg
        - CAR_MAP_YAW_SIGN * (int32_t)run.initial_heading * 9000);
    imu_uart_fault = 0;
    memset(motor_feedback, 0, sizeof(motor_feedback));
    invalid_can_replies = 0;
    rejected_can_frames = 0;
    run.heading = run.target_heading = run.initial_heading;
    run.state = RUN_PREFLIGHT;
    run.segment_ms = clock_ms;
    run.last_query_ms = clock_ms - CAR_VELOCITY_QUERY_MS;
    run_clear_rx();
    run_active = 1;
    screen_clock.start_ms=clock_ms;screen_clock.elapsed_ms=0;
    screen_clock.started=screen_clock.running=1;
    text("[run route] ");
    for (i = 0; i < FULL_ROUTE_NODES; i++)
    {
        if (i)
            text("->");
        send_byte((uint8_t)('A' + full_route_nodes[run.plan][i]));
    }
    text("; forward rpm=" CAR_STRING_VALUE(CAR_RUN_RPM) "; turn rpm=");
    number(run.turn_rpm);
    text(initial_node == NODE_C ? "; zone=2 bottom_right; initial heading=-Y toward SCAN"
                               : "; zone=1 top_right; initial heading=+Y toward SCAN");
    text("; start lateral rpm<=" CAR_STRING_VALUE(CAR_LATERAL_MAX_RPM) "; continuous forward/backward; station_auto per configuration\r\n");
    run_status();
    return 1;
}

/* 单独定角入口：正值左转、负值右转，相对接到命令时的真实航向。
 * 只启动状态机，完成结果由run_update推进；不在这里阻塞等电机。
 */
static int start_heading_turn(int16_t relative_degrees, uint16_t rpm)
{
    HeadingSample sample = heading_snapshot();
    if (run_active || stop_pending || (drive.state != MOTION_IDLE && drive.state != MOTION_DONE)
        || !relative_degrees || relative_degrees < -180 || relative_degrees > 180
        || rpm < 20 || rpm > 300
        || !heading_sample_fresh(&sample, clock_ms, CAR_IMU_MAX_AGE_MS))
        return 0;
    nav_stop(&nav);
    turn_postcheck.active = 0;
    pose_set = nav.position_confirmed = 0;
    test_mode = MODE_ROTATION_TEST;
    memset(&run, 0, sizeof(run));
    run.imu_mode = run.turn_only = 1;
    run.turn_rpm = rpm;
    if (!heading_control_start(&run.heading_control, sample, (int32_t)relative_degrees * 100, clock_ms))
        return 0;
    run.state = RUN_PREFLIGHT;
    run.segment_ms = clock_ms;
    run.last_query_ms = clock_ms - CAR_VELOCITY_QUERY_MS;
    imu_uart_fault = 0;
    memset(motor_feedback, 0, sizeof(motor_feedback));
    invalid_can_replies = rejected_can_frames = 0;
    run_clear_rx();
    run_active = 1;
    run_status();
    return 1;
}

/* 单次横移与导航复用同一速度状态机；无新IMU时拒绝，不能静默退回脉冲模式。 */
static int start_heading_shift(int32_t mm, uint16_t rpm)
{
    HeadingSample sample = heading_snapshot();
    if (run_active || stop_pending || drive.state == MOTION_RUNNING || !mm
        || !heading_sample_fresh(&sample, clock_ms, CAR_IMU_MAX_AGE_MS)) return 0;
    nav_stop(&nav);
    turn_postcheck.active = 0;
    pose_set = nav.position_confirmed = 0;
    memset(&run, 0, sizeof(run));
    run.imu_mode = run.translation_only = 1;
    run.travel_axis = BODY_LEFT;
    run.travel_sign = mm > 0 ? 1 : -1;
    run.remaining_mm = (uint16_t)(mm > 0 ? mm : -mm);
    run.lateral_rpm = rpm > CAR_LATERAL_MAX_RPM ? CAR_LATERAL_MAX_RPM : rpm;
    run.turn_rpm = CAR_TURN_CLOSED_LOOP_RPM;
    if (!heading_control_start(&run.heading_control, sample, 0, clock_ms)) return 0;
    run.state = RUN_PREFLIGHT;
    run.segment_ms = clock_ms;
    run.last_query_ms = clock_ms - CAR_VELOCITY_QUERY_MS;
    imu_uart_fault = 0;
    memset(motor_feedback, 0, sizeof(motor_feedback));
    invalid_can_replies = rejected_can_frames = 0;
    run_clear_rx();
    run_active = 1;
    run_status();
    return 1;
}

/* 工位内仅允许高位收回；两区索引/视觉微调允许保持各自观察角。
 * 下一环位允许实际收拢后的末尾旋转并行；道路仅离站回正阶段可用此例外。 */
static int station_chassis_pose_safe(void)
{
    int slide,lift;float yaw;
    float work_yaw=(run.job==3u || run.job==6u) ? STN_STORAGE_ALIGN_YAW_DEG : STN_ROUGH_YAW_DEG;
    arm_pose_now(&slide,&lift,&yaw,NULL);
#if CAR_ARM_TRANSLATION_OVERLAP_ENABLED
    if(arm_translation_overlap_ready()) {
        arm_target_now(&slide,&lift,&yaw,NULL);
        return yaw==ARM_YAW_HOME_DEG || yaw==work_yaw;
    }
#endif
    return arm_ready() && !arm_busy() && slide==ARM_SLIDE_HOME_MM && lift==ARM_LIFT_HOME_MM
        && (yaw==ARM_YAW_HOME_DEG || yaw==work_yaw);
}
/* 仅增加工位调用接口；复用当前速度/编码器/IMU状态机，不引入远程底盘实现。 */
static int station_chassis_start_axis(int32_t mm,BodyAxis axis)
{
    uint8_t i;
#if CAR_ARM_TRANSLATION_OVERLAP_ENABLED
    static MotorFeedback parked_feedback[MOTOR_COUNT];
    int overlap=arm_translation_overlap_ready();
#endif
    int storage = run.job == 3u || run.job == 6u;
    int index_rpm = storage ? STN_STORAGE_INDEX_RPM : STN_ROUGH_INDEX_RPM;
    if (run_active || stop_pending || run.state != RUN_STATION || !run.station_phase
        || (run.job != 2u && run.job != 5u && !storage) || arm_lock.state != 2
        || !station_chassis_pose_safe() || !mm) return 0;
    for (i = 0; i < MOTOR_COUNT; i++) {
        MotorFeedback *f = &motor_feedback[i];
        station_log_wheel_speed(i,"launch");
        if (!f->speed_valid || !(f->status_flags & MOTOR_FLAG_ENABLED)
            || (f->status_flags & (MOTOR_FLAG_STALLED | MOTOR_FLAG_PROTECTED))
            || (uint32_t)(clock_ms - f->speed_ms) > ARM_FEEDBACK_TIMEOUT_MS
            || (uint32_t)(clock_ms - f->status_ms) > ARM_FEEDBACK_TIMEOUT_MS) return 0;
    }
    station_parent = run;
#if CAR_ARM_TRANSLATION_OVERLAP_ENABLED
    if(overlap)memcpy(parked_feedback,motor_feedback,sizeof(parked_feedback));
#endif
    if (!start_heading_shift(mm, (uint16_t)index_rpm)) {run = station_parent;return 0;}
    run.station_shift = run.station_enabled = 1;run.vision_debug=station_parent.vision_debug;
    run.travel_axis = axis;
    run.plan = station_parent.plan;run.edge = station_parent.edge;run.job = station_parent.job;
    run.heading = run.target_heading = station_parent.heading;
#if CAR_ARM_TRANSLATION_OVERLAP_ENABLED
    if(overlap) {
        /* 复用仍新鲜的停车状态/速度；四轮CPOS必须重新读取，不能继承旧里程基线。 */
        memcpy(motor_feedback,parked_feedback,sizeof(parked_feedback));
        for(i=0;i<MOTOR_COUNT;i++)motor_feedback[i].position_valid=0;
        run.heading_control=station_parent.heading_control;
        run.zero_ms=station_parent.zero_ms;run.stopped_ms=station_parent.stopped_ms;
        run.yaw_zero_cdeg=station_parent.yaw_zero_cdeg;run.transition_ready=1;
    }
#endif
    return 1;
}

int station_chassis_start(int32_t mm) {return station_chassis_start_axis(mm,BODY_FORWARD);}
int station_chassis_lateral_start(int32_t mm) {return station_chassis_start_axis(mm,BODY_LEFT);}
int station_chassis_xy_start(int32_t forward_mm,int32_t left_mm)
{
    int32_t magnitude;
    if(!left_mm)return station_chassis_start(forward_mm);
    if(!forward_mm)return station_chassis_lateral_start(left_mm);
    if(!station_chassis_start_axis(forward_mm,BODY_FORWARD))return 0;
    run.vector_move=1;
    run.vector_target_mm[0]=forward_mm;run.vector_target_mm[1]=left_mm;
    magnitude=forward_mm<0 ? -forward_mm : forward_mm;
    if((left_mm<0 ? -left_mm : left_mm)>magnitude)magnitude=left_mm<0 ? -left_mm : left_mm;
    run.remaining_mm=(uint16_t)magnitude;
    text("[station MOVEXY] forward_mm=");signed_number(forward_mm);
    text(" left_mm=");signed_number(left_mm);text("; concurrent encoder/IMU correction\r\n");
    return 1;
}


int station_chassis_busy(void) { return (run.station_shift && run_active) || run.station_relock; }
int station_chassis_failed(void)
{
    return run.station_shift && (run.state == RUN_FAULT || run.state == RUN_STOPPED);
}

static int vision_debug_command(const char *s)
{
    const char *argument;
    char operation='D';uint8_t color=0;
    int slide,lift;
    HeadingSample sample;
    if(strcmp(s,"vision debug") && strncmp(s,"vision debug ",13))return 0;
    argument=s+12;if(*argument==' ')argument++;
    if(!strcmp(argument,"status")) {
        text("[vision debug] active=");number(run.vision_debug);text("\r\n");
        (void)vision_command("vision status",clock_ms);return 1;
    }
    if(!strcmp(argument,"off")) {
        if(run.vision_debug) {stop_pending=1;text("[vision debug] stopping; pending moves cancelled by global stop\r\n");}
        else text("[vision debug] inactive\r\n");
        return 1;
    }
    if(*argument) {
        if((argument[0]=='D' || argument[0]=='E') && !argument[1])operation=argument[0];
        else if(argument[0]=='F' && argument[1]==' ' && argument[2]>='1' && argument[2]<='4' && !argument[3]) {
            operation='F';color=(uint8_t)(argument[2]-'0');
        } else {
            text("[vision debug usage] vision debug [D|E|F 1..4] | vision debug status | vision debug off\r\n");return 1;
        }
    }
    if(run_active || run.state==RUN_STATION || station_active() || stop_pending
        || drive.state==MOTION_RUNNING || drive.state==MOTION_WAIT_CONFIRM
        || nav.state==NAV_PLAN || nav.state==NAV_MOVING || nav.state==NAV_WAIT_NODE || nav.state==NAV_WAIT_RETURN) {
        text("[vision debug rejected: motion/station active; use ! or finish current work]\r\n");return 1;
    }
    arm_pose_now(&slide,&lift,NULL,NULL);sample=heading_snapshot();
    if(arm_lock.state!=2 || !arm_ready() || slide!=ARM_SLIDE_HOME_MM || lift!=ARM_LIFT_HOME_MM) {
        text("[vision debug rejected: arm on + declared pose/platform and high/retracted pose required]\r\n");return 1;
    }
    if(!heading_sample_fresh(&sample,clock_ms,CAR_IMU_MAX_AGE_MS)) {
        text("[vision debug rejected: fresh IMU required for correction motion]\r\n");return 1;
    }
    if(!station_start_vision_debug(operation,color,clock_ms)) {
        text("[vision debug rejected: cannot start alignment]\r\n");return 1;
    }
    memset(&run,0,sizeof(run));run.state=RUN_STATION;
    run.imu_mode=run.station_enabled=run.station_phase=run.vision_debug=1;
    run.job=operation=='D' ? 2u : operation=='E' ? 3u : 6u;
    run.turn_rpm=CAR_TURN_CLOSED_LOOP_RPM;imu_uart_fault=0;run_clear_rx();
    pose_set=nav.position_confirmed=0;turn_postcheck.active=0;
    text("[vision debug started] operation=");send_byte((uint8_t)operation);
    if(color){text(" color=");number(color);}
    text("; upper MOVE/MOVEY/MOVEXY FIFO; no automatic grip or route\r\n");
    return 1;
}

/* 四轮到位后推进当前动作；此函数可能提交下一动作或结束全程。 */
static void run_advance_segment(void);
static void run_finish_node(void);

static int run_station_heading(uint8_t node, RoadHeading *heading)
{
    switch (node) {
    case NODE_F: *heading = (RoadHeading)CAR_STATION_RAW_HEADING; return 1;
    case NODE_D: *heading = (RoadHeading)CAR_STATION_ROUGH_HEADING; return 1;
    case NODE_H: *heading = (RoadHeading)CAR_STATION_STORAGE_HEADING; return 1;
    default: return 0;
    }
}

static void run_advance_segment(void)
{
    RoadHeading station_heading;
    uint8_t node;
    run.segment++;
    if (run.segment < run.road.count) {
        if (!run_segment()) run_fault("next segment failed");
        return;
    }
    run.completed_roads |= (uint16_t)(1u << run.road.road);
    run.road_path_count = 0;
    run.edge++;
    node = full_route_nodes[run.plan][run.edge];
    if (run.station_enabled && node == NODE_B && run.job == 0u) {
        text("[scan station] first B arrival; automatic qr init\r\n");
        scanner_init(scanner.baud ? scanner.baud : CAR_SCANNER_UART_BAUD);
        run.scan_turn_stage=1;
        run.target_heading=(RoadHeading)STN_SCAN_HEADING;run.remaining_mm=0;
        text("[scan station] B reached; configuring QR while aligning rear scanner; accept when READY\r\n");
        run_begin_alignment();return;
    }
    if (node == full_route_jobs[run.plan][run.job]
        && run_station_heading(node, &station_heading)) {
        run.station_align_pending = 1;
        run.target_heading = station_heading;
        run.remaining_mm = 0;
        text("[station alignment] node="); send_byte((uint8_t)('A' + node));
        text(" right_side=WORK_AREA target_heading_dec="); number(station_heading);
        text("; checkpoint pending\r\n");
        run_begin_alignment();
        return;
    }
    run_finish_node();
}

/* 节点只推进一次；工位必须先完成右侧朝向，才能计数并选择下一道路。 */
static void run_finish_node(void)
{
    static const char *station_names[FULL_ROUTE_JOBS] = {
        "SCAN", "RAW_MATERIAL", "ROUGH_PROCESSING", "TEMPORARY_STORAGE",
        "RAW_MATERIAL", "ROUGH_PROCESSING", "TEMPORARY_STORAGE", "START_ZONE"
    };
    if (full_route_nodes[run.plan][run.edge] == full_route_jobs[run.plan][run.job])
    {
        if (run.station_enabled && run.job < 7u) {
            run_active=0;run.state=RUN_STATION;run.turning=0;run.transition_ready=0;run.station_phase=0;
            text("[run station] waiting for fresh four-wheel status; job=");number(run.job);text("\r\n");
            arm_lock_begin();return;
        }
        run.job++;
        text("[run checkpoint] job=");
        number(run.job);
        text("/8; station action skipped\r\n");
        text("[station estimate] node="); send_byte((uint8_t)('A' + full_route_nodes[run.plan][run.edge]));
        text(" station="); text(station_names[run.job - 1u]);
        text("; physical_pose=unconfirmed\r\n");
    }
    if (run.edge == FULL_ROUTE_NODES - 1u)
    {
        /* 最后平移已通过编码器、IMU和停车确认，不再恢复发车朝向。 */
        run_done();
        return;
    }
    if (!run_road())
        run_fault("next road failed");
    return;
}


/* 工位与行驶交接时重新采四轮反馈，不能复用作业前的CPOS/速度缓存。 */
static void run_station_handoff(void)
{
    run_active=1;run.state=RUN_PREFLIGHT;run.segment_ms=clock_ms;
    run.station_departing=1;
    run.station_phase=0;run.pending=run.velocity_waiting=run.velocity_sent=0;
    run.velocity_query_index=0;run.last_query_ms=clock_ms-CAR_VELOCITY_QUERY_MS;
    run.transition_ready=CAR_ARM_TRANSLATION_OVERLAP_ENABLED ? 1u : 0u;
    if(!CAR_ARM_TRANSLATION_OVERLAP_ENABLED)run.zero_ms=run.stopped_ms=0;
    run_clear_rx();memset(motor_feedback,0,sizeof(motor_feedback));
}
static void run_resume_after_station(void)
{
    int overlap=station_departure_ready();
    if (full_route_nodes[run.plan][run.edge] != full_route_jobs[run.plan][run.job]
        || (!(arm_ready() && arm_parked()) && !overlap)) {
        run_fault("station completion/parked pose mismatch");return;
    }
    run.departure_overlap=(uint8_t)overlap;
    if(overlap)text(CAR_ARM_TRANSLATION_OVERLAP_ENABLED ? "[station departure] final rotation overlaps turn/translation\r\n" : "[station departure] final rotation overlaps turn; translation waits\r\n");
    run.job++;text("[run station] complete; job=");number(run.job);text("/8; resume after fresh feedback\r\n");
    run_station_handoff();
}
static void run_station_update(void)
{
    int started;
    HeadingSample sample=heading_snapshot();
    if (arm_lock.state==3 || arm_failed()) {
        run_fault("station interlock/arm fault");return;
    }
    if (imu_uart_fault) {run_warning("station IMU UART error");imu_uart_fault=0;}
    if (!heading_sample_fresh(&sample,clock_ms,CAR_IMU_MAX_AGE_MS)) {
        run_warning("station IMU sample stale; task retained");return;
    }
#if CAR_ARM_TRANSLATION_OVERLAP_ENABLED
    /* 作业时不输出底盘指令，但持续确认原航向，避免离站重复等待已满足的稳定时间。 */
    (void)heading_control_update(&run.heading_control,sample,clock_ms);
#endif
    if (run.station_relock) {
        if (arm_lock.state != 2) return;
        run.station_relock = 0;
    }
    if (!run.station_phase) {
        if (arm_lock.state!=2) return;
        started=station_start(full_route_nodes[run.plan][run.edge],run.job,clock_ms);
        if (started<0) {run_fault("station action refused");return;}
        run.station_phase=1;
    }
    station_update(clock_ms);
    if (station_failed()) {run_fault("station work failed");return;}
    if(run.vision_debug && !station_active()) {
        run.vision_debug=run.station_enabled=run.station_phase=0;run.job=0;run.state=RUN_IDLE;
        text("[vision debug done] commanded moves complete; hold position/yaw; no grip, return or route\r\n");
        return;
    }
    if (!station_active() || station_departure_ready()) {
        if (!station_departure_ready() && (!arm_ready() || !arm_parked())) {run_fault("station ended without parked arm");return;}
        /* 扫码完成也直接选择下一道路，不再恢复进站前航向后又转回B->E。 */
        run_resume_after_station();
    }
}

static int16_t run_heading_error(HeadingSample sample)
{
    return heading_wrap_cdeg(CAR_MAP_YAW_SIGN * (int32_t)run.target_heading * 9000
                             - ((int32_t)sample.yaw_cdeg - run.yaw_zero_cdeg));
}

/* 四轮已持续停稳；复用此区间的位置采样，不再要求对齐结束后重采一轮。 */
static void run_begin_baseline(void)
{
    run.turning = 0;
    run.velocity_phase = VELOCITY_BASELINE;
    if(run.state==RUN_PREFLIGHT){run.state=RUN_ALIGN;drive.state=MOTION_RUNNING;}
    run.phase_ms = clock_ms;
    run.baseline_after_ms = run.stopped_ms;
    run.station_departing = 0;
    run.stopped_ms = 0;
}

/* 每个道路几何段仅开始一次航向目标；同向衔接可复用刚通过的完成判据。 */
static void run_begin_alignment(void)
{
    HeadingSample sample = heading_snapshot();
    int16_t error = run_heading_error(sample);
    uint8_t reuse = run.transition_ready;
    run.transition_ready = 0;
    if (reuse && run.heading == run.target_heading && !run.station_align_pending
        && !run.turn_only && (!run.translation_only || run.station_shift)) {
        HeadingDecision decision = heading_control_update(&run.heading_control, sample, clock_ms);
        if (decision == HEADING_FAULT) {run.alignment_waiting=1;run_wait_feedback(run.heading_control.fault);return;}
        /* 串口输出期间可能收到新角度：保留原目标更新它，不能把新偏航重设为零。 */
        if (error <= CAR_HEADING_TOLERANCE_CDEG && error >= -CAR_HEADING_TOLERANCE_CDEG
            && run.heading_control.last_yaw_cdeg == sample.yaw_cdeg
            && run.heading_control.stable_samples >= CAR_HEADING_STABLE_SAMPLES
            && (uint32_t)(run.heading_control.sample_ms - run.heading_control.stable_ms) >= CAR_HEADING_STABLE_MS
            && run_velocity_stopped()) {
            text((run.station_departing || run.station_shift) ? "[station heading retained; fresh wheels confirmed]\r\n" : "[heading reused; four wheels stopped]\r\n");
            run_begin_baseline();
            return;
        }
    }
    if (!heading_control_start(&run.heading_control, sample, error, clock_ms)) {
        run.alignment_waiting=1;run_wait_feedback("IMU sample stale/invalid before alignment");
        return;
    }
    run.velocity_phase = VELOCITY_TURN;
    run.phase_ms = clock_ms;
    run.turning = 1;
    run.state = RUN_ALIGN;
    drive.state = MOTION_RUNNING;
    run.steps++;
    run.stopped_ms = 0;
}

/* 所有CAN模式共享格式解析；诊断值不会代替ACK或查询事务。 */
static int run_record_frame(const RunCanFrame *frame)
{
    MotorFeedback *feedback = &motor_feedback[(frame->id >> 8) - 1u];
    if (frame->data[frame->length - 1u] != MOTOR_REPLY_CHECK_BYTE
        || (frame->length != CAN_REPLY_BYTES && frame->data[1] > 1u)
        || (frame->length == CAN_REPLY_BYTES && (frame->data[0] == MOTOR_SPEED_COMMAND
            || frame->data[0] == MOTOR_ERROR_COMMAND || frame->data[0] == MOTOR_CURRENT_POSITION_COMMAND))) {
        invalid_can_replies++;
        return 0;
    }
    feedback->replies++;
    feedback->last_ms = frame->received_ms;
    feedback->last_command = frame->data[0];
    feedback->last_value = frame->data[1];
    if (frame->length == 7u) {
        uint32_t magnitude = ((uint32_t)frame->data[2] << 24) | ((uint32_t)frame->data[3] << 16)
                            | ((uint32_t)frame->data[4] << 8) | frame->data[5];
        if (frame->data[0] == MOTOR_ERROR_COMMAND) {
            feedback->error_counts = magnitude;
            feedback->error_negative = frame->data[1];
            feedback->error_ms = frame->received_ms;
            feedback->error_valid = 1;
        } else {
            int64_t position = frame->data[1] ? -(int64_t)magnitude : (int64_t)magnitude;
            int64_t change = position - feedback->position;
            if (!feedback->position_valid || change > 4 || change < -4)
                feedback->position_changed_ms = frame->received_ms;
            feedback->position = position;
            feedback->position_ms = frame->received_ms;
            feedback->position_valid = 1;
        }
    } else if (frame->length == 5u) {
        feedback->speed_rpm = (int32_t)(((uint16_t)frame->data[2] << 8) | frame->data[3]);
        if (frame->data[1]) feedback->speed_rpm = -feedback->speed_rpm;
        feedback->speed_ms = frame->received_ms;
        feedback->speed_valid = 1;
    } else if (frame->data[0] == MOTOR_STATUS_COMMAND) {
        feedback->status_flags = frame->data[1];
        feedback->status_replies++;
        feedback->status_ms = frame->received_ms;
    } else if (frame->data[0] == MOTOR_POSITION_COMMAND && frame->data[1] == MOTOR_ARRIVAL_NOTICE)
        feedback->arrival_notices++;
    return 1;
}

static void arm_lock_fault(const char *reason)
{
    arm_lock.state = 3;
    arm_stop();
    nav_stop(&nav);
    pose_set = nav.position_confirmed = 0;
    text("[arm interlock fault] "); text(reason);
    text("; chassis stopped; inspect then arm on to recheck\r\n");
    if (run.station_enabled && run.state==RUN_STATION) run_fault("station chassis interlock failed");
}

static void arm_lock_begin(void)
{
    memset(&arm_lock, 0, sizeof(arm_lock));
    run_clear_rx();
    memset(motor_feedback, 0, sizeof(motor_feedback));
    arm_lock.state = 1;
    nav_stop(&nav);
    pose_set = nav.position_confirmed = 0;
    memset(&turn_postcheck, 0, sizeof(turn_postcheck));
    arm_lock.started_ms = clock_ms;
    arm_lock.poll_ms = clock_ms - ARM_POLL_MS;
    text(station_speed_log_only() ? "[arm interlock: waiting for fresh four-wheel status; speed logged only]\r\n"
                                : "[arm interlock: waiting for fresh four-wheel zero-speed/status feedback]\r\n");
}

/* 独占会话期间持续查询四轮；估时、旧反馈或单轮回复均不能放行。 */
static void arm_lock_update(void)
{
    RunCanFrame frame;
    uint8_t i, ready = 1;
    if (!arm_lock.state || arm_lock.state == 3 || stop_pending) return;
    /* 行驶闭环独占四轮接收队列；机构必须保持已声明的收拢姿态。 */
    if (run_active) return;
#ifdef NAV_ENTRY_HOST_TEST
    if (entry_host_can_bus_off()) can_rx_fault |= CAN_BUS_OFF_FLAG;
#else
    if (CAN1->ESR & CAN_BUS_OFF_FLAG) can_rx_fault |= CAN_BUS_OFF_FLAG;
#endif
    if (run_rx_overflow || (can_rx_fault & 1u)) {
        run_warning("interlock CAN receive overflow; continue polling");run_recover_receive();
    }
    if (can_rx_fault) {arm_lock_fault("CAN transmit/bus fault");return;}
    while (run_read_frame(&frame)) {
        uint8_t kind;
        i = (uint8_t)((frame.id >> CAN_MOTOR_ADDRESS_SHIFT) - 1u);
        if (frame.data[0] == MOTOR_STATUS_COMMAND && frame.length == 3u) kind = 0;
        else if (frame.data[0] == MOTOR_SPEED_COMMAND && frame.length == 5u) kind = 1;
        else continue;
        if (!(arm_lock.requested[i] & (1u << kind))
            || (int32_t)(frame.received_ms - arm_lock.query_ms[i][kind]) < 0
            || !run_record_frame(&frame)) continue;
        arm_lock.seen[i] |= (uint8_t)(1u << kind);
        if (kind) station_log_wheel_speed(i,"interlock");
        else if (!(motor_feedback[i].status_flags & MOTOR_FLAG_ENABLED)
                 || (motor_feedback[i].status_flags & (MOTOR_FLAG_STALLED | MOTOR_FLAG_PROTECTED))) {
            arm_lock_fault("wheel disabled/stalled/protected");return;
        }
    }
    for (i = 0; i < MOTOR_COUNT; i++) {
        MotorFeedback *f = &motor_feedback[i];
        if (arm_lock.seen[i] != 3u
            || (uint32_t)(clock_ms - f->speed_ms) > ARM_FEEDBACK_TIMEOUT_MS
            || (uint32_t)(clock_ms - f->status_ms) > ARM_FEEDBACK_TIMEOUT_MS) ready = 0;
    }
    if (arm_lock.state == 1) {
        if (!ready) arm_lock.settling = 0;
        else if (!arm_lock.settling) {arm_lock.settling = 1;arm_lock.stable_ms = clock_ms;}
        else if ((uint32_t)(clock_ms - arm_lock.stable_ms) >= ARM_CHASSIS_STOP_HOLD_MS) {
            if (stop_pending) return;
            if (!arm_enabled()) arm_command("arm on", clock_ms);
            arm_lock.state = 2;
            text("[arm interlock acquired: chassis commands locked until parked arm off]\r\n");
        }
        if (arm_lock.state == 1 && (uint32_t)(clock_ms - arm_lock.started_ms) > ARM_CHASSIS_LOCK_TIMEOUT_MS) {
            run_warning("four-wheel confirmation delayed; continue polling");
        }
    } else if (!ready) {run_warning("arm interlock feedback stale; continue polling");}
    if (!stop_pending && (uint32_t)(clock_ms - arm_lock.poll_ms) >= ARM_POLL_MS) {
        uint8_t wheel = (uint8_t)(arm_lock.query / 2u), kind = arm_lock.query % 2u;
        arm_lock.query = (uint8_t)((arm_lock.query + 1u) % 8u);
        arm_lock.poll_ms = arm_lock.query_ms[wheel][kind] = clock_ms;
        arm_lock.requested[wheel] |= (uint8_t)(1u << kind);
        Emm_V5_Read_Sys_Params((uint8_t)(wheel + 1u), kind ? S_VEL : S_FLAG);
    }
}

static int arm_serial_command(const char *s)
{
    if (strcmp(s,"@") && strcmp(s, "arm") && strncmp(s, "arm ", 4) && strncmp(s, "claw ", 5)) return 0;
    if (!strcmp(s, "arm") || !strcmp(s, "arm status") || !strcmp(s, "claw status")) {
        arm_command(s, clock_ms);
        text("[arm interlock] state="); number(arm_lock.state); text("\r\n");return 1;
    }
    if (!strcmp(s, "arm stop")) {
        arm_command(s,clock_ms);
        if (run.station_enabled && (run_active || run.state==RUN_STATION)) run_fault("operator stopped station arm");
        return 1;
    }
    if (run_active || run.state==RUN_STATION) {text("[arm rejected: route/station owns motion; use ! to stop]\r\n");return 1;}
    if (!strcmp(s, "arm off")) {
        if (arm_enabled() && !arm_parked()) {
            text("[arm rejected: park with arm home before arm off; ! remains emergency stop]\r\n");return 1;
        }
        arm_command(s, clock_ms);
        if (arm_lock.state) {
            memset(&arm_lock, 0, sizeof(arm_lock));run_clear_rx();
        }
        return 1;
    }
    if (!strcmp(s, "arm on")) {
        if (run_active || stop_pending || drive.state == MOTION_RUNNING || drive.state == MOTION_WAIT_CONFIRM
            || nav.state == NAV_PLAN || nav.state == NAV_MOVING || nav.state == NAV_WAIT_NODE || nav.state == NAV_WAIT_RETURN) {
            text("[arm rejected: chassis active; stop and inspect first]\r\n");return 1;
        }
        if (arm_lock.state == 1 || arm_lock.state == 2) {
            text("[arm rejected: session pending/already on]\r\n");return 1;
        }
        arm_lock_begin();return 1;
    }
    if (arm_lock.state != 2 || stop_pending) {
        text("[arm rejected: arm on and four-wheel stop confirmation required]\r\n");return 1;
    }
    arm_command(s, clock_ms);return 1;
}

static int run_velocity_feedback_fresh(void)
{
    uint8_t i;
    run.ready = 0;
    for (i = 0; i < MOTOR_COUNT; i++) {
        MotorFeedback *f = &motor_feedback[i];
        if (f->position_valid && f->speed_valid && f->status_replies
            && (uint32_t)(clock_ms - f->position_ms) <= CAR_VELOCITY_FEEDBACK_MS
            && (uint32_t)(clock_ms - f->speed_ms) <= CAR_VELOCITY_FEEDBACK_MS
            && (uint32_t)(clock_ms - f->status_ms) <= CAR_VELOCITY_FEEDBACK_MS)
            run.ready |= (uint8_t)(1u << i);
    }
    return run.ready == ALL_MOTORS_MASK;
}

/* 每批四轮速度先收到ACK，才提交下一批；同步启动前检查急停和发送错误。 */
static int run_velocity_send(int16_t forward, int16_t left, int16_t turn)
{
    int16_t wheels[4];
    uint8_t i, changed = !run.velocity_sent, all_zero = 1;
    heading_control_translate_wheels(forward, left, turn, wheels);
    for (i = 0; i < MOTOR_COUNT; i++) {
        if (wheels[i] != run.wheel_rpm[i]) changed = 1;
        if (wheels[i]) all_zero = 0;
    }
    if (!changed || run.velocity_waiting) return 1;
    run.acks = 0;
    run.velocity_waiting = 1;
    run.velocity_ms = clock_ms;
    for (i = 0; i < MOTOR_COUNT; i++) {
        int16_t value = wheels[i];
        if (stop_pending || can_rx_fault || run_rx_overflow) return 0;
        if (!run.wheel_rpm[i] && value) run.wheel_start_ms[i] = clock_ms;
        run.wheel_rpm[i] = value;
        Emm_V5_Vel_Control(i + 1u, value > 0 ? 1u : 0u,
            (uint16_t)(value < 0 ? -value : value), CAR_MOTION_ACCELERATION, true);
        delay_ms(1);
    }
    if (!motor_sync(0)) return 0;
    run.velocity_sent = 1;
    if (!all_zero) {run.zero_ms = 0;run.stopped_ms = 0;}
    else if (!run.zero_ms) {run.zero_ms = clock_ms;run.stopped_ms = 0;}
    return 1;
}

static int run_velocity_stopped(void)
{
    uint8_t i;
    if (!run.zero_ms || run.velocity_waiting || !run_velocity_feedback_fresh()) return 0;
    for (i = 0; i < MOTOR_COUNT; i++) {
        MotorFeedback *f = &motor_feedback[i];
        uint32_t after_zero = f->speed_ms - run.zero_ms;
        station_log_wheel_speed(i,"stop");
        if (run.obstacle_phase && (f->speed_rpm>CAR_VELOCITY_STOP_RPM || f->speed_rpm<-CAR_VELOCITY_STOP_RPM)) {
            run.stopped_ms=0;return 0; /* 主动obstacle要求真正停稳后才允许反向。 */
        }
        if (after_zero < CAR_HEADING_SETTLE_MS || after_zero > 0x7fffffffu) {
            run.stopped_ms = 0;
            return 0;
        }
    }
    if (!run.stopped_ms) run.stopped_ms = clock_ms;
    return (uint32_t)(clock_ms - run.stopped_ms) >= CAR_HEADING_STABLE_MS;
}

static void run_velocity_query(void)
{
    static const SysParams_t parameters[3] = {S_CPOS, S_VEL, S_FLAG};
    static const uint8_t commands[3] = {MOTOR_CURRENT_POSITION_COMMAND, MOTOR_SPEED_COMMAND, MOTOR_STATUS_COMMAND};
    uint8_t kind = run.velocity_query_index % 3u;
    if (run.pending || (uint32_t)(clock_ms - run.last_query_ms) < CAR_VELOCITY_QUERY_MS) return;
#if CAR_ARM_TRANSLATION_OVERLAP_ENABLED
    if(run.state==RUN_PREFLIGHT && run.station_shift && run.transition_ready) {
        unsigned count;
        for(count=0;count<12u;count++) {
            MotorFeedback *f=&motor_feedback[run.velocity_query_index/3u];
            kind=run.velocity_query_index%3u;
            if(!((kind==1u && f->speed_valid && (uint32_t)(clock_ms-f->speed_ms)<=CAR_VELOCITY_FEEDBACK_MS)
                 || (kind==2u && f->status_replies && (uint32_t)(clock_ms-f->status_ms)<=CAR_VELOCITY_FEEDBACK_MS)))break;
            run.velocity_query_index=(uint8_t)((run.velocity_query_index+1u)%12u);
        }
    }
#endif
    run.pending = (uint8_t)(run.velocity_query_index / 3u + 1u);
    run.pending_command = commands[kind];
    run.query_ms = run.last_query_ms = clock_ms;
    Emm_V5_Read_Sys_Params(run.pending, parameters[kind]);
}

static void run_velocity_log(void);

static int run_save_leg(uint16_t mm)
{
    RunLeg *leg;
    if (!mm) return 1;
    if (run.road_path_count >= ROAD_MAX_SEGMENTS) return 0;
    leg = &run.road_path[run.road_path_count++];
    leg->axis = run.travel_axis;leg->sign = run.travel_sign;
    leg->heading = run.target_heading;leg->mm = mm;
    return 1;
}

static int run_pass_straight(void)
{
    RoadPlan next_road = run.road;
    uint8_t edge = run.edge, segment = run.segment;
    uint16_t completed = run.completed_roads;
    RunLeg current, next;
    current.axis = run.travel_axis;current.sign = run.travel_sign;
    current.heading = run.target_heading;current.mm = run.action_mm;
    if (!run_next_straight(&next_road,&edge,&segment,&completed,current,&next)) return 0;
    /* 先输出旧段的统计，避免同一窗口混入节点两侧的相对里程。 */
    if (run.heading_window.samples) {run.log_ms = clock_ms - CAR_CONTROL_LOG_MS;run_velocity_log();}
    if (edge == run.edge) {
        if (!run_save_leg(run.action_mm)) return 0;
    } else run.road_path_count = 0;
    run.encoder_offset_mm += run.action_mm;
    run.progress_mm -= run.action_mm;
    run.road = next_road;run.edge = edge;run.segment = segment;run.completed_roads = completed;
    run.remaining_mm = run.action_mm = next.mm;
    run.continuation_mm = run_straight_remaining(next);
    run.steps++;run.distance_mm += next.mm;
    text("[route pass] node_est=");send_byte((uint8_t)('A' + run.road.from));
    text("; no stop; physical_pose=unconfirmed\r\n");
    run_segment_log(next.mm);
    return 1;
}

/* 车轮混控的逆变换同时观测两个车身轴，不能把斜向行程当成单轴平均绝对距离。
 * 编码器极性由两份实际净位移和同向轮速指令确认；对角线上暂不动的轮子不阻塞另两轮。
 * 一旦本轮开始运动再确认其极性，并从共同起点计入全部里程。 */
static int run_vector_distance(void)
{
    static const int8_t forward_sign[4]={-1,1,1,-1};
    static const int8_t left_sign[4]={-1,-1,1,1};
    int64_t sum_forward=0,sum_left=0;
    float ppm=CALIBRATED_PPM_X<CALIBRATED_PPM_Y ? CALIBRATED_PPM_X : CALIBRATED_PPM_Y;
    int64_t learn_counts=(int64_t)(CAR_ENCODER_LEARN_MM*ppm*CAR_ENCODER_COUNTS_PER_REV/CAR_POSITION_PULSES_PER_REV)+1;
    uint8_t i;
    for(i=0;i<MOTOR_COUNT;i++) {
        int64_t delta=motor_feedback[i].position-run.encoder_start[i];
        if(!run.encoder_sign[i] && motor_feedback[i].position_ms!=run.encoder_sample_ms[i]) {
            int sign=run.wheel_rpm[i]>0 ? 1 : run.wheel_rpm[i]<0 ? -1 : 0;
            int64_t previous=run.encoder_previous_delta[i];
            if(sign && sign==run.vector_wheel_sign[i]
                && ((delta>=learn_counts && previous>=learn_counts && delta>previous)
                    || (delta<=-learn_counts && previous<=-learn_counts && delta<previous))) {
                run.encoder_sign[i]=(int8_t)((delta>0 ? 1 : -1)*sign);
                run.encoder_ready|=(uint8_t)(1u<<i);
            }
            run.vector_wheel_sign[i]=(int8_t)sign;
            run.encoder_previous_delta[i]=delta;run.encoder_sample_ms[i]=motor_feedback[i].position_ms;
        }
        sum_forward+=delta*run.encoder_sign[i]*forward_sign[i];
        sum_left+=delta*run.encoder_sign[i]*left_sign[i];
    }
    run.vector_progress_mm[0]=(int32_t)((float)sum_forward*CAR_POSITION_PULSES_PER_REV
        /(4.0f*CAR_ENCODER_COUNTS_PER_REV*CALIBRATED_PPM_Y));
    run.vector_progress_mm[1]=(int32_t)((float)sum_left*CAR_POSITION_PULSES_PER_REV
        /(4.0f*CAR_ENCODER_COUNTS_PER_REV*CALIBRATED_PPM_X));
    for(i=0;i<2;i++) {
        int32_t target=run.vector_target_mm[i],progress=run.vector_progress_mm[i];
        if(target<0){target=-target;progress=-progress;}
        if(progress < -CAR_DISTANCE_OVERRUN_MM || progress>target+CAR_DISTANCE_OVERRUN_MM)return 0;
    }
    i=(uint8_t)(abs(run.vector_target_mm[1])>abs(run.vector_target_mm[0]));
    run.progress_mm=run.vector_progress_mm[i]*(run.vector_target_mm[i]>0 ? 1 : -1);
    return 1;
}

/* 双轴分别根据剩余毫米制动；沿用原有工位速度、加减速、距离窗口，无新增视觉门槛。 */
static int16_t run_vector_target(uint8_t axis)
{
    int32_t remaining=run.vector_target_mm[axis]-run.vector_progress_mm[axis];
    int sign=remaining<0 ? -1 : 1;
    int32_t speed,limit;
    float ppm=axis ? CALIBRATED_PPM_X : CALIBRATED_PPM_Y;
    float delay=CAR_DRIVE_DECEL_RPM_PER_S*(CAR_DRIVE_BRAKE_LOOKAHEAD_MS/1000.0f);
    float braking;
    if(remaining<0)remaining=-remaining;
    if(remaining<=CAR_DISTANCE_TOLERANCE_MM)return 0;
    speed=remaining*CAR_DISTANCE_KP_X100/100;
    braking=sqrtf(delay*delay+2.0f*CAR_DRIVE_DECEL_RPM_PER_S
        *(remaining-CAR_DISTANCE_TOLERANCE_MM)*60.0f*ppm/CAR_POSITION_PULSES_PER_REV)-delay;
    if(speed>(int32_t)braking)speed=(int32_t)braking;
    if(speed<CAR_DISTANCE_MIN_RPM)speed=CAR_DISTANCE_MIN_RPM;
    limit=(run.job==3u || run.job==6u) ? STN_STORAGE_INDEX_RPM : STN_ROUGH_INDEX_RPM;
    if(!run.encoder_ready && limit>CAR_ENCODER_LEARN_RPM)limit=CAR_ENCODER_LEARN_RPM;
    if(speed>limit)speed=limit;
    return (int16_t)(speed*sign);
}

static int16_t run_vector_slew(uint8_t axis,int16_t desired,uint8_t due)
{
    if(due) {
        uint32_t elapsed=clock_ms-run.vector_profile_ms[axis];
        int32_t target=abs(desired)*1000,change;
        int sign=desired<0 ? -1 : desired>0 ? 1 : run.vector_profile_sign[axis];
        uint32_t rate;
        if(!run.vector_mrpm[axis])run.vector_profile_sign[axis]=(int8_t)sign;
        if(run.vector_profile_sign[axis]!=sign)target=0;
        rate=target>run.vector_mrpm[axis] ? CAR_DRIVE_ACCEL_RPM_PER_S : CAR_DRIVE_DECEL_RPM_PER_S;
        if(elapsed>CAR_DRIVE_SLEW_MAX_DT_MS)elapsed=CAR_DRIVE_SLEW_MAX_DT_MS;
        change=(int32_t)(rate*elapsed);
        if(target>run.vector_mrpm[axis]) {
            run.vector_mrpm[axis]+=change;
            if(run.vector_mrpm[axis]>target)run.vector_mrpm[axis]=target;
        } else {
            run.vector_mrpm[axis]-=change;
            if(run.vector_mrpm[axis]<target)run.vector_mrpm[axis]=target;
        }
        run.vector_profile_ms[axis]=clock_ms;
    }
    return (int16_t)(run.vector_mrpm[axis]/1000*run.vector_profile_sign[axis]);
}

static int run_velocity_distance(void)
{
    int64_t sum = 0;
    float ppm = run.travel_axis == BODY_LEFT ? CALIBRATED_PPM_X : CALIBRATED_PPM_Y;
    int64_t learn_counts = (int64_t)(CAR_ENCODER_LEARN_MM * ppm * CAR_ENCODER_COUNTS_PER_REV
                                    / CAR_POSITION_PULSES_PER_REV) + 1;
    uint8_t i;
    if(run.vector_move)return run_vector_distance();
    for (i = 0; i < MOTOR_COUNT; i++) {
        int64_t delta = motor_feedback[i].position - run.encoder_start[i];
        /* 不用第一次微小位移锁定极性。两份不同采样时刻的CPOS均超过净位移门槛，
         * 且继续同向增长才确认；重复处理同一缓存不算第二份样本。 */
        if (!run.encoder_sign[i] && motor_feedback[i].position_ms != run.encoder_sample_ms[i]) {
            int64_t previous = run.encoder_previous_delta[i];
            if ((delta >= learn_counts && previous >= learn_counts && delta > previous)
                || (delta <= -learn_counts && previous <= -learn_counts && delta < previous)) {
                run.encoder_sign[i] = delta > 0 ? 1 : -1;
                run.encoder_ready |= (uint8_t)(1u << i);
                if (run.encoder_ready == 15u) {
                    uint8_t wheel;
                    text("[encoder directions confirmed] signs=");
                    for (wheel = 0; wheel < MOTOR_COUNT; wheel++) {
                        if (wheel) send_byte(',');
                        signed_number(run.encoder_sign[wheel]);
                    }
                    text("\r\n");
                }
            }
            run.encoder_previous_delta[i] = delta;
            run.encoder_sample_ms[i] = motor_feedback[i].position_ms;
        }
        /* 单轮明显反向不能与其他三轮平均后冒充正常里程。末端微量纠角保留容差。 */
        if (run.encoder_sign[i] && delta * run.encoder_sign[i] < -learn_counts) {
            text("[encoder direction conflict] wheel=");number(i + 1u);text("\r\n");
            return 0;
        }
        sum += delta * run.encoder_sign[i];
    }
    if (run.encoder_ready != 15u) {
        if ((uint32_t)(clock_ms - run.phase_ms) >= CAR_ENCODER_LEARN_TIMEOUT_MS) {
            text("[encoder directions unconfirmed] mask=");number(run.encoder_ready);text("\r\n");
            return 0;
        }
        return 1;
    }
    run.progress_mm = (int32_t)((float)sum * CAR_POSITION_PULSES_PER_REV
                          / (4.0f * CAR_ENCODER_COUNTS_PER_REV * ppm))
                        - run.encoder_offset_mm;
    while (!run.obstacle_phase && run.continuation_mm && run.action_mm
           && run.progress_mm >= run.action_mm) {
        if (!run_pass_straight()) return 0;
    }
    if (run.progress_mm < -CAR_DISTANCE_OVERRUN_MM
        || (run.obstacle_phase != 1 && run.progress_mm > run.action_mm + CAR_DISTANCE_OVERRUN_MM)) return 0;
    return 1;
}

/* 以指令减速度和延迟预留计算可停车速度；不是对真实制动距离的测量。 */
static int16_t run_forward_target(int32_t remaining)
{
    float mm_per_rpm_s, delay_rpm, braking_rpm;
    int32_t speed, max_rpm;
    int sign = run.travel_sign;
    if (run.travel_axis == BODY_LEFT && remaining < -CAR_DISTANCE_TOLERANCE_MM) {
        remaining = -remaining;sign = -sign;
    }
    if (remaining <= CAR_DISTANCE_TOLERANCE_MM) return 0;
    speed = remaining * CAR_DISTANCE_KP_X100 / 100;
    mm_per_rpm_s = CAR_POSITION_PULSES_PER_REV
        / (60.0f * (run.travel_axis == BODY_LEFT ? CALIBRATED_PPM_X : CALIBRATED_PPM_Y));
    delay_rpm = CAR_DRIVE_DECEL_RPM_PER_S * (CAR_DRIVE_BRAKE_LOOKAHEAD_MS / 1000.0f);
    braking_rpm = sqrtf(delay_rpm * delay_rpm
        + 2.0f * CAR_DRIVE_DECEL_RPM_PER_S * (remaining - CAR_DISTANCE_TOLERANCE_MM) / mm_per_rpm_s) - delay_rpm;
    if (speed > (int32_t)braking_rpm) speed = (int32_t)braking_rpm;
    if (speed < CAR_DISTANCE_MIN_RPM) speed = CAR_DISTANCE_MIN_RPM;
    max_rpm = run.travel_axis == BODY_LEFT
        ? (run.lateral_rpm ? run.lateral_rpm : CAR_LATERAL_MAX_RPM) : CAR_RUN_RPM;
    if (run.obstacle_phase == 2 && max_rpm > CAR_OBSTACLE_RETURN_RPM) max_rpm = CAR_OBSTACLE_RETURN_RPM;
    if (speed > max_rpm) speed = max_rpm;
    if (run.station_shift) {
        float index_rpm = (run.job == 3u || run.job == 6u) ? STN_STORAGE_INDEX_RPM : STN_ROUGH_INDEX_RPM;
        if (speed > index_rpm) speed = index_rpm;
    }
    if (run.encoder_ready != 15u && speed > CAR_ENCODER_LEARN_RPM) speed = CAR_ENCODER_LEARN_RPM;
    return (int16_t)(speed * sign);
}

static int16_t run_forward_slew(int16_t desired)
{
    uint32_t elapsed = clock_ms - run.forward_profile_ms;
    int32_t target = (desired < 0 ? -(int32_t)desired : desired) * 1000;
    int sign = desired < 0 ? -1 : desired > 0 ? 1 : run.profile_sign;
    uint32_t rate;
    int32_t change;
    if (!run.forward_mrpm) run.profile_sign = (int8_t)sign;
    if (run.profile_sign != sign) target = 0;
    rate = target > run.forward_mrpm ? CAR_DRIVE_ACCEL_RPM_PER_S : CAR_DRIVE_DECEL_RPM_PER_S;
    /* 串口输出或ACK等待后不追补大步进；保持单次指令变化量有界。 */
    if (elapsed > CAR_DRIVE_SLEW_MAX_DT_MS) elapsed = CAR_DRIVE_SLEW_MAX_DT_MS;
    change = (int32_t)(rate * elapsed);
    if (target > run.forward_mrpm) {
        run.forward_mrpm += change;
        if (run.forward_mrpm > target) run.forward_mrpm = target;
    } else {
        run.forward_mrpm -= change;
        if (run.forward_mrpm < target) run.forward_mrpm = target;
    }
    run.forward_profile_ms = clock_ms;
    return (int16_t)(run.forward_mrpm / 1000 * run.profile_sign);
}

/* 每个处理到的新IMU样本只记一次；打印窗口清零不重记同一序号。 */
static void run_heading_window_sample(void)
{
    HeadingControl *h = &run.heading_control;
    if (run.heading_window.seen && run.heading_window.sequence == h->sample_sequence) return;
    run.heading_window.seen = 1;
    run.heading_window.sequence = h->sample_sequence;
    if (!run.heading_window.samples || h->error_cdeg < run.heading_window.min_cdeg) {
        run.heading_window.min_cdeg = h->error_cdeg;
        run.heading_window.min_mm = run.progress_mm;
        run.heading_window.min_ms = h->sample_ms;
    }
    if (!run.heading_window.samples || h->error_cdeg > run.heading_window.max_cdeg) {
        run.heading_window.max_cdeg = h->error_cdeg;
        run.heading_window.max_mm = run.progress_mm;
        run.heading_window.max_ms = h->sample_ms;
    }
    run.heading_window.samples++;
}

static void run_velocity_log(void)
{
    uint8_t i;
    uint32_t age = 0;
    if ((uint32_t)(clock_ms - run.log_ms) < CAR_CONTROL_LOG_MS) return;
    run.log_ms = clock_ms;
    text("[control] phase="); number(run.velocity_phase);
    text(" target_yaw_cdeg="); signed_number(heading_wrap_cdeg((int32_t)run.heading_control.last_yaw_cdeg + run.heading_control.error_cdeg));
    text(" yaw_cdeg="); signed_number(run.heading_control.last_yaw_cdeg);
    text(" error_cdeg="); signed_number(run.heading_control.error_cdeg);
    text(" rate_cdeg_s="); signed_number(run.heading_control.rate_cdeg_s);
    text(" distance_mm="); signed_number(run.progress_mm);
    text(" target_mm="); number(run.action_mm);
    if(run.vector_move) {
        text(" vector_target_mm=");signed_number(run.vector_target_mm[0]);text(",");signed_number(run.vector_target_mm[1]);
        text(" vector_progress_mm=");signed_number(run.vector_progress_mm[0]);text(",");signed_number(run.vector_progress_mm[1]);
    }
    if (run.travel_axis == BODY_LEFT) text(run.travel_sign < 0 ? " travel=RIGHT" : " travel=LEFT");
    else text(run.travel_sign < 0 ? " travel=BACKWARD" : " travel=FORWARD");
    text(" ack="); number(run.acks);
    text(" fresh_mask="); number(run.ready);
    if (run.continuation_mm) {text(" continue_mm=");number(run.continuation_mm);}
    if (run.travel_axis == BODY_FORWARD && (run.velocity_phase == VELOCITY_FORWARD || run.velocity_phase == VELOCITY_STOP_FORWARD)) {
        text(" t_ms="); number(clock_ms);
        text(" v_ref="); signed_number(run.forward_ref_rpm);
        text(" v_cmd="); signed_number(run.forward_cmd_rpm);
        text(" turn_ref="); signed_number(run.turn_ref_rpm);
        text(" turn_cmd="); signed_number(run.turn_cmd_rpm);
        text(" control_dt_ms="); number(run.control_dt_ms);
        text(" wheel_cmd_rpm=");
        for (i = 0; i < MOTOR_COUNT; i++) {if (i) text(",");signed_number(run.wheel_rpm[i]);}
        text(" wheel_rpm_raw=");
        for (i = 0; i < MOTOR_COUNT; i++) {
            uint32_t elapsed = clock_ms - motor_feedback[i].speed_ms;
            if (elapsed > age) age = elapsed;
            if (i) text(",");
            signed_number(motor_feedback[i].speed_rpm);
        }
        text(" speed_age_max_ms="); number(age);
    }
    if ((run.velocity_phase == VELOCITY_FORWARD || run.velocity_phase == VELOCITY_STOP_FORWARD)
        && run.heading_window.samples) {
        /* 紧凑字段减少阻塞串口开销：样本数,min角差/里程/时间,max角差/里程/时间。 */
        text(" yaw_win="); number(run.heading_window.samples);
        text(","); signed_number(run.heading_window.min_cdeg);
        text(","); signed_number(run.heading_window.min_mm);
        text(","); number(run.heading_window.min_ms);
        text(","); signed_number(run.heading_window.max_cdeg);
        text(","); signed_number(run.heading_window.max_mm);
        text(","); number(run.heading_window.max_ms);
        run.heading_window.samples = 0;
    }
    text("\r\n");
}

static void run_return_wait(void)
{
    run_active = 0;run.state = RUN_WAIT_OBSTACLE;
    run.obstacle_phase = 3;run.turning = 0;run.forward_mrpm = 0;
    run.pending = 0;run.transition_ready = 0;pose_set = 0;nav.position_confirmed = 0;
    text("[obstacle return stopped] node_est=");send_byte((uint8_t)('A' + run.obstacle_node));
    text("; encoder-based; physical_pose=unconfirmed; waiting; no automatic resume\r\n");
    run_status();
}

static void run_return_next(void)
{
    RunLeg leg;
    if (!run.return_index) {run_return_wait();return;}
    leg = run.road_path[--run.return_index];
    run.travel_axis = leg.axis;run.travel_sign = (int8_t)-leg.sign;
    run.target_heading = leg.heading;run.remaining_mm = leg.mm;run.continuation_mm = 0;
    run.lateral_rpm = CAR_OBSTACLE_RETURN_RPM < CAR_LATERAL_MAX_RPM
                      ? CAR_OBSTACLE_RETURN_RPM : CAR_LATERAL_MAX_RPM;
    run.station_align_pending = run.transition_ready = 0;
    run.obstacle_phase = 2;
    text("[obstacle returning] node_est=");send_byte((uint8_t)('A' + run.obstacle_node));
    text(" distance_mm=");number(leg.mm);text(" remaining_parts=");number(run.return_index);
    text(" rpm_limit=" CAR_STRING_VALUE(CAR_OBSTACLE_RETURN_RPM) "; physical_pose=unconfirmed\r\n");
    run_begin_alignment();
}

/* obstacle是外部触发入口；此工程没有自动检测障碍物的传感器。 */
static int run_obstacle_request(void)
{
    if (!run_active || !run.imu_mode || run.turn_only || run.translation_only) return 0;
    if (run.obstacle_phase) {run_fault("obstacle repeated during stop/return; hold position");return 1;}
    run.obstacle_preflight = run.state == RUN_PREFLIGHT;
    run.obstacle_leg_active = !run.turning
        && (run.velocity_phase == VELOCITY_FORWARD || run.velocity_phase == VELOCITY_STOP_FORWARD);
    run.obstacle_node = full_route_nodes[run.plan][run.edge];
    if (!run.obstacle_preflight && run.road.count && !run.station_align_pending
        && run.edge + 1u < FULL_ROUTE_NODES) {
        run.blocked_roads |= (uint16_t)(1u << run.road.road);
        run.completed_roads &= (uint16_t)~(1u << run.road.road);
    }
    run.obstacle_phase = 1;run.alignment_waiting=0;run.state = RUN_OBSTACLE_STOP;run.phase_ms = clock_ms;
    /* 障碍停车绕过正常斜坡和尚未收齐的速度ACK；之后以停车后的新反馈确认。 */
    motion_stop(&drive);
    if (stop_pending || run_rx_overflow || can_rx_fault) {run_fault("obstacle stop command failed");return 1;}
    run.velocity_waiting = run.velocity_sent = run.pending = 0;
    memset(run.wheel_rpm,0,sizeof(run.wheel_rpm));
    run.zero_ms = clock_ms ? clock_ms : 1u;run.stopped_ms = 0;
    text("[obstacle stopping] return_node_est=");send_byte((uint8_t)('A' + run.obstacle_node));
    text("; reverse only after four-wheel stop and fresh feedback\r\n");
    return 1;
}

/* 反馈不足时只暂停底盘输出，保留本段和工位现场；新数据恢复后继续。 */
static void run_wait_feedback(const char *reason)
{
    run_warning(reason);
    run.forward_mrpm=0;run.forward_profile_ms=clock_ms;run.phase_ms=clock_ms;
    if(run.vector_move) {
        run.vector_mrpm[0]=run.vector_mrpm[1]=0;
        run.vector_profile_ms[0]=run.vector_profile_ms[1]=clock_ms;
    }
    if ((run.state!=RUN_PREFLIGHT || run.velocity_sent)
        && !run_velocity_send(0,0,0)) {run_fault("velocity batch submit failed");return;}
    run_velocity_query();
}

static void run_velocity_update(void)
{
    RunCanFrame frame;
    HeadingDecision decision;
    uint8_t i;
    uint8_t drive_heading, control_due;
    int16_t forward = 0, left = 0, turn = 0;
    HeadingSample sample = heading_snapshot();
    while (run_read_frame(&frame)) {
        uint8_t address = (uint8_t)(frame.id >> 8);
        if (!run_record_frame(&frame)) continue;
        if (frame.data[0] == MOTOR_VELOCITY_COMMAND) {
            if (frame.data[1] != MOTOR_POSITION_ACK) {run_fault("velocity command rejected");return;}
            if (run.velocity_waiting && (uint32_t)(frame.received_ms - run.velocity_ms) < CAR_VELOCITY_ACK_MS) {
                run.acks |= (uint8_t)(1u << (address - 1u));
                if (run.acks == ALL_MOTORS_MASK) run.velocity_waiting = 0;
            }
        }
        if (frame.data[0] == MOTOR_STATUS_COMMAND
            && (!(frame.data[1] & MOTOR_FLAG_ENABLED) || (frame.data[1] & (MOTOR_FLAG_STALLED | MOTOR_FLAG_PROTECTED)))) {
            run_fault("motor disabled or stalled");return;
        }
        if (address == run.pending && frame.data[0] == run.pending_command
            && (int32_t)(frame.received_ms - run.query_ms) >= 0) {
            if (frame.data[0] == MOTOR_STATUS_COMMAND) motor_feedback[address - 1u].action_status_replies++;
            run.pending = 0;
            run.velocity_query_index = (uint8_t)((run.velocity_query_index + 1u) % 12u);
        }
    }
    if (run.pending && (uint32_t)(clock_ms - run.query_ms) >= CAR_VELOCITY_REPLY_MS) {
        run_warning("velocity feedback reply timeout; continue polling");
        run.pending=0;
        run.velocity_query_index=(uint8_t)((run.velocity_query_index+1u)%12u);
        run_wait_feedback("feedback transaction incomplete; await next actual sample");return;
    }
    if (run.velocity_waiting && (uint32_t)(clock_ms - run.velocity_ms) >= CAR_VELOCITY_ACK_MS) {
        run_warning("missing motor velocity ACK; refresh commanded batch");
        run.velocity_waiting=0;run.velocity_sent=0;
    }
    if (imu_uart_fault) {run_warning("IMU UART error; await a fresh valid sample");imu_uart_fault=0;}
    if (!heading_sample_fresh(&sample,clock_ms,CAR_IMU_MAX_AGE_MS)) {
        run_wait_feedback("IMU sample stale/invalid; awaiting feedback");return;
    }
    if (!run_velocity_feedback_fresh()) {
        run_wait_feedback("four-wheel feedback incomplete/stale; awaiting feedback");return;
    }
    if (run.alignment_waiting) {run.alignment_waiting=0;run_begin_alignment();return;}
    if (run.obstacle_phase == 1) {
        if ((uint32_t)(clock_ms - run.phase_ms) >= CAR_HEADING_TIMEOUT_MS) {
            run_warning("obstacle stop delayed; waiting for feedback");run.phase_ms=clock_ms;
        }
        if (run.obstacle_leg_active && !run_velocity_distance()) {run_fault("obstacle stop encoder invalid");return;}
        if (!run_velocity_stopped()) {run_velocity_query();return;}
        for (i = 0;i < MOTOR_COUNT;i++)
            if ((uint32_t)(motor_feedback[i].position_ms - run.stopped_ms) > 0x7fffffffu) {run_velocity_query();return;}
        if (run.obstacle_leg_active) {
            if (!run_velocity_distance() || run.progress_mm < 0 || run.progress_mm > 65535
                || !run_save_leg((uint16_t)run.progress_mm)) {run_fault("obstacle return path invalid");return;}
        }
        run.return_index = run.road_path_count;
        run_return_next();return;
    }
    if (run.state == RUN_PREFLIGHT) {
        for (i = 0; i < MOTOR_COUNT; i++) {
            station_log_wheel_speed(i,"preflight");
        }
        if(run.station_shift && run.translation_only && run.transition_ready) {
            run_begin_alignment();
        } else if (run.turn_only || run.translation_only) {
            run.velocity_phase = VELOCITY_TURN;run.phase_ms = clock_ms;run.turning = 1;
            run.state = RUN_ALIGN;drive.state = MOTION_RUNNING;run.steps++;
        } else if (!run_road()) run_fault("invalid road/start failure");
        return;
    }
    decision = heading_control_update(&run.heading_control, sample, clock_ms);
    if (decision == HEADING_FAULT) {run_wait_feedback(run.heading_control.fault);return;}
    if ((uint32_t)(clock_ms - run.phase_ms) >= (run.turning ? CAR_HEADING_TIMEOUT_MS : CAR_DRIVE_TIMEOUT_MS)) {
        run_warning("motion deadline exceeded; feedback completion still required");
        run.phase_ms=clock_ms;
    }
    for (i = 0; i < MOTOR_COUNT; i++)
        if (run.wheel_rpm[i] && (uint32_t)(clock_ms - run.wheel_start_ms[i]) > CAR_VELOCITY_STALL_MS
            && (uint32_t)(clock_ms - motor_feedback[i].position_changed_ms) > CAR_VELOCITY_STALL_MS) {
            run_fault("wheel position did not follow velocity command");return;
        }
    if (run.velocity_phase == VELOCITY_TURN && (run.heading_control.target_cdeg > 4500 || run.heading_control.target_cdeg < -4500)
        && ((run.heading_control.target_cdeg > 0 && run.heading_control.progress_cdeg < -500)
            || (run.heading_control.target_cdeg < 0 && run.heading_control.progress_cdeg > 500))) {
        run_fault("IMU rotation direction mismatch");return;
    }
    if (run.velocity_phase == VELOCITY_BASELINE) {
        if (run.departure_overlap && !CAR_ARM_TRANSLATION_OVERLAP_ENABLED) {run_velocity_query();return;}
        for (i = 0; i < MOTOR_COUNT; i++)
            if ((uint32_t)(motor_feedback[i].position_ms - run.baseline_after_ms) > 0x7fffffffu) {run_velocity_query();return;}
        for (i = 0; i < MOTOR_COUNT; i++) run.encoder_start[i] = motor_feedback[i].position;
        memset(run.encoder_sign, 0, sizeof(run.encoder_sign));
        memset(run.encoder_previous_delta, 0, sizeof(run.encoder_previous_delta));
        for (i = 0; i < MOTOR_COUNT; i++) run.encoder_sample_ms[i] = motor_feedback[i].position_ms;
        run.encoder_ready = 0;
        memset(run.vector_progress_mm,0,sizeof(run.vector_progress_mm));
        memset(run.vector_mrpm,0,sizeof(run.vector_mrpm));
        memset(run.vector_profile_sign,0,sizeof(run.vector_profile_sign));
        memset(run.vector_wheel_sign,0,sizeof(run.vector_wheel_sign));
        run.vector_profile_ms[0]=run.vector_profile_ms[1]=clock_ms;
        run.progress_mm = 0;run.action_mm = run.remaining_mm;run.encoder_offset_mm = 0;
        run.forward_mrpm = 0;run.forward_profile_ms = clock_ms;
        run.forward_ref_rpm = run.forward_cmd_rpm = run.turn_ref_rpm = run.turn_cmd_rpm = 0;
        run.control_dt_ms = 0;
        memset(&run.heading_window, 0, sizeof(run.heading_window));
        run.velocity_phase = VELOCITY_FORWARD;run.state = RUN_MOVING;
        run.phase_ms = clock_ms;run.steps++;run.distance_mm += run.action_mm;
        run_status();
    }
    if (run.velocity_phase == VELOCITY_FORWARD || run.velocity_phase == VELOCITY_STOP_FORWARD) {
        if (!run_velocity_distance()) {run_fault("encoder distance overrun/reversed");return;}
        run_heading_window_sample();
        if (run.velocity_phase == VELOCITY_FORWARD) {
            int32_t remaining = (int32_t)run.action_mm - run.progress_mm + run.continuation_mm;
            if (run.heading_control.error_cdeg > CAR_HEADING_MAX_DRIFT_CDEG || run.heading_control.error_cdeg < -CAR_HEADING_MAX_DRIFT_CDEG) {
                run_fault("IMU excessive translation heading error");return;
            }
            if(run.vector_move) {
                forward=run_vector_target(0);left=run_vector_target(1);
                if(!forward && !left)run.velocity_phase=VELOCITY_STOP_FORWARD;
            } else if (run.travel_axis == BODY_LEFT) {
                if (remaining <= CAR_DISTANCE_TOLERANCE_MM && remaining >= -CAR_DISTANCE_TOLERANCE_MM)
                    run.velocity_phase = VELOCITY_STOP_FORWARD;
                else left = run_forward_target(remaining);
            } else if (remaining <= CAR_DISTANCE_TOLERANCE_MM) run.velocity_phase = VELOCITY_STOP_FORWARD;
            else {
                forward = run_forward_target(remaining);
            }
        } else if(run.vector_move) {
            if(run_vector_target(0) || run_vector_target(1)) {
                run.velocity_phase=VELOCITY_FORWARD;run.stopped_ms=0;
            }
        } else if (run.travel_axis == BODY_LEFT
                   && ((int32_t)run.action_mm - run.progress_mm > CAR_DISTANCE_TOLERANCE_MM
                       || run.progress_mm - (int32_t)run.action_mm > CAR_DISTANCE_TOLERANCE_MM)) {
            /* 减速/末端纠角产生的行程漂移必须重新收敛，不能在距离超差时报告完成。 */
            run.velocity_phase = VELOCITY_FORWARD;
            run.stopped_ms = 0;
        }
    }
    drive_heading = run.travel_axis == BODY_FORWARD
        && (run.velocity_phase == VELOCITY_FORWARD || run.velocity_phase == VELOCITY_STOP_FORWARD);
    control_due = (uint32_t)(clock_ms - run.control_ms) >= CAR_VELOCITY_CONTROL_MS && !run.velocity_waiting;
    if(run.vector_move && (run.velocity_phase==VELOCITY_FORWARD || run.velocity_phase==VELOCITY_STOP_FORWARD)) {
        run.forward_ref_rpm=forward;
        forward=run_vector_slew(0,forward,control_due);left=run_vector_slew(1,left,control_due);
    }
    if (run.travel_axis == BODY_LEFT
        && (run.velocity_phase == VELOCITY_FORWARD || run.velocity_phase == VELOCITY_STOP_FORWARD))
        left = control_due ? run_forward_slew(left) : (int16_t)(run.forward_mrpm / 1000 * run.profile_sign);
    if (drive_heading) {
        if(!run.vector_move) {
            run.forward_ref_rpm = forward;
            forward = control_due ? run_forward_slew(forward) : (int16_t)(run.forward_mrpm / 1000 * run.profile_sign);
        }
        /* 行驶时连续阻尼；前进指令归零后，沿用已验证的停车对齐/停稳判据。 */
        turn = (forward || (run.vector_move && left)) ? heading_control_drive_rpm(&run.heading_control, CAR_HEADING_DRIVE_TRIM_RPM)
                       : heading_control_rpm(&run.heading_control, run.turn_rpm);
    } else turn = heading_control_rpm(&run.heading_control, run.turn_rpm);
    run.turn_ref_rpm = turn;
    if (forward || left) {
        int16_t primary = run.vector_move ? (abs(left)>abs(forward) ? left : forward) : left ? left : forward;
        int16_t limit = (int16_t)((primary < 0 ? -primary : primary) - 1);
        if (limit > CAR_HEADING_DRIVE_TRIM_RPM) limit = CAR_HEADING_DRIVE_TRIM_RPM;
        if (turn > limit) turn = limit;
        if (turn < -limit) turn = -limit;
    }
    if (left && !run.vector_move) {
        int16_t per_m = left > 0 ? LEFT_FORWARD_MM_PER_M : RIGHT_FORWARD_MM_PER_M;
        /* 沿用前移mm/米的经验补偿，换算为轮速；此项不是位置观测。 */
        float trim = (float)(left > 0 ? left : -left) * per_m * CALIBRATED_PPM_Y
                     / (1000.0f * CALIBRATED_PPM_X);
        forward = (int16_t)(trim >= 0 ? trim + 0.5f : trim - 0.5f);
    }
    if (control_due) {
        run.control_dt_ms = clock_ms - run.control_ms;
        run.control_ms = clock_ms;
        if (!run_velocity_send(forward, left, turn)) {run_fault("velocity batch submit failed");return;}
        run.forward_cmd_rpm = forward;run.turn_cmd_rpm = turn;
    }
    if (!forward && !left && !turn && run_velocity_stopped() && run.heading_control.stable_samples >= CAR_HEADING_STABLE_SAMPLES
        && (uint32_t)(run.heading_control.sample_ms - run.heading_control.stable_ms) >= CAR_HEADING_STABLE_MS) {
        run.heading = run.target_heading;
        if (run.velocity_phase == VELOCITY_TURN) {
            text(station_speed_log_only() ? "[heading aligned; zero commands confirmed; wheel speed logged only]\r\n"
                                         : "[heading aligned; four wheels stopped]\r\n");
            run.log_ms = clock_ms - CAR_CONTROL_LOG_MS;
            run_velocity_log();
            if (run.station_align_pending) {
                run.station_align_pending = 0;
                run.turning = 0;
                text("[station right-side ready] node=");
                send_byte((uint8_t)('A' + full_route_nodes[run.plan][run.edge]));
                text(" heading_dec="); number(run.heading);
                text("; by_IMU_and_four_wheel_stop; physical_pose=unconfirmed\r\n");
                run_finish_node();
                return;
            }
            if (run.scan_turn_stage==1u) {
                run.scan_turn_stage=0;run.turning=0;
                if (station_task_ready()) {
                    /* 扫码已与转向重叠，不再进入扫码工位重复获取机构互锁。
                     * 下一段仍按IMU确认出站朝向，不把途中扫码当作航向已到位。 */
                    run.job++;run.transition_ready=1;
                    text("[scan station] task latched; depart directly on route heading\r\n");
                    if (!run_road()) run_fault("scan departure road failed");
                } else run_finish_node();
                return;
            }
            if (run.turn_only) {run_done();return;}
            run_begin_baseline();
        } else if (run.velocity_phase == VELOCITY_STOP_FORWARD) {
            /* 即使不足500ms，也在段末补齐尚未打印的样本。 */
            if (run.heading_window.samples) {
                run.log_ms = clock_ms - CAR_CONTROL_LOG_MS;
                run_velocity_log();
            }
            text("[control segment done] distance_mm=");signed_number(run.progress_mm);
            text(" error_cdeg=");signed_number(run.heading_control.error_cdeg);text("\r\n");
            run.remaining_mm = 0;
            if (run.obstacle_phase == 2) {
                if (run.progress_mm - (int32_t)run.action_mm > CAR_DISTANCE_TOLERANCE_MM) {
                    run_fault("obstacle return distance outside tolerance");return;
                }
                run_return_next();return;
            }
            if (run.translation_only) run_done();
            else {
                if (!run_save_leg((uint16_t)run.progress_mm)) {run_fault("road return path full");return;}
                run.transition_ready = 1;run_advance_segment();
            }
            return;
        }
    }
    run_velocity_log();
    run_velocity_query();
}

/* 每轮先检查故障，再处理CAN回复、等待条件和下一动作。
 * 这里不执行工位作业，也不把理论节点当作真实定位结果。
 */
static void run_update(void)
{
    if (run_rx_overflow || (can_rx_fault & 1u)) {
        run_warning("CAN receive queue/FIFO overflow; continue polling");
        run_recover_receive();
    }
#ifdef NAV_ENTRY_HOST_TEST
    if (entry_host_can_bus_off())
    {
        run_fault("CAN bus off");
        return;
    }
#else
    if (CAN1->ESR & CAN_BUS_OFF_FLAG)
    {
        run_fault("CAN bus off");
        return;
    }
#endif
    if (can_rx_fault) {
        run_fault((can_rx_fault & 2u) ? "CAN transmit queue full; command not queued"
                                    : "CAN hardware FIFO overrun; feedback lost");
        return;
    }
    run_velocity_update();
}

/* ===== 6. 串口命令：状态显示、参数解析、模式选择 ===== */
static const char *state_name(NavState state)
{
    static const char *names[] =
    {
        "IDLE", "PLAN", "MOVING", "WAIT_NODE", "WAIT_RETURN", "ARRIVED", "STOPPED", "FAULT"
    };
    return names[state];
}

static void status(void)
{
    if (run.state != RUN_IDLE && (!test_mode || run.turn_only || run.translation_only))
    {
        run_status();
        return;
    }
    if (test_mode)
    {
        static const char *names[] =
        {
            "READY", "RUNNING", "OBSERVE", "DONE", "FAULT"
        };
        text(test_mode == MODE_ROTATION_TEST ? "[turn test] " : "[test] ");
        text(names[drive.state]);
        if (test_mode == MODE_ROTATION_TEST)
        {
            text(" rpm="); number(turn_rpm);
            text(" imu_turn=continuous_velocity");
            text("; RAM only; physical_angle=unconfirmed\r\n");
            return;
        }
        text(" dist=");
        number(test_distance);
        text(" mm rpm=");
        number(test_rpm);
        text(" acc=" CAR_STRING_VALUE(CAR_MOTION_ACCELERATION) " id=");
        number(drive.generation);
        text(" trim_l=");
        signed_number(drive.config.left_forward_mm_per_m);
        text(" trim_r=");
        signed_number(drive.config.right_forward_mm_per_m);
        text(" mm/m");
        text("; estimates are NOT arrival\r\n");
        return;
    }
    text("[nav] ");
    text(state_name(nav.state));
    text(" current=");
    send_byte((uint8_t)('A' + nav.current));
    text(" goal=");
    send_byte((uint8_t)('A' + nav.target));
    text(" next=");
    send_byte((uint8_t)('A' + nav.next));
    text(" pose=");
    text(pose_set && nav.position_confirmed ? "confirmed" : "unconfirmed");
    text(" blocked=");
    number(nav.blocked);
    text(" passed=");
    number(nav.passed);
    text(" error=");
    number((uint32_t) nav.error);
    text("\r\n");
}

static int parse_uint(const char *s, uint16_t *value)
{
    uint32_t n = 0;
    if (! * s)
        return 0;
    while (*s)
    {
        if (*s < '0' || *s > '9')
            return 0;
        n = n * 10u + (uint32_t)(*s++ - '0');
        if (n > 65535u)
            return 0;
    }
    * value = (uint16_t) n;
    return 1;
}

static int parse_node(const char *s, uint8_t *node)
{
    if (s[0] < 'A' || s[0] > 'I' || s[1])
        return 0;
    * node = (uint8_t)(s[0] - 'A');
    return 1;
}

static void help(void)
{
    text("VISION: vision debug [D|E|F 1..4] = standalone correction, no grip/route; vision debug off = stop\r\n");
    text("ARM: arm on (wait four-wheel stop) | arm status | arm pose SLIDE LIFT YAW CLAW\r\n");
    text("ARM: arm slide/lift/yaw N | arm open/close | arm pick/place/cycle | arm home\r\n");
    text("ARM: arm platform pose DEG | arm platform 1/2/3 | arm stop | arm off (park first)\r\n");
    text("ARM: arm load1 | arm load3 (or @) = calibrated loading sequences\r\n");
    text("STATION: task 134+123+314+231 | station | station auto on/off (default on)\r\n");
    text("VISION: vision status/ping/range/pick N/clear; UART5 AA55+CRC8\r\n");
    text("screen main/chassis/arm/sensors/refresh = read-only USART6 display\r\n");
    text("qr = scanner status | qr init | qr baud 9600/115200 (MCU only)\r\n");
    text("runimu A/C = continuous IMU turn/drive + encoder distance; imu / feedback = diagnostics\r\n");
    text("turn = IMU closed-loop rotation test | rpm 20..300\r\n");
    text("TURN: left90 / right90 / left180 / right180 = IMU relative angle closed-loop\r\n");
    text("runimu obstacle = stop, return by encoder to last tracked node, then wait; no automatic detector\r\n");
    text("test = single-segment calibration | nav = return to navigation (stops)\r\n");
    text("TEST: dist 50..2000 mm | rpm 20..300 | f/b/l/r (body directions)\r\n");
    text("TEST l/r: IMU heading + encoder distance, lateral rpm limited to " CAR_STRING_VALUE(CAR_LATERAL_MAX_RPM) "; fresh IMU required\r\n");
    text("Inspect actual stop and reset physical start before the next direction key.\r\n");
    text("pose A | heading 0..3 (+Y,-X,-Y,+X) | go H | rpm 20..300 (0=road table)\r\n");
    text("confirm (segment) | node B (actual arrival) | obstacle | return A\r\n");
    text("block 0..11 | unblock 0..11 | progress measured_mm | info | help\r\n");
    text("! = immediate stop request (no Enter); s/z stop after Enter\r\n");
    text("MANUAL TEST: estimates never confirm arrival. No automatic return.\r\n");
}

/* f/b/l/r 只启动一个车身方向的标定段。 */
static int start_test(char direction)
{
    MotionParameters parameters;
    int result;
    if (stop_pending || drive.state == MOTION_RUNNING)
        return 0;
    parameters.axis = (direction == 'f' || direction == 'b') ? BODY_FORWARD : BODY_LEFT;
    parameters.mm = (direction == 'b'
        || direction == 'r') ? - (int16_t) test_distance : (int16_t) test_distance;
    parameters.rpm = test_rpm;
    parameters.acceleration = CAR_MOTION_ACCELERATION;
    if (parameters.axis == BODY_LEFT) return start_heading_shift(parameters.mm, parameters.rpm);
    run.translation_only = run.turn_only = 0;
    run.state = RUN_IDLE;
    /* 新方向键是操作者主动开始下一次试验；先请求停止旧命令。
     * 不把等待估时、超时或停止当作实际到位，也不恢复节点位置。
     */
    motion_stop(&drive);
    result = port_start(&drive, parameters, clock_ms);
    if (result)
    {
        text("[test start] ");
        send_byte((uint8_t) direction);
        text(" dist=");
        number(test_distance);
        text(" mm rpm=");
        number(test_rpm);
        text(" acc=" CAR_STRING_VALUE(CAR_MOTION_ACCELERATION) " base_pulses=");
        number(drive.pulses);
        text("; no automatic next segment\r\n");
    }
    return result;
}

static int command_turn_test(const char *command_text)
{
    uint16_t value;
    int accepted = 0;
    if (drive.state == MOTION_RUNNING || stop_pending)
        return 0;
    if (!strncmp(command_text, "rpm ", 4) && parse_uint(command_text + 4, &value)
        && value >= 20 && value <= 300)
    {
        turn_rpm = value;
        accepted = 1;
    }
    else if (!strcmp(command_text, "left90") || !strcmp(command_text, "right90"))
        accepted = start_heading_turn(command_text[0] == 'r' ? -90 : 90, turn_rpm);
    else if (!strcmp(command_text, "left180") || !strcmp(command_text, "right180"))
        accepted = start_heading_turn(command_text[0] == 'r' ? -180 : 180, turn_rpm);
    if (accepted && !run_active && run.turn_only) {
        run.turn_only = 0;
        run.state = RUN_IDLE;
    }
    return accepted;
}

static int command_translation_test(const char *command_text)
{
    uint16_t value;
    int accepted = 0;
    if (!strncmp(command_text, "dist ", 5) && parse_uint(command_text + 5, &value)
        && value >= 50 && value <= 2000
        && drive.state != MOTION_RUNNING)
    {
        test_distance = value;
        accepted = 1;
    }
    else if (!strncmp(command_text, "rpm ", 4) && parse_uint(command_text + 4, &value)
        && value >= 20 && value <= 300
        && drive.state != MOTION_RUNNING)
    {
        test_rpm = value;
        accepted = 1;
    }
    else if (!command_text[1] && (command_text[0] == 'f' || command_text[0] == 'b'
        || command_text[0] == 'l' || command_text[0] == 'r'))
        accepted = start_test(command_text[0]);
    return accepted;
}

/* 状态屏只读取控制现场，不查询电机、不推进任务、不消费视觉偏差。 */
static void screen_snapshot(HmiSnapshot *snapshot)
{
    static const uint8_t addresses[4]={HMI_LF_ADDRESS,HMI_RF_ADDRESS,HMI_LR_ADDRESS,HMI_RR_ADDRESS};
    unsigned i;int slide,lift;float yaw,claw;uint8_t healthy=1,seen=0;
    HeadingSample imu=heading_snapshot();
    for(i=0;i<HMI_FIELDS;i++)snapshot->value[i]=HMI_UNKNOWN;
    station_task_words(&snapshot->value[HMI_TASKA],&snapshot->value[HMI_TASKB]);
    snapshot->value[HMI_TASKOK]=station_task_ready();
    snapshot->value[HMI_PICK]=snapshot->value[HMI_PLACE]=-1;
    snapshot->value[HMI_VOLTAGE]=-1;
    if(screen_clock.running) {
        screen_clock.elapsed_ms=clock_ms-screen_clock.start_ms;
        if(run.state==RUN_DONE || run.state==RUN_FAULT || run.state==RUN_STOPPED)screen_clock.running=0;
    }
    snapshot->value[HMI_ELAPSED]=(int32_t)(screen_clock.elapsed_ms/1000u);
    snapshot->value[HMI_PHASE]=screen_clock.started ? (run.job>=7u ? 8 : run.job+1u) : 0;
    if(run.state==RUN_DONE && !run.turn_only && !run.translation_only)snapshot->value[HMI_PHASE]=9;
    if(run.state==RUN_STOPPED)snapshot->value[HMI_PHASE]=0;
    if(run.state==RUN_FAULT || (run.state!=RUN_STOPPED && (arm_failed() || arm_lock.state==3)))
        snapshot->value[HMI_PHASE]=10;
    for(i=0;i<4;i++) {
        MotorFeedback *f=&motor_feedback[i];
        if(f->speed_valid || f->status_replies)seen=1;
        if(!f->speed_valid || !f->status_replies || (uint32_t)(clock_ms-f->speed_ms)>500u
            || (uint32_t)(clock_ms-f->status_ms)>500u || !(f->status_flags&1u) || (f->status_flags&12u))healthy=0;
    }
    snapshot->value[HMI_COMM]=!seen ? 0 : healthy && !can_rx_fault ? 1 : 2;
    for(i=0;i<4;i++)if(addresses[i]>=1u && addresses[i]<=4u) {
        unsigned index=addresses[i]-1u;MotorFeedback *f=&motor_feedback[index];
        if(run_active)snapshot->value[HMI_LF_T+i*2u]=run.wheel_rpm[index];
        else if(arm_lock.state==2)snapshot->value[HMI_LF_T+i*2u]=0;
        if(f->speed_valid && (uint32_t)(clock_ms-f->speed_ms)<=500u)
            snapshot->value[HMI_LF_R+i*2u]=f->speed_rpm;
    }
    snapshot->value[HMI_MECHANISM]=!arm_enabled() ? 0 : arm_busy() ? 1 : -1;
    if(arm_pose_known()) {
        arm_target_now(&slide,&lift,&yaw,&claw);
        snapshot->value[HMI_LIFT]=lift;snapshot->value[HMI_PUSH]=slide;
        snapshot->value[HMI_GRIP]=(int32_t)claw;snapshot->value[HMI_PAN]=(int32_t)yaw;
    }
    if(arm_platform_known())snapshot->value[HMI_TRAY]=arm_platform_slot() ? arm_platform_slot() : HMI_UNKNOWN;
    snapshot->value[HMI_IMU_AGE]=imu.valid ? (int32_t)(clock_ms-imu.received_ms) : -1;
    if(heading_sample_fresh(&imu,clock_ms,CAR_IMU_MAX_AGE_MS))snapshot->value[HMI_YAW]=imu.yaw_cdeg/10;
    if(run_active || run.state==RUN_STATION)
        snapshot->value[HMI_YAW_T]=heading_wrap_cdeg((int32_t)run.heading_control.last_yaw_cdeg+run.heading_control.error_cdeg)/10;
    vision_display(clock_ms,&snapshot->value[HMI_COLOR],&snapshot->value[HMI_DX],
        &snapshot->value[HMI_DY],&snapshot->value[HMI_VISION_AGE]);
}
static void screen_update(void)
{
    static HmiSnapshot snapshot;
    screen_snapshot(&snapshot);hmi_update(&snapshot,clock_ms);
}
static int screen_command(const char *s)
{
    if(!strcmp(s,"screen") || !strcmp(s,"screen status")) {
        text("[screen] USART6 PC6 TX 9600 8N1; read-only; tx_busy=");number(hmi_busy());
        text("; physical screen reception unconfirmed\r\n");return 1;
    }
    if(!strcmp(s,"screen main"))hmi_select(0,0);
    else if(!strcmp(s,"screen chassis"))hmi_select(1,0);
    else if(!strcmp(s,"screen arm"))hmi_select(1,1);
    else if(!strcmp(s,"screen sensors"))hmi_select(1,2);
    else if(!strcmp(s,"screen refresh"))hmi_resync();
    else return 0;
    text("[screen] display update requested\r\n");return 1;
}

/* 先处理公共命令和模式切换，再交给当前模式；只裁剪首尾空白。 */
static void command(char *command_text)
{
    char *trimmed_end;
    uint16_t value;
    uint8_t node;
    int accepted = 0;
    while (*command_text == ' ' || *command_text == '\t')
        command_text++;
    trimmed_end = command_text + strlen(command_text);
    while (trimmed_end > command_text && (trimmed_end[-1] == ' ' || trimmed_end[-1] == '\t'))
        trimmed_end--;
    * trimmed_end = 0;
    if (! * command_text)
        return;
    if(screen_command(command_text))return;
    if (!strcmp(command_text, "info"))
    {
        status();
        return;
    }
    if (!strcmp(command_text, "feedback")) {
        feedback_status();
        return;
    }
    if (!strcmp(command_text, "imu")) {
        imu_status();
        return;
    }
    if (!strcmp(command_text, "qr")) {scanner_status();return;}
    if (!strcmp(command_text, "qr init")) {scanner_init(scanner.baud ? scanner.baud : CAR_SCANNER_UART_BAUD);return;}
    if (!strcmp(command_text, "qr baud 9600") || !strcmp(command_text, "qr baud 115200")) {
        scanner_init(!strcmp(command_text + 8, "9600") ? 9600u : 115200u);return;
    }
    if (!strcmp(command_text,"s") || !strcmp(command_text,"z") || !strcmp(command_text,"S") || !strcmp(command_text,"Z")) {stop_pending=1;return;}
    if(vision_debug_command(command_text))return;
    if (!strncmp(command_text,"vision",6)) {
        if ((run_active || run.state==RUN_STATION) && strcmp(command_text,"vision") && strcmp(command_text,"vision status"))
            text("[vision rejected: route/station owns request]\r\n");
        else (void)vision_command(command_text,clock_ms);
        return;
    }
    if (!strncmp(command_text,"task ",5) || !strncmp(command_text,"station",7)
        || (command_text[0]>='1' && command_text[0]<='5')) {
        if ((run_active || run.state==RUN_STATION) && strcmp(command_text,"station")
            && !(run.state==RUN_STATION && !run.job && strncmp(command_text,"station auto ",13)))
            text("[task rejected: route/station active]\r\n");
        else (void)station_command(command_text);
        return;
    }
    if (!strcmp(command_text,"obstacle") && (run.state==RUN_STATION || run.station_shift || run.departure_overlap)) {run_fault("obstacle during station work; no automatic reverse");return;}
    if (arm_serial_command(command_text)) return;
    if (arm_lock.state || arm_enabled()) {
        int allow_run=station_auto_enabled() && arm_lock.state==2 && arm_ready() && arm_parked()
            && (!strcmp(command_text,"runimu A") || !strcmp(command_text,"runimu C"));
        int allow_obstacle=!strcmp(command_text,"obstacle") && run_active;
        if (!allow_run && !allow_obstacle) {
            if (!strcmp(command_text,"help")) help();
            else text("[rejected: arm session owns motion; park/declare arm for auto run, or arm off for manual chassis]\r\n");
            return;
        }
    }
    if (!strcmp(command_text,"obstacle") && run.imu_mode && (run_active || run.state == RUN_WAIT_OBSTACLE)) {
        accepted = run_obstacle_request();
        text(accepted ? "[accepted]\r\n" : "[rejected: obstacle return inactive]\r\n");return;
    }
    if (!strcmp(command_text, "runimu A") || !strcmp(command_text, "runimu C")) {
        accepted = start_run(command_text[7] == 'C' ? NODE_C : NODE_A);
        text(accepted ? "[accepted]\r\n" : "[rejected: runimu busy/stop pending/heading unavailable]\r\n");
        return;
    }
    if (!strcmp(command_text, "help"))
    {
        help();
        return;
    }
    if (!strcmp(command_text, "test") || !strcmp(command_text, "nav") || !strcmp(command_text, "turn"))
    {
        run_active = 0;
        run.state = RUN_IDLE;
        run_clear_rx();
        nav_stop(&nav);
        nav.position_confirmed = 0;
        pose_set = 0;
        if (!strcmp(command_text, "turn"))
            test_mode = MODE_ROTATION_TEST;
        else if (!strcmp(command_text, "test"))
            test_mode = MODE_TRANSLATION_TEST;
        else
            test_mode = MODE_NAVIGATION;
        text("[accepted]\r\n");
        status();
        if (test_mode == MODE_ROTATION_TEST)
            text("left90/right90/left180/right180 start ONE IMU closed-loop turn; fresh IMU required.\r\n");
        else if (test_mode)
            text("f/b/l/r start ONE segment; inspect actual stop before repeating.\r\n");
        else
            text("Confirm actual pose and heading before go.\r\n");
        return;
    }
    if (run_active)
    {
        text(run.translation_only ? "[rejected: closed-loop shift active; use ! to stop]\r\n"
             : run.turn_only ? "[rejected: closed-loop turn active; use ! to stop]\r\n"
                           : "[rejected: full route active; use ! to stop]\r\n");
        return;
    }
    if (test_mode == MODE_ROTATION_TEST)
    {
        accepted = command_turn_test(command_text);
        text(accepted ? "[accepted]\r\n" : "[rejected: turn command/range/busy/heading unavailable]\r\n");
        return;
    }
    if (test_mode)
    {
        accepted = command_translation_test(command_text);
        text(accepted ? "[accepted]\r\n" : "[rejected: test command/range/busy/heading unavailable]\r\n");
        return;
    }
    if (!strncmp(command_text, "pose ", 5) && parse_node(command_text + 5, &node))
    {
        accepted = nav_confirm_pose(&nav, node);
        if (accepted)
            pose_set = 1;
    }
    else if (!strncmp(command_text, "heading ", 8) && parse_uint(command_text + 8, &value) && value <= 3)
        accepted = nav_set_heading(&nav,(RoadHeading) value);
    else if (!strncmp(command_text, "go ", 3) && pose_set && parse_node(command_text + 3, &node))
        accepted = nav_start(&nav, node, speed_rpm);
    else if (!strncmp(command_text, "rpm ", 4) && parse_uint(command_text + 4, &value) && (!value || (value >= 20
        && value <= 300)) && (nav.state == NAV_IDLE || nav.state == NAV_ARRIVED
        || nav.state == NAV_STOPPED))
    {
        speed_rpm = value;
        accepted = 1;
    }
    else if (!strcmp(command_text, "confirm") && nav.state == NAV_MOVING && drive.state == MOTION_WAIT_CONFIRM)
        accepted = motion_confirm(&drive, drive.generation);
    else if (!strncmp(command_text, "node ", 5) && parse_node(command_text + 5, &node))
        accepted = nav_confirm_node(&nav, node);
    else if (!strcmp(command_text, "obstacle"))
        accepted = nav_obstacle(&nav);
    else if (!strncmp(command_text, "return ", 7) && parse_node(command_text + 7, &node))
        accepted = nav_confirm_return(&nav, node);
    else if (!strncmp(command_text, "block ", 6) && parse_uint(command_text + 6, &value) && value < 12)
        accepted = nav_set_blocked(&nav,(uint8_t) value, 1);
    else if (!strncmp(command_text, "unblock ", 8) && parse_uint(command_text + 8, &value) && value < 12)
        accepted = nav_set_blocked(&nav,(uint8_t) value, 0);
    else if (!strncmp(command_text, "progress ", 9) && parse_uint(command_text + 9, &value))
        accepted = nav_report_progress(&nav, value);
    text(accepted ? "[accepted]\r\n" : "[rejected: command/state/confirmation]\r\n");
}

/* ===== 7. 主循环：停车优先，每轮最多处理一个串口字节 ===== */
/* 只读观察完成后0.5/1/2秒的IMU变化，不延长任务、不发电机命令。 */
static void turn_postcheck_update(void)
{
    HeadingSample sample;
    if (!turn_postcheck.active) return;
    if (run_active || run.state != RUN_DONE || !run.turn_only || drive.state != MOTION_IDLE) {
        turn_postcheck.active = 0;
        return;
    }
    if ((uint32_t)(clock_ms - turn_postcheck.done_ms) < (500u << turn_postcheck.next)) return;
    sample = heading_snapshot();
    text("[turn postcheck] elapsed_ms="); number(clock_ms - turn_postcheck.done_ms);
    text(" yaw_cdeg="); signed_number(sample.yaw_cdeg);
    text(" change_since_done_cdeg="); signed_number(heading_wrap_cdeg((int32_t)sample.yaw_cdeg - turn_postcheck.done_yaw_cdeg));
    text(" fresh="); number(heading_sample_fresh(&sample, clock_ms, CAR_IMU_MAX_AGE_MS));
    text("; observation only\r\n");
    if (++turn_postcheck.next == 3u) turn_postcheck.active = 0;
}

static void firmware_step(void)
{
    uint8_t value,stopped_this_step=stop_pending;
    if (stop_pending)
    {
        uint8_t overflow;
        can_tx_cancel(0);if(run.vision_debug)run.job=0;run.vision_debug=0;
        if (arm_lock.state || arm_enabled()) {
            arm_stop();
            arm_lock.state = 3;
        }
        station_abort();vision_cancel_pick();run.departure_overlap=0;
        if (run.state != RUN_IDLE)
        {
            run_active = 0;
            run.state = RUN_STOPPED;
            run_clear_rx();
        }
        nav_stop(&nav);
        __disable_irq();
        overflow = rx_overflow;
        rx_tail = rx_head;
        stop_pending = 0;
        rx_overflow = 0;
        __enable_irq();
        line_length = 0;
        text(overflow ? "[RX overflow: stopped]\r\n" : "[stopped]\r\n");
        if (run.state == RUN_STOPPED)
            run_status();
    }
    if(!stop_pending)can_tx_update();
    if(start_button_update() && !stopped_this_step && !stop_pending) {
        char start_command[]="runimu C";
        text("[button] requesting runimu C\r\n");
        command(start_command); /* 与串口共用入口，不绕过当前任务占用、IMU或机构准备条件。 */
    }
    scanner_update();
    vision_update(clock_ms);
    arm_lock_update();
    if (read_byte(&value))
    {
        if (value == '\r' || value == '\n')
        {
            if (line_length)
            {
                line[line_length] = 0;
                command(line);
                line_length = 0;
            }
        }
        else if (line_length < LINE_SIZE - 1u)
            line[line_length++] = (char) value;
        else
        {
            stop_pending = 1;
            rx_overflow = 1;
        }
    }
    screen_update();
    if (!stop_pending)
    {
        if (arm_lock.state==2) arm_update(clock_ms);
        if (run.state==RUN_STATION) {run_station_update();return;}
        if(run.departure_overlap) {
            station_update(clock_ms);
            if(station_failed() || arm_failed()) {run_fault("departure parking failed");return;}
            if(!station_active()) {
                if(!arm_ready() || !arm_parked() || arm_platform_slot()!=1) {run_fault("departure parking mismatch");return;}
                run.departure_overlap=0;
                text("[station departure] parking finished\r\n");
            }
        }
        if (run_active && run.station_enabled
            && !(run.station_shift ? station_chassis_pose_safe()
                 : ((arm_ready() && arm_parked()) || (run.departure_overlap && arm_departure_rotation_active())))) {
            run_fault("arm left parked pose during route");return;
        }
        if ((arm_lock.state || arm_enabled()) && !run_active) return;
        if (run_active)
            run_update();
        else if (test_mode)
        {
            MotionState previous = drive.state;
            MotionState state = motion_update(&drive, clock_ms);
            if (state == MOTION_WAIT_CONFIRM && state != previous)
                text("[test estimate elapsed; measure actual distance; NOT arrival]\r\n");
            if (state == MOTION_FAULT)
            {
                motion_stop(&drive);
                text("[test timeout/failure: stop requested; inspect vehicle before next key]\r\n");
            }
        }
        else
            nav_update(&nav, clock_ms);
    }
    if (test_mode || run.state != RUN_IDLE)
    {
        turn_postcheck_update();
        return;
    }
    if (nav.state != reported_nav)
    {
        reported_nav = nav.state;
        status();
    }
    if (drive.state != reported_motion)
    {
        reported_motion = drive.state;
        if (drive.state == MOTION_WAIT_CONFIRM)
            text("[estimate elapsed; inspect vehicle then confirm; NOT arrived]\r\n");
    }
}

#ifndef NAV_ENTRY_HOST_TEST
/* ===== 8. 硬件启动：默认等待命令，上电不自动跑路线 ===== */
int main(void)
{
    MotionIO io =
    {
        motor_position, motor_sync, motor_stop, 0
    };
    MotionConfig config =
    {
        CALIBRATED_PPM_X, CALIBRATED_PPM_Y, 1, 0, -1, 1, LEFT_FORWARD_MM_PER_M, RIGHT_FORWARD_MM_PER_M
    };
    NavMotionPort port =
    {
        port_start, port_poll, port_stop, &drive
    };
    uint8_t i;
    delay_init(168);
    board_init();
    uart_init();
    clock_init_ms();
    arm_init();
    vision_init();station_init();hmi_init(clock_ms);
    start_button_init();
    scanner_init(CAR_SCANNER_UART_BAUD);
    heading_sensor_reset(&heading_sensor);
#if CAR_IMU_UART_ENABLED
    /* 被动接收，沿用现有USART2引脚；不调用会扫描/写指令的imu_init。 */
    {
        extern void Usart2Init(unsigned int baud);
        Usart2Init(CAR_IMU_UART_BAUD);
    }
#endif
    if (!motion_init(&drive, &io, &config) || !nav_init(&nav, 0, HEAD_Y_POS, &port))
        for (;;)
    {
    }
    motion_stop(&drive);
    for (i = 1; i <= MOTOR_COUNT; i++)
    {
        Emm_V5_En_Control(i, true, false);
        delay_ms(1);
    }
    text("== runimu full route + manual single-step diagnostics ==\r\n");
    text("== merged chassis + arm serial control; ! always emergency stop ==\r\n");
    text("[drive] version=20261022-merge2-r63; return cup preselection parallel with ground approach\r\n");
    text("Place at a known node, verify physical heading, then pose/heading/go.\r\n");
    help();
    /* 每轮只处理一个字节，使运动状态与停止请求持续获得处理。 */
    for (;;)
        firmware_step();
}

#endif
