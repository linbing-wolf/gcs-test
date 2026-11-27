/* 小车控制入口：硬件初始化、串口命令、单段标定、完整路线调度。
 * 日常调参：car_config.h；道路动作：structure.h；全程顺序：full_route.h。
 * 主循环入口：firmware_step()；全程状态推进：run_update()。
 * runimu道路直接前后/横移，工位单独对准；估时不确认实际到位或车身角度。
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
#include "start_scan.h"
#if !defined(NAV_ENTRY_HOST_TEST) && !defined(CHASSIS_OBSTACLE_TEST)
#include "debug_radio.h"
#endif
#ifdef CHASSIS_OBSTACLE_TEST
#include "route_table.h"
#include "../diagnostics/ultrasonic-chassis/obstacle_io.h"
#endif
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
#define MOTOR_FLAG_STALLED           4u /* 仅记录堵转标志；未使能/保护及其他运动故障仍阻止运行。 */
#define MOTOR_FLAG_PROTECTED         8u
/* ===== 1. 全局状态：Keil Watch 窗口可观察这些变量 ===== */
static struct {uint32_t start_ms,elapsed_ms;uint8_t running,started;} screen_clock;
static volatile uint32_t clock_ms;       /* 系统毫秒计时，TIM5 中断更新。 */
#define SCANNER_RX_SIZE 128u
typedef enum {SCANNER_OFF, SCANNER_BOOT, SCANNER_SET_ENDING, SCANNER_SET_CONTINUOUS,
              SCANNER_READY, SCANNER_FAULT} ScannerState;
typedef enum {SCANNER_NO_FAULT, SCANNER_TX_TIMEOUT, SCANNER_ACK_TIMEOUT,
              SCANNER_RX_ERROR} ScannerFailure;
static struct {
    ScannerState state;
    uint32_t baud, phase_ms, rx_bytes, results;
    uint32_t uart_errors, overflows;
    volatile uint8_t tx_offset, tx_active, tx_complete, tx_command;
    volatile uint32_t tx_completed_ms;
    uint8_t waiting_ack, ack_offset, ack_peak, binary_length, binary_good, length, discard;
    uint8_t fault_stage, fault_tx, fault_ack;
    ScannerFailure fault_reason;
    uint8_t recent[32], recent_head, recent_count;
    char result[128];
} scanner;
static volatile uint8_t scanner_rx[SCANNER_RX_SIZE], scanner_head, scanner_tail;
static volatile uint8_t scanner_rx_error, scanner_rx_overflow;
static struct {
    uint8_t candidate, stable, preparing, pressed_valid, cancelled;
    char origin;
    uint32_t changed_ms, pressed_ms;
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
static uint8_t encoder_drift_feedback_enabled=CAR_ENCODER_DRIFT_DEFAULT_ON;
static uint8_t gyro_drive_feedback_enabled=CAR_GYRO_HEADING_DEFAULT_ON;
static uint8_t lateral_hold_feedback_enabled=CAR_LATERAL_HOLD_DEFAULT_ON;
static uint8_t accel_extra_enabled;
static uint8_t station_hold_enabled=CAR_STATION_HOLD_DEFAULT_ON;
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

typedef struct {
    float x,y,ux,uy,length,end_s,radius;
    uint8_t edge,segment;
} CornerLeg;
typedef struct {
    float x,y,ux,uy,length,s,cx,cy,radius,angle,turn,yaw,dyaw,speed_mm_s;
    float blend_x,blend_y,blend_length;
} CornerSpan;
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
    RoadHeading initial_heading; /* runimu：A/C原启停位置，由操作者预先朝-X，首段直接横移。 */
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
    uint8_t station_enabled, station_phase, scan_turn_stage, vision_debug; /* 扫码窗口：1首次F前行驶中，2已到F等待任务。 */
    uint8_t station_departing, station_speed_logged; /* 工位离站预检范围及每轮日志去重。 */
    uint8_t departure_overlap; /* 1高位收回后的末尾回正；2粗加工最后抓回离物后，道路与车载收尾并行。 */
    int16_t station_return_mm; /* 工位末环到2环的车身前向距离，作为离站路径前缀，不计入道路里程。 */
    uint8_t station_shift, station_relock; /* 粗加工子动作及完成后的四轮互锁交接。 */
    uint8_t station_visual_shift; /* 仅视觉MOVE/MOVEY/MOVEXY；固定环位索引不启用视觉末端控制。 */
    uint8_t station_approach_overlap; /* 视觉/索引最后一段与下一靠近动作并行；夹爪仍等子动作完成。 */
    uint8_t transition_ready; /* 已完成道路或工位持续确认的航向/停车状态，只向下一段传递一次。 */
    BodyAxis travel_axis;     /* 连续平移轴：默认前后；仅授权段/手动l/r使用横移。 */
    uint16_t lateral_rpm;
    uint16_t test_forward_rpm; /* 单段前后测试的单轮速度上限。 */
    struct {
        uint8_t active, passed, arriving, polarity_ready, start_edge, reversing, visual_terminal;
        uint8_t start_segment, start_job, leg_count, leg_index, span_count;
        RoadHeading start_heading, end_heading;
        int16_t prefix_mm; /* 非零时首段是工位回2环；此路径起点为最后抓取环，不是地图节点。 */
        CornerLeg legs[8];
        CornerSpan spans[16];
        float first_mm, second_mm, arc_mm, total_mm, x_mm, y_mm, s_mm, speed_mm_s;
        float path_ref_mm_s;
        uint32_t path_ref_ms;
        float vx_mm_s,vy_mm_s,measured_s_mm;
        float command[3], integrated_yaw_cdeg; /* 同一比例限制四轮变化率；上次里程对应的IMU角度。 */
        int32_t heading_target;
        int16_t origin_yaw_cdeg;
        int8_t polarity[4];
        int64_t previous[4], latest[4], integrated[4];
        uint32_t previous_ms[4], latest_ms[4], integrated_ms, command_ms, projection_ms;
        int16_t yaw[48];
        uint32_t yaw_ms[48];
        uint8_t yaw_head, yaw_count;
    } corner; /* 到下一作业点的直线/圆弧串；同一时刻四轮增量配合IMU积分。 */
    uint8_t vector_move;      /* 仅MOVEXY工位子动作：前后/横移目标独立闭环，同一批四轮速度。 */
    int8_t vector_last_wheel_sign[4];
    uint16_t vector_wheel_reversals[4]; /* 本段成功提交的非零轮速命令换向次数，零速不清除上一方向。 */
    int32_t vector_target_mm[2], vector_progress_mm[2]; /* 0前后、1向左，均为有符号毫米。 */
    int32_t vector_mrpm[2];
    uint32_t vector_profile_ms[2];
    int8_t vector_profile_sign[2];
    struct {
        int64_t origin[4];
        uint32_t command_ms[4], after_ms[4];
        int8_t command_sign[4], before_speed_sign[4], speed_sign[4];
        uint8_t wait_speed, origin_ready, first_origin;
    } vector_learn; /* 仅未确认极性时使用；换向后等待实际速度过零/变向，再建立新CPOS区间。 */
    uint8_t station_align_pending; /* 工位已走到，但尚未完成右侧朝向和停稳确认。 */
    int8_t travel_sign;       /* 平移方向：+1前/左，-1后/右；行程仍为正数。 */
    uint16_t remaining_mm, action_mm;
    int16_t yaw_zero_cdeg;
    uint8_t start_reference_valid; /* 仅正式A/C发车建立；工位子动作和到站不得重取零点。 */
    uint16_t turn_rpm;
    HeadingControl heading_control;
    VelocityPhase velocity_phase;
    uint8_t velocity_query_index, velocity_waiting, velocity_sent;
    uint32_t velocity_ms, control_ms, phase_ms, zero_ms, stopped_ms, log_ms;
    uint32_t baseline_after_ms; /* 编码器基线必须采样于已确认的停稳区间内。 */
    uint32_t wheel_start_ms[4];
    int16_t wheel_rpm[4];
    int32_t settle_turn_mrpm;
    uint8_t settle_turn_active;
    int64_t encoder_start[4];
    int8_t encoder_sign[4];
    int8_t wheel_polarity[4]; /* 原始CPOS相对逻辑轮速的物理极性；本轮已确认后跨视觉动作保留。 */
    int64_t encoder_previous_delta[4];
    uint32_t encoder_sample_ms[4];
    uint8_t encoder_ready;    /* 四轮方向确认掩码；同一直行串连续通过节点时保留。 */
    struct {
        int64_t previous[4], latest[4];
        uint32_t previous_ms[4], latest_ms[4], sample_ms;
        int32_t lateral_mm, command_mrpm;
        int16_t command_rpm;
        uint8_t valid;
    } straight_hold; /* 同时刻编码器投影；相对本直行串起点的车身横向位移，不是地面绝对位置。 */
    struct {
        float forward_mm,left_mm,body_forward_mm,body_left_mm,last_yaw_cdeg;
        float cross_mm,forward_rate_mm_s; /* 正交误差及速度：l/r为前后，f/b为左右。 */
        int16_t origin_yaw_cdeg;
        uint32_t sample_ms,sequence;
        struct {uint32_t ms;int16_t yaw_cdeg;} yaw[64];
        uint8_t head,count,valid;
    } lateral_pose; /* 手动四方向共用起步坐标系里程计；沿用横移测试建立的内部成员名。 */
    int32_t progress_mm;
    int32_t encoder_offset_mm; /* 同一直行串内已通过的段长；不中断编码器基线。 */
    uint16_t continuation_mm, blocked_roads;
    RunLeg road_path[ROAD_MAX_SEGMENTS]; /* 当前道路已完成的车身动作，供反序退回。 */
    uint8_t road_path_count, return_index, obstacle_phase, obstacle_leg_active, obstacle_node, obstacle_preflight;
    int32_t forward_mrpm;      /* 平移速度斜坡，0.001RPM；仅在可提交速度批次时推进。 */
    int8_t profile_sign;      /* 平移斜坡当前方向；横移过冲先减到零再反向。 */
    uint32_t forward_profile_ms, control_dt_ms;
    int16_t forward_ref_rpm, forward_cmd_rpm, turn_ref_rpm, turn_cmd_rpm;
    int16_t left_ref_rpm, left_cmd_rpm;
    struct {
        uint32_t sequence, samples, min_ms, max_ms;
        int32_t min_cdeg, max_cdeg, min_mm, max_mm;
        uint8_t seen;
    } heading_window;
    struct {
        uint32_t sequence,samples,first_ms,last_ms;
        int16_t minimum[3],maximum[3];
        int32_t sum[3];
        uint8_t seen;
    } acceleration_window; /* 主循环处理到的新0x51帧；机体XYZ，含重力，单位mg。 */
    struct {
        float bias_x_mg,bias_y_mg,velocity_mm_s,deficit_mm,reference_mm_s;
        uint32_t sample_ms,sequence;
        int16_t roll_cdeg,pitch_cdeg;
        uint16_t mm;
        uint8_t samples,state;
    } accel_extra;
    uint8_t pending_command;
    uint8_t route_terminal;
    RoadPlan road;            /* structure.h 中取出的当前有向道路参数。 */
} FullRouteState;
static FullRouteState run;
enum { ACCEL_EXTRA_IDLE, ACCEL_EXTRA_READY, ACCEL_EXTRA_FROZEN, ACCEL_EXTRA_UNAVAILABLE };

static uint16_t run_target_mm(void)
{
    return (uint16_t)(run.action_mm+run.accel_extra.mm);
}
static FullRouteState station_parent; /* 索引子动作完成后恢复原路线现场，不推进节点/任务。 */
static struct {
    uint8_t active,releasing,waiting,acks,sent;
    int16_t target_cdeg,command_rpm;
    int32_t command_mrpm;
    uint32_t control_ms,velocity_ms,log_ms;
    HeadingControl heading;
} station_hold;
static struct {
    uint32_t last_ms,stage[6],total[6];
    uint8_t active,job,category;
} route_timing;
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

static uint32_t heading_tilt_warnings, heading_tilt_sequence;

static HeadingSample heading_snapshot(void)
{
    HeadingSample sample;
    __disable_irq();
    sample = heading_sensor.sample;
    __enable_irq();
    sample.yaw_cdeg = heading_wrap_cdeg((int32_t)sample.yaw_cdeg * CAR_IMU_YAW_SIGN);
    /* 倾斜只记录诊断；新鲜融合Yaw不能因此失效并让旧轮速滞留。 */
    if (!heading_sample_level(&sample, CAR_IMU_MAX_TILT_CDEG)) {
        if (sample.sequence != heading_tilt_sequence) {
            heading_tilt_sequence = sample.sequence;
            heading_tilt_warnings++;
        }
    }
    return sample;
}

static HeadingRate heading_rate_snapshot(void)
{
    HeadingRate rate;
    __disable_irq();
    rate = heading_sensor.rate;
    __enable_irq();
    rate.rate_cdeg_s *= CAR_IMU_YAW_SIGN;
    return rate;
}

/* 航向闭环统一入口：本拍有新鲜角速度帧时用它作D项，否则回退角度差分。 */
static HeadingDecision heading_update(HeadingControl *control, HeadingSample sample)
{
#if CAR_IMU_RATE_ENABLED
    HeadingRate rate = heading_rate_snapshot();
    uint8_t fresh = (uint8_t)(heading_rate_fresh(&rate, clock_ms, CAR_IMU_RATE_MAX_AGE_MS)
        && heading_sample_level(&sample,CAR_IMU_MAX_TILT_CDEG));
    heading_control_measured_rate(control, fresh ? rate.rate_cdeg_s : 0, fresh);
#else
    heading_control_measured_rate(control, 0, 0);
#endif
    return heading_control_update(control, sample, clock_ms);
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
#ifdef CHASSIS_OBSTACLE_TEST
static uint8_t obstacle_nodes[1u+8u*FULL_ROUTE_JOBS], obstacle_node_count;
static uint8_t obstacle_hits, obstacle_shot_valid, obstacle_shot_edge, obstacle_shot_segment;
static int obstacle_ranges[2] = {-1,-1};
static uint8_t obstacle_range_log, obstacle_entry_samples, obstacle_map_changed;
static uint8_t obstacle_inset_node = 255u;
enum { OBSTACLE_ENTRY_X=1u, OBSTACLE_ENTRY_Y=2u, OBSTACLE_ENTRY_BOTH=3u };
static uint8_t obstacle_inset_axes;
static int obstacle_plan_remaining(uint8_t from);
static int obstacle_close_approach(const RoadPlan *road)
{
    /* 方向策略：接近固定设施延后判障；离开B/F不自动豁免。 */
    return road->to==NODE_B || road->to==NODE_F;
}
static struct {
    uint8_t hits,road;
    int16_t x,y;
    uint32_t ms;
} obstacle_observations[2];
static uint16_t obstacle_observed_roads;
static struct {
    uint8_t valid,edge,segment,sensor;
    float x,y,dx,dy;
    uint32_t ms;
} obstacle_ray;
/* 保留封路当时的数据，退回、绕行及人工停止后仍可查询；复位后清空。 */
static struct {
    uint8_t valid, from, to, sensor, segment, heading;
    int mm;
    int32_t progress_mm, remaining_mm;
    uint32_t time_ms;
} obstacle_last;
#define RUN_NODES (obstacle_nodes)
#define RUN_NODE_COUNT (obstacle_node_count)
#define RUN_JOB_NODE(job) ((job)==FULL_ROUTE_JOBS-1u ? RUN_NODES[RUN_NODE_COUNT-1u] : full_route_jobs[run.plan][job])
#else
#define RUN_NODES (full_route_nodes[run.plan])
#define RUN_NODE_COUNT FULL_ROUTE_NODES
#define RUN_JOB_NODE(job) (full_route_jobs[run.plan][job])
#endif
static void run_begin_alignment(void);
static int run_velocity_stopped(void);
static uint16_t run_wheel_limit_rpm(void);
static void arm_lock_begin(void);
static void run_resume_after_station(void);
static void run_station_update(void);
static int station_hold_release(void);
static void station_hold_update(uint8_t ready);
static void run_vector_polarity_command(uint8_t wheel,int8_t sign);
static int run_corner_rotating_span(void);
/* ===== 2. CAN 接收队列：中断只收报文，全程主循环负责解析 ===== */
/* CAN IRQ只复制短报文；主循环处理应答，不在IRQ内调用运动函数。 */
void route_test_can_rx(uint32_t id, uint8_t ide, uint8_t rtr, uint8_t length, const uint8_t *data)
{
    uint8_t i, next;
#ifndef CHASSIS_OBSTACLE_TEST
    arm_can_rx(id, ide, rtr, length, data, clock_ms);
#endif
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
#elif defined(CHASSIS_OBSTACLE_TEST)
    obstacle_io_send(b);
#else
    debug_radio_send(b);
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
    uint8_t level;
#ifdef NAV_ENTRY_HOST_TEST
    level=entry_host_start_button_level();
#else
    level=GPIO_ReadInputDataBit(CAR_START_BUTTON_GPIO, CAR_START_BUTTON_PIN);
#endif
    return level == 0; /* PC9按钮另一端接GND，低电平为按下。 */
}

static void start_button_init(void)
{
#ifndef NAV_ENTRY_HOST_TEST
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(CAR_START_BUTTON_GPIO_CLOCK, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = CAR_START_BUTTON_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(CAR_START_BUTTON_GPIO, &gpio);
#endif
    /* 上电时已按住不算新按下；松开再按才请求启动，避免上电自动发车。 */
    start_button.candidate = start_button.stable = start_button_pressed();
    start_button.preparing = 0;
    start_button.pressed_valid = 0;
    start_button.cancelled = 0;
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
        if (pressed) {
            start_button.pressed_valid=!start_button.cancelled;
            start_button.pressed_ms=start_button.changed_ms;
            text("[button] START pressed\r\n");
        } else if(start_button.pressed_valid) {
            start_button.pressed_valid=0;
            /* 按两端电平开始稳定的时刻计时，松手消抖不会把临界短按变成长按。 */
            return (uint32_t)(start_button.changed_ms-start_button.pressed_ms)
                >= CAR_START_BUTTON_LONG_PRESS_MS ? 'A' : 'C';
        }
    }
    if(!pressed && (uint32_t)(clock_ms-start_button.changed_ms)>=CAR_START_BUTTON_DEBOUNCE_MS)
        start_button.cancelled=0;
    return 0;
}

static void start_button_cancel(void)
{
    start_button.preparing=0;
    start_button.pressed_valid=0;
    /* 连按下消抖尚未完成的手势也取消，必须等本次松开后才能重新选择。 */
    start_button.cancelled=start_button.stable || start_button_pressed();
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

/* USART1接收中断仅入队；字符串处理、诊断输出留在主循环。 */
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
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    USART_ITConfig(USART1, USART_IT_TC, DISABLE);
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

/* 每次TXE中断最多送一个字节；最后一字节的TC中断记录真实发送完成时间。 */
void route_test_scanner_tx_irq(void)
{
    if(!scanner.tx_active)return;
    if(scanner.tx_offset<sizeof(scanner_commands[0])) {
        if(scanner_tx_ready()) {
            scanner_send_byte(scanner_commands[scanner.tx_command][scanner.tx_offset++]);
#ifndef NAV_ENTRY_HOST_TEST
            if(scanner.tx_offset==sizeof(scanner_commands[0])) {
                USART_ITConfig(USART1,USART_IT_TXE,DISABLE);
                USART_ITConfig(USART1,USART_IT_TC,ENABLE);
            }
#endif
        }
    } else {
#ifdef NAV_ENTRY_HOST_TEST
        if(!entry_host_scanner_tx_complete())return;
#else
        if(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET)return;
        USART_ITConfig(USART1,USART_IT_TC,DISABLE);
#endif
        scanner.tx_completed_ms=clock_ms;
        scanner.tx_active=0;scanner.tx_complete=1;
    }
}

static void scanner_begin_command(uint8_t command_index)
{
    scanner.state=command_index ? SCANNER_SET_CONTINUOUS : SCANNER_SET_ENDING;
    scanner.phase_ms=clock_ms;scanner.waiting_ack=0;scanner.ack_peak=0;
    scanner.binary_length=scanner.ack_offset=0;
    scanner.tx_command=command_index;scanner.tx_offset=0;scanner.tx_complete=0;scanner.tx_active=1;
#ifndef NAV_ENTRY_HOST_TEST
    USART_ClearFlag(USART1,USART_FLAG_TC);
    USART_ITConfig(USART1,USART_IT_TXE,ENABLE);
#else
    route_test_scanner_tx_irq(); /* 主机替身仅模拟一次TXE事件，后续可由专项独立驱动。 */
#endif
}

static void scanner_recent_status(void)
{
    static const char hex[]="0123456789ABCDEF";
    unsigned i;
    text("[scanner rx hex] oldest_to_newest=");
    for(i=0;i<scanner.recent_count;i++) {
        uint8_t byte=scanner.recent[(scanner.recent_head+sizeof(scanner.recent)-scanner.recent_count+i)%sizeof(scanner.recent)];
        if(i)send_byte(' ');
        send_byte((uint8_t)hex[byte>>4]);send_byte((uint8_t)hex[byte&15u]);
    }
    text("\r\n");
}

static void scanner_configuration_fault(ScannerFailure reason)
{
    scanner.fault_stage=(uint8_t)scanner.state;
    scanner.fault_tx=scanner.tx_offset;scanner.fault_ack=scanner.ack_peak;scanner.fault_reason=reason;
    scanner.tx_active=0;scanner.waiting_ack=0;scanner.state=SCANNER_FAULT;
    scanner.binary_length=scanner.ack_offset=0;
#ifndef NAV_ENTRY_HOST_TEST
    USART_ITConfig(USART1,USART_IT_TXE,DISABLE);
    USART_ITConfig(USART1,USART_IT_TC,DISABLE);
#endif
    text("[scanner] configuration fault stage=");
    text(scanner.fault_stage==SCANNER_SET_ENDING ? "CRLF" : scanner.fault_stage==SCANNER_SET_CONTINUOUS ? "CONTINUOUS" : "BOOT");
    text(" reason=");text(reason==SCANNER_TX_TIMEOUT ? "TX_TIMEOUT" : reason==SCANNER_ACK_TIMEOUT ? "ACK_TIMEOUT" : "RX_ERROR");
    text(" tx=");number(scanner.fault_tx);text("/9 ack=");number(scanner.fault_ack);
    text("/7; text receiver remains active; qr for status\r\n");
    scanner_recent_status();
}

static void scanner_status(void)
{
    static const char *names[] = {"OFF","BOOT","SET_ENDING","SET_CONTINUOUS","READY","FAULT"};
    text("[scanner] state="); text(names[scanner.state]);
    text(" baud="); number(scanner.baud);
    text(" rx_bytes="); number(scanner.rx_bytes);
    text(" results="); number(scanner.results);
    text(" uart_errors="); number(scanner.uart_errors);
    text(" overflows="); number(scanner.overflows);
    text(" phase=");text(scanner.tx_active ? "TX" : scanner.waiting_ack ? "WAIT_ACK" : "IDLE");
    text(" tx=");number(scanner.tx_offset);text("/9 ack=");number(scanner.ack_peak);text("/7");
    if(scanner.fault_reason) {
        text(" fault_stage=");text(scanner.fault_stage==SCANNER_SET_ENDING ? "CRLF" : scanner.fault_stage==SCANNER_SET_CONTINUOUS ? "CONTINUOUS" : "BOOT");
        text(" reason=");text(scanner.fault_reason==SCANNER_TX_TIMEOUT ? "TX_TIMEOUT" : scanner.fault_reason==SCANNER_ACK_TIMEOUT ? "ACK_TIMEOUT" : "RX_ERROR");
        text(" fault_tx=");number(scanner.fault_tx);text(" fault_ack=");number(scanner.fault_ack);
    }
    text("; text_rx=enabled\r\n");scanner_recent_status();
}

static void scanner_text_byte(uint8_t byte)
{
    if(byte=='\r' || byte=='\n') {
        /* 实车已捕获CR单独结束；在CR提交，后续LF只清空边界，不重复提交。 */
        if(byte=='\r' && !scanner.discard && scanner.length) {
            scanner.result[scanner.length]=0;
            if(scanner.length) {
                text("[scan] ");text(scanner.result);text("\r\n");scanner.results++;
                /* 配置状态与任务接收独立；首次横移、B后直行及首次F待码均可接收。 */
                if(run.station_enabled && run.job<=1u && run.scan_turn_stage
                    && run.edge<=3u && (run_active || run.state==RUN_STATION)
                    && !stop_pending && !station_task_ready()) {
                    char task_line[sizeof(scanner.result)+6u];
                    snprintf(task_line,sizeof(task_line),"task %s",scanner.result);
                    (void)station_command(task_line);
                    if(run.job==1u && station_task_ready())run.scan_turn_stage=0;
                }
            }
        }
        scanner.length=scanner.discard=0;
    } else if(!scanner.discard) {
        if(scanner.length>=sizeof(scanner.result)-2u || byte<0x20u) { /* 保持原126字节正文上限。 */
            scanner.discard=1;
            text("[scanner] invalid/overlong result discarded\r\n");
        } else scanner.result[scanner.length++]=(char)byte;
    }
}

static void scanner_wait_ack_after_tc(void)
{
    if(scanner.tx_complete && !scanner.waiting_ack
        && (scanner.state==SCANNER_SET_ENDING || scanner.state==SCANNER_SET_CONTINUOUS)) {
        scanner.waiting_ack=1;scanner.phase_ms=scanner.tx_completed_ms;
    }
}

static void scanner_update(void)
{
    uint8_t budget = 32;
    if (scanner.state == SCANNER_OFF) return;
#ifdef NAV_ENTRY_HOST_TEST
    route_test_scanner_tx_irq();
#endif
    scanner_wait_ack_after_tc();
    if (scanner_rx_error || scanner_rx_overflow) {
        __disable_irq();
        scanner.uart_errors += scanner_rx_error != 0;
        scanner.overflows += scanner_rx_overflow != 0;
        scanner_rx_error = scanner_rx_overflow = 0; scanner_tail = scanner_head;
        __enable_irq();
        scanner.length = scanner.ack_offset = 0; scanner.discard = 1;
        if (scanner.state != SCANNER_READY && scanner.state != SCANNER_FAULT) scanner_configuration_fault(SCANNER_RX_ERROR);
        text("[scanner] RX error/overflow; partial data discarded\r\n");
    }
    while (budget-- && scanner_tail != scanner_head) {
        uint8_t byte = scanner_rx[scanner_tail];
        scanner_wait_ack_after_tc(); /* TC可能发生在主循环处理/打印上一条文本期间。 */
        scanner_tail = (uint8_t)((scanner_tail + 1u) % SCANNER_RX_SIZE);
        scanner.rx_bytes++;
        scanner.recent[scanner.recent_head]=byte;
        scanner.recent_head=(uint8_t)((scanner.recent_head+1u)%sizeof(scanner.recent));
        if(scanner.recent_count<sizeof(scanner.recent))scanner.recent_count++;
        /* 0x02开头的7字节配置应答整体分流，包括错误应答；不把其校验尾字节拼进文本。 */
        if(scanner.binary_length || byte==scanner_ack[0]) {
            if(!scanner.binary_length) {
                scanner.binary_good=1;scanner.ack_offset=0;
                if(scanner.length) {scanner.length=0;scanner.discard=1;}
            }
            if(byte!=scanner_ack[scanner.binary_length])scanner.binary_good=0;
            if(scanner.binary_good) {
                scanner.ack_offset++;
                if(scanner.waiting_ack && scanner.ack_offset>scanner.ack_peak)scanner.ack_peak=scanner.ack_offset;
            }
            if(++scanner.binary_length==sizeof(scanner_ack)) {
                scanner.binary_length=scanner.ack_offset=0;
                if(scanner.binary_good && scanner.waiting_ack) {
                    scanner.waiting_ack=0;
                    if(scanner.state==SCANNER_SET_ENDING) {
                        scanner_begin_command(1u);text("[scanner] CRLF acknowledged\r\n");
                    } else if(scanner.state==SCANNER_SET_CONTINUOUS) {
                        scanner.state=SCANNER_READY;text("[scanner] READY continuous acknowledged\r\n");
                    }
                }
            }
        } else scanner_text_byte(byte);
    }
    if (scanner.state == SCANNER_BOOT && (uint32_t)(clock_ms - scanner.phase_ms) >= CAR_SCANNER_BOOT_MS) {
        scanner_begin_command(0u);
    }
    if (scanner.state == SCANNER_SET_ENDING || scanner.state == SCANNER_SET_CONTINUOUS) {
        scanner_wait_ack_after_tc();
        if((uint32_t)(clock_ms-scanner.phase_ms)>=(scanner.waiting_ack ? CAR_SCANNER_ACK_MS : CAR_SCANNER_TX_MS))
            scanner_configuration_fault(scanner.waiting_ack ? SCANNER_ACK_TIMEOUT : SCANNER_TX_TIMEOUT);
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
        return;
    }
    rx_buffer[rx_head] = value;
    rx_head = next;
}

#if defined(CHASSIS_OBSTACLE_TEST) && !defined(NAV_ENTRY_HOST_TEST)
void obstacle_io_receive(uint8_t byte) {queue_byte(byte);}
#endif
#if !defined(NAV_ENTRY_HOST_TEST) && !defined(CHASSIS_OBSTACLE_TEST)
static void uart_receive_error(void) {rx_overflow=1;}
static void uart_init(void)
{
    debug_radio_init(queue_byte,uart_receive_error);
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
        uint32_t target_mm=run.remaining_mm ? run.remaining_mm+run.accel_extra.mm : run_target_mm();
        text(run.travel_axis==BODY_FORWARD ? "[drive closed-loop] " : "[shift closed-loop] ");
        text(run.state == RUN_DONE ? "DONE_BY_IMU_ENCODER" : names[run.state]);
        if(run.travel_axis==BODY_FORWARD)text(run.travel_sign>0 ? " travel=FORWARD" : " travel=BACKWARD");
        else text(run.travel_sign>0 ? " travel=LEFT" : " travel=RIGHT");
        text(" distance_mm="); signed_number(run.progress_mm);
        text(" target_mm="); number(target_mm);
        text(" error_cdeg="); signed_number(run.heading_control.error_cdeg);
        text(" rpm_limit="); number(run.test_forward_rpm ? run.test_forward_rpm : run.lateral_rpm);
        if(run.test_forward_rpm && CAR_STRAIGHT_HOLD_ENABLED) {
            text(" trim_left=");
            signed_number((int32_t)lroundf(car_straight_left_trim((float)target_mm*run.travel_sign)));
            text(" mm");
        }
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
    send_byte((uint8_t)('A' + RUN_NODES[run.edge]));
    text(" ack=");
    number(run.acks);
    text(" ready=");
    number(run.ready);
    text(" pending=");
    number(run.pending);
    text(" heading_est="); number((uint32_t)run.heading);
    text(run.corner.active ? " action=corner" : run.turning ? " action=turn" : run.travel_axis == BODY_LEFT ? " action=lateral" : " action=forward");
    text(" physical_pose=unconfirmed");
    if (run.imu_mode) {
        text(" heading_control=IMU control=continuous_velocity");
        text(" target_yaw_cdeg="); signed_number(heading_wrap_cdeg((int32_t)run.heading_control.last_yaw_cdeg + run.heading_control.error_cdeg));
        text(" yaw_cdeg="); signed_number(run.heading_control.last_yaw_cdeg);
        text(" error_cdeg="); signed_number(run.heading_control.error_cdeg);
        text(run.corner.active ? " path_s_mm=" : " distance_mm=");signed_number(run.progress_mm);
        if(run.corner.active){text(" path_target_mm=");number((uint32_t)lroundf(run.corner.total_mm));}
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

/* 原始九轴与解算量仅观测；保留各自时间戳，不把不同帧拼成同一时刻的状态。 */
static void imu_nine_axis_log(void)
{
    HeadingRate gyro;HeadingMagnetic mag;HeadingTemperature temp;HeadingQuaternion quat;
    unsigned i;
    __disable_irq();gyro=heading_sensor.rate;mag=heading_sensor.magnetic;
    temp=heading_sensor.temperature;quat=heading_sensor.quaternion;__enable_irq();
    text(" gyro_fresh=");number(heading_rate_fresh(&gyro,clock_ms,CAR_IMU_RATE_MAX_AGE_MS));
    text(" gyro_age_ms=");if(gyro.valid)number(clock_ms-gyro.received_ms);else text("-1");
    text(" gyro_axes=");number(gyro.valid ? gyro.axes_valid : 0);
    text(" gyro_sensor_cdeg_s=");
    if(gyro.valid) {
        if(gyro.axes_valid&1u)signed_number(gyro.x_cdeg_s);else text("NA");text(",");
        if(gyro.axes_valid&2u)signed_number(gyro.y_cdeg_s);else text("NA");text(",");signed_number(gyro.rate_cdeg_s);
    } else text("NA");
    text(" gyro_seq=");number(gyro.sequence);
    text(" mag_fresh=");number(mag.valid && (uint32_t)(clock_ms-mag.received_ms)<=CAR_IMU_MAX_AGE_MS);
    text(" mag_age_ms=");if(mag.valid)number(clock_ms-mag.received_ms);else text("-1");
    text(" mag_lsb=");
    if(mag.valid) {
        signed_number(mag.x_lsb);text(",");signed_number(mag.y_lsb);text(",");signed_number(mag.z_lsb);
        text(" mag_norm_lsb=");number((uint32_t)lroundf(sqrtf((float)mag.x_lsb*mag.x_lsb
            +(float)mag.y_lsb*mag.y_lsb+(float)mag.z_lsb*mag.z_lsb)));
    } else text("NA");
    text(" mag_seq=");number(mag.sequence);
    text(" temp_fresh=");number(temp.valid && (uint32_t)(clock_ms-temp.received_ms)<=CAR_IMU_MAX_AGE_MS);
    text(" temp_age_ms=");if(temp.valid)number(clock_ms-temp.received_ms);else text("-1");
    text(" temp_cdeg=");if(temp.valid)signed_number(temp.cdeg);else text("NA");
    text(" quat_fresh=");number(quat.valid && (uint32_t)(clock_ms-quat.received_ms)<=CAR_IMU_MAX_AGE_MS);
    text(" quat_age_ms=");if(quat.valid)number(clock_ms-quat.received_ms);else text("-1");
    text(" quat_q15=");
    if(quat.valid)for(i=0;i<4;i++) {if(i)text(",");signed_number(quat.q15[i]);}
    else text("NA");
    text(" quat_seq=");number(quat.sequence);
}

static void imu_status(void)
{
    HeadingSample sample = heading_snapshot();
    HeadingRate rate = heading_rate_snapshot();
    text("[imu] uart_enabled="); number(CAR_IMU_UART_ENABLED);
    text(" fresh="); number(heading_sample_fresh(&sample, clock_ms, CAR_IMU_MAX_AGE_MS));
    text(" yaw_cdeg="); signed_number(sample.yaw_cdeg);
    text(" age_ms="); number(clock_ms - sample.received_ms);
    text(" samples="); number(sample.sequence);
    text(" roll_cdeg="); signed_number(sample.roll_cdeg);
    text(" pitch_cdeg="); signed_number(sample.pitch_cdeg);
    text(" max_tilt_cdeg="); number(CAR_IMU_MAX_TILT_CDEG);
    text(" tilt_warn="); number(heading_tilt_warnings);
    {HeadingAcceleration a;
     __disable_irq();a=heading_sensor.acceleration;__enable_irq();
     text(" accel_fresh=");number(a.valid && (uint32_t)(clock_ms-a.received_ms)<=CAR_IMU_MAX_AGE_MS);
     text(" accel_body_mg=");signed_number(a.x_mg);text(",");signed_number(a.y_mg);text(",");signed_number(a.z_mg);}
    text(" rate_cdeg_s="); signed_number(rate.rate_cdeg_s);
    text(" rate_fresh="); number(heading_rate_fresh(&rate, clock_ms, CAR_IMU_RATE_MAX_AGE_MS));
    text(" rate_max_age_ms=");number(CAR_IMU_RATE_MAX_AGE_MS);
    text(" rate_samples="); number(rate.sequence);
    text(" checksum_errors="); number(heading_sensor.checksum_errors);
    imu_nine_axis_log();
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

/* 自动异常只告警，当前指令/任务/姿态账本不由故障分支取消。 */
static void run_fault(const char *reason)
{
    run_warning(reason);
}

#ifdef CHASSIS_OBSTACLE_TEST
static int obstacle_start_node(uint8_t node)
{
    return node==NODE_A || node==NODE_C;
}

static int obstacle_arrival_inset(const RoadPlan *road,uint8_t edge)
{
    return obstacle_start_node(road->to)
        && !(run.job==FULL_ROUTE_JOBS-1u && road->to==RUN_JOB_NODE(FULL_ROUTE_JOBS-1u)
             && edge+2u==RUN_NODE_COUNT);
}

static int16_t obstacle_entry_y(uint8_t node)
{
    return node==NODE_A ? ROAD_A_TOP_LINK_MM : -ROAD_C_BOTTOM_LINK_MM;
}

/* 沿指定地图轴调整连接点，不改变其他轴或原始道路表。 */
static void obstacle_shift_endpoint(RoadPlan *road,BodyAxis axis,int16_t shift)
{
    uint8_t i,j;
    for(i=0;i<road->count;i++)if(road->segments[i].axis==axis
        && ((shift<0 && road->segments[i].mm>=-shift) || (shift>0 && road->segments[i].mm<=-shift))) {
        road->segments[i].mm+=shift;road->length=(uint16_t)(road->length-abs(shift));
        if(!road->segments[i].mm) {
            for(j=(uint8_t)(i+1u);j<road->count;j++)road->segments[j-1u]=road->segments[j];
            road->count--;
        }
        return;
    }
}

static void obstacle_adjust_inset(RoadPlan *road,uint8_t node,uint8_t axes,int sign)
{
    if(axes&OBSTACLE_ENTRY_X)obstacle_shift_endpoint(road,BODY_LEFT,(int16_t)(sign*OBSTACLE_START_INSET_MM));
    if(axes&OBSTACLE_ENTRY_Y)obstacle_shift_endpoint(road,BODY_FORWARD,(int16_t)(sign*obstacle_entry_y(node)));
}

/* G/I仅作道路衔接，直接使用作业区行驶线交点，消除走出后又移回的短段。 */
static void obstacle_adjust_corner(RoadPlan *road)
{
    if(road->from==NODE_I)obstacle_shift_endpoint(road,BODY_LEFT,CAR_MAP_STORAGE_INSET_MM);
    if(road->to==NODE_I)obstacle_shift_endpoint(road,BODY_LEFT,-CAR_MAP_STORAGE_INSET_MM);
    if(road->from==NODE_G) {
        obstacle_shift_endpoint(road,BODY_LEFT,CAR_MAP_STORAGE_INSET_MM);
        obstacle_shift_endpoint(road,BODY_FORWARD,-CAR_MAP_RAW_EXTRA_INSET_MM);
    }
    if(road->to==NODE_G) {
        obstacle_shift_endpoint(road,BODY_LEFT,-CAR_MAP_STORAGE_INSET_MM);
        obstacle_shift_endpoint(road,BODY_FORWARD,CAR_MAP_RAW_EXTRA_INSET_MM);
    }
}

static int obstacle_map_point(uint8_t node,RoadPoint *point)
{
    if(!road_point(node,point))return 0;
    if(node==NODE_I)point->x+=CAR_MAP_STORAGE_INSET_MM;
    if(node==NODE_G) {
        point->x+=CAR_MAP_STORAGE_INSET_MM;
        point->y+=CAR_MAP_RAW_EXTRA_INSET_MM;
    }
    return 1;
}

static unsigned obstacle_direction_bit(MotionParameters p)
{
    return 1u<<(p.axis==BODY_FORWARD ? (p.mm>0 ? HEAD_Y_POS : HEAD_Y_NEG)
                                      : (p.mm>0 ? HEAD_X_NEG : HEAD_X_POS));
}

static unsigned obstacle_exit_coverage(const RoadPlan *road,uint8_t segment,RoadHeading heading,
                                       uint16_t completed,uint16_t blocked)
{
    unsigned mask=obstacle_direction_bit(road->segments[segment]);
    uint8_t i;
    /* 多段道路的后续段不在原节点，不拿原节点的另一出路指导转头。 */
    if(segment)return mask;
    for(i=0;i<ROAD_DIRECTION_COUNT;i++) {
        RoadPlan exit=road_motion_table[i];
        MotionParameters body;
        if(exit.from!=road->from || ((completed|blocked)&(1u<<exit.road)))continue;
        if(obstacle_start_node(exit.from))obstacle_adjust_inset(&exit,exit.from,OBSTACLE_ENTRY_BOTH,-1);
        if(obstacle_start_node(exit.to))obstacle_adjust_inset(&exit,exit.to,OBSTACLE_ENTRY_BOTH,1);
        obstacle_adjust_corner(&exit);
        if(road_motion_parameters(exit.segments[0],heading,&body) && body.mm>0)
            mask|=obstacle_direction_bit(exit.segments[0]);
    }
    return mask;
}

static unsigned obstacle_direction_count(unsigned mask)
{
    unsigned count=0;
    while(mask){count+=mask&1u;mask>>=1;}
    return count;
}

static RoadHeading obstacle_branch_heading(const RoadPlan *road,uint8_t segment,RoadHeading heading,
                                           uint16_t completed,uint16_t blocked)
{
    MotionParameters body;
    RoadHeading best=heading;
    unsigned candidate,best_count=0,best_turn=5;
    (void)road_motion_parameters(road->segments[segment],heading,&body);
    if(body.mm>0 || (completed&(1u<<road->road)))return heading;
    /* 必须转头时，先覆盖更多未知出路，再选较小转角；相反出路无法同时覆盖。 */
    for(candidate=0;candidate<4u;candidate++) {
        unsigned count,turn;
        (void)road_motion_parameters(road->segments[segment],(RoadHeading)candidate,&body);
        if(body.mm<0)continue;
        count=obstacle_direction_count(obstacle_exit_coverage(road,segment,(RoadHeading)candidate,completed,blocked));
        turn=(candidate+4u-(unsigned)heading)%4u;if(turn>2u)turn=4u-turn;
        if(count>best_count || (count==best_count && turn<best_turn)) {
            best=(RoadHeading)candidate;best_count=count;best_turn=turn;
        }
    }
    return best;
}

static void obstacle_record_arrival(const RoadPlan *road,uint8_t edge)
{
    obstacle_inset_axes=obstacle_arrival_inset(road,edge) ? OBSTACLE_ENTRY_BOTH : 0u;
    obstacle_inset_node=obstacle_inset_axes ? road->to : 255u;
}

static uint16_t obstacle_transit_length(const RoadPlan *road)
{
    uint16_t length=road->length;
    /* 首次出区及最终入区的连接距离对同一对起终点的候选路径是共同常量。 */
    if(obstacle_start_node(road->from))length=(uint16_t)(length-OBSTACLE_START_INSET_MM-abs(obstacle_entry_y(road->from)));
    if(obstacle_start_node(road->to))length=(uint16_t)(length-OBSTACLE_START_INSET_MM-abs(obstacle_entry_y(road->to)));
    if(road->from==NODE_I || road->to==NODE_I)length=(uint16_t)(length-CAR_MAP_STORAGE_INSET_MM);
    if(road->from==NODE_G || road->to==NODE_G)
        length=(uint16_t)(length-CAR_MAP_STORAGE_INSET_MM-CAR_MAP_RAW_EXTRA_INSET_MM);
    return length;
}
#endif

/* 首段60mm在A/C->B内完成，B->E沿原990mm；连续模式把偏移平滑并入1030mm。
 * 连续C回程在B->C前中段同时完成原60mm侧移；E->B保持原990mm。 */
static int run_plan_road(uint8_t edge,RoadPlan *road)
{
    const uint8_t *nodes=RUN_NODES;
    if(!road_plan(nodes[edge],nodes[edge+1u],road))return 0;
#ifdef CHASSIS_OBSTACLE_TEST
    /* 途中A/C统一用连接点；edge>0的预看也必须遵守，不能依赖尚未更新的到达状态。 */
    if(obstacle_start_node(road->from)) {
        uint8_t axes=edge>0u ? OBSTACLE_ENTRY_BOTH : (road->from==obstacle_inset_node ? obstacle_inset_axes : 0u);
        obstacle_adjust_inset(road,road->from,axes,-1);
    }
    if(obstacle_arrival_inset(road,edge))obstacle_adjust_inset(road,road->to,OBSTACLE_ENTRY_BOTH,1);
    obstacle_adjust_corner(road);
#endif
#ifndef CHASSIS_OBSTACLE_TEST
    if(!edge && CAR_RUN_CORNER_ENABLED && CAR_RUN_OMNI_ENABLED) {
        road->segments[0]=road->segments[1];
        road->count=1;road->length=(uint16_t)abs(road->segments[0].mm);
    } else if(CAR_RUN_CORNER_ENABLED && CAR_RUN_OMNI_ENABLED && run.plan==1u && edge==RUN_NODE_COUNT-2u) {
        road->count=1;road->length=(uint16_t)abs(road->segments[0].mm);
    }
#endif
    return 1;
}

/* 统一计算实际车身动作；预看下一段不能改动当前道路和完成记录。 */
static int run_resolve_leg(const RoadPlan *road, uint8_t segment, uint8_t edge,
                           RoadHeading heading, uint16_t completed, RunLeg *leg)
{
    MotionParameters p, body;
#if !CAR_RUN_OMNI_ENABLED && !defined(CHASSIS_OBSTACLE_TEST)
    uint8_t done;
#endif
    if (segment >= road->count || road->count > ROAD_MAX_SEGMENTS) return 0;
    p = road->segments[segment];
    if ((unsigned)p.axis > BODY_LEFT || !p.mm) return 0;
    leg->mm = (uint16_t)(p.mm < 0 ? -p.mm : p.mm);
#ifdef CHASSIS_OBSTACLE_TEST
    (void)edge;
    if (!road_motion_parameters(p,heading,&body)) return 0;
    /* 已能前/左行驶则不额外转头；盲区新路的转头兼顾同节点另一未知出路。 */
    if (body.mm<0 && !(completed & (uint16_t)(1u<<road->road))) {
        heading=obstacle_branch_heading(road,segment,heading,completed,run.blocked_roads);
        if (!road_motion_parameters(p,heading,&body)) return 0;
    }
    leg->axis=body.axis;leg->sign=body.mm>0 ? 1 : -1;leg->heading=heading;
    return 1;
#else
    /* A/C车头已由操作者摆好；首路60mm和1030mm都保持初始航向，含旧道路策略变体。 */
    if(!edge && road->to==NODE_B && (road->from==NODE_A || road->from==NODE_C)) {
        if(!road_motion_parameters(p,heading,&body))return 0;
        leg->axis=body.axis;leg->sign=body.mm>0 ? 1 : -1;leg->heading=heading;
        return 1;
    }
#if CAR_RUN_OMNI_ENABLED
    /* 无障碍时首次/已走道路同等处理；接口保留旧策略参数供兼容变体使用。 */
    (void)edge;(void)completed;
    if(!road_motion_parameters(p,heading,&body))return 0;
    leg->axis=body.axis;leg->sign=body.mm>0 ? 1 : -1;leg->heading=heading;
    return 1;
#else
    /* 第二次暂存作业结束后，H->E->B->A/C保持离站航向，只换算车身平移方向。 */
    if (CAR_RUN_RETURN_LATERAL_ENABLED && run.job == FULL_ROUTE_JOBS - 1u
        && edge >= RUN_NODE_COUNT - 4u) {
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
#endif
#endif
}

static int run_next_straight(RoadPlan *road, uint8_t *edge, uint8_t *segment,
                              uint16_t *completed, RunLeg current, RunLeg *next)
{
    if (++*segment >= road->count) {
        uint8_t node = road->to;
#ifdef CHASSIS_OBSTACLE_TEST
        if(obstacle_map_changed)return 0;
#endif
        if (node == RUN_JOB_NODE(run.job) || *edge + 2u >= RUN_NODE_COUNT) return 0;
        *completed |= (uint16_t)(1u << road->road);
        (*edge)++;*segment = 0;
        if (!run_plan_road(*edge,road)) return 0;
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
    for (count = 0;count < RUN_NODE_COUNT * ROAD_MAX_SEGMENTS;count++) {
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

/* 无障碍模式的所有道路直接前后/左右平移；工位朝向仍单独对准。 */
static int run_segment(void)
{
    RunLeg leg;
    if (!run_resolve_leg(&run.road,run.segment,run.edge,run.heading,run.completed_roads,&leg)) return 0;
#ifdef CHASSIS_OBSTACLE_TEST
    if(leg.heading!=run.heading) {
        text("[obstacle heading] node=");send_byte((uint8_t)('A'+run.road.from));
        text(" target_heading_dec=");number(leg.heading);
        text(" exit_directions=");number(obstacle_direction_count(obstacle_exit_coverage(&run.road,run.segment,
            leg.heading,run.completed_roads,run.blocked_roads)));text("\r\n");
    }
#endif
    run.travel_axis = leg.axis;run.travel_sign = leg.sign;run.target_heading = leg.heading;
    run.remaining_mm = leg.mm;run.continuation_mm = run_straight_remaining(leg);
    if(run.station_return_mm) {
        run.travel_axis=BODY_FORWARD;run.travel_sign=run.station_return_mm>0 ? 1 : -1;
        run.remaining_mm=(uint16_t)abs(run.station_return_mm);run.continuation_mm=0;
    }
    run.lateral_rpm=run.travel_axis==BODY_LEFT
        ? car_lateral_limit_rpm((uint32_t)run.remaining_mm+run.continuation_mm) : 0;
    run_segment_log(leg.mm);
    run_begin_alignment();
    return 1;
}

/* 底盘已满足原完成判据，维持正在执行的机构动作并交还四轮轮询权。 */
static void run_station_continue_arm(void)
{
    uint8_t i;
    nav_stop(&nav); /* 结束drive状态，供下一环位正常启动。 */
    run_clear_rx();memset(&arm_lock,0,sizeof(arm_lock));arm_lock.state=2;
    for(i=0;i<MOTOR_COUNT;i++)arm_lock.seen[i]=3u;
    arm_lock.poll_ms=clock_ms-ARM_POLL_MS;
}

static void run_done(void)
{
    run_active = 0;
    if (run.station_shift) {
        uint8_t approach_overlap=run.station_approach_overlap;
        int index_overlap=station_index_overlap_active();
        memcpy(station_parent.wheel_polarity,run.wheel_polarity,sizeof(run.wheel_polarity));
#if CAR_ARM_TRANSLATION_OVERLAP_ENABLED
        /* 子动作已真实完成停车，保留其最新航向/停车证明供后续同向衔接。 */
        station_parent.heading_control=run.heading_control;
        station_parent.zero_ms=run.zero_ms;station_parent.stopped_ms=run.stopped_ms;
#endif
        run = station_parent;
        run.state = RUN_STATION;
        if(approach_overlap || index_overlap) {
            run_station_continue_arm();
            text(index_overlap ? "[station pipeline] chassis index complete; onboard arm work continues\r\n"
                               : "[station approach overlap] chassis correction complete; arm approach continues\r\n");
            return;
        }
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
    if (!run_plan_road(run.edge, &run.road))
        return 0;
#ifdef CHASSIS_OBSTACLE_TEST
    if (run.blocked_roads & (uint16_t)(1u<<run.road.road)) return 0;
#endif
    run.segment = 0;
    return run_segment();
}

/* 互斥的墙钟时间归类；并行动作只计一次，道路时间包含其预检/转向/反馈等待。 */
static void run_timing_line(const char *label,const uint32_t *values)
{
    static const char *names[6]={" road_ms="," scan_wait_ms="," station_move_ms="," arm_ms="," vision_wait_ms="," other_ms="};
    unsigned i;uint32_t total=0;
    text("[run timing] ");text(label);text(" job=");number(route_timing.job);
    for(i=0;i<6u;i++){total+=values[i];text(names[i]);number(values[i]);}
    text(" total_ms=");number(total);text("\r\n");
}
static void run_timing_poll(void)
{
    FullRouteState *root=run.station_shift ? &station_parent : &run;
    uint8_t done;
    uint32_t elapsed;
    if(!route_timing.active)return;
    elapsed=clock_ms-route_timing.last_ms;route_timing.last_ms=clock_ms;
    route_timing.stage[route_timing.category]+=elapsed;route_timing.total[route_timing.category]+=elapsed;
    done=run.state==RUN_STOPPED || run.state==RUN_FAULT || (!run.station_shift && run.state==RUN_DONE);
    if(root->job!=route_timing.job || done) {
        run_timing_line("stage",route_timing.stage);memset(route_timing.stage,0,sizeof(route_timing.stage));
        route_timing.job=root->job;
    }
    if(done){run_timing_line("total",route_timing.total);route_timing.active=0;return;}
    if(run.station_shift)route_timing.category=2;
    else if(run_active)route_timing.category=0;
    else if(run.state==RUN_STATION && !root->job)route_timing.category=1;
    else if(arm_busy())route_timing.category=3;
    else if(vision_pick_state()==VISION_PICK_WAITING || vision_align_state()==VISION_PICK_WAITING
        || vision_stack_state()==VISION_PICK_WAITING)route_timing.category=4;
    else route_timing.category=5;
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
#ifndef CHASSIS_OBSTACLE_TEST
    if (station_auto_enabled()) {
        const char *error = station_fixed_ring_calibration_error();
        if (error) {
            text("[runimu rejected: ");text(error);text("]\r\n");
            return 0;
        }
    }
    if (station_auto_enabled() && (!arm_ready() || !arm_parked() || arm_lock.state != 2 || arm_platform_slot() != 1)) {
        text("[runimu rejected: automatic work needs arm on, declared pose/platform slot 1 and parked arm]\r\n");return 0;
    }
#endif
    nav_stop(&nav);
    turn_postcheck.active = 0;
    pose_set = 0;
    nav.position_confirmed = 0;
    test_mode = MODE_NAVIGATION;
    memset(&run, 0, sizeof(run));
    run.plan = initial_node == NODE_C;
    run.imu_mode = 1;
#ifdef CHASSIS_OBSTACLE_TEST
    obstacle_node_count=FULL_ROUTE_NODES;
    memcpy(obstacle_nodes,full_route_nodes[run.plan],FULL_ROUTE_NODES);
    obstacle_hits=obstacle_shot_valid=0;
    obstacle_entry_samples=obstacle_map_changed=0;
    memset(obstacle_observations,0,sizeof(obstacle_observations));obstacle_ray.valid=0;
    obstacle_observed_roads=0;
    obstacle_inset_node=255u;
    obstacle_inset_axes=0;
    run.turn_rpm=CAR_TURN_CLOSED_LOOP_RPM;
    run.initial_heading=run.plan ? HEAD_Y_NEG : HEAD_X_NEG;
#else
    run.station_enabled=(uint8_t)station_auto_enabled();
    run.turn_rpm = CAR_TURN_CLOSED_LOOP_RPM;
    run.initial_heading = HEAD_X_NEG;
#endif
    run.yaw_zero_cdeg = heading_wrap_cdeg((int32_t)sample.yaw_cdeg
        - CAR_MAP_YAW_SIGN * (int32_t)run.initial_heading * 9000);
    run.start_reference_valid=1;
    memset(&station_hold,0,sizeof(station_hold));
    imu_uart_fault = 0;
    memset(motor_feedback, 0, sizeof(motor_feedback));
    invalid_can_replies = 0;
    rejected_can_frames = 0;
    run.heading = run.target_heading = run.initial_heading;
#ifdef CHASSIS_OBSTACLE_TEST
    if(!obstacle_plan_remaining(initial_node))return 0;
#endif
    run.state = RUN_PREFLIGHT;
    run.segment_ms = clock_ms;
    run.last_query_ms = clock_ms - CAR_VELOCITY_QUERY_MS;
    run_clear_rx();
    run_active = 1;
#ifndef CHASSIS_OBSTACLE_TEST
    if(run.station_enabled) {
        run.scan_turn_stage=1;
        text("[scan station] first approach; automatic qr init; scan through first raw arrival\r\n");
        scanner_init(scanner.baud ? scanner.baud : CAR_SCANNER_UART_BAUD);
    }
#endif
    screen_clock.start_ms=clock_ms;screen_clock.elapsed_ms=0;
    screen_clock.started=screen_clock.running=1;
    memset(&route_timing,0,sizeof(route_timing));route_timing.active=1;route_timing.last_ms=clock_ms;
    text("[run route] ");
    for (i = 0; i < RUN_NODE_COUNT; i++)
    {
        if (i)
            text("->");
        send_byte((uint8_t)('A' + RUN_NODES[i]));
    }
#ifdef CHASSIS_OBSTACLE_TEST
    text("; forward_rpm=");number(CAR_RUN_RPM);
    text("; lateral_short_rpm=");number(CAR_LATERAL_SHORT_RPM);
    text("; lateral_long_rpm=");number(CAR_LATERAL_LONG_RPM);
    text("; turn_rpm=");number(run.turn_rpm);
    text("; retreat_rpm=");number(CAR_OBSTACLE_RETURN_RPM);
    text(run.plan ? "; C nose=-Y toward B; chassis only\r\n" : "; A nose=-X; chassis only\r\n");
#else
    text("; forward rpm=" CAR_STRING_VALUE(CAR_RUN_RPM) "; turn rpm=");
    number(run.turn_rpm);
    text(initial_node == NODE_C ? "; zone=2 bottom_right; original start position; initial heading=-X toward D/E/F"
                               : "; zone=1 top_right; original start position; initial heading=-X toward D/E/F");
    text("; start lateral rpm<=");number(CAR_LATERAL_MAX_RPM);text("; continuous forward/backward; station_auto per configuration\r\n");
    run_status();
#endif
    return 1;
}

/* 单独定角入口：正值左转、负值右转，相对接到命令时的真实航向。
 * 只启动状态机，完成结果由run_update推进；不在这里阻塞等电机。
 */
static int start_heading_operation(int32_t relative_cdeg, uint16_t rpm)
{
    HeadingSample sample = heading_snapshot();
    if (run_active || stop_pending || (drive.state != MOTION_IDLE && drive.state != MOTION_DONE)
        || relative_cdeg < -18000 || relative_cdeg > 18000
        || rpm < 20 || rpm > CAR_TURN_CLOSED_LOOP_RPM
        || !heading_sample_fresh(&sample, clock_ms, CAR_IMU_MAX_AGE_MS))
        return 0;
    nav_stop(&nav);
    turn_postcheck.active = 0;
    pose_set = nav.position_confirmed = 0;
    test_mode = MODE_ROTATION_TEST;
    memset(&run, 0, sizeof(run));
    run.imu_mode = run.turn_only = 1;
    run.turn_rpm = rpm;
    if (!heading_control_start(&run.heading_control, sample, relative_cdeg, clock_ms))
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

static int start_heading_turn(int16_t relative_degrees, uint16_t rpm)
{
    return relative_degrees && start_heading_operation((int32_t)relative_degrees*100,rpm);
}

static int scan_turn_start(int16_t relative_cdeg,uint16_t rpm)
{
    if(arm_lock.state || arm_enabled() || nav.state==NAV_MOVING || nav.state==NAV_WAIT_NODE
        || nav.state==NAV_WAIT_RETURN || run.state==RUN_STATION || run.state==RUN_WAIT_OBSTACLE)return 0;
    return start_heading_operation(relative_cdeg,rpm);
}
static int scan_turn_state(void)
{
    return run_active ? 1 : (run.turn_only && run.state==RUN_DONE ? 0 : -1);
}
static void scan_port_init(void)
{
    StartScanIO io={scan_turn_start,scan_turn_state,heading_snapshot,text};
    start_scan_init(&io);
}

/* 单次平移与导航复用同一速度状态机，保持发令时的车头朝向。 */
static int start_heading_translation(BodyAxis axis, int32_t mm, uint16_t rpm)
{
    HeadingSample sample = heading_snapshot();
    if (run_active || stop_pending || drive.state == MOTION_RUNNING || !mm
        || !heading_sample_fresh(&sample, clock_ms, CAR_IMU_MAX_AGE_MS)) return 0;
    nav_stop(&nav);
    turn_postcheck.active = 0;
    pose_set = nav.position_confirmed = 0;
    memset(&run, 0, sizeof(run));
    run.imu_mode = run.translation_only = 1;
    run.travel_axis = axis;
    run.travel_sign = mm > 0 ? 1 : -1;
    run.remaining_mm = (uint16_t)(mm > 0 ? mm : -mm);
    if(axis==BODY_FORWARD)run.test_forward_rpm=rpm<CAR_RUN_RPM ? rpm : CAR_RUN_RPM;
    else {
        run.lateral_rpm=car_lateral_limit_rpm(run.remaining_mm);
        if(rpm<run.lateral_rpm)run.lateral_rpm=rpm;
    }
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

static int start_heading_shift(int32_t mm, uint16_t rpm)
{
    return start_heading_translation(BODY_LEFT,mm,rpm);
}

/* 固定环位流水线通过离物净空后允许车载搬运；视觉微调仍只允许原有高位/末段靠近。
 * 普通道路仅允许进站高位观察准备及离站末尾回正，不放开地面取放。 */
static int station_chassis_pose_safe(void)
{
    int slide,lift;float yaw;
    float work_yaw=(run.job==1u || run.job==4u) ? ARM_PICK_YAW_DEG
        : (run.job==3u || run.job==6u) ? STN_STORAGE_ALIGN_YAW_DEG : STN_ROUGH_YAW_DEG;
    arm_pose_now(&slide,&lift,&yaw,NULL);
    if(station_index_overlap_active())return arm_enabled() && arm_pose_known() && !arm_failed();
    if(run_active && run.station_shift && run.station_approach_overlap)
        return arm_enabled() && arm_pose_known() && !arm_failed();
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
    int overlap=arm_translation_overlap_ready() || station_index_overlap_active();
#endif
    int storage = run.job == 3u || run.job == 6u;
    int index_rpm = storage ? STN_STORAGE_INDEX_RPM : STN_ROUGH_INDEX_RPM;
    if (run_active || stop_pending || run.state != RUN_STATION || !run.station_phase
        || (run.job != 1u && run.job != 2u && run.job != 4u && run.job != 5u && !storage) || arm_lock.state != 2
        || !station_chassis_pose_safe() || !mm) return 0;
    for (i = 0; i < MOTOR_COUNT; i++) {
        MotorFeedback *f = &motor_feedback[i];
        station_log_wheel_speed(i,"launch");
        if (!f->speed_valid || !(f->status_flags & MOTOR_FLAG_ENABLED)
            || (f->status_flags & MOTOR_FLAG_PROTECTED)
            || (uint32_t)(clock_ms - f->speed_ms) > ARM_FEEDBACK_TIMEOUT_MS
            || (uint32_t)(clock_ms - f->status_ms) > ARM_FEEDBACK_TIMEOUT_MS) return 0;
    }
    if(!station_hold_release())return 0;
    station_parent = run;
#if CAR_ARM_TRANSLATION_OVERLAP_ENABLED
    if(overlap)memcpy(parked_feedback,motor_feedback,sizeof(parked_feedback));
#endif
    if (!start_heading_shift(mm, (uint16_t)index_rpm)) {run = station_parent;return 0;}
    run.station_shift = run.station_enabled = 1;run.vision_debug=station_parent.vision_debug;
    run.travel_axis = axis;
    run.plan = station_parent.plan;run.edge = station_parent.edge;run.job = station_parent.job;
    run.heading = run.target_heading = station_parent.heading;
    memcpy(run.wheel_polarity,station_parent.wheel_polarity,sizeof(run.wheel_polarity));
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
    if(!station_chassis_start_axis(forward_mm ? forward_mm : left_mm,forward_mm ? BODY_FORWARD : BODY_LEFT))return 0;
    run.station_visual_shift=1;
    if(!forward_mm || !left_mm)return 1;
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
int station_chassis_vision_early(void)
{
    if(!station_chassis_busy())return 0;
    if(run.station_relock)return 1;
    if(run.encoder_ready!=ALL_MOTORS_MASK)return 0;
    if(run.vector_move)return abs(run.vector_target_mm[0]-run.vector_progress_mm[0])<=CAR_SETTLE_DISTANCE_MM
        && abs(run.vector_target_mm[1]-run.vector_progress_mm[1])<=CAR_SETTLE_DISTANCE_MM;
    return abs((int32_t)run.action_mm-run.progress_mm)<=CAR_SETTLE_DISTANCE_MM;
}
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

/* 包括F->E->D共线预看段；不能等进入最后730mm才从高速开始制动。 */
static int run_rough_arrival(void)
{
    return run.station_enabled && !run.station_shift && !run.translation_only && !run.turn_only && !run.obstacle_phase
        && run.job<FULL_ROUTE_JOBS && RUN_JOB_NODE(run.job)==NODE_D;
}

/* F/H仍交接近站残差；D保留编码器定位，减速后超程不能直接交给视觉。 */
static int run_visual_arrival(void)
{
    RoadHeading heading;
    return run.road.to!=NODE_D && run.station_enabled && !run.station_shift && !run.translation_only && !run.turn_only && !run.obstacle_phase
        && run.job<FULL_ROUTE_JOBS && run.road.to==RUN_JOB_NODE(run.job)
        && run.segment+1u==run.road.count && run_station_heading(run.road.to,&heading);
}

/* 下一步是工位视觉的转向：最后10mm即可交接，不把末段慢爬到1mm当作转向前提。
 * D允许提前转向，仍不接收超过原到位窗口的向前超程。 */
static int run_station_turn_early(void)
{
    RoadHeading heading;
    int32_t remaining=(int32_t)run.action_mm-run.progress_mm;
    int window=CAR_SETTLE_DISTANCE_MM;
    return CAR_RUN_CORNER_ENABLED && run.station_enabled && !run.station_shift && !run.translation_only && !run.turn_only
        && !run.obstacle_phase && !run.continuation_mm && run.encoder_ready==ALL_MOTORS_MASK
        && run.job<FULL_ROUTE_JOBS && run.road.to==RUN_JOB_NODE(run.job)
        && run.segment+1u==run.road.count && abs(remaining)<=window
        && (run.road.to!=NODE_D || remaining>=-CAR_DISTANCE_TOLERANCE_MM)
        && run_station_heading(run.road.to,&heading) && heading!=run.heading;
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
#ifdef CHASSIS_OBSTACLE_TEST
    obstacle_record_arrival(&run.road,run.edge);
#endif
    run.road_path_count = 0;
    run.edge++;
    node = RUN_NODES[run.edge];
    if (run.station_enabled && node == NODE_B && run.job == 0u) {
        run.scan_turn_stage=station_task_ready() ? 0u : 1u;run.turning=0;
        run.job++;run.transition_ready=1;
        text(station_task_ready() ? "[scan station] B reached; task latched; depart directly on route heading\r\n"
             : "[scan station] B reached; no task yet; continue scanning during straight travel\r\n");
        if(!run_road())run_fault("scan departure road failed");
        return;
    }
#ifdef CHASSIS_OBSTACLE_TEST
    if (node==NODE_B && run.job==0u && run.heading!=HEAD_X_NEG) {
        run.station_align_pending=1;run.target_heading=HEAD_X_NEG;run.remaining_mm=0;
        text("[chassis test] B: rear toward QR board\r\n");
        run_begin_alignment();return;
    }
#endif
    if (node == RUN_JOB_NODE(run.job)
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
    if (RUN_NODES[run.edge] == RUN_JOB_NODE(run.job))
    {
        if (run.station_enabled && run.job < 7u) {
            run_active=0;run.state=RUN_STATION;run.turning=0;run.transition_ready=0;
            run.station_phase=0;
            text("[run station] waiting for fresh four-wheel status; job=");number(run.job);text("\r\n");
            arm_lock_begin();return;
        }
        run.job++;
        text("[run checkpoint] job=");
        number(run.job);
        text("/8; station action skipped\r\n");
        text("[station estimate] node="); send_byte((uint8_t)('A' + RUN_NODES[run.edge]));
        text(" station="); text(station_names[run.job - 1u]);
        text("; physical_pose=unconfirmed\r\n");
    }
#ifdef CHASSIS_OBSTACLE_TEST
    if(run.job<FULL_ROUTE_JOBS && !obstacle_plan_remaining(RUN_NODES[run.edge])) {
        run_fault("no route to pending job");return;
    }
#endif
    if (run.edge == RUN_NODE_COUNT - 1u)
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
    int16_t return_mm=0;
    int through=station_through_departure(&return_mm);
    int overlap=station_departure_ready();
    if(through)overlap=2;
    if (RUN_NODES[run.edge] != RUN_JOB_NODE(run.job)
        || (!(arm_ready() && arm_parked()) && !overlap)) {
        run_fault("station completion/parked pose mismatch");return;
    }
    if(!station_hold_release())return;
    run.departure_overlap=(uint8_t)overlap;
    run.station_return_mm=through ? return_mm : 0;
    if(through) {
        text("[station through departure] return_mm=");signed_number(return_mm);
        text((run.job==2u || run.job==5u)
            ? "; platform release complete; onboard return continues; ring2 joins next road without stop\r\n"
            : "; final release complete; arm return continues; ring2 joins next road without stop\r\n");
    } else if(overlap)text(CAR_ARM_TRANSLATION_OVERLAP_ENABLED ? "[station departure] arm return overlaps turn/translation\r\n" : "[station departure] final rotation overlaps turn; translation waits\r\n");
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
    /* 持续确认原航向；工位保持独立输出，不修改这里的离站目标和完成判据。 */
    (void)heading_update(&run.heading_control,sample);
#endif
    if (run.station_relock) {
        if (arm_lock.state != 2) return;
        run.station_relock = 0;
    }
    if (!run.station_phase) {
        if (arm_lock.state!=2) return;
        if(run.job==1u && !station_task_ready()) {
            if(run.scan_turn_stage!=2u)text("[scan station] first F reached; waiting for task before raw work; scanner remains active\r\n");
            run.scan_turn_stage=2;return;
        }
        if(run.job==1u)run.scan_turn_stage=0;
        started=station_start(RUN_NODES[run.edge],run.job,clock_ms);
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
    if (!station_active() || station_departure_ready() || station_through_departure(NULL)) {
        if (!station_departure_ready() && !station_through_departure(NULL) && (!arm_ready() || !arm_parked())) {run_fault("station ended without parked arm");return;}
        /* 扫码完成也直接选择下一道路，不再恢复进站前航向后又转回B->E。 */
        run_resume_after_station();
    }
}

static int16_t run_heading_error(HeadingSample sample)
{
    return heading_wrap_cdeg(CAR_MAP_YAW_SIGN * (int32_t)run.target_heading * 9000
                             - ((int32_t)sample.yaw_cdeg - run.yaw_zero_cdeg));
}

/* 道路复用已停稳区间的位置；工位离站用本次交接后新采的位置，不重复停稳保持。 */
static void run_begin_baseline(void)
{
    run.settle_turn_active=0;
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
        HeadingDecision decision = heading_update(&run.heading_control, sample);
        if (decision == HEADING_FAULT) {run.alignment_waiting=1;run_wait_feedback(run.heading_control.fault);return;}
        /* 工位放料期间底盘未被驱动；离站已重采四轮反馈，不再重复等待零速保持。
         * 保留绝对航向参考，起步后的原陀螺仪闭环继续纠正小角差。 */
        if(CAR_ARM_TRANSLATION_OVERLAP_ENABLED && run.station_departing
            && error<=CAR_HEADING_TOLERANCE_CDEG && error>=-CAR_HEADING_TOLERANCE_CDEG) {
            text("[station departure] fresh wheels ready; start without repeated stop hold\r\n");
            run_begin_baseline();run.baseline_after_ms=run.segment_ms;return;
        }
        /* 串口输出期间可能收到新角度：保留原目标更新它，不能把新偏航重设为零。 */
        if (error <= CAR_HEADING_TOLERANCE_CDEG && error >= -CAR_HEADING_TOLERANCE_CDEG
            && run.heading_control.last_yaw_cdeg == sample.yaw_cdeg
            && run.heading_control.stable_samples >= CAR_HEADING_STABLE_SAMPLES
#if CAR_HEADING_STABLE_MS > 0
            && (uint32_t)(run.heading_control.sample_ms - run.heading_control.stable_ms) >= CAR_HEADING_STABLE_MS
#endif
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
        if ((feedback->status_flags ^ frame->data[1]) & MOTOR_FLAG_STALLED) {
            text("[wheel stall] addr=");number(frame->id >> CAN_MOTOR_ADDRESS_SHIFT);
            text(" flags_dec=");number(frame->data[1]);
            text(frame->data[1] & MOTOR_FLAG_STALLED
                ? "; reported; log only; driver protection unchanged\r\n"
                : "; cleared\r\n");
        }
        feedback->status_flags = frame->data[1];
        feedback->status_replies++;
        feedback->status_ms = frame->received_ms;
    } else if (frame->data[0] == MOTOR_POSITION_COMMAND && frame->data[1] == MOTOR_ARRIVAL_NOTICE)
        feedback->arrival_notices++;
    return 1;
}

static void arm_lock_fault(const char *reason)
{
    run_warning(reason);
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
        if(frame.data[0]==MOTOR_VELOCITY_COMMAND) {
            if(run_record_frame(&frame) && frame.data[1]==MOTOR_POSITION_ACK && station_hold.waiting
                && (uint32_t)(frame.received_ms-station_hold.velocity_ms)<CAR_VELOCITY_ACK_MS) {
                station_hold.acks|=(uint8_t)(1u<<i);
                if(station_hold.acks==ALL_MOTORS_MASK)station_hold.waiting=0;
            }
            continue;
        }
        if (frame.data[0] == MOTOR_STATUS_COMMAND && frame.length == 3u) kind = 0;
        else if (frame.data[0] == MOTOR_SPEED_COMMAND && frame.length == 5u) kind = 1;
        else continue;
        if (!(arm_lock.requested[i] & (1u << kind))
            || (int32_t)(frame.received_ms - arm_lock.query_ms[i][kind]) < 0
            || !run_record_frame(&frame)) continue;
        arm_lock.seen[i] |= (uint8_t)(1u << kind);
        if (kind) station_log_wheel_speed(i,"interlock");
        else if (!(motor_feedback[i].status_flags & MOTOR_FLAG_ENABLED)
                 || (motor_feedback[i].status_flags & MOTOR_FLAG_PROTECTED)) {
            arm_lock_fault("wheel disabled/protected");return;
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
        else {
            if (!arm_lock.settling) {arm_lock.settling = 1;arm_lock.stable_ms = clock_ms;}
#if ARM_CHASSIS_STOP_HOLD_MS > 0
            if ((uint32_t)(clock_ms - arm_lock.stable_ms) >= ARM_CHASSIS_STOP_HOLD_MS) {
#endif
            if (stop_pending) return;
            if (!arm_enabled()) arm_command("arm on", clock_ms);
            arm_lock.state = 2;
            text("[arm interlock acquired: chassis commands locked until parked arm off]\r\n");
#if ARM_CHASSIS_STOP_HOLD_MS > 0
            }
#endif
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
    station_hold_update(ready);
}

/* 交给视觉/索引/离站前归零并等实际停稳；只等待底盘，不阻塞机械臂状态机。 */
static int station_hold_release(void)
{
    unsigned i;
    if(!station_hold.active)return 1;
    station_hold.releasing=1;
    if(station_hold.command_mrpm || station_hold.command_rpm || station_hold.waiting || !station_hold.sent)return 0;
    for(i=0;i<MOTOR_COUNT;i++)if(!motor_feedback[i].speed_valid
        || (uint32_t)(clock_ms-motor_feedback[i].speed_ms)>ARM_FEEDBACK_TIMEOUT_MS
        || (int32_t)(motor_feedback[i].speed_ms-station_hold.velocity_ms)<0
        || abs(motor_feedback[i].speed_rpm)>CAR_VELOCITY_STOP_RPM)return 0;
    memset(&station_hold,0,sizeof(station_hold));return 1;
}

static void station_hold_update(uint8_t ready)
{
    HeadingSample sample=heading_snapshot();
    CanTxSnapshot tx;
    int32_t effort,desired=0,target,step;
    int16_t command,wheels[4];
    uint32_t elapsed;
    unsigned i;
    if(run_active || stop_pending || run.state!=RUN_STATION || !run.station_phase || run.station_relock
        || !run.start_reference_valid || run.vision_debug || run.job<1u || run.job>6u || arm_lock.state!=2)return;
    if(!station_hold.active) {
        if(!station_hold_enabled || !ready || !heading_sample_fresh(&sample,clock_ms,CAR_IMU_MAX_AGE_MS))return;
        /* yaw_zero只由开局姿态建立；目标为工位地图方向，绝不采纳到站偏角。 */
        station_hold.target_cdeg=heading_wrap_cdeg(run.yaw_zero_cdeg+CAR_MAP_YAW_SIGN*(int32_t)run.heading*9000);
        if(!heading_control_start(&station_hold.heading,sample,
            heading_wrap_cdeg(station_hold.target_cdeg-sample.yaw_cdeg),clock_ms))return;
        station_hold.active=1;station_hold.control_ms=clock_ms;
        text("[station hold] start target_yaw_cdeg=");signed_number(station_hold.target_cdeg);
        text("; reference=start; arm continues\r\n");
    }
    if(!station_hold_enabled)station_hold.releasing=1;
    if(!ready || can_rx_fault || run_rx_overflow || !heading_sample_fresh(&sample,clock_ms,CAR_IMU_MAX_AGE_MS))return;
    if(heading_update(&station_hold.heading,sample)==HEADING_FAULT)return;
    if(station_hold.waiting) {
        if((uint32_t)(clock_ms-station_hold.velocity_ms)<CAR_VELOCITY_ACK_MS)return;
        run_warning("station hold ACK incomplete; refresh wheels, arm continues");
        station_hold.waiting=station_hold.sent=0;
    }
    elapsed=clock_ms-station_hold.control_ms;
    if(elapsed<CAR_VELOCITY_CONTROL_MS)return;
    if(elapsed>CAR_DRIVE_SLEW_MAX_DT_MS)elapsed=CAR_DRIVE_SLEW_MAX_DT_MS;
    can_tx_snapshot(&tx);
    /* 纠偏是低优先级批次，给机构5/7及反馈查询留发送空间；不清队列、不取消机构。 */
    if(tx.blocked || tx.queued>8u)return;
    if(!station_hold.releasing && !(abs(station_hold.heading.error_cdeg)<=CAR_STATION_HOLD_QUIET_CDEG
        && abs(station_hold.heading.rate_cdeg_s)<=CAR_STATION_HOLD_RATE_CDEG_S)) {
        effort=station_hold.heading.error_cdeg*CAR_STATION_HOLD_KP_X100
            -station_hold.heading.rate_cdeg_s*CAR_STATION_HOLD_KD_X100;
        desired=effort>=0 ? (effort+5000)/10000 : -((-effort+5000)/10000);
        if(effort>0 && desired<1)desired=1;
        if(effort<0 && desired> -1)desired= -1;
        if(desired>CAR_STATION_HOLD_MAX_RPM)desired=CAR_STATION_HOLD_MAX_RPM;
        if(desired< -CAR_STATION_HOLD_MAX_RPM)desired= -CAR_STATION_HOLD_MAX_RPM;
    }
    target=desired*1000;step=(int32_t)(CAR_STATION_HOLD_SLEW_RPM_S*elapsed);
    if(target>station_hold.command_mrpm)station_hold.command_mrpm+=target-station_hold.command_mrpm<step ? target-station_hold.command_mrpm : step;
    else station_hold.command_mrpm-=station_hold.command_mrpm-target<step ? station_hold.command_mrpm-target : step;
    station_hold.control_ms=clock_ms;command=(int16_t)(station_hold.command_mrpm/1000);
    if(!station_hold.sent || command!=station_hold.command_rpm) {
        heading_control_translate_wheels(0,0,command,wheels);
        station_hold.acks=0;station_hold.waiting=1;station_hold.velocity_ms=clock_ms;
        for(i=0;i<MOTOR_COUNT;i++) {
            if(stop_pending || can_rx_fault || run_rx_overflow)return;
            Emm_V5_Vel_Control((uint8_t)(i+1u),wheels[i]>0 ? 1u : 0u,
                (uint16_t)abs(wheels[i]),CAR_MOTION_ACCELERATION,true);
        }
        if(!motor_sync(NULL))return;
        station_hold.command_rpm=command;station_hold.sent=1;
    }
    if((uint32_t)(clock_ms-station_hold.log_ms)>=CAR_CONTROL_LOG_MS) {
        station_hold.log_ms=clock_ms;text("[station hold] target_yaw_cdeg=");signed_number(station_hold.target_cdeg);
        text(" error_cdeg=");signed_number(station_hold.heading.error_cdeg);
        text(" turn_cmd=");signed_number(station_hold.command_rpm);text("; arm continues\r\n");
    }
    if(station_hold.releasing)(void)station_hold_release();
}

static int arm_serial_command(const char *s)
{
    if (strcmp(s,"@") && strcmp(s, "arm") && strncmp(s, "arm ", 4) && strncmp(s, "claw ", 5)) return 0;
    if (!strcmp(s, "arm") || !strcmp(s, "arm status") || !strcmp(s, "claw status")) {
        arm_command(s, clock_ms);
        text("[arm interlock] state="); number(arm_lock.state); text("\r\n");return 1;
    }
    if (!strcmp(s, "arm stop")) {
        start_button_cancel();
        arm_command(s,clock_ms);
        if (run.station_enabled && (run_active || run.state==RUN_STATION)) run_fault("operator stopped station arm");
        return 1;
    }
    if (run_active || run.state==RUN_STATION) {text("[arm rejected: route/station owns motion; use ! to stop]\r\n");return 1;}
    if (!strcmp(s, "arm off")) {
        start_button_cancel();
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

/* 只登记已经进入最后低速几毫米的视觉/索引修正，不改变原到位/停车判据。 */
int station_chassis_approach_begin(void)
{
    uint8_t i;
    HeadingSample sample=heading_snapshot();
    if(!run_active || !run.station_shift || run.vision_debug || run.station_approach_overlap
        || stop_pending || can_rx_fault || run_rx_overflow || arm_lock.state!=2
        || !arm_ready() || arm_busy() || run.encoder_ready!=ALL_MOTORS_MASK
        || run.velocity_waiting || run.acks!=ALL_MOTORS_MASK
        || (run.velocity_phase!=VELOCITY_FORWARD && run.velocity_phase!=VELOCITY_STOP_FORWARD)
        || !heading_sample_fresh(&sample,clock_ms,CAR_IMU_MAX_AGE_MS)
        || !run_velocity_feedback_fresh()
        || abs(run.heading_control.error_cdeg)>STN_APPROACH_OVERLAP_YAW_CDEG
        || abs(run.forward_cmd_rpm)>STN_APPROACH_OVERLAP_RPM
        || abs(run.left_cmd_rpm)>STN_APPROACH_OVERLAP_RPM
        || abs(run.turn_cmd_rpm)>STN_APPROACH_OVERLAP_RPM) return 0;
    for(i=0;i<MOTOR_COUNT;i++)
        if(!(motor_feedback[i].status_flags&MOTOR_FLAG_ENABLED)
            || (motor_feedback[i].status_flags&MOTOR_FLAG_PROTECTED))return 0;
    if(run.vector_move) {
        for(i=0;i<2u;i++)
            if(abs(run.vector_target_mm[i]-run.vector_progress_mm[i])>STN_APPROACH_OVERLAP_MM)return 0;
    } else if(abs((int32_t)run.action_mm-run.progress_mm)>STN_APPROACH_OVERLAP_MM)return 0;
    run.station_approach_overlap=1;
    return 1;
}

/* 每批四轮速度先收到ACK，才提交下一批；同步启动前检查急停和发送错误。 */
static int run_velocity_send(int16_t forward, int16_t left, int16_t turn)
{
    int16_t wheels[4];
    int32_t maximum=0;uint16_t limit=run_wheel_limit_rpm();
    uint8_t i, changed = !run.velocity_sent, all_zero = 1;
    /* 先保留航向修正，再统一缩减平移，避免满速时把转向也一起稀释。 */
    if(turn>(int16_t)limit)turn=(int16_t)limit;
    if(turn< -(int16_t)limit)turn= -(int16_t)limit;
    heading_control_translate_wheels(forward, left, 0, wheels);
    for(i=0;i<MOTOR_COUNT;i++)if(abs(wheels[i])>maximum)maximum=abs(wheels[i]);
    if(maximum>limit-abs(turn))for(i=0;i<MOTOR_COUNT;i++)
        wheels[i]=(int16_t)((int32_t)wheels[i]*(limit-abs(turn))/maximum);
    for(i=0;i<MOTOR_COUNT;i++)wheels[i]+=(int16_t)((CAR_TURN_CCW_DIR ? 1 : -1)*turn);
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
        run_vector_polarity_command(i,(int8_t)(value>0 ? 1 : value<0 ? -1 : 0));
        run.wheel_rpm[i] = value;
        Emm_V5_Vel_Control(i + 1u, value > 0 ? 1u : 0u,
            (uint16_t)(value < 0 ? -value : value), CAR_MOTION_ACCELERATION, true);
        delay_ms(1);
    }
    if (!motor_sync(0)) return 0;
    for(i=0;i<MOTOR_COUNT;i++) {
        int8_t sign=run.wheel_rpm[i]>0 ? 1 : run.wheel_rpm[i]<0 ? -1 : 0;
        if(sign && run.vector_last_wheel_sign[i] && sign!=run.vector_last_wheel_sign[i]
            && run.vector_wheel_reversals[i]<65535u)run.vector_wheel_reversals[i]++;
        if(sign)run.vector_last_wheel_sign[i]=sign;
    }
    run.velocity_sent = 1;
    if (!all_zero) {run.zero_ms = 0;run.stopped_ms = 0;}
    else if (!run.zero_ms) {run.zero_ms = clock_ms;run.stopped_ms = 0;}
    return 1;
}

static int run_velocity_stopped(void)
{
    uint8_t i;
    int node_handoff=!CAR_RUN_CORNER_ENABLED && run_active && run.imu_mode
        && !run.translation_only && !run.turn_only && !run.station_shift && !run.obstacle_phase
        && !run.continuation_mm
        && (run.velocity_phase==VELOCITY_STOP_FORWARD || run.velocity_phase==VELOCITY_TURN);
    int node_stop=node_handoff && run.velocity_phase==VELOCITY_STOP_FORWARD;
    int manual_stop=!run.station_shift && (run.translation_only || run.turn_only);
    if (!run.zero_ms || run.velocity_waiting || !run_velocity_feedback_fresh()) return 0;
    for (i = 0; i < MOTOR_COUNT; i++) {
        MotorFeedback *f = &motor_feedback[i];
        int32_t after_zero = (int32_t)(f->speed_ms - run.zero_ms);
        station_log_wheel_speed(i,"stop");
        if ((manual_stop || run.obstacle_phase || node_handoff)
            && (f->speed_rpm>CAR_VELOCITY_STOP_RPM || f->speed_rpm<-CAR_VELOCITY_STOP_RPM)) {
            run.stopped_ms=0;return 0; /* 手动单段/定角转向也必须等待四轮速度反馈进入原停稳窗。 */
        }
        if (after_zero < (int32_t)CAR_HEADING_SETTLE_MS || (manual_stop && after_zero<=0)) {
            run.stopped_ms = 0;
            return 0;
        }
    }
    if (!run.stopped_ms) run.stopped_ms = clock_ms;
    if(node_stop && (uint32_t)(clock_ms-run.stopped_ms)<CAR_ROUTE_NODE_STOP_MS)return 0;
#if CAR_HEADING_STABLE_MS > 0
    return (uint32_t)(clock_ms - run.stopped_ms) >= CAR_HEADING_STABLE_MS;
#else
    return 1;
#endif
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
#ifdef CHASSIS_OBSTACLE_TEST
    if(obstacle_start_node(run.road.from)) {
        MotionParameters entry=run.road.segments[run.segment];
        uint8_t axis=entry.axis==BODY_LEFT ? OBSTACLE_ENTRY_X : OBSTACLE_ENTRY_Y;
        int16_t amount=entry.axis==BODY_LEFT ? OBSTACLE_START_INSET_MM : obstacle_entry_y(run.road.from);
        uint8_t kept=obstacle_inset_node==run.road.from ? obstacle_inset_axes : 0u;
        if(!(kept&axis) && ((amount>0 && entry.mm>=amount) || (amount<0 && entry.mm<=amount))
            && (int)mm>=abs(amount)-CAR_DISTANCE_TOLERANCE_MM) {
            obstacle_inset_node=run.road.from;obstacle_inset_axes=(uint8_t)(kept|axis);
            mm=mm>(uint16_t)abs(amount) ? (uint16_t)(mm-abs(amount)) : 0u;
            text("[obstacle entry] keep axis=");text(axis==OBSTACLE_ENTRY_X ? "X" : "Y");
            text(" mm=");number((uint32_t)abs(amount));text(" at ");send_byte((uint8_t)('A'+run.road.from));text("\r\n");
            if(!mm)return 1;
        }
    }
#endif
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
    } else {
        run.road_path_count = 0;
#ifdef CHASSIS_OBSTACLE_TEST
        obstacle_record_arrival(&run.road,run.edge);
#endif
    }
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

static void run_cpos_number(int64_t value)
{
    /* CAN原始位置是32位幅值另带符号；沿用轻量输出，避免在运动深调用链内用printf。 */
    if(value<0){text("-");value=-value;}
    number((uint32_t)value);
}

static void run_polarity_confirm(uint8_t wheel,int8_t polarity,int64_t origin,int64_t previous,
                                 int64_t position,int8_t command,const char *source)
{
    if(run.wheel_polarity[wheel])return;
    run.wheel_polarity[wheel]=polarity;
    text("[encoder polarity] addr=");number(wheel+1u);text(" source=");text(source);
    text(" cpos=");run_cpos_number(origin);text(",");run_cpos_number(previous);text(",");run_cpos_number(position);
    text(" command_sign=");signed_number(command);text(" polarity=");signed_number(polarity);text("\r\n");
}

/* 指令换向必须在提交时登记，不能等收到下一份CPOS才发现；其间可能已经换回。 */
static void run_vector_polarity_command(uint8_t wheel,int8_t sign)
{
    uint8_t bit=(uint8_t)(1u<<wheel);
    int32_t speed=motor_feedback[wheel].speed_rpm;
    if(!sign || run.wheel_polarity[wheel] || sign==run.vector_learn.command_sign[wheel])return;
    run.vector_learn.command_sign[wheel]=sign;
    run.vector_learn.command_ms[wheel]=run.vector_learn.after_ms[wheel]=clock_ms;
    run.vector_learn.before_speed_sign[wheel]=(int8_t)(speed>0 ? 1 : speed<0 ? -1 : 0);
    run.vector_learn.speed_sign[wheel]=0;
    if(speed)run.vector_learn.wait_speed|=bit;else run.vector_learn.wait_speed&=(uint8_t)~bit;
    run.vector_learn.origin_ready&=(uint8_t)~bit;
    run.vector_learn.first_origin&=(uint8_t)~bit;
    /* 本段第一次发非零指令且实际速度为零时，共同起点就是有效学习原点。
     * 后续换向一律重建，不能把旧方向累积位移带入。 */
    if(sign && !speed && !run.vector_last_wheel_sign[wheel]) {
        run.vector_learn.origin[wheel]=run.encoder_start[wheel];
        run.vector_learn.origin_ready|=bit;run.vector_learn.first_origin|=bit;
    }
    run.encoder_previous_delta[wheel]=0;
}

/* 单轴/MOVEXY共用物理极性确认；学习区间跟随实际提交的轮指令，不跟随主轴假定方向。 */
static void run_learn_wheel_polarity(uint8_t i,int64_t learn_counts,const char *source)
{
    MotorFeedback *f=&motor_feedback[i];
    uint8_t bit=(uint8_t)(1u<<i);
    int8_t sign=run.vector_learn.command_sign[i];
    int8_t applied=(int8_t)(run.wheel_rpm[i]>0 ? 1 : run.wheel_rpm[i]<0 ? -1 : 0);
    int8_t speed=(int8_t)(f->speed_rpm>0 ? 1 : f->speed_rpm<0 ? -1 : 0);
    if(run.wheel_polarity[i] || !sign || applied!=sign || run.velocity_waiting || !f->speed_valid
        || (int32_t)(f->speed_ms-run.vector_learn.command_ms[i])<=0)return;
    if((run.vector_learn.wait_speed&bit) && speed
        && speed==run.vector_learn.before_speed_sign[i])return;
    run.vector_learn.wait_speed&=(uint8_t)~bit;
    if(speed!=run.vector_learn.speed_sign[i]) {
        run.vector_learn.speed_sign[i]=speed;
        if(run.vector_learn.first_origin&bit)run.vector_learn.first_origin&=(uint8_t)~bit;
        else {
            run.vector_learn.after_ms[i]=f->speed_ms;
            run.vector_learn.origin_ready&=(uint8_t)~bit;
        }
    }
    if((int32_t)(f->position_ms-run.vector_learn.after_ms[i])<0)return;
    if(!(run.vector_learn.origin_ready&bit)) {
        run.vector_learn.origin[i]=f->position;run.encoder_previous_delta[i]=0;
        run.encoder_sample_ms[i]=f->position_ms;run.vector_learn.origin_ready|=bit;
    } else if(f->position_ms!=run.encoder_sample_ms[i]) {
        int64_t observed=f->position-run.vector_learn.origin[i];
        int64_t previous=run.encoder_previous_delta[i];
        if((observed>=learn_counts && previous>=learn_counts && observed>previous)
            || (observed<=-learn_counts && previous<=-learn_counts && observed<previous))
            run_polarity_confirm(i,(int8_t)((observed>0 ? 1 : -1)*sign),run.vector_learn.origin[i],
                run.vector_learn.origin[i]+previous,f->position,sign,source);
        run.encoder_previous_delta[i]=observed;run.encoder_sample_ms[i]=f->position_ms;
    }
}

/* 逆混控使用整段共同起点；学习使用换向后的独立区间，两种原点不能混用。
 * 已确认的物理极性不随MOVEXY重新猜测；冷启动只在新指令/新速度/新位置相互对应后学习。 */
static int run_vector_distance(void)
{
    static const int8_t forward_sign[4]={-1,1,1,-1};
    static const int8_t left_sign[4]={-1,-1,1,1};
    static const int8_t null_sign[4]={1,-1,1,-1};
    int64_t wheel_delta[4];
    int64_t sum_forward=0,sum_left=0;
    uint32_t age=0,common_ms;
    float ppm=CALIBRATED_PPM_X<CALIBRATED_PPM_Y ? CALIBRATED_PPM_X : CALIBRATED_PPM_Y;
    int64_t learn_counts=(int64_t)(CAR_ENCODER_LEARN_MM*ppm*CAR_ENCODER_COUNTS_PER_REV/CAR_POSITION_PULSES_PER_REV)+1;
    uint8_t i,known=0,missing=0;
    for(i=0;i<MOTOR_COUNT;i++) {
        int64_t delta=motor_feedback[i].position-run.encoder_start[i];
        MotorFeedback *sample=&motor_feedback[i];
        if(sample->position_ms!=run.straight_hold.latest_ms[i]) {
            run.straight_hold.previous[i]=run.straight_hold.latest[i];
            run.straight_hold.previous_ms[i]=run.straight_hold.latest_ms[i];
            run.straight_hold.latest[i]=sample->position;run.straight_hold.latest_ms[i]=sample->position_ms;
        }
        if((uint32_t)(clock_ms-sample->position_ms)>age)age=clock_ms-sample->position_ms;
        wheel_delta[i]=0;
        if(!run.encoder_sign[i])missing=i;
        if(!run.encoder_sign[i]) {
            run_learn_wheel_polarity(i,learn_counts,"vector");
            if(run.wheel_polarity[i]) {
                run.encoder_sign[i]=run.wheel_polarity[i];run.encoder_ready|=(uint8_t)(1u<<i);
            }
        }
        wheel_delta[i]=delta*run.encoder_sign[i];
        if(run.encoder_sign[i])known++;else missing=i;
    }
    /* 与道路观测一样，串行CPOS先插值到同一时刻，才可作四轮逆混控或三轮补齐。 */
    common_ms=clock_ms-age;
    for(i=0;i<MOTOR_COUNT;i++) {
        uint32_t span=run.straight_hold.latest_ms[i]-run.straight_hold.previous_ms[i];
        uint32_t offset=common_ms-run.straight_hold.previous_ms[i];
        int64_t position;
        if(!span || span>CAR_VELOCITY_FEEDBACK_MS || offset>span)return 1;
        position=run.straight_hold.previous[i]
            +(run.straight_hold.latest[i]-run.straight_hold.previous[i])*offset/span;
        wheel_delta[i]=(position-run.encoder_start[i])*run.encoder_sign[i];
    }
    if(known==3u) {
        int64_t residual=0;
        /* 现有混控矩阵满足w1-w2+w3-w4=0。三轮已知时可完整观测三个车身分量，
         * 未确认轮不能当成0；这里只补齐逆解观测，不把推导值当作该轮极性标定。 */
        for(i=0;i<MOTOR_COUNT;i++)if(i!=missing)residual+=wheel_delta[i]*null_sign[i];
        wheel_delta[missing]=-residual*null_sign[missing];
    }
    for(i=0;i<MOTOR_COUNT;i++) {
        sum_forward+=wheel_delta[i]*forward_sign[i];
        sum_left+=wheel_delta[i]*left_sign[i];
    }
    run.vector_progress_mm[0]=(int32_t)((float)sum_forward*CAR_POSITION_PULSES_PER_REV
        /(4.0f*CAR_ENCODER_COUNTS_PER_REV*CALIBRATED_PPM_Y));
    run.vector_progress_mm[1]=(int32_t)((float)sum_left*CAR_POSITION_PULSES_PER_REV
        /(4.0f*CAR_ENCODER_COUNTS_PER_REV*CALIBRATED_PPM_X));
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
    if(speed<CAR_MOVEXY_MIN_RPM)speed=CAR_MOVEXY_MIN_RPM;
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

static int run_straight_hold_enabled(void)
{
    return (run.travel_axis==BODY_LEFT || CAR_STRAIGHT_HOLD_ENABLED)
        && !run.vector_move && !run.station_shift && !run.obstacle_phase;
}

static int run_path_pose_supported(void)
{
    return run.translation_only && !run.turn_only
        && !run.station_shift && !run.vector_move && !run.obstacle_phase;
}
static int run_path_hold_enabled(void)
{
    return run_path_pose_supported() && gyro_drive_feedback_enabled
        && (run.travel_axis==BODY_LEFT ? lateral_hold_feedback_enabled
            : CAR_STRAIGHT_HOLD_ENABLED && encoder_drift_feedback_enabled);
}
static int run_lateral_path_enabled(void)
{
    return run_path_hold_enabled() && run.travel_axis==BODY_LEFT;
}
static int run_drive_gain_schedule_enabled(void)
{
    return CAR_DRIVE_GAIN_SCHEDULE_ENABLED && run_path_pose_supported() && gyro_drive_feedback_enabled;
}
static HeadingDriveGains run_drive_heading_gains(void)
{
    const HeadingDriveGains fixed={CAR_DRIVE_HEADING_KP_X100,CAR_DRIVE_HEADING_KD_X100};
    /* forward_mrpm是两个主轴共用的速度斜坡幅值，补偿/转向不参与选档，反向也沿用同一幅值。 */
    return run_drive_gain_schedule_enabled()
        ? heading_control_drive_gains(run.travel_axis==BODY_LEFT,(uint16_t)(run.forward_mrpm/1000)) : fixed;
}
static void run_lateral_yaw_sample(HeadingSample sample)
{
    unsigned index;
    if(!run_path_pose_supported() || !sample.valid
        || (run.lateral_pose.count && run.lateral_pose.sequence==sample.sequence))return;
    index=run.lateral_pose.head;
    run.lateral_pose.yaw[index].ms=sample.received_ms;
    run.lateral_pose.yaw[index].yaw_cdeg=sample.yaw_cdeg;
    run.lateral_pose.head=(uint8_t)((index+1u)%64u);
    if(run.lateral_pose.count<64u)run.lateral_pose.count++;
    run.lateral_pose.sequence=sample.sequence;
}
static int run_lateral_yaw_at(uint32_t ms,float *relative_cdeg)
{
    unsigned first=(run.lateral_pose.head+64u-run.lateral_pose.count)%64u;
    for(unsigned n=0;n<run.lateral_pose.count;n++) {
        unsigned a=(first+n)%64u,b=(a+1u)%64u;
        uint32_t span,offset;
        if(ms==run.lateral_pose.yaw[a].ms) {
            *relative_cdeg=(float)heading_wrap_cdeg(run.lateral_pose.yaw[a].yaw_cdeg-run.lateral_pose.origin_yaw_cdeg);return 1;
        }
        if(n+1u==run.lateral_pose.count)break;
        span=run.lateral_pose.yaw[b].ms-run.lateral_pose.yaw[a].ms;
        offset=ms-run.lateral_pose.yaw[a].ms;
        if(span && span<=CAR_IMU_MAX_AGE_MS && offset<=span) {
            *relative_cdeg=(float)heading_wrap_cdeg(run.lateral_pose.yaw[a].yaw_cdeg-run.lateral_pose.origin_yaw_cdeg)
                +(float)heading_wrap_cdeg(run.lateral_pose.yaw[b].yaw_cdeg-run.lateral_pose.yaw[a].yaw_cdeg)*offset/span;
            return 1;
        }
    }
    return 0; /* 不拿最新Yaw冒充旧轮位移时刻的航向，也不外推丢失的角度。 */
}
static int run_lateral_yaw_interval(uint32_t from,uint32_t to)
{
    float yaw;
    unsigned first=(run.lateral_pose.head+64u-run.lateral_pose.count)%64u;
    if(!run_lateral_yaw_at(from,&yaw))return 0;
    for(unsigned n=0;n+1u<run.lateral_pose.count;n++) {
        unsigned a=(first+n)%64u,b=(a+1u)%64u;
        if(run.lateral_pose.yaw[b].ms-run.lateral_pose.yaw[a].ms>CAR_IMU_MAX_AGE_MS
            && (int32_t)(to-run.lateral_pose.yaw[a].ms)>0
            && (int32_t)(run.lateral_pose.yaw[b].ms-from)>0)return 0;
    }
    return 1;
}
static void run_lateral_pose_observe(float body_forward,float body_left,uint32_t ms)
{
    float yaw,mid,delta_forward,delta_left,c,s,cross;
    if(!run_path_pose_supported() || (run.lateral_pose.valid && ms==run.lateral_pose.sample_ms)
        || !run_lateral_yaw_at(ms,&yaw) || !run_lateral_yaw_interval(run.lateral_pose.sample_ms,ms))return;
    delta_forward=body_forward-run.lateral_pose.body_forward_mm;
    delta_left=body_left-run.lateral_pose.body_left_mm;
    mid=(run.lateral_pose.last_yaw_cdeg
        +heading_wrap_cdeg((int32_t)lroundf(yaw-run.lateral_pose.last_yaw_cdeg))*0.5f)*0.0001745329252f;
    c=cosf(mid);s=sinf(mid);
    run.lateral_pose.forward_mm+=c*delta_forward-s*delta_left;
    run.lateral_pose.left_mm+=s*delta_forward+c*delta_left;
    cross=run.travel_axis==BODY_LEFT
        ? run.lateral_pose.forward_mm-car_lateral_forward_pulses(run.lateral_pose.left_mm)/CALIBRATED_PPM_Y
        : run.lateral_pose.left_mm-car_straight_left_trim(run.lateral_pose.forward_mm);
    if(run.lateral_pose.valid) {
        uint32_t dt=ms-run.lateral_pose.sample_ms;
        float rate=(cross-run.lateral_pose.cross_mm)*1000.0f/dt;
        float alpha=(float)dt/(CAR_LATERAL_HOLD_RATE_FILTER_MS+dt);
        run.lateral_pose.forward_rate_mm_s+=alpha*(rate-run.lateral_pose.forward_rate_mm_s);
    }
    /* 首份位姿只建立速度基线，既有偏移不当作瞬时速度尖峰。 */
    run.lateral_pose.cross_mm=cross;
    run.lateral_pose.body_forward_mm=body_forward;run.lateral_pose.body_left_mm=body_left;
    run.lateral_pose.last_yaw_cdeg=yaw;run.lateral_pose.sample_ms=ms;run.lateral_pose.valid=1;
}

/* 一段内按目标方向锁定标定，末端回拉不切换；工位/双轴仍用原系数。 */
static float run_lateral_ppm(void)
{
    return run.travel_axis==BODY_LEFT && (run_path_pose_supported() || run.obstacle_phase)
        ? car_road_ppm_x(run.travel_sign) : CALIBRATED_PPM_X;
}

/* CAN轮询的四个CPOS不是同时取得；先在各轮前后两份采样间插值到共同时间，
 * 避免把高速直行时的采样先后差误认为侧移。只插值，不用轮速读数外推位置。 */
static void run_straight_observe(void)
{
    static const int8_t projection[4]={1,-1,1,-1};
    uint32_t age=0,common_ms;
    int64_t sum=0,along=0;
    unsigned i;
    if(!run_straight_hold_enabled())return;
    for(i=0;i<MOTOR_COUNT;i++) {
        MotorFeedback *f=&motor_feedback[i];
        if(f->position_ms!=run.straight_hold.latest_ms[i]) {
            run.straight_hold.previous[i]=run.straight_hold.latest[i];
            run.straight_hold.previous_ms[i]=run.straight_hold.latest_ms[i];
            run.straight_hold.latest[i]=f->position;
            run.straight_hold.latest_ms[i]=f->position_ms;
        }
        if((uint32_t)(clock_ms-run.straight_hold.latest_ms[i])>age)
            age=clock_ms-run.straight_hold.latest_ms[i];
    }
    if(run.encoder_ready!=ALL_MOTORS_MASK || age>CAR_VELOCITY_FEEDBACK_MS)return;
    common_ms=clock_ms-age;
    if((uint32_t)(common_ms-run.straight_hold.sample_ms)>0x7fffffffu)return;
    for(i=0;i<MOTOR_COUNT;i++) {
        uint32_t span=run.straight_hold.latest_ms[i]-run.straight_hold.previous_ms[i];
        uint32_t offset=common_ms-run.straight_hold.previous_ms[i];
        int64_t position;
        if(!span || span>CAR_VELOCITY_FEEDBACK_MS || offset>span)return;
        position=run.straight_hold.previous[i]
            +(run.straight_hold.latest[i]-run.straight_hold.previous[i])*offset/span;
        /* 单轴encoder_sign把本段前/后运动都归一为正；恢复车身左右符号要乘travel_sign。 */
        position=(position-run.encoder_start[i])*run.encoder_sign[i];
        along+=position;
        sum+=position*projection[i]*run.travel_sign;
    }
    /* 轮位移里的主动横移要扣除标定补偿，剩余量才是地面侧偏的估计。 */
    if(run.travel_axis==BODY_LEFT) {
        float along_mm=(float)along*run.travel_sign*CAR_POSITION_PULSES_PER_REV
            /(4.0f*CAR_ENCODER_COUNTS_PER_REV*run_lateral_ppm());
        float body_forward=(float)sum*CAR_POSITION_PULSES_PER_REV
            /(4.0f*CAR_ENCODER_COUNTS_PER_REV*CALIBRATED_PPM_Y);
        run_lateral_pose_observe(body_forward,along_mm,common_ms);
        if(run_lateral_path_enabled() && run.lateral_pose.valid)
            run.straight_hold.lateral_mm=(int32_t)lroundf(run.lateral_pose.forward_mm
                -car_lateral_forward_pulses(run.lateral_pose.left_mm)/CALIBRATED_PPM_Y);
        else run.straight_hold.lateral_mm=(int32_t)(body_forward-car_lateral_forward_pulses(along_mm)/CALIBRATED_PPM_Y);
    } else {
        float body_forward=(float)along*run.travel_sign*CAR_POSITION_PULSES_PER_REV
            /(4.0f*CAR_ENCODER_COUNTS_PER_REV*car_road_ppm_y(run.travel_sign));
        float body_left=(float)sum*CAR_POSITION_PULSES_PER_REV
            /(4.0f*CAR_ENCODER_COUNTS_PER_REV*CALIBRATED_PPM_X);
        run_lateral_pose_observe(body_forward,body_left,common_ms);
        if(run_path_hold_enabled() && run.lateral_pose.valid)
            run.straight_hold.lateral_mm=(int32_t)lroundf(run.lateral_pose.cross_mm);
        else run.straight_hold.lateral_mm=(int32_t)(body_left-car_straight_left_trim(body_forward));
    }
    run.straight_hold.sample_ms=common_ms;run.straight_hold.valid=1;
}

/* 手动固定参考系四方向允许末端正交补位，其余动作只在主轴运动时纠偏。 */
static int16_t run_straight_correction(int16_t forward,int16_t turn,uint8_t due)
{
    int32_t error=run.straight_hold.lateral_mm,target,limit,change;
    uint8_t feedback_enabled=run.travel_axis==BODY_LEFT ? lateral_hold_feedback_enabled : encoder_drift_feedback_enabled;
    int32_t max_rpm=run.travel_axis==BODY_LEFT ? CAR_LATERAL_HOLD_MAX_RPM : CAR_STRAIGHT_HOLD_MAX_RPM;
    int32_t gain=run.travel_axis==BODY_LEFT ? CAR_LATERAL_HOLD_KP_X100 : CAR_STRAIGHT_HOLD_KP_X100;
    if((!forward && !run_path_hold_enabled()) || !run_straight_hold_enabled()
        || (feedback_enabled && (!run.straight_hold.valid
            || (uint32_t)(clock_ms-run.straight_hold.sample_ms)>CAR_VELOCITY_FEEDBACK_MS))) {
        run.straight_hold.command_mrpm=run.straight_hold.command_rpm=0;return 0;
    }
    limit=run_path_hold_enabled() ? run_wheel_limit_rpm()-abs(turn) : abs(forward)-abs(turn)-1;
    if(limit<0)limit=0;
    if(limit>max_rpm)limit=max_rpm;
    target=run.travel_axis==BODY_LEFT ? (int32_t)lroundf(car_lateral_forward_pulses((float)forward*CAR_POSITION_PULSES_PER_REV/CALIBRATED_PPM_X)/CAR_POSITION_PULSES_PER_REV)
        : (int32_t)lroundf(car_straight_left_trim(forward)*CALIBRATED_PPM_X/car_road_ppm_y(forward));
    if(feedback_enabled && abs(error)>CAR_STRAIGHT_HOLD_DEADBAND_MM) {
        int32_t feedback=(abs(error)-CAR_STRAIGHT_HOLD_DEADBAND_MM)*gain/100;
        if(feedback<1)feedback=1;
        target+=error>0 ? -feedback : feedback;
    }
    if(run_path_hold_enabled())
        target-=(int32_t)lroundf(run.lateral_pose.forward_rate_mm_s
            *(run.travel_axis==BODY_LEFT ? CAR_LATERAL_HOLD_KD_X100 : CAR_STRAIGHT_HOLD_KD_X100)/100.0f);
    if(target>limit)target=limit;
    if(target< -limit)target=-limit;
    if(due) {
        uint32_t elapsed=clock_ms-run.control_ms;
        if(elapsed>CAR_DRIVE_SLEW_MAX_DT_MS)elapsed=CAR_DRIVE_SLEW_MAX_DT_MS;
        change=(int32_t)((run.travel_axis==BODY_LEFT ? CAR_LATERAL_HOLD_SLEW_RPM_S : CAR_DRIVE_ACCEL_RPM_PER_S)*elapsed);
        target*=1000;
        if(target>run.straight_hold.command_mrpm) {
            run.straight_hold.command_mrpm+=change;
            if(run.straight_hold.command_mrpm>target)run.straight_hold.command_mrpm=target;
        } else {
            run.straight_hold.command_mrpm-=change;
            if(run.straight_hold.command_mrpm<target)run.straight_hold.command_mrpm=target;
        }
    }
    if(run.straight_hold.command_mrpm>limit*1000)run.straight_hold.command_mrpm=limit*1000;
    if(run.straight_hold.command_mrpm< -limit*1000)run.straight_hold.command_mrpm=-limit*1000;
    run.straight_hold.command_rpm=(int16_t)(run.straight_hold.command_mrpm/1000);
    return run.straight_hold.command_rpm;
}

static float run_single_axis_ppm(void)
{
    if (run.travel_axis == BODY_LEFT) return run_lateral_ppm();
    return run.station_shift ? CALIBRATED_PPM_Y : car_road_ppm_y(run.travel_sign);
}

static int run_velocity_distance(void)
{
    int64_t sum = 0;
    float ppm = run_single_axis_ppm();
    int64_t learn_counts = (int64_t)(CAR_ENCODER_LEARN_MM * ppm * CAR_ENCODER_COUNTS_PER_REV
                                    / CAR_POSITION_PULSES_PER_REV) + 1;
    uint8_t i;
    if(run.vector_move)return run_vector_distance();
    for (i = 0; i < MOTOR_COUNT; i++) {
        int64_t delta = motor_feedback[i].position - run.encoder_start[i];
        if(!run.encoder_sign[i]) {
            static const int8_t fs[4]={-1,1,1,-1},ls[4]={-1,-1,1,1};
            run_learn_wheel_polarity(i,learn_counts,"straight");
            if(run.wheel_polarity[i]) {
                run.encoder_sign[i]=(int8_t)(run.wheel_polarity[i]
                    *(run.travel_axis==BODY_LEFT ? ls[i] : fs[i])*run.travel_sign);
                run.encoder_ready|=(uint8_t)(1u<<i);
                if(run.encoder_ready==ALL_MOTORS_MASK) {
                    text("[encoder directions confirmed] signs=");
                    for(uint8_t wheel=0;wheel<MOTOR_COUNT;wheel++) {
                        if(wheel)send_byte(',');
                        signed_number(run.encoder_sign[wheel]);
                    }
                    text("\r\n");
                }
            }
        }
        /* 极性冲突只告警；仍保留按原符号计算出的实际位移，不自动停车。 */
        if (run.encoder_sign[i] && delta * run.encoder_sign[i] < -learn_counts) {
            run_warning("encoder direction conflict; no automatic stop");
        }
        sum += delta * run.encoder_sign[i];
    }
    run_straight_observe();
    if (run.encoder_ready != 15u) {
        return 1;
    }
    run.progress_mm = (int32_t)((float)sum * CAR_POSITION_PULSES_PER_REV
                          / (4.0f * CAR_ENCODER_COUNTS_PER_REV * ppm))
                        - run.encoder_offset_mm;
    if(run_path_hold_enabled() && run.lateral_pose.valid)
        run.progress_mm=(int32_t)lroundf((run.travel_axis==BODY_LEFT
            ? run.lateral_pose.left_mm : run.lateral_pose.forward_mm)*run.travel_sign);
    while (!run.obstacle_phase && run.continuation_mm && run.action_mm
           && run.progress_mm >= run.action_mm) {
        if (!run_pass_straight()) return 0;
    }
    return 1;
}

static int run_lateral_braking(void)
{
    return run.travel_axis==BODY_LEFT && !run.station_shift && !run.vector_move && !run.obstacle_phase;
}

/* 以指令减速度和延迟预留计算可停车速度；不是对真实制动距离的测量。 */
static int16_t run_forward_target(int32_t remaining)
{
    float mm_per_rpm_s, delay_rpm, braking_rpm;
    int32_t speed, max_rpm;
    uint32_t decel=run_lateral_braking() ? CAR_LATERAL_DECEL_RPM_PER_S : CAR_DRIVE_DECEL_RPM_PER_S;
    uint32_t lookahead=run_lateral_braking() ? CAR_LATERAL_BRAKE_LOOKAHEAD_MS
        : run_rough_arrival() ? CAR_ROUGH_BRAKE_LOOKAHEAD_MS : CAR_DRIVE_BRAKE_LOOKAHEAD_MS;
    int sign = run.travel_sign;
    if ((run.travel_axis == BODY_LEFT || run_path_hold_enabled()) && remaining < -CAR_DISTANCE_TOLERANCE_MM) {
        remaining = -remaining;sign = -sign;
    }
    if (remaining <= CAR_DISTANCE_TOLERANCE_MM) return 0;
    speed = remaining * CAR_DISTANCE_KP_X100 / 100;
    mm_per_rpm_s = CAR_POSITION_PULSES_PER_REV / (60.0f * run_single_axis_ppm());
    if(CAR_RUN_OMNI_ENABLED && !run.station_shift && !run.translation_only && !run.obstacle_phase && run.encoder_ready==ALL_MOTORS_MASK) {
        static const int8_t fs[4]={-1,1,1,-1},ls[4]={-1,-1,1,1};
        unsigned i;int32_t rpm=0;uint32_t age=0;
        for(i=0;i<4u;i++) {
            rpm+=motor_feedback[i].speed_rpm*(run.travel_axis==BODY_LEFT ? ls[i] : fs[i]);
            age+=clock_ms-motor_feedback[i].position_ms;
        }
        /* 只给道路制动参考补偿CPOS平均年龄，不外推到位判据，也不改工位微调。 */
        rpm=abs(rpm)/4;age/=4;
        if(age>CAR_DRIVE_BRAKE_LOOKAHEAD_MS)age=CAR_DRIVE_BRAKE_LOOKAHEAD_MS;
        if(rpm>CAR_ENCODER_LEARN_RPM)remaining-=(int32_t)(rpm*mm_per_rpm_s*age/1000.0f);
        if(remaining<=CAR_DISTANCE_TOLERANCE_MM)return 0;
        speed=remaining*CAR_DISTANCE_KP_X100/100;
    }
    delay_rpm = decel * (lookahead / 1000.0f);
    braking_rpm = sqrtf(delay_rpm * delay_rpm
        + 2.0f * decel * (remaining - CAR_DISTANCE_TOLERANCE_MM) / mm_per_rpm_s) - delay_rpm;
    if (speed > (int32_t)braking_rpm) speed = (int32_t)braking_rpm;
    {
        int32_t minimum=CAR_DISTANCE_MIN_RPM;
        if(run.station_shift && run.station_visual_shift)minimum=CAR_MOVEXY_MIN_RPM;
        else if(run.travel_axis==BODY_LEFT && run.encoder_ready==ALL_MOTORS_MASK
            && !run.station_shift && !run.obstacle_phase && !run.continuation_mm)minimum=CAR_LATERAL_SETTLE_MIN_RPM;
        if(speed<minimum)speed=minimum;
    }
    max_rpm = run.travel_axis == BODY_LEFT
        ? (run.lateral_rpm ? run.lateral_rpm : CAR_LATERAL_MAX_RPM)
        : (run.test_forward_rpm ? run.test_forward_rpm : CAR_RUN_RPM);
    if (run.obstacle_phase == 2 && max_rpm > CAR_OBSTACLE_RETURN_RPM) max_rpm = CAR_OBSTACLE_RETURN_RPM;
    if (speed > max_rpm) speed = max_rpm;
    if (run.station_shift) {
        float index_rpm = (run.job == 3u || run.job == 6u) ? STN_STORAGE_INDEX_RPM : STN_ROUGH_INDEX_RPM;
        if (speed > index_rpm) speed = index_rpm;
    }
    if (run.encoder_ready != 15u && speed > CAR_ENCODER_LEARN_RPM) speed = CAR_ENCODER_LEARN_RPM;
    return (int16_t)(speed * sign);
}

/* 末端角度和角速度都稳定才归零；保留PD阻尼，不使用定角转向的最低RPM下限。 */
static int16_t run_settle_heading_rpm(void)
{
    HeadingControl *c=&run.heading_control;
    int32_t effort,rpm;
    if(abs(c->error_cdeg)<=CAR_HEADING_TOLERANCE_CDEG
        && abs(c->rate_cdeg_s)<=CAR_HEADING_STABLE_RATE_CDEG_S)return 0;
    if(run.station_visual_shift) {
        effort=c->error_cdeg*CAR_DRIVE_HEADING_KP_X100-c->rate_cdeg_s*CAR_DRIVE_HEADING_KD_X100;
        /* 保留视觉抵消轮的亚1RPM静噪；角度超出到位窗时仍允许1RPM慢纠偏。 */
        if(abs(c->error_cdeg)<=CAR_HEADING_TOLERANCE_CDEG && effort>-10000 && effort<10000)return 0;
    } else effort=c->error_cdeg*CAR_HEADING_KP_X100-c->rate_cdeg_s*CAR_HEADING_KD_X100;
    rpm=effort>=0 ? (effort+5000)/10000 : -((-effort+5000)/10000);
    if(rpm>CAR_HEADING_DRIVE_TRIM_RPM)rpm=CAR_HEADING_DRIVE_TRIM_RPM;
    if(rpm< -CAR_HEADING_DRIVE_TRIM_RPM)rpm= -CAR_HEADING_DRIVE_TRIM_RPM;
    return (int16_t)rpm;
}

static int16_t run_settle_turn_slew(int16_t desired,uint8_t due)
{
    int32_t target=(int32_t)desired*1000,change;
    uint32_t elapsed;
    if(!due)return run.turn_cmd_rpm;
    if(!run.settle_turn_active) {
        run.settle_turn_mrpm=(int32_t)run.turn_cmd_rpm*1000;
        run.settle_turn_active=1;
    }
    elapsed=clock_ms-run.control_ms;
    if(elapsed>CAR_DRIVE_SLEW_MAX_DT_MS)elapsed=CAR_DRIVE_SLEW_MAX_DT_MS;
    change=(int32_t)(CAR_SETTLE_TURN_RPM_PER_S*elapsed);
    if(target>run.settle_turn_mrpm) {
        run.settle_turn_mrpm+=change;
        if(run.settle_turn_mrpm>target)run.settle_turn_mrpm=target;
    } else {
        run.settle_turn_mrpm-=change;
        if(run.settle_turn_mrpm<target)run.settle_turn_mrpm=target;
    }
    return (int16_t)(run.settle_turn_mrpm/1000);
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
    rate = target > run.forward_mrpm ? CAR_DRIVE_ACCEL_RPM_PER_S
        : run_lateral_braking() ? CAR_LATERAL_DECEL_RPM_PER_S : CAR_DRIVE_DECEL_RPM_PER_S;
    if(target>run.forward_mrpm && !run.station_shift && !run.translation_only && !run.obstacle_phase)
        rate=run.travel_axis==BODY_LEFT ? CAR_LATERAL_ACCEL_RPM_PER_S : CAR_ROUTE_ACCEL_RPM_PER_S;
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
    HeadingAcceleration acceleration;
    int16_t values[3];
    __disable_irq();acceleration=heading_sensor.acceleration;__enable_irq();
    values[0]=acceleration.x_mg;values[1]=acceleration.y_mg;values[2]=acceleration.z_mg;
    /* 先积累窗口再判断打印周期；不把重复缓存或过期数据当作新采样。 */
    if(acceleration.valid && (uint32_t)(clock_ms-acceleration.received_ms)<=CAR_IMU_MAX_AGE_MS
        && (!run.acceleration_window.seen || acceleration.sequence!=run.acceleration_window.sequence)) {
        run.acceleration_window.seen=1;run.acceleration_window.sequence=acceleration.sequence;
        if(!run.acceleration_window.samples)run.acceleration_window.first_ms=acceleration.received_ms;
        run.acceleration_window.last_ms=acceleration.received_ms;
        for(i=0;i<3;i++) {
            if(!run.acceleration_window.samples)run.acceleration_window.sum[i]=values[i];
            else run.acceleration_window.sum[i]+=values[i];
            if(!run.acceleration_window.samples || values[i]<run.acceleration_window.minimum[i])
                run.acceleration_window.minimum[i]=values[i];
            if(!run.acceleration_window.samples || values[i]>run.acceleration_window.maximum[i])
                run.acceleration_window.maximum[i]=values[i];
        }
        run.acceleration_window.samples++;
    }
    if ((uint32_t)(clock_ms - run.log_ms) < CAR_CONTROL_LOG_MS) return;
    run.log_ms = clock_ms;
    text("[control] phase="); number(run.velocity_phase);
    text(" target_yaw_cdeg="); signed_number(heading_wrap_cdeg((int32_t)run.heading_control.last_yaw_cdeg + run.heading_control.error_cdeg));
    text(" yaw_cdeg="); signed_number(run.heading_control.last_yaw_cdeg);
    text(" error_cdeg="); signed_number(run.heading_control.error_cdeg);
    text(" rate_cdeg_s="); signed_number(run.heading_control.rate_cdeg_s);
    text(" rate_source=");text(run.heading_control.rate_source ? "GYRO" : "YAW_DIFF");
    text(run.corner.active ? " path_s_mm=" : " distance_mm=");signed_number(run.progress_mm);
    text(run.corner.active ? " path_target_mm=" : " target_mm=");
    number(run.corner.active ? (uint32_t)lroundf(run.corner.total_mm) : run_target_mm());
    if(run.accel_extra.state) {
        text(" nominal_mm=");number(run.action_mm);
        text(" accel_extra_mm=");number(run.accel_extra.mm);
        text(" accel_extra_state=");
        text(run.accel_extra.state==ACCEL_EXTRA_READY ? "tracking"
            : run.accel_extra.state==ACCEL_EXTRA_FROZEN ? "frozen" : "unavailable");
    }
    if(run.vector_move) {
        text(" vector_target_mm=");signed_number(run.vector_target_mm[0]);text(",");signed_number(run.vector_target_mm[1]);
        text(" vector_progress_mm=");signed_number(run.vector_progress_mm[0]);text(",");signed_number(run.vector_progress_mm[1]);
    }
    if (run.travel_axis == BODY_LEFT) text(run.travel_sign < 0 ? " travel=RIGHT" : " travel=LEFT");
    else text(run.travel_sign < 0 ? " travel=BACKWARD" : " travel=FORWARD");
    text(" ack="); number(run.acks);
    text(" fresh_mask="); number(run.ready);
    if (run.continuation_mm) {text(" continue_mm=");number(run.continuation_mm);}
    if (run.velocity_phase == VELOCITY_FORWARD || run.velocity_phase == VELOCITY_STOP_FORWARD) {
        text(" t_ms="); number(clock_ms);
        text(" v_ref="); signed_number(run.forward_ref_rpm);
        text(" v_cmd="); signed_number(run.forward_cmd_rpm);
        text(" wheel_limit_rpm=");number(run_wheel_limit_rpm());
        if(run_path_pose_supported()) {
            HeadingDriveGains gains=run_drive_heading_gains();
            if(run.settle_turn_active) {gains.kp_x100=CAR_HEADING_KP_X100;gains.kd_x100=CAR_HEADING_KD_X100;}
            text(" heading_schedule=");number(run_drive_gain_schedule_enabled() && !run.settle_turn_active);
            text(" heading_pd_mode=");text(!gyro_drive_feedback_enabled ? "OFF" : run.settle_turn_active ? "SETTLE" : "DRIVE");
            text(" heading_speed_rpm=");number((uint32_t)(run.forward_mrpm/1000));
            text(" heading_kp_x100=");number(gains.kp_x100);
            text(" heading_kd_x100=");number(gains.kd_x100);
        }
        if(run.travel_axis == BODY_LEFT || run.vector_move) {
            text(" left_ref="); signed_number(run.left_ref_rpm);
            text(" left_cmd="); signed_number(run.left_cmd_rpm);
        }
        text(" turn_ref="); signed_number(run.turn_ref_rpm);
        text(" turn_cmd="); signed_number(run.turn_cmd_rpm);
        text(" applied_v=");signed_number((-run.wheel_rpm[0]+run.wheel_rpm[1]+run.wheel_rpm[2]-run.wheel_rpm[3])/4);
        text(" applied_left=");signed_number((-run.wheel_rpm[0]-run.wheel_rpm[1]+run.wheel_rpm[2]+run.wheel_rpm[3])/4);
        text(" applied_turn=");signed_number((CAR_TURN_CCW_DIR ? 1 : -1)*(run.wheel_rpm[0]+run.wheel_rpm[1]+run.wheel_rpm[2]+run.wheel_rpm[3])/4);
        if(run_straight_hold_enabled()) {
            text(" cross_axis=");text(run_path_hold_enabled()
                ? (run.travel_axis==BODY_LEFT ? "START_FORWARD" : "START_LEFT")
                : run.travel_axis==BODY_LEFT ? "FORWARD" : "LEFT");
            text(" cross_mm=");signed_number(run.straight_hold.lateral_mm);
            text(" cross_rpm=");signed_number(run.straight_hold.command_rpm);
            text(" cross_valid=");number(run.straight_hold.valid);
        }
        text(" control_dt_ms="); number(run.control_dt_ms);
        text(" wheel_cmd_rpm=");
        for (i = 0; i < MOTOR_COUNT; i++) {if (i) text(",");signed_number(run.wheel_rpm[i]);}
        if(run.vector_move) {
            text(" wheel_rev=");
            for(i=0;i<MOTOR_COUNT;i++){if(i)text(",");number(run.vector_wheel_reversals[i]);}
        }
        text(" wheel_rpm_raw=");
        for (i = 0; i < MOTOR_COUNT; i++) {
            uint32_t elapsed = clock_ms - motor_feedback[i].speed_ms;
            if (elapsed > age) age = elapsed;
            if (i) text(",");
            signed_number(motor_feedback[i].speed_rpm);
        }
        text(" speed_age_max_ms="); number(age);
        if(run_path_pose_supported()) {
            text(" path_hold=");number(run_path_hold_enabled());
            text(" pose_valid=");number(run.lateral_pose.valid);
            text(" pose_age_ms=");if(run.lateral_pose.valid)number(clock_ms-run.lateral_pose.sample_ms);else text("-1");
            text(" pose_forward_mm=");signed_number((int32_t)lroundf(run.lateral_pose.forward_mm));
            text(" pose_left_mm=");signed_number((int32_t)lroundf(run.lateral_pose.left_mm));
            text(" cross_rate_mm_s=");signed_number((int32_t)lroundf(run.lateral_pose.forward_rate_mm_s));
            text(" cross_d_rpm=");signed_number(-(int32_t)lroundf(run.lateral_pose.forward_rate_mm_s
                *(run.travel_axis==BODY_LEFT ? CAR_LATERAL_HOLD_KD_X100 : CAR_STRAIGHT_HOLD_KD_X100)/100.0f));
            text(" path_forward_cmd=");signed_number(run.travel_axis==BODY_LEFT ? run.straight_hold.command_rpm
                : (int16_t)(run.forward_mrpm/1000*run.profile_sign));
            text(" path_left_cmd=");signed_number(run.travel_axis==BODY_LEFT
                ? (int16_t)(run.forward_mrpm/1000*run.profile_sign) : run.straight_hold.command_rpm);
        }
    }
    text(" acc_fresh=");number(acceleration.valid && (uint32_t)(clock_ms-acceleration.received_ms)<=CAR_IMU_MAX_AGE_MS);
    text(" acc_age_ms=");if(acceleration.valid)number(clock_ms-acceleration.received_ms);else text("-1");
    text(" acc_mg=");
    if(acceleration.valid)for(i=0;i<3;i++) {if(i)text(",");signed_number(values[i]);}
    else text("NA");
    text(" acc_n=");number(run.acceleration_window.samples);
    if(run.acceleration_window.samples) {
        text(" acc_mean_mg=");
        for(i=0;i<3;i++) {
            if(i)text(",");
            signed_number(run.acceleration_window.sum[i]/(int32_t)run.acceleration_window.samples);
        }
        /* 首/末采样时间，X最小/最大，Y最小/最大，Z最小/最大；各轴极值未必同时出现。 */
        text(" acc_win=");number(run.acceleration_window.first_ms);text(",");number(run.acceleration_window.last_ms);
        for(i=0;i<3;i++) {
            text(",");signed_number(run.acceleration_window.minimum[i]);
            text(",");signed_number(run.acceleration_window.maximum[i]);
        }
        run.acceleration_window.samples=0;
    }
    {HeadingSample attitude=heading_snapshot();
     text(" tilt_cdeg=");signed_number(attitude.roll_cdeg);text(",");signed_number(attitude.pitch_cdeg);}
    imu_nine_axis_log();
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

#ifdef CHASSIS_OBSTACLE_TEST
#include "../diagnostics/ultrasonic-chassis/route_policy.h"

static int obstacle_replan(void)
{
    uint8_t i;
    if(!obstacle_plan_remaining(run.obstacle_node))return 0;
    run.obstacle_phase=0;run.road_path_count=0;run.station_align_pending=0;
    obstacle_hits=obstacle_shot_valid=0;
    text("[obstacle detour] ");
    for (i=0;i<RUN_NODE_COUNT;i++) {
        if (i) text("->");
        send_byte((uint8_t)('A'+RUN_NODES[i]));
    }
    text("; resuming from encoder-estimated node\r\n");
    if (!run_road()) {run_warning("detour first road failed");return 0;}
    return 1;
}
#endif

static void run_return_wait(void)
{
#ifdef CHASSIS_OBSTACLE_TEST
    if (!stop_pending && run.obstacle_phase==2u && obstacle_replan()) return;
#endif
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
    if (!run.return_index) {
#ifdef CHASSIS_OBSTACLE_TEST
        if (!run.obstacle_preflight) run.obstacle_phase=2;
#endif
        run_return_wait();return;
    }
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

typedef struct {float x,y,dx,dy,yaw_cdeg,yaw_per_mm;uint8_t straight;} CornerPoint;

static float corner_clip(float value,float low,float high)
{
    return value<low ? low : value>high ? high : value;
}

/* 局部x=起步车身左侧、y=起步车身后方；路径由连续直线和相切圆弧组成。 */
static CornerPoint run_corner_point(float s)
{
    CornerPoint p;
    unsigned i;
    memset(&p,0,sizeof(p));
    for(i=0;i<run.corner.span_count;i++) {
        CornerSpan *span=&run.corner.spans[i];
        float d=corner_clip(s-span->s,0,span->length);
        if(s>span->s+span->length && i+1u<run.corner.span_count)continue;
        p.yaw_cdeg=span->yaw;
        if(span->radius>0) {
            float u=d/span->length,a=span->angle+span->turn*d/span->radius;
            float u2=u*u,u3=u2*u;
            p.x=span->cx+span->radius*cosf(a);p.y=span->cy+span->radius*sinf(a);
            p.dx=-span->turn*sinf(a);p.dy=span->turn*cosf(a);
            /* 三次插值保持两端角速度为零，峰值为均值1.5倍，低于原五次的1.875倍。 */
            p.yaw_cdeg+=span->dyaw*(3*u2-2*u3);
            p.yaw_per_mm=span->dyaw*6*(u-u2)/span->length;
        } else {
            p.x=span->x+span->ux*d;p.y=span->y+span->uy*d;
            p.dx=span->ux;p.dy=span->uy;
            p.straight=d>=span->blend_length;
            if(span->blend_length>0) {
                float u=corner_clip(d/span->blend_length,0,1),u2=u*u,u3=u2*u;
                float shift=10*u3-15*u3*u+6*u3*u2;
                float slope=(30*u2-60*u3+30*u3*u)/span->blend_length;
                p.x+=span->blend_x*shift;p.y+=span->blend_y*shift;
                p.dx+=span->blend_x*slope;p.dy+=span->blend_y*slope;
            }
        }
        break;
    }
    return p;
}

static int run_corner_lateral_straight(CornerPoint point)
{
    float angle=point.yaw_cdeg*0.0001745329252f;
    return point.straight && fabsf(sinf(angle)*point.dx-cosf(angle)*point.dy)<=0.001f
        && fabsf(cosf(angle)*point.dx+sinf(angle)*point.dy)>0.001f;
}

/* 在纯轴向直线段，发令叠加标定补偿，编码器积分反向扣除同一补偿。 */
static void run_corner_apply_straight_trim(CornerPoint point,float trim_sign,float *forward,float *left)
{
    float angle,sine,cosine;
    if(!point.straight || run.corner.reversing || run.obstacle_phase)return;
    angle=point.yaw_cdeg*0.0001745329252f;
    sine=sinf(angle);cosine=cosf(angle);
    if(fabsf(cosine*point.dx+sine*point.dy)<=0.001f)
        *left+=trim_sign*car_straight_left_trim(*forward);
    else if(run_corner_lateral_straight(point)) {
        float pulses=*forward*car_road_ppm_y(*forward)+trim_sign*car_lateral_forward_pulses(*left);
        /* 路径前后轴使用道路标定；先按test的9.68生成补偿脉冲，再换回该轴单位。
         * 在脉冲域加减，保留位置修正过零时前进/后退标定不同的原语义。 */
        *forward=pulses/car_road_ppm_y(pulses);
    }
}

/* 连续路径按规划切线和车头识别横移，不因位置/航向反馈噪声跳档。
 * 同一直线上的逻辑道路合并计长，经过E或接近终点时不把长横移改成短档。 */
static uint16_t run_wheel_limit_rpm(void)
{
    if(run.corner.active) {
        CornerPoint p=run_corner_point(run.corner.s_mm);
        float angle=p.yaw_cdeg*0.0001745329252f;
        float lateral;
        unsigned first=0,last,i;uint32_t distance=0;
        while(first+1u<run.corner.leg_count && run.corner.s_mm>=run.corner.legs[first].end_s)first++;
        lateral=cosf(angle)*run.corner.legs[first].ux+sinf(angle)*run.corner.legs[first].uy;
        if(fabsf(lateral)<0.001f)return CAR_RUN_RPM;
        last=first;
        while(first && run.corner.legs[first-1u].ux*run.corner.legs[first].ux
            +run.corner.legs[first-1u].uy*run.corner.legs[first].uy>0.5f)first--;
        while(last+1u<run.corner.leg_count && run.corner.legs[last+1u].ux*run.corner.legs[last].ux
            +run.corner.legs[last+1u].uy*run.corner.legs[last].uy>0.5f)last++;
        for(i=first;i<=last;i++)distance+=(uint32_t)lroundf(run.corner.legs[i].length);
        return car_lateral_limit_rpm(distance);
    }
    if(run.test_forward_rpm)return run.test_forward_rpm;
    if(run.turn_only || run.turning || run.station_shift || run.obstacle_phase)return CAR_TURN_CLOSED_LOOP_RPM;
    return run.travel_axis==BODY_LEFT
        ? (run.lateral_rpm ? run.lateral_rpm : CAR_LATERAL_MAX_RPM) : CAR_RUN_RPM;
}

static void run_corner_yaw_record(HeadingSample sample)
{
    unsigned last=(run.corner.yaw_head+47u)%48u,index=run.corner.yaw_head;
    /* 启动时可能再次读到较旧的同一份IMU；禁止倒序时间形成跨越约49天的插值区间。 */
    if(!sample.valid || (run.corner.yaw_count
        && (int32_t)(sample.received_ms-run.corner.yaw_ms[last])<10))return;
    run.corner.yaw[index]=sample.yaw_cdeg;run.corner.yaw_ms[index]=sample.received_ms;
    run.corner.yaw_head=(uint8_t)((index+1u)%48u);
    if(run.corner.yaw_count<48u)run.corner.yaw_count++;
}

static int run_corner_yaw_at(uint32_t when,float *yaw)
{
    unsigned i,oldest=(run.corner.yaw_head+48u-run.corner.yaw_count)%48u;
    for(i=0;i+1u<run.corner.yaw_count;i++) {
        unsigned a=(oldest+i)%48u,b=(a+1u)%48u;
        uint32_t span=run.corner.yaw_ms[b]-run.corner.yaw_ms[a],offset=when-run.corner.yaw_ms[a];
        if(span && offset<=span) {
            *yaw=run.corner.yaw[a]+heading_wrap_cdeg((int32_t)run.corner.yaw[b]-run.corner.yaw[a])*(float)offset/span;
            return 1;
        }
    }
    return 0;
}

/* 先限制规划本身所需的轮速变化，避免入弯后只能靠统一斜坡牺牲转向或平移。
 * 恒定路径速度v下，轮速变化率为d(RPM/v)/ds*v^2；预留20%给位置/航向反馈。 */
static float run_corner_span_speed(CornerSpan *span)
{
    static const int8_t fs[4]={-1,1,1,-1},ls[4]={-1,-1,1,1};
    float previous[4]={0},step=span->length/32.0f;
    float speed=span->radius>0 ? (span->dyaw ? CAR_CORNER_SPEED_MM_S : CAR_CORNER_FIXED_YAW_SPEED_MM_S)
        *sqrtf(span->radius/CAR_CORNER_RADIUS_MM)
        : CAR_WHEEL_MAX_RPM*CAR_POSITION_PULSES_PER_REV/(60.0f*CALIBRATED_PPM_X);
    unsigned sample,wheel;
    for(sample=0;sample<=32u;sample++) {
        CornerPoint p=run_corner_point(span->s+step*sample);
        float a=p.yaw_cdeg*0.0001745329252f;
        float forward=sinf(a)*p.dx-cosf(a)*p.dy;
        float left=cosf(a)*p.dx+sinf(a)*p.dy;
        float f,l;
        run_corner_apply_straight_trim(p,1.0f,&forward,&left);
        f=forward*60.0f*car_road_ppm_y(forward)/CAR_POSITION_PULSES_PER_REV;
        l=left*60.0f*CALIBRATED_PPM_X/CAR_POSITION_PULSES_PER_REV;
        for(wheel=0;wheel<4u;wheel++) {
            float next=fs[wheel]*f+ls[wheel]*l+p.yaw_per_mm/CAR_CORNER_YAW_GAIN_X100;
            if(fabsf(next)*speed>CAR_WHEEL_MAX_RPM)speed=CAR_WHEEL_MAX_RPM/fabsf(next);
            if(sample) {
                float slope=fabsf(next-previous[wheel])/step;
                float rate=next*previous[wheel]>=0 && fabsf(next)>fabsf(previous[wheel])
                    ? (span->dyaw ? CAR_CORNER_WHEEL_ACCEL_RPM_PER_S
                        : run_corner_lateral_straight(p) ? CAR_LATERAL_ACCEL_RPM_PER_S : CAR_ROUTE_ACCEL_RPM_PER_S)
                    : (span->dyaw ? CAR_CORNER_WHEEL_DECEL_RPM_PER_S : CAR_DRIVE_DECEL_RPM_PER_S);
                if(slope*speed*speed>rate*0.8f)speed=sqrtf(rate*0.8f/slope);
            }
            previous[wheel]=next;
        }
    }
    return speed;
}

/* 只收集到下一取放作业点；首个B是扫码条件点，可在有码时穿过。 */
static int run_corner_plan(void)
{
#ifdef CHASSIS_OBSTACLE_TEST
    /* 首轮实车用直段归属回波，圆弧改走原地图的折线。 */
    return 0;
#else
    RoadPlan road;
    uint8_t edge=run.edge,part=run.segment,job=run.job,finished=0;
    unsigned i,last_arc=0,arcs=0;
    float x=0,y=0,total=0,yaw=0,yaw_delta,start_x=0,start_y=0;
    memset(&run.corner,0,sizeof(run.corner));
    run.corner.start_edge=edge;run.corner.start_segment=part;run.corner.start_job=job;
    run.corner.start_heading=run.corner.end_heading=run.heading;
    if(!edge && !part && CAR_RUN_CORNER_ENABLED && CAR_RUN_OMNI_ENABLED) {
        MotionParameters body;
        if(!road_plan(RUN_NODES[0],NODE_B,&road)
            || !road_motion_parameters(road.segments[0],run.heading,&body))return 0;
        start_x=body.axis==BODY_LEFT ? body.mm : 0;
        start_y=body.axis==BODY_FORWARD ? -body.mm : 0;
    }
    if(run.station_return_mm) {
        CornerLeg *leg=&run.corner.legs[run.corner.leg_count++];
        run.corner.prefix_mm=run.station_return_mm;
        leg->edge=edge;leg->segment=255u;leg->length=(float)abs(run.station_return_mm);
        leg->uy=run.station_return_mm>0 ? -1.0f : 1.0f;
        y=leg->uy*leg->length;leg->y=y; /* 此段终点就是原道路的2环基准。 */
    }
    while(run.corner.leg_count<8u && edge+1u<RUN_NODE_COUNT) {
        if(!run_plan_road(edge,&road) || (run.blocked_roads&(1u<<road.road)))return 0;
        for(;part<road.count;part++) {
            CornerLeg *leg;MotionParameters body,p=road.segments[part];
            if(run.corner.leg_count>=8u || !road_motion_parameters(p,run.heading,&body))return 0;
            leg=&run.corner.legs[run.corner.leg_count++];
            leg->edge=edge;leg->segment=part;leg->length=(float)abs(p.mm);
            leg->ux=body.axis==BODY_LEFT ? (body.mm>0 ? 1.0f : -1.0f) : 0;
            leg->uy=body.axis==BODY_FORWARD ? (body.mm>0 ? -1.0f : 1.0f) : 0;
            x+=leg->ux*leg->length;y+=leg->uy*leg->length;
            if(!edge && !part) {x+=start_x;y+=start_y;}
            leg->x=x;leg->y=y;
        }
        if(road.to==RUN_JOB_NODE(job)) {
            if(!job && road.to==NODE_B)job++;
            else {finished=1;break;}
        }
        edge++;part=0;
    }
    if(!finished || run.corner.leg_count<2u)return 0;
    (void)run_station_heading(road.to,&run.corner.end_heading);
    for(i=0;i+1u<run.corner.leg_count;i++) {
        CornerLeg *a=&run.corner.legs[i],*next=&run.corner.legs[i+1u];
        float dot=a->ux*next->ux+a->uy*next->uy;
        if(dot< -0.5f)return 0; /* 折返仍沿用停车路径；当前非作业路线没有此情形。 */
        if(dot<0.5f) {
            a->radius=CAR_CORNER_RADIUS_MM;
            if(a->radius>a->length*0.5f)a->radius=a->length*0.5f;
            if(a->radius>next->length*0.5f)a->radius=next->length*0.5f;
            arcs++;
        }
    }
    if(!arcs)return 0;
    x=y=0;
    for(i=0;i<run.corner.leg_count;i++) {
        CornerLeg *leg=&run.corner.legs[i];CornerSpan *span;
        float previous=i ? run.corner.legs[i-1u].radius : 0;
        float length=leg->length-previous-leg->radius;
        if(length>0.001f) {
            span=&run.corner.spans[run.corner.span_count++];
            span->x=x+leg->ux*previous;span->y=y+leg->uy*previous;
            span->ux=leg->ux;span->uy=leg->uy;span->s=total;span->length=length;total+=length;
        }
        if(leg->radius>0) {
            CornerLeg *next=&run.corner.legs[i+1u];
            span=&run.corner.spans[run.corner.span_count++];
            span->x=leg->x-leg->ux*leg->radius;span->y=leg->y-leg->uy*leg->radius;
            span->turn=leg->ux*next->uy-leg->uy*next->ux;span->radius=leg->radius;
            span->cx=span->x-span->turn*leg->uy*span->radius;
            span->cy=span->y+span->turn*leg->ux*span->radius;
            span->angle=atan2f(span->y-span->cy,span->x-span->cx);
            span->s=total;span->length=1.570796327f*span->radius;total+=span->length;
            run.corner.arc_mm+=span->length;last_arc=run.corner.span_count-1u;
        }
        leg->end_s=total;x=leg->x;y=leg->y;
    }
    yaw_delta=(float)heading_wrap_cdeg(CAR_MAP_YAW_SIGN*((int32_t)run.corner.end_heading-run.heading)*9000);
    for(i=0;i<run.corner.span_count;i++) {
        run.corner.spans[i].yaw=yaw;
        if(i==last_arc){run.corner.spans[i].dyaw=yaw_delta;yaw=yaw_delta;}
    }
    if(start_x || start_y) {
        CornerSpan *span=&run.corner.spans[0];
        span->blend_x=start_x;span->blend_y=start_y;
        span->blend_length=CAR_START_BLEND_MM<span->length ? CAR_START_BLEND_MM : span->length;
    }
    if(run.plan==1u && road.to==NODE_C) {
        CornerSpan *span=&run.corner.spans[run.corner.span_count-1u];
        CornerLeg *last=&run.corner.legs[run.corner.leg_count-1u];MotionParameters body;
        (void)road_motion_parameters(road.segments[1],run.heading,&body);
        span->blend_x=body.axis==BODY_LEFT ? body.mm : 0;
        span->blend_y=body.axis==BODY_FORWARD ? -body.mm : 0;
        span->blend_length=CAR_C_RETURN_BLEND_MM<span->length ? CAR_C_RETURN_BLEND_MM : span->length;
        last->x+=span->blend_x;last->y+=span->blend_y;x=last->x;y=last->y;
    }
    for(i=0;i<run.corner.span_count;i++)if(run.corner.spans[i].radius>0 || run.corner.spans[i].blend_length>0)
        run.corner.spans[i].speed_mm_s=run_corner_span_speed(&run.corner.spans[i]);
    run.corner.total_mm=total;run.corner.first_mm=x;run.corner.second_mm=y;
    return 1;
#endif
}

static void run_corner_begin(void)
{
    unsigned i;uint32_t age=0;
    HeadingSample sample=heading_snapshot();
    if(!CAR_RUN_CORNER_ENABLED || !CAR_RUN_OMNI_ENABLED
        || run.station_shift || run.turn_only || run.translation_only || run.obstacle_phase)return;
    if(!run_corner_plan()) {
        if(run.station_return_mm)run_fault("station return path planning failed");
        return;
    }
    run.corner.active=1;
    if(!run.edge && !run.segment)
        run.distance_mm+=(uint16_t)lroundf(hypotf(run.corner.spans[0].blend_x,run.corner.spans[0].blend_y));
    run.corner.origin_yaw_cdeg=heading_wrap_cdeg(run.yaw_zero_cdeg+CAR_MAP_YAW_SIGN*(int32_t)run.heading*9000);
    run.corner.heading_target=run.heading_control.target_cdeg;
    run.corner.command_ms=clock_ms;run.corner.integrated_yaw_cdeg=sample.yaw_cdeg;
    for(i=0;i<4u;i++) {
        run.corner.previous[i]=run.corner.latest[i]=run.corner.integrated[i]=run.encoder_start[i];
        run.corner.previous_ms[i]=run.corner.latest_ms[i]=motor_feedback[i].position_ms;
        if((uint32_t)(clock_ms-motor_feedback[i].position_ms)>age)age=clock_ms-motor_feedback[i].position_ms;
    }
    run.corner.integrated_ms=clock_ms-age;run.corner.projection_ms=run.corner.integrated_ms;
    run.corner.yaw[0]=sample.yaw_cdeg;run.corner.yaw_ms[0]=sample.received_ms;
    run.corner.yaw_head=run.corner.yaw_count=1;run.continuation_mm=0;
    text("[route corner begin] start_edge=");number(run.corner.start_edge);
    text(" legs=");number(run.corner.leg_count);text("; continuous forward/lateral, yaw only as required by destination\r\n");
}

/* 串行轮询先插值四轮CPOS到共同时间，再旋转增量；纯转动的轮增量在逆混控中抵消。
 * 沿用起步两份实际位移的极性学习，之后轮子换向不能重学极性或取绝对值。 */
static int run_corner_observe(void)
{
    static const int8_t fs[4]={-1,1,1,-1},ls[4]={-1,-1,1,1};
    unsigned i;uint32_t age=0,common;
    int64_t position[4];float yaw,forward=0,left=0,angle,delta;
    if(!run.corner.polarity_ready) {
        if(!run_velocity_distance())return 0;
        if(run.encoder_ready==ALL_MOTORS_MASK) {
            for(i=0;i<4u;i++)run.corner.polarity[i]=(int8_t)(run.encoder_sign[i]
                *(run.travel_axis==BODY_LEFT ? ls[i] : fs[i])*run.travel_sign);
            run.corner.polarity_ready=1;
        }
    }
    for(i=0;i<4u;i++) {
        MotorFeedback *f=&motor_feedback[i];
        if(f->position_ms!=run.corner.latest_ms[i]) {
            run.corner.previous[i]=run.corner.latest[i];run.corner.previous_ms[i]=run.corner.latest_ms[i];
            run.corner.latest[i]=f->position;run.corner.latest_ms[i]=f->position_ms;
        }
        if((uint32_t)(clock_ms-run.corner.latest_ms[i])>age)age=clock_ms-run.corner.latest_ms[i];
    }
    common=clock_ms-age;
    if(!run.corner.polarity_ready || !common || common==run.corner.integrated_ms
        || (uint32_t)(common-run.corner.integrated_ms)>0x7fffffffu
        || !run_corner_yaw_at(common,&yaw))return 1;
    for(i=0;i<4u;i++) {
        uint32_t span=run.corner.latest_ms[i]-run.corner.previous_ms[i],offset=common-run.corner.previous_ms[i];
        float counts;
        if(!span || offset>span || span>CAR_VELOCITY_FEEDBACK_MS)return 1;
        position[i]=run.corner.previous[i]+(run.corner.latest[i]-run.corner.previous[i])*offset/span;
        counts=(float)(position[i]-run.corner.integrated[i])*run.corner.polarity[i];
        forward+=counts*fs[i];left+=counts*ls[i];
    }
    forward*=CAR_POSITION_PULSES_PER_REV/(4.0f*CAR_ENCODER_COUNTS_PER_REV*car_road_ppm_y(forward));
    left*=CAR_POSITION_PULSES_PER_REV/(4.0f*CAR_ENCODER_COUNTS_PER_REV*CALIBRATED_PPM_X);
    run_corner_apply_straight_trim(run_corner_point(run.corner.measured_s_mm),-1.0f,&forward,&left);
    delta=heading_wrap_cdeg((int32_t)lroundf(yaw-run.corner.integrated_yaw_cdeg));
    angle=heading_wrap_cdeg((int32_t)lroundf(run.corner.integrated_yaw_cdeg+delta*0.5f)-run.corner.origin_yaw_cdeg)*0.0001745329252f;
    {
        float dx=sinf(angle)*forward+cosf(angle)*left,dy=-cosf(angle)*forward+sinf(angle)*left;
        float dt=(common-run.corner.integrated_ms)/1000.0f,weight=dt/(0.08f+dt);
        run.corner.vx_mm_s+=(dx/dt-run.corner.vx_mm_s)*weight;
        run.corner.vy_mm_s+=(dy/dt-run.corner.vy_mm_s)*weight;
        run.corner.x_mm+=dx;run.corner.y_mm+=dy;
    }
    memcpy(run.corner.integrated,position,sizeof(position));run.corner.integrated_ms=common;run.corner.integrated_yaw_cdeg=yaw;
    return 1; /* 圆角位置超出原500mm边界仍继续路径闭环，不按估计值中止任务。 */
}

static float run_corner_project(float x,float y)
{
    unsigned i;float best=1.0e30f,result=0;
    for(i=0;i<run.corner.span_count;i++) {
        CornerSpan *span=&run.corner.spans[i];CornerPoint point;
        float d,s,dx,dy,distance;
        if(span->radius>0) {
            float angle=atan2f(y-span->cy,x-span->cx)-span->angle;
            if(angle>3.141592654f)angle-=6.283185307f;
            if(angle< -3.141592654f)angle+=6.283185307f;
            d=corner_clip(span->turn*angle,0,1.570796327f)*span->radius;
        } else d=corner_clip((x-span->x)*span->ux+(y-span->y)*span->uy,0,span->length);
        s=span->s+d;point=run_corner_point(s);
        dx=point.x-x;dy=point.y-y;distance=dx*dx+dy*dy;
        if(distance<best){best=distance;result=s;}
    }
    return result;
}

/* 在圆弧出口推进逻辑路段；不把切角过程当作停车到过节点。 */
static void run_corner_advance(float s,CornerPoint point)
{
    while(run.corner.leg_index+1u<run.corner.leg_count
        && s>=run.corner.legs[run.corner.leg_index].end_s) {
        CornerLeg *next=&run.corner.legs[++run.corner.leg_index];RunLeg leg;
        if(run.station_return_mm) {
            run.station_return_mm=0;
            text("[station ring2 join] continuous velocity into departure road; no visual or stop\r\n");
        }
        if(next->edge!=run.edge) {
            uint8_t node=run.road.to;
            run.completed_roads|=(uint16_t)(1u<<run.road.road);
            if(!run.job && node==NODE_B) {
                run.job++;run.scan_turn_stage=station_task_ready() ? 0u : 1u;
                text(station_task_ready() ? "[scan station] B bypassed; task latched; continuous forward/lateral transition\r\n"
                     : "[scan station] B bypassed; no task yet; continue scanning during straight travel\r\n");
            }
            run.edge=next->edge;
            (void)run_plan_road(run.edge,&run.road);
        }
        run.segment=next->segment;
        if(fabsf(point.yaw_cdeg-heading_wrap_cdeg(CAR_MAP_YAW_SIGN*((int32_t)run.corner.end_heading-run.corner.start_heading)*9000))<0.5f)
            run.target_heading=run.corner.end_heading;
        (void)run_resolve_leg(&run.road,run.segment,run.edge,run.target_heading,run.completed_roads,&leg);
        run.travel_axis=leg.axis;run.travel_sign=leg.sign;run.action_mm=run.remaining_mm=leg.mm;
        run.steps++;run.distance_mm+=leg.mm;run.corner.passed=1;
        if(run.plan==1u && next->edge==RUN_NODE_COUNT-2u)
            run.distance_mm+=(uint16_t)lroundf(hypotf(run.corner.spans[run.corner.span_count-1u].blend_x,
                run.corner.spans[run.corner.span_count-1u].blend_y));
        text("[route corner pass] edge=");number(run.edge);text(" segment=");number(run.segment);
        text("; continuous velocity, no stop/realign\r\n");
    }
}

static int run_corner_rotating_span(void)
{
    unsigned i;
    if(!run.corner.active || run.corner.reversing || run.corner.visual_terminal)return 0;
    for(i=0;i<run.corner.span_count;i++) {
        CornerSpan *span=&run.corner.spans[i];
        if(span->dyaw && run.corner.s_mm>=span->s && run.corner.s_mm<span->s+span->length)return 1;
    }
    return 0;
}

/* 几何转向只剩最后5度时即可进入恢复段，不必等圆弧全走完；弧内规划上限仍保留。 */
static int run_corner_exit_pending(float *yaw_error)
{
    unsigned i;
    if(!CAR_CORNER_EXIT_BLEND_ENABLED || !run.corner.active || run.corner.reversing || run.corner.visual_terminal)return 0;
    for(i=0;i<run.corner.span_count;i++) {
        CornerSpan *span=&run.corner.spans[i];
        if(span->dyaw && run.corner.s_mm>=span->s) {
            float final_yaw=span->yaw+span->dyaw;
            if(run.corner.s_mm<span->s+span->length) {
                float u=(run.corner.s_mm-span->s)/span->length;
                float remaining=span->dyaw*(1-3*u*u+2*u*u*u);
                if(fabsf(remaining)>CAR_CORNER_EXIT_ALIGNED_CDEG)continue;
            }
            *yaw_error=(float)abs(heading_wrap_cdeg(run.corner.heading_target+(int32_t)lroundf(final_yaw)
                -run.heading_control.progress_cdeg));
            return *yaw_error<CAR_CORNER_EXIT_YAW_CDEG;
        }
    }
    return 0;
}

static float run_corner_exit_speed(float speed)
{
    float yaw_error=0;int pending=run_corner_exit_pending(&yaw_error);
    uint32_t elapsed=clock_ms-run.corner.path_ref_ms;
    if(elapsed>CAR_DRIVE_SLEW_MAX_DT_MS)elapsed=CAR_DRIVE_SLEW_MAX_DT_MS;
    if(pending && yaw_error>CAR_CORNER_EXIT_ALIGNED_CDEG && speed>run.corner.path_ref_mm_s) {
        float aligned=corner_clip((CAR_CORNER_EXIT_YAW_CDEG-yaw_error)
            /(CAR_CORNER_EXIT_YAW_CDEG-CAR_CORNER_EXIT_ALIGNED_CDEG),0,1);
        float rise=CAR_CORNER_EXIT_ACCEL_RPM_PER_S+(CAR_CORNER_EXIT_MAX_ACCEL_RPM_PER_S-CAR_CORNER_EXIT_ACCEL_RPM_PER_S)*aligned;
        float limit=run.corner.path_ref_mm_s+rise*elapsed*CAR_POSITION_PULSES_PER_REV/(60000.0f*CALIBRATED_PPM_X);
        if(speed>limit)speed=limit;
    }
    run.corner.path_ref_mm_s=speed;run.corner.path_ref_ms=clock_ms;
    return speed;
}

static void run_corner_slew(float desired[3],int16_t output[3])
{
    static const int8_t fs[4]={-1,1,1,-1},ls[4]={-1,-1,1,1};
    uint32_t elapsed=clock_ms-run.corner.command_ms;
    float maximum=0,scale=1,limit=run_wheel_limit_rpm(),seconds,accel;
    float decel=CAR_DRIVE_DECEL_RPM_PER_S;
    int lateral=(desired[0] || desired[1]) && !run.corner.reversing && !run.obstacle_phase
        && run_corner_lateral_straight(run_corner_point(run.corner.s_mm));
    unsigned i;
    if(elapsed>CAR_DRIVE_SLEW_MAX_DT_MS)elapsed=CAR_DRIVE_SLEW_MAX_DT_MS;
    if(run.corner.reversing && limit>CAR_OBSTACLE_RETURN_RPM)limit=CAR_OBSTACLE_RETURN_RPM;
    for(i=0;i<4u;i++) {
        float value=fabsf(fs[i]*desired[0]+ls[i]*desired[1]+desired[2]);
        if(value>maximum)maximum=value;
    }
    if(maximum>limit)for(i=0;i<3u;i++)desired[i]*=limit/maximum;
    seconds=elapsed/1000.0f;
    accel=run.corner.reversing ? CAR_DRIVE_DECEL_RPM_PER_S
        : lateral ? CAR_LATERAL_ACCEL_RPM_PER_S : CAR_ROUTE_ACCEL_RPM_PER_S;
    /* 提高弯内平移/旋转混控的变化率，不改变直线、退回和终点停车的制动计算。 */
    if(run_corner_rotating_span() && (desired[0] || desired[1])) {
        accel=CAR_CORNER_WHEEL_ACCEL_RPM_PER_S;
        decel=CAR_CORNER_WHEEL_DECEL_RPM_PER_S;
    } else if(!lateral && (desired[0] || desired[1])) {
        float yaw_error=0;
        if(run_corner_exit_pending(&yaw_error)) {
            float aligned=corner_clip((CAR_CORNER_EXIT_YAW_CDEG-yaw_error)
                /(CAR_CORNER_EXIT_YAW_CDEG-CAR_CORNER_EXIT_ALIGNED_CDEG),0,1);
            accel=CAR_CORNER_EXIT_ACCEL_RPM_PER_S
                +(CAR_CORNER_EXIT_MAX_ACCEL_RPM_PER_S-CAR_CORNER_EXIT_ACCEL_RPM_PER_S)*aligned;
        }
    }
    for(i=0;i<4u;i++) {
        float previous=fs[i]*run.corner.command[0]+ls[i]*run.corner.command[1]+run.corner.command[2];
        float next=fs[i]*desired[0]+ls[i]*desired[1]+desired[2];
        float change=fabsf(next-previous),step;
        /* 每轮先按当前路段减速率过零，再升速；三轴仍共同比例推进。 */
        if(previous*next<0) {
            float stop_seconds=fabsf(previous)/decel;
            step=seconds<=stop_seconds ? decel*seconds
                : fabsf(previous)+accel*(seconds-stop_seconds);
        } else step=(fabsf(next)>fabsf(previous) ? accel : decel)*seconds;
        if(change>step && step/change<scale)scale=step/change;
    }
    for(i=0;i<3u;i++) {
        run.corner.command[i]+=(desired[i]-run.corner.command[i])*scale;
        output[i]=(int16_t)lroundf(run.corner.command[i]);
    }
    run.corner.command_ms=clock_ms;
}

static void run_corner_update(void)
{
    CornerPoint point;
    float s,remaining,speed;
    float coefficient=60.0f*CALIBRATED_PPM_X/CAR_POSITION_PULSES_PER_REV;
    float decel=CAR_DRIVE_DECEL_RPM_PER_S/coefficient;
    float delay=decel*(run_rough_arrival() ? CAR_ROUGH_BRAKE_LOOKAHEAD_MS : CAR_DRIVE_BRAKE_LOOKAHEAD_MS)/1000.0f;
    float ex,ey,vx,vy,angle,desired[3],rate,effort,x,y,measured_s,age;
    int16_t command[3];int32_t target;
    unsigned i;uint8_t due,visual_end;
    if(!run_corner_observe()){run_fault("encoder corner direction learning failed");return;}
    if(run.corner.polarity_ready && (uint32_t)(clock_ms-run.corner.integrated_ms)>CAR_VELOCITY_FEEDBACK_MS) {
        /* 单轮恢复的第一份CPOS尚不能形成共同时间；先沿用原反馈等待，不能拿旧位置配新航向。 */
        run_wait_feedback("four-wheel position timestamps incomplete; awaiting coherent feedback");return;
    }
    measured_s=run_corner_project(run.corner.x_mm,run.corner.y_mm);
    measured_s=run_corner_project(run.corner.x_mm,run.corner.y_mm);
    age=(uint32_t)(clock_ms-run.corner.integrated_ms)/1000.0f;
    if(age>CAR_DRIVE_BRAKE_LOOKAHEAD_MS/1000.0f)age=CAR_DRIVE_BRAKE_LOOKAHEAD_MS/1000.0f;
    /* 本次只补偿固定车头长横移；同步转头仍沿用共同采样时刻的既有角度前馈。 */
    if(run.corner.start_heading!=run.corner.end_heading)age=0;
    x=run.corner.x_mm+run.corner.vx_mm_s*age;y=run.corner.y_mm+run.corner.vy_mm_s*age;
    s=run_corner_project(x,y);point=run_corner_point(s);
    if(run.corner.integrated_ms!=run.corner.projection_ms) {
        run.corner.speed_mm_s=(measured_s-run.corner.measured_s_mm)*1000.0f/(run.corner.integrated_ms-run.corner.projection_ms);
        run.corner.measured_s_mm=measured_s;
        run.corner.projection_ms=run.corner.integrated_ms;
    }
    run.corner.s_mm=s;
    target=run.corner.heading_target+(int32_t)lroundf(point.yaw_cdeg);
    if(target!=run.heading_control.target_cdeg)run.heading_control.stable_samples=0;
    run.heading_control.target_cdeg=target;run.heading_control.error_cdeg=target-run.heading_control.progress_cdeg;
    if(!run.corner.reversing)run_corner_advance(measured_s,point);
    visual_end=run.station_enabled && !run.corner.reversing && (run.corner.arriving || run_visual_arrival());
    if(!run.corner.visual_terminal && visual_end && measured_s>=run.corner.total_mm-CAR_DISTANCE_TOLERANCE_MM
        && hypotf(run.corner.first_mm-run.corner.x_mm,run.corner.second_mm-run.corner.y_mm)<=CAR_ROUTE_VISUAL_WINDOW_MM)
    {
        run.corner.visual_terminal=1;text("[route arrival] node=");send_byte((uint8_t)('A'+run.road.to));
        text(" residual_xy_mm=");signed_number((int32_t)(run.corner.first_mm-run.corner.x_mm));text(",");
        signed_number((int32_t)(run.corner.second_mm-run.corner.y_mm));text("; position to vision; zero/yaw feedback still required\r\n");
    }
    remaining=run.corner.reversing ? s : run.corner.total_mm-s;
    speed=sqrtf(delay*delay+2*decel*corner_clip(remaining-CAR_DISTANCE_TOLERANCE_MM,0,100000))-delay;
    speed=corner_clip(speed,0,run_wheel_limit_rpm()/coefficient);
    if(speed>remaining*CAR_CORNER_TRACK_GAIN_X100/100.0f)speed=remaining*CAR_CORNER_TRACK_GAIN_X100/100.0f;
    if(remaining>CAR_DISTANCE_TOLERANCE_MM && speed<CAR_CORNER_MIN_MM_S)speed=CAR_CORNER_MIN_MM_S;
    if(remaining<=CAR_DISTANCE_TOLERANCE_MM)speed=0;
    for(i=0;i<run.corner.span_count;i++)if(run.corner.spans[i].radius>0 || run.corner.spans[i].blend_length>0) {
        CornerSpan *span=&run.corner.spans[i];
        float arc_speed=span->speed_mm_s;
        float distance=run.corner.reversing ? s-span->s-span->length : span->s-s;
        if(distance>=0) {
            float entry=sqrtf(delay*delay+arc_speed*arc_speed+2*decel*distance)-delay;
            if(entry<arc_speed)entry=arc_speed;
            if(speed>entry)speed=entry;
        } else if(s>=span->s && s<=span->s+span->length && speed>arc_speed)speed=arc_speed;
    }
    if(!run.corner.polarity_ready && speed>CAR_ENCODER_LEARN_RPM/coefficient)speed=CAR_ENCODER_LEARN_RPM/coefficient;
    {
        float yaw_error=0;
        int slow_yaw=run_corner_exit_pending(&yaw_error) ? CAR_CORNER_EXIT_ALIGNED_CDEG : CAR_CORNER_SLOW_YAW_CDEG;
        if(abs(run.heading_control.error_cdeg)>slow_yaw)
            speed*=slow_yaw/(float)abs(run.heading_control.error_cdeg);
    }
    /* 只限制沿路径的升速，后面的横向位置反馈及转向纠偏保留原能力。 */
    speed=run_corner_exit_speed(speed);
    if(run.corner.reversing)speed=-corner_clip(speed,0,CAR_CORNER_RETURN_MM_S);
    ex=point.x-x;ey=point.y-y;
    if(remaining<=CAR_DISTANCE_TOLERANCE_MM) {
        point=run_corner_point(run.corner.reversing ? 0 : run.corner.total_mm);
        ex=point.x-x;ey=point.y-y;
    }
    vx=fabsf(ex)<=CAR_DISTANCE_TOLERANCE_MM ? 0 : corner_clip(ex*CAR_CORNER_TRACK_GAIN_X100/100.0f,-CAR_CORNER_TRACK_MAX_MM_S,CAR_CORNER_TRACK_MAX_MM_S);
    vy=fabsf(ey)<=CAR_DISTANCE_TOLERANCE_MM ? 0 : corner_clip(ey*CAR_CORNER_TRACK_GAIN_X100/100.0f,-CAR_CORNER_TRACK_MAX_MM_S,CAR_CORNER_TRACK_MAX_MM_S);
    if(vx && fabsf(vx)<CAR_CORNER_MIN_MM_S)vx=vx>0 ? CAR_CORNER_MIN_MM_S : -CAR_CORNER_MIN_MM_S;
    if(vy && fabsf(vy)<CAR_CORNER_MIN_MM_S)vy=vy>0 ? CAR_CORNER_MIN_MM_S : -CAR_CORNER_MIN_MM_S;
    vx+=point.dx*speed;vy+=point.dy*speed;
    if(run.plan==1u && run.corner.start_edge==RUN_NODE_COUNT-4u && !run.corner.reversing && vx>0) {
        /* C回程外侧分量单独预制动，前进可继续；四轮共用变化率，留一半能力给另一轴。 */
        float axis_decel=decel*0.5f,axis_delay=axis_decel*CAR_DRIVE_BRAKE_LOOKAHEAD_MS/1000.0f;
        float room=corner_clip(run.corner.first_mm-x,0,100000);
        float maximum=sqrtf(axis_delay*axis_delay+2*axis_decel*room)-axis_delay;
        if(vx>maximum)vx=maximum;
    }
    if(run.corner.visual_terminal && visual_end)vx=vy=0;
    angle=heading_wrap_cdeg((int32_t)run.heading_control.last_yaw_cdeg-run.corner.origin_yaw_cdeg)*0.0001745329252f;
    desired[0]=sinf(angle)*vx-cosf(angle)*vy;
    desired[1]=cosf(angle)*vx+sinf(angle)*vy;
    run_corner_apply_straight_trim(point,1.0f,&desired[0],&desired[1]);
    desired[0]*=60.0f*car_road_ppm_y(desired[0])/CAR_POSITION_PULSES_PER_REV;
    desired[1]*=coefficient;
    /* 航向目标由实测路径进度决定，其前馈导数也须用实测进度速度；
     * 减速时用较小的期望速度代替实际速度，会造成入弯转向滞后。 */
    rate=point.yaw_per_mm*run.corner.speed_mm_s;
    effort=(run.heading_control.error_cdeg*CAR_DRIVE_HEADING_KP_X100
        -(run.heading_control.rate_cdeg_s-rate)*CAR_DRIVE_HEADING_KD_X100)/10000.0f;
    desired[2]=corner_clip(rate/CAR_CORNER_YAW_GAIN_X100+effort,-CAR_CORNER_YAW_MAX_RPM,CAR_CORNER_YAW_MAX_RPM);
    if(!vx && !vy)desired[2]=heading_control_rpm(&run.heading_control,run.turn_rpm);
    if(!run.corner.reversing && run.corner.leg_index+1u==run.corner.leg_count && !run.corner.arriving
        && run_station_heading(run.road.to,&run.target_heading)
        && ((fabsf(run.corner.first_mm-run.corner.x_mm)<=STN_APPROACH_OVERLAP_MM
            && fabsf(run.corner.second_mm-run.corner.y_mm)<=STN_APPROACH_OVERLAP_MM)
            || (run.corner.visual_terminal && hypotf(run.corner.first_mm-run.corner.x_mm,
                run.corner.second_mm-run.corner.y_mm)<=CAR_ROUTE_VISUAL_WINDOW_MM))) {
        uint8_t low=1;
        for(i=0;i<4u;i++)if(abs(run.wheel_rpm[i])>STN_APPROACH_OVERLAP_RPM)low=0;
        if(low) {
            run.completed_roads|=(uint16_t)(1u<<run.road.road);run.edge++;run.segment=run.road.count;
            run.corner.arriving=1;run.station_align_pending=1;run.turning=1;run.velocity_phase=VELOCITY_TURN;
            text("[station alignment] node=");send_byte((uint8_t)('A'+run.road.to));
            text(" right_side=WORK_AREA target_heading_dec=");number(run.target_heading);
            text("; curved arrival checkpoint pending\r\n");
        }
    }
    due=(uint32_t)(clock_ms-run.control_ms)>=CAR_VELOCITY_CONTROL_MS && !run.velocity_waiting;
    if(due) {
        run_corner_slew(desired,command);run.control_dt_ms=clock_ms-run.control_ms;run.control_ms=clock_ms;
        if(!run_velocity_send(command[0],command[1],command[2])){run_fault("velocity batch submit failed");return;}
        run.forward_ref_rpm=(int16_t)lroundf(desired[0]);run.left_ref_rpm=(int16_t)lroundf(desired[1]);run.turn_ref_rpm=(int16_t)lroundf(desired[2]);
        run.forward_cmd_rpm=command[0];run.left_cmd_rpm=command[1];run.turn_cmd_rpm=command[2];
    }
    if(!vx && !vy && !run.wheel_rpm[0] && !run.wheel_rpm[1] && !run.wheel_rpm[2] && !run.wheel_rpm[3]
        && run_velocity_stopped() && run.heading_control.stable_samples>=CAR_HEADING_STABLE_SAMPLES
        && ((run.corner.visual_terminal && visual_end)
            || (fabsf(point.x-run.corner.x_mm)<=CAR_DISTANCE_TOLERANCE_MM && fabsf(point.y-run.corner.y_mm)<=CAR_DISTANCE_TOLERANCE_MM))
#if CAR_HEADING_STABLE_MS > 0
        && (uint32_t)(run.heading_control.sample_ms-run.heading_control.stable_ms)>=CAR_HEADING_STABLE_MS
#endif
        ) {
        run.corner.active=0;run.remaining_mm=0;run.turning=0;run.transition_ready=1;
        run.heading=run.target_heading=run.corner.reversing ? run.corner.start_heading : run.corner.end_heading;
        if(run.corner.reversing){run_return_wait();return;}
        if(run.corner.visual_terminal && !run.corner.arriving) {
            run.completed_roads|=(uint16_t)(1u<<run.road.road);run.edge++;run.segment=run.road.count;
            run.corner.arriving=1;
        }
        if(run.corner.arriving) {
            run.station_align_pending=0;
            text("[station right-side ready] node=");send_byte((uint8_t)('A'+run.road.to));
            text(" heading_dec=");number(run.heading);
            text(run.corner.visual_terminal ? "; navigation residual to vision; physical_pose=unconfirmed\r\n"
                : "; encoder/IMU curved path complete; physical_pose=unconfirmed\r\n");
            run_finish_node();
        } else run_advance_segment();
        return;
    }
    run.progress_mm=(int32_t)s;run_heading_window_sample();
    if((uint32_t)(clock_ms-run.log_ms)>=CAR_CONTROL_LOG_MS) {
        text("[route corner] s_mm=");signed_number((int32_t)s);
        text(" xy_mm=");signed_number((int32_t)run.corner.x_mm);text(",");signed_number((int32_t)run.corner.y_mm);
        text(" returning=");number(run.corner.reversing);text("\r\n");
    }
    run_velocity_log();run_velocity_query();
}

/* obstacle是外部触发入口；此工程没有自动检测障碍物的传感器。 */
static int run_obstacle_request(void)
{
    if (!run_active || !run.imu_mode || run.turn_only || run.translation_only) return 0;
    if (run.obstacle_phase) {run_fault("obstacle repeated during stop/return; hold position");return 1;}
    if(run.corner.active && run.corner.prefix_mm) {run_fault("obstacle during station work; no automatic reverse");return 1;}
    run.obstacle_preflight = run.state == RUN_PREFLIGHT;
    run.obstacle_leg_active = !run.turning
        && (run.velocity_phase == VELOCITY_FORWARD || run.velocity_phase == VELOCITY_STOP_FORWARD);
    run.obstacle_node = RUN_NODES[run.edge];
    if (!run.obstacle_preflight && run.road.count && !run.station_align_pending
        && run.edge + 1u < RUN_NODE_COUNT) {
        run.blocked_roads |= (uint16_t)(1u << run.road.road);
        run.completed_roads &= (uint16_t)~(1u << run.road.road);
    }
    if(run.corner.active) {
        unsigned part;RoadPlan road;
        run.obstacle_node=RUN_NODES[run.corner.start_edge];run.obstacle_leg_active=0;
        for(part=0;part<run.corner.leg_count;part++)if(run_plan_road(run.corner.legs[part].edge,&road))
            run.completed_roads&=(uint16_t)~(1u<<road.road);
        run.scan_turn_stage=0;
        memset(run.corner.command,0,sizeof(run.corner.command));run.corner.command_ms=clock_ms;
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

#ifdef CHASSIS_OBSTACLE_TEST
#include "../diagnostics/ultrasonic-chassis/observe_roads.h"
static int obstacle_sensor_for_leg(void)
{
    HeadingSample sample=heading_snapshot();
    if (stop_pending || !run_active || !run.road.count || (run.completed_roads&(1u<<run.road.road))
        || run.turning || run.station_align_pending
        || run.obstacle_phase || run.travel_sign<1 || run.state!=RUN_MOVING
        || run.velocity_phase!=VELOCITY_FORWARD
        || !heading_sample_fresh(&sample,clock_ms,CAR_IMU_MAX_AGE_MS)
        || abs(heading_wrap_cdeg(sample.yaw_cdeg-run.yaw_zero_cdeg
                    - CAR_MAP_YAW_SIGN*(int32_t)run.target_heading*9000))>500) return -1;
    return run.travel_axis==BODY_FORWARD ? 0 : 1;
}

int obstacle_io_range_enabled(void)
{
    return 1; /* 持续轮流采样；已走道路不判障，但仍可观测旁侧未知道路。 */
}

static void obstacle_log_range(uint8_t sensor)
{
    text(sensor ? "[range] LEFT mm=" : "[range] FRONT mm=");
    if (obstacle_ranges[sensor]<0) text("UNKNOWN");else number((uint32_t)obstacle_ranges[sensor]);
    text("\r\n");
}

static void obstacle_report_ranges(void)
{
    obstacle_log_range(0);obstacle_log_range(1);
    text("[obstacle] blocked_mask_dec=");number(run.blocked_roads);
    text(" observed_mask_dec=");number(obstacle_observed_roads);
    text("; watched=");
    switch (obstacle_sensor_for_leg()) {case 0: text("FRONT");break;case 1: text("LEFT");break;default: text("NONE");}
    text(obstacle_close_approach(&run.road) ? "; mode=CLOSE300" : "; mode=EARLY550");
    text("\r\n");
}

static void obstacle_report_last(void)
{
    if (!obstacle_last.valid) {text("[obstacle last] NONE since reset\r\n");return;}
    text("[obstacle last] road=");send_byte((uint8_t)('A'+obstacle_last.from));
    send_byte('-');send_byte((uint8_t)('A'+obstacle_last.to));
    text(obstacle_last.sensor ? " sensor=LEFT mm=" : " sensor=FRONT mm=");
    number((uint32_t)obstacle_last.mm);
    text(" progress_mm=");signed_number(obstacle_last.progress_mm);
    text(" remaining_mm=");signed_number(obstacle_last.remaining_mm);
    text(" segment=");number(obstacle_last.segment);
    text(" heading_dec=");number(obstacle_last.heading);
    text(" time_ms=");number(obstacle_last.time_ms);text("\r\n");
}

void obstacle_io_shot(uint8_t sensor)
{
    obstacle_capture_ray(sensor);
    obstacle_shot_valid=obstacle_sensor_for_leg()==sensor;
    obstacle_shot_edge=run.edge;obstacle_shot_segment=run.segment;
}

void obstacle_io_result(uint8_t sensor,int mm)
{
    static uint8_t hit_edge,hit_segment;
    static uint32_t last_hit_ms;
    int32_t remaining=(int32_t)run.action_mm-run.progress_mm;
    obstacle_ranges[sensor]=mm;
    if (obstacle_range_log) obstacle_log_range(sensor);
    obstacle_observe_result(mm);
    if (!obstacle_shot_valid || obstacle_shot_edge!=run.edge || obstacle_shot_segment!=run.segment
        || obstacle_sensor_for_leg()!=sensor) return;
    obstacle_shot_valid=0;
    if(obstacle_entry_samples<OBSTACLE_ENTRY_SAMPLES)++obstacle_entry_samples;
    if (hit_edge!=run.edge || hit_segment!=run.segment || clock_ms-last_hit_ms>350u) obstacle_hits=0;
    hit_edge=run.edge;hit_segment=run.segment;
    /* 只封当前路段；探头装在车体边缘，直接用剩余行程保留提前停车余量。 */
    if (mm<0 || (obstacle_close_approach(&run.road) ? mm>=(int)OBSTACLE_CLOSE_MM : mm>(int)OBSTACLE_EARLY_MM)
        || mm>remaining+OBSTACLE_EDGE_ALLOWANCE_MM) {
        obstacle_hits=0;return;
    }
    last_hit_ms=clock_ms;
    if (++obstacle_hits<OBSTACLE_REQUIRED_HITS) return;
    obstacle_hits=0;
    obstacle_last.valid=1;obstacle_last.from=run.road.from;obstacle_last.to=run.road.to;
    obstacle_last.sensor=sensor;obstacle_last.segment=run.segment;obstacle_last.heading=(uint8_t)run.target_heading;
    obstacle_last.mm=mm;obstacle_last.progress_mm=run.progress_mm;
    obstacle_last.remaining_mm=remaining;obstacle_last.time_ms=clock_ms;
    text("[obstacle confirmed] road=");send_byte((uint8_t)('A'+run.road.from));
    send_byte('-');send_byte((uint8_t)('A'+run.road.to));
    text(sensor ? " sensor=LEFT mm=" : " sensor=FRONT mm=");number((uint32_t)mm);text("\r\n");
    (void)run_obstacle_request();
}
#endif

/* 反馈不足只继续查询；保留最后指令，不自动发零速，也不伪造新反馈。 */
static void run_wait_feedback(const char *reason)
{
    run_warning(reason);
    run_velocity_query();
}

static HeadingAcceleration accel_extra_snapshot(void)
{
    HeadingAcceleration sample;
    __disable_irq();sample=heading_sensor.acceleration;__enable_irq();
    return sample;
}

/* 只给独立 test l/r 校零。拿不到静止样本时按原编码器距离执行。 */
static int accel_extra_baseline(HeadingSample attitude)
{
    HeadingAcceleration accel;
    unsigned i;
    if(!accel_extra_enabled || run.travel_axis!=BODY_LEFT || !run.translation_only
        || !run_path_pose_supported())return 1;
    if((uint32_t)(clock_ms-run.phase_ms)>=CAR_ACCEL_EXTRA_CAL_TIMEOUT_MS) {
        run.accel_extra.state=ACCEL_EXTRA_UNAVAILABLE;
        text("[accel extra] no stationary baseline; nominal encoder target\r\n");
        return 1;
    }
    for(i=0;i<MOTOR_COUNT;i++)if(abs(motor_feedback[i].speed_rpm)>CAR_VELOCITY_STOP_RPM) {
        run.accel_extra.samples=0;
        run.accel_extra.bias_x_mg=run.accel_extra.bias_y_mg=0;
        return 0;
    }
    accel=accel_extra_snapshot();
    if(!accel.valid || !heading_sample_fresh(&attitude,clock_ms,CAR_ACCEL_EXTRA_MAX_AGE_MS)
        || (int32_t)(accel.received_ms-run.phase_ms)<=0
        || (uint32_t)(clock_ms-accel.received_ms)>CAR_ACCEL_EXTRA_MAX_AGE_MS
        || (run.accel_extra.samples && run.accel_extra.sequence==accel.sequence))return 0;
    run.accel_extra.sequence=accel.sequence;
    run.accel_extra.sample_ms=accel.received_ms;
    run.accel_extra.bias_x_mg+=accel.x_mg;
    run.accel_extra.bias_y_mg+=accel.y_mg;
    if(++run.accel_extra.samples<CAR_ACCEL_EXTRA_CAL_SAMPLES)return 0;
    run.accel_extra.bias_x_mg/=run.accel_extra.samples;
    run.accel_extra.bias_y_mg/=run.accel_extra.samples;
    run.accel_extra.roll_cdeg=attitude.roll_cdeg;
    run.accel_extra.pitch_cdeg=attitude.pitch_cdeg;
    run.accel_extra.state=ACCEL_EXTRA_READY;
    return 1;
}

static void accel_extra_update(HeadingSample attitude)
{
    HeadingAcceleration accel;
    uint32_t elapsed;
    float dt,yaw,along_mg,reference_mm_s,deficit_accel,candidate;
    int32_t nominal_remaining=(int32_t)run.action_mm-run.progress_mm;
    if(run.accel_extra.state!=ACCEL_EXTRA_READY)return;
    /* 用原距离判断制动开始，避免追加量改变制动点后继续推动目标。 */
    if(run.velocity_phase!=VELOCITY_FORWARD || nominal_remaining<=CAR_SETTLE_DISTANCE_MM
        || abs(run_forward_target(nominal_remaining))<run.forward_mrpm/1000) {
        run.accel_extra.state=ACCEL_EXTRA_FROZEN;
        return;
    }
    accel=accel_extra_snapshot();
    elapsed=accel.received_ms-run.accel_extra.sample_ms;
    if(!accel.valid || (uint32_t)(clock_ms-accel.received_ms)>CAR_ACCEL_EXTRA_MAX_AGE_MS
        || !heading_sample_fresh(&attitude,clock_ms,CAR_ACCEL_EXTRA_MAX_AGE_MS)
        || elapsed>CAR_ACCEL_EXTRA_MAX_AGE_MS
        || abs((int)attitude.roll_cdeg-run.accel_extra.roll_cdeg)>CAR_ACCEL_EXTRA_TILT_CDEG
        || abs((int)attitude.pitch_cdeg-run.accel_extra.pitch_cdeg)>CAR_ACCEL_EXTRA_TILT_CDEG) {
        run.accel_extra.state=ACCEL_EXTRA_FROZEN;
        return;
    }
    if(!elapsed || accel.sequence==run.accel_extra.sequence)return;
    dt=elapsed/1000.0f;
    yaw=heading_wrap_cdeg(attitude.yaw_cdeg-run.lateral_pose.origin_yaw_cdeg)*0.0001745329252f;
    along_mg=run.travel_sign*(sinf(yaw)*(accel.x_mg-run.accel_extra.bias_x_mg)
        +cosf(yaw)*(accel.y_mg-run.accel_extra.bias_y_mg));
    reference_mm_s=run.profile_sign*run.travel_sign*run.forward_mrpm
        * (CAR_POSITION_PULSES_PER_REV/(60000.0f*run_single_axis_ppm()));
    deficit_accel=(reference_mm_s-run.accel_extra.reference_mm_s)/dt-along_mg*9.80665f;
    if(fabsf(deficit_accel)<CAR_ACCEL_EXTRA_NOISE_MG*9.80665f)deficit_accel=0;
    run.accel_extra.velocity_mm_s=expf(-(float)elapsed/CAR_ACCEL_EXTRA_V_LEAK_MS)
        *(run.accel_extra.velocity_mm_s+deficit_accel*dt);
    run.accel_extra.velocity_mm_s=fmaxf(-CAR_ACCEL_EXTRA_MAX_SPEED_MM_S,
        fminf(CAR_ACCEL_EXTRA_MAX_SPEED_MM_S,run.accel_extra.velocity_mm_s));
    run.accel_extra.deficit_mm=expf(-(float)elapsed/CAR_ACCEL_EXTRA_POS_LEAK_MS)
        *run.accel_extra.deficit_mm+run.accel_extra.velocity_mm_s*dt;
    run.accel_extra.reference_mm_s=reference_mm_s;
    run.accel_extra.sample_ms=accel.received_ms;
    run.accel_extra.sequence=accel.sequence;
    candidate=(run.accel_extra.deficit_mm-CAR_ACCEL_EXTRA_DEADBAND_MM)*CAR_ACCEL_EXTRA_GAIN;
    if(run.encoder_ready==ALL_MOTORS_MASK && candidate>run.accel_extra.mm) {
        run.accel_extra.mm=(uint16_t)fminf(candidate,CAR_ACCEL_EXTRA_LIMIT_MM);
        if(run.accel_extra.mm==CAR_ACCEL_EXTRA_LIMIT_MM)run.accel_extra.state=ACCEL_EXTRA_FROZEN;
    }
}

static void run_velocity_update(void)
{
    RunCanFrame frame;
    HeadingDecision decision;
    uint8_t i;
    uint8_t drive_heading, control_due, settling;
    int16_t forward = 0, left = 0, turn = 0;
    HeadingDriveGains drive_gains;
    int32_t settle_distance;
    HeadingSample sample = heading_snapshot();
    run_lateral_yaw_sample(sample);
    while (run_read_frame(&frame)) {
        uint8_t address = (uint8_t)(frame.id >> 8);
        if (!run_record_frame(&frame)) continue;
        if (frame.data[0] == MOTOR_VELOCITY_COMMAND) {
            if (frame.data[1] != MOTOR_POSITION_ACK) {run_fault("velocity command rejected");continue;}
            if (run.velocity_waiting && (uint32_t)(frame.received_ms - run.velocity_ms) < CAR_VELOCITY_ACK_MS) {
                run.acks |= (uint8_t)(1u << (address - 1u));
                if (run.acks == ALL_MOTORS_MASK) run.velocity_waiting = 0;
            }
        }
        if (frame.data[0] == MOTOR_STATUS_COMMAND
            && (!(frame.data[1] & MOTOR_FLAG_ENABLED) || (frame.data[1] & MOTOR_FLAG_PROTECTED))) {
            run_fault("motor disabled or protected");
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
    if(run.corner.active)run_corner_yaw_record(sample);
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
        if(run.corner.active && !run_corner_observe()){run_fault("obstacle stop encoder invalid");return;}
        if (run.obstacle_leg_active && !run_velocity_distance()) {run_fault("obstacle stop encoder invalid");return;}
        if (!run_velocity_stopped()) {run_velocity_query();return;}
        for (i = 0;i < MOTOR_COUNT;i++)
            if ((uint32_t)(motor_feedback[i].position_ms - run.stopped_ms) > 0x7fffffffu) {run_velocity_query();return;}
        if(run.corner.active) {
            run.corner.reversing=1;run.corner.arriving=0;run.corner.command_ms=clock_ms;
            run.obstacle_phase=2;run.state=RUN_MOVING;run.turning=0;
            run.station_align_pending=0;run.velocity_phase=VELOCITY_FORWARD;run.phase_ms=clock_ms;
            run.edge=run.corner.start_edge;run.segment=run.corner.start_segment;run.job=run.corner.start_job;
            if(!run_plan_road(run.edge,&run.road)){run_fault("corner return road missing");return;}
            text("[obstacle returning] curved path to ");send_byte((uint8_t)('A'+run.obstacle_node));
            text("; encoder/IMU; original speed limit retained\r\n");
            return;
        }
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
    decision = heading_update(&run.heading_control, sample);
    if (decision == HEADING_FAULT) {run_wait_feedback(run.heading_control.fault);return;}
    if ((uint32_t)(clock_ms - run.phase_ms) >= (run.turning ? CAR_HEADING_TIMEOUT_MS : CAR_DRIVE_TIMEOUT_MS)) {
        run_warning("motion deadline exceeded; feedback completion still required");
        run.phase_ms=clock_ms;
    }
    for (i = 0; i < MOTOR_COUNT; i++)
        if (run.wheel_rpm[i] && (uint32_t)(clock_ms - run.wheel_start_ms[i]) > CAR_VELOCITY_STALL_MS
            && (uint32_t)(clock_ms - motor_feedback[i].position_changed_ms) > CAR_VELOCITY_STALL_MS) {
            run_fault("wheel position did not follow velocity command");
        }
    if (run.velocity_phase == VELOCITY_TURN && (run.heading_control.target_cdeg > 4500 || run.heading_control.target_cdeg < -4500)
        && ((run.heading_control.target_cdeg > 0 && run.heading_control.progress_cdeg < -500)
            || (run.heading_control.target_cdeg < 0 && run.heading_control.progress_cdeg > 500))) {
        run_fault("IMU rotation direction mismatch");
    }
    if (run.velocity_phase == VELOCITY_BASELINE) {
        if (run.departure_overlap && !CAR_ARM_TRANSLATION_OVERLAP_ENABLED) {run_velocity_query();return;}
        for (i = 0; i < MOTOR_COUNT; i++)
            if ((uint32_t)(motor_feedback[i].position_ms - run.baseline_after_ms) > 0x7fffffffu) {run_velocity_query();return;}
        if(!accel_extra_baseline(sample)){run_velocity_query();return;}
        for (i = 0; i < MOTOR_COUNT; i++) run.encoder_start[i] = motor_feedback[i].position;
        memset(run.encoder_sign, 0, sizeof(run.encoder_sign));
        memset(run.encoder_previous_delta, 0, sizeof(run.encoder_previous_delta));
        for (i = 0; i < MOTOR_COUNT; i++) run.encoder_sample_ms[i] = motor_feedback[i].position_ms;
        run.encoder_ready = 0;
        memset(&run.straight_hold,0,sizeof(run.straight_hold));
        memset(&run.lateral_pose,0,sizeof(run.lateral_pose));
        run.lateral_pose.origin_yaw_cdeg=heading_wrap_cdeg(run.heading_control.last_yaw_cdeg+run.heading_control.error_cdeg);
        run.lateral_pose.last_yaw_cdeg=(float)heading_wrap_cdeg(sample.yaw_cdeg-run.lateral_pose.origin_yaw_cdeg);
        run.lateral_pose.sample_ms=sample.received_ms;
        run_lateral_yaw_sample(sample);
        for(i=0;i<MOTOR_COUNT;i++) {
            run.straight_hold.previous[i]=run.straight_hold.latest[i]=run.encoder_start[i];
            run.straight_hold.previous_ms[i]=run.straight_hold.latest_ms[i]=motor_feedback[i].position_ms;
        }
        run.straight_hold.sample_ms=clock_ms;
        memset(run.vector_progress_mm,0,sizeof(run.vector_progress_mm));
        memset(run.vector_mrpm,0,sizeof(run.vector_mrpm));
        memset(run.vector_profile_sign,0,sizeof(run.vector_profile_sign));
        memset(&run.vector_learn,0,sizeof(run.vector_learn));
        /* 物理极性跨段保留；单轴符号由本段混控推导，不能再按最初两份位移猜方向。 */
        for(i=0;i<MOTOR_COUNT;i++)if(run.wheel_polarity[i]) {
            static const int8_t fs[4]={-1,1,1,-1},ls[4]={-1,-1,1,1};
            run.encoder_sign[i]=run.wheel_polarity[i];
            if(!run.vector_move)run.encoder_sign[i]=(int8_t)(run.encoder_sign[i]
                *(run.travel_axis==BODY_LEFT ? ls[i] : fs[i])*run.travel_sign);
            run.encoder_ready|=(uint8_t)(1u<<i);
        }
        memset(run.vector_last_wheel_sign,0,sizeof(run.vector_last_wheel_sign));
        memset(run.vector_wheel_reversals,0,sizeof(run.vector_wheel_reversals));
        run.vector_profile_ms[0]=run.vector_profile_ms[1]=clock_ms;
        run.progress_mm = 0;run.action_mm = run.remaining_mm;run.encoder_offset_mm = 0;
        run.forward_mrpm = 0;run.forward_profile_ms = clock_ms;
        run.forward_ref_rpm = run.forward_cmd_rpm = run.turn_ref_rpm = run.turn_cmd_rpm = 0;
        run.left_ref_rpm = run.left_cmd_rpm = 0;
        run.control_dt_ms = 0;
        memset(&run.heading_window, 0, sizeof(run.heading_window));
        memset(&run.acceleration_window,0,sizeof(run.acceleration_window));
        run.velocity_phase = VELOCITY_FORWARD;run.state = RUN_MOVING;run.route_terminal=0;
        run.phase_ms = clock_ms;
        if(!run.station_return_mm){run.steps++;run.distance_mm += run.action_mm;}
#ifdef CHASSIS_OBSTACLE_TEST
        obstacle_entry_samples=obstacle_hits=obstacle_shot_valid=0;
#endif
        run_corner_begin();
        if(!run_active)return;
        run_status();
    }
    if(run.corner.active){run_corner_update();return;}
#ifdef CHASSIS_OBSTACLE_TEST
    /* 仅普通未知道路在起步前取两次结果；无回波仍为未知，随后按原限速尝试。 */
    if(!obstacle_close_approach(&run.road) && obstacle_sensor_for_leg()>=0
        && obstacle_entry_samples<OBSTACLE_ENTRY_SAMPLES){run_velocity_query();return;}
#endif
    if (run.velocity_phase == VELOCITY_FORWARD || run.velocity_phase == VELOCITY_STOP_FORWARD) {
        if (!run_velocity_distance()) {run_fault("encoder distance overrun/reversed");return;}
        if(run_path_hold_enabled() && run.encoder_ready==ALL_MOTORS_MASK
            && (!run.lateral_pose.valid || (uint32_t)(clock_ms-run.lateral_pose.sample_ms)>CAR_VELOCITY_FEEDBACK_MS)) {
            run_wait_feedback("path pose awaiting coherent encoder/Yaw samples");return;
        }
        accel_extra_update(sample);
        run_heading_window_sample();
        if(run_station_turn_early()) {
            if(!run_save_leg((uint16_t)run.progress_mm)){run_fault("road return path full");return;}
            text("[station turn overlap] position reached; turn before stop confirmation; residual_mm=");
            signed_number((int32_t)run.action_mm-run.progress_mm);text("\r\n");
            run.remaining_mm=0;run.transition_ready=0;run_advance_segment();
            run_velocity_query();return;
        }
        if (run.velocity_phase == VELOCITY_FORWARD) {
            int32_t remaining = (int32_t)run_target_mm() - run.progress_mm + run.continuation_mm;
            if (gyro_drive_feedback_enabled && (run.heading_control.error_cdeg > CAR_HEADING_MAX_DRIFT_CDEG
                || run.heading_control.error_cdeg < -CAR_HEADING_MAX_DRIFT_CDEG)) {
                run_fault("IMU excessive translation heading error");
            }
            if(run.vector_move) {
                forward=run_vector_target(0);left=run_vector_target(1);
                if(!forward && !left)run.velocity_phase=VELOCITY_STOP_FORWARD;
            } else if (run.travel_axis == BODY_LEFT) {
                if(!run.route_terminal && remaining<=CAR_DISTANCE_TOLERANCE_MM && abs(remaining)<=CAR_ROUTE_VISUAL_WINDOW_MM && run_visual_arrival()) {
                    run.route_terminal=1;text("[route arrival] node=");send_byte((uint8_t)('A'+run.road.to));
                    text(" residual_mm=");signed_number(remaining);text("; position to vision; zero/yaw feedback still required\r\n");
                }
                if (remaining <= CAR_DISTANCE_TOLERANCE_MM && (remaining >= -CAR_DISTANCE_TOLERANCE_MM || run.route_terminal))
                    run.velocity_phase = VELOCITY_STOP_FORWARD;
                else left = run_forward_target(remaining);
            } else if (remaining <= CAR_DISTANCE_TOLERANCE_MM
                && (!run_path_hold_enabled() || remaining>=-CAR_DISTANCE_TOLERANCE_MM)) run.velocity_phase = VELOCITY_STOP_FORWARD;
            else {
                forward = run_forward_target(remaining);
            }
        } else if(run.vector_move) {
            if(run_vector_target(0) || run_vector_target(1)) {
                run.velocity_phase=VELOCITY_FORWARD;run.stopped_ms=0;
            }
        } else if ((run.travel_axis == BODY_LEFT || run_path_hold_enabled()) && !run.route_terminal
                   && abs((int32_t)run_target_mm()-run.progress_mm)>CAR_DISTANCE_TOLERANCE_MM) {
            /* 减速/末端纠角产生的行程漂移必须重新收敛，不能在距离超差时报告完成。 */
            run.velocity_phase = VELOCITY_FORWARD;
            run.stopped_ms = 0;
        }
    }
    drive_heading = run.travel_axis == BODY_FORWARD
        && (run.velocity_phase == VELOCITY_FORWARD || run.velocity_phase == VELOCITY_STOP_FORWARD);
    control_due = (uint32_t)(clock_ms - run.control_ms) >= CAR_VELOCITY_CONTROL_MS && !run.velocity_waiting;
    run.left_ref_rpm = left;
    if(run.vector_move && (run.velocity_phase==VELOCITY_FORWARD || run.velocity_phase==VELOCITY_STOP_FORWARD)) {
        run.forward_ref_rpm=forward;
        forward=run_vector_slew(0,forward,control_due);left=run_vector_slew(1,left,control_due);
    }
    if (run.travel_axis == BODY_LEFT
        && (run.velocity_phase == VELOCITY_FORWARD || run.velocity_phase == VELOCITY_STOP_FORWARD))
        left = control_due ? run_forward_slew(left) : (int16_t)(run.forward_mrpm / 1000 * run.profile_sign);
    if(drive_heading && !run.vector_move) {
        run.forward_ref_rpm = forward;
        forward = control_due ? run_forward_slew(forward) : (int16_t)(run.forward_mrpm / 1000 * run.profile_sign);
    }
    settle_distance=abs((int32_t)run_target_mm()-run.progress_mm);
    if(run.vector_move) {
        int32_t other=abs(run.vector_target_mm[1]-run.vector_progress_mm[1]);
        settle_distance=abs(run.vector_target_mm[0]-run.vector_progress_mm[0]);
        if(other>settle_distance)settle_distance=other;
    }
    settling=((!run.station_shift && !run.vector_move) || (run.station_shift && run.station_visual_shift))
        && !run.obstacle_phase && !run.continuation_mm
        && run.encoder_ready==ALL_MOTORS_MASK
        && (run.velocity_phase==VELOCITY_FORWARD || run.velocity_phase==VELOCITY_STOP_FORWARD)
        && (run.settle_turn_active || settle_distance<=CAR_SETTLE_DISTANCE_MM);
    drive_gains=run_drive_heading_gains();
    if(settling)turn=run_settle_heading_rpm();
    else if (drive_heading) {
        /* 行驶时连续阻尼；前进指令归零后，沿用已验证的停车对齐/停稳判据。 */
        turn = (forward || (run.vector_move && left)) ? heading_control_drive_rpm_with_gains(&run.heading_control, CAR_HEADING_DRIVE_TRIM_RPM,drive_gains)
                       : heading_control_rpm(&run.heading_control, run.turn_rpm);
    } else if (run.travel_axis == BODY_LEFT && left
        && (run.velocity_phase == VELOCITY_FORWARD || run.velocity_phase == VELOCITY_STOP_FORWARD)) {
        /* 横移仍在运动时保留角速度阻尼；归零后沿用原停车对齐逻辑。 */
        turn = heading_control_drive_rpm_with_gains(&run.heading_control, CAR_HEADING_DRIVE_TRIM_RPM,drive_gains);
    } else turn = heading_control_rpm(&run.heading_control, run.turn_rpm);
    if(!gyro_drive_feedback_enabled
        && (run.velocity_phase==VELOCITY_FORWARD || run.velocity_phase==VELOCITY_STOP_FORWARD)) {
        turn=0;settling=0;run.settle_turn_active=0;run.settle_turn_mrpm=0;
    }
    run.turn_ref_rpm = turn;
    if(settling)turn=run_settle_turn_slew(turn,control_due);
    else run.settle_turn_active=0;
    if(!settling && run.station_shift && run.vector_move && (forward || left)
        && abs(forward)<=CAR_MOVEXY_FINE_MAX_RPM && abs(left)<=CAR_MOVEXY_FINE_MAX_RPM) {
        int32_t effort=run.heading_control.error_cdeg*CAR_DRIVE_HEADING_KP_X100
            -run.heading_control.rate_cdeg_s*CAR_DRIVE_HEADING_KD_X100;
        /* 两轴抵消的轮子会被最小±1RPM反复换向；微调时不足1RPM的PD量保留为零。
         * 有效阻尼仍按原PD输出，平移归零后继续使用原停车对齐函数。 */
        if(effort>-10000 && effort<10000)turn=0;
    }
    if ((forward || left) && !settling) {
        int16_t primary = run.vector_move ? (abs(left)>abs(forward) ? left : forward) : left ? left : forward;
        int16_t limit = (int16_t)((primary < 0 ? -primary : primary) - 1);
        if (limit > CAR_HEADING_DRIVE_TRIM_RPM) limit = CAR_HEADING_DRIVE_TRIM_RPM;
        if (turn > limit) turn = limit;
        if (turn < -limit) turn = -limit;
    }
    if(run_straight_hold_enabled() && drive_heading && !run_path_hold_enabled()) {
        int16_t available;
        left=run_straight_correction(forward,turn,control_due);
        /* 有横向修正时优先保留横移/航向余量，避免单轮限幅稀释纠偏。 */
        available=(int16_t)(run_wheel_limit_rpm()-abs(left)-abs(turn));
        if(left && abs(forward)>available)forward=forward>0 ? available : -available;
    }
    if (run_path_hold_enabled()
        && (run.velocity_phase==VELOCITY_FORWARD || run.velocity_phase==VELOCITY_STOP_FORWARD)) {
        float angle=heading_wrap_cdeg(run.heading_control.last_yaw_cdeg-run.lateral_pose.origin_yaw_cdeg)*0.0001745329252f;
        float c=cosf(angle),s=sinf(angle),fixed_forward=forward,fixed_left=left;
        float forward_ppm=run.travel_axis==BODY_LEFT ? CALIBRATED_PPM_Y : car_road_ppm_y(run.travel_sign);
        float lateral_ppm=run_lateral_ppm();
        if(run.travel_axis==BODY_LEFT)fixed_forward=run_straight_correction(left,turn,control_due);
        else fixed_left=run_straight_correction(forward,turn,control_due);
        forward=(int16_t)lroundf(c*fixed_forward+s*fixed_left*forward_ppm/lateral_ppm);
        left=(int16_t)lroundf(-s*fixed_forward*lateral_ppm/forward_ppm+c*fixed_left);
    } else if (left && !run.vector_move && run.travel_axis==BODY_LEFT) {
        /* 沿用前移mm/米的经验补偿，换算为轮速；此项不是位置观测。 */
        float trim=car_lateral_forward_pulses((float)left*CAR_POSITION_PULSES_PER_REV/CALIBRATED_PPM_X)
            /CAR_POSITION_PULSES_PER_REV;
        forward = run_straight_hold_enabled() ? run_straight_correction(left,turn,control_due)
            : (int16_t)(trim >= 0 ? trim + 0.5f : trim - 0.5f);
    }
    if (run.travel_axis == BODY_LEFT) run.forward_ref_rpm = forward;
    if (control_due) {
        run.control_dt_ms = clock_ms - run.control_ms;
        run.control_ms = clock_ms;
        if (!run_velocity_send(forward, left, turn)) {run_fault("velocity batch submit failed");return;}
        run.forward_cmd_rpm = forward;run.left_cmd_rpm = left;run.turn_cmd_rpm = turn;
    }
    if (!forward && !left && !turn && run_velocity_stopped()
        && (!run_path_hold_enabled() || run.velocity_phase==VELOCITY_TURN || (run.lateral_pose.valid
            && (int32_t)(run.lateral_pose.sample_ms-run.zero_ms)>0
            && abs(run.straight_hold.lateral_mm)<=CAR_STRAIGHT_HOLD_DEADBAND_MM
            && abs((int32_t)run_target_mm()-run.progress_mm)<=CAR_DISTANCE_TOLERANCE_MM))
        && ((!gyro_drive_feedback_enabled && run.velocity_phase!=VELOCITY_TURN)
            || run.heading_control.stable_samples >= CAR_HEADING_STABLE_SAMPLES)
#if CAR_HEADING_STABLE_MS > 0
        && ((!gyro_drive_feedback_enabled && run.velocity_phase!=VELOCITY_TURN)
            || (uint32_t)(run.heading_control.sample_ms-run.heading_control.stable_ms)>=CAR_HEADING_STABLE_MS)
#endif
        ) {
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
                send_byte((uint8_t)('A' + RUN_NODES[run.edge]));
                text(" heading_dec="); number(run.heading);
                text("; by_IMU_and_four_wheel_stop; physical_pose=unconfirmed\r\n");
                run_finish_node();
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
    }
#else
    if (CAN1->ESR & CAN_BUS_OFF_FLAG)
    {
        run_fault("CAN bus off");
    }
#endif
    if (can_rx_fault) {
        run_fault((can_rx_fault & 2u) ? "CAN transmit queue full; command not queued"
                                    : "CAN hardware FIFO overrun; feedback lost");
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
#if !defined(NAV_ENTRY_HOST_TEST) && !defined(CHASSIS_OBSTACLE_TEST)
    DebugRadioStats radio;
    debug_radio_stats(&radio);
    text("[radio] U6 TX=PB0 RX=PB1 baud=" CAR_STRING_VALUE(CAR_DEBUG_UART_BAUD) " 8N1 software UART rx=");
    number(radio.received);text(" framing_errors=");number(radio.framing_errors);
    text(" false_starts=");number(radio.false_starts);
    text(" tx_dropped=");number(radio.tx_dropped);text("\r\n");
#endif
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
    text("SCAN: scan size BASE_LENGTH WIDTH MARGIN (mm, rear platform +30) | scan A/C (head toward B, end heading -X) | scan status | scan stop; diagnostics only\r\n");
    text("VISION: vision debug [D|E|F 1..4] = standalone correction, no grip/route; vision debug off = stop\r\n");
    text("ARM: arm on (wait four-wheel stop) | arm status | arm pose SLIDE LIFT YAW CLAW\r\n");
    text("ARM: arm slide/lift/yaw N | arm open/close | arm pick/place/cycle | arm home\r\n");
    text("ARM: arm platform pose DEG | arm platform 1/2/3 | arm stop | arm off (park first)\r\n");
    text("ARM: arm load1 | arm load3 (or @) = calibrated loading sequences\r\n");
    text("STATION: task 134+123+314+231 | station | station auto on/off (default on)\r\n");
    text("LOOP: loop status | loop encoder on/off | loop gyro on/off | loop station on/off; station uses start reference\r\n");
    text("VISION: vision status/h/j/ping/range/pick N/clear; soft I2C 0x42 PG15 SCL / PD7 SDA mailbox32\r\n");
    text("screen main/chassis/arm/sensors/refresh = read-only USART6 display\r\n");
    text("qr = scanner status | qr init | qr baud 9600/115200 (MCU only)\r\n");
    text("raw region left/middle/right XMIN XMAX YMIN YMAX | raw region status | raw observe (retry completed J only)\r\n");
    text("runimu A/C = segmented route + switchable driving feedback + encoder distance; imu / feedback = diagnostics\r\n");
    text("PC9 START = release after short press: C; hold >=");number(CAR_START_BUTTON_LONG_PRESS_MS);
    text("ms then release: A; confirmed home/open claw/platform1; ! cancels\r\n");
    text("turn = IMU closed-loop rotation test | rpm 20..");number(CAR_TURN_CLOSED_LOOP_RPM);text("\r\n");
    text("TURN: left90 / right90 / left180 / right180 = IMU relative angle closed-loop\r\n");
    text("runimu obstacle = stop, return by encoder to last tracked node, then wait; no automatic detector\r\n");
    text("test = single-segment calibration | nav = return to navigation (stops)\r\n");
    text("TEST: dist 50..2000 mm | rpm 20..");number(CAR_TRANSLATION_MAX_RPM);text(" | f/b/l/r (body directions)\r\n");
    text("TEST f/b/l/r: switchable gyro feedback + encoder distance; fresh IMU required\r\n");
    text("TEST l/r: lateral distance tiers <=");number(CAR_LATERAL_SHORT_MM);text("mm:");number(CAR_LATERAL_SHORT_RPM);
    text("RPM, longer:");number(CAR_LATERAL_LONG_RPM);text("RPM\r\n");
    text("Inspect actual stop and reset physical start before the next direction key.\r\n");
    text("pose A | heading 0..3 (+Y,-X,-Y,+X) | go H | rpm 20..");number(CAR_TRANSLATION_MAX_RPM);text(" (0=road table)\r\n");
    text("confirm (segment) | node B (actual arrival) | obstacle | return A\r\n");
    text("block 0..11 | unblock 0..11 | progress measured_mm | info | help\r\n");
    text("! = immediate stop request (no Enter); s/z stop after Enter\r\n");
    text("MANUAL TEST: wait for DONE_BY_IMU_ENCODER and measure the physical path. No automatic return.\r\n");
}

/* f/b/l/r 只启动一个车身方向的标定段。 */
static int start_test(char direction)
{
    BodyAxis axis = (direction=='f' || direction=='b') ? BODY_FORWARD : BODY_LEFT;
    int32_t mm = (direction=='b' || direction=='r') ? -(int32_t)test_distance : test_distance;
    return start_heading_translation(axis,mm,test_rpm);
}

static int command_turn_test(const char *command_text)
{
    uint16_t value;
    int accepted = 0;
    if (drive.state == MOTION_RUNNING || stop_pending)
        return 0;
    if (!strncmp(command_text, "rpm ", 4) && parse_uint(command_text + 4, &value)
        && value >= 20 && value <= CAR_TURN_CLOSED_LOOP_RPM)
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
        && value >= 20 && value <= CAR_TRANSLATION_MAX_RPM
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
        else if(station_hold.active)snapshot->value[HMI_LF_T+i*2u]=(CAR_TURN_CCW_DIR ? 1 : -1)*station_hold.command_rpm;
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
    if(station_hold.active)snapshot->value[HMI_YAW_T]=station_hold.target_cdeg/10;
    else if(run_active || run.state==RUN_STATION)
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

#ifndef CHASSIS_OBSTACLE_TEST
static int drive_loop_command(const char *s)
{
    if(strncmp(s,"loop",4) || (s[4] && s[4]!=' '))return 0;
    if(!strcmp(s,"loop encoder on")) {
        if(!CAR_STRAIGHT_HOLD_ENABLED) {
            text("[loop rejected: CAR_STRAIGHT_HOLD_ENABLED=0 disables encoder drift controller]\r\n");return 1;
        }
        encoder_drift_feedback_enabled=1;
    } else if(!strcmp(s,"loop encoder off"))encoder_drift_feedback_enabled=0;
    else if(!strcmp(s,"loop gyro on"))gyro_drive_feedback_enabled=1;
    else if(!strcmp(s,"loop forward on"))lateral_hold_feedback_enabled=1;
    else if(!strcmp(s,"loop forward off"))lateral_hold_feedback_enabled=0;
    else if(!strcmp(s,"loop gyro off")) {
        gyro_drive_feedback_enabled=0;run.settle_turn_active=0;run.settle_turn_mrpm=0;
    } else if(!strcmp(s,"loop station on"))station_hold_enabled=1;
    else if(!strcmp(s,"loop station off")) {
        station_hold_enabled=0;if(station_hold.active)station_hold.releasing=1;
    } else if(strcmp(s,"loop") && strcmp(s,"loop status")) {
        text("[loop rejected: use loop status | loop encoder on/off | loop gyro on/off | loop forward on/off | loop station on/off]\r\n");return 1;
    }
    text("[loop] encoder=");text(encoder_drift_feedback_enabled && CAR_STRAIGHT_HOLD_ENABLED ? "ON" : "OFF");
    text(" gyro=");text(gyro_drive_feedback_enabled ? "ON" : "OFF");
    text(" forward=");text(lateral_hold_feedback_enabled ? "ON" : "OFF");
    text(" station=DISABLED_IN_DIAGNOSTIC");
    text("; station uses start reference; fixed trims, encoder distance, fresh IMU and angle turns retained\r\n");
    return 1;
}
#endif

/* 先处理公共命令和模式切换，再交给当前模式；只裁剪首尾空白。 */
static uint32_t diagnostic_imu_baud=CAR_IMU_UART_BAUD;
static void diagnostic_init(void)
{
    diagnostic_imu_baud=CAR_IMU_UART_BAUD;
    test_mode=MODE_TRANSLATION_TEST;
    arm_fixed_hold_init();
}
static void diagnostic_help(void)
{
    text("== JY901B CHASSIS ONLY; USART2 PD6 RX, default115200; U6 PB0/PB1 debug115200 ==\r\n");
    text("test | dist 50..2000 | rpm 20..250 | f/b/l/r = one segment, no automatic next action\r\n");
    text("turn | rpm 20..300 | left90/right90/left180/right180\r\n");
    text("loop status | loop gyro on/off | loop encoder on/off | loop forward on/off | imu | feedback | info | !\r\n");
    text("imutest extra 0/1 = lateral distance experiment, idle only, default OFF; imutest extra = status\r\n");
    text("imu baud 9600/19200/38400/57600/115200/230400 = MCU RX only, idle required\r\n");
    text("arm fixed: manually lift135 slide0; yaw0 claw45 platform20; fixed PWM and axes enabled, NO axis motion\r\n");
    text("NO route, PC9 start, arm actions, camera or scanner; verify physical pose BEFORE power-on\r\n");
}
static void diagnostic_set_baud(uint32_t baud)
{
    diagnostic_imu_baud=baud;
#ifndef NAV_ENTRY_HOST_TEST
    {extern void Usart2Init(unsigned int value);Usart2Init(baud);}
#endif
    heading_sensor_reset(&heading_sensor);imu_uart_fault=0;
    text("[imu baud] MCU_RX=");number(baud);text("; sensor unchanged; waiting for new WIT55/53 frames\r\n");
}
static void command(char *s)
{
    int accepted=0;
    while(*s==' ' || *s=='\t')s++;
    {char *end=s+strlen(s);while(end>s && (end[-1]==' ' || end[-1]=='\t'))*--end=0;}
    if(!*s)return;
    if(!strcmp(s,"!") || !strcmp(s,"s") || !strcmp(s,"z") || !strcmp(s,"stop")){stop_pending=1;return;}
    if(!strcmp(s,"help")){diagnostic_help();return;}
    if(!strcmp(s,"imutest extra") || !strncmp(s,"imutest extra ",14)) {
        if(s[13]) {
            if(run_active || drive.state==MOTION_RUNNING) {
                text("[imutest extra rejected: active segment]\r\n");return;
            }
            if(strcmp(s+14,"0") && strcmp(s+14,"1")) {
                text("[imutest extra rejected: use 0 or 1]\r\n");return;
            }
            accel_extra_enabled=(uint8_t)(s[14]=='1');
        }
        text("[imutest extra] enabled=");number(accel_extra_enabled);text("\r\n");return;
    }
    if(!strcmp(s,"imu")){
        imu_status();text("[JY901B] USART2 PD6 RX; baud=");number(diagnostic_imu_baud);
        text("; axes: +X=forward +Y=left +Z=up; verified by f/r/l direction tests");
        text("; passive WIT55/53 yaw, no configuration writes\r\n");return;
    }
    if(!strncmp(s,"imu baud ",9)) {
        const char *value=s+9;uint32_t baud=0;
        if(!strcmp(value,"9600"))baud=9600;else if(!strcmp(value,"19200"))baud=19200;
        else if(!strcmp(value,"38400"))baud=38400;else if(!strcmp(value,"57600"))baud=57600;
        else if(!strcmp(value,"115200"))baud=115200;else if(!strcmp(value,"230400"))baud=230400;
        if(!baud || run_active || drive.state==MOTION_RUNNING)text("[baud rejected: supported baud and idle chassis required]\r\n");
        else diagnostic_set_baud(baud);
        return;
    }
    if(!strcmp(s,"feedback")){feedback_status();return;}
    if(!strcmp(s,"info") || !strcmp(s,"status")){run_status();return;}
    if(!strcmp(s,"arm status")) {
        text("[arm fixed] lift135/slide0 MANUALLY PLACED; yaw0 claw45 platform20 PWM held; axes5/7 enabled; no position feedback claim\r\n");return;
    }
    if(!strcmp(s,"loop") || !strcmp(s,"loop status") || !strcmp(s,"loop gyro on")
        || !strcmp(s,"loop gyro off") || !strcmp(s,"loop encoder on") || !strcmp(s,"loop encoder off")
        || !strcmp(s,"loop forward on") || !strcmp(s,"loop forward off")) {
        (void)drive_loop_command(s);return;
    }
    if(!strcmp(s,"test") || !strcmp(s,"turn")) {
        if(run_active || drive.state==MOTION_RUNNING){text("[rejected: active segment; wait or !]\r\n");return;}
        test_mode=!strcmp(s,"test") ? MODE_TRANSLATION_TEST : MODE_ROTATION_TEST;
        text(test_mode==MODE_TRANSLATION_TEST ? "[test READY: single-segment IMU/encoder; fixed arm]\r\n" : "[turn READY: IMU angle; fixed arm]\r\n");return;
    }
    if(test_mode==MODE_ROTATION_TEST)accepted=command_turn_test(s);
    else if(test_mode==MODE_TRANSLATION_TEST)accepted=command_translation_test(s);
    text(accepted ? "[accepted]\r\n" : "[rejected: chassis-test command/range/busy/heading; help]\r\n");
}

static void start_button_request(char origin)
{
    char start_command[]="runimu C";
    if(start_button.preparing) {
        text("[button] preparation already pending\r\n");return;
    }
    start_button.origin=origin;
    start_command[7]=origin;
#ifndef CHASSIS_OBSTACLE_TEST
    if(station_auto_enabled() && !arm_enabled() && !arm_pose_known()
        && !arm_platform_known() && !arm_lock.state) {
        arm_serial_command("arm on");
        if(arm_lock.state==1) {
            start_button.preparing=1;
            text("[button] preparing ");text(start_command);
            text("; operator confirms home pose and empty open claw\r\n");
        }
        return;
    }
#endif
    text("[button] requesting ");text(start_command);text("\r\n");
    command(start_command);
}

static void start_button_prepare_update(void)
{
    HeadingSample sample;
    char pose_command[80];
    char start_command[]="runimu C";
    if(!start_button.preparing)return;
    start_command[7]=start_button.origin;
    if(run_active || !station_auto_enabled() || !arm_lock.state || arm_lock.state==3) {
        start_button.preparing=0;return;
    }
    if(arm_lock.state!=2)return;
    if(start_button.preparing==1) {
        /* 仅冷启动未知姿态使用已确认的摆放约定，不覆盖后来人工声明的坐标。 */
        if(!arm_enabled() || arm_pose_known() || arm_platform_known()) {
            start_button.preparing=0;return;
        }
        snprintf(pose_command,sizeof(pose_command),"arm pose %d %d %.1f %.1f",
            ARM_SLIDE_HOME_MM,ARM_LIFT_HOME_MM,(double)ARM_YAW_HOME_DEG,(double)ARM_CLAW_OPEN_DEG);
        arm_command(pose_command,clock_ms);
        snprintf(pose_command,sizeof(pose_command),"arm platform pose %.1f",(double)ARM_PLATFORM_FIRST_DEG);
        arm_command(pose_command,clock_ms);
        start_button.preparing=2;
        text("[button] fixed startup pose declared; waiting for fresh IMU\r\n");
    }
    if(!arm_ready() || !arm_parked() || arm_platform_slot()!=1) {
        start_button.preparing=0;return;
    }
    sample=heading_snapshot();
    if(!heading_sample_fresh(&sample,clock_ms,CAR_IMU_MAX_AGE_MS))return;
    start_button.preparing=0;
    text("[button] requesting ");text(start_command);text("\r\n");
    command(start_command);
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
    if(rx_overflow && !stop_pending) {
        __disable_irq();rx_tail=rx_head;rx_overflow=0;__enable_irq();
        line_length=0;text("[RX overflow warning: discarded input; no automatic stop]\r\n");
    }
    if (stop_pending)
    {
        uint8_t overflow;
        memset(&station_hold,0,sizeof(station_hold));run.start_reference_valid=0;
        start_button_cancel();
#ifdef CHASSIS_OBSTACLE_TEST
        obstacle_hits=obstacle_shot_valid=0;
        can_tx_cancel(0);
#else
        /* No scan controller. */
        can_tx_cancel(0);if(run.vision_debug)run.job=0;run.vision_debug=0;
        if (arm_lock.state || arm_enabled()) {
            arm_stop();
            arm_lock.state = 3;
        }
        arm_fixed_hold_stop();run.departure_overlap=0;
#endif
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
    run_timing_poll();
    value=0; /* PC9 disabled in chassis diagnostic. */
    if(stopped_this_step || stop_pending)start_button.pressed_valid=0;
    else if(value) {
#ifndef CHASSIS_OBSTACLE_TEST
        if(start_scan_active())text("[button rejected: start scan active; use !]\r\n");
        else
#endif
            start_button_request((char)value);
    }
#ifdef CHASSIS_OBSTACLE_TEST
#ifndef NAV_ENTRY_HOST_TEST
    if (!stop_pending) obstacle_io_poll();
#endif
#else
    /* No scanner in chassis diagnostic. */
    /* No vision in chassis diagnostic. */
    /* No arm state-machine interlock in fixed-pose diagnostic. */
    (void)stopped_this_step;
#endif
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
            rx_overflow = 1;
            line_length = 0;
        }
    }
#ifdef CHASSIS_OBSTACLE_TEST
    if (!stop_pending && run_active) run_update();
#else
    /* No screen polling in chassis diagnostic. */
    if (!stop_pending)
    {
        /* Fixed arm PWM only; never arm_update(). */
        /* No delayed route launch. */
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
                 : ((arm_ready() && arm_parked()) || (run.departure_overlap
                       && (run.departure_overlap==2u ? station_through_departure(NULL) : station_departure_ready()))))) {
            run_fault("arm left parked pose during route");return;
        }
        if ((arm_lock.state || arm_enabled()) && !run_active) return;
        if (run_active) {
            run_update();
            /* 底盘先处理反馈，工位内的纠偏子动作仍可并行推进既有作业。 */
            if(run_active && run.station_shift && !stop_pending) {
                station_update(clock_ms);
                if(station_failed() || arm_failed())run_fault("station work failed");
            }
        }
        else if (!test_mode)
            nav_update(&nav, clock_ms);
    }
#endif
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
        CALIBRATED_PPM_X, CALIBRATED_PPM_Y, CALIBRATED_PPM_FORWARD, CALIBRATED_PPM_BACKWARD,
        1, 0, -1, 1, LEFT_FORWARD_MM_PER_M, RIGHT_FORWARD_MM_PER_M
    };
    NavMotionPort port =
    {
        port_start, port_poll, port_stop, &drive
    };
    uint8_t i;
    delay_init(168);
    board_init();
#ifdef CHASSIS_OBSTACLE_TEST
    clock_init_ms();
    obstacle_io_init();
    /* No PC9 start in chassis diagnostic. */
#else
    uart_init();
    clock_init_ms();
    diagnostic_init();
    /* No PC9 start in chassis diagnostic. */
    /* No scanner hardware initialization. */
#endif
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
#ifdef CHASSIS_OBSTACLE_TEST
    text("[ultrasonic chassis test] CAN wheels 1-4; U4 FRONT; U5 LEFT; U6 software UART 115200 8N1\r\n");
    text("[obstacle policy] route=TIME_COST horizon=REMAINING passed_detection=OFF sonar=CONTINUOUS transit_AC=INSET_XY final_home=INSIDE\r\n");
    text("[start links] X_mm=");number(OBSTACLE_START_INSET_MM);
    text(" A_Y_mm=");number(ROAD_A_TOP_LINK_MM);text(" C_Y_mm=");number(ROAD_C_BOTTOM_LINK_MM);text("\r\n");
    text("[obstacle threshold] toward_B_F_lt_mm=");number(OBSTACLE_CLOSE_MM);
    text(" ordinary_le_mm=");number(OBSTACLE_EARLY_MM);
    text(" hits=");number(OBSTACLE_REQUIRED_HITS);text(" entry_wait=ORDINARY_ONLY\r\n");
    text("[route estimate] turn_event_ms=");number(OBSTACLE_TURN_COST_MS);
    text(" unknown_cost_ms=");number(OBSTACLE_UNKNOWN_COST_MS);text("; initial estimates, not measured\r\n");
    text("PC9 short=C nose toward B; long=A nose toward D/E/F; ! stops\r\n");
#else
    text("== JY901B independent CHASSIS diagnostic ==\r\n");
    text("== FIXED ARM PWM, NO ARM ACTIONS; ! stops motors, retains PWM ==\r\n");
    text("[radio] U6 PB0 TX -> module RX; PB1 RX <- module TX; 115200 8N1 software UART\r\n");
    text("[drive] JY901B115200; fixed arm PWM; isolated chassis test, camera/scanner/PC9 disabled\r\n");
    text("Inspect fixed arm pose and fresh IMU; use test/turn only.\r\n");
    diagnostic_help();
#endif
    /* 每轮只处理一个字节，使运动状态与停止请求持续获得处理。 */
    for (;;)
        firmware_step();
}

#endif
