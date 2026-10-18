#ifndef NINE_MOTION_H
#define NINE_MOTION_H
#include "road_map.h"
typedef enum
{
    MOTION_IDLE,          /* 可接受下一次运动。 */
    MOTION_RUNNING,       /* 当前相对位置命令执行中。 */
    MOTION_WAIT_CONFIRM,  /* 估时已过，仍等待人工确认或全程电机反馈。 */
    MOTION_DONE,          /* 当前命令已得到相应确认。 */
    MOTION_FAULT          /* 提交失败或等待超时，已请求停车。 */
} MotionState;
typedef struct
{
    /* 返回1仅表示驱动提交成功，不等于驱动器ACK或电机到位。 */
    int (*position)(void *user, uint8_t addr, uint8_t dir,
                    uint16_t rpm, uint8_t acc, uint32_t pulses);
    int (*synchronize)(void *user);
    void (*stop)(void *user, uint8_t addr);
    void *user;
} MotionIO;
typedef struct
{
    float ppm_x;               /* X 方向脉冲/mm：当前对应车身左右。 */
    float ppm_y;               /* Y 方向脉冲/mm：当前对应车身前后。 */
    uint8_t forward_family;    /* 前后使用哪组轮子符号，当前为 1。 */
    uint8_t left_family;       /* 左右使用哪组轮子符号，当前为 0。 */
    int8_t forward_sign;       /* 电机安装方向映射，当前为 -1。 */
    int8_t left_sign;          /* 电机安装方向映射，当前为 +1。 */
    /* 横移每1000 mm叠加的车身前移量；正值向前，0关闭对应方向补偿。 */
    int16_t left_forward_mm_per_m;
    int16_t right_forward_mm_per_m;
} MotionConfig;
typedef struct
{
    MotionIO io;
    MotionConfig config;
    MotionState state;
    uint32_t generation;      /* 每次开始/停车都会变化的命令代号。 */
    uint32_t started_ms;      /* 当前段提交时的系统时间。 */
    uint32_t estimate_ms;     /* 估时，不是到位信号。 */
    uint32_t timeout_ms;      /* 超过此等待时长会请求停车。 */
    uint32_t pulses;          /* 基础运动脉冲；预补偿后四轮脉冲可能不同。 */
    MotionParameters parameters;
    int16_t forward_trim_mm;  /* 本次横移叠加的前移量，正值向前。 */
    uint8_t rotating;         /* 1：四轮脉冲旋转；此时parameters.mm不表示距离。 */
    uint8_t clockwise;        /* 旋转指令方向，物理方向由小脉冲测试验证。 */
} Motion;
int motion_init(Motion *motion, const MotionIO *io, const MotionConfig *config);
int motion_start(Motion *motion, MotionParameters parameters, uint32_t now_ms);
/* pulses是每轮相对位置脉冲，不是车身角度；无IMU时必须实测标定。 */
int motion_start_rotation(Motion *motion, uint8_t clockwise, uint32_t pulses,
                          uint16_t rpm, uint8_t acceleration, uint32_t now_ms);
MotionState motion_update(Motion *motion, uint32_t now_ms);
/* 实际反馈或人工确认必须绑定本次命令代号，拒绝上次命令的迟到反馈。 */
int motion_confirm(Motion *motion, uint32_t generation);
void motion_stop(Motion *motion);
#endif
