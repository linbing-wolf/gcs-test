#include "motion.h"
#include "car_config.h"
#include <math.h>
#include <string.h>
typedef struct
{
    uint8_t addr, dir_pos;
    int8_t sx, sy;
} Wheel;
/* 保留队友的地址、安装方向和车身X/Y方向映射。 */
static const Wheel wheels[4] =
{
    /* 地址，正向方向位，X方向符号，Y方向符号。日常标定保持安装映射。 */
    {1, 1, -1, 1},
    {2, 0, 1, 1},
    {3, 0, -1, 1},
    {4, 1, 1, 1}
};
static void stop_wheels(Motion *motion)
{
    uint8_t wheel_index;
    for (wheel_index = 0; wheel_index < 4; wheel_index++)
        motion->io.stop(motion->io.user, wheels[wheel_index].addr);
}

/* 平移和旋转共用原来的脉冲估时；不是车身到位或转角测量。 */
static void estimate_motion(Motion *motion, uint32_t pulses, uint16_t rpm, uint8_t acc)
{
    float acceleration = acc ? acc : 256.0f;
    float ramp_pulses = 53.3333f * (float)rpm * (float)rpm / acceleration;
    float ramp_time_ms = 2000.0f * (float)rpm / acceleration;
    float estimated_time_ms = ramp_pulses >= (float)pulses
        ? ramp_time_ms * sqrtf((float)pulses / ramp_pulses)
        : 1000.0f * ((float)pulses - ramp_pulses) / (53.3333f * (float)rpm) + ramp_time_ms;
    motion->estimate_ms = estimated_time_ms < (float)CAR_MOTION_MIN_ESTIMATE_MS
        ? CAR_MOTION_MIN_ESTIMATE_MS : (uint32_t)estimated_time_ms;
    motion->timeout_ms = CAR_MOTION_TIMEOUT_FACTOR * motion->estimate_ms + CAR_MOTION_TIMEOUT_EXTRA_MS;
}

int motion_init(Motion *motion, const MotionIO *io, const MotionConfig *config)
{
    if (!motion || !io || !config
        || !io->position || !io->synchronize || !io->stop
        || !(config->ppm_x >= 1.0f && config->ppm_x <= 100.0f)
        || !(config->ppm_y >= 1.0f && config->ppm_y <= 100.0f)
        || config->forward_family > 1 || config->left_family > 1
        || config->forward_family == config->left_family
        || (config->forward_sign != 1 && config->forward_sign != -1)
        || (config->left_sign != 1 && config->left_sign != -1)
        || config->left_forward_mm_per_m < -100 || config->left_forward_mm_per_m > 100
        || config->right_forward_mm_per_m < -100 || config->right_forward_mm_per_m > 100)
        return 0;
    memset(motion, 0, sizeof(*motion));
    motion->io = *io;
    motion->config = *config;
    return 1;
}

int motion_start(Motion *motion, MotionParameters parameters, uint32_t now_ms)
{
    int distance_mm, direction_sign;
    uint8_t wheel_family, wheel_index;
    uint16_t rpm = parameters.rpm;
    float pulse_count, trim_mm, trim_pulses;
    float wheel_signed[4];
    uint32_t wheel_pulses[4], max_pulses = 0;
    if (!motion || (motion->state != MOTION_IDLE && motion->state != MOTION_DONE)
        || (unsigned) parameters.axis > BODY_LEFT || rpm < 20 || rpm > 300)
        return 0;
    distance_mm = parameters.mm < 0 ? - (int) parameters.mm : parameters.mm;
    if (!distance_mm || distance_mm > 2000)
        return 0;
    wheel_family = parameters.axis == BODY_FORWARD ? motion->config.forward_family : motion->config.left_family;
    direction_sign = parameters.axis == BODY_FORWARD ? motion->config.forward_sign : motion->config.left_sign;
    if (parameters.mm < 0)
        direction_sign = -direction_sign;
    /* 1. 按当前车身方向，把目标毫米数换算成基础脉冲。 */
    pulse_count = (wheel_family ? motion->config.ppm_y : motion->config.ppm_x)
                  * (float) distance_mm;
    motion->pulses = (uint32_t)(pulse_count + 0.5f);
    motion->generation++;
    motion->parameters = parameters;
    motion->started_ms = now_ms;
    motion->forward_trim_mm = 0;
    motion->rotating = motion->clockwise = 0;
    if (parameters.axis == BODY_LEFT)
    {
        int16_t per_m = parameters.mm > 0
                        ? motion->config.left_forward_mm_per_m
                        : motion->config.right_forward_mm_per_m;
        trim_mm = (float) distance_mm * (float) per_m / 1000.0f;
        motion->forward_trim_mm = (int16_t)(trim_mm >= 0 ? trim_mm + 0.5f : trim_mm - 0.5f);
    }
    trim_pulses = (float) motion->forward_trim_mm
                  * (motion->config.forward_family ? motion->config.ppm_y : motion->config.ppm_x);
    /* 一次合成横移和微量前移：沿用两个车身方向的轮子符号。
     * 不追加运动段，不根据估时或开环补偿确认到位。
     */
    for (wheel_index = 0; wheel_index < 4; wheel_index++)
    {
        int primary = (wheel_family ? wheels[wheel_index].sy : wheels[wheel_index].sx) * direction_sign;
        int forward = (motion->config.forward_family
                       ? wheels[wheel_index].sy : wheels[wheel_index].sx)
                      * motion->config.forward_sign;
        wheel_signed[wheel_index] = (float) primary * pulse_count + (float) forward * trim_pulses;
        wheel_pulses[wheel_index] = (uint32_t)(fabsf(wheel_signed[wheel_index]) + 0.5f);
        if (wheel_pulses[wheel_index] > max_pulses)
            max_pulses = wheel_pulses[wheel_index];
    }
    /* 2. 以最大轮脉冲计算原有估时；仅用于等待和超时，不证明到位。 */
    estimate_motion(motion, max_pulses, rpm, parameters.acceleration);
    motion->state = MOTION_RUNNING;
    /* 3. 各轮提交一次位置命令，最后一次广播同步启动。 */
    for (wheel_index = 0; wheel_index < 4; wheel_index++)
    {
        uint8_t dir = wheel_signed[wheel_index] >= 0
                      ? wheels[wheel_index].dir_pos : (uint8_t) !wheels[wheel_index].dir_pos;
        /* 大行程轮使用请求速度，小行程轮同比减速，减少末段单侧独走。
         * RPM取整和驱动加减速仍会影响轨迹；这不是姿态反馈闭环。
         */
        uint16_t wheel_rpm = (uint16_t)((wheel_pulses[wheel_index] * (uint32_t) rpm
                                         + max_pulses / 2u) / max_pulses);
        if (wheel_pulses[wheel_index] && !wheel_rpm)
            wheel_rpm = 1;
        if (!motion->io.position(motion->io.user, wheels[wheel_index].addr, dir, wheel_rpm, parameters.acceleration, wheel_pulses[wheel_index]))
        {
            stop_wheels(motion);
            motion->state = MOTION_FAULT;
            return 0;
        }
    }
    if (!motion->io.synchronize(motion->io.user))
    {
        stop_wheels(motion);
        motion->state = MOTION_FAULT;
        return 0;
    }
    return 1;
}

int motion_start_rotation(Motion *motion, uint8_t clockwise, uint32_t pulses,
                          uint16_t rpm, uint8_t acceleration, uint32_t now_ms)
{
    uint8_t wheel_index;
    uint8_t direction = clockwise ? (uint8_t)!CAR_TURN_CCW_DIR : CAR_TURN_CCW_DIR;
    if (!motion || (motion->state != MOTION_IDLE && motion->state != MOTION_DONE)
        || clockwise > 1u || !pulses || pulses > 2u * CAR_TURN_MAX_TEST_PULSES
        || rpm < 20 || rpm > 300 || CAR_TURN_CCW_DIR > 1u)
        return 0;
    motion->parameters.axis = BODY_FORWARD;
    motion->parameters.mm = 0;
    motion->parameters.rpm = rpm;
    motion->parameters.acceleration = acceleration;
    motion->generation++;
    motion->started_ms = now_ms;
    motion->pulses = pulses;
    motion->forward_trim_mm = 0;
    motion->rotating = 1;
    motion->clockwise = clockwise;
    estimate_motion(motion, pulses, rpm, acceleration);
    motion->state = MOTION_RUNNING;
    /* 原 car_move 的纯旋转映射：四轮方向位相同，沿用已有位置接口同步启动。 */
    for (wheel_index = 0; wheel_index < 4; wheel_index++)
    {
        if (!motion->io.position(motion->io.user, wheels[wheel_index].addr,
                                 direction, rpm, acceleration, pulses))
        {
            stop_wheels(motion);
            motion->state = MOTION_FAULT;
            return 0;
        }
    }
    if (!motion->io.synchronize(motion->io.user))
    {
        stop_wheels(motion);
        motion->state = MOTION_FAULT;
        return 0;
    }
    return 1;
}

MotionState motion_update(Motion *motion, uint32_t now_ms)
{
    uint32_t elapsed;
    if (!motion)
        return MOTION_FAULT;
    if (motion->state != MOTION_RUNNING && motion->state != MOTION_WAIT_CONFIRM)
        return motion->state;
    elapsed = now_ms - motion->started_ms;
    if (elapsed >= motion->timeout_ms)
    {
        stop_wheels(motion);
        motion->state = MOTION_FAULT;
    }
    else if (elapsed >= motion->estimate_ms)
        motion->state = MOTION_WAIT_CONFIRM;
    return motion->state;
}

int motion_confirm(Motion *motion, uint32_t generation)
{
    if (!motion || generation != motion->generation || (motion->state != MOTION_RUNNING
        && motion->state != MOTION_WAIT_CONFIRM))
        return 0;
    motion->state = MOTION_DONE;
    return 1;
}

void motion_stop(Motion *motion)
{
    if (!motion)
        return;
    stop_wheels(motion);
    motion->generation++;
    motion->state = MOTION_IDLE;
}
