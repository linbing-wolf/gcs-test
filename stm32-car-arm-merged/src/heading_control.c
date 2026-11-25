#include "heading_control.h"
#include "car_config.h"
#include <string.h>
#if CHASSIS_IMU_EXPERIMENT
#include <math.h>
#endif

int heading_control_start(HeadingControl *control, HeadingSample sample,
                          int32_t relative_cdeg, uint32_t now_ms)
{
    if (!control || relative_cdeg < -18000 || relative_cdeg > 18000
        || !heading_sample_fresh(&sample, now_ms, CAR_IMU_MAX_AGE_MS))
        return 0;
    memset(control, 0, sizeof(*control));
    control->target_cdeg = control->error_cdeg = relative_cdeg;
    control->last_yaw_cdeg = sample.yaw_cdeg;
    control->started_ms = now_ms;
    control->sample_ms = sample.received_ms;
    control->sample_sequence = sample.sequence;
    return 1;
}

HeadingDecision heading_control_update(HeadingControl *control, HeadingSample sample,
                                       uint32_t now_ms)
{
    uint32_t elapsed_sample;
    int32_t magnitude, delta;
    if (control->fault && sample.sequence==control->sample_sequence) return HEADING_FAULT;
    control->fault = 0;
    if (!heading_sample_fresh(&sample, now_ms, CAR_IMU_MAX_AGE_MS))
        control->fault = "IMU sample stale/invalid";
    if (control->fault) return HEADING_FAULT;
    if (sample.sequence == control->sample_sequence) return HEADING_WAIT;
    control->sample_sequence = sample.sequence;
    delta = heading_wrap_cdeg((int32_t)sample.yaw_cdeg - control->last_yaw_cdeg);
    if (delta > 4500 || delta < -4500) {
        control->fault = "IMU discontinuous angle sample";
        return HEADING_FAULT;
    }
    elapsed_sample = sample.received_ms - control->sample_ms;
    if (!elapsed_sample) return HEADING_WAIT;
    control->rate_cdeg_s = delta * 1000 / (int32_t)elapsed_sample;
#if CHASSIS_IMU_EXPERIMENT
    if (control->experiment_mode >= IMUTEST_GYRO && control->gyro_seeded)
        control->rate_cdeg_s = (int32_t)control->filtered_rate_cdeg_s;
#endif
    control->sample_ms = sample.received_ms;
    control->progress_cdeg += delta;
    control->last_yaw_cdeg = sample.yaw_cdeg;
    control->error_cdeg = control->target_cdeg - control->progress_cdeg;
    magnitude = control->error_cdeg < 0 ? -control->error_cdeg : control->error_cdeg;
    if (magnitude <= CAR_HEADING_TOLERANCE_CDEG
        && control->rate_cdeg_s <= CAR_HEADING_STABLE_RATE_CDEG_S
        && control->rate_cdeg_s >= -CAR_HEADING_STABLE_RATE_CDEG_S) {
        if (!control->stable_samples) control->stable_ms = sample.received_ms;
        if (control->stable_samples < 255u) control->stable_samples++;
        if (control->stable_samples >= CAR_HEADING_STABLE_SAMPLES
            && (uint32_t)(sample.received_ms - control->stable_ms) >= CAR_HEADING_STABLE_MS)
            return HEADING_DONE;
        return HEADING_WAIT;
    }
    control->stable_samples = 0;
    return HEADING_CORRECT;
}

int16_t heading_control_rpm(const HeadingControl *control, uint16_t max_rpm)
{
    int32_t error = control->error_cdeg;
    int32_t rpm;
    if (error <= CAR_HEADING_TOLERANCE_CDEG && error >= -CAR_HEADING_TOLERANCE_CDEG) return 0;
    rpm = error * CAR_HEADING_KP_X100 - control->rate_cdeg_s * CAR_HEADING_KD_X100;
#if !CHASSIS_IMU_EXPERIMENT
    rpm /= 10000;
#else
    if (!control->experiment_mode) rpm /= 10000;
#endif
    /* 接近目标但仍有惯性时先给零速，不提前反向；实际过冲后才反修。 */
    if ((error > 0 && rpm < 0) || (error < 0 && rpm > 0)) return 0;
#if CHASSIS_IMU_EXPERIMENT
    if (control->experiment_mode) rpm /= 10000;
#endif
    if (error > 0 && rpm < CAR_HEADING_MIN_RPM) rpm = CAR_HEADING_MIN_RPM;
    if (error < 0 && rpm > -CAR_HEADING_MIN_RPM) rpm = -CAR_HEADING_MIN_RPM;
    if (rpm > max_rpm) rpm = max_rpm;
    if (rpm < -(int32_t)max_rpm) rpm = -(int32_t)max_rpm;
    return (int16_t)rpm;
}

#if CHASSIS_IMU_EXPERIMENT
void heading_experiment_reset(HeadingControl *control)
{
    control->gyro_seeded = control->integral_seeded = 0;
    control->integral_rpm = 0.0f;
    control->stable_samples = 0;
}

int heading_experiment_gyro(HeadingControl *control, HeadingSample sample, uint32_t now_ms)
{
    float rate, alpha;
    if (control->experiment_mode < IMUTEST_GYRO) return 1;
    if (!sample.gyro_valid || now_ms - sample.gyro_ms > IMUTEST_GYRO_MAX_AGE_MS)
        return 0;
    if (control->gyro_seeded && control->gyro_sequence == sample.gyro_sequence) return 1;
    rate = (float)(sample.gyro_z_cdeg_s - IMUTEST_GYRO_BIAS_CDEG_S);
    if (!control->gyro_seeded || sample.gyro_ms - control->gyro_ms > IMUTEST_GYRO_MAX_AGE_MS) {
        control->filtered_rate_cdeg_s = rate;
    } else {
        alpha = expf(-(float)(sample.gyro_ms - control->gyro_ms) / IMUTEST_GYRO_TAU_MS);
        control->filtered_rate_cdeg_s = alpha * control->filtered_rate_cdeg_s + (1.0f - alpha) * rate;
    }
    control->gyro_seeded = 1;
    control->gyro_sequence = sample.gyro_sequence;
    control->gyro_ms = sample.gyro_ms;
    control->rate_cdeg_s = (int32_t)control->filtered_rate_cdeg_s;
    if (control->rate_cdeg_s > CAR_HEADING_STABLE_RATE_CDEG_S
        || control->rate_cdeg_s < -CAR_HEADING_STABLE_RATE_CDEG_S) control->stable_samples = 0;
    return 1;
}

int16_t heading_experiment_drive(HeadingControl *control, int16_t forward, int integrate)
{
    int16_t speed = forward < 0 ? -forward : forward;
    int16_t limit = speed > 0 ? speed - 1 : 0;
    int16_t rounded;
    float error = control->error_cdeg / 100.0f;
    uint16_t kp = control->pid_kp_x100 ? control->pid_kp_x100 : CAR_DRIVE_HEADING_KP_X100;
    uint16_t kd = control->pid_kp_x100 ? control->pid_kd_x100 : CAR_DRIVE_HEADING_KD_X100;
    float ki = control->pid_kp_x100 ? control->pid_ki_x100 / 100.0f : IMUTEST_I_KI;
    float pd = (control->error_cdeg * kp - control->rate_cdeg_s * kd) / 10000.0f;
    float output;
    if (limit > CAR_HEADING_DRIVE_TRIM_RPM) limit = CAR_HEADING_DRIVE_TRIM_RPM;
    if (control->experiment_mode != IMUTEST_PID)
        return heading_control_drive_rpm(control, (uint16_t)limit);
    if (!integrate || speed < IMUTEST_I_MIN_SPEED_RPM) {
        control->integral_seeded = 0;
        control->integral_rpm = 0.0f;
    } else if (!control->integral_seeded || control->integral_sequence != control->sample_sequence) {
        uint32_t elapsed = control->sample_ms - control->integral_ms;
        if (control->integral_seeded && elapsed <= CAR_IMU_MAX_AGE_MS) {
            float alpha = expf(-(float)elapsed / (1000.0f * IMUTEST_I_TAU_S));
            float leaked = alpha * control->integral_rpm;
            float candidate = leaked + (1.0f - alpha) * ki * IMUTEST_I_TAU_S * error;
            if (candidate > IMUTEST_I_MAX_RPM) candidate = IMUTEST_I_MAX_RPM;
            if (candidate < -IMUTEST_I_MAX_RPM) candidate = -IMUTEST_I_MAX_RPM;
            control->integral_rpm = ((error > 0.0f && pd + candidate > limit)
                || (error < 0.0f && pd + candidate < -limit)) ? leaked : candidate;
        }
        control->integral_seeded = 1;
        control->integral_sequence = control->sample_sequence;
        control->integral_ms = control->sample_ms;
    }
    if (control->error_cdeg <= CAR_DRIVE_HEADING_QUIET_CDEG
        && control->error_cdeg >= -CAR_DRIVE_HEADING_QUIET_CDEG
        && control->rate_cdeg_s <= CAR_DRIVE_HEADING_QUIET_RATE_CDEG_S
        && control->rate_cdeg_s >= -CAR_DRIVE_HEADING_QUIET_RATE_CDEG_S) pd = 0.0f;
    output = pd + control->integral_rpm;
    if (output > limit) return limit;
    if (output < -limit) return -limit;
    rounded = (int16_t)(output < 0.0f ? output - 0.5f : output + 0.5f);
    if (!rounded && pd != 0.0f && output != 0.0f) return output > 0.0f ? 1 : -1;
    return rounded;
}
#endif

int16_t heading_control_drive_rpm(const HeadingControl *control, uint16_t max_rpm)
{
    int32_t error = control->error_cdeg, rate = control->rate_cdeg_s;
    int32_t effort, rpm;
    if (error <= CAR_DRIVE_HEADING_QUIET_CDEG && error >= -CAR_DRIVE_HEADING_QUIET_CDEG
        && rate <= CAR_DRIVE_HEADING_QUIET_RATE_CDEG_S && rate >= -CAR_DRIVE_HEADING_QUIET_RATE_CDEG_S)
        return 0;
    effort = error * CAR_DRIVE_HEADING_KP_X100 - rate * CAR_DRIVE_HEADING_KD_X100;
    rpm = effort >= 0 ? (effort + 5000) / 10000 : -((-effort + 5000) / 10000);
    /* 电机接口仅接受整数RPM；静稳区外，保留PD方向而非按角差强制转向。 */
    if (effort > 0 && rpm < CAR_DRIVE_HEADING_MIN_RPM) rpm = CAR_DRIVE_HEADING_MIN_RPM;
    if (effort < 0 && rpm > -CAR_DRIVE_HEADING_MIN_RPM) rpm = -CAR_DRIVE_HEADING_MIN_RPM;
    if (rpm > max_rpm) rpm = max_rpm;
    if (rpm < -(int32_t)max_rpm) rpm = -(int32_t)max_rpm;
    return (int16_t)rpm;
}

void heading_control_wheels(int16_t forward, int16_t turn, int16_t rpm[4])
{
    heading_control_translate_wheels(forward, 0, turn, rpm);
}

void heading_control_translate_wheels(int16_t forward, int16_t left,
                                     int16_t turn, int16_t rpm[4])
{
    static const int8_t forward_sign[4] = {-1, 1, 1, -1};
    static const int8_t left_sign[4] = {-1, -1, 1, 1};
    int32_t values[4], maximum = 0;
    uint8_t i;
    for (i = 0; i < 4; i++) {
        int32_t magnitude;
        values[i] = forward_sign[i] * (int32_t)forward
                    + left_sign[i] * (int32_t)left
                    + (CAR_TURN_CCW_DIR ? 1 : -1) * (int32_t)turn;
        magnitude = values[i] < 0 ? -values[i] : values[i];
        if (magnitude > maximum) maximum = magnitude;
    }
    for (i = 0; i < 4; i++)
        rpm[i] = (int16_t)(maximum > 300 ? values[i] * 300 / maximum : values[i]);
}
