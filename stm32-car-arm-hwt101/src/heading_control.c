#include "heading_control.h"
#include "car_config.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

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
    control->rate_ms=now_ms;
    return 1;
}

void heading_control_measured_rate(HeadingControl *control, int32_t rate_cdeg_s, uint8_t valid)
{
    if (!control) return;
    control->measured_rate_cdeg_s = rate_cdeg_s;
    control->measured_rate_valid = valid ? 1u : 0u;
}

static int32_t heading_tolerance_cdeg(const HeadingControl *control)
{
    return control->tolerance_cdeg ? control->tolerance_cdeg : CAR_HEADING_TOLERANCE_CDEG;
}

HeadingDecision heading_control_update(HeadingControl *control, HeadingSample sample,
                                       uint32_t now_ms)
{
    uint32_t elapsed_sample;
    int32_t magnitude, delta,wanted;
#if !CAR_IMU_YAW_ONLY
    uint32_t rate_dt;
#endif
    uint8_t new_angle;
    if (control->fault && sample.sequence==control->sample_sequence) return HEADING_FAULT;
    control->fault = 0;
    if (!heading_sample_fresh(&sample, now_ms, CAR_IMU_MAX_AGE_MS))
        control->fault = "IMU sample stale/invalid";
    if (control->fault) return HEADING_FAULT;
    new_angle=sample.sequence!=control->sample_sequence;
    if(new_angle) {
        control->sample_sequence = sample.sequence;
        delta = heading_wrap_cdeg((int32_t)sample.yaw_cdeg - control->last_yaw_cdeg);
        if (delta > 4500 || delta < -4500) {
            control->fault = "IMU discontinuous angle sample";
            return HEADING_FAULT;
        }
        elapsed_sample = sample.received_ms - control->sample_ms;
        if (!elapsed_sample) return HEADING_WAIT;
        control->angle_rate_cdeg_s = delta * 1000 / (int32_t)elapsed_sample;
        control->sample_ms = sample.received_ms;
        control->progress_cdeg += delta;
        control->last_yaw_cdeg = sample.yaw_cdeg;
        control->error_cdeg = control->target_cdeg - control->progress_cdeg;
    }
#if CAR_IMU_YAW_ONLY
    if(!new_angle)return HEADING_WAIT;
    control->rate_cdeg_s = wanted = control->angle_rate_cdeg_s;
#else
    /* 角速度独立于角度更新；同一Yaw序号不能阻塞新陀螺仪反馈。两种来源连续滤波。 */
    wanted=control->measured_rate_valid ? control->measured_rate_cdeg_s : control->angle_rate_cdeg_s;
    control->rate_source=control->measured_rate_valid;control->measured_rate_valid=0;
    rate_dt=now_ms-control->rate_ms;
    if(rate_dt>CAR_DRIVE_SLEW_MAX_DT_MS)rate_dt=CAR_DRIVE_SLEW_MAX_DT_MS;
    if(rate_dt) {
        float alpha=(float)rate_dt/(CAR_IMU_RATE_FILTER_MS+rate_dt);
        control->filtered_rate_cdeg_s+=alpha*((float)wanted-control->filtered_rate_cdeg_s);
        control->rate_cdeg_s=(int32_t)lroundf(control->filtered_rate_cdeg_s);control->rate_ms=now_ms;
    }
    if(abs(wanted)>CAR_HEADING_STABLE_RATE_CDEG_S || abs(control->rate_cdeg_s)>CAR_HEADING_STABLE_RATE_CDEG_S)
        control->stable_samples=0;
    if(!new_angle)return HEADING_WAIT; /* 不用重复角度样本累加到位确认。 */
#endif
    magnitude = control->error_cdeg < 0 ? -control->error_cdeg : control->error_cdeg;
    if (magnitude <= heading_tolerance_cdeg(control) && abs(wanted)<=CAR_HEADING_STABLE_RATE_CDEG_S
        && control->rate_cdeg_s <= CAR_HEADING_STABLE_RATE_CDEG_S
        && control->rate_cdeg_s >= -CAR_HEADING_STABLE_RATE_CDEG_S) {
        if (!control->stable_samples) control->stable_ms = sample.received_ms;
        if (control->stable_samples < 255u) control->stable_samples++;
        if (control->stable_samples >= CAR_HEADING_STABLE_SAMPLES
#if CAR_HEADING_STABLE_MS > 0
            && (uint32_t)(sample.received_ms - control->stable_ms) >= CAR_HEADING_STABLE_MS
#endif
            )
            return HEADING_DONE;
        return HEADING_WAIT;
    }
    control->stable_samples = 0;
    return HEADING_CORRECT;
}

static int16_t heading_control_rpm_with_kd(const HeadingControl *control, uint16_t max_rpm, int32_t kd_x100)
{
    int32_t error = control->error_cdeg;
    int32_t minimum_rpm = CAR_HEADING_MIN_RPM;
    int32_t rpm;
    if (abs(error) <= heading_tolerance_cdeg(control)) return 0;
    if (control->tolerance_cdeg && abs(error)<=CAR_HEADING_TOLERANCE_CDEG) {
        /* 90° 低速补转，避免原最低轮速在两个 IMU 样本之间越过窄窗。 */
        minimum_rpm=CAR_MANUAL_TURN_90_FINE_RPM;
        if(max_rpm>minimum_rpm)max_rpm=(uint16_t)minimum_rpm;
    }
    rpm = (error * CAR_HEADING_KP_X100 - control->rate_cdeg_s * kd_x100) / 10000;
    /* 接近目标但仍有惯性时先给零速，不提前反向；实际过冲后才反修。 */
    if ((error > 0 && rpm < 0) || (error < 0 && rpm > 0)) return 0;
    if (error > 0 && rpm < minimum_rpm) rpm = minimum_rpm;
    if (error < 0 && rpm > -minimum_rpm) rpm = -minimum_rpm;
    if (rpm > max_rpm) rpm = max_rpm;
    if (rpm < -(int32_t)max_rpm) rpm = -(int32_t)max_rpm;
    return (int16_t)rpm;
}

int16_t heading_control_rpm(const HeadingControl *control, uint16_t max_rpm)
{
    return heading_control_rpm_with_kd(control,max_rpm,CAR_HEADING_KD_X100);
}

int16_t heading_control_turn_rpm(const HeadingControl *control, uint16_t max_rpm, int quarter_turn)
{
    const int32_t brake_start_cdeg=CAR_TURN_BRAKE_START_CDEG;
    int32_t remaining_cdeg=abs(control->error_cdeg);
    int32_t kd_x100=CAR_HEADING_KD_X100;
    if(quarter_turn && remaining_cdeg>CAR_TURN_90_KD_BLEND_END_CDEG) {
        const int32_t blend_span=CAR_TURN_90_KD_BLEND_START_CDEG-CAR_TURN_90_KD_BLEND_END_CDEG;
        int32_t blend_position=remaining_cdeg-CAR_TURN_90_KD_BLEND_END_CDEG;
        if(blend_position>blend_span)blend_position=blend_span;
        kd_x100+=(CAR_TURN_90_KD_X100-CAR_HEADING_KD_X100)*blend_position/blend_span;
    }
    if(remaining_cdeg<brake_start_cdeg && max_rpm>CAR_HEADING_MIN_RPM) {
        float remaining=(float)remaining_cdeg/brake_start_cdeg;
        float curve_weight=CAR_TURN_BRAKE_CURVE_X100/100.0f;
        float start_rpm=brake_start_cdeg*CAR_HEADING_KP_X100/10000.0f;
        float fraction=remaining*(1.0f-curve_weight*(1.0f-remaining));
        /* 入口沿用该角度的比例速度，曲线在末段连续降至低速修正档。 */
        start_rpm=fmaxf(CAR_HEADING_MIN_RPM,fminf(start_rpm,max_rpm));
        max_rpm=(uint16_t)lroundf(CAR_HEADING_MIN_RPM+(start_rpm-CAR_HEADING_MIN_RPM)*fraction);
    }
    return heading_control_rpm_with_kd(control,max_rpm,kd_x100);
}

HeadingDriveGains heading_control_drive_gains(uint8_t lateral,uint16_t speed_rpm)
{
    static const uint16_t speed[3]={CAR_DRIVE_GAIN_LOW_RPM,CAR_DRIVE_GAIN_MID_RPM,CAR_DRIVE_GAIN_HIGH_RPM};
    static const HeadingDriveGains table[2][3]={
        {{CAR_FORWARD_HEADING_KP_LOW_X100,CAR_FORWARD_HEADING_KD_LOW_X100},
         {CAR_FORWARD_HEADING_KP_MID_X100,CAR_FORWARD_HEADING_KD_MID_X100},
         {CAR_FORWARD_HEADING_KP_HIGH_X100,CAR_FORWARD_HEADING_KD_HIGH_X100}},
        {{CAR_LATERAL_HEADING_KP_LOW_X100,CAR_LATERAL_HEADING_KD_LOW_X100},
         {CAR_LATERAL_HEADING_KP_MID_X100,CAR_LATERAL_HEADING_KD_MID_X100},
         {CAR_LATERAL_HEADING_KP_HIGH_X100,CAR_LATERAL_HEADING_KD_HIGH_X100}}
    };
    const HeadingDriveGains *points=table[lateral ? 1 : 0];
    HeadingDriveGains gains;unsigned i;
    if(speed_rpm<=speed[0])return points[0];
    for(i=1;i<3u;i++)if(speed_rpm<=speed[i]) {
        uint32_t span=speed[i]-speed[i-1],high=speed_rpm-speed[i-1],low=span-high;
        gains.kp_x100=(uint16_t)((points[i-1].kp_x100*low+points[i].kp_x100*high+span/2)/span);
        gains.kd_x100=(uint16_t)((points[i-1].kd_x100*low+points[i].kd_x100*high+span/2)/span);
        return gains;
    }
    return points[2];
}

int16_t heading_control_drive_rpm(const HeadingControl *control,uint16_t max_rpm)
{
    const HeadingDriveGains gains={CAR_DRIVE_HEADING_KP_X100,CAR_DRIVE_HEADING_KD_X100};
    return heading_control_drive_rpm_with_gains(control,max_rpm,gains);
}

int16_t heading_control_drive_rpm_with_gains(const HeadingControl *control,uint16_t max_rpm,
                                          HeadingDriveGains gains)
{
    int32_t error = control->error_cdeg, rate = control->rate_cdeg_s;
    int32_t effort, rpm;
    if (error <= CAR_DRIVE_HEADING_QUIET_CDEG && error >= -CAR_DRIVE_HEADING_QUIET_CDEG
        && rate <= CAR_DRIVE_HEADING_QUIET_RATE_CDEG_S && rate >= -CAR_DRIVE_HEADING_QUIET_RATE_CDEG_S)
        return 0;
    effort = error * gains.kp_x100 - rate * gains.kd_x100;
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
        rpm[i] = (int16_t)(maximum > CAR_WHEEL_MAX_RPM ? values[i] * CAR_WHEEL_MAX_RPM / maximum : values[i]);
}
