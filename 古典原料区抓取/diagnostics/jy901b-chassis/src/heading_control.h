#ifndef HEADING_CONTROL_H
#define HEADING_CONTROL_H
#include "heading_sensor.h"

typedef enum {
    HEADING_WAIT, HEADING_CORRECT, HEADING_DONE, HEADING_FAULT
} HeadingDecision;

/* 累计新IMU样本以保留左右180度的方向；连续PD调速不访问硬件。
 * 转向结束后仍保留同一目标供直行纠偏，不能重置为当前歪斜的航向。
 */
typedef struct {
    int32_t target_cdeg, progress_cdeg, error_cdeg, rate_cdeg_s;
    int16_t last_yaw_cdeg;
    int32_t measured_rate_cdeg_s,angle_rate_cdeg_s;
    float filtered_rate_cdeg_s;
    uint8_t stable_samples, measured_rate_valid,rate_source;
    uint32_t rate_ms;
    uint32_t started_ms, stable_ms, sample_sequence, sample_ms;
    const char *fault;
} HeadingControl;

int heading_control_start(HeadingControl *control, HeadingSample sample,
                          int32_t relative_cdeg, uint32_t now_ms);
/* 每个控制拍在update前提供实测角速度；valid=0或未调用时回退角度差分。 */
void heading_control_measured_rate(HeadingControl *control, int32_t rate_cdeg_s, uint8_t valid);
HeadingDecision heading_control_update(HeadingControl *control, HeadingSample sample,
                                       uint32_t now_ms);
int16_t heading_control_rpm(const HeadingControl *control, uint16_t max_rpm);
/* 前进/后退保持航向；允许越过零误差前的角速度阻尼，不使用原地转向3RPM规则。 */
int16_t heading_control_drive_rpm(const HeadingControl *control, uint16_t max_rpm);
typedef struct {uint16_t kp_x100,kd_x100;} HeadingDriveGains;
/* lateral=1为横移，0为前后；低于/高于表范围保持端点，中间按速度线性插值。 */
HeadingDriveGains heading_control_drive_gains(uint8_t lateral,uint16_t speed_rpm);
int16_t heading_control_drive_rpm_with_gains(const HeadingControl *control,uint16_t max_rpm,
                                          HeadingDriveGains gains);
/* forward>0为沿车头前进，turn>0为左转；输出正值对应原驱动dir=1。 */
void heading_control_wheels(int16_t forward, int16_t turn, int16_t rpm[4]);
/* left>0为车身左移；前进、横移、旋转叠加后统一限幅。 */
void heading_control_translate_wheels(int16_t forward, int16_t left,
                                     int16_t turn, int16_t rpm[4]);
#endif
