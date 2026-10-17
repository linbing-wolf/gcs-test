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
    uint8_t stable_samples;
    uint32_t started_ms, stable_ms, sample_sequence, sample_ms;
    const char *fault;
} HeadingControl;

int heading_control_start(HeadingControl *control, HeadingSample sample,
                          int32_t relative_cdeg, uint32_t now_ms);
HeadingDecision heading_control_update(HeadingControl *control, HeadingSample sample,
                                       uint32_t now_ms);
int16_t heading_control_rpm(const HeadingControl *control, uint16_t max_rpm);
/* forward>0为沿车头前进，turn>0为左转；输出正值对应原驱动dir=1。 */
void heading_control_wheels(int16_t forward, int16_t turn, int16_t rpm[4]);
/* left>0为车身左移；前进、横移、旋转叠加后统一限幅。 */
void heading_control_translate_wheels(int16_t forward, int16_t left,
                                     int16_t turn, int16_t rpm[4]);
#endif
