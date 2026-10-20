#ifndef HEADING_SENSOR_H
#define HEADING_SENSOR_H
#include <stdint.h>

/* 角度统一为0.01度，逆时针为正；外部适配器只发布真实的新采样。 */
typedef struct {
    int16_t yaw_cdeg;
    uint32_t received_ms, sequence;
    uint8_t valid;
} HeadingSample;
typedef struct {
    volatile HeadingSample sample;
    uint8_t packet[11], count;
    volatile uint32_t checksum_errors;
} HeadingSensor;

void heading_sensor_reset(HeadingSensor *sensor);
void heading_sensor_publish(HeadingSensor *sensor, int16_t yaw_cdeg, uint32_t now_ms);
void heading_sensor_byte(HeadingSensor *sensor, uint8_t byte, uint32_t now_ms);
void heading_sensor_invalidate(HeadingSensor *sensor);
int heading_sample_fresh(const HeadingSample *sample, uint32_t now_ms, uint32_t max_age_ms);
int16_t heading_wrap_cdeg(int32_t angle);
#endif
