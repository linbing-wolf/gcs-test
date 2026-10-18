#include "heading_sensor.h"
#include <string.h>

int16_t heading_wrap_cdeg(int32_t angle)
{
    angle %= 36000;
    if (angle >= 18000) angle -= 36000;
    if (angle < -18000) angle += 36000;
    return (int16_t)angle;
}

void heading_sensor_reset(HeadingSensor *sensor)
{
    memset(sensor, 0, sizeof(*sensor));
}

void heading_sensor_invalidate(HeadingSensor *sensor)
{
    sensor->sample.valid = 0;
    sensor->count = 0;
}

void heading_sensor_publish(HeadingSensor *sensor, int16_t yaw_cdeg, uint32_t now_ms)
{
    if (yaw_cdeg < -18000 || yaw_cdeg > 18000) {
        heading_sensor_invalidate(sensor);
        return;
    }
    sensor->sample.yaw_cdeg = heading_wrap_cdeg(yaw_cdeg);
    sensor->sample.received_ms = now_ms;
    sensor->sample.sequence++;
    sensor->sample.valid = 1;
}

int heading_sample_fresh(const HeadingSample *sample, uint32_t now_ms, uint32_t max_age_ms)
{
    return sample->valid && (uint32_t)(now_ms - sample->received_ms) <= max_age_ms;
}

/* 沿用本项目WIT SDK的普通串口帧：55 53 + 8字节 + 累加校验。
 * 仅角度帧刷新时效；加速度/温度帧不能让旧航向重新变成有效数据。
 */
void heading_sensor_byte(HeadingSensor *sensor, uint8_t byte, uint32_t now_ms)
{
    uint8_t sum = 0, i;
    uint16_t raw;
    int32_t signed_raw;
    if (!sensor->count && byte != 0x55) return;
    sensor->packet[sensor->count++] = byte;
    if (sensor->count < sizeof(sensor->packet)) return;
    for (i = 0; i < 10; i++) sum = (uint8_t)(sum + sensor->packet[i]);
    if (sum != sensor->packet[10]) {
        sensor->checksum_errors++;
        /* 丢失一个字节后滑动寻找帧头，保留可能已收到的下一帧前缀。 */
        for (i = 1; i < 11 && sensor->packet[i] != 0x55; i++) {}
        sensor->count = (uint8_t)(11u - i);
        memmove(sensor->packet, sensor->packet + i, sensor->count);
        return;
    }
    if (sensor->packet[1] == 0x53) {
        raw = (uint16_t)((uint16_t)sensor->packet[6] | ((uint16_t)sensor->packet[7] << 8));
        signed_raw = raw >= 32768u ? (int32_t)raw - 65536 : raw;
        heading_sensor_publish(sensor, (int16_t)(signed_raw * 18000 / 32768), now_ms);
    }
    sensor->count = 0;
}
