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
    sensor->rate.valid = 0;
    sensor->acceleration.valid = 0;
    sensor->magnetic.valid = sensor->temperature.valid = sensor->quaternion.valid = 0;
    sensor->count = 0;
}

void heading_sensor_publish_attitude(HeadingSensor *sensor, int16_t roll_cdeg,
                                     int16_t pitch_cdeg, int16_t yaw_cdeg, uint32_t now_ms)
{
    if (yaw_cdeg < -18000 || yaw_cdeg > 18000) {
        heading_sensor_invalidate(sensor);
        return;
    }
    sensor->sample.roll_cdeg = roll_cdeg;
    sensor->sample.pitch_cdeg = pitch_cdeg;
    sensor->sample.yaw_cdeg = heading_wrap_cdeg(yaw_cdeg);
    sensor->sample.received_ms = now_ms;
    sensor->sample.sequence++;
    sensor->sample.valid = 1;
}

void heading_sensor_publish(HeadingSensor *sensor, int16_t yaw_cdeg, uint32_t now_ms)
{
    heading_sensor_publish_attitude(sensor, 0, 0, yaw_cdeg, now_ms);
}

void heading_sensor_publish_rate(HeadingSensor *sensor, int32_t rate_cdeg_s, uint32_t now_ms)
{
    sensor->rate.rate_cdeg_s = rate_cdeg_s;
    sensor->rate.x_cdeg_s=sensor->rate.y_cdeg_s=0;
    sensor->rate.axes_valid=4u;
    sensor->rate.received_ms = now_ms;
    sensor->rate.sequence++;
    sensor->rate.valid = 1;
}

int heading_sample_fresh(const HeadingSample *sample, uint32_t now_ms, uint32_t max_age_ms)
{
    return sample->valid && (uint32_t)(now_ms - sample->received_ms) <= max_age_ms;
}

int heading_rate_fresh(const HeadingRate *rate, uint32_t now_ms, uint32_t max_age_ms)
{
    return rate->valid && (uint32_t)(now_ms - rate->received_ms) <= max_age_ms;
}

int heading_sample_level(const HeadingSample *sample, int16_t max_tilt_cdeg)
{
    return sample->roll_cdeg <= max_tilt_cdeg && sample->roll_cdeg >= -max_tilt_cdeg
        && sample->pitch_cdeg <= max_tilt_cdeg && sample->pitch_cdeg >= -max_tilt_cdeg;
}

static int16_t wit_signed16(uint8_t low, uint8_t high)
{
    uint16_t raw = (uint16_t)((uint16_t)low | ((uint16_t)high << 8));
    return (int16_t)(raw >= 32768u ? (int32_t)raw - 65536 : raw);
}

/* 沿用本项目WIT SDK的普通串口帧：55 TT + 8字节 + 累加校验。
 * 0x51加速度/温度，0x52三轴角速度，0x53姿态，0x54磁场，可选0x59四元数。
 * 各通道只由对应合法帧刷新；磁场/四元数帧不能让旧Yaw或角速度复活。
 */
void heading_sensor_byte(HeadingSensor *sensor, uint8_t byte, uint32_t now_ms)
{
    uint8_t sum = 0, i;
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
        int16_t roll = wit_signed16(sensor->packet[2], sensor->packet[3]);
        int16_t pitch = wit_signed16(sensor->packet[4], sensor->packet[5]);
        int16_t yaw = wit_signed16(sensor->packet[6], sensor->packet[7]);
        heading_sensor_publish_attitude(sensor,
            (int16_t)((int32_t)roll * 18000 / 32768),
            (int16_t)((int32_t)pitch * 18000 / 32768),
            (int16_t)((int32_t)yaw * 18000 / 32768), now_ms);
    } else if (sensor->packet[1] == 0x52) {
        /* 量程±2000度/秒；200000/32768与6250/1024相等，后者不溢出int32。 */
        int32_t rate = (int32_t)wit_signed16(sensor->packet[6], sensor->packet[7]) * 6250 / 1024;
        heading_sensor_publish_rate(sensor, rate, now_ms);
        sensor->rate.x_cdeg_s=(int32_t)wit_signed16(sensor->packet[2],sensor->packet[3])*6250/1024;
        sensor->rate.y_cdeg_s=(int32_t)wit_signed16(sensor->packet[4],sensor->packet[5])*6250/1024;
        sensor->rate.axes_valid=7u;
    } else if(sensor->packet[1]==0x51) {
        sensor->acceleration.x_mg=(int16_t)((int32_t)wit_signed16(sensor->packet[2],sensor->packet[3])*16000/32768);
        sensor->acceleration.y_mg=(int16_t)((int32_t)wit_signed16(sensor->packet[4],sensor->packet[5])*16000/32768);
        sensor->acceleration.z_mg=(int16_t)((int32_t)wit_signed16(sensor->packet[6],sensor->packet[7])*16000/32768);
        sensor->acceleration.received_ms=now_ms;sensor->acceleration.sequence++;sensor->acceleration.valid=1;
        sensor->temperature.cdeg=wit_signed16(sensor->packet[8],sensor->packet[9]);
        sensor->temperature.received_ms=now_ms;sensor->temperature.sequence++;sensor->temperature.valid=1;
    } else if(sensor->packet[1]==0x54) {
        sensor->magnetic.x_lsb=wit_signed16(sensor->packet[2],sensor->packet[3]);
        sensor->magnetic.y_lsb=wit_signed16(sensor->packet[4],sensor->packet[5]);
        sensor->magnetic.z_lsb=wit_signed16(sensor->packet[6],sensor->packet[7]);
        sensor->magnetic.received_ms=now_ms;sensor->magnetic.sequence++;sensor->magnetic.valid=1;
    } else if(sensor->packet[1]==0x59) {
        for(i=0;i<4;i++)sensor->quaternion.q15[i]=wit_signed16(sensor->packet[2+i*2],sensor->packet[3+i*2]);
        sensor->quaternion.received_ms=now_ms;sensor->quaternion.sequence++;sensor->quaternion.valid=1;
    }
    sensor->count = 0;
}
