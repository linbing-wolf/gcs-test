#ifndef HEADING_SENSOR_H
#define HEADING_SENSOR_H
#include <stdint.h>

/* 角度统一为0.01度，逆时针为正；外部适配器只发布真实的新采样。 */
typedef struct {
    int16_t yaw_cdeg;
    int16_t roll_cdeg, pitch_cdeg; /* 与yaw同一0x53帧；用于姿态诊断。 */
    uint32_t received_ms, sequence;
    uint8_t valid;
} HeadingSample;

/* 0x52角速度帧单独发布：不刷新角度时效，角度帧也不能让旧角速度复活。 */
typedef struct {
    int32_t rate_cdeg_s;           /* 绕Z轴角速度，0.01度/秒；保留协议±2000度/秒量程。 */
    int32_t x_cdeg_s,y_cdeg_s;      /* 传感器X/Y轴；不应用车体轴映射或Yaw符号。 */
    uint32_t received_ms, sequence;
    uint8_t valid,axes_valid;       /* XYZ有效位1/2/4；仅Z适配器不能伪装为完整三轴。 */
} HeadingRate;
typedef struct {
    int16_t x_mg,y_mg,z_mg; /* 机体系原始加速度，含重力；只读诊断，不积分进位置环。 */
    uint32_t received_ms,sequence;
    uint8_t valid;
} HeadingAcceleration;

typedef struct {
    int16_t x_lsb,y_lsb,z_lsb; /* 协议原始磁场LSB，不冒充微特斯拉。 */
    uint32_t received_ms,sequence;
    uint8_t valid;
} HeadingMagnetic;
typedef struct {
    int16_t cdeg; /* 温度0.01摄氏度，来自0x51；不是0x53尾部的版本字段。 */
    uint32_t received_ms,sequence;
    uint8_t valid;
} HeadingTemperature;
typedef struct {
    int16_t q15[4]; /* 可选0x59姿态解算输出，q0..q3，各分量除32768；不是新传感器轴。 */
    uint32_t received_ms,sequence;
    uint8_t valid;
} HeadingQuaternion;

typedef struct {
    volatile HeadingSample sample;
    volatile HeadingRate rate;
    volatile HeadingAcceleration acceleration;
    volatile HeadingMagnetic magnetic;
    volatile HeadingTemperature temperature;
    volatile HeadingQuaternion quaternion;
    uint8_t packet[11], count;
    volatile uint32_t checksum_errors;
} HeadingSensor;

void heading_sensor_reset(HeadingSensor *sensor);
/* 只含角度的发布入口；姿态角按0处理，供测试和纯航向适配器使用。 */
void heading_sensor_publish(HeadingSensor *sensor, int16_t yaw_cdeg, uint32_t now_ms);
void heading_sensor_publish_attitude(HeadingSensor *sensor, int16_t roll_cdeg,
                                     int16_t pitch_cdeg, int16_t yaw_cdeg, uint32_t now_ms);
void heading_sensor_publish_rate(HeadingSensor *sensor, int32_t rate_cdeg_s, uint32_t now_ms);
void heading_sensor_byte(HeadingSensor *sensor, uint8_t byte, uint32_t now_ms);
void heading_sensor_invalidate(HeadingSensor *sensor);
int heading_sample_fresh(const HeadingSample *sample, uint32_t now_ms, uint32_t max_age_ms);
int heading_rate_fresh(const HeadingRate *rate, uint32_t now_ms, uint32_t max_age_ms);
/* 横滚/俯仰都在阈值内；用于倾斜诊断和Z轴角速度的使用条件。 */
int heading_sample_level(const HeadingSample *sample, int16_t max_tilt_cdeg);
int16_t heading_wrap_cdeg(int32_t angle);
#endif
