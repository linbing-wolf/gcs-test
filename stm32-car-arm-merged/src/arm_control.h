#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H
#include <stdint.h>

/* 机械机构会话：5/7号电机、PA1夹爪、PA2载物台、PA3云台。 */
void arm_init(void);
int arm_enabled(void);
int arm_parked(void);
/* 高位收回后，仅末尾云台/载物台回正尚在插值，可与离站预检和转弯重叠。 */
int arm_departure_rotation_active(void);
/* 当前动作两轴已实际到135/0且不会再移动，只剩末尾旋转；可衔接底盘平移。 */
int arm_translation_overlap_ready(void);
/* 地面取放后的抬升已通过原有实际净空判据；仅供已编排的工位索引衔接。 */
int arm_station_clearance_ready(uint32_t now);
int arm_pose_known(void);
int arm_command(const char *command, uint32_t now);
void arm_update(uint32_t now);
void arm_stop(void);
int arm_busy(void);
int arm_failed(void);
int arm_platform_known(void);
float arm_platform_angle(void);
int arm_ready(void);
int arm_platform_slot(void);
/* 工位内部换盘：调用方须持有停车互锁；仅在高位收回且云台等于指定作业角时接受。
 * 普通arm platform命令仍要求车头0度，不放宽手动接口。 */
int arm_platform_at_yaw(uint8_t slot, float work_yaw, uint32_t now);
/* 工位高位收回时，载物台选杯与云台转向共用一次插值；完成后才可伸出/下降。 */
int arm_platform_with_yaw(uint8_t slot, float target_yaw, uint32_t now);
/* 高位收回起步：平推、云台、换杯、空爪与下降到lift目标并行。
 * 车右侧直接传最终抓放高度，车载杯传净空高度后再下降；slot=0保持载物台。 */
int arm_high_approach(int slide, int lift, float yaw, uint8_t slot, uint8_t open_claw, uint32_t now);
/* 连续抬升至最高处，实际位置达到净空高度后启动收回/转向/可选选杯。
 * 不在净空高度另停一次，保持夹爪；调用方先完成合爪保持或松爪。 */
int arm_retract_with_yaw(float yaw, uint8_t slot, uint32_t now);
/* 从离杯净空姿态直接到下一个取放姿态，升降/平推/云台同时执行，保持夹爪。 */
int arm_transfer_to(int slide, int lift, float yaw, uint8_t slot, uint32_t now);
/* 显示指令目标；不把舵机插值估计当作实测到位。 */
void arm_target_now(int *slide,int *lift,float *yaw,float *claw);
void arm_pose_now(int *slide, int *lift, float *yaw, float *claw);
void arm_can_error(uint8_t flags);
void arm_can_rx(uint32_t id, uint8_t ide, uint8_t rtr,
                uint8_t length, const uint8_t *data, uint32_t now);
/* 沿用入口的USART3文本输出，不另开串口。 */
void arm_output(const char *message);
#endif
