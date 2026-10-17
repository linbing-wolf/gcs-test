#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H
#include <stdint.h>

/* 机械机构会话：5/7号电机、PA1夹爪、PA2载物台、PA3云台。 */
void arm_init(void);
int arm_enabled(void);
int arm_parked(void);
int arm_command(const char *command, uint32_t now);
void arm_update(uint32_t now);
void arm_stop(void);
void arm_can_error(uint8_t flags);
void arm_can_rx(uint32_t id, uint8_t ide, uint8_t rtr,
                uint8_t length, const uint8_t *data, uint32_t now);
/* 沿用入口的USART3文本输出，不另开串口。 */
void arm_output(const char *message);
#endif
