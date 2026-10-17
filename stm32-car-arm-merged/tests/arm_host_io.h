#ifndef ARM_HOST_IO_H
#define ARM_HOST_IO_H
#include <stdint.h>
#include <stdbool.h>
void entry_host_send_byte(uint8_t value);
void entry_host_arm_pwm(uint16_t claw_us,uint16_t yaw_us);
void entry_host_arm_platform_pwm(uint16_t pulse_us);
int entry_host_can_bus_off(void);
void Emm_V5_En_Control(uint8_t address,bool enabled,bool synchronous);
void Emm_V5_Pos_Control(uint8_t address,uint8_t dir,uint16_t rpm,uint8_t acceleration,
                        uint32_t pulses,bool absolute,bool synchronous);
void Emm_V5_Stop_Now(uint8_t address,bool synchronous);
#include "entry_host_io.h"
void Emm_V5_Read_Sys_Params(uint8_t address,SysParams_t parameter);


#endif
