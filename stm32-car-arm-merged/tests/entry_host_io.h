#ifndef ENTRY_HOST_IO_H
#define ENTRY_HOST_IO_H
#include <stdint.h>
#include <stdbool.h>
/* 仅电脑行为测试使用；ARM 构建不启用 NAV_ENTRY_HOST_TEST。 */
void entry_host_send_byte(uint8_t value);
uint8_t entry_host_start_button_pressed(void);
void entry_host_scanner_init(uint32_t baud);
int entry_host_scanner_tx_ready(void);
int entry_host_scanner_tx_complete(void);
void entry_host_scanner_send_byte(uint8_t value);
void delay_ms(uint16_t milliseconds);
void Emm_V5_Pos_Control(uint8_t addr,uint8_t dir,uint16_t rpm,uint8_t acc,
                        uint32_t pulses,bool absolute,bool synchronous);
void Emm_V5_Synchronous_motion(uint8_t addr);
void Emm_V5_Stop_Now(uint8_t addr,bool synchronous);
void Emm_V5_Vel_Control(uint8_t addr,uint8_t dir,uint16_t rpm,uint8_t acc,bool synchronous);
typedef enum {S_VEL=9,S_CPOS=10,S_PERR=11,S_FLAG=13} SysParams_t;
void Emm_V5_Read_Sys_Params(uint8_t addr,SysParams_t parameter);
int entry_host_can_bus_off(void);
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
#endif
