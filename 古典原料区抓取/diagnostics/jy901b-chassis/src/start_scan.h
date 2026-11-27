#ifndef START_SCAN_H
#define START_SCAN_H
#include "heading_sensor.h"

typedef struct {
    int (*turn_start)(int16_t relative_cdeg, uint16_t rpm);
    int (*turn_state)(void); /* 1进行中、0已按原反馈判据完成、-1尚无完成依据。 */
    HeadingSample (*heading)(void);
    void (*write)(const char *text);
} StartScanIO;

void start_scan_init(const StartScanIO *io);
int start_scan_command(const char *command, uint32_t now_ms);
void start_scan_update(uint32_t now_ms);
int start_scan_active(void);
void start_scan_cancel(void); /* 只取消扫描；整车停车仍由主入口的人工停止处理。 */
void start_scan_echo_irq(void); /* 与视觉EXTI15共享向量，只处理EXTI10。 */

#ifdef ARM_HOST_TEST
void start_scan_test_time(uint32_t microseconds);
void start_scan_test_edge(uint8_t level, uint32_t microseconds);
#endif
#endif
