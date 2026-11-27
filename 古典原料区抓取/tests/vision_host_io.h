#ifndef VISION_HOST_IO_H
#define VISION_HOST_IO_H
#include <stdint.h>
#include "entry_host_io.h"
void entry_host_vision_init(uint32_t baud);
int entry_host_vision_tx_ready(void);
void entry_host_vision_send_byte(uint8_t byte);
#endif
