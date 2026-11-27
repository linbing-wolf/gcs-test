#ifndef DEBUG_RADIO_H
#define DEBUG_RADIO_H
#include <stdint.h>

typedef struct {
    uint32_t received,framing_errors,false_starts,tx_dropped;
} DebugRadioStats;

/* U6.5/PB0 TX、U6.6/PB1 RX，115200 8N1；接收回调在中断中执行。 */
void debug_radio_init(void (*receive)(uint8_t),void (*receive_error)(void));
void debug_radio_send(uint8_t byte);
void debug_radio_stats(DebugRadioStats *stats);
#endif
