#ifndef VISION_LINK_H
#define VISION_LINK_H
#include <stdint.h>

/* MaixCAM 视觉链路：UART5，PD2=RX / PC12=TX，115200 8N1。
 * 帧格式：AA 55 LEN payload CRC8 0D（CRC8 poly=0x07 init=0x00，范围 LEN+payload）。
 * 原料区由上位机在物块转到固定取料口后回 PICK,<颜色>,x,y；
 * 视觉端不会主动重传，超时后由调用方决定是否重新发 C,<颜色>。
 */
enum {
    VISION_PICK_IDLE = 0,
    VISION_PICK_WAITING = 1,
    VISION_PICK_READY = 2,
    VISION_PICK_TIMEOUT = 3
};

void vision_init(void);
void vision_update(uint32_t now);
int vision_command(const char *s, uint32_t now);

/* 发 C,<color> 并进入等待；重复调用会重新起算 20 秒超时。 */
void vision_request_pick(uint8_t color, uint32_t now);
int vision_pick_state(void);
void vision_pick_result(uint8_t *color, int16_t *x, int16_t *y);
void vision_cancel_pick(void);
const char *vision_last_message(void);

#endif
