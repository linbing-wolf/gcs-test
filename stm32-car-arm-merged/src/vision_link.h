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
    VISION_PICK_TIMEOUT = 3,
    VISION_PICK_FAULT = 4
};

void vision_init(void);
void vision_update(uint32_t now);
/* UART5中断只提交字节；uart_error非零时丢弃本轮请求，主循环处理故障。 */
void vision_receive_byte(uint8_t byte, uint8_t uart_error);
int vision_command(const char *s, uint32_t now);

/* 发 C,<color> 并进入等待；重复调用会重新起算 20 秒超时。 */
void vision_request_pick(uint8_t color, uint32_t now);
int vision_pick_state(void);
void vision_pick_result(uint8_t *color, int16_t *x, int16_t *y);
/* F,<颜色>返回STACK,x,y；不带高度。结束/急停沿用vision_cancel_pick取消全部任务。 */
void vision_request_stack(uint8_t color,uint32_t now);
int vision_stack_state(void);
void vision_stack_result(int16_t *x,int16_t *y);
void vision_cancel_pick(void);
/* 每次进入作业区只请求一次（粗加工D、暂存E）：持续接收MOVE,dx（像素，支持+/-，正值后退、负值前进），格式正确的DISC,x,y,dx,1置READY，不设偏差幅值或dx与cx的一致性门槛。
 * 自动对准不发R、不等READY；ring仅用于本地环位编排。 */
void vision_request_align(char command, uint8_t ring, uint32_t now);
int vision_align_state(void);
/* 最近MOVE像素偏差，距离换算由工位控制处理。 */
int16_t vision_align_offset_px(void);
/* 一次性取走最新MOVE像素值及接收时刻，0覆盖未执行指令；不累计排队。移车期间的旧数据仍丢弃。 */
int vision_align_take_move(int16_t *offset, uint32_t *received_ms);
const char *vision_last_message(void);
/* 最近真实观测仅供显示，不消耗MOVE，不修改视觉状态或完成判据。 */
void vision_display(uint32_t now,int32_t *color,int32_t *dx,int32_t *dy,int32_t *age);

#endif
