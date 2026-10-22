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
/* F,<颜色>接收MOVE/MOVEY，终包STACK,x,y,dx,ok；兼容旧两坐标终包，不带高度。 */
void vision_request_stack(uint8_t color,uint32_t now);
int vision_stack_state(void);
void vision_stack_result(int16_t *x,int16_t *y);
void vision_cancel_pick(void);
/* 粗加工D、首次暂存E各一次：MOVE,dx前后，MOVEY,dy横移（像素，支持+/-）。
 * 每条位移按接收顺序入队，0不覆盖旧指令；DISC,x,y,dx,1结束接收，不清除已接收指令。
 * 不重算dx，不设偏差门槛；不发R、不等READY；ring只用于本地环位编排。 */
void vision_request_align(char command, uint8_t ring, uint32_t now);
int vision_align_state(void);
int16_t vision_align_offset_px(void);
/* axis=0前后、1横移；WAITING/READY均可读取。peek不消费，启动成功后take。
 * 兼容轴查询读取本轴最早未执行项；取消/新请求清空指令队列。 */
/* D/E/F共用离散指令FIFO；启动成功后才消费，终包不清除未执行项。 */
int vision_motion_peek(uint8_t *axis,int16_t *offset,uint32_t *received_ms);
void vision_motion_take(void);
int vision_align_peek_axis(uint8_t axis,int16_t *offset,uint32_t *received_ms);
int vision_align_take_axis(uint8_t axis,int16_t *offset,uint32_t *received_ms);
/* 兼容既有前后轴调用。 */
int vision_align_take_move(int16_t *offset, uint32_t *received_ms);
const char *vision_last_message(void);
/* 最近真实观测仅供显示，不消耗MOVE，不修改视觉状态或完成判据。 */
void vision_display(uint32_t now,int32_t *color,int32_t *dx,int32_t *dy,int32_t *age);

#endif
