#ifndef VISION_LINK_H
#define VISION_LINK_H
#include <stdint.h>
#define VISION_PAYLOAD_MAX 96u /* 覆盖J四颜色报告及有符号坐标；32字节邮箱仍按帧流分片。 */
#define VISION_J_REPEAT_MS 500u /* 合法J过程包done=0后持续重发J，直到合法done=1。 */

/* MaixCAM 视觉链路：模拟I2C从机0x42，PG15=SCL / PD7=SDA；每次读写32字节。
 * 邮箱首字节N=0..31，随后N个帧流字节，其余填0；无寄存器地址或SMBus计数字节。
 * 帧格式：AA 55 LEN payload CRC8 0D（CRC8 poly=0x07 init=0x00，范围 LEN+payload）。
 * 原料H使用HCOLORS/HDISC方向格式；J使用MSTATE停稳报告。
 * C/PICK接口保留为手动诊断，不再是自动原料夹取的前置条件。
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
/* 传输层仅提交有效帧流字节；uart_error沿用既有接口名，非零表示传输错误。 */
void vision_receive_byte(uint8_t byte, uint8_t uart_error);
int vision_command(const char *s, uint32_t now);

/* 发 C,<color> 并进入等待；重复调用会重新起算 20 秒超时。 */
void vision_request_pick(uint8_t color, uint32_t now);
int vision_pick_state(void);
void vision_pick_result(uint8_t *color, int16_t *x, int16_t *y);
/* F,<颜色>接收MOVE/MOVEY/MOVEXY，终包STACK,x,y,dx,ok；兼容旧两坐标终包，不带高度。 */
void vision_request_stack(uint8_t color,uint32_t now);
int vision_stack_state(void);
void vision_stack_result(int16_t *x,int16_t *y);
void vision_cancel_pick(void);
/* 每批原料H、粗加工D、首次暂存E各一次：MOVE,dx前后，MOVEY,dy横移，MOVEXY,dx,dy两轴并行（像素，支持+/-）。
 * 每条位移按接收顺序入队，0不覆盖旧指令；D/E以DISC,x,y,dx,1结束，不清除已接收指令。
 * H接收HCOLORS,c1,c2,dir预报，由HDISC,x,y,dx,c1,c2,dir结束；dir严格0/1，两色不同且1~4。
 * 不重算dx，不设偏差门槛；不发R、不等READY；ring只用于本地环位编排。 */
void vision_request_align(char command, uint8_t ring, uint32_t now);
int vision_align_state(void);
/* 完整HDISC终包后返回先到/后到两色；预报本身不能放行夹取。 */
int vision_material_colors(uint8_t *first,uint8_t *second);
int vision_material_direction(void); /* HDISC完成后0=顺时针、1=逆时针；否则-1。 */
typedef struct { uint8_t color,state; int16_t x,y; } VisionMaterial;
void vision_request_material_state(uint32_t now);
int vision_material_state(void);
/* 最新J报告，最多4项；state为0运动、1停稳、2证据不足。 */
uint8_t vision_material_state_result(VisionMaterial samples[4]);
int16_t vision_align_offset_px(void);
/* axis=0前后、1横移、2双轴；旧轴查询只读取单轴元素，不拆开MOVEXY。
 * WAITING/READY均可读取。peek不消费，启动成功后take。
 * 兼容轴查询读取本轴最早未执行项；取消/新请求清空指令队列。 */
/* H/D/E/F共用离散指令FIFO；启动成功后才消费，终包不清除未执行项。 */
int vision_motion_peek(uint8_t *axis,int16_t *offset,uint32_t *received_ms);
/* 工位统一使用本接口，MOVE/MOVEY缺省的另一轴为0；MOVEXY一次返回完整两轴。 */
int vision_motion_peek_xy(int16_t *dx,int16_t *dy,uint32_t *received_ms);
void vision_motion_take(void);
int vision_align_peek_axis(uint8_t axis,int16_t *offset,uint32_t *received_ms);
int vision_align_take_axis(uint8_t axis,int16_t *offset,uint32_t *received_ms);
/* 兼容既有前后轴调用。 */
int vision_align_take_move(int16_t *offset, uint32_t *received_ms);
const char *vision_last_message(void);
/* 最近真实观测仅供显示，不消耗MOVE，不修改视觉状态或完成判据。 */
void vision_display(uint32_t now,int32_t *color,int32_t *dx,int32_t *dy,int32_t *age);

#endif
