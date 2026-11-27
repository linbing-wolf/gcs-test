#ifndef VISION_LINK_H
#define VISION_LINK_H
#include <stdint.h>
#define VISION_PAYLOAD_MAX 96u /* 覆盖J四颜色报告及有符号坐标；32字节邮箱仍按帧流分片。 */

/* MaixCAM 视觉链路：模拟I2C从机0x42，PG15=SCL / PD7=SDA；每次读写32字节。
 * 邮箱首字节N=0..31，随后N个帧流字节，其余填0；无寄存器地址或SMBus计数字节。
 * 帧格式：AA 55 LEN payload CRC8 0D（CRC8 poly=0x07 init=0x00，范围 LEN+payload）。
 * 自动原料每批握手 T/TASK、定位 H/HDISC2，再串行执行三次 C/PICK。
 * 手动 H 诊断也接受旧 HDISC，手动 J 保留 MSTATE 诊断。
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
/* 使用主入口TIM5毫秒时钟，接收中断只记录时间，不打印日志。 */
uint32_t vision_time_ms(void);
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
/* 主动取消原料会话或未完成的手动 H/J：发一次 R 并等 READY。 */
void vision_cancel_pick(void);
/* 每批原料H、粗加工D、首次暂存E各一次：MOVE,dx前后，MOVEY,dy横移，MOVEXY,dx,dy两轴并行（像素，支持+/-）。
 * 每条位移按接收顺序入队，0不覆盖旧指令；D/E以DISC,x,y,dx,1结束，不清除已接收指令。
 * H可接收HCOLORS预报，由HDISC,x,y,dx,c1,c2,dir或HDISC2,arrival,n[,color,x,y...]结束。
 * HDISC2的arrival为0/1，接受0～4个不同颜色的校准后坐标，不依赖HCOLORS。
 * 不重算dx，不设偏差门槛；正常完成不发R/等READY；ring只用于本地环位编排。 */
void vision_request_align(char command, uint8_t ring, uint32_t now);
/* 自动原料校准。仅 HDISC2 完成，NONE,H 或 ERR 终止并取消会话。 */
void vision_request_raw_align(uint32_t now);
void vision_request_raw_task(const char *code,uint32_t now);
int vision_raw_task_state(void);
int vision_raw_failed(void);
int vision_raw_rx_pending(void);
void vision_finish_raw(void);
int vision_align_state(void);
/* 完整HDISC终包后返回先到/后到两色；预报本身不能放行夹取。 */
int vision_material_colors(uint8_t *first,uint8_t *second);
int vision_material_direction(void); /* HDISC完成后0=顺时针、1=逆时针；否则-1。 */
typedef struct { uint8_t color,state; int16_t x,y; } VisionMaterial;
/* HDISC2就绪时返回arrival=0/1，输出0～4项停稳坐标；其他状态返回-1。 */
int vision_material_h_result(uint8_t *count,VisionMaterial samples[4]);
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
