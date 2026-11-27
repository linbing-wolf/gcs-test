#include "vision_link.h"
#include "raw_region_config.h"
#include "start_scan.h"
#include "arm_control.h"
#ifdef ARM_HOST_TEST
#include "../tests/vision_host_io.h"
#else
#include "stm32f4xx.h"
#endif
#include <string.h>
#include <stdio.h>

#define VISION_RX_RING          256u
#define VISION_FRAME_OVERHEAD     5u /* AA 55 LEN ... CRC 0D */
#define VISION_HALF_FRAME_MS    200u
#define VISION_PICK_TIMEOUT_MS 20000u
#define VISION_ALIGN_TIMEOUT_MS 25000u
#define VISION_RAW_RX_RECOVERY_MS 1000u
#define VISION_MOTION_QUEUE      32u /* 有界离散指令队列；MOVEXY两轴整体占1条，终包不清空。 */
#define VISION_RX_ERROR        0x100u
#define VISION_RX_MAILBOX      0x200u
#ifndef ARM_HOST_TEST
static void vision_receive_mailbox(const uint8_t *bytes,uint8_t count);
#endif

#ifndef ARM_HOST_TEST
#include "vision_soft_i2c.h"
#endif

typedef struct {
    uint32_t received_ms;
    int16_t pixels[2];
    uint8_t axis;
} VisionMotion;

enum { ALIGN_IDLE = 0, ALIGN_MEASURE = 2 };

/* 容量仍为256项；时间和错误/邮箱边界元数据用于按实际到达顺序组帧。 */
static volatile uint16_t rx_ring[VISION_RX_RING];
static volatile uint32_t rx_received_ms[VISION_RX_RING];
static volatile uint16_t rx_head, rx_tail;
static volatile uint8_t rx_fault;
static volatile uint32_t rx_transport_errors,rx_ring_overflows;
static uint8_t tx_frame[VISION_PAYLOAD_MAX + VISION_FRAME_OVERHEAD];
static uint8_t tx_length, tx_offset;
static uint8_t cancel_pending; /* R只发送一次，主机读走后仍须等READY才允许下一任务。 */
static char deferred_request[18]; /* 最长 T,ddd+ddd+ddd+ddd，包含结束符。 */
static struct {
    uint8_t active,failed,task_state,rx_pending,move_seen;
    uint32_t task_ms,rx_error_ms;
    char expected_task[13]; /* TASK,ddd,ddd */
} raw;

static int vision_tx_complete(void)
{
    if(tx_offset<tx_length)return 0;
#ifndef ARM_HOST_TEST
    return vision_i2c.head==vision_i2c.tail; /* 已排队不等于主机已读走。 */
#else
    return 1;
#endif
}

static struct {
    uint8_t frame[VISION_PAYLOAD_MAX + VISION_FRAME_OVERHEAD];
    uint8_t length;
    uint32_t last_byte_ms;
    uint8_t pick_state;
    uint8_t request_color, pick_color;
    uint8_t stack_state, stack_color;
    int16_t stack_x, stack_y;
    uint32_t stack_warning_ms;
    uint8_t align_state, align_stage, motion_count;
    char align_command;
    uint8_t hdisc2_required; /* 自动原料作业；手动 H 诊断仍可解析旧方向包。 */
    uint8_t material_colors[2]; /* 旧 H 的先到/后到两色，供诊断。 */
    uint8_t material_direction; /* 0顺时针、1逆时针、2尚无有效方向。 */
    uint8_t material_state,material_count;
    uint8_t h_arrival; /* 0无HDISC2结果，1/2分别表示arrival=0/1；H/J互斥共用materials。 */
    VisionMaterial materials[4];
    uint32_t material_warning_ms;
    VisionMotion motion[VISION_MOTION_QUEUE];
    int16_t align_error, align_offset[2];
    uint32_t align_ms;
    int16_t pick_x, pick_y;
    uint32_t request_ms, pick_warning_ms, align_warning_ms;
    uint8_t busy_reported, mismatch_reported;
    uint8_t display_known,display_color;
    int16_t display_dx,display_dy;
    uint32_t display_ms;
    char message[VISION_PAYLOAD_MAX + 1u];
    uint32_t frames, crc_errors, bad_frames, overflows, timeouts;
    char line[256]; /* 中文UTF-8状态和最长协议原文。 */
} vision;

static struct {
    uint32_t id,start_ms,delivered_ms,last_chunk_ms;
    uint32_t frames,crc,bad,transport,ring_full;
    uint32_t chunks,bytes,half_frames,ignored,orphans;
    uint8_t delivered;
#ifndef ARM_HOST_TEST
    uint32_t reads,writes,invalid,bus_errors;
#endif
} j_diag;

static void vision_j_diagnostics(uint32_t now)
{
    snprintf(vision.line,sizeof(vision.line),
        "[vision J diag] local=%lu start_ms=%lu delivered=%u delivered_ms=%lu elapsed_ms=%lu rx_pending=%u\r\n",
        (unsigned long)j_diag.id,(unsigned long)j_diag.start_ms,(unsigned)j_diag.delivered,
        (unsigned long)j_diag.delivered_ms,(unsigned long)(now-j_diag.start_ms),
        (unsigned)((rx_head-rx_tail+VISION_RX_RING)%VISION_RX_RING));
    arm_output(vision.line);
    snprintf(vision.line,sizeof(vision.line),
        "[vision J delta] frames=%lu crc=%lu bad=%lu transport=%lu ring_full=%lu half=%lu chunks=%lu bytes=%lu ignored=%lu\r\n",
        (unsigned long)(vision.frames-j_diag.frames),(unsigned long)(vision.crc_errors-j_diag.crc),
        (unsigned long)(vision.bad_frames-j_diag.bad),(unsigned long)(rx_transport_errors-j_diag.transport),
        (unsigned long)(rx_ring_overflows-j_diag.ring_full),(unsigned long)j_diag.half_frames,
        (unsigned long)j_diag.chunks,(unsigned long)j_diag.bytes,(unsigned long)j_diag.ignored);
    arm_output(vision.line);
    snprintf(vision.line,sizeof(vision.line),"[vision J parser] have=%u need=%u last_byte_ms=%lu orphans=%lu\r\n",
        (unsigned)vision.length,vision.length>=3u ? (unsigned)vision.frame[2]+VISION_FRAME_OVERHEAD : 0u,
        (unsigned long)vision.last_byte_ms,(unsigned long)j_diag.orphans);
    arm_output(vision.line);
#ifndef ARM_HOST_TEST
    snprintf(vision.line,sizeof(vision.line),
        "[vision J i2c delta] reads=%lu writes=%lu invalid=%lu bus_err=%lu last_error_ms=%lu phase=%u bits=%u rx_pos=%u\r\n",
        (unsigned long)(vision_i2c.reads-j_diag.reads),(unsigned long)(vision_i2c.writes-j_diag.writes),
        (unsigned long)(vision_i2c.invalid-j_diag.invalid),(unsigned long)(vision_i2c.bus_errors-j_diag.bus_errors),
        (unsigned long)vision_i2c.last_error_ms,(unsigned)vision_i2c.error_phase,
        (unsigned)vision_i2c.error_bits,(unsigned)vision_i2c.error_rx_pos);
    arm_output(vision.line);
#endif
}

static void vision_j_drop(const char *reason,uint32_t at_ms)
{
    if(vision.material_state!=VISION_PICK_WAITING)return;
    snprintf(vision.line,sizeof(vision.line),
        "[vision J drop] local=%lu at_ms=%lu reason=%s have=%u need=%u\r\n",
        (unsigned long)j_diag.id,(unsigned long)at_ms,reason,(unsigned)vision.length,
        vision.length>=3u ? (unsigned)vision.frame[2]+VISION_FRAME_OVERHEAD : 0u);
    arm_output(vision.line);
}

void vision_display(uint32_t now,int32_t *color,int32_t *dx,int32_t *dy,int32_t *age)
{
    uint32_t elapsed=now-vision.display_ms;
    *age=vision.display_known ? (elapsed>0x7fffffffu ? 0x7fffffff : (int32_t)elapsed) : -1;
    *color=vision.display_known && elapsed<=1000u ? vision.display_color : 0;
    *dx=vision.display_dx;*dy=vision.display_dy;
}
static void vision_observed(uint8_t color,int x,int y,uint32_t now)
{
    vision.display_known=1;vision.display_color=color;
    vision.display_dx=(int16_t)x;vision.display_dy=(int16_t)y;vision.display_ms=now;
}
static uint8_t vision_crc8(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0;
    uint16_t i;
    uint8_t bit;
    for (i = 0; i < length; i++) {
        crc ^= data[i];
        for (bit = 0; bit < 8u; bit++)
            crc = (uint8_t)((crc & 0x80u) ? ((uint8_t)(crc << 1) ^ 0x07u) : (uint8_t)(crc << 1));
    }
    return crc;
}

static void vision_receive_event(uint16_t event,uint32_t at_ms)
{
    uint16_t next=(uint16_t)((rx_head+1u)%VISION_RX_RING);
    if(next==rx_tail){rx_fault|=2u;rx_ring_overflows++;return;}
    rx_ring[rx_head]=event;rx_received_ms[rx_head]=at_ms;rx_head=next;
}
void vision_receive_byte(uint8_t byte, uint8_t uart_error)
{
    if(uart_error){rx_fault|=1u;rx_transport_errors++;}
    vision_receive_event(uart_error ? (uint16_t)(VISION_RX_ERROR|uart_error) : byte,vision_time_ms());
}
#ifndef ARM_HOST_TEST
static void vision_receive_mailbox(const uint8_t *bytes,uint8_t count)
{
    uint32_t at_ms=vision_time_ms();unsigned i;
    if(count)vision_receive_event((uint16_t)(VISION_RX_MAILBOX|count),at_ms);
    for(i=0;i<count;i++)vision_receive_event(bytes[i],at_ms);
}
#endif

/* 发送分次排入传输层；主机未取走时保留待发数据，不阻塞主循环。 */
static int vision_send(const char *payload)
{
    size_t size = strlen(payload);
    uint8_t length = (uint8_t)size;
    if (size > VISION_PAYLOAD_MAX || tx_offset < tx_length) return 0;
    tx_frame[0] = 0xAAu;tx_frame[1] = 0x55u;tx_frame[2] = length;
    memcpy(&tx_frame[3],payload,length);
    tx_frame[3u + length] = vision_crc8(&tx_frame[2],(uint16_t)(length + 1u));
    tx_frame[4u + length] = 0x0Du;
    tx_offset = 0;tx_length = (uint8_t)(length + VISION_FRAME_OVERHEAD);
    return 1;
}
static void vision_transmit(void)
{
    if (tx_offset >= tx_length) return;
#ifdef ARM_HOST_TEST
    if (!entry_host_vision_tx_ready()) return;
    entry_host_vision_send_byte(tx_frame[tx_offset++]);
#else
    if (!vision_i2c_tx_ready()) return;
    vision_i2c_send_byte(tx_frame[tx_offset++]);
#endif
}

static int vision_send_request(const char *payload)
{
    if(cancel_pending) {
        strcpy(deferred_request,payload);
        arm_output("[vision] next request retained; wait for R/READY cancellation\r\n");
        return 1;
    }
    return vision_send(payload);
}

static int vision_number_separator(const char **cursor, int *value, char separator)
{
    const char *s = *cursor;
    int sign = 1, found = 0;
    long result = 0;
    if (*s != separator && !(separator==' ' && *s=='\t')) return 0;
    s++;
    while (*s == ' ' || *s == '\t') s++;
    if (*s == '-' || *s == '+') {if (*s == '-') sign = -1;s++;}
    while (*s >= '0' && *s <= '9') {
        if (result > 3276 || (result == 3276 && *s - '0' > (sign < 0 ? 8 : 7))) return 0;
        result = result * 10 + (*s++ - '0');found = 1;
    }
    if (!found || (*s && *s != separator && !(separator==' ' && *s=='\t'))) return 0;
    *value = (int)(sign * result);*cursor = s;
    return 1;
}

static int vision_number(const char **cursor, int *value)
{
    return vision_number_separator(cursor,value,',');
}

/* 方向字段必须是最后一个单字符ASCII数字，拒绝+1、01、空格和多余字段。 */
static int vision_direction(const char **cursor,int *direction)
{
    const char *s=*cursor;
    if(*s!=',' || (s[1]!='0' && s[1]!='1') || s[2])return 0;
    *direction=s[1]-'0';*cursor=s+2;return 1;
}

static int vision_h_number(const char **cursor,int max,int *value)
{
    const char *s=*cursor;
    if(*s++!=',' || s[0]<'0' || s[0]>'9')return 0;
    if(s[0]=='0' && s[1]>='0' && s[1]<='9')return 0;
    return vision_number(cursor,value) && *value<=max;
}

static int vision_hdisc2(const char *cursor)
{
    VisionMaterial samples[4];
    int arrival,count,i;
    unsigned colors=0;
    if(!vision_h_number(&cursor,1,&arrival) || !vision_h_number(&cursor,4,&count))return 0;
    for(i=0;i<count;i++) {
        int color,x,y;
        if(!vision_h_number(&cursor,4,&color) || !color || (colors&(1u<<color))
            || !vision_h_number(&cursor,RAW_J_IMAGE_WIDTH-1,&x)
            || !vision_h_number(&cursor,RAW_J_IMAGE_HEIGHT-1,&y))return 0;
        colors|=1u<<color;
        samples[i]=(VisionMaterial){(uint8_t)color,1,(int16_t)x,(int16_t)y};
    }
    if(*cursor)return 0;
    vision.h_arrival=(uint8_t)(arrival+1);
    vision.material_count=(uint8_t)count;
    memcpy(vision.materials,samples,(size_t)count*sizeof(samples[0]));
    vision.material_colors[0]=vision.material_colors[1]=0;
    vision.material_direction=2u;
    return 1;
}

static int vision_j_packet_is_current(void)
{
#ifndef ARM_HOST_TEST
    uint32_t delivered_ms=vision_i2c.tx_drained_ms;
#else
    uint32_t delivered_ms=j_diag.delivered_ms;
#endif
    return (int32_t)(vision.last_byte_ms-delivered_ms)>=0;
}

static void vision_raw_fault(const char *reason)
{
    if(!raw.active)return;
    snprintf(vision.line,sizeof(vision.line),"[vision raw fault] %s\r\n",reason);
    arm_output(vision.line);
    vision_cancel_pick();
    raw.failed=1;
}

static void vision_raw_damaged(uint32_t now)
{
    if(!raw.active)return;
    if(raw.rx_pending)vision_raw_fault("receive error repeated");
    else {raw.rx_pending=1;raw.rx_error_ms=now;}
}

static int vision_reply_after_delivery(void)
{
    if(!vision_tx_complete())return 0;
#ifndef ARM_HOST_TEST
    return (int32_t)(vision.last_byte_ms-vision_i2c.tx_drained_ms)>=0;
#else
    return 1;
#endif
}

static void vision_handle(const uint8_t *payload, uint8_t length, uint32_t now)
{
    char message[VISION_PAYLOAD_MAX + 1u];
    const char *cursor;
    int value, x = 0, y = 0, has_coordinates = 0;
    if (memchr(payload,0,length)) {vision.bad_frames++;vision_raw_fault("embedded NUL in payload");return;}
    memcpy(message, payload, length);
    message[length] = 0;
    memcpy(vision.message, message, (size_t)length + 1u);
    vision.frames++;
    if(cancel_pending) {
        if(!strcmp(message,"READY") && vision_tx_complete()) {
            cancel_pending=0;
            arm_output("[vision] R acknowledged by READY; cancellation complete\r\n");
            if(deferred_request[0]) {
                (void)vision_send(deferred_request);
                deferred_request[0]=0;
                if(raw.active && raw.task_state==VISION_PICK_WAITING)raw.task_ms=now;
            }
        }
        return; /* READY前的旧H/J结果不能放行已取消或尚未发送的新任务。 */
    }
    if(raw.active) {
        if(!strcmp(message,"READY")) {vision_raw_fault("unexpected READY; camera restarted");return;}
        if(!strcmp(message,"BUSY") || !strncmp(message,"BUSY,",5))return;
        if(!vision_reply_after_delivery())return;
        if(!strncmp(message,"ERR",3) || !strncmp(message,"TIMEOUT",7)
            || !strcmp(message,"NONE,H") || !strcmp(message,"NONE,C")) {
            vision_raw_fault(message);return;
        }
        if(raw.task_state==VISION_PICK_WAITING) {
            if(!strncmp(message,"TASK",4)) {
                if(strcmp(message,raw.expected_task))vision_raw_fault("TASK colors/order mismatch");
                else {raw.task_state=VISION_PICK_READY;raw.rx_pending=0;}
            }
            return;
        }
        if(!strncmp(message,"HCOLORS,",8) && vision.material_colors[0]) {
            vision.bad_frames++;arm_output("[vision warning] duplicate HCOLORS ignored\r\n");return;
        }
        if(!strncmp(message,"MSTATE,",7))return;
        if(vision.pick_state==VISION_PICK_WAITING && !strcmp(message,"PICK")) {
            vision_raw_fault("incomplete PICK");return;
        }
        if(vision.align_state==VISION_PICK_WAITING && (!strcmp(message,"HDISC2") || !strncmp(message,"HDISC,",6))) {
            vision_raw_fault("expected complete HDISC2");return;
        }
    }
    if(!strncmp(message,"MSTATE,",7)) {
        int done,count,all_still=1;
        unsigned colors=0;
        VisionMaterial samples[4];
        unsigned i;
        if(vision.material_state!=VISION_PICK_WAITING || !vision_tx_complete()) {
            j_diag.ignored++;
            snprintf(vision.line,sizeof(vision.line),"[vision J ignored] local=%lu state=%u delivered=%u; %s\r\n",
                (unsigned long)j_diag.id,(unsigned)vision.material_state,(unsigned)vision_tx_complete(),message);
            arm_output(vision.line);return;
        }
        if(!vision_j_packet_is_current()) {
            j_diag.ignored++;
            arm_output("[vision J ignored] reason=arrived_before_request_delivery; request retained\r\n");return;
        }
        cursor=message+6;
        if(!vision_number(&cursor,&done) || !vision_number(&cursor,&count)
            || done<0 || done>1 || count<0 || count>4)goto invalid_material;
        for(i=0;i<(unsigned)count;i++) {
            int color,cx,cy,state;
            if(!vision_number(&cursor,&color) || !vision_number(&cursor,&cx)
                || !vision_number(&cursor,&cy) || !vision_number(&cursor,&state)
                || color<1 || color>4 || state<0 || state>2)goto invalid_material;
            if(colors&(1u<<color))goto invalid_material;
            colors|=1u<<color;
            samples[i].color=(uint8_t)color;samples[i].state=(uint8_t)state;
            samples[i].x=(int16_t)cx;samples[i].y=(int16_t)cy;
            if(state!=1)all_still=0;
        }
        if(*cursor || (done && (!count || !all_still)))goto invalid_material;
        vision.material_count=(uint8_t)count;
        memcpy(vision.materials,samples,(size_t)count*sizeof(samples[0]));
        if(done)vision.material_state=VISION_PICK_READY;
        snprintf(vision.line,sizeof(vision.line),"[vision J] %s\r\n",message);arm_output(vision.line);
        return;
invalid_material:
        vision.bad_frames++;
        snprintf(vision.line,sizeof(vision.line),"[vision warning] invalid J MSTATE; local=%lu request retained; %s\r\n",
            (unsigned long)j_diag.id,message);arm_output(vision.line);return;
    }
    if(vision.material_state==VISION_PICK_WAITING) {
        if(!vision_tx_complete() || !vision_j_packet_is_current())return;
        if(!strncmp(message,"ERR,",4)) {
            snprintf(vision.line,sizeof(vision.line),"[vision J fault] %s\r\n",message);
            arm_output(vision.line);
            vision_cancel_pick();
            vision.material_state=VISION_PICK_FAULT;
        } else if(!strcmp(message,"BUSY") || !strncmp(message,"NONE",4))
            arm_output("[vision warning] J unavailable/busy; request retained\r\n");
        return; /* J只报告状态，不能从其会话接入车辆MOVE或旧H终包。 */
    }
    if ((vision.align_state==VISION_PICK_WAITING || vision.stack_state==VISION_PICK_WAITING)
        && vision_tx_complete()
        && (!strncmp(message,"MOVE,",5) || !strncmp(message,"MOVEY,",6)
            || !strncmp(message,"MOVEXY,",7) || !strncmp(message,"MOVEXY ",7)
            || !strncmp(message,"MOVEXY\t",7) || !strncmp(message,"movexy,",7)
            || !strncmp(message,"movexy ",7) || !strncmp(message,"movexy\t",7))) {
        int offsets[2]={0,0};
        uint8_t axis=(uint8_t)(message[4]=='Y' ? 1 : (message[4]=='X' || message[4]=='x') ? 2 : 0);
        VisionMotion *motion;
        cursor=message+(axis==2 ? 6 : axis ? 5 : 4);
        if (axis==2) {
            char separator=*cursor==',' ? ',' : ' ';
            if (!vision_number_separator(&cursor,&offsets[0],separator)
                || !vision_number_separator(&cursor,&offsets[1],separator)) goto invalid_motion;
        } else if (!vision_number(&cursor,&offsets[axis])) goto invalid_motion;
        if (*cursor) goto invalid_motion;
        if(raw.active && vision.align_command=='H') {
            if(axis!=2 || raw.move_seen) {
                vision.bad_frames++;vision_raw_fault("expected one H MOVEXY");return;
            }
            raw.move_seen=1;raw.rx_pending=0;
        }
        if (vision.motion_count>=VISION_MOTION_QUEUE) {
            vision.overflows++;
            arm_output("[vision warning] motion queue full; incoming MOVE/MOVEY/MOVEXY not queued\r\n");return;
        }
        /* 两轴同帧作为一个队列元素；不能拆成两个独立动作或覆盖正在执行的指令。 */
        motion=&vision.motion[vision.motion_count++];
        motion->axis=axis;motion->pixels[0]=(int16_t)offsets[0];motion->pixels[1]=(int16_t)offsets[1];
        motion->received_ms=now;
        if (axis!=1) vision.align_offset[0]=vision.align_error=(int16_t)offsets[0];
        if (axis!=0) vision.align_offset[1]=(int16_t)offsets[1];
        vision_observed(vision.stack_state==VISION_PICK_WAITING ? vision.stack_color : 0,
                        vision.align_offset[0],vision.align_offset[1],now);
        return;
invalid_motion:
        vision.bad_frames++;
        if(raw.active)vision_raw_fault("invalid H MOVEXY");
        else arm_output("[vision warning] invalid MOVE/MOVEY/MOVEXY; request retained\r\n");
        return;
    }
    if (!strncmp(message,"STACK,",6)) {
        int dx,ok=1;
        if (vision.stack_state!=VISION_PICK_WAITING || !vision_tx_complete()) return;
        cursor=message+5;
        if (!vision_number(&cursor,&x) || !vision_number(&cursor,&y)) goto invalid_stack;
        dx=x-320;
        /* 兼容旧STACK,x,y作为结束报告；新版本为STACK,x,y,dx,ok。两者均不生成移车。 */
        if (*cursor && (!vision_number(&cursor,&dx) || !vision_number(&cursor,&ok))) goto invalid_stack;
        if (*cursor || ok!=1) goto invalid_stack;
        vision.stack_x=(int16_t)x;vision.stack_y=(int16_t)y;
        vision.stack_state=VISION_PICK_READY;
        vision_observed(vision.stack_color,dx,y-240,now);
        snprintf(vision.line,sizeof(vision.line),"[vision] STACK color=%u x=%d y=%d dx=%d ok=%d; drain commanded moves\r\n",
                 (unsigned)vision.stack_color,x,y,dx,ok);
        arm_output(vision.line);return;
invalid_stack:
        vision.bad_frames++;arm_output("[vision warning] invalid/unsuccessful STACK; request retained\r\n");return;
    }
    if (vision.stack_state==VISION_PICK_WAITING) {
        if (!strcmp(message,"BUSY") || !strncmp(message,"NONE",4) || !strncmp(message,"ERR",3))
            arm_output("[vision warning] STACK target unavailable/busy; request retained\r\n");
        return;
    }
    if (vision.align_state==VISION_PICK_WAITING) {
        if (!vision_tx_complete() || !strcmp(message,"READY")) return;
        if (vision.align_command=='H' && !strncmp(message,"HCOLORS,",8)) {
            int first,second,direction;
            cursor=message+7;
            if (!vision_number(&cursor,&first) || !vision_number(&cursor,&second) || !vision_direction(&cursor,&direction)
                || first<1 || first>4 || second<1 || second>4 || first==second
                || (vision.material_colors[0] && (vision.material_colors[0]!=first
                    || vision.material_colors[1]!=second || vision.material_direction!=direction))) {
                vision.bad_frames++;arm_output("[vision warning] invalid/inconsistent HCOLORS; request retained\r\n");return;
            }
            vision.material_colors[0]=(uint8_t)first;vision.material_colors[1]=(uint8_t)second;
            vision.material_direction=(uint8_t)direction;
            if(raw.active)raw.rx_pending=0;
            snprintf(vision.line,sizeof(vision.line),"[vision H] %s; waiting for HDISC/HDISC2\r\n",message);
            arm_output(vision.line);
            return;
        }
        if(vision.align_command=='H' && !strncmp(message,"HDISC2,",7)) {
#ifndef ARM_HOST_TEST
            if((int32_t)(vision.last_byte_ms-vision_i2c.tx_drained_ms)<0)return;
#endif
            if(!vision_hdisc2(message+6)) {
                vision.bad_frames++;
                if(raw.active)vision_raw_fault("invalid HDISC2");
                else arm_output("[vision warning] invalid HDISC2; H request retained\r\n");
                return;
            }
            if(raw.active && !vision.material_count) {vision_raw_fault("HDISC2 contains no coordinates");return;}
            if(raw.active)raw.rx_pending=0;
            snprintf(vision.line,sizeof(vision.line),"[vision H] %s; queued_moves=%u; wait for physical motion\r\n",
                     message,(unsigned)vision.motion_count);
            arm_output(vision.line);
            vision.align_error=0;vision.align_state=VISION_PICK_READY;return;
        }
        if(vision.align_command=='H' && !strncmp(message,"HDISC,",6)) {
            int cx,cy,dx,first,second,direction;
            if(vision.hdisc2_required) {
                arm_output("[vision warning] automatic H requires HDISC2; request retained\r\n");return;
            }
            cursor=message+5;
            if(!vision_number(&cursor,&cx) || !vision_number(&cursor,&cy) || !vision_number(&cursor,&dx)
                || !vision_number(&cursor,&first) || !vision_number(&cursor,&second)
                || !vision_direction(&cursor,&direction) || first<1 || first>4 || second<1 || second>4 || first==second
                || (vision.material_colors[0] && (vision.material_colors[0]!=first
                    || vision.material_colors[1]!=second || vision.material_direction!=direction))) {
                vision.bad_frames++;arm_output("[vision warning] invalid/inconsistent HDISC; request retained\r\n");return;
            }
            vision.material_colors[0]=(uint8_t)first;vision.material_colors[1]=(uint8_t)second;
            vision.material_direction=(uint8_t)direction;
            vision_observed((uint8_t)second,dx,cy-240,now);
            snprintf(vision.line,sizeof(vision.line),"[vision H] %s; queued_moves=%u; wait for physical motion\r\n",
                     message,(unsigned)vision.motion_count);
            arm_output(vision.line);
            vision.align_error=0;vision.align_state=VISION_PICK_READY;return;
        }
        if (!strncmp(message,"DISC,",5)) {
            int cx,cy,dx,ok;
            cursor=message+4;
            if (!vision_number(&cursor,&cx) || !vision_number(&cursor,&cy)
                || !vision_number(&cursor,&dx) || !vision_number(&cursor,&ok) || *cursor
                || ok!=1 || vision.align_command=='H') {
                vision.bad_frames++;arm_output("[vision warning] DISC not successful/valid; request retained\r\n");return;
            }
            /* 上位机决定完成；坐标/残差仅显示，终包之前的离散指令继续按序执行。 */
            vision_observed(vision.material_colors[1],dx,cy-240,now);
            vision.align_error=0;vision.align_state=VISION_PICK_READY;return;
        }
        if (!strncmp(message,"NONE",4) || !strncmp(message,"ERR",3)) {
            if(vision.hdisc2_required && (!strcmp(message,"NONE,H") || !strncmp(message,"ERR,",4))) {
#ifndef ARM_HOST_TEST
                if((int32_t)(vision.last_byte_ms-vision_i2c.tx_drained_ms)<0)return;
#endif
                snprintf(vision.line,sizeof(vision.line),"[vision H fault] %s\r\n",message);
                arm_output(vision.line);
                vision_cancel_pick();
                vision.align_state=VISION_PICK_FAULT;
            } else arm_output("[vision warning] target unavailable; request retained\r\n");
            return;
        }
    }
    if (!strncmp(message, "PICK,", 5)) {
        if(raw.active && vision.pick_state!=VISION_PICK_WAITING)return;
        cursor = message + 4;
        if (!vision_number(&cursor, &value) || value < 1 || value > 4) {
            if(raw.active)vision_raw_fault("invalid PICK color");
            return;
        }
        if(!vision_tx_complete())return; /* 主机未取走本次C时，不能用积压PICK提前完成它。 */
        if (vision.pick_state != VISION_PICK_WAITING) {
            snprintf(vision.line, sizeof(vision.line),
                     "[vision] PICK color=%d ignored; no request pending\r\n", value);
            arm_output(vision.line);
            return;
        }
        if (value != vision.request_color) {
            if(raw.active) {vision_raw_fault("PICK color differs from current C");return;}
            if (!vision.mismatch_reported) {
                snprintf(vision.line, sizeof(vision.line),
                         "[vision] PICK color=%d but request=%d; keep waiting\r\n",
                         value, vision.request_color);
                arm_output(vision.line);
                vision.mismatch_reported = 1;
            }
            return;
        }
        if (*cursor) {
            if (!vision_number(&cursor,&x) || !vision_number(&cursor,&y) || *cursor) {
                vision.bad_frames++;vision_raw_fault("invalid PICK coordinates");return;
            }
            has_coordinates=1;
        }
        if(raw.active && (!has_coordinates || x<0 || x>=RAW_J_IMAGE_WIDTH || y<0 || y>=RAW_J_IMAGE_HEIGHT)) {
            vision_raw_fault("PICK outside the 640x480 image or missing coordinates");return;
        }
        if(raw.active)raw.rx_pending=0;
        vision.pick_x = (int16_t)x;vision.pick_y = (int16_t)y;
        vision.pick_color = vision.request_color;
        if(has_coordinates && x>=0 && x<640 && y>=0 && y<480)
            vision_observed(vision.pick_color,x-320,y-240,now);
        else vision_observed(0,0,0,now);
        vision.pick_state = VISION_PICK_READY;
        snprintf(vision.line, sizeof(vision.line),
                 "[vision] PICK color=%u x=%d y=%d at fixed pick port; %lu ms\r\n",
                 (unsigned)vision.pick_color, (int)vision.pick_x, (int)vision.pick_y,
                 (unsigned long)(now - vision.request_ms));
        arm_output(vision.line);
        return;
    }
    if (!strcmp(message, "BUSY") || !strncmp(message, "BUSY,", 5)) {
        if (vision.pick_state == VISION_PICK_WAITING && !vision.busy_reported) {
            arm_output("[vision] BUSY: target not at pick port yet; keep waiting\r\n");
            vision.busy_reported = 1;
        }
        return;
    }
    if (!strncmp(message, "ERR", 3) || !strncmp(message, "TIMEOUT", 7)) {
        snprintf(vision.line, sizeof(vision.line), "[vision] error reply: %s\r\n", message);
        arm_output(vision.line);
        return;
    }
    snprintf(vision.line, sizeof(vision.line), "[vision] rx: %s\r\n", message);
    arm_output(vision.line);
}

static void vision_resync(void)
{
    uint8_t i;
    for (i = 1; i < vision.length; i++)
        if (vision.frame[i] == 0xAAu)
            break;
    if (i >= vision.length) {
        vision.length = 0;
        return;
    }
    memmove(vision.frame, &vision.frame[i], (size_t)(vision.length - i));
    vision.length = (uint8_t)(vision.length - i);
}

static void vision_expire_partial(uint32_t now)
{
    /* 更新函数入口时间可早于处理中断的新到达时间，不能把负差当成32位回绕超时。 */
    if(vision.length && (int32_t)(now-vision.last_byte_ms)>(int32_t)VISION_HALF_FRAME_MS) {
        if(vision.material_state==VISION_PICK_WAITING) {
            j_diag.half_frames++;
            snprintf(vision.line,sizeof(vision.line),"[vision J gap] local=%lu gap_ms=%lu limit_ms=%u\r\n",
                (unsigned long)j_diag.id,(unsigned long)(now-vision.last_byte_ms),(unsigned)VISION_HALF_FRAME_MS);
            arm_output(vision.line);
        }
        vision_j_drop("half_frame",now);vision.length=0;vision.bad_frames++;
        vision_raw_damaged(now);
    }
}
static void vision_byte(uint8_t byte, uint32_t now,uint32_t processed_ms)
{
    uint8_t need;
    vision_expire_partial(now);
    vision.last_byte_ms = now;
    if (vision.length == 0 && byte != 0xAAu) {
        if(vision.material_state==VISION_PICK_WAITING)j_diag.orphans++;
        return;
    }
    if (vision.length == 1 && byte != 0x55u) {
        vision.length = (uint8_t)(byte == 0xAAu ? 1u : 0u);
        if (vision.length)
            vision.frame[0] = 0xAAu;
        return;
    }
    vision.frame[vision.length++] = byte;
    if (vision.length < 3u)
        return;
    need = (uint8_t)(vision.frame[2] + VISION_FRAME_OVERHEAD);
    if (need > sizeof(vision.frame) || vision.frame[2] > VISION_PAYLOAD_MAX) {
        vision_j_drop("payload_length",now);
        vision.bad_frames++;
        vision_resync();
        vision_raw_damaged(now);
        return;
    }
    if (vision.length < need)
        return;
    if (vision.frame[need - 1u] == 0x0Du
        && vision.frame[need - 2u] == vision_crc8(&vision.frame[2], (uint16_t)(vision.frame[2] + 1u))) {
        vision_handle(&vision.frame[3], vision.frame[2], processed_ms);
        vision.length = 0;
    } else {
        vision_j_drop(vision.frame[need-1u]==0x0Du ? "crc" : "tail_or_missing_fragment",now);
        if (vision.frame[need - 1u] == 0x0Du)
            vision.crc_errors++;
        else
            vision.bad_frames++;
        vision_resync();
        vision_raw_damaged(now);
    }
}

void vision_init(void)
{
    memset(&vision, 0, sizeof(vision));
    memset(&raw,0,sizeof(raw));
    memset(&j_diag,0,sizeof(j_diag));rx_transport_errors=rx_ring_overflows=0;
    vision.material_direction=2;
    cancel_pending=0;deferred_request[0]=0;
    rx_head = rx_tail = rx_fault = tx_length = tx_offset = 0;
#ifdef ARM_HOST_TEST
    entry_host_vision_init(115200u);
#else
    vision_i2c_init();
#endif
    arm_output("[vision] soft I2C slave 0x42 PG15=SCL PD7=SDA; mailbox=32 AA55+CRC8\r\n");
}

void vision_update(uint32_t now)
{
    uint8_t fault;uint16_t event;uint32_t at_ms;
    __disable_irq();fault = rx_fault;rx_fault = 0;
    if (fault&2u) rx_tail = rx_head; /* 满队列无法保留断点；仅此情况丢弃积压流。 */
    __enable_irq();
    if (fault) {
        vision.overflows++;
        if(fault&2u){vision_j_drop("rx_ring_full",now);vision.length=0;}
        arm_output("[vision warning] transport/receive overflow; error boundaries retained, requests retained\r\n");
        if(fault&2u)vision_raw_fault("receive queue overflow");
    }
    vision_transmit();
    if(vision.material_state==VISION_PICK_WAITING && !cancel_pending && !j_diag.delivered && vision_tx_complete()) {
        j_diag.delivered=1;
#ifndef ARM_HOST_TEST
        j_diag.delivered_ms=vision_i2c.tx_drained_ms;
#else
        j_diag.delivered_ms=now;
#endif
        snprintf(vision.line,sizeof(vision.line),"[vision J delivered] local=%lu at_ms=%lu; request read completely\r\n",
            (unsigned long)j_diag.id,(unsigned long)j_diag.delivered_ms);arm_output(vision.line);
    }
    if (vision.align_state == VISION_PICK_WAITING
        && (uint32_t)(now-vision.align_warning_ms) >= VISION_ALIGN_TIMEOUT_MS) {
        vision.align_warning_ms=now;vision.timeouts++;
        arm_output("[vision warning] alignment reply delayed; keep same H/D/E request active\r\n");
    }
    if (vision.stack_state==VISION_PICK_WAITING
        && (uint32_t)(now-vision.stack_warning_ms)>=VISION_ALIGN_TIMEOUT_MS) {
        vision.stack_warning_ms=now;vision.timeouts++;
        arm_output("[vision warning] STACK reply delayed; same F request retained\r\n");
    }
    if(vision.material_state==VISION_PICK_WAITING
        && (uint32_t)(now-vision.material_warning_ms)>=VISION_ALIGN_TIMEOUT_MS) {
        vision.material_warning_ms=now;
        arm_output("[vision warning] J still observing; same request retained\r\n");
        vision_j_diagnostics(now);
    }
    while (rx_tail != rx_head) {
        event=rx_ring[rx_tail];at_ms=rx_received_ms[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1u) % VISION_RX_RING);
        if(event&VISION_RX_ERROR) {
            vision_j_drop((event&2u) ? "invalid_mailbox" : "transport_edge",at_ms);
            vision.length=0;
            vision_raw_fault("I2C receive interrupted");
        } else if(event&VISION_RX_MAILBOX) {
            if(vision.material_state==VISION_PICK_WAITING) {
                snprintf(vision.line,sizeof(vision.line),"[vision J rx] local=%lu at_ms=%lu n=%u gap_ms=%lu have=%u\r\n",
                    (unsigned long)j_diag.id,(unsigned long)at_ms,(unsigned)(event&0xffu),
                    (unsigned long)(j_diag.chunks ? at_ms-j_diag.last_chunk_ms : 0u),(unsigned)vision.length);
                arm_output(vision.line);j_diag.chunks++;j_diag.bytes+=event&0xffu;j_diag.last_chunk_ms=at_ms;
            }
        } else vision_byte((uint8_t)event,at_ms,now);
    }
    /* 先按中断到达时间消费积压流，再对真正没有续片的半帧计时。 */
    vision_expire_partial(now);
    if(raw.active && raw.rx_pending && (int32_t)(now-raw.rx_error_ms)>=(int32_t)VISION_RAW_RX_RECOVERY_MS)
        vision_raw_fault("receive error did not recover");
    if(raw.active && ((raw.task_state==VISION_PICK_WAITING && (uint32_t)(now-raw.task_ms)>=VISION_ALIGN_TIMEOUT_MS)
        || (vision.align_state==VISION_PICK_WAITING && (uint32_t)(now-vision.align_ms)>=VISION_ALIGN_TIMEOUT_MS)
        || (vision.pick_state==VISION_PICK_WAITING && (uint32_t)(now-vision.request_ms)>=VISION_ALIGN_TIMEOUT_MS)))
        vision_raw_fault("camera command deadline expired");
    if (vision.pick_state == VISION_PICK_WAITING
        && (uint32_t)(now - vision.pick_warning_ms) >= VISION_PICK_TIMEOUT_MS) {
        vision.pick_warning_ms = now;
        vision.timeouts++;
        snprintf(vision.line, sizeof(vision.line),
                 "[vision warning] PICK delayed %lu ms; color=%u; request retained\r\n",
                 (unsigned long)VISION_PICK_TIMEOUT_MS, (unsigned)vision.request_color);
        arm_output(vision.line);
    }
}

void vision_request_raw_task(const char *code,uint32_t now)
{
    char payload[18];
    vision_cancel_pick();
    memset(&raw,0,sizeof(raw));
    raw.active=1;
    if(strlen(code)!=15u) {vision_raw_fault("invalid canonical task code");return;}
    snprintf(payload,sizeof(payload),"T,%s",code);
    snprintf(raw.expected_task,sizeof(raw.expected_task),"TASK,%.3s,%.3s",code,code+8);
    raw.task_state=VISION_PICK_WAITING;raw.task_ms=now;
    if(!vision_send_request(payload))vision_raw_fault("cannot queue T");
}

int vision_raw_task_state(void) {return raw.task_state;}
int vision_raw_failed(void) {return raw.failed;}
int vision_raw_rx_pending(void) {return raw.rx_pending;}

void vision_finish_raw(void)
{
    raw.active=0;raw.task_state=VISION_PICK_IDLE;
    vision.pick_state=VISION_PICK_IDLE;
}

void vision_request_pick(uint8_t color, uint32_t now)
{
    char payload[8];
    if(raw.active) {
        if(raw.task_state!=VISION_PICK_READY || vision.align_state!=VISION_PICK_READY || vision.motion_count
            || vision.pick_state==VISION_PICK_WAITING || raw.rx_pending || color<1u || color>4u) {
            vision_raw_fault("C requested outside the completed H/PICK sequence");return;
        }
    } else {
        if(cancel_pending || vision.material_state==VISION_PICK_WAITING
            || (vision.align_state==VISION_PICK_WAITING && vision.align_command=='H'))vision_cancel_pick();
        vision.material_state=VISION_PICK_IDLE;vision.material_count=0;vision.material_direction=2;
        vision.align_state=VISION_PICK_IDLE;vision.align_stage=ALIGN_IDLE;vision.motion_count=0;vision.h_arrival=0;
        vision.align_command=0;vision.material_colors[0]=vision.material_colors[1]=0;
        vision.stack_state=VISION_PICK_IDLE;vision.stack_color=0;
    }
    if (color < 1u || color > 4u)
        return;
    snprintf(payload, sizeof(payload), "C,%u", (unsigned)color);
    vision.pick_state = VISION_PICK_WAITING;
    vision.request_color = color;
    vision.pick_color = 0;
    vision.request_ms = vision.pick_warning_ms = now;
    vision.busy_reported = 0;
    vision.mismatch_reported = 0;
    if(!cancel_pending && !raw.active) {
        vision.length=0;
        __disable_irq();rx_tail=rx_head;rx_fault=0;__enable_irq();
    }
    if (!vision_send_request(payload)) {vision.pick_state = VISION_PICK_FAULT;vision_raw_fault("cannot queue C");return;}
    snprintf(vision.line, sizeof(vision.line),
             "[vision] queued request %s; wait for PICK (20s)\r\n", payload);
    arm_output(vision.line);
}

int vision_pick_state(void) { return vision.pick_state; }

void vision_pick_result(uint8_t *color, int16_t *x, int16_t *y)
{
    if (color)
        *color = vision.pick_color;
    if (x)
        *x = vision.pick_x;
    if (y)
        *y = vision.pick_y;
}

void vision_cancel_pick(void)
{
    uint8_t abort_task=(uint8_t)(raw.active || vision.material_state==VISION_PICK_WAITING
        || (vision.align_state==VISION_PICK_WAITING && vision.align_command=='H'));
    raw.active=0;raw.task_state=VISION_PICK_IDLE;raw.rx_pending=0;
    if(!cancel_pending) {
#ifndef ARM_HOST_TEST
        vision_i2c_cancel_tx();
#endif
        tx_length=tx_offset=0;vision.length=0;
        __disable_irq();rx_tail=rx_head;rx_fault=0;__enable_irq();
    }
    deferred_request[0]=0;
    vision.pick_state = VISION_PICK_IDLE;
    vision.request_color=vision.pick_color=0;vision.pick_x=vision.pick_y=0;
    vision.stack_state=VISION_PICK_IDLE;vision.stack_color=0;
    vision.align_state = VISION_PICK_IDLE;
    vision.align_command=0;vision.hdisc2_required=0;vision.material_colors[0]=vision.material_colors[1]=0;
    vision.align_stage = ALIGN_IDLE;vision.motion_count=0;vision.h_arrival=0;
    vision.material_state=VISION_PICK_IDLE;vision.material_count=0;vision.material_direction=2;
    vision.align_offset[0]=vision.align_offset[1]=vision.align_error=0;
    if(abort_task && !cancel_pending) {
        cancel_pending=1;
        (void)vision_send("R");
    }
}

void vision_request_stack(uint8_t color,uint32_t now)
{
    char payload[8];
    vision_cancel_pick();
    if(color<1u || color>4u){vision.stack_state=VISION_PICK_FAULT;return;}
    vision.stack_state=VISION_PICK_WAITING;vision.stack_color=color;vision.stack_warning_ms=now;
    snprintf(payload,sizeof(payload),"F,%u",(unsigned)color);
    if(!vision_send_request(payload)){vision.stack_state=VISION_PICK_FAULT;return;}
    snprintf(vision.line,sizeof(vision.line),"[vision] queued request %s; wait for MOVE/MOVEY/MOVEXY and STACK,x,y,dx,ok\r\n",payload);
    arm_output(vision.line);
}
int vision_stack_state(void){return vision.stack_state;}
void vision_stack_result(int16_t *x,int16_t *y)
{
    if(x)*x=vision.stack_x;
    if(y)*y=vision.stack_y;
}

static void vision_start_align(char command,uint8_t ring,uint8_t hdisc2_required,uint32_t now)
{
    char payload[2];
    if(raw.active && hdisc2_required) {
        if(raw.task_state!=VISION_PICK_READY || vision.align_state!=VISION_PICK_IDLE || raw.rx_pending) {
            vision_raw_fault("H requested before TASK confirmation or more than once");return;
        }
    } else vision_cancel_pick();
    if ((command!='D' && command!='E' && command!='H') || ring < 1 || ring > 3) {
        vision.align_state=VISION_PICK_FAULT;return;
    }
    vision.align_command=command;
    vision.hdisc2_required=hdisc2_required;
    vision.align_stage=ALIGN_MEASURE;vision.align_ms=vision.align_warning_ms=now;
    vision.align_state=VISION_PICK_WAITING;
    payload[0]=command;payload[1]=0;
    if (!vision_send_request(payload)) {vision.align_state=VISION_PICK_FAULT;vision_raw_fault("cannot queue H");}
}
void vision_request_align(char command,uint8_t ring,uint32_t now)
{
    vision_start_align(command,ring,0u,now);
}
void vision_request_raw_align(uint32_t now)
{
    vision_start_align('H',2u,1u,now);
}
int vision_align_state(void) {return vision.align_state;}
int vision_material_colors(uint8_t *first,uint8_t *second)
{
    if(vision.align_command!='H' || vision.align_state!=VISION_PICK_READY || !vision.material_colors[0])return 0;
    if(first)*first=vision.material_colors[0];
    if(second)*second=vision.material_colors[1];
    return 1;
}
int vision_material_direction(void)
{
    return vision.align_command=='H' && vision.align_state==VISION_PICK_READY && vision.material_direction<2u
        ? (int)vision.material_direction : -1;
}
int vision_material_h_result(uint8_t *count,VisionMaterial samples[4])
{
    if(vision.align_command!='H' || vision.align_state!=VISION_PICK_READY || !vision.h_arrival)return -1;
    if(count)*count=vision.material_count;
    if(samples)memcpy(samples,vision.materials,vision.material_count*sizeof(samples[0]));
    return (int)vision.h_arrival-1;
}
void vision_request_material_state(uint32_t now)
{
    vision_cancel_pick();vision.material_state=VISION_PICK_WAITING;vision.material_warning_ms=now;
    {uint32_t id=j_diag.id+1u;memset(&j_diag,0,sizeof(j_diag));j_diag.id=id;}
    j_diag.start_ms=now;j_diag.frames=vision.frames;j_diag.crc=vision.crc_errors;j_diag.bad=vision.bad_frames;
    j_diag.transport=rx_transport_errors;j_diag.ring_full=rx_ring_overflows;
#ifndef ARM_HOST_TEST
    j_diag.reads=vision_i2c.reads;j_diag.writes=vision_i2c.writes;
    j_diag.invalid=vision_i2c.invalid;j_diag.bus_errors=vision_i2c.bus_errors;
#endif
    if(!vision_send_request("J"))vision.material_state=VISION_PICK_FAULT;
    snprintf(vision.line,sizeof(vision.line),"[vision J start] local=%lu at_ms=%lu; J starts once; local id is diagnostic only\r\n",
        (unsigned long)j_diag.id,(unsigned long)now);arm_output(vision.line);
}
int vision_material_state(void){return vision.material_state;}
uint8_t vision_material_state_result(VisionMaterial samples[4])
{
    if(vision.material_state==VISION_PICK_IDLE)return 0;
    if(samples)memcpy(samples,vision.materials,vision.material_count*sizeof(samples[0]));
    return vision.material_count;
}
int16_t vision_align_offset_px(void) {return vision.align_error;}
int vision_motion_peek(uint8_t *axis,int16_t *offset,uint32_t *received_ms)
{
    if (!vision.motion_count) return 0;
    if (axis) *axis=vision.motion[0].axis;
    if (offset) *offset=vision.motion[0].pixels[vision.motion[0].axis==1 ? 1 : 0];
    if (received_ms) *received_ms=vision.motion[0].received_ms;
    return 1;
}
/* 单轴和双轴统一以完整队首读取，配对命令只能整体消费。 */
int vision_motion_peek_xy(int16_t *dx,int16_t *dy,uint32_t *received_ms)
{
    if (!vision.motion_count) return 0;
    if (dx) *dx=vision.motion[0].pixels[0];
    if (dy) *dy=vision.motion[0].pixels[1];
    if (received_ms) *received_ms=vision.motion[0].received_ms;
    return 1;
}
static void vision_motion_remove(unsigned index)
{
    vision.motion_count--;
    memmove(&vision.motion[index],&vision.motion[index+1u],
            (vision.motion_count-index)*sizeof(vision.motion[0]));
}
void vision_motion_take(void)
{
    if (vision.motion_count) vision_motion_remove(0);
}
/* 兼容原有轴查询接口；工位执行只消费总队列队首，不重排轴指令。 */
int vision_align_peek_axis(uint8_t axis,int16_t *offset,uint32_t *received_ms)
{
    unsigned i;
    if(axis>1)return 0;
    for (i=0;i<vision.motion_count;i++) if (vision.motion[i].axis==axis) {
        if (offset) *offset=vision.motion[i].pixels[axis];
        if (received_ms) *received_ms=vision.motion[i].received_ms;
        return 1;
    }
    return 0;
}
int vision_align_take_axis(uint8_t axis,int16_t *offset,uint32_t *received_ms)
{
    unsigned i;
    if (!vision_align_peek_axis(axis,offset,received_ms)) return 0;
    for (i=0;i<vision.motion_count;i++) if (vision.motion[i].axis==axis) {
        vision_motion_remove(i);return 1;
    }
    return 0;
}
int vision_align_take_move(int16_t *offset, uint32_t *received_ms)
{
    return vision_align_take_axis(0,offset,received_ms);
}

const char *vision_last_message(void) { return vision.message; }

int vision_command(const char *s, uint32_t now)
{
    int color = 0;
    if (strcmp(s, "vision") && strncmp(s, "vision ", 7))
        return 0;
    if (!strcmp(s, "vision") || !strcmp(s, "vision status")) {
        const char *states[5] = { "idle", "waiting", "ready", "timeout", "fault" };
#ifndef ARM_HOST_TEST
        snprintf(vision.line,sizeof(vision.line),"[vision i2c] addr=0x42 mailbox=32 reads=%lu writes=%lu queued=%u invalid=%lu bus_err=%lu\r\n",
            (unsigned long)vision_i2c.reads,(unsigned long)vision_i2c.writes,
            (unsigned)((vision_i2c.head-vision_i2c.tail+VISION_I2C_TX_RING)%VISION_I2C_TX_RING),
            (unsigned long)vision_i2c.invalid,(unsigned long)vision_i2c.bus_errors);
        arm_output(vision.line);
#endif
        snprintf(vision.line, sizeof(vision.line),
                 "[vision] state=%s request=%u pick=%u frames=%lu crc_err=%lu bad=%lu overflow=%lu timeout=%lu\r\n",
                 states[vision.pick_state], (unsigned)vision.request_color,
                 (unsigned)vision.pick_color, (unsigned long)vision.frames,
                 (unsigned long)vision.crc_errors, (unsigned long)vision.bad_frames,
                 (unsigned long)vision.overflows, (unsigned long)vision.timeouts);
        arm_output(vision.line);
        snprintf(vision.line,sizeof(vision.line),"[vision rough] state=%u phase=%u offset_px=%d offset_y_px=%d queued_moves=%u\r\n",
                 (unsigned)vision.align_state,(unsigned)vision.align_stage,(int)vision.align_error,
                 (int)vision.align_offset[1],(unsigned)vision.motion_count);
        arm_output(vision.line);
        snprintf(vision.line,sizeof(vision.line),"[vision stack] state=%u color=%u x=%d y=%d\r\n",
                 (unsigned)vision.stack_state,(unsigned)vision.stack_color,(int)vision.stack_x,(int)vision.stack_y);
        arm_output(vision.line);
        snprintf(vision.line,sizeof(vision.line),"[vision H] first=%u second=%u terminal=%u direction=%d\r\n",
                 (unsigned)vision.material_colors[0],(unsigned)vision.material_colors[1],
                 (unsigned)(vision.align_command=='H' && vision.align_state==VISION_PICK_READY),vision_material_direction());
        arm_output(vision.line);
        snprintf(vision.line,sizeof(vision.line),"[vision J] state=%u count=%u\r\n",(unsigned)vision.material_state,(unsigned)vision.material_count);
        arm_output(vision.line);
        if(j_diag.id)vision_j_diagnostics(now);
        snprintf(vision.line,sizeof(vision.line),"[vision cancel] waiting_READY=%u next=\"%s\"\r\n",
                 (unsigned)cancel_pending,deferred_request);
        arm_output(vision.line);
        snprintf(vision.line, sizeof(vision.line), "[vision] last=\"%s\"\r\n", vision.message);
        arm_output(vision.line);
        return 1;
    }
    if (!strcmp(s, "vision ping")) {
        if(cancel_pending) {arm_output("[vision rejected: waiting for R/READY cancellation]\r\n");return 1;}
        if(vision.align_state==VISION_PICK_WAITING && vision.align_command=='H') {
            arm_output("[vision rejected: H active; vision range or vision clear cancels]\r\n");return 1;
        }
        if(vision.material_state==VISION_PICK_WAITING) {
            arm_output("[vision rejected: J active; vision range or vision clear cancels]\r\n");return 1;
        }
        if (!vision_send("D")) {arm_output("[vision rejected: transmit busy]\r\n");return 1;}
        arm_output("[vision] queued D; expect MOVE/MOVEY/MOVEXY/DISC frame\r\n");
        return 1;
    }
    if(vision.material_state==VISION_PICK_WAITING
        && (!strcmp(s,"vision h") || !strcmp(s,"vision j") || !strncmp(s,"vision pick ",12))) {
        arm_output("[vision rejected: J active; vision range or vision clear cancels]\r\n");return 1;
    }
    if (!strcmp(s,"vision h")) {
        vision_request_align('H',2,now);
        arm_output("[vision] queued H; observe MOVE/HCOLORS/HDISC/HDISC2 only; no automatic chassis or arm motion\r\n");
        return 1;
    }
    if(!strcmp(s,"vision j")) {
        vision_request_material_state(now);
        arm_output("[vision] queued J; observe MSTATE only; no chassis or arm motion\r\n");return 1;
    }
    if (!strcmp(s, "vision range")) {
        vision_cancel_pick();
        if(!cancel_pending) {cancel_pending=1;(void)vision_send("R");}
        arm_output("[vision] R cancellation pending; wait for READY\r\n");
        return 1;
    }
    if (!strcmp(s, "vision clear")) {
        vision_cancel_pick();
        arm_output("[vision] request cleared; counters kept\r\n");
        return 1;
    }
    if (!strncmp(s, "vision pick ", 12)) {
        color = (int)(s[12] - '0');
        if (color >= 1 && color <= 4 && !s[13]) {
            vision_request_pick((uint8_t)color, now);
        } else {
            arm_output("[vision rejected: pick COLOR 1..4]\r\n");
        }
        return 1;
    }
    arm_output("[vision rejected: use vision|vision status|vision ping|vision h|vision j|vision range|vision pick N|vision clear]\r\n");
    return 1;
}
