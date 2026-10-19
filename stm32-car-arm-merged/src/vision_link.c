#include "vision_link.h"
#include "arm_control.h"
#ifdef ARM_HOST_TEST
#include "../tests/vision_host_io.h"
#else
#include "usart5.h"
#include "stm32f4xx.h"
#endif
#include <string.h>
#include <stdio.h>

#define VISION_RX_RING          256u
#define VISION_PAYLOAD_MAX       48u
#define VISION_FRAME_OVERHEAD     5u /* AA 55 LEN ... CRC 0D */
#define VISION_HALF_FRAME_MS    200u
#define VISION_PICK_TIMEOUT_MS 20000u
#define VISION_ALIGN_TIMEOUT_MS 25000u
#define VISION_ALIGN_RESET_MS 2000u
#define VISION_ALIGN_TOL_PX 5 /* 与MaixCAM2 ALIGN_TOL一致，对外640x480口径。 */
#define VISION_ALIGN_MAX_MOVE_PX 40
enum { ALIGN_IDLE, ALIGN_RESET, ALIGN_MEASURE, ALIGN_CANCEL };

static volatile uint8_t rx_ring[VISION_RX_RING];
static volatile uint16_t rx_head, rx_tail;
static volatile uint8_t rx_fault;
static uint8_t tx_frame[VISION_PAYLOAD_MAX + VISION_FRAME_OVERHEAD];
static uint8_t tx_length, tx_offset;

static struct {
    uint8_t frame[VISION_PAYLOAD_MAX + VISION_FRAME_OVERHEAD];
    uint8_t length;
    uint32_t last_byte_ms;
    uint8_t pick_state;
    uint8_t request_color, pick_color;
    uint8_t align_state, align_stage; /* 1握手，2测量，3取消MOVE任务等待READY。 */
    int16_t align_error;
    uint32_t align_ms;
    int16_t pick_x, pick_y;
    uint32_t request_ms;
    uint8_t busy_reported, mismatch_reported;
    char message[VISION_PAYLOAD_MAX + 1u];
    uint32_t frames, crc_errors, bad_frames, overflows, timeouts;
    char line[256]; /* 中文UTF-8状态和最长协议原文。 */
} vision;

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

void vision_receive_byte(uint8_t byte, uint8_t uart_error)
{
    uint16_t next;
    if (uart_error) {rx_fault |= 1u;return;}
    next = (uint16_t)((rx_head + 1u) % VISION_RX_RING);
    if (next == rx_tail) {rx_fault |= 2u;return;}
    rx_ring[rx_head] = byte;rx_head = next;
}

/* 发送按TXE分次推进，视觉断线/发送器异常不能堵住急停和底盘主循环。 */
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
    if (USART_GetFlagStatus(UART5,USART_FLAG_TXE) == RESET) return;
    USART_SendData(UART5,tx_frame[tx_offset++]);
#endif
}

static int vision_number(const char **cursor, int *value)
{
    const char *s = *cursor;
    int sign = 1, found = 0;
    long result = 0;
    if (*s != ',') return 0;
    s++;
    while (*s == ' ') s++;
    if (*s == '-') {sign = -1;s++;}
    while (*s >= '0' && *s <= '9') {
        if (result > 3276 || (result == 3276 && *s - '0' > (sign < 0 ? 8 : 7))) return 0;
        result = result * 10 + (*s++ - '0');found = 1;
    }
    if (!found || (*s && *s != ',')) return 0;
    *value = (int)(sign * result);*cursor = s;
    return 1;
}

static void vision_handle(const uint8_t *payload, uint8_t length, uint32_t now)
{
    char message[VISION_PAYLOAD_MAX + 1u];
    const char *cursor;
    int value, x = 0, y = 0;
    if (memchr(payload,0,length)) {vision.bad_frames++;return;}
    memcpy(message, payload, length);
    message[length] = 0;
    memcpy(vision.message, message, (size_t)length + 1u);
    vision.frames++;
    if (vision.align_state==VISION_PICK_WAITING) {
        if ((uint32_t)(now-vision.align_ms) >= (vision.align_stage==ALIGN_MEASURE ? VISION_ALIGN_TIMEOUT_MS : VISION_ALIGN_RESET_MS)) {
            vision.align_state=VISION_PICK_TIMEOUT;return;
        }
        if (!strcmp(message,"READY")) {
            if (vision.align_stage==ALIGN_RESET) {
                vision.align_stage=ALIGN_MEASURE;vision.align_ms=now;
                if (!vision_send("D")) vision.align_state=VISION_PICK_FAULT;
            } else if (vision.align_stage==ALIGN_CANCEL) vision.align_state=VISION_PICK_READY;
            return;
        }
        if (vision.align_stage!=ALIGN_MEASURE) return; /* R围栏之前的MOVE/DISC全部丢弃。 */
        if (!strncmp(message,"MOVE,",5)) {
            int offset;
            cursor=message+4;
            if (!vision_number(&cursor,&offset) || *cursor || offset < -VISION_ALIGN_MAX_MOVE_PX || offset > VISION_ALIGN_MAX_MOVE_PX
                || (offset >= -VISION_ALIGN_TOL_PX && offset <= VISION_ALIGN_TOL_PX)) {vision.align_state=VISION_PICK_FAULT;return;}
            vision.align_error=(int16_t)offset;
            vision.align_stage=ALIGN_CANCEL;vision.align_ms=now;
            if (!vision_send("R")) vision.align_state=VISION_PICK_FAULT;
            return;
        }
        if (!strncmp(message,"DISC,",5)) {
            int cx,cy,dx,ok;
            cursor=message+4;
            if (!vision_number(&cursor,&cx) || !vision_number(&cursor,&cy)
                || !vision_number(&cursor,&dx) || !vision_number(&cursor,&ok) || *cursor
                || cx<0 || cx>=640 || cy<0 || cy>=480 || dx!=cx-320
                || dx < -VISION_ALIGN_TOL_PX || dx > VISION_ALIGN_TOL_PX || ok!=1) {
                vision.align_state=VISION_PICK_FAULT;return;
            }
            vision.align_error=0;vision.align_state=VISION_PICK_READY;return;
        }
        if (!strncmp(message,"NONE",4) || !strncmp(message,"ERR",3)) {
            vision.align_state=VISION_PICK_FAULT;return;
        }
    }
    if (!strncmp(message, "PICK,", 5)) {
        cursor = message + 4;
        if (!vision_number(&cursor, &value) || value < 1 || value > 4)
            return;
        if (vision.pick_state != VISION_PICK_WAITING) {
            snprintf(vision.line, sizeof(vision.line),
                     "[vision] PICK color=%d ignored; no request pending\r\n", value);
            arm_output(vision.line);
            return;
        }
        if (value != vision.request_color) {
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
                vision.bad_frames++;return;
            }
        }
        if ((uint32_t)(now - vision.request_ms) >= VISION_PICK_TIMEOUT_MS) return;
        vision.pick_x = (int16_t)x;vision.pick_y = (int16_t)y;
        vision.pick_color = vision.request_color;
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

static void vision_byte(uint8_t byte, uint32_t now)
{
    uint8_t need;
    vision.last_byte_ms = now;
    if (vision.length == 0 && byte != 0xAAu)
        return;
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
        vision.bad_frames++;
        vision_resync();
        return;
    }
    if (vision.length < need)
        return;
    if (vision.frame[need - 1u] == 0x0Du
        && vision.frame[need - 2u] == vision_crc8(&vision.frame[2], (uint16_t)(vision.frame[2] + 1u))) {
        vision_handle(&vision.frame[3], vision.frame[2], now);
        vision.length = 0;
    } else {
        if (vision.frame[need - 1u] == 0x0Du)
            vision.crc_errors++;
        else
            vision.bad_frames++;
        vision_resync();
    }
}

void vision_init(void)
{
    memset(&vision, 0, sizeof(vision));
    rx_head = rx_tail = rx_fault = tx_length = tx_offset = 0;
#ifdef ARM_HOST_TEST
    entry_host_vision_init(115200u);
#else
    uart5_init(115200u);
#endif
    arm_output("[vision] UART5 115200 PD2=RX PC12=TX; AA55+CRC8 frames\r\n");
}

void vision_update(uint32_t now)
{
    uint8_t byte, fault;
    __disable_irq();fault = rx_fault;rx_fault = 0;
    if (fault) rx_tail = rx_head;
    __enable_irq();
    if (fault) {
        vision.length = 0;vision.overflows++;
        if (vision.pick_state == VISION_PICK_WAITING) vision.pick_state = VISION_PICK_FAULT;
        if (vision.align_state == VISION_PICK_WAITING) vision.align_state = VISION_PICK_FAULT;
    }
    vision_transmit();
    if (vision.align_state == VISION_PICK_WAITING
        && (uint32_t)(now-vision.align_ms) >= (vision.align_stage==ALIGN_MEASURE ? VISION_ALIGN_TIMEOUT_MS : VISION_ALIGN_RESET_MS))
        vision.align_state=VISION_PICK_TIMEOUT;
    if (vision.length && (uint32_t)(now - vision.last_byte_ms) > VISION_HALF_FRAME_MS) {
        vision.length = 0;
        vision.bad_frames++;
    }
    while (rx_tail != rx_head) {
        byte = rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1u) % VISION_RX_RING);
        vision_byte(byte, now);
    }
    if (vision.pick_state == VISION_PICK_WAITING
        && (uint32_t)(now - vision.request_ms) >= VISION_PICK_TIMEOUT_MS) {
        vision.pick_state = VISION_PICK_TIMEOUT;
        vision.timeouts++;
        snprintf(vision.line, sizeof(vision.line),
                 "[vision] PICK timeout after %lu ms; color=%u\r\n",
                 (unsigned long)VISION_PICK_TIMEOUT_MS, (unsigned)vision.request_color);
        arm_output(vision.line);
    }
}

void vision_request_pick(uint8_t color, uint32_t now)
{
    char payload[8];
    vision.align_state=VISION_PICK_IDLE;vision.align_stage=ALIGN_IDLE;
    if (color < 1u || color > 4u)
        return;
    snprintf(payload, sizeof(payload), "C,%u", (unsigned)color);
    vision.pick_state = VISION_PICK_WAITING;
    vision.request_color = color;
    vision.pick_color = 0;
    vision.request_ms = now;
    vision.busy_reported = 0;
    vision.mismatch_reported = 0;
    vision.length = 0;
    __disable_irq();rx_tail = rx_head;rx_fault = 0;__enable_irq();
    if (!vision_send(payload)) {vision.pick_state = VISION_PICK_FAULT;return;}
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
    vision.pick_state = VISION_PICK_IDLE;
    vision.request_color = 0;
    vision.align_state = VISION_PICK_IDLE;
    vision.align_stage = ALIGN_IDLE;
    tx_length = tx_offset = 0;vision.length = 0;
    __disable_irq();rx_tail = rx_head;rx_fault = 0;__enable_irq();
}

void vision_request_align(uint8_t ring, uint32_t now)
{
    vision_cancel_pick();
    if (ring < 1 || ring > 3) {
        vision.align_state=VISION_PICK_FAULT;return;
    }
    vision.align_stage=ALIGN_RESET;vision.align_ms=now;
    vision.align_state=VISION_PICK_WAITING;
    if (!vision_send("R")) vision.align_state=VISION_PICK_FAULT;
}
int vision_align_state(void) {return vision.align_state;}
int16_t vision_align_offset_px(void) {return vision.align_error;}

const char *vision_last_message(void) { return vision.message; }

int vision_command(const char *s, uint32_t now)
{
    int color = 0;
    if (strcmp(s, "vision") && strncmp(s, "vision ", 7))
        return 0;
    if (!strcmp(s, "vision") || !strcmp(s, "vision status")) {
        const char *states[5] = { "idle", "waiting", "ready", "timeout", "fault" };
        snprintf(vision.line, sizeof(vision.line),
                 "[vision] state=%s request=%u pick=%u frames=%lu crc_err=%lu bad=%lu overflow=%lu timeout=%lu\r\n",
                 states[vision.pick_state], (unsigned)vision.request_color,
                 (unsigned)vision.pick_color, (unsigned long)vision.frames,
                 (unsigned long)vision.crc_errors, (unsigned long)vision.bad_frames,
                 (unsigned long)vision.overflows, (unsigned long)vision.timeouts);
        arm_output(vision.line);
        snprintf(vision.line,sizeof(vision.line),"[vision rough] state=%u phase=%u offset_px=%d\r\n",
                 (unsigned)vision.align_state,(unsigned)vision.align_stage,(int)vision.align_error);
        arm_output(vision.line);
        snprintf(vision.line, sizeof(vision.line), "[vision] last=\"%s\"\r\n", vision.message);
        arm_output(vision.line);
        return 1;
    }
    if (!strcmp(s, "vision ping")) {
        if (!vision_send("D")) {arm_output("[vision rejected: transmit busy]\r\n");return 1;}
        arm_output("[vision] queued D; expect R frame\r\n");
        return 1;
    }
    if (!strcmp(s, "vision range")) {
        if (!vision_send("R")) {arm_output("[vision rejected: transmit busy]\r\n");return 1;}
        arm_output("[vision] queued R\r\n");
        return 1;
    }
    if (!strcmp(s, "vision clear")) {
        vision_cancel_pick();
        vision.length = 0;
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
    arm_output("[vision rejected: use vision|vision status|vision ping|vision range|vision pick N|vision clear]\r\n");
    return 1;
}
