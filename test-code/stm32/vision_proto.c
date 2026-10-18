/**
  ******************************************************************************
  * @file    vision_proto.c
  * @brief   视觉端(MaixCAM2) ⇄ STM32 串口协议驱动实现
  *
  * 依赖：只有 stm32f4xx.h + string.h；不依赖具体工程，也不依赖 stdio。
  * 主机自检见 vision_host_test.c（gcc -DVISION_HOST_TEST）。
  ******************************************************************************
  */

#include "vision_proto.h"
#include <string.h>

/* ==========================================================================
 *  0. 串口选择：集成板 3.1 把 MaixCAM2 的串口接在 UART5
 *       PC12 = UART5_TX  → MaixCAM2 的 RX
 *       PD2  = UART5_RX  ← MaixCAM2 的 TX
 *     想换成别的口（例如 UART4 的 PA0/PA1）时，只改下面这组宏。
 *
 *  ★ 中断函数的处理方式（两种二选一，见 README_视觉联调.md）：
 *     ① 把 vendor/stm32/lanya/usart5.c 从 Keil 工程里移除（它是蓝牙死代码），
 *        再在工程选项里定义 VISION_PROVIDE_UART5_IRQ，本文件就会提供中断；
 *     ② 保留 usart5.c，只在它的 UART5_IRQHandler 里加一句 Vision_IsrByte(...)。
 *     默认按 ② 处理（不定义中断函数），避免和 usart5.c 重复定义链接报错。
 * ========================================================================== */
#ifndef VISION_HOST_TEST
#define VISION_UART                    UART5
#define VISION_UART_IRQn               UART5_IRQn
#define VISION_UART_IRQHandler         UART5_IRQHandler
#endif

/* ==========================================================================
 *  1. CRC8 / 组帧 / 十六进制打印（纯函数，可单测）
 * ========================================================================== */

/* 增量式 CRC8（poly 0x07 / init 0x00 / 不反射 / 无最终异或）
 * Vision_Crc8(data,len) == 依次 Crc8Update(0, data[i]) */
static uint8_t Crc8Update(uint8_t crc, uint8_t byte)
{
    uint8_t bit;
    crc ^= byte;
    for (bit = 0u; bit < 8u; bit++)
    {
        if (crc & 0x80u)
            crc = (uint8_t)((uint8_t)(crc << 1) ^ 0x07u);
        else
            crc = (uint8_t)(crc << 1);
    }
    return crc;
}

uint8_t Vision_Crc8(const uint8_t *data, uint16_t len)
{
    uint8_t  crc = 0x00u;
    uint16_t i;

    if (!data)
        return 0u;
    for (i = 0; i < len; i++)
        crc = Crc8Update(crc, data[i]);
    return crc;
}

uint16_t Vision_BuildFrame(const char *payload, uint8_t *out, uint16_t cap)
{
    uint16_t len = 0;
    uint16_t i;
    uint8_t  crc;

    if (!payload || !out)
        return 0;
    while (payload[len])
        len++;
    if (len > VISION_PAYLOAD_MAX || cap < (uint16_t)(len + 5u))
        return 0;

    out[0] = VISION_HEAD0;
    out[1] = VISION_HEAD1;
    out[2] = (uint8_t)len;                       /* LEN */
    for (i = 0; i < len; i++)
        out[3u + i] = (uint8_t)payload[i];       /* payload */
    /* CRC 覆盖 LEN + payload：在 out 里这两段正好连续（out[2..2+len]）*/
    crc = Vision_Crc8(&out[2], (uint16_t)(len + 1u));
    out[3u + len] = crc;
    out[4u + len] = VISION_TAIL;

    return (uint16_t)(len + 5u);
}

void Vision_HexFrame(const char *payload, char *out, uint16_t cap)
{
    static const char hex[] = "0123456789ABCDEF";
    uint8_t  frame[VISION_FRAME_MAX];
    uint16_t n, i, pos = 0;

    if (!out || cap < 4u)
        return;
    n = Vision_BuildFrame(payload, frame, sizeof(frame));
    if (n == 0u)
    {
        out[0] = 0;
        return;
    }
    for (i = 0; i < n; i++)
    {
        uint16_t need = (uint16_t)(2u + ((i + 1u < n) ? 1u : 0u));   /* 2 位 + 可能的空格 */
        if ((uint16_t)(pos + need + 1u) > cap)
            break;
        out[pos++] = hex[(frame[i] >> 4) & 0x0Fu];
        out[pos++] = hex[frame[i] & 0x0Fu];
        if (i + 1u < n)
            out[pos++] = ' ';
    }
    out[pos] = 0;
}

/* ==========================================================================
 *  2. 拆帧状态机
 * ========================================================================== */

void Vision_ParserReset(VisionParser *p, uint32_t now_ms)
{
    if (!p)
        return;
    memset(p, 0, sizeof(*p));                        /* 连统计一起清零，避免读到未初始化值 */
    p->last_ms = now_ms;
}

uint8_t Vision_ParserFeed(VisionParser *p, uint8_t byte, uint32_t now_ms)
{
    p->last_ms = now_ms;

    switch (p->state)
    {
    case 0u:                                        /* 找 AA */
        if (byte == VISION_HEAD0)
            p->state = 1u;
        else
            p->noise++;
        return 0u;

    case 1u:                                        /* 找 55 */
        if (byte == VISION_HEAD1)
            p->state = 2u;
        else
        {
            p->noise++;
            p->state = (byte == VISION_HEAD0) ? 1u : 0u;   /* AA AA 55 也能对齐 */
        }
        return 0u;

    case 2u:                                        /* LEN */
        if (byte > VISION_PAYLOAD_MAX)
        {
            p->bad_len++;
            p->state = (byte == VISION_HEAD0) ? 1u : 0u;
            return 0u;
        }
        p->len   = byte;
        p->index = 0u;
        p->crc   = Crc8Update(0u, byte);            /* CRC 种子 = LEN */
        p->payload[0] = 0;
        p->state = (byte == 0u) ? 4u : 3u;          /* LEN=0：直接等 CRC */
        return 0u;

    case 3u:                                        /* payload */
        p->payload[p->index++] = byte;
        p->crc = Crc8Update(p->crc, byte);
        if (p->index >= p->len)
        {
            p->payload[p->index] = 0;               /* 补 '\0'，方便当字符串解析 */
            p->state = 4u;
        }
        return 0u;

    case 4u:                                        /* CRC */
        if (byte != p->crc)
        {
            p->bad_crc++;
            p->state = (byte == VISION_HEAD0) ? 1u : 0u;
            return 0u;
        }
        p->state = 5u;
        return 0u;

    case 5u:                                        /* 帧尾 */
        if (byte == VISION_TAIL)
        {
            p->frames_ok++;
            p->state = 0u;
            return 1u;
        }
        p->bad_tail++;
        p->state = (byte == VISION_HEAD0) ? 1u : 0u;
        return 0u;

    default:
        p->state = 0u;
        return 0u;
    }
}

void Vision_ParserTick(VisionParser *p, uint32_t now_ms)
{
    if (!p)
        return;
    if (p->state != 0u && (uint32_t)(now_ms - p->last_ms) >= VISION_HALF_FRAME_MS)
    {
        p->half_frame_drops++;
        p->state = 0u;                              /* 丢弃半帧，避免断线后吞帧 */
    }
}

/* ==========================================================================
 *  3. 回复解析
 * ========================================================================== */

static uint8_t ParseInt(const char *s, int32_t *out)
{
    int32_t  value = 0;
    uint8_t  negative = 0u;
    uint8_t  digits = 0u;

    while (*s == ' ' || *s == '\t')
        s++;
    if (*s == '+')
        s++;
    else if (*s == '-')
    {
        negative = 1u;
        s++;
    }
    while (*s >= '0' && *s <= '9')
    {
        value = value * 10 + (*s - '0');
        if (value > 1000000)
            value = 1000000;                        /* 防溢出，够用 */
        s++;
        digits = 1u;
    }
    *out = negative ? -value : value;
    return digits;
}

static VisionMsgId NameToId(const char *name)
{
    static const char *const kNames[] =
    {
        "READY", "OK", "BUSY", "NONE", "START", "TASK",
        "PICK", "DISC", "STACK", "OBST", "CLEAR", "ERR", "MOVE"
    };
    static const VisionMsgId kIds[] =
    {
        VISION_MSG_READY, VISION_MSG_OK, VISION_MSG_BUSY, VISION_MSG_NONE,
        VISION_MSG_START, VISION_MSG_TASK, VISION_MSG_PICK, VISION_MSG_DISC,
        VISION_MSG_STACK, VISION_MSG_OBST, VISION_MSG_CLEAR, VISION_MSG_ERR,
        VISION_MSG_MOVE
    };
    uint8_t i;

    for (i = 0; i < (uint8_t)(sizeof(kIds) / sizeof(kIds[0])); i++)
        if (!strcmp(name, kNames[i]))
            return kIds[i];
    return VISION_MSG_UNKNOWN;
}

const char *Vision_MsgName(VisionMsgId id)
{
    switch (id)
    {
    case VISION_MSG_READY:   return "READY";
    case VISION_MSG_OK:      return "OK";
    case VISION_MSG_BUSY:    return "BUSY";
    case VISION_MSG_NONE:    return "NONE";
    case VISION_MSG_START:   return "START";
    case VISION_MSG_TASK:    return "TASK";
    case VISION_MSG_PICK:    return "PICK";
    case VISION_MSG_DISC:    return "DISC";
    case VISION_MSG_STACK:   return "STACK";
    case VISION_MSG_OBST:    return "OBST";
    case VISION_MSG_CLEAR:   return "CLEAR";
    case VISION_MSG_ERR:     return "ERR";
    case VISION_MSG_MOVE:    return "MOVE";
    case VISION_MSG_UNKNOWN: return "UNKNOWN";
    default:                 return "-";
    }
}

uint8_t Vision_ParseReply(const char *payload, uint32_t now_ms, VisionMsg *msg)
{
    const char *q;
    uint16_t    len = 0;
    uint8_t     n = 0;

    if (!payload || !msg)
        return 0u;
    memset(msg, 0, sizeof(*msg));
    msg->rx_ms = now_ms;

    while (*payload == ' ' || *payload == '\t')
        payload++;
    while (len < (uint16_t)(VISION_TEXT_MAX - 1u) && payload[len])
    {
        msg->text[len] = payload[len];
        len++;
    }
    msg->text[len] = 0;
    while (len > 0u && (msg->text[len - 1u] == ' ' || msg->text[len - 1u] == '\t'
                        || msg->text[len - 1u] == '\r' || msg->text[len - 1u] == '\n'))
        msg->text[--len] = 0;
    if (msg->text[0] == 0)
        return 0u;

    /* 命令字：第一个逗号之前（没有逗号就取第一个字符，与视觉端解析规则对称）*/
    while (n < 8u && msg->text[n] && msg->text[n] != ',')
    {
        msg->name[n] = msg->text[n];
        n++;
    }
    if (n == 1u && msg->text[1] && msg->text[1] != ',')
        msg->name[0] = msg->text[0];                /* 兜底，单字符命令 */
    msg->name[n] = 0;

    /* 参数：按逗号切分 */
    q = msg->text + n;
    if (*q == ',')
        q++;
    while (*q && msg->argc < VISION_ARGS_MAX)
    {
        uint8_t k = 0u;
        while (k < (uint8_t)(VISION_ARG_LEN - 1u) && *q && *q != ',')
            msg->arg[msg->argc][k++] = *q++;
        msg->arg[msg->argc][k] = 0;
        ParseInt(msg->arg[msg->argc], &msg->num[msg->argc]);
        msg->argc++;
        if (*q == ',')
            q++;
    }

    msg->id = NameToId(msg->name);
    return 1u;
}

/* ==========================================================================
 *  4. 消息信箱（拆帧器与解析器解耦，会话引擎从信箱取）
 * ========================================================================== */
#define VISION_MAILBOX  4u

static VisionMsg g_mailbox[VISION_MAILBOX];
static uint8_t   g_mb_head = 0u, g_mb_tail = 0u;

void Vision_ResetMailbox(void)
{
    g_mb_head = g_mb_tail = 0u;
}

void Vision_PushMsg(const VisionMsg *msg)
{
    uint8_t next = (uint8_t)((g_mb_head + 1u) % VISION_MAILBOX);
    if (next == g_mb_tail)
        g_mb_tail = (uint8_t)((g_mb_tail + 1u) % VISION_MAILBOX);   /* 满了丢最旧 */
    g_mailbox[g_mb_head] = *msg;
    g_mb_head = next;
}

uint8_t Vision_TakeMsg(VisionMsg *msg)
{
    if (g_mb_head == g_mb_tail)
        return 0u;
    *msg = g_mailbox[g_mb_tail];
    g_mb_tail = (uint8_t)((g_mb_tail + 1u) % VISION_MAILBOX);
    return 1u;
}

/* ==========================================================================
 *  5. 会话引擎：一问一答 + BUSY + 超时 + 重发
 * ========================================================================== */

int Vision_SessionActive(const VisionSession *s)
{
    return (s && s->state == 1u);
}

void Vision_SessionCancel(VisionSession *s)
{
    if (s)
        s->state = 0u;
}

void Vision_SessionStart(VisionSession *s, const char *payload, uint32_t now_ms,
                         uint32_t timeout_ms, uint8_t max_retry)
{
    uint8_t i = 0u;

    if (!s || !payload)
        return;
    while (i < (uint8_t)(VISION_TEXT_MAX - 1u) && payload[i])
    {
        s->payload[i] = payload[i];
        i++;
    }
    s->payload[i]  = 0;
    s->state       = 1u;
    s->sent_ms     = now_ms;
    s->timeout_ms  = timeout_ms;
    s->retry       = 0u;
    s->max_retry   = max_retry;
    s->replies     = 0u;
    s->busy        = 0u;
    s->moves       = 0u;
    s->timeouts    = 0u;
    s->resends     = 0u;

    Vision_TxPayload(s->payload);
}

int Vision_SessionPoll(VisionSession *s, uint32_t now_ms, VisionMsg *msg)
{
    VisionMsg m;

    if (!s || s->state != 1u)
        return 0;

    while (Vision_TakeMsg(&m))
    {
        s->replies++;
        if (m.id == VISION_MSG_BUSY)
        {
            /* 视觉端还在算：不是失败，重置计时继续等（交接文档 §8.1）*/
            s->busy++;
            s->sent_ms = now_ms;
            continue;
        }
        if (m.id == VISION_MSG_MOVE)
        {
            /* 圆盘对齐微调请求：不是终态，交给上层执行动作后继续等 */
            s->moves++;
            s->sent_ms = now_ms;
            if (msg)
                *msg = m;
            return 2;
        }
        s->state = 0u;
        if (msg)
            *msg = m;
        return 1;
    }

    if ((uint32_t)(now_ms - s->sent_ms) >= s->timeout_ms)
    {
        s->timeouts++;
        if (s->retry < s->max_retry)
        {
            s->retry++;
            s->resends++;
            s->sent_ms = now_ms;
            Vision_TxPayload(s->payload);           /* 超时才重发，最多 max_retry 次 */
            return 0;
        }
        s->state = 0u;
        return -1;
    }
    return 0;
}

void Vision_TxPayload(const char *payload)
{
    uint8_t  frame[VISION_FRAME_MAX];
    uint16_t n = Vision_BuildFrame(payload, frame, sizeof(frame));

    /* Vision_TxBytes 是唯一的硬件边界：目标机写 UART5，主机测试里记录并模拟视觉端 */
    if (n)
        Vision_TxBytes(frame, n);
}

/* ==========================================================================
 *  6. 硬件层（UART5: PC12=TX / PD2=RX）—— 主机测试时整段跳过
 * ========================================================================== */
#ifndef VISION_HOST_TEST

#define VISION_RX_RING  256u
static volatile uint8_t  vr_buf[VISION_RX_RING];
static volatile uint16_t vr_head = 0u, vr_tail = 0u;

void Vision_IsrByte(uint8_t byte)
{
    uint16_t next = (uint16_t)((vr_head + 1u) % VISION_RX_RING);
    if (next == vr_tail)
        return;                                     /* 环形缓冲满：丢新字节 */
    vr_buf[vr_head] = byte;
    vr_head = next;
}

int Vision_RxPop(void)
{
    uint8_t b;
    if (vr_head == vr_tail)
        return -1;
    b = vr_buf[vr_tail];
    vr_tail = (uint16_t)((vr_tail + 1u) % VISION_RX_RING);
    return (int)b;
}

void Vision_TxBytes(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        USART_SendData(VISION_UART, data[i]);
        while (USART_GetFlagStatus(VISION_UART, USART_FLAG_TXE) == RESET);
    }
}

void Vision_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef  nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);   /* UART5 在 APB1 */

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);  /* PC12 = UART5_TX */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2,  GPIO_AF_UART5);  /* PD2  = UART5_RX */

    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;

    gpio.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOC, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &gpio);

    usart.USART_BaudRate            = baudrate;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(VISION_UART, &usart);

    USART_ITConfig(VISION_UART, USART_IT_RXNE, ENABLE);

    nvic.NVIC_IRQChannel                   = VISION_UART_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;     /* 低于运动控制，高于空闲 */
    nvic.NVIC_IRQChannelSubPriority        = 3;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(VISION_UART, ENABLE);
    (void)VISION_UART->SR;                          /* 清掉上电残留的错位数据 */
    (void)VISION_UART->DR;
}

/* 中断主体：既可被本文件的中断调用，也可从 usart5.c 的中断里调 */
void Vision_Isr(void)
{
    if (USART_GetITStatus(VISION_UART, USART_IT_RXNE) != RESET)
        Vision_IsrByte((uint8_t)USART_ReceiveData(VISION_UART));
    if (USART_GetFlagStatus(VISION_UART, USART_FLAG_ORE) != RESET)
        (void)USART_ReceiveData(VISION_UART);
}

#ifdef VISION_PROVIDE_UART5_IRQ
void UART5_IRQHandler(void)
{
    Vision_Isr();
}
#endif

#endif /* !VISION_HOST_TEST */

/* ==========================================================================
 *  7. 主循环泵：环形缓冲 → 拆帧器 → 信箱
 * ========================================================================== */
uint8_t Vision_Pump(VisionParser *p, uint32_t now_ms)
{
    uint8_t found = 0u;
    int     b;

    if (!p)
        return 0u;
    while ((b = Vision_RxPop()) >= 0)
    {
        if (Vision_ParserFeed(p, (uint8_t)b, now_ms))
        {
            VisionMsg m;
            if (Vision_ParseReply((const char *)p->payload, now_ms, &m))
            {
                Vision_PushMsg(&m);
                found++;
            }
        }
    }
    Vision_ParserTick(p, now_ms);
    return found;
}