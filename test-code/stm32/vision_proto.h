/**
  ******************************************************************************
  * @file    vision_proto.h
  * @brief   视觉端(MaixCAM2) ⇄ STM32 串口协议驱动
  *
  * 对应视觉端 v3.3.0 / 交接文档 AI_HANDOFF_STM32.md。
  * 本文件只依赖 stm32f4xx.h，不依赖具体 main，可直接搬进整车工程。
  *
  * 帧格式（双向一致）：
  *   ┌──────┬──────┬─────┬─────────────┬──────┬──────┐
  *   │ 0xAA │ 0x55 │ LEN │ payload[LEN]│ CRC8 │ 0x0D │
  *   └──────┴──────┴─────┴─────────────┴──────┴──────┘
  *   LEN  = payload 字节数（不含帧头/CRC/帧尾）
  *   CRC8 = 对 LEN + payload 计算，poly=0x07 init=0x00 不反射 无最终异或
  *
  * 层次：
  *   Vision_Crc8 / Vision_BuildFrame      纯函数，可单测（自检向量见 §2.1）
  *   VisionParser                          拆帧状态机，可脱离硬件单测
  *   Vision_ParseReply                     文本 → VisionMsg
  *   VisionSession                         一问一答 + BUSY + 超时 + 重发
  *   Vision_Init / Vision_TxBytes / Vision_RxPop / Vision_IsrByte
  *                                         硬件边界（主机测试里由测试文件提供）
  ******************************************************************************
  */

#ifndef VISION_PROTO_H
#define VISION_PROTO_H

#ifdef VISION_HOST_TEST
#include <stdint.h>          /* 电脑端自检：不带 STM32 头 */
#else
#include "stm32f4xx.h"
#endif

/* ------------------------------------------------------------------ 常量 */
#define VISION_HEAD0              0xAAu
#define VISION_HEAD1              0x55u
#define VISION_TAIL               0x0Du

#define VISION_PAYLOAD_MAX        48u   /* 最长 OBST 帧约 25B，留足余量 */
#define VISION_FRAME_MAX          (VISION_PAYLOAD_MAX + 5u)
#define VISION_TEXT_MAX           52u   /* 原始 payload 缓存（含结尾 0） */
#define VISION_ARGS_MAX            6u   /* 参数个数上限（OBST 有 5 个） */
#define VISION_ARG_LEN            12u

#define VISION_HALF_FRAME_MS     200u   /* 半帧超时：超过则丢弃残留（交接文档 §2.2）*/

/* 会话超时（交接文档 §8：必须 > 视觉端 TASK_TIMEOUT_MS=20000） */
#define VISION_TIMEOUT_MS      22000u   /* A/T/C/D/E/F 等 */
#define VISION_TIMEOUT_G_MS     3500u   /* G（视觉端 3s 必然回 OBST/CLEAR）*/
#define VISION_HANDSHAKE_MS     1000u   /* 握手等待 */
#define VISION_HANDSHAKE_RETRY     3u   /* 握手最多 3 次 */
#define VISION_CMD_RETRY           2u   /* 普通命令超时后最多重发 2 次 */

/* 圆盘对齐：MOVE 的物理映射，必须现场标定！
 * dx 是像素偏差，本车 MOVE 映射成前进/后退。
 * 0.5mm/px 只是占位初值：先在 d 命令下看 MOVE 的 dx 和实际位移，
 * 反推出 mm/px 再改这里，否则越调越偏。方向反了改 ALIGN_INVERT（视觉端）。 */
#define VISION_MOVE_MM_PER_PX     0.5f

/* ------------------------------------------------------------------ 消息 */
typedef enum
{
    VISION_MSG_EMPTY = 0,      /* 空消息（未解析）*/
    VISION_MSG_READY,          /* READY                 握手/复位成功 */
    VISION_MSG_OK,             /* OK                    Z 的回复 */
    VISION_MSG_BUSY,           /* BUSY                  还在算，继续等 */
    VISION_MSG_NONE,           /* NONE / NONE,tag       任务失败 */
    VISION_MSG_START,          /* START,zone,x,y,w,h    启停区 */
    VISION_MSG_TASK,           /* TASK,g1,g3            任务码解析结果 */
    VISION_MSG_PICK,           /* PICK,color,x,y        原料区 */
    VISION_MSG_DISC,           /* DISC,x,y,dx,ok        圆盘 */
    VISION_MSG_STACK,          /* STACK,x,y             码垛（无高度）*/
    VISION_MSG_OBST,           /* OBST,x,y,w,h,mm       障碍物（可能出现多次）*/
    VISION_MSG_CLEAR,          /* CLEAR                 无障碍 */
    VISION_MSG_ERR,            /* ERR,camera            视觉端异常 */
    VISION_MSG_MOVE,           /* MOVE,dx               视觉端请求微调（非终态）*/
    VISION_MSG_UNKNOWN         /* 未知帧：只记录，不报错 */
} VisionMsgId;

typedef struct
{
    VisionMsgId id;
    char        name[10];                        /* 命令字，如 "DISC" */
    char        text[VISION_TEXT_MAX];           /* 原始 payload */
    char        arg[VISION_ARGS_MAX][VISION_ARG_LEN];  /* 逗号分隔的参数 */
    int32_t     num[VISION_ARGS_MAX];            /* 参数转数字（转不了=0）*/
    uint8_t     argc;
    uint32_t    rx_ms;
} VisionMsg;

/* ------------------------------------------------------------ 拆帧状态机 */
typedef struct
{
    uint8_t  state;                 /* 0等AA 1等55 2等LEN 3收payload 4等CRC 5等0D */
    uint8_t  len;
    uint8_t  index;
    uint8_t  crc;
    uint8_t  payload[VISION_PAYLOAD_MAX + 1u];  /* 末位留给 '\0' */
    uint32_t last_ms;
    /* 统计（现场排障就靠这些数）*/
    uint32_t frames_ok, bad_crc, bad_tail, bad_len, noise, half_frame_drops;
} VisionParser;

/* ------------------------------------------------------------ 会话引擎 */
typedef struct
{
    uint8_t  state;                 /* 0=空闲 1=已发·等回复 */
    char     payload[VISION_TEXT_MAX];
    uint32_t sent_ms;
    uint32_t timeout_ms;
    uint8_t  retry;
    uint8_t  max_retry;
    /* 统计 */
    uint32_t replies, busy, moves, timeouts, resends;
} VisionSession;

/* ============================================================ 纯函数接口 */
uint8_t  Vision_Crc8(const uint8_t *data, uint16_t len);

/**
 * 组帧：payload 为 ASCII 文本（不含帧头/CRC/帧尾）
 * @return 写入 out 的字节数；0 = 参数非法或 out 太小
 */
uint16_t Vision_BuildFrame(const char *payload, uint8_t *out, uint16_t cap);

/** 打印用：把 payload 对应的整帧写成 "AA 55 03 43 2C 31 C4 0D" */
void     Vision_HexFrame(const char *payload, char *out, uint16_t cap);

/* ============================================================ 拆帧器 */
void    Vision_ParserReset(VisionParser *p, uint32_t now_ms);
/** 喂一个字节；返回 1 表示组出一帧完整 payload（在 p->payload，已补 '\0'）*/
uint8_t Vision_ParserFeed(VisionParser *p, uint8_t byte, uint32_t now_ms);
/** 半帧超时检查，主循环周期性调用 */
void    Vision_ParserTick(VisionParser *p, uint32_t now_ms);

/* ============================================================ 回复解析 */
uint8_t     Vision_ParseReply(const char *payload, uint32_t now_ms, VisionMsg *msg);
const char *Vision_MsgName(VisionMsgId id);

/* ============================================================ 消息邮箱 */
/** 把解析好的消息投递给信箱（拆帧器不带解析，主循环调 Vision_Pump 时会自动投递）*/
void    Vision_PushMsg(const VisionMsg *msg);
uint8_t Vision_TakeMsg(VisionMsg *msg);
void    Vision_ResetMailbox(void);

/* ============================================================ 会话引擎 */
void Vision_SessionStart(VisionSession *s, const char *payload, uint32_t now_ms,
                         uint32_t timeout_ms, uint8_t max_retry);
/**
 * 驱动一次会话。
 * @return  1 = 收到最终回复（已写入 *msg）
 *          2 = 收到 MOVE 微调请求（*msg 有效，会话继续；上层可执行动作）
 *          0 = 还在等
 *         -1 = 超时且重发已用尽，会话结束
 */
int  Vision_SessionPoll(VisionSession *s, uint32_t now_ms, VisionMsg *msg);
int  Vision_SessionActive(const VisionSession *s);
void Vision_SessionCancel(VisionSession *s);

/* ============================================================ 硬件边界 */
/** 初始化视觉串口（UART5: PC12=TX / PD2=RX，115200 8N1）*/
void    Vision_Init(uint32_t baudrate);
/** 真正把字节写出去（主机测试里由测试文件提供同名函数）*/
void    Vision_TxBytes(const uint8_t *data, uint16_t len);
/** 从接收环形缓冲取一个字节；返回 -1 = 空 */
int     Vision_RxPop(void);
/** 中断里调用：收到一个字节 */
void    Vision_IsrByte(uint8_t byte);

/** 把 payload 组帧后发出去 */
void    Vision_TxPayload(const char *payload);
/** 把环形缓冲里的字节全部喂给拆帧器，完整帧自动投递到信箱；返回本次新增帧数 */
uint8_t Vision_Pump(VisionParser *p, uint32_t now_ms);

#endif /* VISION_PROTO_H */