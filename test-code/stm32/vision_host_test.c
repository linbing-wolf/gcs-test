/**
  ******************************************************************************
  * @file    vision_host_test.c
  * @brief   vision_proto.c 的电脑端自检（不需要 STM32 硬件）
  *
  * 编译运行（Linux / 有 gcc 的机器）：
  *   gcc -DVISION_HOST_TEST -std=c99 -Wall -Wextra -Werror \
  *       vision_host_test.c vision_proto.c -o vision_host_test && ./vision_host_test
  *
  * 覆盖：
  *   1) CRC 与组帧：与交接文档 §2.1 的 5 条实测向量逐字节比对
  *   2) 拆帧：正常帧 / 前置噪声 / 粘连两帧 / 逐字节分片 / 错 CRC 丢弃 /
  *            错帧尾丢弃 / LEN 超限 / 半帧超时丢弃
  *   3) 回复解析：13 种回复 + 负数参数 + 未知帧
  *   4) 会话引擎：正常 / BUSY 不算失败 / MOVE 循环 / 超时重发 / 重发用尽
  *   5) 全流程脚本：模拟视觉端跑完交接文档 §7 的 11 条往来
  *   6) 边界与接口完整性：组帧的 NULL/容量/超长、MsgName 全覆盖、回复字段
  *      （argc 截断/空/NULL/未知名/超长命令字）、信箱 FIFO 与溢出、
  *      会话 Active/Cancel 与"残留回复不串台"、TxPayload 一致性、Pump 多帧
  ******************************************************************************
  */

#define VISION_HOST_TEST 1

#include "vision_proto.h"

#include <stdio.h>
#include <string.h>

/* ==========================================================================
 *  测试框架
 * ========================================================================== */
static int g_checks = 0;
static int g_fails  = 0;

#define CHECK(cond, msg)                                           \
    do {                                                           \
        g_checks++;                                                \
        if (!(cond)) { g_fails++; printf("  [FAIL] %s\n", (msg)); }\
    } while (0)

static void Group(const char *name)
{
    printf("== %s ==\n", name);
}

static void Ok(const char *name)
{
    printf("  [OK] %s\n", name);
}

/* ==========================================================================
 *  硬件边界替身：TX 记录 + 模拟视觉端 + RX 假队列
 * ========================================================================== */
#define RXQ_SIZE 4096
static uint8_t g_rxq[RXQ_SIZE];
static int     g_rxq_head = 0, g_rxq_tail = 0;

static void RxPush(uint8_t b)
{
    int next = (g_rxq_head + 1) % RXQ_SIZE;
    if (next == g_rxq_tail)
        return;                                  /* 满了就丢，测试不会到这 */
    g_rxq[g_rxq_head] = b;
    g_rxq_head = next;
}

static void RxPushPayload(const char *payload)
{
    uint8_t  frame[VISION_FRAME_MAX];
    uint16_t n = Vision_BuildFrame(payload, frame, sizeof(frame));
    uint16_t i;
    for (i = 0; i < n; i++)
        RxPush(frame[i]);
}

static void RxPushRaw(const uint8_t *bytes, uint16_t n)
{
    uint16_t i;
    for (i = 0; i < n; i++)
        RxPush(bytes[i]);
}

int Vision_RxPop(void)
{
    uint8_t b;
    if (g_rxq_head == g_rxq_tail)
        return -1;
    b = g_rxq[g_rxq_tail];
    g_rxq_tail = (g_rxq_tail + 1) % RXQ_SIZE;
    return (int)b;
}

/* --- 发送记录 --- */
#define TXLOG_MAX 32
static char     g_tx_payload[TXLOG_MAX][VISION_TEXT_MAX];
static uint16_t g_tx_len[TXLOG_MAX];
static uint8_t  g_tx_bytes[TXLOG_MAX][VISION_FRAME_MAX];
static int      g_tx_count = 0;

static const char *g_last_cmd = "";

/* 模拟视觉端的行为开关 */
enum { FAKE_NORMAL = 0, FAKE_SILENT, FAKE_BUSY_FIRST };
static int g_fake_mode = FAKE_NORMAL;

/* 命令 → 回复（模拟视觉端；D 会先发两条 MOVE）*/
static void FakeReact(const char *cmd)
{
    if (g_fake_mode == FAKE_SILENT)
        return;

    if (g_fake_mode == FAKE_BUSY_FIRST)
    {
        RxPushPayload("BUSY");
        g_fake_mode = FAKE_NORMAL;
    }

    if (!strcmp(cmd, "D"))
    {
        RxPushPayload("MOVE,35");
        RxPushPayload("MOVE,-8");
        RxPushPayload("DISC,318,242,-2,1");
        return;
    }
    if (!strcmp(cmd, "E")) { RxPushPayload("DISC,320,240,0,1");  return; }
    if (!strcmp(cmd, "R")) { RxPushPayload("READY");             return; }
    if (!strcmp(cmd, "A")) { RxPushPayload("START,1,320,240,120,80"); return; }
    if (!strcmp(cmd, "T,134+123+314+231")) { RxPushPayload("TASK,134,314"); return; }
    if (!strcmp(cmd, "T,999+000+999+000")) { RxPushPayload("NONE,T");        return; }
    if (!strcmp(cmd, "C,1")) { RxPushPayload("PICK,1,312,238");  return; }
    if (!strcmp(cmd, "C,3")) { RxPushPayload("PICK,3,280,250");  return; }
    if (!strcmp(cmd, "C,4")) { RxPushPayload("PICK,4,360,260");  return; }
    if (!strcmp(cmd, "C,9")) { RxPushPayload("NONE,C");          return; }
    if (!strcmp(cmd, "F,2")) { RxPushPayload("STACK,340,180");   return; }
    if (!strcmp(cmd, "G"))   { RxPushPayload("CLEAR");           return; }
    if (!strcmp(cmd, "Z"))   { RxPushPayload("OK");              return; }
    /* 其它命令：模拟视觉端"静默忽略未知命令"，什么也不回 */
}

static char *FrameToPayload(const uint8_t *f, uint16_t n)
{
    static char buf[VISION_TEXT_MAX];
    uint16_t len, i;
    if (n < 5)
        return NULL;
    len = f[2];
    for (i = 0; i < len && i < VISION_TEXT_MAX - 1; i++)
        buf[i] = (char)f[3 + i];
    buf[len] = 0;
    return buf;
}

void Vision_TxBytes(const uint8_t *data, uint16_t len)
{
    if (g_tx_count < TXLOG_MAX)
    {
        char *p = FrameToPayload(data, len);
        if (p)
            strncpy(g_tx_payload[g_tx_count], p, VISION_TEXT_MAX - 1);
        g_tx_payload[g_tx_count][VISION_TEXT_MAX - 1] = 0;
        g_tx_len[g_tx_count] = len;
        memcpy(g_tx_bytes[g_tx_count], data, len);
        g_tx_count++;
        g_last_cmd = g_tx_payload[g_tx_count - 1];
    }
    FakeReact(g_last_cmd);                        /* 模拟视觉端的回复 */
}

static void ResetAll(void)
{
    g_rxq_head = g_rxq_tail = 0;
    g_tx_count = 0;
    g_fake_mode = FAKE_NORMAL;
    Vision_ResetMailbox();
    g_last_cmd = "";
}

/* ==========================================================================
 *  1. CRC / 组帧：交接文档 §2.1 的实测向量
 * ========================================================================== */
static void TestFrameVectors(void)
{
    static const struct { const char *payload; const char *hex; } kVec[] =
    {
        { "D",                   "AA 55 01 44 CE 0D" },
        { "R",                   "AA 55 01 52 AC 0D" },
        { "Z",                   "AA 55 01 5A 94 0D" },
        { "C,1",                 "AA 55 03 43 2C 31 C4 0D" },
        { "T,134+123+314+231",   "AA 55 11 54 2C 31 33 34 2B 31 32 33 2B 33 31 34 2B 32 33 31 B1 0D" }
    };
    unsigned i;

    Group("CRC / 组帧（交接文档 §2.1 向量）");
    for (i = 0; i < sizeof(kVec) / sizeof(kVec[0]); i++)
    {
        char     hex[128];
        uint8_t  frame[VISION_FRAME_MAX];
        uint16_t n = Vision_BuildFrame(kVec[i].payload, frame, sizeof(frame));

        Vision_HexFrame(kVec[i].payload, hex, sizeof(hex));
        CHECK(strcmp(hex, kVec[i].hex) == 0, kVec[i].payload);
        /* CRC 必须只覆盖 LEN+payload：单独再算一遍比对 */
        CHECK(Vision_Crc8(&frame[2], (uint16_t)(n - 4u)) == frame[n - 2u], "CRC 覆盖 LEN+payload");
    }
    Ok("5 条向量逐字节一致（CRC 只覆盖 LEN+payload，不含 AA55/0D）");
}

/* ==========================================================================
 *  2. 拆帧
 * ========================================================================== */
static uint8_t FeedString(VisionParser *p, const char *payload, uint32_t now)
{
    uint8_t  frame[VISION_FRAME_MAX];
    uint16_t n = Vision_BuildFrame(payload, frame, sizeof(frame));
    uint16_t i;
    uint8_t  hits = 0;
    for (i = 0; i < n; i++)
        if (Vision_ParserFeed(p, frame[i], now))
            hits++;
    return hits;
}

static void TestParser(void)
{
    VisionParser p;

    Group("拆帧：噪声 / 粘连 / 分片 / 坏帧 / 半帧");

    /* 正常帧 */
    Vision_ParserReset(&p, 0);
    CHECK(FeedString(&p, "READY", 0) == 1, "正常帧应解析出 1 帧");
    CHECK(strcmp((char *)p.payload, "READY") == 0, "payload 内容");

    /* 前置噪声字节 */
    Vision_ParserReset(&p, 0);
    {
        uint8_t noise[4] = { 0x00, 0xAA, 0x13, 0x55 };
        RxPushRaw(noise, 0);                      /* 只跑解析器，不经过队列 */
        {
            uint8_t  frame[VISION_FRAME_MAX];
            uint16_t n = Vision_BuildFrame("OK", frame, sizeof(frame));
            uint16_t i;
            uint8_t  hits = 0;
            for (i = 0; i < 4; i++)
                Vision_ParserFeed(&p, noise[i], 0);
            for (i = 0; i < n; i++)
                if (Vision_ParserFeed(&p, frame[i], 0))
                    hits++;
            CHECK(hits == 1, "噪声前置：仍能对齐并解析");
        }
    }

    /* 粘连：一包两帧 */
    Vision_ParserReset(&p, 0);
    CHECK(FeedString(&p, "READY", 0) + FeedString(&p, "OK", 0) == 2, "粘连两帧都能出");

    /* 逐字节分片（每次只喂 1 字节，中间穿插半帧超时检查但不能丢）*/
    Vision_ParserReset(&p, 0);
    {
        uint8_t  frame[VISION_FRAME_MAX];
        uint16_t n = Vision_BuildFrame("DISC,318,242,-2,1", frame, sizeof(frame));
        uint16_t i;
        uint8_t  hits = 0;
        for (i = 0; i < n; i++)
        {
            Vision_ParserTick(&p, (uint32_t)i * 10u);     /* 累计 <200ms，不该丢 */
            if (Vision_ParserFeed(&p, frame[i], (uint32_t)i * 10u))
                hits++;
        }
        CHECK(hits == 1, "逐字节分片应能组出帧");
        CHECK(strcmp((char *)p.payload, "DISC,318,242,-2,1") == 0, "分片内容正确");
    }

    /* 错 CRC：丢弃，且不能误判成好帧 */
    Vision_ParserReset(&p, 0);
    {
        uint8_t  frame[VISION_FRAME_MAX];
        uint16_t n = Vision_BuildFrame("READY", frame, sizeof(frame));
        uint16_t i;
        uint8_t  hits = 0;
        frame[n - 2u] ^= 0xFFu;                   /* 破坏 CRC */
        for (i = 0; i < n; i++)
            if (Vision_ParserFeed(&p, frame[i], 0))
                hits++;
        CHECK(hits == 0, "坏 CRC 必须丢弃");
        CHECK(p.bad_crc == 1u, "bad_crc 计数");
        CHECK(FeedString(&p, "OK", 0) == 1, "坏帧之后仍能收好帧");
    }

    /* 错帧尾：丢弃 */
    Vision_ParserReset(&p, 0);
    {
        uint8_t  frame[VISION_FRAME_MAX];
        uint16_t n = Vision_BuildFrame("READY", frame, sizeof(frame));
        uint16_t i;
        uint8_t  hits = 0;
        frame[n - 1u] = 0x00u;                    /* 破坏帧尾 */
        for (i = 0; i < n; i++)
            if (Vision_ParserFeed(&p, frame[i], 0))
                hits++;
        CHECK(hits == 0 && p.bad_tail == 1u, "坏帧尾丢弃并计数");
    }

    /* LEN 超限：立刻判非法并重新对齐 */
    Vision_ParserReset(&p, 0);
    {
        uint8_t bad[3] = { 0xAA, 0x55, 0xFF };
        uint8_t i;
        for (i = 0; i < 3; i++)
            Vision_ParserFeed(&p, bad[i], 0);
        CHECK(p.bad_len == 1u, "LEN>上限应计数");
        CHECK(FeedString(&p, "CLEAR", 0) == 1, "LEN 出错后仍能对齐");
    }

    /* 半帧超时 */
    Vision_ParserReset(&p, 0);
    {
        uint8_t  frame[VISION_FRAME_MAX];
        uint16_t n = Vision_BuildFrame("READY", frame, sizeof(frame));
        uint16_t i;
        for (i = 0; i < 3; i++)
            Vision_ParserFeed(&p, frame[i], 0);       /* 只喂半帧 */
        Vision_ParserTick(&p, VISION_HALF_FRAME_MS - 1u);
        CHECK(p.half_frame_drops == 0u, "未到 200ms 不该丢");
        Vision_ParserTick(&p, VISION_HALF_FRAME_MS);
        CHECK(p.half_frame_drops == 1u && p.state == 0u, "满 200ms 丢弃半帧");
        CHECK(FeedString(&p, "OK", VISION_HALF_FRAME_MS + 1u) == 1, "丢弃后仍能收新帧");
        (void)n;
    }
    Ok("正常/噪声前置/粘连/分片/坏CRC/坏帧尾/LEN超限/半帧200ms 全部符合预期");
}

/* ==========================================================================
 *  3. 回复解析
 * ========================================================================== */
static void TestReplyParse(void)
{
    static const struct
    {
        const char *payload; VisionMsgId id; uint8_t argc; int32_t a0, a1;
    } kCase[] =
    {
        { "READY",                 VISION_MSG_READY,  0, 0,    0   },
        { "OK",                    VISION_MSG_OK,     0, 0,    0   },
        { "BUSY",                  VISION_MSG_BUSY,   0, 0,    0   },
        { "CLEAR",                 VISION_MSG_CLEAR,  0, 0,    0   },
        { "NONE,T",                VISION_MSG_NONE,   1, 0,    0   },
        { "NONE,A",                VISION_MSG_NONE,   1, 0,    0   },
        { "ERR,camera",            VISION_MSG_ERR,    1, 0,    0   },
        { "START,1,320,240,120,80",VISION_MSG_START,  5, 1,  320   },
        { "TASK,134,314",          VISION_MSG_TASK,   2, 134,  314 },
        { "PICK,1,312,238",        VISION_MSG_PICK,   3, 1,  312   },
        { "DISC,318,242,-2,1",     VISION_MSG_DISC,   4, 318,  242 },
        { "STACK,340,180",         VISION_MSG_STACK,  2, 340,  180 },
        { "OBST,300,200,60,120,850",VISION_MSG_OBST,  5, 300,  200 },
        { "MOVE,35",               VISION_MSG_MOVE,   1, 35,   0   },
        { "MOVE,-8",               VISION_MSG_MOVE,   1, -8,   0   },
        { "WHATEVER,1",            VISION_MSG_UNKNOWN,1, 1,    0   }
    };
    unsigned i;
    VisionMsg m;

    Group("回复解析");
    for (i = 0; i < sizeof(kCase) / sizeof(kCase[0]); i++)
    {
        CHECK(Vision_ParseReply(kCase[i].payload, 1234u, &m) == 1u, kCase[i].payload);
        CHECK(m.id == kCase[i].id, kCase[i].payload);
        CHECK(m.argc == kCase[i].argc, kCase[i].payload);
        if (kCase[i].argc > 0u)
            CHECK(m.num[0] == kCase[i].a0, kCase[i].payload);
        if (kCase[i].argc > 1u)
            CHECK(m.num[1] == kCase[i].a1, kCase[i].payload);
        CHECK(m.rx_ms == 1234u, kCase[i].payload);
    }
    /* 负数 dx 必须解析成负值（对齐方向靠它）*/
    Vision_ParseReply("DISC,318,242,-2,1", 0u, &m);
    CHECK(m.num[2] == -2, "DISC 的 dx 要保留负号");
    Vision_ParseReply("MOVE,-8", 0u, &m);
    CHECK(m.num[0] == -8, "MOVE 的 dx 要保留负号");
    /* 尾随空格/回车要好处理 */
    Vision_ParseReply("READY \r", 0u, &m);
    CHECK(m.id == VISION_MSG_READY, "尾部空白要裁掉");
    Vision_MsgName(VISION_MSG_DISC);
    Ok("16 类回复 + 负数 dx + 尾部空白 全部解析正确");
}

/* ==========================================================================
 *  4. 会话引擎
 * ========================================================================== */
static int RunSession(VisionSession *s, VisionParser *p, uint32_t *t, VisionMsg *out,
                      uint32_t step_ms, uint32_t max_ms)
{
    int rc;
    uint32_t end = *t + max_ms;
    while (*t < end)
    {
        Vision_Pump(p, *t);
        rc = Vision_SessionPoll(s, *t, out);
        if (rc == 1 || rc == -1)
            return rc;
        /* rc==2（MOVE）和其它：继续等最终回复 */
        *t += step_ms;
    }
    return 0;
}

static void TestSession(void)
{
    VisionSession s;
    VisionParser  p;
    VisionMsg     m;
    uint32_t      t;

    Group("会话引擎：BUSY / MOVE / 超时重发");

    /* 正常一问一答 */
    ResetAll();
    Vision_ParserReset(&p, 0);
    t = 0;
    Vision_SessionStart(&s, "R", t, VISION_HANDSHAKE_MS, VISION_HANDSHAKE_RETRY);
    CHECK(g_tx_count == 1, "发送后立刻写了一帧");
    CHECK(RunSession(&s, &p, &t, &m, 1, 100) == 1, "应收到最终回复");
    CHECK(m.id == VISION_MSG_READY, "回复是 READY");
    Ok("正常一问一答：READY");

    /* BUSY 不能当失败 */
    ResetAll();
    Vision_ParserReset(&p, 0);
    g_fake_mode = FAKE_BUSY_FIRST;
    t = 0;
    Vision_SessionStart(&s, "C,1", t, VISION_TIMEOUT_MS, VISION_CMD_RETRY);
    CHECK(RunSession(&s, &p, &t, &m, 1, 100) == 1, "BUSY 之后仍应拿到结果");
    CHECK(m.id == VISION_MSG_PICK && m.num[1] == 312, "结果是 PICK 坐标");
    CHECK(s.busy == 1u, "BUSY 计数");
    CHECK(g_tx_count == 1, "BUSY 期间不该重发");
    Ok("收到 BUSY：重置计时且不重发，最终拿到 PICK");

    /* MOVE 循环：D 先来 2 条 MOVE 再来 DISC */
    ResetAll();
    Vision_ParserReset(&p, 0);
    t = 0;
    Vision_SessionStart(&s, "D", t, VISION_TIMEOUT_MS, VISION_CMD_RETRY);
    {
        int rc, moves = 0;
        uint32_t end = t + 500;
        do
        {
            Vision_Pump(&p, t);
            rc = Vision_SessionPoll(&s, t, &m);
            if (rc == 2)
            {
                moves++;
                CHECK(m.id == VISION_MSG_MOVE, "MOVE 消息");
            }
            t++;
        } while ((rc == 0 || rc == 2) && t < end);
        CHECK(rc == 1, "D 最终应回 DISC");
        CHECK(moves == 2, "应收到 2 条 MOVE");
        CHECK(m.id == VISION_MSG_DISC && m.num[2] == -2 && m.num[3] == 1, "DISC 字段（dx 与 ok）");
        CHECK(s.moves == 2u, "会话 MOVE 计数");
    }
    Ok("D：MOVE×2 → DISC,x,y,-2,1（MOVE 不当终态）");

    /* 超时 → 重发（2 次）后再超时 → -1 */
    ResetAll();
    Vision_ParserReset(&p, 0);
    g_fake_mode = FAKE_SILENT;
    t = 0;
    Vision_SessionStart(&s, "A", t, 1000, VISION_CMD_RETRY);
    CHECK(RunSession(&s, &p, &t, &m, 10, 10000) == -1, "一直不回应判超时失败");
    CHECK(g_tx_count == 3, "首帧 + 重发 2 次 = 3 帧");
    CHECK(s.timeouts == 3u && s.resends == 2u, "超时/重发计数");
    Ok("超时：重发 2 次后判失败（共 3 帧）");

    /* 超时之后重发成功 */
    ResetAll();
    Vision_ParserReset(&p, 0);
    g_fake_mode = FAKE_SILENT;
    t = 0;
    Vision_SessionStart(&s, "G", t, 500, VISION_CMD_RETRY);
    {
        int rc;
        /* 第一次静默：跑到超时，会话应重发 */
        rc = RunSession(&s, &p, &t, &m, 10, 600);
        CHECK(rc == 0 && g_tx_count == 2, "第一次超时应重发");
        g_fake_mode = FAKE_NORMAL;              /* 恢复：模拟视觉端活了 */
        rc = RunSession(&s, &p, &t, &m, 10, 600);
        CHECK(rc == 1 && m.id == VISION_MSG_CLEAR, "重发后应拿到 CLEAR");
    }
    Ok("重发后视觉端恢复：拿到 CLEAR");
}

/* ==========================================================================
 *  5. 全流程脚本（交接文档 §7）
 * ========================================================================== */
static void TestScript(void)
{
    static const struct { const char *cmd; VisionMsgId expect; } kSteps[] =
    {
        { "R",                   VISION_MSG_READY },
        { "A",                   VISION_MSG_START },
        { "T,134+123+314+231",   VISION_MSG_TASK  },
        { "C,1",                 VISION_MSG_PICK  },
        { "C,3",                 VISION_MSG_PICK  },
        { "C,4",                 VISION_MSG_PICK  },
        { "D",                   VISION_MSG_DISC  },
        { "E",                   VISION_MSG_DISC  },
        { "F,2",                 VISION_MSG_STACK },
        { "G",                   VISION_MSG_CLEAR },
        { "Z",                   VISION_MSG_OK    }
    };
    unsigned i;
    int pass = 0;

    Group("全流程脚本（模拟视觉端跑完 §7 的 11 条往来）");

    for (i = 0; i < sizeof(kSteps) / sizeof(kSteps[0]); i++)
    {
        VisionSession s;
        VisionParser  p;
        VisionMsg     m;
        uint32_t      t = 0;
        int           rc;

        ResetAll();
        Vision_ParserReset(&p, 0);
        Vision_SessionStart(&s, kSteps[i].cmd, 0u, VISION_TIMEOUT_MS, VISION_CMD_RETRY);
        rc = RunSession(&s, &p, &t, &m, 5, 2000);
        CHECK(rc == 1, kSteps[i].cmd);
        CHECK(m.id == kSteps[i].expect, kSteps[i].cmd);
        if (rc == 1 && m.id == kSteps[i].expect)
            pass++;
    }
    printf("  脚本通过 %d/%d\n", pass, (int)(sizeof(kSteps) / sizeof(kSteps[0])));

    /* 非法任务码：NONE,T */
    {
        VisionSession s;
        VisionParser  p;
        VisionMsg     m;
        uint32_t      t = 0;
        ResetAll();
        Vision_ParserReset(&p, 0);
        Vision_SessionStart(&s, "T,999+000+999+000", 0u, VISION_TIMEOUT_MS, VISION_CMD_RETRY);
        CHECK(RunSession(&s, &p, &t, &m, 5, 2000) == 1 && m.id == VISION_MSG_NONE,
              "非法任务码应回 NONE");
        CHECK(!strcmp(m.arg[0], "T"), "NONE 的 tag=T");
    }

    /* 未知命令静默：不能把"没回复"当成收到帧 */
    {
        VisionSession s;
        VisionParser  p;
        VisionMsg     m;
        uint32_t      t = 0;
        ResetAll();
        Vision_ParserReset(&p, 0);
        g_fake_mode = FAKE_SILENT;
        Vision_SessionStart(&s, "X,9", 0u, 300u, 0u);
        CHECK(RunSession(&s, &p, &t, &m, 10, 1000) == -1, "未知命令无回复 → 超时失败");
    }
    Ok("脚本 + 非法任务码 + 未知命令静默");
}

/* ==========================================================================
 *  6. 边界与接口完整性（补齐原测试未覆盖的公开 API）
 * ========================================================================== */
static void TestBuildFrameEdges(void)
{
    uint8_t frame[VISION_FRAME_MAX];
    char    big[VISION_PAYLOAD_MAX + 2u];

    Group("组帧边界：空指针 / 容量 / 超长");

    CHECK(Vision_BuildFrame(NULL, frame, sizeof(frame)) == 0, "payload=NULL → 0");
    CHECK(Vision_BuildFrame("R", NULL, 16) == 0, "out=NULL → 0");
    CHECK(Vision_BuildFrame("R", frame, 5) == 0, "cap=5 不够(R 需 6) → 0");
    CHECK(Vision_BuildFrame("R", frame, 6) == 6, "cap=6 刚好 → 6");
    CHECK(Vision_BuildFrame("", frame, sizeof(frame)) == 5, "空 payload → 5 字节帧");

    memset(big, 'A', VISION_PAYLOAD_MAX);
    big[VISION_PAYLOAD_MAX] = 0;
    CHECK(Vision_BuildFrame(big, frame, sizeof(frame)) == VISION_PAYLOAD_MAX + 5u,
          "payload=LEN_MAX 允许");
    big[VISION_PAYLOAD_MAX] = 'A';
    big[VISION_PAYLOAD_MAX + 1u] = 0;
    CHECK(Vision_BuildFrame(big, frame, sizeof(frame)) == 0, "payload=LEN_MAX+1 拒绝");
    Ok("组帧边界（NULL / 容量 / 超长）");
}

static void TestMsgName(void)
{
    static const struct { VisionMsgId id; const char *s; } kNames[] =
    {
        { VISION_MSG_READY,   "READY" },   { VISION_MSG_OK,     "OK" },
        { VISION_MSG_BUSY,    "BUSY" },    { VISION_MSG_NONE,   "NONE" },
        { VISION_MSG_START,   "START" },   { VISION_MSG_TASK,   "TASK" },
        { VISION_MSG_PICK,    "PICK" },    { VISION_MSG_DISC,   "DISC" },
        { VISION_MSG_STACK,   "STACK" },   { VISION_MSG_OBST,   "OBST" },
        { VISION_MSG_CLEAR,   "CLEAR" },   { VISION_MSG_ERR,    "ERR" },
        { VISION_MSG_MOVE,    "MOVE" },    { VISION_MSG_UNKNOWN,"UNKNOWN" }
    };
    unsigned i;

    Group("名字表：每个 MsgId 都要有名字，越界要安全");
    for (i = 0; i < sizeof(kNames) / sizeof(kNames[0]); i++)
        CHECK(strcmp(Vision_MsgName(kNames[i].id), kNames[i].s) == 0, kNames[i].s);
    CHECK(strcmp(Vision_MsgName(VISION_MSG_EMPTY), "-") == 0, "EMPTY → -");
    CHECK(strcmp(Vision_MsgName((VisionMsgId)999), "-") == 0, "越界 id → -");
    Ok("MsgName 全覆盖（含 EMPTY 与越界）");
}

static void TestReplyFields(void)
{
    VisionMsg m;

    Group("回复字段：name / argc / arg / num / text / rx_ms");

    CHECK(Vision_ParseReply("OBST,12,34,56,78,910", 1234u, &m) == 1, "OBST 解析");
    CHECK(m.id == VISION_MSG_OBST && m.argc == 5u, "OBST 5 个参数");
    CHECK(m.num[4] == 910 && m.rx_ms == 1234u, "第 5 个参数与 rx_ms");
    CHECK(strcmp(m.name, "OBST") == 0, "name");
    CHECK(strcmp(m.arg[0], "12") == 0, "arg 文本");

    CHECK(Vision_ParseReply("  \t DISC,318,242,-2,1 \r\n", 7u, &m) == 1, "首尾空白");
    CHECK(m.id == VISION_MSG_DISC && m.num[2] == -2 && m.argc == 4u, "DISC 负数 dx");
    CHECK(strcmp(m.text, "DISC,318,242,-2,1") == 0, "text 已去空白");

    CHECK(Vision_ParseReply("OBST,1,2,3,4,5,6,7,8", 0u, &m) == 1, "参数超上限不越界");
    CHECK(m.argc == VISION_ARGS_MAX, "参数截断到 VISION_ARGS_MAX");
    CHECK(m.num[0] == 1 && m.num[VISION_ARGS_MAX - 1u] == 6, "截断后数值正确");

    CHECK(Vision_ParseReply("", 0u, &m) == 0, "空串 → 0");
    CHECK(Vision_ParseReply("   \t ", 0u, &m) == 0, "纯空白 → 0");
    CHECK(Vision_ParseReply(NULL, 0u, &m) == 0, "NULL → 0");

    CHECK(Vision_ParseReply("HELLO,1", 0u, &m) == 1 && m.id == VISION_MSG_UNKNOWN,
          "未知名 → UNKNOWN（不报错）");
    CHECK(strcmp(m.name, "HELLO") == 0, "未知名保留");

    CHECK(Vision_ParseReply("READY", 0u, &m) == 1 && strcmp(m.name, "READY") == 0
          && m.argc == 0u, "无参数命令");

    /* 命令字超过 8 字符：截断但不越界、不崩 */
    CHECK(Vision_ParseReply("ABCDEFGHIJK,1", 0u, &m) == 1, "超长命令字不崩");
    CHECK(strlen(m.name) <= 8u, "命令字截断到 8 字符");
    Ok("回复字段与边界（超参数截断 / 空 / NULL / 未知名）");
}

static void TestMailbox(void)
{
    VisionMsg m, in;
    int i;

    Group("消息信箱：FIFO / 空 / 溢出丢最旧");

    ResetAll();
    CHECK(Vision_TakeMsg(&m) == 0, "空信箱 → 0");

    memset(&in, 0, sizeof(in));
    in.id = VISION_MSG_READY; in.num[0] = 11; Vision_PushMsg(&in);
    in.id = VISION_MSG_OK;    in.num[0] = 22; Vision_PushMsg(&in);
    CHECK(Vision_TakeMsg(&m) == 1 && m.num[0] == 11, "FIFO 顺序 1");
    CHECK(Vision_TakeMsg(&m) == 1 && m.num[0] == 22, "FIFO 顺序 2");
    CHECK(Vision_TakeMsg(&m) == 0, "取完变空");

    /* 容量为 VISION_MAILBOX-1 条；塞满再塞应丢最旧、保留最新 */
    for (i = 1; i <= 6; i++)
    {
        memset(&in, 0, sizeof(in));
        in.id = VISION_MSG_OBST; in.num[0] = i;
        Vision_PushMsg(&in);
    }
    i = 0;
    while (Vision_TakeMsg(&m))
        i++;
    CHECK(i == 3, "溢出后最多保留 3 条");
    Ok("信箱 FIFO / 空 / 溢出策略");
}

static void TestSessionStateApi(void)
{
    VisionSession s;
    VisionMsg     m;
    uint32_t      t = 0;
    int           rc;

    Group("会话状态接口：Active / Cancel / 残留回复不串台");

    memset(&s, 0, sizeof(s));
    CHECK(Vision_SessionActive(&s) == 0, "未启动 → 不活跃");
    CHECK(Vision_SessionActive(NULL) == 0, "Active(NULL) 安全");

    ResetAll();
    Vision_SessionStart(&s, "R", 0u, VISION_TIMEOUT_MS, VISION_CMD_RETRY);
    CHECK(Vision_SessionActive(&s) == 1, "启动后 → 活跃");
    CHECK(g_tx_count == 1 && strcmp(g_last_cmd, "R") == 0, "启动即发 R");

    Vision_SessionCancel(&s);
    CHECK(Vision_SessionActive(&s) == 0, "Cancel 后 → 不活跃");
    Vision_SessionCancel(NULL);
    CHECK(1, "Cancel(NULL) 不崩");
    rc = Vision_SessionPoll(&s, 0u, &m);
    CHECK(rc == 0, "已取消的会话不再消费回复");
    CHECK(g_tx_count == 1, "取消后不再发帧");

    /* 上一条命令的残留回复不能被下一条命令认领（信箱残留）*/
    ResetAll();
    {
        VisionParser p;
        VisionMsg    stale;
        memset(&p, 0, sizeof(p));
        memset(&stale, 0, sizeof(stale));
        Vision_ParserReset(&p, 0);

        Vision_SessionStart(&s, "R", 0u, VISION_TIMEOUT_MS, VISION_CMD_RETRY);
        Vision_Pump(&p, 0u);                       /* READY 被解析进信箱 */
        Vision_SessionCancel(&s);                  /* 故意不取，留下残留 */
        CHECK(Vision_TakeMsg(&stale) == 1 && stale.id == VISION_MSG_READY,
              "确认信箱里确实有残留 READY");
        Vision_PushMsg(&stale);                    /* 再塞回去，复现现场情形 */

        t = 100u;
        Vision_SessionStart(&s, "Z", t, VISION_TIMEOUT_MS, VISION_CMD_RETRY);
        CHECK(Vision_TakeMsg(&stale) == 0, "SessionStart 必须清掉残留回复");
        rc = RunSession(&s, &p, &t, &m, 5u, 500u);
        CHECK(rc == 1 && m.id == VISION_MSG_OK, "新会话只拿到自己的 OK，不认领残留 READY");
    }
    Ok("Active / Cancel / 残留回复隔离");
}

static void TestTxPath(void)
{
    uint8_t  want[VISION_FRAME_MAX];
    uint16_t n;
    char     big[VISION_PAYLOAD_MAX + 2u];

    Group("发送路径：TxPayload 与 BuildFrame 逐字节一致 + 非法不发");

    ResetAll();
    Vision_TxPayload("C,1");
    n = Vision_BuildFrame("C,1", want, sizeof(want));
    CHECK(g_tx_count == 1, "发出一帧");
    CHECK(g_tx_len[0] == n, "长度一致");
    CHECK(memcmp(g_tx_bytes[0], want, n) == 0, "字节一致");
    CHECK(strcmp(g_tx_payload[0], "C,1") == 0, "payload 一致");

    memset(big, 'A', VISION_PAYLOAD_MAX);
    big[VISION_PAYLOAD_MAX] = 'A';
    big[VISION_PAYLOAD_MAX + 1u] = 0;
    ResetAll();
    Vision_TxPayload(big);
    CHECK(g_tx_count == 0, "非法 payload：一个字节都不发");
    Ok("TxPayload 组帧 / 拒发");
}

static void TestPumpMulti(void)
{
    VisionParser p;
    VisionMsg    m;
    uint8_t      got;

    Group("Vision_Pump：粘连多帧 / 空队列 / NULL");

    ResetAll();
    Vision_ParserReset(&p, 0);
    RxPushPayload("READY");
    RxPushPayload("BUSY");
    RxPushPayload("CLEAR");
    got = Vision_Pump(&p, 10u);
    CHECK(got == 3, "3 帧一次泵出");
    CHECK(Vision_TakeMsg(&m) == 1 && m.id == VISION_MSG_READY, "顺序 1 READY");
    CHECK(Vision_TakeMsg(&m) == 1 && m.id == VISION_MSG_BUSY,  "顺序 2 BUSY");
    CHECK(Vision_TakeMsg(&m) == 1 && m.id == VISION_MSG_CLEAR, "顺序 3 CLEAR");
    CHECK(Vision_TakeMsg(&m) == 0, "信箱取空");
    CHECK(p.frames_ok >= 3u, "统计 frames_ok");
    CHECK(Vision_Pump(&p, 20u) == 0, "空队列 → 0");
    CHECK(Vision_Pump(NULL, 20u) == 0, "Pump(NULL) 安全");
    Ok("Pump 多帧 / 空队列 / NULL");
}

/* ==========================================================================
 *  main
 * ========================================================================== */
int main(void)
{
    printf("vision_proto host test (visual end v3.3.0 protocol)\n");
    TestFrameVectors();
    TestParser();
    TestReplyParse();
    TestSession();
    TestScript();
    TestBuildFrameEdges();
    TestMsgName();
    TestReplyFields();
    TestMailbox();
    TestSessionStateApi();
    TestTxPath();
    TestPumpMulti();

    printf("\n%d checks, %d failed\n", g_checks, g_fails);
    if (g_fails)
    {
        printf("[FAIL]\n");
        return 1;
    }
    printf("[ALL PASS]\n");
    return 0;
}