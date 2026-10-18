#include "station_job.h"
#include "station_config.h"
#include "arm_control.h"
#include "arm_config.h"
#include "vision_link.h"
#include "road_map.h"
#include <stdio.h>
#include <string.h>

enum { SS_SLIDE, SS_LIFT, SS_YAW, SS_CLAW, SS_PLATFORM, SS_VISION, SS_LOAD1 };
typedef struct {
    uint8_t kind;
    int16_t value;
} StationStep;

enum {
    ST_IDLE = 0,
    ST_RUN,      /* 顺序执行 program[] */
    ST_DONE,
    ST_FAULT
};

#define STN_STEP_MAX 128u
#define STN_RETRY_MAX 2u
#define STN_STEP_TIMEOUT_MS 40000u
#define STN_VISION_TIMEOUT_MS 25000u

static struct {
    uint8_t phase, job, batch, cursor, count, issued, retries;
    uint32_t step_ms;
    char reason[48];
} stn;
static StationStep program[STN_STEP_MAX];
static uint8_t task_colors[2][3]; /* 第一批、第二批的颜色，按抓取顺序。 */
static uint8_t task_rings[2][3];  /* 粗加工区目标环号，按车上第 1/2/3 块。 */
static uint8_t task_ready;
static uint8_t automatic = 1;

static void stn_text(const char *s) { arm_output(s); }

static void stn_fault(const char *reason)
{
    strncpy(stn.reason, reason, sizeof(stn.reason) - 1u);
    stn.reason[sizeof(stn.reason) - 1u] = 0;
    stn.phase = ST_FAULT;
    arm_stop();vision_cancel_pick();
    stn_text("[station fault] ");
    stn_text(stn.reason);
    stn_text("; vehicle held, inspect before retry\r\n");
}

static int push(uint8_t kind, int value)
{
    if (stn.count >= STN_STEP_MAX) {stn_fault("station program too large");return 0;}
    program[stn.count].kind = kind;
    program[stn.count].value = (int16_t)value;
    stn.count++;
    return 1;
}

/* 编排时的实际姿态只代表第一步，不能据此省掉后续步骤的目标。 */
static void push_move_to(int slide, int lift, float yaw_tenths)
{
    push(SS_YAW,(int)yaw_tenths);
    push(SS_SLIDE,slide);
    push(SS_LIFT,lift);
}
/* 把“十分之一度”的整数格式化成可再被串口解析的角度。
 * 不能直接用 value/10：负数会被截断成 0 并丢掉符号。 */
static void format_tenths(char *line, size_t size, int value)
{
    int magnitude = value < 0 ? -value : value;
    snprintf(line, size, "%s%d.%d", value < 0 ? "-" : "",
             magnitude / 10, magnitude % 10);
}
/* 抬手 -> 收齿条 -> 回云台 0°，这是最安全的离开工位的姿态。 */
static void push_retreat_home(void)
{
    push(SS_LIFT, ARM_LIFT_HOME_MM);
    push(SS_SLIDE, ARM_SLIDE_HOME_MM);
    push(SS_YAW, (int)(ARM_YAW_HOME_DEG * 10.0f));
}
/* 从车载载物台当前杯位把物块夹起来，再让载物台转到下一个杯位。 */
static void push_take_from_platform(void)
{
    /* 先在高处转向杯位并张开爪子，再伸齿条、下降；避免闭爪贴着杯壁下降。 */
    push(SS_YAW, (int)(STN_PLATFORM_TAKE_YAW * 10.0f));
    push(SS_SLIDE, STN_PLATFORM_TAKE_SLIDE);
    push(SS_CLAW, (int)(ARM_CLAW_OPEN_DEG * 10.0f));
    push(SS_LIFT, STN_PLATFORM_TAKE_LIFT);
    push(SS_CLAW, (int)(ARM_CLAW_CLOSE_DEG * 10.0f));
    push(SS_LIFT, ARM_LIFT_HOME_MM);
    push(SS_SLIDE, ARM_SLIDE_HOME_MM);
    push(SS_YAW, (int)(ARM_YAW_HOME_DEG * 10.0f));
    push(SS_PLATFORM, 0); /* 0 = 自动转到下一个杯位 */
}
static void push_place_on_platform(void)
{
    push_move_to(ARM_PLACE_SLIDE_MM, ARM_PLACE_LIFT_MM, (int)(ARM_PLACE_YAW_DEG * 10.0f));
    push(SS_CLAW, (int)(ARM_CLAW_OPEN_DEG * 10.0f));
    push_retreat_home();
    push(SS_PLATFORM, 0);
}
static void push_release_at(const StationPose *pose)
{
    push_move_to(pose->slide_mm, pose->lift_mm, (int)(pose->yaw_deg * 10.0f));
    push(SS_CLAW, (int)(ARM_CLAW_OPEN_DEG * 10.0f));
    push_retreat_home();
}
static void push_grip_at(const StationPose *pose)
{
    push_move_to(pose->slide_mm, pose->lift_mm, (int)(pose->yaw_deg * 10.0f));
    push(SS_CLAW, (int)(ARM_CLAW_CLOSE_DEG * 10.0f));
    push_retreat_home();
}

static int build_raw(void)
{
    uint8_t i;
    int slide, lift;
    float claw;
    arm_pose_now(&slide, &lift, NULL, &claw);
    /* 仅首块预定位可以按此刻已声明的姿态省略无变化目标；
     * 不能用这份起始姿态省略后续load1或离站收拢中的动作。 */
    if (lift != ARM_LIFT_HOME_MM) push(SS_LIFT, ARM_LIFT_HOME_MM);
    if (slide != ARM_SLIDE_HOME_MM) push(SS_SLIDE, ARM_SLIDE_HOME_MM);
    /* 云台步骤始终保留双轴新反馈预检及完整插值，完成后才允许视觉请求。
     * 后续load1结束也会等云台回抓取角、载物台换位完成，才请求下一块。 */
    push(SS_YAW, (int)(ARM_PICK_YAW_DEG * 10.0f));
    if (claw != ARM_CLAW_OPEN_DEG) push(SS_CLAW, (int)(ARM_CLAW_OPEN_DEG * 10.0f));
    for (i = 0; i < 3u; i++) {
        if (!task_colors[stn.batch][i]) {
            stn_fault("task colors incomplete; send task 123 or task 134123314231");
            return 0;
        }
        push(SS_VISION, task_colors[stn.batch][i]);
        push(SS_LOAD1, 0);
    }
    return 1;
}
static int build_rough(void)
{
    uint8_t i;
    if (!STN_ROUGH_PLACE_READY || !STN_PLATFORM_TAKE_READY) {
        stn_fault("rough-process poses not calibrated (station_config.h)");
        return 0;
    }
    if (arm_platform_slot() != 1) {
        stn_fault("platform must start at slot 1 before unloading");
        return 0;
    }
    /* 车上第 1/2/3 块分别放到任务码指定的环位。 */
    for (i = 0; i < 3u; i++) {
        uint8_t ring = task_rings[stn.batch][i];
        if (ring < 1u || ring > 5u) {
            stn_fault("rough ring in task code must be 1..5");
            return 0;
        }
        push_take_from_platform();
        push_release_at(&stn_rough_place[ring - 1u]);
    }
    /* 三块全部放完后立即按环位 1、2、3 抓回车上载物台。 */
    for (i = 0; i < 3u; i++) {
        push_grip_at(&stn_rough_place[i]);
        push_place_on_platform();
    }
    return 1;
}
static int build_storage(void)
{
    uint8_t i;
    if (!STN_STORAGE_PLACE_READY || !STN_PLATFORM_TAKE_READY) {
        stn_fault("storage poses not calibrated (station_config.h)");
        return 0;
    }
    if (arm_platform_slot() != 1) {
        stn_fault("platform must start at slot 1 before unloading");
        return 0;
    }
    for (i = 0; i < 3u; i++) {
        push_take_from_platform();
        push_release_at(&stn_storage_place[i]);
    }
    return 1;
}

static uint8_t batch_of_job(uint8_t job)
{
    /* job：0扫码 1原料1 2粗加工1 3暂存1 4原料2 5粗加工2 6暂存2 7启停区。 */
    return job >= 4u && job <= 6u ? 1u : 0u;
}

int station_start(uint8_t node, uint8_t job, uint32_t now)
{
    static const uint8_t nodes[8] = {NODE_B,NODE_F,NODE_D,NODE_H,NODE_F,NODE_D,NODE_H,255};
    if (stn.phase == ST_RUN) return -1;
    if (job > 7u || (job < 7u && node != nodes[job])) {stn_fault("station node/job mismatch");return -1;}
    stn.phase = ST_RUN;
    stn.job = job;
    stn.batch = batch_of_job(job);
    stn.cursor = stn.count = stn.issued = stn.retries = 0;
    stn.reason[0] = 0;
    if (job == 7u) { /* 启停区没有机构动作。 */
        stn.phase = ST_DONE;
        return 0;
    }
    if (job == 0u) {
        /* B 点扫码：车已转向二维码，机构不动，原地等扫码模块或串口送任务码。
         * 不能因为还没扫到就 fault，否则车停在 B 点不动。 */
        stn.count = 0;
        stn.step_ms = now;
        if (task_ready) {
            stn.phase = ST_DONE;
            stn_text("[scan station] task already set; no mechanism action\r\n");
            return 1;
        }
        stn_text("[scan station] waiting for task code; scan QR at B or send task 134+123+314+231\r\n");
        return 1;
    }
    if ((job == 1u || job == 2u || job == 4u || job == 5u) && !task_ready) {
        stn_fault("task code not set; raw/rough work needs colors and ring numbers");
        return -1;
    }
    if (!arm_ready() || !arm_platform_slot()) {
        stn_fault("arm on + arm pose + arm platform pose 20 must be done before run");
        return -1;
    }
    if (job == 1u || job == 4u) {
        if (!build_raw()) return -1;
    } else if (job == 2u || job == 5u) {
        if (!build_rough()) return -1;
    } else if (job == 3u || job == 6u) {
        if (!build_storage()) return -1;
    } else {
        stn_fault("unknown station job");
        return -1;
    }
    push_retreat_home();
    if (stn.phase == ST_FAULT) return -1;
    {
        unsigned i;
        for (i=0;i<stn.count;i++) {
            int value=program[i].value;
            if ((program[i].kind==SS_SLIDE && (value<ARM_SLIDE_MIN_MM || value>ARM_SLIDE_MAX_MM))
                || (program[i].kind==SS_LIFT && (value<ARM_LIFT_MIN_MM || value>ARM_LIFT_MAX_MM))
                || (program[i].kind==SS_YAW && (value<ARM_YAW_MIN_DEG*10 || value>ARM_YAW_MAX_DEG*10))
                || (program[i].kind==SS_CLAW && (value<ARM_CLAW_MIN_DEG*10 || value>ARM_CLAW_MAX_DEG*10))) {
                stn_fault("station pose outside configured travel");return -1;
            }
        }
    }
    stn.step_ms = now;
    stn_text("[station] program built; steps=");
    {
        char line[40];
        snprintf(line, sizeof(line), "%u; waiting for vision/arm\r\n", (unsigned)stn.count);
        stn_text(line);
    }
    return 1;
}

static int issue_step(uint32_t now)
{
    const StationStep *s = &program[stn.cursor];
    char line[40];
    int ok = 1;
    stn.step_ms = now;
    line[0] = 0;
    switch (s->kind) {
    case SS_VISION:
        vision_request_pick((uint8_t)s->value, now);
        return 1;
    case SS_LOAD1:
        ok = arm_command("arm load1", now);
        break;
    case SS_SLIDE:
        snprintf(line, sizeof(line), "arm slide %d", (int)s->value);
        ok = arm_command(line, now);
        break;
    case SS_LIFT:
        snprintf(line, sizeof(line), "arm lift %d", (int)s->value);
        ok = arm_command(line, now);
        break;
    case SS_YAW:
        format_tenths(line, sizeof(line), (int)s->value);
        {
            char command[64];
            snprintf(command, sizeof(command), "arm yaw %s", line);
            ok = arm_command(command, now);
        }
        break;
    case SS_CLAW:
        format_tenths(line, sizeof(line), (int)s->value);
        {
            char command[64];
            snprintf(command, sizeof(command), "claw angle %s", line);
            ok = arm_command(command, now);
        }
        break;
    case SS_PLATFORM: {
        int slot = arm_platform_slot() % 3 + 1;
        snprintf(line, sizeof(line), "arm platform %d", slot);
        ok = arm_command(line, now);
        break;
    }
    default:
        ok = 0;
        break;
    }
    /* arm_command返回已处理，并不保证接受动作；每个工位动作都应进入busy。 */
    if (!ok || !arm_busy() || arm_failed()) {
        stn_fault("arm rejected a station step; check pose/platform declaration");
        return 0;
    }
    return 1;
}

static void step_finished(void)
{
    stn.retries = 0;
    if (++stn.cursor >= stn.count) {
        stn.phase = ST_DONE;
        stn_text("[station done] all programmed actions finished\r\n");
        return;
    }
    stn.issued = 0;
}

void station_update(uint32_t now)
{
    const StationStep *s;
    if (stn.phase != ST_RUN) return;
    if (stn.count == 0u) {
        /* 扫码工位没有机构步骤；收到任务码后重新采四轮反馈并衔接下一道路。 */
        if (!task_ready) {
            if ((uint32_t)(now - stn.step_ms) >= 5000u) {
                stn.step_ms = now;
                stn_text("[scan station] still waiting for task code\r\n");
            }
            return;
        }
        stn.phase = ST_DONE;
        stn_text("[scan station] task accepted; no mechanism action\r\n");
        return;
    }
    if (arm_failed() || !arm_enabled() || !arm_pose_known()) {
        stn_fault("arm reported a fault during station work");
        return;
    }
    s = &program[stn.cursor];
    if (!stn.issued) {
        stn.issued = 1;
        if (!issue_step(now)) return;
        if (s->kind != SS_VISION && !arm_busy() && !arm_failed()) {
            /* 无变化或瞬时完成的单轴命令也要等一轮，避免跳过预检。 */
        }
        return;
    }
    if (s->kind == SS_VISION) {
        int state = vision_pick_state();
        if (state == VISION_PICK_READY) {
            stn_text("[station] vision confirmed block at fixed port; start calibrated load1\r\n");
            step_finished();
            return;
        }
        if (state == VISION_PICK_FAULT) {stn_fault("vision UART/frame transport fault");return;}
        if (state == VISION_PICK_TIMEOUT) {
            if (stn.retries < STN_RETRY_MAX) {
                stn.retries++;
                stn_text("[station] vision timeout; request the same color again\r\n");
                stn.issued = 0;
                return;
            }
            stn_fault("vision never reported the requested color at the fixed port");
            return;
        }
        if ((uint32_t)(now - stn.step_ms) > STN_VISION_TIMEOUT_MS) {
            stn_fault("vision wait exceeded guard time");
        }
        return;
    }
    if (!arm_busy()) {
        step_finished();
        return;
    }
    if ((uint32_t)(now - stn.step_ms) > STN_STEP_TIMEOUT_MS) {
        stn_fault("arm step timeout");
        return;
    }
}

int station_active(void) { return stn.phase == ST_RUN; }
int station_failed(void) { return stn.phase == ST_FAULT; }
void station_abort(void) { if(stn.phase!=ST_FAULT)stn.phase = ST_IDLE; stn.cursor = stn.count = 0;vision_cancel_pick(); }

void station_init(void)
{
    memset(&stn, 0, sizeof(stn));
    memset(task_colors, 0, sizeof(task_colors));
    memset(task_rings, 0, sizeof(task_rings));
    task_ready = 0;automatic = 1;
}

static int parse_task(const char *s)
{
    uint8_t digits[12], colors[2][3]={{0}}, rings[2][3]={{1,2,3},{1,2,3}};
    uint8_t n=0, separators=0, i;
    while (*s) {
        if (*s==' ' || *s=='\t' || *s=='\r' || *s=='\n') {s++;continue;}
        if (*s=='+') {
            if (n != (uint8_t)((separators+1u)*3u) || separators>=3u) return 0;
            separators++;s++;continue;
        }
        if (*s<'1' || *s>'5' || n>=12u) return 0;
        digits[n++]=(uint8_t)(*s++-'0');
    }
    if (n!=3u && n!=12u) return 0;
    if (separators && (n!=12u || separators!=3u)) return 0;
    for (i=0;i<3u;i++) {
        if (digits[i]>4u || (n==12u && digits[6u+i]>4u)) return 0;
        colors[0][i]=digits[i];
        if (n==12u) {colors[1][i]=digits[6u+i];rings[0][i]=digits[3u+i];rings[1][i]=digits[9u+i];}
    }
    /* 全部字段验证后才提交，坏任务码不能局部覆盖已存任务。 */
    memcpy(task_colors,colors,sizeof(colors));memcpy(task_rings,rings,sizeof(rings));
    task_ready=1;return 1;
}
int station_auto_enabled(void) {return automatic;}

void station_status(void)
{
    char line[160];
    snprintf(line, sizeof(line), "[station] auto=%u phase=%u job=%u batch=%u step=%u/%u\r\n",
             (unsigned)automatic, (unsigned)stn.phase, (unsigned)stn.job, (unsigned)stn.batch,
             (unsigned)stn.cursor, (unsigned)stn.count);
    stn_text(line);
    snprintf(line, sizeof(line), "[station] batch1 colors=%u%u%u rings=%u%u%u\r\n",
             (unsigned)task_colors[0][0], (unsigned)task_colors[0][1], (unsigned)task_colors[0][2],
             (unsigned)task_rings[0][0], (unsigned)task_rings[0][1], (unsigned)task_rings[0][2]);
    stn_text(line);
    snprintf(line, sizeof(line), "[station] batch2 colors=%u%u%u rings=%u%u%u\r\n",
             (unsigned)task_colors[1][0], (unsigned)task_colors[1][1], (unsigned)task_colors[1][2],
             (unsigned)task_rings[1][0], (unsigned)task_rings[1][1], (unsigned)task_rings[1][2]);
    stn_text(line);
}

int station_command(const char *s)
{
    const char *task;
    if (!strcmp(s, "station")) { station_status(); return 1; }
    if (!strcmp(s,"station auto on") || !strcmp(s,"station auto off")) {
        if (station_active()) {stn_text("[station rejected: active]\r\n");return 1;}
        automatic=(uint8_t)!strcmp(s,"station auto on");
        stn_text(automatic ? "[station auto enabled]\r\n" : "[station auto disabled: route only]\r\n");return 1;
    }
    if (!strncmp(s, "task ", 5)) {
        task = s + 5;
    } else if (s[0] >= '1' && s[0] <= '5') {
        /* 串口手输允许省略 task 前缀；扫码仍会拼成 task <内容>。 */
        task = s;
    } else return 0;
    if (station_active() && stn.count) {stn_text("[task rejected: station work active]\r\n");return 1;}
    if (!parse_task(task)) {
        stn_text("[task rejected: need task 123 or four 3-digit groups; colors 1-4, rings 1-5]\r\n");
        return 1;
    }
    stn_text("[task accepted; stored only; run starts with runimu A/C, scan station resumes itself]\r\n");
    station_status();
    return 1;
}
