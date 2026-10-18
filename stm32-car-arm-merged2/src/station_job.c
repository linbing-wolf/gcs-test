#include "station_job.h"
#include "station_config.h"
#include "arm_control.h"
#include "arm_config.h"
#include "vision_link.h"
#include <stdio.h>
#include <string.h>

enum {
    SS_SLIDE, SS_LIFT, SS_YAW, SS_CLAW, SS_PLATFORM,
    SS_VISION, SS_LOAD1, SS_PICK_POSE, SS_CHASSIS
};
typedef struct {
    uint8_t kind;
    int16_t value;
    uint8_t aux;     /* SS_CHASSIS：本次要到达的靶位 1~3 */
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
#define STN_CHASSIS_TIMEOUT_MS 45000u
#define STN_VISION_TIMEOUT_MS 25000u

static struct {
    uint8_t phase, job, batch, cursor, count, issued, retries;
    uint8_t ring;    /* 车当前对准的靶位，进站时是 2 */
    uint32_t step_ms;
    char reason[48];
} stn;
static StationStep program[STN_STEP_MAX];
static uint8_t task_colors[2][3]; /* 第一批、第二批的颜色，按抓取顺序。 */
static uint8_t task_rings[2][3];  /* 靶位号，按车上第 1/2/3 块。 */
static uint8_t task_ready;

static void stn_text(const char *s) { arm_output(s); }

static void stn_fault(const char *reason)
{
    strncpy(stn.reason, reason, sizeof(stn.reason) - 1u);
    stn.reason[sizeof(stn.reason) - 1u] = 0;
    stn.phase = ST_FAULT;
    stn_text("[station fault] ");
    stn_text(stn.reason);
    stn_text("; vehicle held, inspect before retry\r\n");
}

static int push(uint8_t kind, int value)
{
    if (stn.count >= STN_STEP_MAX) return 0;
    program[stn.count].kind = kind;
    program[stn.count].value = (int16_t)value;
    program[stn.count].aux = 0;
    stn.count++;
    return 1;
}

/* READY 置 1 但姿态还是全 0，说明只改了开关没标定；按未标定拒绝。 */
static int pose_configured(const StationPose *pose)
{
    return pose->slide_mm != 0 || pose->lift_mm != 0 || pose->yaw_deg != 0.0f;
}

static void push_move_to(int slide, int lift, float yaw_tenths)
{
    int now_slide, now_lift;
    float now_yaw, now_claw;
    arm_pose_now(&now_slide, &now_lift, &now_yaw, &now_claw);
    /* 顺序固定：先转向，再伸齿条，最后下降，避免扫到台面。 */
    if ((int)(now_yaw * 10.0f + (now_yaw < 0 ? -0.5f : 0.5f)) != (int)yaw_tenths)
        push(SS_YAW, (int)yaw_tenths);
    if (now_slide != slide) push(SS_SLIDE, slide);
    if (now_lift != lift) push(SS_LIFT, lift);
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
/* lift_extra：叠放时抬高的整层高度；第一层传 0。 */
static void push_release_at(const StationPose *pose, int lift_extra)
{
    push_move_to(pose->slide_mm, pose->lift_mm + lift_extra, (int)(pose->yaw_deg * 10.0f));
    push(SS_CLAW, (int)(ARM_CLAW_OPEN_DEG * 10.0f));
    push_retreat_home();
}
static void push_grip_at(const StationPose *pose)
{
    push_move_to(pose->slide_mm, pose->lift_mm, (int)(pose->yaw_deg * 10.0f));
    push(SS_CLAW, (int)(ARM_CLAW_CLOSE_DEG * 10.0f));
    push_retreat_home();
}

/* 工位内底盘索引：把车对准 target 号靶位。
 * sign 是“环号增大方向”相对车头的符号（见 station_config.h）。
 * 1<->3 的 300 mm 必须经 2 号基准拆成两段，每段都是一次独立的
 * 四轮闭环停车，不允许一口气横跨两环。 */
static void push_chassis_step(int16_t mm, uint8_t target_ring)
{
    if (!mm || stn.count >= STN_STEP_MAX) return;
    program[stn.count].kind = SS_CHASSIS;
    program[stn.count].value = mm;
    program[stn.count].aux = target_ring;
    stn.count++;
    stn.ring = target_ring;
}
static void push_chassis_index(uint8_t target, int sign)
{
    if (target < 1u || target > 3u) return;
    if (stn.ring != 2u && stn.ring != target)
        push_chassis_step((int16_t)(((int)2 - (int)stn.ring) * sign * STN_INDEX_MM), 2u);
    if (stn.ring != target)
        push_chassis_step((int16_t)(((int)target - (int)stn.ring) * sign * STN_INDEX_MM), target);
}

static int build_raw(void)
{
    uint8_t i;
    for (i = 0; i < 3u; i++) {
        if (!task_colors[stn.batch][i]) {
            stn_fault("task colors incomplete; send task 123 or task 134123314231");
            return 0;
        }
        /* 云台先转到抓取角并站稳，再让视觉确认物块已转到取料口。 */
        push(SS_PICK_POSE, 0);
        push(SS_VISION, task_colors[stn.batch][i]);
        push(SS_LOAD1, 0);
    }
    return 1;
}
static int build_rough(void)
{
    uint8_t i;
    if (!STN_ROUGH_PLACE_READY || !pose_configured(&stn_rough_place)) {
        stn_fault("rough-process poses not calibrated (station_config.h)");
        return 0;
    }
    if (arm_platform_slot() != 1) {
        stn_fault("platform must start at slot 1 before unloading");
        return 0;
    }
    /* 车上第 1/2/3 块分别放到任务码指定的靶位：先让车索引过去，再取料放下。 */
    for (i = 0; i < 3u; i++) {
        uint8_t ring = task_rings[stn.batch][i];
        if (ring < 1u || ring > 3u) {
            stn_fault("rough target in task code must be 1..3");
            return 0;
        }
        push_chassis_index(ring, STN_ROUGH_INDEX_SIGN);
        push_take_from_platform();
        push_release_at(&stn_rough_place, 0);
    }
    /* 三块全部放完后按车上第 1/2/3 块的顺序抓回，不能按靶位 1→2→3 遍历；
     * 否则载物台的盘位顺序会和任务码颜色顺序错开。 */
    for (i = 0; i < 3u; i++) {
        push_chassis_index(task_rings[stn.batch][i], STN_ROUGH_INDEX_SIGN);
        push_grip_at(&stn_rough_place);
        push_place_on_platform();
    }
    push_chassis_index(2u, STN_ROUGH_INDEX_SIGN);
    return 1;
}
/* 第二批暂存的落点：第一批里同色物块所在的那个暂存位。 */
static uint8_t storage_ring_for_color(uint8_t color)
{
    uint8_t j;
    for (j = 0; j < 3u; j++)
        if (task_colors[0][j] == color) return task_rings[0][j];
    return 0;
}
static int build_storage(void)
{
    uint8_t i;
    uint8_t layers[3] = {0, 0, 0};   /* 每个暂存位已有几层 */
    if (!STN_STORAGE_PLACE_READY || !pose_configured(&stn_storage_place)) {
        stn_fault("storage poses not calibrated (station_config.h)");
        return 0;
    }
    if (arm_platform_slot() != 1) {
        stn_fault("platform must start at slot 1 before unloading");
        return 0;
    }
    /* 第二批要落到第一批上面；层高没标定时同高度释放会撞块，直接拒绝。 */
    if (stn.batch == 1u && STN_STORAGE_LAYER_LIFT_MM <= 0) {
        stn_fault("storage second-layer height not calibrated (STN_STORAGE_LAYER_LIFT_MM)");
        return 0;
    }
    for (i = 0; i < 3u; i++)
        if (task_rings[0][i] >= 1u && task_rings[0][i] <= 3u) layers[task_rings[0][i] - 1u] = 1u;
    for (i = 0; i < 3u; i++) {
        uint8_t ring;
        int lift_extra = 0;
        if (!task_colors[stn.batch][i]) {
            stn_fault("task colors incomplete for storage job");
            return 0;
        }
        if (stn.batch == 0u) {
            ring = task_rings[0][i];
        } else {
            /* 第二批颜色在第一批里找同色位；找不到说明任务码/颜色不对。 */
            ring = storage_ring_for_color(task_colors[1][i]);
            if (ring < 1u || ring > 3u) {
                stn_fault("batch2 color not in batch1; cannot choose storage ring");
                return 0;
            }
            layers[ring - 1u]++;
            lift_extra = (int)(layers[ring - 1u] - 1u) * STN_STORAGE_LAYER_LIFT_MM;
        }
        push_chassis_index(ring, STN_STORAGE_INDEX_SIGN);
        push_take_from_platform();
        push_release_at(&stn_storage_place, lift_extra);
    }
    push_chassis_index(2u, STN_STORAGE_INDEX_SIGN);
    return 1;
}

static uint8_t batch_of_job(uint8_t job)
{
    /* job：0扫码 1原料1 2粗加工1 3暂存1 4原料2 5粗加工2 6暂存2 7启停区。 */
    return job >= 4u && job <= 6u ? 1u : 0u;
}

int station_start(uint8_t node, uint8_t job, uint32_t now)
{
    (void)node;
    if (stn.phase == ST_RUN) return -1;
    stn.phase = ST_RUN;
    stn.job = job;
    stn.batch = batch_of_job(job);
    stn.cursor = stn.count = stn.issued = stn.retries = 0;
    stn.ring = 2u; /* 每个工位都从 2 号基准开始、也在 2 号基准结束 */
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
    if (job >= 1u && job <= 6u && !task_ready) {
        stn_fault("task code not set; raw/rough work needs colors and ring numbers");
        return -1;
    }
    if (!arm_ready()) {
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
    case SS_PICK_POSE:
        ok = arm_command("arm pickpose", now);
        break;
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
            char command[32];
            snprintf(command, sizeof(command), "arm yaw %s", line);
            ok = arm_command(command, now);
        }
        break;
    case SS_CLAW:
        format_tenths(line, sizeof(line), (int)s->value);
        {
            char command[32];
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
    case SS_CHASSIS: {
        char index_line[64];
        if (!station_chassis_start(s->value)) {
            stn_fault("chassis index refused: park arm, stop wheels, need fresh IMU");
            return 0;
        }
        snprintf(index_line, sizeof(index_line), "[station index] ring %u; move %d mm\r\n",
                 (unsigned)s->aux, (int)s->value);
        stn_text(index_line);
        return 1;
    }
    default:
        ok = 0;
        break;
    }
    if (!ok) {
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
        /* 扫码工位没有机构步骤，只等任务码；收到后主程序直接沿当前 -X 航向去中心 E。 */
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
    if (arm_failed()) {
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
    if (s->kind == SS_CHASSIS) {
        /* 工位内索引由主程序按编码器+IMU 四轮闭环执行，完成后车已停稳。 */
        if (station_chassis_failed()) {
            stn_fault("chassis index fault; vehicle held");
            return;
        }
        if (station_chassis_busy()) {
            if ((uint32_t)(now - stn.step_ms) > STN_CHASSIS_TIMEOUT_MS)
                stn_fault("chassis index timeout; vehicle held");
            return;
        }
        step_finished();
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
int station_task_ready(void) { return task_ready; }
uint8_t station_phase(void) { return stn.phase; }
void station_abort(void) { stn.phase = ST_IDLE; stn.cursor = stn.count = 0; }

void station_init(void)
{
    memset(&stn, 0, sizeof(stn));
    memset(task_colors, 0, sizeof(task_colors));
    task_ready = 0;
}

/* 靶位必须是 1/2/3 的一个排列：三项都在 1~3 且互不重复。 */
static int rings_valid(const uint8_t *rings)
{
    uint8_t seen = 0, i;
    for (i = 0; i < 3u; i++) {
        uint8_t bit;
        if (rings[i] < 1u || rings[i] > 3u) return 0;
        bit = (uint8_t)(1u << (rings[i] - 1u));
        if (seen & bit) return 0;
        seen |= bit;
    }
    return 1;
}

static int parse_task(const char *s)
{
    uint8_t digits[12], n = 0, groups = 0, position = 0, separators = 0, i;
    uint8_t colors[2][3];
    uint8_t rings[2][3];
    while (*s) {
        if (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') {
            s++;
            continue;
        }
        if (*s == '+') {
            if (position != 3u || groups >= 3u || n >= 12u) return 0;
            groups++;
            position = 0;
            separators++;
            s++;
            continue;
        }
        if (*s < '1' || *s > '4' || n >= 12u) return 0;
        /* 没有加号时每三位自动分组；有加号时保留 position=3 给 '+' 分支判断。
         * 旧代码在读到第三位后就清零，导致后面的 '+' 永远被判成位置错误。 */
        if (separators == 0u && position == 3u) position = 0;
        if (position >= 3u) return 0;
        digits[n++] = (uint8_t)(*s - '0');
        position++;
        s++;
    }
    if (n == 3u && separators == 0u) {
        /* 台架测试快捷码 task 123：只给第一批三个颜色，靶位固定 1/2/3。 */
        for (i = 0; i < 3u; i++)
            if (digits[i] < 1u || digits[i] > 4u) return 0;
        for (i = 0; i < 3u; i++) {
            colors[0][i] = digits[i];
            colors[1][i] = 0;
            rings[0][i] = (uint8_t)(i + 1u);
            rings[1][i] = (uint8_t)(i + 1u);
        }
    } else {
        /* 比赛任务码是四组三位数：G1 第一批颜色，G2 第一批靶位，
         * G3 第二批颜色，G4 第二批靶位。允许加号或直接连写。 */
        if (n != 12u || (separators != 0u && separators != 3u)) return 0;
        for (i = 0; i < 3u; i++) {
            colors[0][i] = digits[i];
            colors[1][i] = digits[6 + i];
            rings[0][i] = digits[3 + i];
            rings[1][i] = digits[9 + i];
        }
        /* 颜色 1红 2黄 3蓝 4绿（扫描时已限制 1~4）；靶位 1~3 且不重复。 */
        if (!rings_valid(rings[0]) || !rings_valid(rings[1])) return 0;
    }
    /* 全部校验通过后才原子提交，坏码不会留下半套新旧混合数据。 */
    memcpy(task_colors, colors, sizeof(task_colors));
    memcpy(task_rings, rings, sizeof(task_rings));
    task_ready = 1;
    return 1;
}

void station_status(void)
{
    char line[200];
    uint8_t i;
    snprintf(line, sizeof(line), "[station] phase=%u job=%u batch=%u step=%u/%u ring=%u\r\n",
             (unsigned)stn.phase, (unsigned)stn.job, (unsigned)stn.batch,
             (unsigned)stn.cursor, (unsigned)stn.count, (unsigned)stn.ring);
    stn_text(line);
    snprintf(line, sizeof(line), "[station] batch1 colors=%u%u%u rings=%u%u%u\r\n",
             (unsigned)task_colors[0][0], (unsigned)task_colors[0][1], (unsigned)task_colors[0][2],
             (unsigned)task_rings[0][0], (unsigned)task_rings[0][1], (unsigned)task_rings[0][2]);
    stn_text(line);
    snprintf(line, sizeof(line), "[station] batch2 colors=%u%u%u rings=%u%u%u\r\n",
             (unsigned)task_colors[1][0], (unsigned)task_colors[1][1], (unsigned)task_colors[1][2],
             (unsigned)task_rings[1][0], (unsigned)task_rings[1][1], (unsigned)task_rings[1][2]);
    stn_text(line);
    /* 实际落位表：盘位→颜色→粗加工靶位/暂存位，方便现场逐块核对。 */
    for (i = 0; i < 3u; i++) {
        if (!task_colors[0][i]) continue;
        snprintf(line, sizeof(line), "[task] b1 slot%u color=%u rough=%u storage=%u\r\n",
                 (unsigned)(i + 1u), (unsigned)task_colors[0][i],
                 (unsigned)task_rings[0][i], (unsigned)task_rings[0][i]);
        stn_text(line);
    }
    for (i = 0; i < 3u; i++) {
        uint8_t store;
        if (!task_colors[1][i]) continue;
        store = storage_ring_for_color(task_colors[1][i]);
        snprintf(line, sizeof(line), "[task] b2 slot%u color=%u rough=%u storage=%u layer2\r\n",
                 (unsigned)(i + 1u), (unsigned)task_colors[1][i],
                 (unsigned)task_rings[1][i], (unsigned)store);
        stn_text(line);
    }
}

int station_command(const char *s)
{
    const char *task;
    if (!strcmp(s, "station")) { station_status(); return 1; }
    if (!strncmp(s, "task ", 5)) {
        task = s + 5;
    } else if (s[0] >= '1' && s[0] <= '4') {
        /* 串口手输允许省略 task 前缀；扫码仍会拼成 task <内容>。 */
        task = s;
    } else return 0;
    if (!parse_task(task)) {
        stn_text("[task rejected: need four 3-digit groups e.g. 134+123+314+231;"
                 " colors 1-4, targets must be a permutation of 1-3; previous task retained]\r\n");
        return 1;
    }
    stn_text("[task accepted; stored only; run starts with runimu A/C, scan station resumes itself]\r\n");
    station_status();
    return 1;
}
