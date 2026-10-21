#include "station_job.h"
#include "station_config.h"
#include "arm_control.h"
#include "arm_config.h"
#include "vision_link.h"
#include "road_map.h"
#include <stdio.h>
#include <string.h>

enum { SS_SLIDE, SS_LIFT, SS_YAW, SS_CLAW, SS_PLATFORM_YAW, SS_VISION, SS_LOAD1, SS_CHASSIS, SS_ALIGN, SS_STACK, SS_STACK_RETURN };
typedef struct {
    uint8_t kind;
    int16_t value;
    uint8_t ring; /* 仅SS_CHASSIS：此步完成后对准的环号。 */
    uint8_t platform_slot; /* 仅SS_PLATFORM_YAW：与云台一起旋转的杯位。 */
} StationStep;

enum {
    ST_IDLE = 0,
    ST_RUN,      /* 顺序执行 program[] */
    ST_DONE,
    ST_FAULT
};

#define STN_STEP_MAX 224u
#define STN_STEP_TIMEOUT_MS 40000u
#define STN_VISION_TIMEOUT_MS 25000u

static struct {
    uint8_t phase, job, batch, cursor, count, issued;
    uint32_t step_ms;
    uint32_t align_ms;
    uint8_t align_moving, align_next_axis, stack_moving, stack_axis;
    int32_t stack_offset_mm[2]; /* 0前后，1左右；放完按左右、前后的顺序恢复。 */
    uint32_t align_travel, warning_ms;
    uint8_t warning_seen;
    char reason[192]; /* 中文UTF-8故障原因。 */
} stn;
static StationStep program[STN_STEP_MAX];
static uint8_t task_colors[2][3]; /* 第一批、第二批的颜色，按抓取顺序。 */
static uint8_t task_rings[2][3];  /* 粗加工区目标环号，按车上第 1/2/3 块。 */
static uint8_t task_ready;
static uint8_t automatic = 1;

static int storage_job(void) {return stn.job==3u || stn.job==6u;}
static float station_align_yaw(void) {return storage_job() ? STN_STORAGE_ALIGN_YAW_DEG : STN_ROUGH_YAW_DEG;}

/* 像素→毫米的控制比例由操作者指定，整数四舍五入；保留正负和完整幅值。 */
static int32_t vision_pixel_mm(int16_t pixels)
{
    int32_t magnitude=pixels<0 ? -(int32_t)pixels : pixels;
    magnitude=(magnitude*STN_VISION_MM_PER_PX_X1000+500)/1000;
    return pixels<0 ? -magnitude : magnitude;
}
static int32_t vision_move_mm(int16_t pixels)
{
    return vision_pixel_mm(pixels)*STN_ALIGN_MOVE_SIGN;
}

static void stn_text(const char *s) { arm_output(s); }

static void stn_warning(const char *reason, uint32_t now)
{
    if (stn.warning_seen && (uint32_t)(now-stn.warning_ms)<1000u) return;
    stn.warning_seen=1;stn.warning_ms=now;
    stn_text("[station warning] ");stn_text(reason);stn_text("; task retained\r\n");
}

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
    program[stn.count].ring = program[stn.count].platform_slot = 0;
    stn.count++;
    return 1;
}

static void push_platform_yaw(uint8_t slot,float yaw)
{
    if (push(SS_PLATFORM_YAW,(int)(yaw*10.0f)))
        program[stn.count-1u].platform_slot=slot;
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
/* 车辆移动前始终先升高、收齿条；工位内可保持当前观察角。 */
static void push_retract(void)
{
    push(SS_LIFT, ARM_LIFT_HOME_MM);
    push(SS_SLIDE, ARM_SLIDE_HOME_MM);
}
/* 到目标靶位后才从指定杯位取料；下一杯由作业表显式选择。 */
static void push_take_from_platform(uint8_t slot,int lift_mm)
{
    /* 先在高处转向杯位并张开爪子，再伸齿条、下降；避免闭爪贴着杯壁下降。 */
    push_platform_yaw(slot,ARM_PLACE_YAW_DEG);
    push(SS_SLIDE, ARM_PLACE_SLIDE_MM);
    push(SS_CLAW, (int)(ARM_CLAW_OPEN_DEG * 10.0f));
    push(SS_LIFT, lift_mm);
    push(SS_CLAW, (int)(ARM_CLAW_CLOSE_DEG * 10.0f));
    push(SS_LIFT, ARM_LIFT_HOME_MM);
    push(SS_SLIDE, ARM_SLIDE_HOME_MM);
}
/* 粗加工放回原盘，只抬高收回，不回车头或自动换盘。 */
static void push_place_on_platform(uint8_t slot)
{
    push_platform_yaw(slot,ARM_PLACE_YAW_DEG);
    push(SS_SLIDE, ARM_PLACE_SLIDE_MM);
    push(SS_LIFT, ARM_PLACE_LIFT_MM);
    push(SS_CLAW, (int)(ARM_CLAW_OPEN_DEG * 10.0f));
    push_retract();
}
static void push_release_at(const StationPose *pose)
{
    push_move_to(pose->slide_mm, pose->lift_mm, (int)(pose->yaw_deg * 10.0f));
    push(SS_CLAW, (int)(ARM_CLAW_OPEN_DEG * 10.0f));
    push_retract();
}
static void push_grip_at(const StationPose *pose)
{
    push_move_to(pose->slide_mm, pose->lift_mm, (int)(pose->yaw_deg * 10.0f));
    push(SS_CLAW, (int)(ARM_CLAW_CLOSE_DEG * 10.0f));
    push_retract();
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
        push(SS_LOAD1, i == 2u);
    }
    return 1;
}
static int push_station_index_step(int delta, uint8_t target, uint8_t *ring, int sign, int mm)
{
    if (!push(SS_CHASSIS, delta * sign * mm)) return 0;
    program[stn.count - 1u].ring = target;
    *ring = target;
    return 1;
}

static int push_station_index(uint8_t target, uint8_t *ring, int sign, int mm)
{
    /* 相邻150mm，1↔3直接300mm；途中不在2号停车重取互锁。 */
    if (*ring != target)
        if (!push_station_index_step((int)target - (int)*ring, target, ring, sign, mm)) return 0;
    return 1;
}

static int build_rough(void)
{
    uint8_t i, target, ring = 2u, seen = 0;
    if (!STN_ROUGH_PLACE_READY || !STN_ROUGH_PLATFORM_TAKE_READY || (stn_rough_place.slide_mm == 0
        && stn_rough_place.lift_mm == 0 && stn_rough_place.yaw_deg == 0.0f)) {
        stn_fault("rough-process poses not calibrated (station_config.h)");
        return 0;
    }
    if (arm_platform_slot() != 1) {
        stn_fault("platform must start at slot 1 before unloading");
        return 0;
    }
    /* 编排前验证完整批次，禁止同环重复导致覆盖物块。 */
    for (i = 0; i < 3u; i++) {
        uint8_t target = task_rings[stn.batch][i];
        if (target < 1u || target > 3u || (seen & (1u << (target - 1u)))) {
            stn_fault("rough targets must be a permutation of 1,2,3");
            return 0;
        }
        seen |= (uint8_t)(1u << (target - 1u));
    }
    /* 每次到站只对准中间2号一次；此后按150mm索引完成全部作业，不再逐环请求视觉。 */
    push_retract();
    push(SS_YAW, (int)(STN_ROUGH_YAW_DEG * 10.0f));
    push(SS_ALIGN, 2);
    for (i = 0; i < 3u; i++) {
        target = task_rings[stn.batch][i];
        if (!push_station_index(target, &ring, STN_ROUGH_INDEX_SIGN, STN_ROUGH_INDEX_MM)) return 0;
        push_take_from_platform(i+1u,STN_ROUGH_PLATFORM_TAKE_LIFT_MM);
        push_move_to(stn_rough_place.slide_mm, stn_rough_place.lift_mm, (int)(stn_rough_place.yaw_deg * 10.0f));
        push(SS_CLAW, (int)(ARM_CLAW_OPEN_DEG * 10.0f));
        push_retract(); /* 保持126.5度，直接前后移动到下一环。 */
    }
    /* 按原装车颜色顺序抓回，即G2/G4靶位顺序，恢复原杯位。 */
    for (i = 0; i < 3u; i++) {
        target = task_rings[stn.batch][i];
        if (!push_station_index(target, &ring, STN_ROUGH_INDEX_SIGN, STN_ROUGH_INDEX_MM)) return 0;
        push_grip_at(&stn_rough_grip);
        push_place_on_platform(i+1u);
        push(SS_YAW, (int)(STN_ROUGH_YAW_DEG * 10.0f)); /* 平台操作完回观察角，再移车。 */
    }
    if (!push_station_index(2u, &ring, STN_ROUGH_INDEX_SIGN, STN_ROUGH_INDEX_MM)) return 0;
    push_retract();
    push_platform_yaw(1u,ARM_YAW_HOME_DEG); /* 全部工位动作完成后才回车头0度。 */
    return 1;
}
static int build_storage(void)
{
    uint8_t i, j, ring = 2u, targets[3];
    StationPose pose = stn_storage_place;
    if (!STN_STORAGE_PLACE_READY || !STN_PLATFORM_TAKE_READY) {
        stn_fault("storage poses not calibrated (station_config.h)");
        return 0;
    }
    if (arm_platform_slot() != 1) {
        stn_fault("platform must start at slot 1 before unloading");
        return 0;
    }
    if (stn.batch) {
        if (STN_STORAGE_LAYER_LIFT_MM <= 0) {
            stn_fault("storage layer height not calibrated (STN_STORAGE_LAYER_LIFT_MM)");
            return 0;
        }
        /* 先检查增量再相加，防止错误配置溢出或越过升降行程。 */
        if (STN_STORAGE_LAYER_LIFT_MM > ARM_LIFT_MAX_MM - pose.lift_mm) {
            stn_fault("storage second layer exceeds lift travel");
            return 0;
        }
        pose.lift_mm += STN_STORAGE_LAYER_LIFT_MM;
    }
    for (i = 0; i < 3u; i++) {
        j = i;
        if (stn.batch) {
            for (j = 0; j < 3u; j++)
                if (task_colors[0][j] == task_colors[1][i]) break;
            if (j == 3u) {stn_fault("storage color absent from first batch");return 0;}
        }
        targets[i] = task_rings[0][j]; /* 第二批暂存不用G4，沿用第一批同色的G2位置。 */
        if (targets[i] < 1u || targets[i] > 3u) {
            stn_fault("storage targets must be 1,2,3");return 0;
        }
    }
    push_retract();
    push(SS_YAW, (int)(STN_STORAGE_ALIGN_YAW_DEG * 10.0f));
    if(!stn.batch)push(SS_ALIGN,2); /* 首次对空环用E；第二次直接在目标环位F识别已有物块。 */
    else stn_text("[station stack entry] second storage: skip E; F,color at each target\r\n");
    for (i = 0; i < 3u; i++) {
        if (!push_station_index(targets[i], &ring, STN_STORAGE_INDEX_SIGN, STN_STORAGE_INDEX_MM)) return 0;
        if(stn.batch) {
            push(SS_STACK,task_colors[stn.batch][i]);
            program[stn.count-1u].ring=targets[i];
        }
        push_take_from_platform(i+1u,STN_ROUGH_PLATFORM_TAKE_LIFT_MM);
        push_release_at(&pose);
        push(SS_YAW, (int)(STN_STORAGE_ALIGN_YAW_DEG * 10.0f));
        if(stn.batch)push(SS_STACK_RETURN,0); /* 空爪高位恢复F前基准，避免偏移累入后续150mm索引。 */
    }
    if (!push_station_index(2u, &ring, STN_STORAGE_INDEX_SIGN, STN_STORAGE_INDEX_MM)) return 0;
    push_retract();
    push_platform_yaw(1u,ARM_YAW_HOME_DEG);
    return 1;
}

/* 只按作业表中前面已安排的目标去重，不以进站时人工声明的姿态省略首次动作。
 * 前一步未完成时执行器不会推进；保留每次目标变化及全部视觉/底盘交接。 */
static uint8_t compact_station_program(void)
{
    int16_t last[4]={0,0,0,0};
    uint8_t known=0,slot_known=0,last_slot=0,read,write=0,before=stn.count;
    for(read=0;read<before;read++) {
        StationStep s=program[read];
        if(s.kind<=SS_CLAW) {
            uint8_t bit=(uint8_t)(1u<<s.kind);
            if((known&bit) && last[s.kind]==s.value)continue;
            known|=bit;last[s.kind]=s.value;
        } else if(s.kind==SS_PLATFORM_YAW) {
            uint8_t bit=(uint8_t)(1u<<SS_YAW);
            if((known&bit) && last[SS_YAW]==s.value && slot_known && last_slot==s.platform_slot)continue;
            known|=bit;last[SS_YAW]=s.value;slot_known=1;last_slot=s.platform_slot;
        } else if(s.kind==SS_LOAD1) {
            /* 宏动作内部改变姿态，不能沿用宏动作之前的目标。 */
            known=slot_known=0;
        }
        program[write++]=s;
    }
    stn.count=write;
    return (uint8_t)(before-write);
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
    stn.cursor = stn.count = stn.issued = 0;
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
    if (!task_ready) {
        stn_fault("task code not set; station work needs colors and ring numbers");
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
    /* 各工位已编排自己的离站收尾，不重复回零。 */
    if (stn.phase == ST_FAULT) return -1;
    {
        unsigned i;
        for (i=0;i<stn.count;i++) {
            int value=program[i].value;
            if ((program[i].kind==SS_SLIDE && (value<ARM_SLIDE_MIN_MM || value>ARM_SLIDE_MAX_MM))
                || (program[i].kind==SS_LIFT && (value<ARM_LIFT_MIN_MM || value>ARM_LIFT_MAX_MM))
                || ((program[i].kind==SS_YAW || program[i].kind==SS_PLATFORM_YAW)
                    && (value<ARM_YAW_MIN_DEG*10 || value>ARM_YAW_MAX_DEG*10))
                || (program[i].kind==SS_PLATFORM_YAW && (program[i].platform_slot<1u || program[i].platform_slot>3u))
                || (program[i].kind==SS_CLAW && (value<ARM_CLAW_MIN_DEG*10 || value>ARM_CLAW_MAX_DEG*10))) {
                stn_fault("station pose outside configured travel");return -1;
            }
        }
    }
    if(job==2u || job==3u || job==5u || job==6u) {
        unsigned removed=compact_station_program();
        if(removed) {
            char line[72];
            snprintf(line,sizeof(line),"[station] omitted unchanged arm steps=%u\r\n",removed);
            stn_text(line);
        }
    }
    stn.step_ms = now;
    stn_text("[station] program built; steps=");
    {
        char line[80];
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
    case SS_ALIGN: {
        int slide,lift;float yaw;
        char command=storage_job() ? 'E' : 'D';
        arm_pose_now(&slide,&lift,&yaw,NULL);
        if (!arm_ready() || slide!=ARM_SLIDE_HOME_MM || lift!=ARM_LIFT_HOME_MM || yaw!=station_align_yaw()) {
            stn_fault("vision alignment requires high/retracted arm at station angle");return 0;
        }
        stn.align_ms=now;stn.align_travel=0;
        stn.align_moving=0;stn.align_next_axis=0;
        vision_request_align(command,(uint8_t)s->value,now);
        {
            char message[112];
            snprintf(message,sizeof(message),"[station align] job=%u ring=%u yaw_deg=%.1f; request %c once; wait for MOVE stream / final DISC\r\n",
                (unsigned)stn.job,(unsigned)s->value,(double)station_align_yaw(),command);
            stn_text(message);
        }
        return 1;
    }
    case SS_STACK:
        stn.stack_moving=stn.stack_axis=0;
        stn.stack_offset_mm[0]=stn.stack_offset_mm[1]=0;
        vision_request_stack((uint8_t)s->value,now);
        return 1;
    case SS_STACK_RETURN:
        stn.stack_moving=stn.stack_axis=0;
        return 1;
    case SS_VISION:
        vision_request_pick((uint8_t)s->value, now);
        return 1;
    case SS_LOAD1:
        ok = arm_command(s->value ? "arm load1 last" : "arm load1", now);
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
    case SS_PLATFORM_YAW:
        ok=arm_platform_with_yaw(s->platform_slot,s->value/10.0f,now);
        break;
    case SS_CHASSIS: {
        char index_line[64];
        if (!station_chassis_start(s->value)) {
            stn_warning("index awaiting retracted arm/healthy wheel feedback/IMU",now);
            stn.issued=0;return 0;
        }
        snprintf(index_line, sizeof(index_line), "[station index] ring=%u move_mm=%d\r\n",
                 (unsigned)s->ring, (int)s->value);
        stn_text(index_line);
        return 1;
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
    if (++stn.cursor >= stn.count) {
        stn.phase = ST_DONE;
        stn_text("[station done] all programmed actions finished\r\n");
        return;
    }
    stn.issued = 0;
}

/* D/E两轴独立缓存，顺序移车；最终DISC也等待已接收修正全部完成。 */
static void station_align_update(uint8_t ring, uint32_t now)
{
    int state, distance;
    uint8_t axis;
    int16_t pixels;
    int32_t move_mm;
    if ((uint32_t)(now-stn.align_ms) >= STN_ALIGN_TIMEOUT_MS) {
        stn_warning("alignment deadline exceeded; wait for actual DISC",now);stn.align_ms=now;
    }
    state=vision_align_state();
    if (state==VISION_PICK_FAULT || state==VISION_PICK_TIMEOUT) {
        stn_warning("vision reply unavailable; wait for actual DISC",now);return;
    }
    if (stn.align_moving) {
        if (station_chassis_failed()) {stn_fault("station alignment motion failed");return;}
        if (station_chassis_busy()) return;
        stn.align_moving=0;
    }
    axis=stn.align_next_axis;
    if (!vision_align_peek_axis(axis,&pixels,0)) {
        axis^=1u;
        if (!vision_align_peek_axis(axis,&pixels,0)) {
            if (state==VISION_PICK_READY) {
                char line[80];
                vision_cancel_pick();
                snprintf(line,sizeof(line),"[station aligned] job=%u ring=%u DISC confirmed\r\n",(unsigned)stn.job,(unsigned)ring);
                stn_text(line);step_finished();
            }
            return;
        }
    }
    move_mm=axis ? vision_pixel_mm(pixels)*STN_ALIGN_Y_MOVE_SIGN : vision_move_mm(pixels);
    if (!move_mm) {
        vision_align_take_axis(axis,0,0);stn.align_next_axis=(uint8_t)(axis^1u);return;
    }
    if (!(axis ? station_chassis_lateral_start(move_mm) : station_chassis_start(move_mm))) {
        stn_warning("alignment motion awaiting required pose/feedback",now);return;
    }
    /* 成功交给底盘后才消费；不能因暂缺反馈丢掉本轴或配对的另一轴。 */
    vision_align_take_axis(axis,0,0);stn.align_next_axis=(uint8_t)(axis^1u);
    distance=(int)(move_mm<0 ? -move_mm : move_mm);
    if (stn.align_travel+distance > STN_ALIGN_TRAVEL_MM) {
        stn_warning("alignment travel exceeds nominal budget",now);
    }
    stn.align_travel+=(uint32_t)distance;
    {
        char line[152];
        snprintf(line,sizeof(line),"[station correction] job=%u ring=%u offset_px=%d move_mm=%ld total_mm=%u axis=%s\r\n",
                 (unsigned)stn.job,(unsigned)ring,(int)pixels,(long)move_mm,(unsigned)stn.align_travel,
                 axis ? "lateral" : "forward");
        stn_text(line);
    }
    stn.align_moving=1;
}

/* 每块一次F，顺序完成前后和左右修正；两轴完成后才取放。 */
static void station_stack_update(const StationStep *s,uint32_t now)
{
    int16_t x,y;
    int32_t mm;
    char line[176];
    if(stn.stack_moving) {
        if(station_chassis_failed()){stn_fault("stack alignment motion failed");return;}
        if(station_chassis_busy())return;
        stn.stack_moving=0;stn.stack_axis++;
    }
    if(stn.stack_axis==0) {
        if(vision_stack_state()!=VISION_PICK_READY) {
            if((uint32_t)(now-stn.step_ms)>STN_VISION_TIMEOUT_MS) {
                stn_warning("stack target reply delayed; same F request retained",now);stn.step_ms=now;
            }
            return;
        }
        vision_stack_result(&x,&y);
        stn.stack_offset_mm[0]=vision_move_mm((int16_t)(x-STN_STACK_CENTER_X_PX));
        stn.stack_offset_mm[1]=vision_pixel_mm((int16_t)(y-STN_STACK_CENTER_Y_PX))*STN_STACK_Y_MOVE_SIGN;
        snprintf(line,sizeof(line),"[station stack] job=%u ring=%u color=%u x=%d y=%d move_mm=%ld lateral_mm=%ld place_lift_mm=%d\r\n",
                 (unsigned)stn.job,(unsigned)s->ring,(unsigned)s->value,(int)x,(int)y,
                 (long)stn.stack_offset_mm[0],(long)stn.stack_offset_mm[1],STN_STORAGE_LIFT_MM+STN_STORAGE_LAYER_LIFT_MM);
        stn_text(line);
    }
    if(stn.stack_axis<2) {
        mm=stn.stack_offset_mm[stn.stack_axis];
        if(mm) {
            if(!(stn.stack_axis==0 ? station_chassis_start(mm) : station_chassis_lateral_start(mm))) {
                stn_warning("stack correction awaiting required pose/feedback",now);return;
            }
            stn.stack_moving=1;return;
        }
        stn.stack_axis++;return;
    }
    vision_cancel_pick();step_finished();
}

/* 空爪高位撤销左右、前后修正，避免局部坐标修正累入150mm索引。 */
static void station_stack_return_update(uint32_t now)
{
    unsigned axis;
    int32_t mm;
    char line[80];
    if(stn.stack_moving) {
        if(station_chassis_failed()){stn_fault("stack return motion failed");return;}
        if(station_chassis_busy())return;
        stn.stack_offset_mm[1u-stn.stack_axis]=0;
        stn.stack_moving=0;stn.stack_axis++;
    }
    if(stn.stack_axis>=2){step_finished();return;}
    axis=1u-stn.stack_axis;mm=-stn.stack_offset_mm[axis];
    if(mm) {
        if(!(axis==0 ? station_chassis_start(mm) : station_chassis_lateral_start(mm))) {
            stn_warning("stack return awaiting required pose/feedback",now);return;
        }
        snprintf(line,sizeof(line),"[stack return] move_mm=%ld axis=%s\r\n",(long)mm,axis==0 ? "forward" : "lateral");
        stn_text(line);stn.stack_moving=1;return;
    }
    stn.stack_axis++;
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
    if (s->kind==SS_ALIGN) {station_align_update((uint8_t)s->value,now);return;}
    if (s->kind==SS_STACK) {station_stack_update(s,now);return;}
    if (s->kind==SS_STACK_RETURN) {station_stack_return_update(now);return;}
    if (s->kind == SS_VISION) {
        int state = vision_pick_state();
        if (state == VISION_PICK_READY) {
            stn_text("[station] vision confirmed block at fixed port; start calibrated load1\r\n");
            step_finished();
            return;
        }
        if (state == VISION_PICK_FAULT) {stn_warning("vision request unavailable",now);return;}
        if (state == VISION_PICK_TIMEOUT) {
            stn_warning("vision target still unavailable",now);return;
        }
        if ((uint32_t)(now - stn.step_ms) > STN_VISION_TIMEOUT_MS) {
            stn_warning("vision wait exceeds nominal duration",now);stn.step_ms=now;
        }
        return;
    }
    if (s->kind == SS_CHASSIS) {
        if (station_chassis_failed()) stn_fault("station chassis index failed; vehicle held");
        else if (!station_chassis_busy()) step_finished();
        else if ((uint32_t)(now - stn.step_ms) > STN_STEP_TIMEOUT_MS)
            {stn_warning("index duration exceeds estimate",now);stn.step_ms=now;}
        return;
    }
    if (!arm_busy()) {
        step_finished();
        return;
    }
    if ((uint32_t)(now - stn.step_ms) > STN_STEP_TIMEOUT_MS) {
        stn_warning("arm step duration exceeds estimate",now);stn.step_ms=now;
        return;
    }
}

int station_active(void) { return stn.phase == ST_RUN; }
int station_failed(void) { return stn.phase == ST_FAULT; }
void station_clear_task(void)
{
    task_ready = 0;
    memset(task_colors, 0, sizeof(task_colors));
    memset(task_rings, 0, sizeof(task_rings));
}

void station_abort(void) { if(stn.phase!=ST_FAULT)stn.phase = ST_IDLE; stn.cursor = stn.count = 0;vision_cancel_pick();station_clear_task(); }

void station_init(void)
{
    memset(&stn, 0, sizeof(stn));
    station_clear_task();automatic = 1;
}

static int parse_task(const char *s)
{
    uint8_t digits[12], colors[2][3]={{0}}, rings[2][3]={{1,2,3},{1,2,3}};
    uint8_t n=0, separators=0, i, batch;
    uint8_t color_masks[2]={0,0}, ring_mask;
    while (*s) {
        if (*s==' ' || *s=='\t' || *s=='\r' || *s=='\n') {s++;continue;}
        if (*s=='+') {
            if (n != (uint8_t)((separators+1u)*3u) || separators>=3u) return 0;
            separators++;s++;continue;
        }
        if (*s<'1' || *s>'4' || n>=12u) return 0;
        digits[n++]=(uint8_t)(*s++-'0');
    }
    if (n!=3u && n!=12u) return 0;
    if (separators && (n!=12u || separators!=3u)) return 0;
    for (i=0;i<3u;i++) {
        if (digits[i]>4u || (n==12u && digits[6u+i]>4u)) return 0;
        colors[0][i]=digits[i];
        if (n==12u) {colors[1][i]=digits[6u+i];rings[0][i]=digits[3u+i];rings[1][i]=digits[9u+i];}
    }
    /* 同色叠放要求每批颜色唯一、两批颜色集合相同；靶位只能是123的排列。
     * 三位简码仍用于原料/第一批调试，不能据此执行第二批。 */
    for (batch=0;batch<(n==12u ? 2u : 1u);batch++) {
        ring_mask=0;
        for (i=0;i<3u;i++) {
            uint8_t color=colors[batch][i], ring=rings[batch][i];
            if (color_masks[batch] & (1u<<color)) return 0;
            color_masks[batch] |= (uint8_t)(1u<<color);
            if (ring<1u || ring>3u || (ring_mask & (1u<<ring))) return 0;
            ring_mask |= (uint8_t)(1u<<ring);
        }
    }
    if (n==12u && color_masks[0]!=color_masks[1]) return 0;
    /* 全部字段验证后才提交，坏任务码不能局部覆盖已存任务。 */
    memcpy(task_colors,colors,sizeof(colors));memcpy(task_rings,rings,sizeof(rings));
    task_ready=1;return 1;
}
int station_auto_enabled(void) {return automatic;}
int station_task_ready(void) {return task_ready;}
void station_task_words(int32_t *first,int32_t *second)
{
    unsigned batch,i;int32_t words[2]={0,0};
    if(task_ready)for(batch=0;batch<2;batch++) {
        for(i=0;i<3;i++)words[batch]=words[batch]*10+task_colors[batch][i];
        for(i=0;i<3;i++)words[batch]=words[batch]*10+task_rings[batch][i];
    }
    *first=words[0];*second=words[1];
}

void station_status(void)
{
    char line[160];
    snprintf(line,sizeof(line),"[station] vision_mm_per_px_x1000=%d; operator assumption, not measured\r\n",STN_VISION_MM_PER_PX_X1000);
    stn_text(line);
    snprintf(line, sizeof(line), "[station] storage layer_mm=%d second_lift_mm=%d max_lift_mm=%d; second_batch=%s\r\n",
             STN_STORAGE_LAYER_LIFT_MM, STN_STORAGE_LIFT_MM + STN_STORAGE_LAYER_LIFT_MM, ARM_LIFT_MAX_MM,
             STN_STORAGE_LAYER_LIFT_MM <= 0 ? "needs_height" :
             (STN_STORAGE_LAYER_LIFT_MM > ARM_LIFT_MAX_MM-STN_STORAGE_LIFT_MM ? "exceeds_travel" : "configured"));
    stn_text(line);
    snprintf(line, sizeof(line), "[station] auto=%u phase=%u job=%u batch=%u step=%u/%u task_ready=%u\r\n",
             (unsigned)automatic, (unsigned)stn.phase, (unsigned)stn.job, (unsigned)stn.batch,
             (unsigned)stn.cursor, (unsigned)stn.count, (unsigned)task_ready);
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
        stn_text("[task rejected: need task 123 or four 3-digit groups; distinct colors 1-4, same set both batches, rings permutation 123]\r\n");
        return 1;
    }
    stn_text("[task accepted for one full run, both batches; scan station resumes itself]\r\n");
    station_status();
    return 1;
}
