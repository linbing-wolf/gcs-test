/* 比赛启用避障和独立诊断共用；复用实际道路动作和转向选择。 */
typedef struct {
    uint8_t job,root,depth,goal;
    uint16_t blocked;
} ObstacleRouteSearch;

enum { OBSTACLE_ANY_START=9u };

static uint8_t obstacle_job_goal(uint8_t job)
{
    return job==FULL_ROUTE_JOBS-1u ? OBSTACLE_ANY_START : full_route_jobs[run.plan][job];
}

static uint32_t obstacle_turn_cost_ms(RoadHeading a,RoadHeading b)
{
    return a==b ? 0u : OBSTACLE_TURN_COST_MS;
}

static int obstacle_task_heading(uint8_t node)
{
    if(node==NODE_B)return HEAD_X_NEG;
    if(node==NODE_F)return CAR_STATION_RAW_HEADING;
    if(node==NODE_D)return CAR_STATION_ROUGH_HEADING;
    if(node==NODE_H)return CAR_STATION_STORAGE_HEADING;
    return -1;
}

/* 同一参数组的剖面反复用于候选路径。缓存仅节省估算运算，不改变控制器。 */
static struct {
    uint16_t mm;
    uint8_t profile;
    uint32_t ms;
} obstacle_motion_cache[96];
static uint8_t obstacle_motion_cache_next;

static uint32_t obstacle_motion_time_ms(BodyAxis axis,int sign,uint16_t mm)
{
    unsigned i;
    uint8_t key=axis==BODY_LEFT ? (sign<0 ? 3u : 2u) : sign<0 ? 1u : 0u;
    uint32_t elapsed=0,step=CAR_VELOCITY_CONTROL_MS;
    uint32_t decel=axis==BODY_LEFT ? CAR_LATERAL_DECEL_RPM_PER_S : CAR_DRIVE_DECEL_RPM_PER_S;
    uint32_t lookahead=axis==BODY_LEFT ? CAR_LATERAL_BRAKE_LOOKAHEAD_MS : CAR_DRIVE_BRAKE_LOOKAHEAD_MS;
    int32_t profile=0;
    float remaining=(float)mm;
    float ppm=axis==BODY_LEFT ? car_road_ppm_x(sign) : car_road_ppm_y(sign);
    float mm_per_rpm_s=CAR_POSITION_PULSES_PER_REV/(60.0f*ppm);
    float delay_rpm=decel*(lookahead/1000.0f);
    uint16_t limit=axis==BODY_LEFT ? car_lateral_limit_rpm(mm) : CAR_RUN_RPM;
    uint32_t accel=axis==BODY_LEFT ? CAR_LATERAL_ACCEL_RPM_PER_S : CAR_ROUTE_ACCEL_RPM_PER_S;
    int32_t minimum=axis==BODY_LEFT ? CAR_LATERAL_SETTLE_MIN_RPM : CAR_DISTANCE_MIN_RPM;
    for(i=0;i<96u;i++)if(obstacle_motion_cache[i].mm==mm && obstacle_motion_cache[i].profile==key)
        return obstacle_motion_cache[i].ms;
    /* 按实际控制周期积分名义速度包络：位置P、制动预留、加减速和末段最低RPM。
     * 假定反馈及时且无打滑；CAN/IMU交接仍用独立待标定费用，不冒充实测耗时。 */
    while(remaining>CAR_DISTANCE_TOLERANCE_MM) {
        int32_t desired=(int32_t)remaining*CAR_DISTANCE_KP_X100/100;
        float brake=sqrtf(delay_rpm*delay_rpm+2.0f*decel
            *(remaining-CAR_DISTANCE_TOLERANCE_MM)/mm_per_rpm_s)-delay_rpm;
        int32_t target,change;
        if(desired>(int32_t)brake)desired=(int32_t)brake;
        if(desired<minimum)desired=minimum;
        if(desired>limit)desired=limit;
        if(mm-remaining<CAR_ENCODER_LEARN_MM && desired>CAR_ENCODER_LEARN_RPM)desired=CAR_ENCODER_LEARN_RPM;
        target=desired*1000;
        change=(int32_t)((target>profile ? accel : decel)*step);
        if(target>profile){profile+=change;if(profile>target)profile=target;}
        else {profile-=change;if(profile<target)profile=target;}
        remaining-=(profile/1000)*mm_per_rpm_s*(step/1000.0f);
        elapsed+=step;
    }
    elapsed+=(uint32_t)((profile+decel-1u)/decel);
    i=obstacle_motion_cache_next;
    obstacle_motion_cache[i].mm=mm;obstacle_motion_cache[i].profile=key;obstacle_motion_cache[i].ms=elapsed;
    obstacle_motion_cache_next=(uint8_t)((i+1u)%96u);
    return elapsed;
}

static void obstacle_cost_adjust_road(ObstacleRouteSearch *search,RoadPlan *road)
{
    if(obstacle_start_node(road->from)) {
        uint8_t axes=OBSTACLE_ENTRY_BOTH;
        if(!search->depth && search->job==run.job && road->from==search->root)
            axes=road->from==obstacle_inset_node ? obstacle_inset_axes : 0u;
        obstacle_adjust_inset(road,road->from,axes,-1);
    }
    if(obstacle_start_node(road->to)
        && !(search->job==FULL_ROUTE_JOBS-1u && road->to==search->goal))
        obstacle_adjust_inset(road,road->to,OBSTACLE_ENTRY_BOTH,1);
    obstacle_adjust_corner(road);
#ifndef CHASSIS_OBSTACLE_TEST
    if(!search->depth && search->job==run.job)run_apply_departure_distance(road,1);
#endif
}

static uint32_t obstacle_action_cost(RunLeg leg,uint8_t entry_wait)
{
    return obstacle_motion_time_ms(leg.axis,leg.sign,leg.mm)+OBSTACLE_ACTION_COST_MS
        +CAR_ROUTE_NODE_STOP_MS
        +(entry_wait ? OBSTACLE_ENTRY_COST_MS : 0u);
}

/* 简单路径枚举；每个剩余任务阶段使用独立的固定工作区。 */
typedef struct {
    uint8_t nodes[9],next[9],depth,yielded,done;
    uint16_t blocked;
} ObstaclePathWalk;

static void obstacle_walk_start(ObstaclePathWalk *walk,uint8_t from,uint16_t blocked)
{
    memset(walk,0,sizeof(*walk));walk->nodes[0]=from;walk->blocked=blocked;
}

static int obstacle_walk_next(ObstaclePathWalk *walk,uint8_t goal)
{
    if(walk->done)return 0;
    if(walk->yielded) {
        walk->yielded=0;
        if(!walk->depth){walk->done=1;return 0;}
        --walk->depth;
    }
    for(;;) {
        uint8_t node=walk->nodes[walk->depth],i,j,found=0;
        if(node==goal || (goal==OBSTACLE_ANY_START && obstacle_start_node(node))) {
            walk->yielded=1;return 1;
        }
        for(i=walk->next[walk->depth];i<9u;i++) {
            RoadPlan road;
            walk->next[walk->depth]=(uint8_t)(i+1u);
            for(j=0;j<=walk->depth;j++)if(walk->nodes[j]==i)break;
            if(j<=walk->depth || !road_plan(node,i,&road) || (walk->blocked&(1u<<road.road)))continue;
            ++walk->depth;walk->nodes[walk->depth]=i;walk->next[walk->depth]=0;found=1;break;
        }
        if(found)continue;
        if(!walk->depth){walk->done=1;return 0;}
        --walk->depth;
    }
}

static uint32_t obstacle_path_cost(ObstaclePathWalk *walk,uint8_t job,uint8_t root,
                                  RoadHeading *heading,uint16_t *passed,uint16_t *distance)
{
    ObstacleRouteSearch context;
    RunLeg action;
    uint8_t i,j,active=0,entry_wait=0;
    uint32_t cost=0;
    int arrival;
    memset(&context,0,sizeof(context));context.job=job;context.root=root;context.blocked=walk->blocked;*distance=0;
    context.goal=walk->nodes[walk->depth];
    memset(&action,0,sizeof(action));
    for(i=1;i<=walk->depth;i++) {
        RoadPlan road;
        int scan_blend;
        (void)road_plan(walk->nodes[i-1u],walk->nodes[i],&road);
        if(run_uses_scan_waypoint(job,(uint8_t)(i-1u)))run_adjust_scan_waypoint(&road);
        *distance=(uint16_t)(*distance+obstacle_transit_length(&road));
        context.depth=(uint8_t)(i-1u);
        obstacle_cost_adjust_road(&context,&road);
#ifndef CHASSIS_OBSTACLE_TEST
        if(job==FULL_ROUTE_JOBS-1u && i==walk->depth)run_merge_home_offsets(&road);
#endif
        if(!(*passed&(1u<<road.road)))cost+=OBSTACLE_UNKNOWN_COST_MS;
        scan_blend=run_blends_scan_exit(&road,*heading);
        if(scan_blend && active) {
            cost+=obstacle_action_cost(action,entry_wait);
            active=0;
        }
        j=scan_blend ? 1u : 0u;
#ifndef CHASSIS_OBSTACLE_TEST
        if(i==1u && !job && run_blends_obstacle_launch(&road))j=1;
#endif
        for(;j<road.count;j++) {
            RunLeg leg;
            int home=obstacle_home_blend(&road,j,job,context.goal);
            RoadHeading chosen=home && !(*passed&(1u<<road.road))
                ? obstacle_home_heading(&road,j,*heading)
                : obstacle_branch_heading(&road,j,*heading,*passed,walk->blocked,job);
            (void)run_resolve_leg(&road,j,0,chosen,*passed,&leg);
            cost+=obstacle_turn_cost_ms(*heading,leg.heading);
            *heading=leg.heading;
            /* 与run_next_straight相同：同轴、同号、同朝向才能连走。
             * 这里只到当前作业目标，工位边界不合并；持续通过不重复收取静止等待。 */
            if(active && action.axis==leg.axis && action.sign==leg.sign && action.heading==leg.heading)
                action.mm=(uint16_t)(action.mm+leg.mm);
            else {
                if(active)cost+=obstacle_action_cost(action,entry_wait);
                action=leg;active=1;
                entry_wait=(uint8_t)(!(*passed&(1u<<road.road)) && !obstacle_close_approach(&road));
            }
            if(home)break; /* 末尾偏移与主段共用一次起停。 */
        }
        *passed|=(uint16_t)(1u<<road.road);
    }
    if(active)cost+=obstacle_action_cost(action,entry_wait);
    arrival=obstacle_task_heading(walk->nodes[walk->depth]);
    if(arrival>=0) {
        cost+=obstacle_turn_cost_ms(*heading,(RoadHeading)arrival);
        *heading=(RoadHeading)arrival;
    }
    return cost;
}

enum { OBSTACLE_MISSION_MEMO_SLOTS=2048u };
/* 未来阶段的起点/朝向由前一作业固定，故状态只需(job, passed)。
 * 根阶段可在任意节点/朝向及A/C部分连接处，不写入此缓存；封路/起点变化时清空。 */
static struct {uint32_t cost;uint16_t key;} obstacle_mission_memo[OBSTACLE_MISSION_MEMO_SLOTS];
static struct {
    uint32_t candidates,hits;
    uint16_t used;
    uint8_t saturated;
} obstacle_mission_stats;

typedef struct {
    ObstaclePathWalk walk;
    RoadHeading heading;
    uint16_t passed,pending_distance,best_distance;
    uint32_t pending_cost,best;
} ObstacleMissionFrame;
static ObstacleMissionFrame obstacle_mission_frames[FULL_ROUTE_JOBS];

static unsigned obstacle_memo_slot(uint16_t key)
{
    unsigned i,start=((uint32_t)key*2654435761u)>>21;
    for(i=0;i<OBSTACLE_MISSION_MEMO_SLOTS;i++) {
        unsigned slot=(start+i)&(OBSTACLE_MISSION_MEMO_SLOTS-1u);
        if(obstacle_mission_memo[slot].key==key || obstacle_mission_memo[slot].key==UINT16_MAX)return slot;
    }
    return OBSTACLE_MISSION_MEMO_SLOTS;
}

static int obstacle_memo_get(uint8_t job,uint16_t passed,uint32_t *cost)
{
    uint16_t key=(uint16_t)(((uint16_t)job<<ROAD_COUNT)|passed);
    unsigned slot=obstacle_memo_slot(key);
    if(slot==OBSTACLE_MISSION_MEMO_SLOTS || obstacle_mission_memo[slot].key!=key)return 0;
    *cost=obstacle_mission_memo[slot].cost;obstacle_mission_stats.hits++;return 1;
}

static void obstacle_memo_put(uint8_t job,uint16_t passed,uint32_t cost)
{
    uint16_t key=(uint16_t)(((uint16_t)job<<ROAD_COUNT)|passed);
    unsigned slot=obstacle_memo_slot(key);
    /* 缓存只影响运算量。容量耗尽仍精确枚举，不退回短视选路或伪报无路。 */
    if(slot==OBSTACLE_MISSION_MEMO_SLOTS){obstacle_mission_stats.saturated=1;return;}
    if(obstacle_mission_memo[slot].key==UINT16_MAX)obstacle_mission_stats.used++;
    obstacle_mission_memo[slot].key=key;obstacle_mission_memo[slot].cost=cost;
}

static void obstacle_mission_reset(void)
{
    memset(obstacle_mission_memo,255,sizeof(obstacle_mission_memo));
    memset(&obstacle_mission_stats,0,sizeof(obstacle_mission_stats));
}

static void obstacle_mission_frame_start(unsigned depth,uint8_t from,RoadHeading heading,
                                         uint16_t blocked,uint16_t passed)
{
    ObstacleMissionFrame *frame=&obstacle_mission_frames[depth];
    memset(frame,0,sizeof(*frame));obstacle_walk_start(&frame->walk,from,blocked);
    frame->heading=heading;frame->passed=passed;frame->best=UINT32_MAX;frame->best_distance=UINT16_MAX;
}

static void obstacle_mission_consider(unsigned depth,uint32_t future,RouteTablePath *out)
{
    ObstacleMissionFrame *frame=&obstacle_mission_frames[depth];
    uint32_t cost;
    if(future==UINT32_MAX)return;
    cost=frame->pending_cost+future;
    if(cost<frame->best || (cost==frame->best && frame->pending_distance<frame->best_distance)) {
        frame->best=cost;frame->best_distance=frame->pending_distance;
        if(!depth) {
            out->distance=frame->pending_distance;out->count=(uint8_t)(frame->walk.depth+1u);
            memcpy(out->nodes,frame->walk.nodes,out->count);
        }
    }
}

/* 显式栈的后序动态规划：当前候选路径代价 + 全部后续任务的最小代价。
 * 只在停车节点/已确认的退回点调用。每次循环响应IRQ置位的人工急停请求。 */
static int obstacle_mission_route(uint8_t from,RoadHeading heading,uint8_t first_job,
                                  uint16_t blocked,uint16_t passed,RouteTablePath *out,uint32_t *score)
{
    unsigned depth=0;
    out->count=0;obstacle_mission_frame_start(0,from,heading,blocked,passed);
    for(;;) {
        ObstacleMissionFrame *frame=&obstacle_mission_frames[depth];
        uint8_t job=(uint8_t)(first_job+depth);
        if(stop_pending)return 0;
        if(obstacle_walk_next(&frame->walk,obstacle_job_goal(job))) {
            RoadHeading arrival=frame->heading;
            uint16_t learned=frame->passed;uint32_t future;
            frame->pending_cost=obstacle_path_cost(&frame->walk,job,frame->walk.nodes[0],
                &arrival,&learned,&frame->pending_distance);
            obstacle_mission_stats.candidates++;
            if(job==FULL_ROUTE_JOBS-1u)obstacle_mission_consider(depth,0,out);
            else if(obstacle_memo_get((uint8_t)(job+1u),learned,&future))
                obstacle_mission_consider(depth,future,out);
            else {
                uint8_t next_from=frame->walk.nodes[frame->walk.depth];
                depth++;obstacle_mission_frame_start(depth,next_from,arrival,blocked,learned);
            }
        } else {
            uint32_t result=frame->best;
            if(!depth){if(score)*score=result;return result!=UINT32_MAX;}
            obstacle_memo_put(job,frame->passed,result);
            depth--;obstacle_mission_consider(depth,result,out);
        }
    }
}

#if defined(NAV_ENTRY_HOST_TEST) && defined(CHASSIS_OBSTACLE_TEST)
static int obstacle_select_remaining(uint8_t from,RoadHeading heading,uint8_t job,
                                     uint16_t blocked,uint16_t passed,RouteTablePath *out,uint32_t *score)
{
    obstacle_mission_reset();
    return obstacle_mission_route(from,heading,job,blocked,passed,out,score);
}

/* 保留单目标查询供独立地图/代价核对；正式诊断发车及改道使用上面的剩余任务规划。 */
static int obstacle_route_lookup(uint8_t from,uint8_t to,uint16_t blocked,
                                 uint16_t passed,RouteTablePath *out)
{
    ObstaclePathWalk walk;uint32_t best=UINT32_MAX;
    out->count=0;obstacle_walk_start(&walk,from,blocked);
    while(obstacle_walk_next(&walk,to)) {
        RoadHeading heading=run.heading;uint16_t learned=passed,distance;
        uint32_t cost=obstacle_path_cost(&walk,run.job,from,&heading,&learned,&distance);
        if(cost<best || (cost==best && distance<out->distance)) {
            best=cost;out->distance=distance;out->count=(uint8_t)(walk.depth+1u);
            memcpy(out->nodes,walk.nodes,out->count);
        }
    }
    return best!=UINT32_MAX;
}
#endif

static int obstacle_plan_remaining(uint8_t from)
{
    uint8_t nodes[sizeof(obstacle_nodes)],count=1,job,i;
    uint16_t passed=run.completed_roads;
    RoadHeading heading=run.heading;
    RouteTablePath path;
    uint32_t begin=clock_ms;
    if(stop_pending)return 0;
    obstacle_mission_reset();nodes[0]=from;
    for(job=run.job;job<FULL_ROUTE_JOBS;job++) {
        uint32_t score;
        if(!obstacle_mission_route(nodes[count-1u],heading,job,
                                  run.blocked_roads,passed,&path,&score))return 0;
        if(job==run.job) {
            text("[route cost] job=");number(job);text(" horizon=REMAINING score_ms=");number(score);text(" path=");
            for(i=0;i<path.count;i++){if(i)text("->");send_byte((uint8_t)('A'+path.nodes[i]));}
            text("\r\n");
        }
        {
            ObstaclePathWalk chosen;
            uint16_t unused;
            memset(&chosen,0,sizeof(chosen));chosen.blocked=run.blocked_roads;chosen.depth=(uint8_t)(path.count-1u);
            memcpy(chosen.nodes,path.nodes,path.count);
            (void)obstacle_path_cost(&chosen,job,nodes[count-1u],&heading,&passed,&unused);
        }
        for(i=1;i<path.count;i++)nodes[count++]=path.nodes[i];
    }
    if(count<2u || stop_pending)return 0;
    memcpy(obstacle_nodes,nodes,count);obstacle_node_count=count;run.edge=0;obstacle_map_changed=0;
    text("[route planner] remaining_jobs=");number(FULL_ROUTE_JOBS-run.job);
    text(" candidates=");number(obstacle_mission_stats.candidates);text(" memo_entries=");number(obstacle_mission_stats.used);
    text(" memo_hits=");number(obstacle_mission_stats.hits);text(" memo_full=");number(obstacle_mission_stats.saturated);
    text(" workspace_bytes=");number(sizeof(obstacle_mission_memo)+sizeof(obstacle_mission_frames)+sizeof(obstacle_mission_stats));
    text(" compute_ms=");number(clock_ms-begin);text("\r\n");
    text("[return zone] selected=");send_byte((uint8_t)('A'+nodes[count-1u]));
    text(" candidates=A/C; final entry=INSIDE\r\n");
    return 1;
}
