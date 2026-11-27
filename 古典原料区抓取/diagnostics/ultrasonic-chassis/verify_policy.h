/* 仅主机：完整剩余任务的递归参考。路径/坐标/朝向/分组独立计算，
 * 剖面作为已独立核验的公共数值模块使用；稠密表不编入STM32。 */
static uint32_t reference_mission_memo[FULL_ROUTE_JOBS][1u<<ROAD_COUNT];
static uint8_t reference_mission_nodes[FULL_ROUTE_JOBS][9];
static uint8_t reference_mission_root,reference_mission_job;
static RoadHeading reference_mission_heading;
static uint16_t reference_mission_blocked;
static uint32_t reference_mission_cost(uint8_t job,uint16_t passed);

static void reference_mission_paths(uint8_t job,uint8_t node,uint8_t depth,uint16_t seen,
                                    uint16_t passed,uint16_t path_mask,uint32_t *best)
{
    unsigned i;
    uint8_t goal=job==7u ? OBSTACLE_ANY_START : full_route_jobs[run.plan][job];
    reference_mission_nodes[job][depth]=node;
    if(node==goal || (goal==OBSTACLE_ANY_START && (node==NODE_A || node==NODE_C))) {
        RoadHeading heading=job==reference_mission_job ? reference_mission_heading
            : (RoadHeading)obstacle_task_heading(full_route_jobs[run.plan][job-1u]);
        uint32_t cost=reference_path_cost(reference_mission_nodes[job],depth+1u,passed,heading,job,reference_mission_blocked);
        uint32_t later=reference_mission_cost((uint8_t)(job+1u),(uint16_t)(passed|path_mask));
        if(later!=UINT32_MAX && cost+later<*best)*best=cost+later;
        return;
    }
    for(i=0;i<ROAD_DIRECTION_COUNT;i++) {
        const RoadPlan *road=&road_motion_table[i];
        if(road->from!=node || (seen&(1u<<road->to)) || (reference_mission_blocked&(1u<<road->road)))continue;
        reference_mission_paths(job,road->to,(uint8_t)(depth+1u),(uint16_t)(seen|(1u<<road->to)),
            passed,(uint16_t)(path_mask|(1u<<road->road)),best);
    }
}

static uint32_t reference_mission_cost(uint8_t job,uint16_t passed)
{
    uint8_t from;uint32_t best=UINT32_MAX;
    if(job==FULL_ROUTE_JOBS)return 0;
    if(reference_mission_memo[job][passed]!=UINT32_MAX)return reference_mission_memo[job][passed];
    from=job==reference_mission_job ? reference_mission_root : full_route_jobs[run.plan][job-1u];
    reference_mission_paths(job,from,0,(uint16_t)(1u<<from),passed,0,&best);
    reference_mission_memo[job][passed]=best;return best;
}

static uint32_t reference_remaining(uint8_t from,RoadHeading heading,uint8_t job,uint16_t blocked,uint16_t passed)
{
    reference_mission_root=from;reference_mission_heading=heading;
    reference_mission_job=job;reference_mission_blocked=blocked;
    memset(reference_mission_memo,255,sizeof(reference_mission_memo));
    return reference_mission_cost(job,passed);
}

static uint32_t assembled_mission_cost(void)
{
    RoadHeading heading=run.heading;uint16_t passed=run.completed_roads;
    unsigned job,index=0;uint32_t cost=0;
    for(job=run.job;job<FULL_ROUTE_JOBS;job++) {
        ObstaclePathWalk walk;uint16_t distance;
        uint8_t target=job==7u ? RUN_NODES[RUN_NODE_COUNT-1u] : full_route_jobs[run.plan][job];
        memset(&walk,0,sizeof(walk));walk.blocked=run.blocked_roads;walk.nodes[0]=RUN_NODES[index];
        while(walk.nodes[walk.depth]!=target) {
            CHECK(index+1u<RUN_NODE_COUNT && walk.depth<8u);
            walk.nodes[++walk.depth]=RUN_NODES[++index];
        }
        cost+=obstacle_path_cost(&walk,(uint8_t)job,walk.nodes[0],&heading,&passed,&distance);
    }
    CHECK(index+1u==RUN_NODE_COUNT);return cost;
}

static void full_mission_regression_test(void)
{
    uint32_t cost;
    start_at(NODE_A);run.blocked_roads=(uint16_t)(1u<<road_index(NODE_E,NODE_F));
    CHECK(obstacle_plan_remaining(NODE_A));cost=assembled_mission_cost();
    printf("MISSION_PROFILE cost=%lu reference=%lu\n",(unsigned long)cost,(unsigned long)reference_remaining(NODE_A,run.heading,0,run.blocked_roads,0));
    CHECK(cost==reference_remaining(NODE_A,run.heading,0,run.blocked_roads,0));
    if(CAR_START_INSET_MM==70 && CAR_MAP_STORAGE_INSET_MM==80) {
        if(CAR_MAP_SCAN_TOWARD_A_MM==200)CHECK(cost==88671u);
        if(CAR_MAP_SCAN_TOWARD_A_MM==120)CHECK(cost==88921u);
        if(CAR_MAP_SCAN_TOWARD_A_MM==60)CHECK(cost==89121u);
        if(!CAR_MAP_SCAN_TOWARD_A_MM)CHECK(cost==89296u);
    }
    run.job=0;queue_byte('!');plant_step();
    puts("PASS full-mission regression: A start with E-F blocked; matches independent remaining-mission reference without fixed node dwell");
}

static double reference_motion_ms(BodyAxis axis,int sign,unsigned mm)
{
    double remaining=mm,rpm=0,elapsed=0;
    double decel=axis==BODY_LEFT ? CAR_LATERAL_DECEL_RPM_PER_S : CAR_DRIVE_DECEL_RPM_PER_S;
    double lookahead=axis==BODY_LEFT ? CAR_LATERAL_BRAKE_LOOKAHEAD_MS : CAR_DRIVE_BRAKE_LOOKAHEAD_MS;
    double q=3200.0/(60.0*(axis==BODY_LEFT ? (sign<0 ? CALIBRATED_PPM_RIGHT : CALIBRATED_PPM_LEFT) : sign<0 ? CALIBRATED_PPM_BACKWARD : CALIBRATED_PPM_FORWARD));
    double accel=axis==BODY_LEFT ? CAR_LATERAL_ACCEL_RPM_PER_S : CAR_ROUTE_ACCEL_RPM_PER_S;
    double maximum=axis==BODY_LEFT ? car_lateral_limit_rpm(mm) : CAR_RUN_RPM;
    double minimum=axis==BODY_LEFT ? CAR_LATERAL_SETTLE_MIN_RPM : CAR_DISTANCE_MIN_RPM;
    double lag=decel*lookahead/1000.0;
    /* 独立1ms积分核对25ms名义剖面的离散误差，不调用固件估时器。 */
    while(remaining>CAR_DISTANCE_TOLERANCE_MM) {
        double target=(int)remaining*CAR_DISTANCE_KP_X100/100;
        double brake=sqrt(lag*lag+2.0*decel*(remaining-CAR_DISTANCE_TOLERANCE_MM)/q)-lag;
        double change;
        if(target>(int)brake)target=(int)brake;
        if(target<minimum)target=minimum;
        if(target>maximum)target=maximum;
        if(mm-remaining<CAR_ENCODER_LEARN_MM && target>CAR_ENCODER_LEARN_RPM)target=CAR_ENCODER_LEARN_RPM;
        change=(target>rpm ? accel : decel)/1000.0;
        if(target>rpm){rpm+=change;if(rpm>target)rpm=target;}
        else {rpm-=change;if(rpm<target)rpm=target;}
        remaining-=(int)(rpm+0.000001)*q/1000.0;elapsed++;
    }
    return elapsed+ceil(rpm*1000.0/decel);
}

static void motion_cost_regression_test(void)
{
    static const unsigned lengths[]={60,120,170,780,1200,1201,1770,1940};
    ObstaclePathWalk walk;RoadHeading heading=HEAD_X_NEG;
    uint16_t passed=4095u,distance;uint32_t cost,known;
    unsigned i,profile;
    /* 避障程序无原料视觉补偿，F-E-D使用地图纵向距离之和。 */
    start_at(NODE_C);run.job=2;
    memset(&walk,0,sizeof(walk));walk.depth=2;walk.nodes[0]=NODE_F;walk.nodes[1]=NODE_E;walk.nodes[2]=NODE_D;
    cost=obstacle_path_cost(&walk,2,NODE_F,&heading,&passed,&distance);known=cost;
    CHECK(cost>=3039u+OBSTACLE_ACTION_COST_MS+CAR_ROUTE_NODE_STOP_MS+1500u);
    CHECK(cost==obstacle_motion_time_ms(BODY_LEFT,1,ROAD_RAW_LEG_MM+ROAD_BOTTOM_LEG_MM)
        +OBSTACLE_ACTION_COST_MS+CAR_ROUTE_NODE_STOP_MS+1500u);
    CHECK(distance==ROAD_RAW_LEG_MM+ROAD_BOTTOM_LEG_MM);
    passed=0;heading=HEAD_X_NEG;
    cost=obstacle_path_cost(&walk,2,NODE_F,&heading,&passed,&distance);
    CHECK(cost==known+2u*OBSTACLE_UNKNOWN_COST_MS+OBSTACLE_ENTRY_COST_MS);
    CHECK((passed&((1u<<8)|(1u<<9)))==((1u<<8)|(1u<<9)));
    /* 诊断版 H-E 与 E-B 按通道中线连续运动。 */
    walk.nodes[0]=NODE_H;walk.nodes[1]=NODE_E;walk.nodes[2]=NODE_B;
    passed=4095u;heading=HEAD_Y_POS;
    cost=obstacle_path_cost(&walk,7,NODE_H,&heading,&passed,&distance);
    CHECK(cost==obstacle_motion_time_ms(BODY_LEFT,1,ROAD_STORAGE_LEG_MM+ROAD_B_E_MM)
        +OBSTACLE_ACTION_COST_MS+CAR_ROUTE_NODE_STOP_MS+1500u);
    /* D-E-H为真正换轴的L形，保留两次起停与未知路入道等待。 */
    walk.nodes[0]=NODE_D;walk.nodes[1]=NODE_E;walk.nodes[2]=NODE_H;
    passed=0;heading=HEAD_Y_NEG;
    cost=obstacle_path_cost(&walk,3,NODE_D,&heading,&passed,&distance);
    CHECK(cost==obstacle_motion_time_ms(BODY_FORWARD,1,ROAD_BOTTOM_LEG_MM)+obstacle_motion_time_ms(BODY_LEFT,1,ROAD_STORAGE_LEG_MM)
        +2u*(OBSTACLE_ACTION_COST_MS+CAR_ROUTE_NODE_STOP_MS+OBSTACLE_UNKNOWN_COST_MS+OBSTACLE_ENTRY_COST_MS)+1500u);
    for(profile=0;profile<3;profile++)for(i=0;i<sizeof(lengths)/sizeof(lengths[0]);i++) {
        BodyAxis axis=profile==2 ? BODY_LEFT : BODY_FORWARD;int sign=profile==1 ? -1 : 1;
        double fine=reference_motion_ms(axis,sign,lengths[i]);
        uint32_t estimated=obstacle_motion_time_ms(axis,sign,(uint16_t)lengths[i]);
        if(fabs(estimated-fine)>150.0)fprintf(stderr,"profile comparison axis=%u sign=%d mm=%u coarse=%u fine=%.1f\n",axis,sign,lengths[i],estimated,fine);
        CHECK(fabs(estimated-fine)<=150.0);
    }
    run.job=0;queue_byte('!');plant_step();
    puts("PASS motion cost: F-E-D and H-E-B use the configured combined distance; B-side link charged separately; L turns retain separate actions;24 profile cases match1ms integration within150ms");
}

static void turn_event_cost_tests(void)
{
    static const RoadHeading headings[3]={HEAD_X_NEG,HEAD_Y_POS,HEAD_X_POS};
    unsigned i,start;
    start_at(NODE_C);run.job=2;
    for(i=0;i<3;i++) {
        ObstaclePathWalk walk;RoadHeading h=headings[i];uint16_t passed=4095u,distance;
        uint32_t cost,motion;
        memset(&walk,0,sizeof(walk));walk.depth=1;walk.nodes[0]=NODE_I;walk.nodes[1]=NODE_D;
        cost=obstacle_path_cost(&walk,2,NODE_I,&h,&passed,&distance);
        motion=obstacle_motion_time_ms(i==1u ? BODY_LEFT : BODY_FORWARD,i==0u ? -1 : 1,ROAD_STORAGE_LEG_MM);
        CHECK(cost==motion+OBSTACLE_ACTION_COST_MS+CAR_ROUTE_NODE_STOP_MS+(i<2u ? 1500u : 0u));
        CHECK(h==HEAD_X_POS);
    }
    run.job=0;queue_byte('!');plant_step();
    for(start=0;start<2;start++) {
        uint32_t deadline;unsigned jobs=0;
        start_at(start ? NODE_C : NODE_A);deadline=clock_ms+240000u;
        while(run_active && clock_ms<deadline) {
            step_with_sonar(-1,-1);
            if((run.job==2u || run.job==5u) && run.job!=jobs) {
                CHECK(!run.blocked_roads && RUN_NODES[0]==NODE_F
                    && RUN_NODES[1]==NODE_E && RUN_NODES[2]==NODE_D);
                jobs=run.job;
            }
        }
        CHECK(run.state==RUN_DONE && jobs==5u);
    }
    puts("PASS turn estimate: one1500ms event for90/180deg, zero when unchanged; A/C both raw departures use F-E-D when no blocked roads");
}

static void either_start_return_tests(void)
{
    unsigned start,finish;
    for(start=0;start<2u;start++)for(finish=0;finish<2u;finish++) {
        uint8_t home=finish ? NODE_C : NODE_A;
        uint8_t closed=finish ? NODE_A : NODE_C;
        RoadPoint origin,h;
        start_at(start ? NODE_C : NODE_A);
        CHECK(road_point(start ? NODE_C : NODE_A,&origin) && road_point(NODE_H,&h));
        run.job=checked_job=7;run.heading=run.target_heading=HEAD_Y_POS;
        field_x=h.x-origin.x;field_y=h.y-origin.y;
        yaw=initial_field_heading*9000.0;
        heading_sensor_publish(&heading_sensor,heading_wrap_cdeg((int32_t)yaw),clock_ms);
        run.blocked_roads=(uint16_t)((1u<<road_index(closed,NODE_B))
            |(1u<<road_index(closed,closed==NODE_A ? NODE_F : NODE_D)));
        CHECK(obstacle_plan_remaining(NODE_H));
        CHECK(RUN_NODES[RUN_NODE_COUNT-1u]==home);
        CHECK(run_road());finish_route();
        CHECK(obstacle_inset_node==255u && !obstacle_inset_axes);
    }
    /* 两区可达时先选朝向更合适的A；返程B-A遇障后改选C并完整入区。 */
    start_at(NODE_C);run.job=checked_job=7;
    run.heading=run.target_heading=HEAD_Y_NEG;
    field_x=-CAR_START_INSET_MM;field_y=-ROAD_START_TO_B_MM;yaw=0;
    heading_sensor_publish(&heading_sensor,0,clock_ms);
    CHECK(obstacle_plan_remaining(NODE_B));CHECK(RUN_NODES[RUN_NODE_COUNT-1u]==NODE_A);
    CHECK(run_road());
    {uint32_t deadline=clock_ms+20000u;int sensor;
     while(clock_ms<deadline && !(run.road.from==NODE_B && run.road.to==NODE_A
        && run.progress_mm>=100 && obstacle_sensor_for_leg()>=0))step_with_sonar(-1,-1);
     CHECK(clock_ms<deadline);sensor=obstacle_sensor_for_leg();
     obstacle_io_shot((uint8_t)sensor);obstacle_io_result((uint8_t)sensor,180);
     obstacle_io_shot((uint8_t)sensor);obstacle_io_result((uint8_t)sensor,180);
     CHECK(run.state==RUN_OBSTACLE_STOP);
     deadline=clock_ms+40000u;
     while(clock_ms<deadline && run.obstacle_phase)step_with_sonar(-1,-1);
     CHECK(clock_ms<deadline && run_active && RUN_NODES[RUN_NODE_COUNT-1u]==NODE_C);}
    finish_route();CHECK(obstacle_inset_node==255u && !obstacle_inset_axes);
    puts("PASS either start zone: all four launch/finish combinations enter physical zone; both reachable choose by cost; B-A blocked during return switches to C and finishes");
}

static void g_connector_tests(void)
{
    RoadPlan road;
    const int g_y=200+CAR_MAP_TOP_INSET_MM+CAR_MAP_RAW_EXTRA_INSET_MM;
    const int hg_mm=1200-g_y;
    start_at(NODE_C);run.job=3;
    obstacle_nodes[0]=NODE_H;obstacle_nodes[1]=NODE_G;obstacle_nodes[2]=NODE_F;obstacle_node_count=3;
    CHECK(run_plan_road(0,&road));CHECK(road.count==1u && road.length==hg_mm && road.segments[0].mm==-hg_mm);
    CHECK(run_plan_road(1,&road));CHECK(road.count==1u && road.length==ROAD_STORAGE_LEG_MM
        && road.segments[0].axis==BODY_LEFT && road.segments[0].mm==-ROAD_STORAGE_LEG_MM);
    obstacle_nodes[0]=NODE_F;obstacle_nodes[1]=NODE_G;obstacle_nodes[2]=NODE_H;
    CHECK(run_plan_road(0,&road));CHECK(road.count==1u && road.length==ROAD_STORAGE_LEG_MM && road.segments[0].mm==ROAD_STORAGE_LEG_MM);
    CHECK(run_plan_road(1,&road));CHECK(road.count==1u && road.length==hg_mm && road.segments[0].mm==hg_mm);
    /* G连接点跟随当前上侧/原料内移参数。 */
    run.road=road;run.edge=1;run.segment=0;run.action_mm=hg_mm;run.progress_mm=100;
    run.target_heading=run.heading=HEAD_Y_POS;run.yaw_zero_cdeg=0;
    run.state=RUN_MOVING;run.velocity_phase=VELOCITY_FORWARD;run.encoder_ready=ALL_MOTORS_MASK;
    obstacle_capture_ray(0);CHECK(obstacle_ray.valid && fabsf(obstacle_ray.x-(CAR_MAP_LEFT_X_MM+CAR_MAP_STORAGE_INSET_MM))<0.1f
        && fabsf(obstacle_ray.y-(g_y+100+OBSTACLE_FRONT_FORWARD_MM))<0.1f);
    run.job=0;checked_job=0;queue_byte('!');plant_step();
}

static void g_connector_motion_tests(void)
{
    unsigned start,target;
    const int g_y=200+CAR_MAP_TOP_INSET_MM+CAR_MAP_RAW_EXTRA_INSET_MM;
    uint16_t g_roads=(uint16_t)((1u<<road_index(NODE_F,NODE_G))|(1u<<road_index(NODE_G,NODE_H)));
    for(start=0;start<2;start++) {
        start_at(start ? NODE_C : NODE_A);
        /* 封DE/EF，取料后到粗加工必须经G-H-I，检验真实段长与作业位置。 */
        run.blocked_roads=(uint16_t)((1u<<road_index(NODE_D,NODE_E))|(1u<<road_index(NODE_E,NODE_F)));
        CHECK(obstacle_plan_remaining(start ? NODE_C : NODE_A));
        finish_route();CHECK((run.completed_roads&g_roads)==g_roads);
    }
    for(target=0;target<2;target++) {
        uint8_t goal=target ? NODE_H : NODE_F;
        uint32_t deadline;
        int sensor;
        start_at(NODE_C);
        run.job=checked_job=target ? 3u : 1u;
        run.heading=run.target_heading=target ? HEAD_Y_POS : HEAD_X_POS;
        field_x=(CAR_MAP_LEFT_X_MM+CAR_MAP_STORAGE_INSET_MM)-(2190.0+CAR_START_INSET_MM);field_y=g_y-2230.0;yaw=target ? 18000.0 : -9000.0;
        heading_sensor_publish(&heading_sensor,heading_wrap_cdeg((int32_t)yaw),clock_ms);
        CHECK(obstacle_plan_remaining(NODE_G));deadline=clock_ms+20000u;
        while(clock_ms<deadline && !(run.road.from==NODE_G && run.road.to==goal
            && run.progress_mm>=100 && obstacle_sensor_for_leg()>=0))step_with_sonar(-1,-1);
        CHECK(clock_ms<deadline);sensor=obstacle_sensor_for_leg();
        obstacle_io_shot((uint8_t)sensor);obstacle_io_result((uint8_t)sensor,180);
        obstacle_io_shot((uint8_t)sensor);obstacle_io_result((uint8_t)sensor,180);
        CHECK(run.state==RUN_OBSTACLE_STOP && run.obstacle_node==NODE_G);
        deadline=clock_ms+40000u;
        while(clock_ms<deadline && run.obstacle_phase)step_with_sonar(-1,-1);
        CHECK(clock_ms<deadline && run_active && !run.obstacle_phase);
        CHECK(fabs(field_x-((CAR_MAP_LEFT_X_MM+CAR_MAP_STORAGE_INSET_MM)-(2190.0+CAR_START_INSET_MM)))<30.0 && fabs(field_y-(g_y-2230.0))<30.0);
        CHECK(run.road.from==NODE_G && run.road.to==(target ? NODE_F : NODE_H) && run.road.count==1u);
        finish_route();
    }
    puts("PASS G connector: current configured map, no short return legs; A/C forced G full rounds; G-F/G-H obstacles retreat to adjusted G, detour and finish; observer pose agrees");
}

static void branch_heading_tests(void)
{
    RoadPlan road;
    RunLeg leg;
    start_at(NODE_C);CHECK(road_plan(NODE_D,NODE_I,&road));
    /* D处向左去I、向上去E；原先只右转90度朝I，看不到DE。 */
    CHECK(run_resolve_leg(&road,0,0,HEAD_Y_POS,0,&leg));
    CHECK(leg.heading==HEAD_Y_NEG && leg.axis==BODY_LEFT && leg.sign==1);
    run.blocked_roads=(uint16_t)(1u<<road_index(NODE_D,NODE_E));
    CHECK(run_resolve_leg(&road,0,0,HEAD_Y_POS,0,&leg));CHECK(leg.heading==HEAD_X_NEG);
    run.blocked_roads=0;
    CHECK(run_resolve_leg(&road,0,0,HEAD_Y_POS,(uint16_t)(1u<<road_index(NODE_D,NODE_E)),&leg));
    CHECK(leg.heading==HEAD_X_NEG); /* 已走DE不占用探头朝向。 */
    CHECK(run_resolve_leg(&road,0,0,HEAD_Y_NEG,0,&leg));CHECK(leg.heading==HEAD_Y_NEG);
    CHECK(run_resolve_leg(&road,0,0,HEAD_Y_POS,(uint16_t)(1u<<road.road),&leg));
    CHECK(leg.heading==HEAD_Y_POS && leg.sign==-1);
    CHECK(road_plan(NODE_E,NODE_D,&road));
    run.blocked_roads=(uint16_t)((1u<<road_index(NODE_E,NODE_B))|(1u<<road_index(NODE_E,NODE_H)));
    CHECK(run_resolve_leg(&road,0,0,HEAD_Y_NEG,0,&leg));CHECK(leg.heading==HEAD_X_NEG);
    /* 整张路网：各朝向、各运动段及32组封路/已走状态核对独立点积参考。 */
    {unsigned v,i,j,h;uint32_t random=20261120u;
     for(v=0;v<32;v++) {
        uint16_t passed,blocked;
        random=random*1664525u+1013904223u;blocked=(uint16_t)((random>>12)&4095u);
        random=random*1664525u+1013904223u;passed=(uint16_t)((random>>12)&4095u);
        for(i=0;i<ROAD_DIRECTION_COUNT;i++) {
            road=road_motion_table[i];
            if(obstacle_start_node(road.from))obstacle_adjust_inset(&road,road.from,3,-1);
            if(obstacle_start_node(road.to))obstacle_adjust_inset(&road,road.to,3,1);
            obstacle_adjust_corner(&road);run.blocked_roads=(uint16_t)(blocked&~(1u<<road.road));
            for(j=0;j<road.count;j++)for(h=0;h<4;h++) {
                MotionParameters p=road.segments[j];
                int dx=p.axis==BODY_LEFT ? (p.mm>0 ? -1 : 1) : 0;
                int dy=p.axis==BODY_FORWARD ? (p.mm>0 ? 1 : -1) : 0;
                RoadHeading expected=reference_branch_heading(&road,j,dx,dy,(RoadHeading)h,passed,run.blocked_roads);
                CHECK(run_resolve_leg(&road,(uint8_t)j,0,(RoadHeading)h,passed,&leg));
                CHECK(leg.heading==expected);
            }
        }
     }}
    run.blocked_roads=0;
    run.job=0;checked_job=0;queue_byte('!');plant_step();
    puts("PASS branch heading: D chooses front=DE/left=DI once; blocked/passed exits excluded; visible/known roads keep yaw; opposite exits not jointly covered; 24 roads x 4 headings x 32 states checked");
}

static void i_connector_tests(void)
{
    RoadPlan road;
    start_at(NODE_C);run.job=3;
    obstacle_nodes[0]=NODE_D;obstacle_nodes[1]=NODE_I;obstacle_nodes[2]=NODE_H;obstacle_node_count=3;
    CHECK(run_plan_road(0,&road));CHECK(road.count==1u && road.segments[0].mm==ROAD_STORAGE_LEG_MM && road.length==ROAD_STORAGE_LEG_MM);
    CHECK(run_plan_road(1,&road));CHECK(road.count==1u && road.segments[0].axis==BODY_FORWARD
        && road.segments[0].mm==-ROAD_BOTTOM_LEG_MM && road.length==ROAD_BOTTOM_LEG_MM);
    obstacle_nodes[0]=NODE_H;obstacle_nodes[1]=NODE_I;obstacle_nodes[2]=NODE_D;
    CHECK(run_plan_road(0,&road));CHECK(road.count==1u && road.segments[0].mm==ROAD_BOTTOM_LEG_MM && road.length==ROAD_BOTTOM_LEG_MM);
    CHECK(run_plan_road(1,&road));CHECK(road.count==1u && road.segments[0].mm==-ROAD_STORAGE_LEG_MM && road.length==ROAD_STORAGE_LEG_MM);
    /* 旁观测必须使用同一个I连接点，不能仍从旧x=340倒推位置。 */
    obstacle_nodes[0]=NODE_I;obstacle_nodes[1]=NODE_H;obstacle_node_count=2;
    CHECK(run_plan_road(0,&run.road));run.segment=run.edge=0;
    run.target_heading=run.heading=HEAD_Y_NEG;run.yaw_zero_cdeg=-CAR_MAP_YAW_SIGN*18000;
    run.state=RUN_MOVING;run.velocity_phase=VELOCITY_FORWARD;run.encoder_ready=ALL_MOTORS_MASK;
    run.action_mm=ROAD_BOTTOM_LEG_MM;run.progress_mm=100;
    obstacle_capture_ray(0);CHECK(obstacle_ray.valid && fabsf(obstacle_ray.x-(CAR_MAP_LEFT_X_MM+CAR_MAP_STORAGE_INSET_MM))<0.1f
        && fabsf(obstacle_ray.y-(1200.0f+ROAD_BOTTOM_LEG_MM-100.0f-OBSTACLE_FRONT_FORWARD_MM))<0.1f);
    run.job=0;checked_job=0;queue_byte('!');plant_step();
}

static void i_connector_motion_tests(void)
{
    unsigned start;
    for(start=0;start<2;start++) {
        start_at(start ? NODE_C : NODE_A);
        run.blocked_roads=(uint16_t)(1u<<road_index(NODE_D,NODE_E));
        CHECK(obstacle_plan_remaining(start ? NODE_C : NODE_A));
        finish_route();CHECK(run.completed_roads&(1u<<road_index(NODE_D,NODE_I)));
    }
    /* 在I-H中途遇障碍，退回避障行驶线的 I，再换路完成任务。 */
    start_at(NODE_C);
    /* 从D作业已完成、I-H尚未知的状态验证后续流程；整圈可能先经H-I到D而已走通此路。 */
    run.job=checked_job=3;run.heading=run.target_heading=HEAD_X_POS;
    field_x=CAR_MAP_CENTER_X_MM-(ROAD_RIGHT_LANE_X_MM+CAR_START_INSET_MM);field_y=-ROAD_C_BOTTOM_LINK_MM;yaw=-9000.0;
    heading_sensor_publish(&heading_sensor,-9000,clock_ms);
    run.blocked_roads=(uint16_t)(1u<<road_index(NODE_D,NODE_E));
    CHECK(obstacle_plan_remaining(NODE_D));
    {uint32_t deadline=clock_ms+130000u;
     int sensor;
     while(clock_ms<deadline && !(run.road.from==NODE_I && run.road.to==NODE_H
        && run.progress_mm>=100 && obstacle_sensor_for_leg()>=0))step_with_sonar(-1,-1);
     if(clock_ms>=deadline)fprintf(stderr,"I-H test deadline state=%u job=%u road=%u-%u passed=%u blocked=%u progress=%ld watch=%d\n",
         run.state,run.job,run.road.from,run.road.to,run.completed_roads,run.blocked_roads,(long)run.progress_mm,obstacle_sensor_for_leg());
     CHECK(clock_ms<deadline);sensor=obstacle_sensor_for_leg();
     obstacle_io_shot((uint8_t)sensor);obstacle_io_result((uint8_t)sensor,180);
     obstacle_io_shot((uint8_t)sensor);obstacle_io_result((uint8_t)sensor,180);
     CHECK(run.state==RUN_OBSTACLE_STOP && run.obstacle_node==NODE_I);
     deadline=clock_ms+40000u;
     while(clock_ms<deadline && run.obstacle_phase)step_with_sonar(-1,-1);
     CHECK(clock_ms<deadline && run_active && !run.obstacle_phase);
     CHECK(fabs(field_x-(CAR_MAP_LEFT_X_MM+CAR_MAP_STORAGE_INSET_MM-(2190.0+CAR_START_INSET_MM)))<30.0
        && fabs(field_y+ROAD_C_BOTTOM_LINK_MM)<30.0);
     CHECK(run.road.from==NODE_I && run.road.to==NODE_D && run.road.length==ROAD_STORAGE_LEG_MM);
    }
    finish_route();
    puts("PASS I connector: D-I-H and reverse use the simplified connector; A/C DE-blocked full rounds; I-H obstacle retreats to new I and completes detour; observation pose agrees");
}

static void directional_threshold_tests(void)
{
    unsigned i;
    uint32_t deadline;
    for(i=0;i<ROAD_DIRECTION_COUNT;i++) {
        const RoadPlan *road=&road_motion_table[i];
        CHECK(obstacle_close_approach(road)==(road->to==NODE_B || road->to==NODE_F));
    }
    start_at(NODE_C);run.obstacle_node=NODE_C;run.obstacle_phase=2;
    run.blocked_roads=(uint16_t)(1u<<road_index(NODE_C,NODE_B));
    CHECK(obstacle_replan());deadline=clock_ms+15000u;
    while(clock_ms<deadline && obstacle_sensor_for_leg()<0)plant_step();
    CHECK(clock_ms<deadline && !obstacle_close_approach(&run.road));
    for(i=0;i<200u;i++)plant_step();
    CHECK(run.progress_mm==0 && obstacle_entry_samples==0);
    {int sensor=obstacle_sensor_for_leg();
     obstacle_io_shot((uint8_t)sensor);obstacle_io_result((uint8_t)sensor,-1);
     CHECK(obstacle_entry_samples==1 && !obstacle_hits);
     for(i=0;i<30u;i++)plant_step();
     CHECK(run.progress_mm==0);
     obstacle_io_shot((uint8_t)sensor);obstacle_io_result((uint8_t)sensor,-1);
     CHECK(obstacle_entry_samples==2 && !obstacle_hits);}
    while(clock_ms<deadline && run.progress_mm<5)step_with_sonar(-1,-1);
    CHECK(clock_ms<deadline && run.progress_mm>=5);
    /* CD的主横向段有足够剩余距离，550计入，551不计入。 */
    while(clock_ms<deadline && (run.segment!=2u || obstacle_sensor_for_leg()<0))step_with_sonar(-1,-1);
    CHECK(clock_ms<deadline);
    {uint16_t before=run.blocked_roads;int sensor=obstacle_sensor_for_leg();
     obstacle_io_shot((uint8_t)sensor);obstacle_io_result((uint8_t)sensor,551);
     obstacle_io_shot((uint8_t)sensor);obstacle_io_result((uint8_t)sensor,551);
     CHECK(run.blocked_roads==before);
     obstacle_io_shot((uint8_t)sensor);obstacle_io_result((uint8_t)sensor,550);CHECK(run.blocked_roads==before);
     obstacle_io_shot((uint8_t)sensor);obstacle_io_result((uint8_t)sensor,550);
     CHECK(run.state==RUN_OBSTACLE_STOP && (run.blocked_roads&(1u<<road_index(NODE_C,NODE_D))));}
    queue_byte('!');plant_step();
    /* F各方向接近均先走；不等待超声波的两次入道结果。 */
    for(i=0;i<ROAD_DIRECTION_COUNT;i++)if(road_motion_table[i].to==NODE_F) {
        RoadHeading heading=HEAD_X_NEG;
        RunLeg leg;
        start_at(NODE_A);run.road=road_motion_table[i];run.segment=0;
        CHECK(run_resolve_leg(&run.road,0,0,heading,0,&leg));
        run.target_heading=leg.heading;run.travel_axis=leg.axis;run.travel_sign=leg.sign;
        run.state=RUN_MOVING;run.velocity_phase=VELOCITY_FORWARD;
        run.yaw_zero_cdeg=-CAR_MAP_YAW_SIGN*(int32_t)leg.heading*9000;
        run.action_mm=1000;run.progress_mm=0;
        obstacle_io_shot((uint8_t)(leg.axis==BODY_LEFT));obstacle_io_result((uint8_t)(leg.axis==BODY_LEFT),400);
        CHECK(!run.blocked_roads && obstacle_hits==0);
        obstacle_io_shot((uint8_t)(leg.axis==BODY_LEFT));obstacle_io_result((uint8_t)(leg.axis==BODY_LEFT),299);
        obstacle_io_shot((uint8_t)(leg.axis==BODY_LEFT));obstacle_io_result((uint8_t)(leg.axis==BODY_LEFT),299);
        CHECK(run.state==RUN_OBSTACLE_STOP && run.blocked_roads==(1u<<run.road.road));
        queue_byte('!');plant_step();
    }
}

static void prepare_eb_observation(int progress)
{
    start_at(NODE_C);
    obstacle_nodes[0]=NODE_E;obstacle_nodes[1]=NODE_B;obstacle_node_count=2;
    CHECK(road_plan(NODE_E,NODE_B,&run.road));
    run.heading=run.target_heading=HEAD_X_NEG;run.yaw_zero_cdeg=-CAR_MAP_YAW_SIGN*9000;
    run.state=RUN_MOVING;run.velocity_phase=VELOCITY_FORWARD;run.segment=0;
    run.travel_axis=BODY_FORWARD;run.travel_sign=-1;run.action_mm=ROAD_B_E_MM;run.progress_mm=progress;
    run.encoder_ready=ALL_MOTORS_MASK;
    run.completed_roads=(uint16_t)(1u<<run.road.road);
}

static void observation_tests(void)
{
    uint16_t de=(uint16_t)(1u<<road_index(NODE_D,NODE_E));
    unsigned i;
    prepare_eb_observation(0);
    CHECK(obstacle_io_range_enabled() && obstacle_sensor_for_leg()<0);
    obstacle_io_shot(1);CHECK(obstacle_ray.valid);
    CHECK(fabsf(obstacle_ray.x-CAR_MAP_CENTER_X_MM)<0.1f && fabsf(obstacle_ray.y-1335)<0.1f);
    obstacle_io_result(1,300);CHECK(!run.blocked_roads);
    clock_ms+=140;heading_sensor_publish(&heading_sensor,0,clock_ms);
    obstacle_io_shot(1);obstacle_io_result(1,300);
    CHECK(run.blocked_roads==de && obstacle_observed_roads==de && run.state==RUN_MOVING && !run.obstacle_phase);
    CHECK(obstacle_map_changed && run.continuation_mm==0);
    CHECK(!(run.completed_roads&de));
    /* 回波改变未来选路，不触发当前E-B无谓停车。 */
    run.job=3;CHECK(obstacle_plan_remaining(NODE_B));
    for(i=1;i<RUN_NODE_COUNT;i++)CHECK(road_index(RUN_NODES[i-1u],RUN_NODES[i])!=road_index(NODE_D,NODE_E));
    CHECK(!obstacle_map_changed);
    run.job=0;checked_job=0;queue_byte('!');plant_step();

    prepare_eb_observation(200); /* 离开E的射线已偏离DE，不能继续归到DE。 */
    for(i=0;i<3;i++){obstacle_io_shot(1);obstacle_io_result(1,300);}
    CHECK(!run.blocked_roads);
    prepare_eb_observation(0);
    for(i=0;i<3;i++){obstacle_io_shot(1);obstacle_io_result(1,-1);}
    CHECK(!run.blocked_roads && run.completed_roads==(1u<<road_index(NODE_E,NODE_B)));
    for(i=0;i<3;i++){obstacle_io_shot(1);obstacle_io_result(1,600);} /* 靠近D节点：有歧义，不旁观封路。 */
    CHECK(!run.blocked_roads);
    for(i=0;i<3;i++){obstacle_io_shot(1);obstacle_io_result(1,1400);}
    CHECK(!run.blocked_roads);
    obstacle_io_shot(1);run.segment=1;obstacle_io_result(1,300);CHECK(!run.blocked_roads);run.segment=0;
    obstacle_io_shot(1);clock_ms+=101;obstacle_io_result(1,300);CHECK(!run.blocked_roads);
    heading_sensor_publish(&heading_sensor,0,clock_ms);
    obstacle_io_shot(1);obstacle_io_result(1,300);CHECK(!run.blocked_roads);
    clock_ms+=501;heading_sensor_publish(&heading_sensor,0,clock_ms);
    obstacle_io_shot(1);obstacle_io_result(1,300);CHECK(!run.blocked_roads);
    clock_ms+=140;heading_sensor_publish(&heading_sensor,0,clock_ms);
    obstacle_io_shot(1);obstacle_io_result(1,450);CHECK(!run.blocked_roads); /* 世界坐标不一致不能拼两次。 */
    queue_byte('!');plant_step();
    prepare_eb_observation(0);CHECK(!obstacle_observations[1].hits && !obstacle_observed_roads);
    obstacle_io_shot(1);obstacle_io_result(1,300);
    obstacle_io_shot(1);obstacle_io_result(1,-1);
    obstacle_io_shot(1);obstacle_io_result(1,300);CHECK(!run.blocked_roads);
    obstacle_io_shot(0);obstacle_io_result(0,-1); /* 另一只探头不打断左侧连续结果。 */
    obstacle_io_shot(1);obstacle_io_result(1,300);CHECK(run.blocked_roads==de);
    queue_byte('!');plant_step();
    /* 原有同向连走必须在旁观测改变地图后撤销，否则会跳过节点重算。 */
    start_at(NODE_C);run.job=3;
    obstacle_nodes[0]=NODE_F;obstacle_nodes[1]=NODE_E;obstacle_nodes[2]=NODE_D;obstacle_nodes[3]=NODE_H;
    obstacle_node_count=4;run.completed_roads=4095u;
    CHECK(road_plan(NODE_F,NODE_E,&run.road));run.edge=run.segment=0;
    {RunLeg leg;leg.axis=BODY_FORWARD;leg.sign=1;leg.heading=HEAD_Y_POS;leg.mm=run.road.length;
     CHECK(run_straight_remaining(leg)>0);
     obstacle_map_changed=1;CHECK(run_straight_remaining(leg)==0);}
    run.job=0;checked_job=0;queue_byte('!');plant_step();
}

static void mission_route_tests(void)
{
    unsigned start,a,b,job,cases=0;
    for(start=0;start<2u;start++)for(a=0;a<=ROAD_COUNT;a++)for(b=a;b<=ROAD_COUNT;b++) {
        uint8_t node=start ? NODE_C : NODE_A;uint16_t passed=0,blocked;
        RoadHeading heading=start ? HEAD_Y_NEG : HEAD_X_NEG;
        if(a<ROAD_COUNT && b==ROAD_COUNT)continue;
        start_at(node);blocked=(uint16_t)((a<ROAD_COUNT ? 1u<<a : 0u)|(b<ROAD_COUNT ? 1u<<b : 0u));
        run.blocked_roads=blocked;
        {uint32_t expected=reference_remaining(node,heading,0,blocked,0);
         CHECK(obstacle_plan_remaining(node)==(expected!=UINT32_MAX));
         if(expected!=UINT32_MAX)CHECK(assembled_mission_cost()==expected);}
        for(job=0;job<FULL_ROUTE_JOBS;job++) {
            RouteTablePath path;ObstaclePathWalk walk;uint32_t score,expected,first,later;
            uint16_t learned,distance;RoadHeading arrival;unsigned i;
            run.job=(uint8_t)job;run.heading=heading;run.blocked_roads=blocked;
            obstacle_inset_node=(node==NODE_A || node==NODE_C) && job ? node : 255u;
            obstacle_inset_axes=obstacle_inset_node==255u ? 0u : 3u;
            expected=reference_remaining(node,heading,(uint8_t)job,blocked,passed);
            CHECK(obstacle_select_remaining(node,heading,(uint8_t)job,blocked,passed,&path,&score)==(expected!=UINT32_MAX));
            if(expected==UINT32_MAX)break;
            CHECK(score==expected && !obstacle_mission_stats.saturated);
            CHECK(path.count>1u && path.nodes[0]==node);
            CHECK(job==7u ? (path.nodes[path.count-1u]==NODE_A || path.nodes[path.count-1u]==NODE_C)
                : path.nodes[path.count-1u]==full_route_jobs[run.plan][job]);
            memset(&walk,0,sizeof(walk));walk.blocked=blocked;walk.depth=(uint8_t)(path.count-1u);memcpy(walk.nodes,path.nodes,path.count);
            learned=passed;arrival=heading;first=obstacle_path_cost(&walk,(uint8_t)job,node,&arrival,&learned,&distance);
            later=job==7u ? 0u : reference_remaining(path.nodes[path.count-1u],arrival,(uint8_t)(job+1u),blocked,learned);
            CHECK(later!=UINT32_MAX && first+later==score);
            for(i=1;i<path.count;i++)CHECK(!(blocked&(1u<<road_index(path.nodes[i-1u],path.nodes[i]))));
            node=path.nodes[path.count-1u];passed=learned;heading=arrival;
        }
        ++cases;run.job=0;checked_job=0;queue_byte('!');plant_step();
    }
    CHECK(cases==158u);
    puts("PASS remaining mission:158 A/C closure layouts, every pending job score and selected prefix match independent all-remaining-task optimum; cache not saturated");
}

static void remaining_root_state_tests(void)
{
    uint32_t random=20261122u;unsigned variant,node,cases=0;
    start_at(NODE_C);
    for(variant=0;variant<64u;variant++)for(node=0;node<9u;node++) {
        uint16_t blocked,passed;uint8_t job=(uint8_t)(variant%8u);uint32_t score,expected;
        RoadHeading heading=(RoadHeading)((variant/8u)%4u);RouteTablePath path;
        if((job==7u && (node==NODE_A || node==NODE_C)) || (job<7u && node==full_route_jobs[run.plan][job]))continue;
        random=random*1664525u+1013904223u;blocked=(uint16_t)((random>>12)&4095u);
        random=random*1664525u+1013904223u;passed=(uint16_t)(((random>>12)&4095u)&~blocked);
        if(variant<8u)blocked=passed=0;
        run.job=job;run.blocked_roads=blocked;
        obstacle_inset_node=obstacle_start_node((uint8_t)node) ? (uint8_t)node : 255u;
        obstacle_inset_axes=(uint8_t)(variant%4u);
        expected=reference_remaining((uint8_t)node,heading,job,blocked,passed);
        CHECK(obstacle_select_remaining((uint8_t)node,heading,job,blocked,passed,&path,&score)==(expected!=UINT32_MAX));
        CHECK(score==expected && !obstacle_mission_stats.saturated);cases++;
    }
    run.job=0;checked_job=0;queue_byte('!');plant_step();
    start_at(NODE_C);
    {uint8_t nodes[sizeof(obstacle_nodes)],count=obstacle_node_count;
     memcpy(nodes,obstacle_nodes,count);stop_pending=1;
     CHECK(!obstacle_plan_remaining(NODE_C));
     CHECK(obstacle_node_count==count && !memcmp(nodes,obstacle_nodes,count));plant_step();CHECK(!run_active);}
    printf("PASS remaining root states:%u combinations of jobs/nodes/headings/blocked/passed/partial-start links; new context clears cache; manual stop cannot commit a plan\n",cases);
}
