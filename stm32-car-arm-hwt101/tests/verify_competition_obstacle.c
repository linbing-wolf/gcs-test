/* 比赛入口使用双探头避障，并在绕行后继续作业。 */
#define STATION_TEST_MAIN retained_station_suite
#include "verify_station_vision.c"

static unsigned shots, results, last_sensor;
static int last_range;

static void record_shot(uint8_t sensor) { ++shots; last_sensor=sensor; }
static void record_range(uint8_t sensor,int mm)
{
    CHECK(last_sensor==sensor);
    ++results;
    last_range=mm;
}

static void check_capture(void)
{
    reset_fixture();
    start_scan_test_edge(0,0);
    start_scan_obstacle_poll(0,1,record_shot,record_range);
    CHECK(shots==1 && last_sensor==0 && !results);
    start_scan_test_edge(1,20);
    start_scan_test_edge(0,1180);
    start_scan_obstacle_poll(2,1,record_shot,record_range);
    CHECK(results==1 && last_range==200);
    start_scan_obstacle_poll(69,1,record_shot,record_range);
    CHECK(shots==1);
    start_scan_obstacle_poll(70,1,record_shot,record_range);
    CHECK(shots==2 && last_sensor==1);
    start_scan_obstacle_poll(109,1,record_shot,record_range);
    CHECK(results==1);
    start_scan_obstacle_poll(110,1,record_shot,record_range);
    CHECK(results==2 && last_range==-1);
    start_scan_obstacle_poll(140,1,record_shot,record_range);
    CHECK(shots==3 && last_sensor==0);
    start_scan_obstacle_poll(141,0,record_shot,record_range);
    start_scan_test_edge(1,142000);
    start_scan_test_edge(0,143160);
    start_scan_obstacle_poll(220,0,record_shot,record_range);
    CHECK(shots==3 && results==2);
}

static void check_switch(void)
{
    prepare_arm_auto(0);
    send_line("obstacle on");
    CHECK(obstacle_enabled && strstr(output,"[obstacle] ON"));
    send_line("obstacle off");
    CHECK(!obstacle_enabled && strstr(output,"[obstacle] OFF"));
    send_line("obstacle status");
    CHECK(strstr(output,"fixed competition route"));
    send_line("runimu A");
    CHECK(run_active && !run.obstacle_enabled && !obstacle_io_range_enabled());
    send_line("obstacle on");
    CHECK(!obstacle_enabled && strstr(output,"switch rejected"));
    stop_request();
    send_line("obstacle on");
    CHECK(obstacle_enabled && strstr(output,"[obstacle] ON"));
}

/* 检查实际轮速指令、双轴并行与独立的B点扫码对齐。 */
static void check_launch(unsigned plan,int by_button)
{
    const uint8_t start=plan ? NODE_C : NODE_A;
    const RoadHeading heading=plan ? HEAD_Y_NEG : HEAD_X_NEG;
    RoadPoint origin,goal;
    uint32_t deadline,next_shot;
    uint8_t sensor=0;
    int drove_main=0,drove_blend=0,early_blend=0,inside_before_scan=0,turned_at_b=0;

    if(by_button) {
        reset_plant(0,1.15);
        station_init();
    } else prepare_arm_auto(plan);
    CHECK(road_point(start,&origin));goal=road_scan_waypoint();
    field_x=origin.x;field_y=origin.y;
    initial_field_heading=heading;
    checked_job=FULL_ROUTE_JOBS;
    send_line("obstacle on");
    if(by_button) {
        simulated_start_button=1;
        tick(plan ? 100u : CAR_START_BUTTON_LONG_PRESS_MS);
        CHECK(!run_active);
        simulated_start_button=0;
    } else send_line(plan ? "runimu C" : "runimu A");

    deadline=clock_ms+30000u;
    next_shot=clock_ms;
    while(!run.job && clock_ms<deadline) {
        plant_step();
        if(run.velocity_phase==VELOCITY_TURN && simulated_wheel_rpm[0]
            && simulated_wheel_rpm[0]==simulated_wheel_rpm[1]
            && simulated_wheel_rpm[0]==simulated_wheel_rpm[2]
            && simulated_wheel_rpm[0]==simulated_wheel_rpm[3]) {
            if(!run.station_align_pending || RUN_NODES[run.edge]!=NODE_B)
                fprintf(stderr,"Unexpected turn from %c toward %c part=%u heading=%u target=%u\n",
                    'A'+start,'A'+run.road.to,run.segment+1u,run.heading,run.target_heading);
            CHECK(run.station_align_pending && RUN_NODES[run.edge]==NODE_B);
            turned_at_b=1;
        }
        if(run.corner.active && run.forward_cmd_rpm>0 && run.left_cmd_rpm>0) {
            if(run.progress_mm<20)early_blend=1;
            if(run.progress_mm>80 && run.progress_mm<500)drove_blend=1;
        }
        if(run.road.from==start && run.road.to==NODE_B && run.segment==(CAR_START_INSET_MM>0)
            && run.velocity_phase==VELOCITY_FORWARD && run.progress_mm>160) {
            CHECK(run.heading==heading && run.target_heading==heading && run.travel_sign==1);
            if(CAR_START_INSET_MM) {
                CHECK(abs(run.road.segments[0].mm)==CAR_START_INSET_MM);
                CHECK(hypotf(run.corner.spans[0].blend_x,run.corner.spans[0].blend_y)==CAR_START_INSET_MM);
                CHECK(run.corner.map_origin.x==origin.x);
            } else CHECK(run.road.count==1u && !run.obstacle_launch_blend && run.road.segments[0].axis==BODY_FORWARD);
            CHECK(run.travel_axis==(plan ? BODY_FORWARD : BODY_LEFT));
            CHECK(obstacle_sensor_for_leg()==(plan ? 0 : 1));
            if(run.corner.s_mm>=CAR_START_BLEND_MM && fabs(field_x-goal.x)<10.0 && fabs(field_y-goal.y)>50.0)
                inside_before_scan=1;
            drove_main=1;
        }
        if(clock_ms>=next_shot && obstacle_io_range_enabled()) {
            obstacle_io_shot(sensor);
            obstacle_io_result(sensor,-1);
            sensor^=1u;
            next_shot=clock_ms+70u;
        }
    }
    CHECK(run.job==1u && run.plan==plan && run.initial_heading==heading);
    CHECK(run_active && run.state!=RUN_STATION && !station_task_ready());
    CHECK(drove_main && drove_blend==(CAR_START_INSET_MM>0) && early_blend==drove_blend && turned_at_b==(int)plan);
    CHECK(run.heading==HEAD_X_NEG && scanner_init_calls==1 && inside_before_scan);
    CHECK(heading_wrap_cdeg(run.heading_control.last_yaw_cdeg+run.heading_control.error_cdeg)
        ==(plan ? 9000+CAR_MANUAL_TURN_90_OFFSET_CDEG : 0));
    CHECK(hypot(field_x-goal.x,field_y-goal.y)<18.0);
    deadline=clock_ms+750u;
    while(!run.forward_cmd_rpm && !run.left_cmd_rpm && clock_ms<deadline) {
        plant_step();
        if(clock_ms>=next_shot && obstacle_io_range_enabled()) {
            obstacle_io_shot(sensor);obstacle_io_result(sensor,-1);
            sensor^=1u;next_shot=clock_ms+70u;
        }
    }
    CHECK(clock_ms<deadline && run_active && run.road.from==NODE_B && !station_task_ready());
    printf("PASS launch %c via %s: offset complete before scan waypoint, %s probe and departure before QR is received\n",
        'A'+start,by_button ? "PC9" : "serial",plan ? "front" : "left");
}

static void check_competition_half_turn_trim(void)
{
    for(int sign=-1;sign<=1;sign+=2) {
        const int16_t start=17950;
        const int16_t trim=(int16_t)(sign*2*CAR_MANUAL_TURN_90_OFFSET_CDEG);
        int16_t target;
        uint32_t deadline;
        reset_plant(start,1.15);
        run_active=run.imu_mode=run.start_reference_valid=1;
        run.turn_rpm=CAR_TURN_CLOSED_LOOP_RPM;
        run.heading=HEAD_X_NEG;run.target_heading=HEAD_X_POS;
        /* 给起点正负各一个计数的余角，检查原地半圈两种方向。 */
        run.yaw_zero_cdeg=heading_wrap_cdeg(start-sign-CAR_MAP_YAW_SIGN*HEAD_X_NEG*9000);
        for(unsigned attempt=0;attempt<2;attempt++) {
            run_begin_alignment();
            CHECK(run.heading_control.target_cdeg==sign*17999+trim);
            CHECK(run.yaw_trim_cdeg==0);
        }
        deadline=clock_ms+15000;
        while(run.velocity_phase==VELOCITY_TURN && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && run.heading==HEAD_X_POS && run.yaw_trim_cdeg==trim);
        CHECK(fabs(physical_yaw-start-(sign*17999+trim))<=CAR_HEADING_TOLERANCE_CDEG+3);
        target=heading_wrap_cdeg(start+sign*17999+trim);
        /* 同向重对齐以及工位保持都沿用补偿后绝对目标。 */
        run_begin_alignment();
        CHECK(heading_wrap_cdeg(run.heading_control.last_yaw_cdeg+run.heading_control.error_cdeg)==target);
        run_active=0;run.state=RUN_STATION;run.station_phase=run.station_enabled=1;run.job=2;
        arm_lock.state=2;
        station_hold_update(1);
        CHECK(station_hold.active && station_hold.target_cdeg==target);
        stop_request();
    }
    puts("PASS competition half turn: both directions, one compensation across retries, unchanged target through alignment and station hold");
}

static void check_work_station_arrival_heading(void)
{
    static const uint8_t jobs[]={2u,3u,5u,6u};
    for(unsigned visit=0;visit<sizeof(jobs)/sizeof(jobs[0]);visit++) {
        const uint8_t job=jobs[visit];
        uint8_t node;
        RoadHeading target_heading;
        int16_t initial_error,final_target;
        uint32_t deadline;
        prepare_arm_auto(0);
        physical_yaw=origin_yaw=300;sample_yaw(300);
        run_active=run.imu_mode=run.start_reference_valid=run.station_enabled=run.obstacle_enabled=1;
        run.state=RUN_MOVING;run.job=job;checked_job=FULL_ROUTE_JOBS;
        node=RUN_JOB_NODE(job);
        CHECK(run_station_heading(node,&target_heading));
        run.heading=run.target_heading=run.initial_heading=node==NODE_D
            ? (RoadHeading)((target_heading+3u)%4u) : target_heading;
        run.yaw_zero_cdeg=heading_wrap_cdeg(-CAR_MAP_YAW_SIGN*(int32_t)run.heading*9000);
        obstacle_nodes[0]=NODE_E;obstacle_nodes[1]=node;obstacle_node_count=2;
        CHECK(road_plan(NODE_E,node,&run.road) && run.road.count==1);
        initial_error=heading_wrap_cdeg(run_target_yaw(&run,target_heading)-300);
        run_advance_segment();
        CHECK(run.station_align_pending && run.state==RUN_ALIGN && run.velocity_phase==VELOCITY_TURN);
        CHECK(run.heading_control.target_cdeg==(int32_t)initial_error+run.turn_trim_cdeg);
        final_target=heading_wrap_cdeg(300+run.heading_control.target_cdeg);
        run.turn_rpm=CAR_TURN_CLOSED_LOOP_RPM;
        deadline=clock_ms+25000;
        while(run_active && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && !run_active && run.state==RUN_STATION && !run.station_align_pending);
        CHECK(abs(heading_wrap_cdeg(heading_snapshot().yaw_cdeg-final_target))<=CAR_HEADING_TOLERANCE_CDEG);
    }
    puts("PASS D/H arrival: approach yaw error included in the first turn, alignment before station work");
}

static void check_compensated_heading_comparisons(void)
{
    for(unsigned turn=0;turn<2u;turn++) {
        const int16_t trim=turn ? 38 : 0;
        prepare_rough(0,"task 134+123+314+231");
        run.station_phase=run.start_reference_valid=1;
        run.initial_heading=HEAD_X_NEG;
        run.yaw_zero_cdeg=-9000;
        run.yaw_trim_cdeg=trim;
        sample_yaw((int16_t)(trim+40));
        CHECK(station_chassis_start(100));
        CHECK(heading_wrap_cdeg(run.heading_control.last_yaw_cdeg+run.heading_control.error_cdeg)==trim);
        CHECK(run.start_reference_valid && run.yaw_trim_cdeg==trim && run.yaw_zero_cdeg==-9000);
        {
            char line[140];
            snprintf(line,sizeof(line),"[route yaw] start_yaw_cdeg=-18000 carried_trim_cdeg=%d target_yaw_cdeg=%d error_cdeg=-40",trim,trim);
            imu_status();
            CHECK(strstr(output,line));
        }
        stop_request();
    }

    reset_plant(0,1.15);
    run_active=run.imu_mode=run.start_reference_valid=1;
    run.state=RUN_PREFLIGHT;run.station_departing=run.transition_ready=1;
    run.heading=run.target_heading=HEAD_Y_POS;
    run.yaw_trim_cdeg=38;
    CHECK(heading_control_start(&run.heading_control,heading_snapshot(),0,clock_ms));
    run_begin_alignment();
    CHECK(heading_wrap_cdeg(run.heading_control.last_yaw_cdeg+run.heading_control.error_cdeg)==38);

    run.obstacle_enabled=1;run.state=RUN_MOVING;run.velocity_phase=VELOCITY_FORWARD;
    run.travel_axis=BODY_FORWARD;run.travel_sign=1;run.encoder_ready=ALL_MOTORS_MASK;
    CHECK(road_plan(NODE_E,NODE_D,&run.road));
    obstacle_nodes[0]=NODE_E;obstacle_nodes[1]=NODE_D;obstacle_node_count=2;
    run.action_mm=run.road.length;
    sample_yaw(538);CHECK(obstacle_sensor_for_leg()==0);
    sample_yaw(539);CHECK(obstacle_sensor_for_leg()<0);
    sample_yaw(238);obstacle_capture_ray(0);CHECK(obstacle_ray.valid);
    CHECK(fabsf(obstacle_ray.dy-cosf(238*0.0001745329252f))<0.0001f);
    sample_yaw(239);obstacle_capture_ray(0);CHECK(!obstacle_ray.valid);
    stop_request();
    puts("PASS compensated comparisons: station child and departure reuse keep the start reference; sonar heading gates use it and ray geometry follows measured yaw");
}

static void check_scan_turn_handoff(void)
{
    RoadPoint point;
    uint32_t deadline,next_shot;
    uint8_t sensor=0;
    int reused=0;
    prepare_arm_auto(1);
    physical_yaw=origin_yaw=-8850;sample_yaw(-8850);
    send_line("obstacle on");point=road_scan_waypoint();
    field_x=point.x;field_y=point.y;initial_field_heading=HEAD_X_NEG;
    checked_job=FULL_ROUTE_JOBS;translation_yaw_disturbance=0;
    run.plan=1;run.obstacle_enabled=run.station_enabled=run.imu_mode=run_active=1;
    run.station_align_pending=1;run.heading=HEAD_Y_NEG;run.target_heading=HEAD_X_NEG;
    run.turn_rpm=CAR_TURN_CLOSED_LOOP_RPM;run.edge=1;
    run.completed_roads=(uint16_t)(1u<<road_index(NODE_C,NODE_B));
    obstacle_node_count=3;obstacle_nodes[0]=NODE_C;obstacle_nodes[1]=NODE_B;obstacle_nodes[2]=NODE_E;
    run_begin_alignment();
    deadline=clock_ms+4000u;next_shot=clock_ms;
    while(clock_ms<deadline && !run.forward_cmd_rpm && !run.left_cmd_rpm) {
        plant_step();
        if(strstr(output,"[scan heading reused; trim during road]"))reused=1;
        if(run.velocity_phase==VELOCITY_TURN)CHECK(!run.turn_cmd_rpm);
        if(clock_ms>=next_shot && obstacle_io_range_enabled()) {
            obstacle_io_shot(sensor);obstacle_io_result(sensor,-1);
            sensor^=1u;next_shot=clock_ms+70u;
        }
    }
    if(clock_ms>=deadline || !reused || run.job!=1u || run.road.from!=NODE_B || run.road.to!=NODE_E)
        fprintf(stderr,"SCAN TURN stage=%u reused=%u job=%u from=%u to=%u error=%ld\n",
            run.velocity_phase,reused,run.job,run.road.from,run.road.to,(long)run.heading_control.error_cdeg);
    CHECK(clock_ms<deadline && reused && run.job==1u && run.road.from==NODE_B && run.road.to==NODE_E);
    CHECK(run.heading_control.error_cdeg<-CAR_HEADING_TOLERANCE_CDEG && !run.turning);
    stop_request();
    puts("PASS B scan turn: one stop/feedback handoff; outgoing B-E keeps the residual yaw target");
}

static void check_launch_retreat(unsigned plan,int after_blend,int side_probe)
{
    const uint8_t start=plan ? NODE_C : NODE_A;
    const uint8_t watched=(uint8_t)((plan ? 0u : 1u)^(unsigned)side_probe);
    const int emit_after=after_blend ? 740 : (side_probe ? 100 : 160);
    const int range=side_probe ? 25 : (after_blend ? 50 : 220);
    const int road=road_index(start,NODE_B);
    RoadPoint origin;
    uint32_t deadline,next_shot;
    unsigned injected=0;
    uint8_t sensor=0;

    prepare_arm_auto(plan);
    CHECK(road>=0 && road_point(start,&origin));
    CHECK(origin.x==2190+CAR_START_INSET_MM);
    initial_field_heading=plan ? HEAD_Y_NEG : HEAD_X_NEG;
    checked_job=FULL_ROUTE_JOBS;
    send_line("obstacle on");
    send_line(plan ? "runimu C" : "runimu A");
    deadline=clock_ms+30000u;
    next_shot=clock_ms;
    while(!run.obstacle_phase && !run.job && clock_ms<deadline) {
        plant_step();
        if(clock_ms>=next_shot && obstacle_io_range_enabled()) {
            int inject=(!CAR_START_INSET_MM || (run.corner.active && run.corner.polarity_ready))
                && run.progress_mm>=emit_after && sensor==watched;
            obstacle_io_shot(sensor);
            if(inject) {
                double offset_x=plan ? (sensor ? -OBSTACLE_LEFT_LEFT_MM : 0)
                                     : (sensor ? 0 : -OBSTACLE_FRONT_FORWARD_MM);
                CHECK(obstacle_ray.valid && obstacle_shot_valid);
                CHECK(fabs(obstacle_ray.x-field_x-offset_x)<15.0); /* 渐变中地图 X 不得提前扣满启停偏移。 */
                ++injected;
            }
            obstacle_io_result(sensor,inject ? range : -1);
            sensor^=1u;next_shot=clock_ms+70u;
        }
    }
    CHECK(injected==2u && run.obstacle_phase==1u && obstacle_last.sensor==watched);
    CHECK(obstacle_last.progress_mm>=emit_after-20);
    CHECK((obstacle_last.progress_mm>=CAR_START_BLEND_MM)==after_blend);
    CHECK(run.blocked_roads&(1u<<road));
    CHECK(!obstacle_inset_axes && run.scan_turn_stage==1u);
    while(run.obstacle_phase && clock_ms<deadline)plant_step();
    CHECK(!run.obstacle_phase && run_active && !run.job && !run.corner.active && !run.obstacle_launch_blend);
    CHECK(RUN_NODES[0]==start && RUN_NODES[1]==(plan ? NODE_D : NODE_F));
    CHECK(obstacle_inset_axes==(CAR_START_INSET_MM ? 0u : OBSTACLE_ENTRY_Y));
    if(obstacle_inset_axes)origin.y+=obstacle_entry_y(start);
    CHECK(hypot(field_x-origin.x,field_y-origin.y)<15.0);
    printf("PASS launch retreat %c: %s blend window on %s probe, returned and replanned\n",
        'A'+start,after_blend ? "after" : "during",side_probe ? "entry" : "primary");
}

static void check_scan_exit_front_before_left_ready(void)
{
    RoadPoint origin;
    uint32_t deadline;
    int road=road_index(NODE_B,NODE_E);
    reset_plant(0,1.15);
    send_line("obstacle on");
    CHECK(road>=0);origin=road_scan_waypoint();
    field_x=origin.x;field_y=origin.y;initial_field_heading=HEAD_X_NEG;
    run.heading=HEAD_X_NEG;run.obstacle_enabled=1;
    run.yaw_zero_cdeg=-CAR_MAP_YAW_SIGN*HEAD_X_NEG*9000;
    CHECK(station_command("task 213+213+213+213"));
    run.job=1;checked_job=FULL_ROUTE_JOBS;
    run.turn_rpm=CAR_TURN_CLOSED_LOOP_RPM;run.imu_mode=run_active=1;
    obstacle_node_count=3;obstacle_nodes[0]=NODE_B;obstacle_nodes[1]=NODE_E;obstacle_nodes[2]=NODE_F;
    CHECK(run_road() && run.segment==1u);
    deadline=clock_ms+5000u;
    while(!run.corner.active && clock_ms<deadline)plant_step();
    CHECK(run.corner.active && !run.corner.polarity_ready && !run.progress_mm);
    obstacle_io_shot(0);obstacle_io_result(0,432);
    tick(70);obstacle_io_shot(1);obstacle_io_result(1,-1);
    tick(70);obstacle_io_shot(0);obstacle_io_result(0,432);
    CHECK(run.obstacle_phase==1 && obstacle_entry_samples==2u && run.scan_exit_side_samples==1u);
    encoder_value[0]+=1; /* 停车后单轮 CPOS 变化一个计数，进入沿原轨迹回退。 */
    deadline=clock_ms+5000u;
    while(run.obstacle_phase==1 && clock_ms<deadline)plant_step();
    CHECK(run.obstacle_phase==2 && run.corner.active && !run.corner.scan_return_start_s);
    CHECK(!obstacle_io_range_enabled());
    deadline=clock_ms+2000u;
    while(run.obstacle_phase && clock_ms<deadline)plant_step();
    CHECK(clock_ms<deadline && !run.obstacle_phase && !run.corner.active);
    CHECK(run.blocked_roads&(1u<<road));
    CHECK(run.road.from==NODE_B && (run.road.to==NODE_A || run.road.to==NODE_C));
    CHECK(strstr(output,"[obstacle detour]") && station_task_ready());
    puts("PASS B-E early front obstacle: retrace reaches detour without waiting for a second departure-side sample");
}

static void check_scan_exit(int blocked_sensor,int before_moving,int qr_ready,int return_left_blocked)
{
    RoadPoint origin,goal;
    const int road=road_index(NODE_B,NODE_E);
    uint32_t deadline,next_shot;
    uint8_t sensor=0;
    unsigned samples[2]={0,0},retreat=0,started=0,center_return_seen=0;
    int uses_center=qr_ready && blocked_sensor==0 && !before_moving && !return_left_blocked;
    reset_plant(0,1.15);
    send_line("obstacle on");
    CHECK(road>=0 && road_point(NODE_E,&goal));origin=road_scan_waypoint();
    field_x=origin.x;field_y=origin.y;initial_field_heading=HEAD_X_NEG;
    run.heading=HEAD_X_NEG;run.obstacle_enabled=1;
    run.yaw_zero_cdeg=-CAR_MAP_YAW_SIGN*HEAD_X_NEG*9000;
    if(qr_ready) {
        CHECK(station_command("task 134+123+314+231"));
        CHECK(station_task_ready());
    }
    run.job=1;checked_job=FULL_ROUTE_JOBS;
    run.turn_rpm=CAR_TURN_CLOSED_LOOP_RPM;run.imu_mode=run_active=1;
    if(blocked_sensor==1 || return_left_blocked)
        run.completed_roads|=(uint16_t)(1u<<road_index(NODE_B,NODE_C));
    obstacle_node_count=3;obstacle_nodes[0]=NODE_B;obstacle_nodes[1]=NODE_E;obstacle_nodes[2]=NODE_F;
    CHECK(run_road() && run.segment==1u && run.travel_axis==BODY_FORWARD);
    deadline=clock_ms+30000u;next_shot=clock_ms;
    while((run.road.from==NODE_B || run.obstacle_phase) && clock_ms<deadline) {
        plant_step();
        if(clock_ms>=next_shot && obstacle_io_range_enabled() && run.road.to==NODE_E) {
            int inject=run.corner.active && (!run.obstacle_phase
                ? (int)sensor==blocked_sensor && (before_moving || run.corner.s_mm>35)
                : return_left_blocked && sensor==1u);
            obstacle_io_shot(sensor);
            if(obstacle_shot_valid)++samples[sensor];
            obstacle_io_result(sensor,inject ? 25 : -1);
            sensor^=1u;next_shot=clock_ms+70u;
        }
        if(run.corner.active && !run.obstacle_phase && !started && (run.forward_cmd_rpm || run.left_cmd_rpm)) {
            CHECK(samples[0]>=OBSTACLE_ENTRY_SAMPLES && samples[1]>=OBSTACLE_ENTRY_SAMPLES);
            CHECK(run.forward_cmd_rpm>0 && run.left_cmd_rpm>0);
            started=1;
        }
        if(run.obstacle_phase==2u) {
            retreat=1;
            if(run.corner.scan_return_start_s>0)center_return_seen=1;
        }
        if(!run.obstacle_phase && (retreat || (run.blocked_roads&(1u<<road))))break;
    }
    if(started==(unsigned)before_moving || clock_ms>=deadline)
        fprintf(stderr,"SCAN B-E blocked=%d started=%u retreat=%u state=%u phase=%u from=%u to=%u samples=%u/%u watched=%d "
            "heading=%u target=%u s=%.2f xy=%.2f/%.2f cmd=%d/%d tail=%.1000s\n",
            blocked_sensor,started,retreat,run.state,run.obstacle_phase,run.road.from,run.road.to,
            samples[0],samples[1],obstacle_sensor_for_leg(),run.heading,run.target_heading,
            run.corner.s_mm,run.corner.x_mm,run.corner.y_mm,run.forward_cmd_rpm,run.left_cmd_rpm,
            output+((strlen(output)>1000u) ? strlen(output)-1000u : 0u));
    CHECK(started!=(unsigned)before_moving && clock_ms<deadline);
    if(blocked_sensor<0) {
        CHECK(run.road.from==NODE_E && run.heading==HEAD_X_NEG);
        CHECK(hypot(field_x-goal.x,field_y-goal.y)<10.0);
        CHECK(run.distance_mm==ROAD_B_E_MM+CAR_MAP_SCAN_TOWARD_A_MM);
    } else {
        CHECK(retreat==(unsigned)!before_moving);
        CHECK(RUN_NODES[0]==NODE_B && (RUN_NODES[1]==NODE_A || RUN_NODES[1]==NODE_C));
        CHECK(run.blocked_roads&(1u<<road));
        if(blocked_sensor==1 || return_left_blocked) {
            if(run.completed_roads&(1u<<road_index(NODE_B,NODE_C)))
                fprintf(stderr,"side result sensor=%d special=%d center=%u phase=%u state=%u endpoint=%.1f,%.1f samples=%u remaining=%d\n",
                    blocked_sensor,return_left_blocked,center_return_seen,run.obstacle_phase,run.state,
                    field_x,field_y,run.scan_exit_side_samples,obstacle_shot_remaining);
            CHECK(!(run.completed_roads&(1u<<road_index(NODE_B,NODE_C))));
        }
        CHECK(obstacle_last.from==NODE_B && obstacle_last.to==NODE_E && obstacle_last.sensor==blocked_sensor);
        CHECK(run.segment==0u && run.road.count==1u);
        if(qr_ready)CHECK(station_task_ready());
        CHECK(center_return_seen==(unsigned)(uses_center || return_left_blocked));
        if(uses_center) {
            uint8_t bypass=run.road.to;
            CHECK(hypot(field_x-(origin.x),field_y-(origin.y+CAR_MAP_SCAN_TOWARD_A_MM))<5.0);
            CHECK(run.scan_detour_at_center);
            CHECK(abs(run.road.segments[0].mm)==1030-abs(obstacle_entry_y(bypass)));
            CHECK(road_point(bypass,&goal));
            goal.x-=OBSTACLE_START_INSET_MM;goal.y+=obstacle_entry_y(bypass);
            while(run.road.from==NODE_B && clock_ms<deadline) {
                plant_step();
                if(clock_ms>=next_shot && obstacle_io_range_enabled()) {
                    obstacle_io_shot(sensor);obstacle_io_result(sensor,-1);
                    sensor^=1u;next_shot=clock_ms+70u;
                }
            }
            CHECK(clock_ms<deadline && !run.scan_detour_at_center && run.road.from==bypass);
            CHECK(hypot(field_x-goal.x,field_y-goal.y)<10.0);
        } else {
            CHECK(!run.scan_detour_at_center && hypot(field_x-origin.x,field_y-origin.y)<10.0);
        }
    }
    printf("PASS scan exit B-E: %s\n",blocked_sensor<0 ? "combined motion reaches E"
        : uses_center ? "QR latched, return to old B and reach the mapped bypass endpoint"
        : qr_ready ? "QR latched, left obstacle keeps return on the traced path"
        : before_moving ? "preflight obstacle, no motion before selecting lateral exit"
        : blocked_sensor==0 ? "front obstacle, reverse curve and select lateral exit"
        : "left obstacle, reverse curve and select lateral exit");
}

static void check_home(unsigned plan,int from_b,int known,int scanned)
{
    uint8_t home=plan ? NODE_C : NODE_A;
    RoadPoint start,goal;
    int road=road_index(NODE_B,home);
    int saw_home=0,saw_arc=0,saw_blend=0;
    unsigned home_mm=ROAD_START_TO_B_MM;
    int home_offset=CAR_START_INSET_MM;
    uint32_t deadline,next_shot;
    uint8_t sensor=0;
    if(scanned) {
        prepare_arm_auto(plan);
        send_line("task 134+123+314+231");CHECK(station_task_ready());
        run.station_enabled=1;
    } else reset_plant(0,1.15);
    send_line("obstacle on");
    CHECK(road>=0 && road_point(from_b ? NODE_B : NODE_E,&start) && road_point(home,&goal));
    field_x=start.x;field_y=start.y;initial_field_heading=HEAD_Y_POS;
    run.yaw_zero_cdeg=0;run.heading=HEAD_Y_POS;run.obstacle_enabled=1;
    run.job=FULL_ROUTE_JOBS-1u;checked_job=FULL_ROUTE_JOBS;
    run.turn_rpm=CAR_TURN_CLOSED_LOOP_RPM;run.imu_mode=run_active=1;
    run.completed_roads=(uint16_t)((1u<<road_index(NODE_E,NODE_B))|(known ? (1u<<road) : 0u));
    obstacle_node_count=from_b ? 2u : 3u;
    if(!from_b)obstacle_nodes[0]=NODE_E;
    obstacle_nodes[obstacle_node_count-2u]=NODE_B;obstacle_nodes[obstacle_node_count-1u]=home;
    CHECK(run_road());
    deadline=clock_ms+45000u;next_shot=clock_ms;
    while(run_active && clock_ms<deadline) {
        plant_step();
        if(clock_ms>=next_shot && obstacle_io_range_enabled()) {
            obstacle_io_shot(sensor);obstacle_io_result(sensor,-1);
            sensor^=1u;next_shot=clock_ms+70u;
        }
        if(run.corner.active && run.corner.arc_mm>0)saw_arc=1;
        if(run_active && run.road.from==NODE_B && run.road.to==home
            && run.velocity_phase==VELOCITY_FORWARD) {
            CHECK(!run.segment && run.action_mm==home_mm);
            if(home_offset) {
                CHECK(run.corner.active && run.corner.home_blend);
                const CornerSpan *span=&run.corner.spans[run.corner.span_count-1u];
                CHECK(run.corner.map_origin.y==1200);
                CHECK(span->blend_length==CAR_RETURN_BLEND_MM);
                CHECK(hypotf(span->blend_x,span->blend_y)==home_offset);
                if(run.corner.s_mm>span->s+30 && run.corner.s_mm<span->s+span->blend_length-30
                    && run.forward_cmd_rpm && run.left_cmd_rpm)saw_blend=1;
            } else CHECK(!run.corner.home_blend && run.road.count==1u);
            if(!known && (run.forward_cmd_rpm || run.left_cmd_rpm))
                CHECK(obstacle_entry_samples==OBSTACLE_ENTRY_SAMPLES);
            saw_home=1;
        }
    }
    if(run.state!=RUN_DONE || !saw_home || saw_blend!=(home_offset>0))
        fprintf(stderr,"HOME plan=%u from_b=%d known=%d state=%u job=%u edge=%u phase=%u heading=%u target=%u"
            " samples=%u ready=%u s=%.2f xy=%.2f/%.2f goal=%.2f/%.2f saw=%d/%d %s\n",
            plan,from_b,known,run.state,run.job,run.edge,run.velocity_phase,run.heading,run.target_heading,
            obstacle_entry_samples,run.corner.polarity_ready,run.corner.s_mm,run.corner.x_mm,run.corner.y_mm,
            run.corner.first_mm,run.corner.second_mm,saw_home,saw_blend,output);
    CHECK(run.state==RUN_DONE && saw_home && saw_blend==(home_offset>0));
    CHECK(saw_arc==(!from_b && known));
    CHECK(hypot(field_x-goal.x,field_y-goal.y)<18.0 && peer_wheels_stopped());
    if(!known && home_offset)CHECK(run.heading==(plan ? HEAD_Y_POS : HEAD_X_POS));
    CHECK(run.distance_mm==home_mm+CAR_START_INSET_MM
        +(from_b ? 0u : ROAD_B_E_MM));
    printf("PASS obstacle home %s->%c: %dmm blended, %s road, B=%s\n",
        from_b ? "B" : "E",'A'+home,CAR_START_INSET_MM,known ? "passed" : "new",scanned ? "task latched" : "no task");
}

static void check_outer_home(uint8_t from,uint8_t home,int known)
{
    RoadPoint start,goal;
    uint32_t deadline,next_shot;
    uint8_t sensor=0;
    int saw_blend=0;
    reset_plant(0,1.15);
    send_line("obstacle on");
    CHECK(road_point(from,&start) && road_point(home,&goal) && goal.x==2190+CAR_START_INSET_MM);
    field_x=start.x;field_y=start.y;initial_field_heading=HEAD_X_POS;
    run.heading=HEAD_X_POS;run.yaw_zero_cdeg=-CAR_MAP_YAW_SIGN*HEAD_X_POS*9000;
    run.obstacle_enabled=1;run.job=FULL_ROUTE_JOBS-1u;checked_job=FULL_ROUTE_JOBS;
    run.turn_rpm=CAR_TURN_CLOSED_LOOP_RPM;run.imu_mode=run_active=1;
    run.blocked_roads=(uint16_t)((1u<<road_index(NODE_B,NODE_A))|(1u<<road_index(NODE_B,NODE_C)));
    if(known)run.completed_roads=(uint16_t)(1u<<road_index(from,home));
    CHECK(obstacle_plan_remaining(from));
    CHECK(RUN_NODE_COUNT==2 && RUN_NODES[0]==from && RUN_NODES[1]==home);
    CHECK(run_road());
    deadline=clock_ms+45000u;next_shot=clock_ms;
    while(run_active && clock_ms<deadline) {
        plant_step();
        if(clock_ms>=next_shot && obstacle_io_range_enabled()) {
            obstacle_io_shot(sensor);obstacle_io_result(sensor,-1);
            sensor^=1u;next_shot=clock_ms+70u;
        }
        if(run.corner.active && run.corner.home_blend) {
            const CornerSpan *span=&run.corner.spans[run.corner.span_count-1u];
            CHECK(run.segment==0u && run.action_mm==(unsigned)(goal.x-start.x));
            CHECK(hypotf(span->blend_x,span->blend_y)==abs(goal.y-start.y));
            if(run.forward_cmd_rpm && run.left_cmd_rpm)saw_blend=1;
        }
        if(run_active && run.state==RUN_MOVING && run.velocity_phase==VELOCITY_FORWARD)
            CHECK(run.corner.active && run.corner.home_blend && !run.segment);
    }
    CHECK(run.state==RUN_DONE && saw_blend && peer_wheels_stopped());
    CHECK(hypot(field_x-goal.x,field_y-goal.y)<18.0);
    CHECK(run.distance_mm==(unsigned)(goal.x-start.x+abs(goal.y-start.y)));
    printf("PASS obstacle outer home %c-%c: %dmm blended into the %dmm final leg\n",
        'A'+from,'A'+home,abs(goal.y-start.y),goal.x-start.x);
}

static void check_outer_home_join(uint8_t from,uint8_t middle,uint8_t home,int known_home,int block_incoming)
{
    RoadPoint start,join,goal;
    const int incoming=road_index(from,middle),outgoing=road_index(middle,home);
    uint32_t deadline,next_shot;
    unsigned injected=0,beyond=0,passed=0,stopped=0,retreated=0,replanned=0;
    int saw_incoming=0,saw_offset=0;
    reset_plant(0,1.15);
    send_line("obstacle on");
    CHECK(incoming>=0 && outgoing>=0 && road_point(from,&start) && road_point(middle,&join) && road_point(home,&goal));
    field_x=start.x;field_y=start.y;initial_field_heading=HEAD_X_POS;
    run.heading=HEAD_X_POS;run.yaw_zero_cdeg=-CAR_MAP_YAW_SIGN*HEAD_X_POS*9000;
    run.obstacle_enabled=run.imu_mode=run_active=1;
    run.job=FULL_ROUTE_JOBS-1u;checked_job=FULL_ROUTE_JOBS;
    run.turn_rpm=CAR_TURN_CLOSED_LOOP_RPM;
    if(known_home)run.completed_roads=(uint16_t)(1u<<outgoing);
    CHECK(obstacle_plan_remaining(from));
    CHECK(RUN_NODE_COUNT==3u && RUN_NODES[0]==from && RUN_NODES[1]==middle && RUN_NODES[2]==home);
    CHECK(run_road());
    deadline=clock_ms+90000u;next_shot=clock_ms;
    while(run_active && clock_ms<deadline) {
        plant_step();
        if(run.road.road==incoming && run.state==RUN_MOVING && !run.obstacle_phase
            && (run.forward_cmd_rpm || run.left_cmd_rpm)) {
            if(!saw_incoming) {
                CHECK(!(run.completed_roads&(1u<<incoming)));
                if(!obstacle_close_approach(&run.road))CHECK(obstacle_entry_samples==OBSTACLE_ENTRY_SAMPLES);
            }
            saw_incoming=1;
            CHECK((run.corner.active && run.corner.home_blend)==known_home);
        }
        if(clock_ms>=next_shot && obstacle_io_range_enabled()) {
            int mm=-1,watched=obstacle_sensor_for_leg();
            unsigned sensor=watched>=0 ? (unsigned)watched : 0u;
            if(watched>=0 && run.road.road==incoming && run.heading==HEAD_X_POS
                && !(run.blocked_roads&(1u<<incoming))) {
                int remaining=join.x-(int)field_x;
                if(block_incoming && remaining>285 && remaining<585) {mm=135;injected++;}
                if(known_home && !block_incoming && remaining>25 && remaining<145) {mm=280;beyond++;}
            }
            obstacle_io_shot((uint8_t)sensor);obstacle_io_result((uint8_t)sensor,mm);
            next_shot=clock_ms+35u;
        }
        if(run.obstacle_phase==2u)retreated=1;
        if(retreated && !replanned && !run.obstacle_phase) {
            CHECK(RUN_NODES[0]==from && !(run.completed_roads&(1u<<incoming)));
            CHECK((run.completed_roads&(1u<<outgoing)) && hypot(field_x-start.x,field_y-start.y)<18.0);
            replanned=1;
        }
        if(!run.obstacle_phase && run.road.from==middle && run.road.to==home
            && run.corner.active && run.corner.home_blend) {
            if(run.corner.passed) {
                CHECK((run.completed_roads&(1u<<incoming)) && run.heading==HEAD_X_POS);
                if(!run.zero_ms && (run.forward_cmd_rpm || run.left_cmd_rpm))passed=1;
            }
            if(run.forward_cmd_rpm && run.left_cmd_rpm)saw_offset=1;
        }
        if(saw_incoming && !run.obstacle_phase && hypot(field_x-join.x,field_y-join.y)<30.0
            && run.zero_ms && peer_wheels_stopped())stopped=1;
    }
    CHECK(run.state==RUN_DONE && saw_incoming && peer_wheels_stopped());
    if(block_incoming) {
        CHECK(injected>=2 && retreated && replanned && (run.blocked_roads&(1u<<incoming)));
        CHECK(obstacle_last.from==from && obstacle_last.to==middle && obstacle_last.sensor==0);
        CHECK(obstacle_last.progress_mm+obstacle_last.remaining_mm==join.x-start.x);
    } else {
        CHECK(!retreated && !run.blocked_roads && (passed!=0)==known_home && stopped==!known_home && saw_offset);
        if(known_home)CHECK(beyond>=2);
        CHECK(hypot(field_x-goal.x,field_y-goal.y)<18.0);
    }
    printf("PASS outer home %c-%c-%c: %s\n",'A'+from,'A'+middle,'A'+home,
        block_incoming ? "incoming obstacle triggers retreat and reroute"
        : known_home ? "continuous join; ignores echoes beyond the incoming road" : "unknown home road keeps stop");
}

enum ExitProbeCase { EXIT_CLEAR, EXIT_LEFT, EXIT_NEAR, EXIT_MIXED, EXIT_STALE, EXIT_BLIND, EXIT_RING_PREFIX, EXIT_REPLANNED };

static void check_station_exit_probe(enum ExitProbeCase mode)
{
    unsigned offset=0,work_samples=0,work_shots=0,started=0,reused=0,waited=0;
    uint32_t deadline,next_shot;
    uint8_t predicted=mode==EXIT_BLIND ? NODE_G : (mode==EXIT_REPLANNED || mode==EXIT_LEFT) ? NODE_E : NODE_I;
    uint8_t actual=mode==EXIT_BLIND ? NODE_G : mode==EXIT_LEFT ? NODE_E : NODE_I;
    int toward_i=road_index(NODE_H,NODE_I),toward_g=road_index(NODE_H,NODE_G),toward_e=road_index(NODE_H,NODE_E);
    unsigned expected_reuse=mode==EXIT_CLEAR || mode==EXIT_LEFT || mode==EXIT_NEAR;
    prepare_storage(1,mode==EXIT_RING_PREFIX ? "task 134+123+314+231" : "task 134+132+314+231");
    obstacle_enabled=1;
    memset(&obstacle_station_probe,0,sizeof(obstacle_station_probe));
    run.obstacle_enabled=1;run.edge=0;run.completed_roads=0;checked_job=FULL_ROUTE_JOBS;
    obstacle_node_count=2;obstacle_nodes[0]=NODE_H;obstacle_nodes[1]=predicted;
    run.blocked_roads=(uint16_t)((actual==NODE_G ? 0u : 1u<<toward_g)
        | (actual==NODE_I ? 0u : 1u<<toward_i)
        | ((mode==EXIT_REPLANNED || actual==NODE_E) ? 0u : 1u<<toward_e));
    deadline=clock_ms+90000u;next_shot=clock_ms;
    while(clock_ms<deadline && run.state!=RUN_FAULT) {
        serve_peer(&offset);plant_step();
        if(strstr(output,"[obstacle departure] station samples reused"))reused=1;
        if(mode==EXIT_NEAR && run.state==RUN_WAIT_OBSTACLE)break;
        if(run.job==7u && run.road.from==NODE_H && run.road.to==actual && run.state==RUN_MOVING
            && !run.station_return_mm && obstacle_sensor_for_leg()>=0) {
            if(obstacle_entry_samples<OBSTACLE_ENTRY_SAMPLES)waited=1;
            if(run.forward_cmd_rpm || run.left_cmd_rpm){started=1;break;}
        }
        if(clock_ms>=next_shot && obstacle_io_range_enabled()) {
            int work=run.state==RUN_STATION;
            int sensor=work ? (int)(work_shots++&1u) : obstacle_sensor_for_leg();
            if(sensor>=0) {
                int watched_work=work && sensor==(predicted==NODE_E ? 1 : 0);
                obstacle_io_shot((uint8_t)sensor);
                int hit=watched_work && (mode==EXIT_NEAR || (mode==EXIT_MIXED && (work_samples&1u)));
                obstacle_io_result((uint8_t)sensor,hit ? 100 : -1);
                if(watched_work)work_samples++;
            }
            next_shot=clock_ms+70u;
        }
        if(mode==EXIT_STALE && run.state==RUN_PREFLIGHT && obstacle_station_probe.count)
            obstacle_station_probe.samples[0].ms=clock_ms-OBSTACLE_STATION_SAMPLE_MAX_AGE_MS-1u;
        if(mode==EXIT_REPLANNED && work_samples>=2u && run.state==RUN_STATION)
            run.blocked_roads|=(uint16_t)(1u<<toward_e);
    }
    if(started!=(mode!=EXIT_NEAR) || reused!=expected_reuse || waited==expected_reuse)
        fprintf(stderr,"exit probe case=%d work=%u started=%u reused=%u waited=%u state=%u road=%u-%u prefix=%d samples=%u %s\n",
            mode,work_samples,started,reused,waited,run.state,run.road.from,run.road.to,
            run.station_return_mm,obstacle_station_probe.count,output);
    CHECK(started==(mode!=EXIT_NEAR) && reused==expected_reuse && waited!=expected_reuse);
    if(mode==EXIT_NEAR) {
        CHECK(run.state==RUN_WAIT_OBSTACLE && (run.blocked_roads&(1u<<toward_i)));
        CHECK(obstacle_last.from==NODE_H && obstacle_last.to==NODE_I && obstacle_last.progress_mm==0);
    } else CHECK(!(run.blocked_roads&(1u<<run.road.road)));
    if(mode==EXIT_BLIND)CHECK(!work_samples);
    else CHECK(work_samples>=2u);
    stop_request();
}

static void check_home_blocked(void)
{
    RoadPoint origin,other;
    int road=road_index(NODE_B,NODE_A);
    uint32_t deadline,next_shot;
    uint8_t sensor=0,retreat=0,replanned=0;
    prepare_arm_auto(0);
    CHECK(station_command("task 134+123+314+231") && station_task_ready());
    run.station_enabled=1;
    send_line("obstacle on");
    CHECK(road>=0 && road_point(NODE_B,&origin) && road_point(NODE_C,&other));
    field_x=origin.x;field_y=origin.y;initial_field_heading=HEAD_Y_POS;
    run.heading=HEAD_Y_POS;run.obstacle_enabled=1;
    run.job=FULL_ROUTE_JOBS-1u;checked_job=FULL_ROUTE_JOBS;
    run.turn_rpm=CAR_TURN_CLOSED_LOOP_RPM;run.imu_mode=run_active=1;
    obstacle_node_count=2;obstacle_nodes[0]=NODE_B;obstacle_nodes[1]=NODE_A;
    CHECK(run_road());
    deadline=clock_ms+45000u;next_shot=clock_ms;
    while(run_active && clock_ms<deadline) {
        plant_step();
        if(clock_ms>=next_shot && obstacle_io_range_enabled()) {
            int inject=run.corner.active && run.corner.home_blend && run.road.to==NODE_A
                && run.corner.s_mm>run.corner.total_mm-0.9f*CAR_RETURN_BLEND_MM && sensor==0;
            obstacle_io_shot(sensor);obstacle_io_result(sensor,inject ? 22 : -1);
            sensor^=1u;next_shot=clock_ms+70u;
        }
        if(run.obstacle_phase==2u)retreat=1;
        if(retreat && !replanned && !run.obstacle_phase) {
            CHECK(RUN_NODES[0]==NODE_B && RUN_NODES[RUN_NODE_COUNT-1u]==NODE_C);
            CHECK(hypot(field_x-origin.x,field_y-origin.y)<15.0 && !obstacle_inset_axes);
            replanned=1;
        }
    }
    if(!retreat || !replanned || run.state!=RUN_DONE) {
        size_t length=strlen(output);
        fprintf(stderr,"home obstruction: retreat=%u replanned=%u state=%u phase=%u from=%u to=%u x=%.1f y=%.1f %s\n",
            retreat,replanned,run.state,run.obstacle_phase,run.road.from,run.road.to,field_x,field_y,
            output+(length>1800u ? length-1800u : 0u));
    }
    CHECK(retreat && replanned && run.state==RUN_DONE && (run.blocked_roads&(1u<<road)));
    CHECK(obstacle_last.from==NODE_B && obstacle_last.to==NODE_A && obstacle_last.sensor==0);
    CHECK(hypot(field_x-other.x,field_y-other.y)<20.0);
    puts("PASS obstacle home blend: secondary probe blocks B-A, retraces to B and finishes at C");
}

static void check_storage_map(void)
{
    static const uint8_t adjacent[]={NODE_E,NODE_G,NODE_I};
    static const uint8_t outside_nodes[]={NODE_I,NODE_H,NODE_G};
    unsigned batch,side,direction;
    for(side=0;side<3u;side++) {
        RoadPoint base,detour;
        CHECK(road_point(outside_nodes[side],&base) && base.x==360);
        CHECK(obstacle_map_point(outside_nodes[side],&detour) && detour.x==360 && detour.y==base.y);
    }
    for(batch=0;batch<2u;batch++)for(side=0;side<3u;side++)for(direction=0;direction<2u;direction++) {
        RoadPlan road;
        RoadPoint origin,goal;
        int x,y,total=0;
        reset_plant(0,1.15);
        run.station_enabled=run.obstacle_enabled=1;
        run.job=(uint8_t)((batch ? 6u : 3u)+direction);
        obstacle_node_count=2;
        obstacle_nodes[0]=direction ? NODE_H : adjacent[side];
        obstacle_nodes[1]=direction ? adjacent[side] : NODE_H;
        CHECK(run_plan_road(0,&road));
        CHECK(obstacle_map_point(road.from,&origin) && obstacle_map_point(road.to,&goal));
        x=origin.x;y=origin.y;
        for(unsigned j=0;j<road.count;j++) {
            MotionParameters step=road.segments[j];
            if(step.axis==BODY_LEFT)x-=step.mm;
            else y+=step.mm;
            total+=abs(step.mm);
        }
        CHECK(x==goal.x && y==goal.y && total==road.length);
        CHECK(road.count==1 && road.segments[0].axis==(side ? BODY_FORWARD : BODY_LEFT));
        CHECK(road.length==(side==0 ? 820 : side==1 ? 910 : 860));
    }
    puts("PASS storage map: I/H/G aligned on X=360; both batches and all E/G/I directions");
}

static void check_work_departure_map(void)
{
    static const uint8_t rough_exits[]={NODE_E,NODE_I,NODE_C};
    static const uint8_t storage_exits[]={NODE_E,NODE_G,NODE_I};
    RoadPoint point;
    CHECK(road_point(NODE_D,&point) && point.x==1180 && point.y==2060);
    CHECK(road_point(NODE_E,&point) && point.x==1180 && point.y==1200);
    CHECK(road_point(NODE_F,&point) && point.x==1180 && point.y==290);
    CHECK(road_point(NODE_G,&point) && point.x==360 && point.y==290);
    CHECK(road_point(NODE_H,&point) && point.x==360 && point.y==1200);
    CHECK(road_point(NODE_I,&point) && point.x==360 && point.y==2060);
    for(unsigned storage=0;storage<2u;storage++)for(unsigned batch=0;batch<2u;batch++) {
        const uint8_t *exits=storage ? storage_exits : rough_exits;
        unsigned job=storage ? (batch ? 7u : 4u) : (batch ? 6u : 3u);
        for(unsigned i=0;i<3u;i++) {
            RoadPlan road,nominal;
            RoadPoint start,goal;
            int extra=storage ? ((!batch && !STN_FIXED_RING_ARM_ENABLED) ? 95 : i==0u ? 50 : 0)
                : TEST_ROUGH_CAMERA_APPROACH_MM;
            int dx=0,dy=0,length=0;
            reset_plant(0,1.15);
            run.station_enabled=run.obstacle_enabled=1;run.job=(uint8_t)job;
            obstacle_nodes[0]=storage ? NODE_H : NODE_D;obstacle_nodes[1]=exits[i];obstacle_node_count=2;
            CHECK(run_plan_road(0,&nominal));
            run.station_departure_edge=1;
            CHECK(run_plan_road(0,&road));
            CHECK(road_point(road.from,&start) && road_point(road.to,&goal));
            if(road.to==NODE_G || road.to==NODE_I)goal.x+=CAR_MAP_STORAGE_INSET_MM;
            if(road.to==NODE_C){goal.x-=CAR_START_INSET_MM;goal.y=2060;}
            if(storage)start.x-=extra;
            else start.y+=extra;
            for(unsigned j=0;j<road.count;j++) {
                MotionParameters step=road.segments[j];
                if(step.axis==BODY_LEFT)dx-=step.mm;
                else dy+=step.mm;
                length+=abs(step.mm);
            }
            CHECK(start.x+dx==goal.x && start.y+dy==goal.y && road.length==length);
            CHECK(road.length==nominal.length+extra);
        }
    }
    puts("PASS station departure: original D/H coordinates, compensation only on first outbound edge, all bypass directions");
}

static void check_rough_detour_ring2_join(void)
{
    static const char *orders[]={"123","321"};
    const int via_e=road_index(NODE_D,NODE_E);
    const int via_i=road_index(NODE_D,NODE_I);
    CHECK(via_e>=0 && via_i>=0);
    for(unsigned batch=0;batch<2u;batch++)for(unsigned side=0;side<2u;side++) {
        char task[32];
        unsigned peer_offset=0;
        int return_mm=test_station_move_mm(0,orders[side][2]-'0',2);
        int moving=0;
        float first_lateral=-1.0f;
        uint32_t deadline;
        snprintf(task,sizeof(task),"task 134+%s+314+%s",orders[side],orders[side]);
        prepare_rough(batch,task);
        run.obstacle_enabled=1;
        obstacle_nodes[0]=NODE_D;obstacle_node_count=1;run.edge=0;
        checked_job=FULL_ROUTE_JOBS;
        run.completed_roads|=(uint16_t)(1u<<via_i);
        run.blocked_roads=(uint16_t)(1u<<via_e);
        deadline=clock_ms+150000u;
        while(clock_ms<deadline && run.state!=RUN_FAULT) {
            serve_peer(&peer_offset);
            plant_step();
            if(run.job!=(batch ? 6u : 3u) || !run.corner.active)continue;
            CHECK(run.road.from==NODE_D && run.road.to==NODE_I);
            CHECK(run.corner.prefix_mm==return_mm);
            if(run.forward_cmd_rpm || run.left_cmd_rpm)moving=1;
            if(moving)CHECK(!run.zero_ms);
            if(run.left_cmd_rpm && first_lateral<0.0f)first_lateral=run.corner.s_mm;
            if(!run.station_return_mm)break;
        }
        printf("Rough D-I batch=%u return_mm=%d first_lateral_mm=%.1f\n",batch+1u,return_mm,first_lateral);
        CHECK(clock_ms<deadline && run.state!=RUN_FAULT && moving && !run.station_return_mm);
        CHECK(first_lateral>=137.0f && first_lateral<=143.0f);
    }
    puts("PASS rough D-I: both batches and return directions, continuous ring 2 join near 140 mm");
}

enum MissionObstacle { MISSION_CLEAR, MISSION_BLOCK_LAUNCH, MISSION_BLOCK_SCAN_EXIT };

static void check_mission(unsigned plan,enum MissionObstacle obstacle,int obstacle_after_mm)
{
    static const uint8_t work[7]={NODE_B,NODE_F,NODE_D,NODE_H,NODE_F,NODE_D,NODE_H};
    uint32_t next_shot,deadline;
    unsigned offset=0, sensor=0, material_checks=0, visited=0, last_job=0, retreat=0;
    unsigned start=plan ? NODE_C : NODE_A;
    unsigned overlap_motion=0,road_arc=0;
    float home_angle=ARM_YAW_HOME_DEG*ARM_YAW_DIRECTION+ARM_YAW_ZERO_DEG-ARM_YAW_OFFSET_DEG;
    uint16_t home_yaw_us=(uint16_t)(500.0f+fminf(360.0f,fmaxf(0.0f,home_angle))*2000.0f/360.0f+0.5f);
    unsigned obstacle_job=obstacle==MISSION_BLOCK_SCAN_EXIT ? 1u : 0u;
    uint8_t blocked_from=obstacle_job ? NODE_B : (uint8_t)start;
    uint8_t blocked_to=obstacle_job ? NODE_E : NODE_B;
    int blocked_road=road_index(blocked_from,blocked_to);
    int saw_new_return_road=0,passed_return_join=0;
    int injected=0, saw_other_road=0, raw_departure=0, task_sent=0;

    prepare_arm_auto(plan);
    /* 原时序回归保持无视觉位移输入；实际靠环距离由停车点和整圈按钮模型核对。 */
    peer_storage_offset_mm=peer_storage_first_extra_mm=peer_rough_offset_mm=0;
    initial_field_heading=plan ? HEAD_Y_NEG : HEAD_X_NEG;
    checked_job=FULL_ROUTE_JOBS; /* 固定边号的路线检查由此用例自己的动态断言代替。 */
    send_line("obstacle on");
    send_line(plan ? "runimu C" : "runimu A");
    CHECK(blocked_road>=0 && run_active && run.obstacle_enabled && run.station_enabled);
    next_shot=clock_ms;
    deadline=clock_ms+500000;
    while(run.state!=RUN_DONE && run.state!=RUN_FAULT && clock_ms<deadline) {
        if(run.state==RUN_STATION && run.job<7u) {
            CHECK(RUN_JOB_NODE(run.job)==work[run.job]);
            if(obstacle_io_range_enabled()) {
                RoadPlan exit_road;
                MotionParameters body;
                CHECK(!run_active && !run.station_shift && station_wheels_stopped());
                CHECK(run.edge+1u<RUN_NODE_COUNT && road_plan(work[run.job],RUN_NODES[run.edge+1u],&exit_road));
                CHECK(!((run.completed_roads|run.blocked_roads)&(1u<<exit_road.road)) && !obstacle_close_approach(&exit_road));
                obstacle_adjust_corner(&exit_road);
                CHECK(exit_road.count==1u && road_motion_parameters(exit_road.segments[0],run.heading,&body) && body.mm>0);
            }
            visited|=1u<<run.job;

        }
        if(!task_sent && run.job==1u) {
            static const uint8_t task[]="134+123+314+231\r\n";
            CHECK(run.heading==HEAD_X_NEG && RUN_NODES[run.edge]==NODE_B);
            scanner.state=SCANNER_READY;
            scanner_receive(task,sizeof(task)-1u);
            task_sent=1;
        }
        if(clock_ms>=next_shot && obstacle_io_range_enabled()) {
            int mm=-1;
            if(obstacle!=MISSION_CLEAR && !(run.blocked_roads&(1u<<blocked_road)) && run.job==obstacle_job
                && run.road.road==blocked_road && run.progress_mm>=obstacle_after_mm
                && obstacle_sensor_for_leg()==(int)sensor) {
                mm=220;
                ++injected;
            }
            obstacle_io_shot((uint8_t)sensor);
            obstacle_io_result((uint8_t)sensor,mm);
            sensor^=1u;
            next_shot=clock_ms+70;
        }
        if(run.obstacle_phase==2u)retreat=1;
        if(run.corner.active && run.corner.arc_mm>0 && (run.forward_cmd_rpm || run.left_cmd_rpm)) {
            const CornerLeg *leg=&run.corner.legs[run.corner.leg_index];
            if(leg->segment!=255u && leg->radius>0 && run.corner.s_mm>leg->end_s-leg->radius)road_arc=1;
        }
        if(run.departure_overlap==2u && (run.forward_cmd_rpm || run.left_cmd_rpm)
            && arm_busy() && arm_yaw_us!=home_yaw_us)
            overlap_motion|=1u<<run.job;
        if((run.blocked_roads&(1u<<blocked_road)) && run.job==obstacle_job && run_active && !run.obstacle_phase
            && run.road.road!=blocked_road)saw_other_road=1;
        if(obstacle==MISSION_BLOCK_SCAN_EXIT && run.job==FULL_ROUTE_JOBS-1u && !run.station_shift) {
            if(run.road.from==NODE_G && run.road.to==NODE_F && run.velocity_phase==VELOCITY_FORWARD
                && (run.forward_cmd_rpm || run.left_cmd_rpm)) {
                CHECK(!(run.completed_roads&(1u<<run.road.road))
                    && (run.completed_roads&(1u<<road_index(NODE_F,NODE_A))));
                CHECK(run.corner.active && run.corner.home_blend && !run.zero_ms);
                saw_new_return_road=1;
            }
            if(run.road.from==NODE_F && run.road.to==NODE_A && run.corner.active && run.corner.passed
                && !run.zero_ms && (run.forward_cmd_rpm || run.left_cmd_rpm))passed_return_join=1;
        }
        if(run_active && !run.station_shift && run.road.from==NODE_H && run.road.to==NODE_E
            && (run.job==4u || run.job==7u)) {
            int toward_rings=run.job==4u && !STN_FIXED_RING_ARM_ENABLED ? CAR_STORAGE_FIRST_DEPARTURE_EXTRA_MM : 0;
            int expected=ROAD_STORAGE_LEG_MM+CAR_STORAGE_DEPARTURE_EXTRA_MM+toward_rings;
            CHECK(run.road.count==1u && run.road.segments[0].mm==-expected && run.road.length==expected);
            if(run.corner.active && run.corner.legs[0].edge==run.edge)
                CHECK(run.corner.map_origin.x==1180-expected);
            CHECK(run.station_departure_edge);
        }
        if(run_active && !run.station_shift && run.road.from==NODE_F && run.road.to==NODE_E
            && run.station_departure_edge && run.road.count==1u) {
            CHECK(run.road.segments[0].mm==990u);
            raw_departure|=1<<(run.job==5u);
        }
        {
            unsigned frame=offset;
            serve_peer(&offset);
            while(frame<offset) {
                if(vision_tx_bytes[frame+3u]=='J')++material_checks;
                frame+=vision_tx_bytes[frame+2u]+5u;
            }
        }
        if(run.job!=last_job) {last_job=run.job;arm_positions=0;}
        plant_step();
    }
    if(run.state!=RUN_DONE || (visited&0x7eu)!=0x7eu || material_checks!=6u || peer_stack_count!=3u)fprintf(stderr,"route plan=%u job=%u edge=%u state=%u phase=%u visited=%u task=%d J=%u stack=%u output=%s\n",
        plan,run.job,run.edge,run.state,run.obstacle_phase,visited,task_sent,material_checks,peer_stack_count,output);
    CHECK(run.state==RUN_DONE && !run_active && !run.corner.active && run.job==FULL_ROUTE_JOBS);
    CHECK((visited&0x7eu)==0x7eu && task_sent && material_checks==6u && peer_stack_count==3u);
    CHECK(peer_area_count[1]==1 && peer_area_count[2]==1 && peer_area_count[3]==1
        && peer_area_count[4]==1 && peer_area_count[5]==1);
    CHECK(raw_departure==3 && arm_ready() && arm_parked());

    if((overlap_motion&0xd8u)!=0xd8u || !road_arc)
        fprintf(stderr,"mission %c overlap=%u road_arc=%u\n",'A'+start,overlap_motion,road_arc);
    CHECK((overlap_motion&0xd8u)==0xd8u && road_arc);
    if(obstacle!=MISSION_CLEAR) {
        CHECK(injected>=2 && (retreat!=0)==(obstacle_after_mm>0) && saw_other_road);
        CHECK(run.blocked_roads&(1u<<blocked_road));
        CHECK(obstacle_last.valid && obstacle_last.from==blocked_from && obstacle_last.to==blocked_to
            && obstacle_last.sensor==0);
    } else CHECK(!retreat && !run.blocked_roads);
    CHECK(obstacle_start_node(RUN_NODES[RUN_NODE_COUNT-1u]));
    if(obstacle==MISSION_BLOCK_SCAN_EXIT) {
        CHECK(RUN_NODES[RUN_NODE_COUNT-1u]==NODE_A && saw_new_return_road && passed_return_join);
        printf("PASS competition C with B-E blocked after %d mm: both batches completed; continuous G-F-A return\n",
            obstacle_after_mm);
    } else printf("PASS competition obstacle %c: both batches completed, %s\n",'A'+start,
        obstacle==MISSION_BLOCK_LAUNCH ? "retreated and replanned" : "clear roads");
}

int main(void)
{
    {
        RoadPoint scan=road_scan_waypoint(),center,home;
        RoadPlan road;
        CHECK(road_point(NODE_B,&center) && center.x==2190 && center.y==1200);
        CHECK(scan.x==2190 && scan.y==1200-CAR_MAP_SCAN_TOWARD_A_MM);
        CHECK(road_point(NODE_A,&home) && home.x==2260 && home.y==170);
        CHECK(road_point(NODE_C,&home) && home.x==2260 && home.y==2230);
        CHECK(road_plan(NODE_A,NODE_B,&road) && road.length==ROAD_START_TO_B_MM+CAR_START_INSET_MM);
        run_adjust_scan_waypoint(&road);
        CHECK(road.segments[0].mm==70 && road.segments[1].mm==ROAD_A_SCAN_MM);
        CHECK(road_plan(NODE_C,NODE_B,&road) && road.length==ROAD_START_TO_B_MM+CAR_START_INSET_MM);
        run_adjust_scan_waypoint(&road);
        CHECK(road.segments[0].mm==70 && road.segments[1].mm==-ROAD_C_SCAN_MM);
        CHECK(road_plan(NODE_B,NODE_E,&road) && road.count==1u && road.length==ROAD_B_E_MM);
        run_adjust_scan_waypoint(&road);
        CHECK(road.count==2u && road.length==ROAD_B_E_MM+CAR_MAP_SCAN_TOWARD_A_MM
            && road.segments[0].axis==BODY_FORWARD && road.segments[0].mm==CAR_MAP_SCAN_TOWARD_A_MM
            && road.segments[1].axis==BODY_LEFT && road.segments[1].mm==ROAD_B_E_MM);
    }
    CHECK(CAR_OBSTACLE_DEFAULT_ON==1 && obstacle_enabled==CAR_OBSTACLE_DEFAULT_ON);
    check_capture();
    check_switch();
    check_launch(0,0);
    check_launch(1,0);
    check_launch(0,1);
    check_launch(1,1);
    check_competition_half_turn_trim();
    check_work_station_arrival_heading();
    check_compensated_heading_comparisons();
    check_scan_turn_handoff();
    if(CAR_START_INSET_MM)check_launch_retreat(0,0,1);
    check_launch_retreat(0,0,0);
    if(CAR_START_INSET_MM)check_launch_retreat(1,0,1);
    check_launch_retreat(1,0,0);
    check_launch_retreat(1,1,0);
    for(unsigned plan=0;plan<2;plan++) {
        for(int scanned=0;scanned<=1;scanned++) {
            check_home(plan,1,0,scanned);
            check_home(plan,1,1,scanned);
            check_home(plan,0,1,scanned);
        }
    }
    for(int known=0;known<=1;known++) {
        check_outer_home(NODE_F,NODE_A,known);
        check_outer_home(NODE_D,NODE_C,known);
    }
    check_outer_home_join(NODE_G,NODE_F,NODE_A,1,0);
    check_outer_home_join(NODE_I,NODE_D,NODE_C,1,0);
    check_outer_home_join(NODE_G,NODE_F,NODE_A,0,0);
    check_outer_home_join(NODE_I,NODE_D,NODE_C,0,0);
    check_outer_home_join(NODE_G,NODE_F,NODE_A,1,1);
    check_outer_home_join(NODE_I,NODE_D,NODE_C,1,1);
    check_home_blocked();
    for(enum ExitProbeCase mode=EXIT_CLEAR;mode<=EXIT_REPLANNED;mode++)check_station_exit_probe(mode);
    puts("PASS station exit ultrasound: front and left reuse, confirmed blockage at zero travel, mixed/stale echo, blind direction, ring return and reroute");
    check_work_departure_map();
    check_storage_map();
    check_rough_detour_ring2_join();
    check_scan_exit_front_before_left_ready();
    check_scan_exit(-1,0,0,0);
    check_scan_exit(0,0,0,0);
    check_scan_exit(1,0,0,0);
    check_scan_exit(1,1,0,0);
    check_scan_exit(0,0,1,0);
    check_scan_exit(1,0,1,0);
    check_scan_exit(0,0,1,1);
    check_mission(0,MISSION_CLEAR,0);
    check_mission(1,MISSION_BLOCK_LAUNCH,160);
    check_mission(1,MISSION_BLOCK_SCAN_EXIT,0);
    check_mission(1,MISSION_BLOCK_SCAN_EXIT,240);
    printf("PASS competition obstacle: %u assertions; host models only\n",assertions);
    return 0;
}
