/* 分段比赛换向停车：真实主控、CAN/IMU替身，不连接车辆。 */
#define STATION_TEST_MAIN retained_node_stop_station_suite
#include "verify_station_vision.c"

static void stopped_fixture(uint32_t now)
{
    reset_heading_fixture();clock_ms=now;
    run_active=run.imu_mode=1;run.state=RUN_MOVING;
    run.velocity_phase=VELOCITY_STOP_FORWARD;run.zero_ms=now;
    for(unsigned i=0;i<4;i++) {
        MotorFeedback *f=&motor_feedback[i];
        f->position_valid=f->speed_valid=1;f->status_replies=1;f->status_flags=MOTOR_FLAG_ENABLED;
        f->position_ms=f->speed_ms=f->status_ms=now;
    }
}
static void fresh_at(uint32_t now)
{
    clock_ms=now;
    for(unsigned i=0;i<4;i++)motor_feedback[i].position_ms=motor_feedback[i].speed_ms=motor_feedback[i].status_ms=now;
}

static void node_stop_gate_tests(void)
{
    stopped_fixture(1000);
    motor_feedback[3].speed_rpm=CAR_ROUTE_STOP_RPM;
    fresh_at(1001);CHECK(run_velocity_stopped());
    motor_feedback[3].speed_rpm=CAR_ROUTE_STOP_RPM+1;
    CHECK(!run_velocity_stopped() && !run.stopped_ms);
    motor_feedback[3].speed_rpm=0;
    motor_feedback[3].speed_ms=999;CHECK(!run_velocity_stopped());
    fresh_at(1001);CHECK(run_velocity_stopped() && run.stopped_ms==clock_ms);
    motor_feedback[2].speed_rpm=-CAR_ROUTE_STOP_RPM;CHECK(run_velocity_stopped());
    motor_feedback[2].speed_rpm=-CAR_ROUTE_STOP_RPM-1;
    CHECK(!run_velocity_stopped() && !run.stopped_ms);
    motor_feedback[2].speed_rpm=0;fresh_at(1002);CHECK(run_velocity_stopped() && run.stopped_ms==clock_ms);
    motor_feedback[1].speed_ms=clock_ms-CAR_VELOCITY_FEEDBACK_MS-1;
    CHECK(!run_velocity_stopped());
    stopped_fixture(UINT32_MAX-100);
    fresh_at(98);CHECK(run_velocity_stopped() && run.stopped_ms==98);
    stopped_fixture(1000);run.translation_only=1;CHECK(!run_velocity_stopped());
    fresh_at(1001);CHECK(run_velocity_stopped()); /* 手动单段要求零速命令后的新轮速，不增加200ms。 */
    motor_feedback[0].speed_rpm=CAR_VELOCITY_STOP_RPM+1;CHECK(!run_velocity_stopped());
    stopped_fixture(1000);run.station_shift=1;CHECK(run_velocity_stopped());
    stopped_fixture(1000);run.velocity_phase=VELOCITY_TURN;CHECK(run_velocity_stopped());
    run.remaining_mm=600;
    motor_feedback[0].speed_rpm=CAR_ROUTE_STOP_RPM;CHECK(run_velocity_stopped());
    motor_feedback[0].speed_rpm=CAR_ROUTE_STOP_RPM+1;CHECK(!run_velocity_stopped());
    motor_feedback[0].speed_rpm=0;CHECK(run_velocity_stopped()); /* 转头完成后也先停稳，但不另加200ms。 */
    run.job=2;run.station_align_pending=1;run.remaining_mm=0;
    motor_feedback[0].speed_rpm=CAR_VELOCITY_STOP_RPM+1;CHECK(!run_velocity_stopped());
    stopped_fixture(1000);run.corner.active=1;run.velocity_phase=VELOCITY_FORWARD;
    motor_feedback[0].speed_rpm=CAR_ROUTE_STOP_RPM;CHECK(run_velocity_stopped());
    motor_feedback[0].speed_rpm=CAR_ROUTE_STOP_RPM+1;CHECK(!run_velocity_stopped());
    run.job=FULL_ROUTE_JOBS-1u;run.road.to=RUN_JOB_NODE(run.job);run.road.count=1;
    motor_feedback[0].speed_rpm=CAR_VELOCITY_STOP_RPM+1;CHECK(!run_velocity_stopped());
    stopped_fixture(1000);run.obstacle_phase=2;
    motor_feedback[0].speed_rpm=2;CHECK(!run_velocity_stopped());
    motor_feedback[0].speed_rpm=0;CHECK(run_velocity_stopped());
    CHECK(!velocity_batches && !arm_stops);
    puts("PASS node stop: first qualifying wheel feedback permits handoff, nonzero or old feedback blocks; manual, station and obstacle rules retained");
}

static void departure_feedback_reuse_tests(void)
{
    for(unsigned mode=0;mode<4u;mode++) {
        uint32_t departed,deadline,position_queries,status_requests;
        reset_plant(0,1.15);clock_ms=1000;plant_ms=imu_ms=clock_ms;sample_yaw(0);
        checked_job=FULL_ROUTE_JOBS;
        run.job=2;run.edge=3;run.imu_mode=1;
        run.heading=run.target_heading=HEAD_X_NEG;run.yaw_zero_cdeg=-CAR_MAP_YAW_SIGN*HEAD_X_NEG*9000;
        run.turn_rpm=CAR_TURN_CLOSED_LOOP_RPM;
        for(unsigned i=0;i<MOTOR_COUNT;i++) {
            MotorFeedback *f=&motor_feedback[i];
            f->position_valid=f->speed_valid=1;f->status_replies=1;f->status_flags=MOTOR_FLAG_ENABLED;
            f->position_ms=f->speed_ms=f->status_ms=clock_ms-10u;
            f->position=9876;
        }
        if(mode==1u) {
            motor_feedback[0].speed_ms=clock_ms-CAR_VELOCITY_FEEDBACK_MS-1u;
            motor_feedback[1].status_ms=clock_ms-CAR_VELOCITY_FEEDBACK_MS-1u;
        }
        if(mode==2u) {
            moving_at_stop_motor=1;
            motor_feedback[0].speed_rpm=CAR_VELOCITY_STOP_RPM+1;
            motor_feedback[1].status_flags=MOTOR_FLAG_PROTECTED;
        }
        position_queries=diagnostic_queries;status_requests=status_queries;
        if(mode==3u)missing_feedback_motor=4;
        departed=clock_ms;run_station_handoff();
        if(mode>=2u) {
            deadline=clock_ms+80u;
            while(clock_ms<deadline)plant_step();
            CHECK(run.state==RUN_PREFLIGHT && !velocity_batches);
            moving_at_stop_motor=0;missing_feedback_motor=0;
        }
        deadline=clock_ms+1000u;
        while(run.state==RUN_PREFLIGHT && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && run.state==RUN_ALIGN && !velocity_batches);
        for(unsigned i=0;i<MOTOR_COUNT;i++)
            CHECK(motor_feedback[i].position_valid && motor_feedback[i].position_ms>departed && motor_feedback[i].position!=9876);
        if(mode<2u) {
            CHECK(diagnostic_queries-position_queries==4u+mode);
            CHECK(status_queries-status_requests==mode);
        }
        stop_request();
    }
    puts("PASS departure feedback: four new CPOS, reuse stopped speed/healthy status, reacquire stale or unsafe feedback");
}

static void road_turn_handoff_feedback_tests(void)
{
    stopped_fixture(1000);
    run.state=RUN_ALIGN;run.velocity_phase=VELOCITY_TURN;
    run.remaining_mm=600;run.continuation_mm=500;
    fresh_at(1001);
    run.heading_control.sample_ms=1000;CHECK(!run_road_turn_handoff_stopped());
    run.heading_control.sample_ms=1001;
    motor_feedback[2].speed_ms=1000;CHECK(!run_road_turn_handoff_stopped());
    motor_feedback[2].speed_ms=1001;
    motor_feedback[0].speed_rpm=CAR_ROUTE_STOP_RPM+1;CHECK(!run_road_turn_handoff_stopped());
    motor_feedback[0].speed_rpm=-CAR_ROUTE_STOP_RPM;
    run.heading_control.rate_source=1;run.heading_control.measured_rate_cdeg_s=CAR_ROUTE_TURN_HANDOFF_RATE_CDEG_S+1;
    CHECK(!run_road_turn_handoff_stopped());
    run.heading_control.measured_rate_cdeg_s=0;
    run.heading_control.rate_cdeg_s=-CAR_ROUTE_TURN_HANDOFF_RATE_CDEG_S-1;
    CHECK(!run_road_turn_handoff_stopped());
    run.heading_control.rate_cdeg_s=0;run.velocity_waiting=1;CHECK(!run_road_turn_handoff_stopped());
    run.velocity_waiting=0;CHECK(run_road_turn_handoff_stopped());
    run.heading_control.error_cdeg=CAR_ROUTE_TURN_HANDOFF_CDEG+1;
    CHECK(!run_road_turn_handoff_window());
    run.heading_control.error_cdeg=-CAR_ROUTE_TURN_HANDOFF_CDEG;
    CHECK(run_road_turn_handoff_window()==(CAR_ROUTE_TURN_HANDOFF_CDEG>0));
    run.heading_control.error_cdeg=0;
    CHECK(run_road_turn_handoff_window()==(CAR_ROUTE_TURN_HANDOFF_CDEG>0));
    run.turn_only=1;CHECK(!run_road_turn_handoff_window());run.turn_only=0;
    run.station_align_pending=1;run.edge=3;CHECK(!run_road_turn_handoff_window());
    run.edge=1;CHECK(run_road_turn_handoff_window()==(CAR_ROUTE_TURN_HANDOFF_CDEG>0));
    gyro_drive_feedback_enabled=0;CHECK(!run_road_turn_handoff_window());
    puts("PASS turn handoff gate: post-zero IMU and each wheel, measured/filtered rotation, ACK and scope");
}

static void road_turn_handoff_tests(void)
{
    for(unsigned lateral=0;lateral<2u;lateral++)for(int sign=-1;sign<=1;sign+=2) {
        uint32_t begin,deadline,first_motion_ms=0;
        int started=0,trimmed=0,corrected=0,saw_turn=0;
        const int residual_cdeg=250;
        int carry=CAR_ROUTE_TURN_HANDOFF_CDEG>=residual_cdeg;
        reset_plant((int16_t)(-9000-sign*residual_cdeg),1.15);
        translation_yaw_disturbance=0;checked_job=FULL_ROUTE_JOBS;
        run_active=run.imu_mode=1;run.job=1u;run.edge=1u;
        run.heading=HEAD_Y_POS;run.target_heading=HEAD_X_NEG;
        run.travel_axis=lateral ? BODY_LEFT : BODY_FORWARD;run.travel_sign=1;
        run.remaining_mm=600;run.turn_rpm=CAR_TURN_CLOSED_LOOP_RPM;
        CHECK(road_plan(NODE_B,NODE_E,&run.road));
        begin=clock_ms;run_begin_alignment();
        deadline=clock_ms+5000u;
        while(run_active && !corrected && clock_ms<deadline) {
            plant_step();
            if(run.velocity_phase==VELOCITY_TURN && run.turn_cmd_rpm)saw_turn=1;
            if(run.velocity_phase!=VELOCITY_FORWARD || (!run.forward_cmd_rpm && !run.left_cmd_rpm))continue;
            if(!started) {
                CHECK(run.heading==HEAD_X_NEG);
                CHECK(carry ? sign*run.heading_control.error_cdeg>CAR_HEADING_TOLERANCE_CDEG
                            : abs(run.heading_control.error_cdeg)<=CAR_HEADING_TOLERANCE_CDEG);
                CHECK(heading_wrap_cdeg(run.heading_control.last_yaw_cdeg+run.heading_control.error_cdeg)==-9000);
                first_motion_ms=clock_ms-begin;started=1;
            }
            if(run.turn_cmd_rpm*sign>0)trimmed=1;
            if(abs(run.heading_control.error_cdeg)<=CAR_HEADING_TOLERANCE_CDEG)corrected=1;
        }
        CHECK(started && corrected && run.progress_mm<400);
        CHECK(carry ? trimmed && !saw_turn : saw_turn);
        printf("TURN_HANDOFF_MODEL axis=%s sign=%d enabled=%d first_motion_ms=%lu\n",
               lateral ? "lateral" : "forward",sign,carry,(unsigned long)first_motion_ms);
        stop_request();
    }
    puts("PASS road turn handoff: signed errors in both motion axes, original map yaw retained");
}

static void start_and_b_corner_tests(void)
{
    for(unsigned plan=0;plan<2;plan++) {
        RoadPoint start;uint32_t deadline,last_zero=0;unsigned stops=0,seen=0;
        uint32_t completed_zero[3]={0};
        reset_plant(0,1.15);translation_yaw_disturbance=0;
        CHECK(road_point(plan ? NODE_C : NODE_A,&start));
        planned_x=field_x=start.x;planned_y=field_y=start.y;initial_field_heading=HEAD_X_NEG;
        send_line(plan ? "runimu C" : "runimu A");deadline=clock_ms+60000;
        while(run_active && clock_ms<deadline) {
            if(run.velocity_phase==VELOCITY_FORWARD && run.progress_mm>30
                && (plan ? run.edge==0 && run.segment==1 : run.edge==1 && run.travel_axis==BODY_FORWARD))break;
            if(run.velocity_phase==VELOCITY_STOP_FORWARD)last_zero=run.zero_ms;
            plant_step();CHECK(!run.corner.active);
            if(strstr(output,"[control segment done]")) {
                CHECK(last_zero);
                for(unsigned i=0;i<4;i++) {
                    CHECK(abs(motor_feedback[i].speed_rpm)<=CAR_ROUTE_STOP_RPM);
                    CHECK((int32_t)(motor_feedback[i].speed_ms-last_zero)>=0);
                }
                CHECK(stops<3);completed_zero[stops++]=clock_ms;last_zero=0;
            }
            if(strstr(output,"[route pass]"))CHECK(0); /* 起点折线和B换向都不能不断速穿过。 */
            if(run.velocity_phase==VELOCITY_FORWARD
                && (plan ? run.edge==0 && run.segment==1 && run.left_cmd_rpm : run.edge==1 && run.forward_cmd_rpm))seen=1;
        }
        CHECK(clock_ms<deadline && run_active && stops==(plan ? 1u : (CAR_MAP_SCAN_TOWARD_A_MM ? 3u : 2u)) && seen);
        CHECK(run.target_heading==HEAD_X_NEG && !run.continuation_mm);
        if(plan)CHECK(run.edge==0 && run.segment==1 && run.travel_axis==BODY_LEFT && run.travel_sign==-1);
        else {
            CHECK(completed_zero[0]<completed_zero[1] && run.travel_axis==BODY_FORWARD);
            CHECK(run.road.from==NODE_B && run.road.to==NODE_E);
        }
        stop_request();
        printf("PASS direction change plan=%u: %u measured stop/handoff(s), no arc or mixed-axis blend\n",plan,stops);
    }
}

static void work_turn_policy_tests(void)
{
    for(unsigned batch=0;batch<2;batch++) {
        reset_heading_fixture();run.station_enabled=1;run.job=(uint8_t)(batch ? 5 : 2);
        run.heading=HEAD_X_NEG;run.plan=0;run.edge=(uint8_t)(batch ? 10 : 4);
        CHECK(run_plan_road(run.edge,&run.road));run.segment=(uint8_t)(run.road.count-1u);
        run.action_mm=860;run.progress_mm=855;run.encoder_ready=15;
        CHECK(!run_station_turn_early());
    }
    CHECK(CAR_RUN_CORNER_ENABLED==0);
    CHECK(CAR_ROUTE_NODE_STOP_MS==0);
    CHECK(CAR_LATERAL_SHORT_RPM==250 && CAR_LATERAL_LONG_RPM==200 && CAR_RUN_RPM==250);
    CHECK(LEFT_FORWARD_MM_PER_M==0 && RIGHT_FORWARD_MM_PER_M==0);
    puts("PASS work turns: both rough180 visits cannot start before translation stop; race speed/calibration retained");
}

static void raw_station_departure_tests(void)
{
    for(unsigned plan=0;plan<2;plan++)for(unsigned batch=0;batch<2;batch++) {
        RoadPlan road;
        uint8_t departure_edge=(uint8_t)(batch ? 9 : 3);
        uint32_t deadline;
        prepare_arm_auto(plan);translation_yaw_disturbance=0;
        run.plan=(uint8_t)plan;run.station_enabled=run.imu_mode=1;
        run.job=(uint8_t)(batch ? 4 : 1);run.edge=departure_edge-1u;
        CHECK(run_plan_road(run.edge,&road) && road.to==NODE_F && road.segments[0].mm==-910);
        CHECK(run_plan_road(departure_edge,&road) && road.from==NODE_F && road.segments[0].mm==910);

        /* 模拟视觉已把车移近盘面 80mm，并行交接仍走实际工位离站入口。 */
        field_x=planned_x=1200;field_y=planned_y=210;
        run.edge=departure_edge;run.state=RUN_STATION;run.station_phase=1;
        run.heading=run.target_heading=HEAD_X_NEG;run.turn_rpm=CAR_TURN_CLOSED_LOOP_RPM;
        run.yaw_zero_cdeg=heading_wrap_cdeg(-CAR_MAP_YAW_SIGN*9000);
        CHECK(heading_control_start(&run.heading_control,heading_snapshot(),0,clock_ms));
        run_resume_after_station();CHECK(run_active && run.station_departing);
        checked_job=run.job;
        CHECK(run_plan_road(departure_edge-1u,&road) && road.segments[0].mm==-910);
        CHECK(run_plan_road(departure_edge,&road) && road.segments[0].mm==990);
        CHECK(run_plan_road(batch ? 3u : 9u,&road) && road.segments[0].mm==910);

        deadline=clock_ms+15000;
        while(run_active && run.edge==departure_edge && clock_ms<deadline)plant_step();
        CHECK(run_active && clock_ms<deadline && run.edge==departure_edge+1u);
        CHECK(run.encoder_offset_mm==990 && run.distance_mm==1850);
        CHECK(run.road.from==NODE_E && run.road.to==NODE_D && run.action_mm==860);
        while(run_active && clock_ms<deadline)plant_step();
        if(run.state!=RUN_STATION || clock_ms>=deadline || fabs(field_y-2060.0)>=6.0)
            fprintf(stderr,"F-D plan=%u batch=%u state=%u phase=%u pos=%.2f distance=%ld deadline=%ld %s\n",
                    plan,batch,run.state,run.velocity_phase,field_y,(long)run.progress_mm,(long)(deadline-clock_ms),output);
        CHECK(run.state==RUN_STATION && clock_ms<deadline && fabs(field_y-2060.0)<6.0);
        stop_request();
    }
    puts("PASS raw station both starts and batches: E-F910, F-E990 only after work, E-D860");
}

/* 从视觉偏了50mm的实际暂存坐标起步，核对两批均回到原E线。 */
static void storage_station_departure_tests(void)
{
    for(unsigned avoid=0;avoid<2;avoid++)for(unsigned plan=0;plan<2;plan++)for(unsigned batch=0;batch<2;batch++) {
        const uint8_t fixed_edge=(uint8_t)(batch ? 13u : 7u);
        uint32_t deadline;
        RoadPlan road;
        prepare_arm_auto(plan);translation_yaw_disturbance=0;
        run.plan=(uint8_t)plan;run.station_enabled=run.imu_mode=1;
        run.job=(uint8_t)(batch ? 6u : 3u);run.edge=fixed_edge;
        if(avoid) {
            send_line("obstacle on");
            run.obstacle_enabled=1;
            run.edge=0;obstacle_node_count=2;
            obstacle_nodes[0]=NODE_H;obstacle_nodes[1]=NODE_E;
            run.completed_roads=(uint16_t)((1u<<road_index(NODE_H,NODE_E))
                |(1u<<road_index(NODE_E,NODE_F))|(1u<<road_index(NODE_E,NODE_B)));
            obstacle_inset_node=255u;obstacle_inset_axes=0;
            if(batch)run.blocked_roads=(uint16_t)((1u<<road_index(NODE_A,NODE_F))|(1u<<road_index(NODE_C,NODE_D)));
        }
        int departure=ROAD_STORAGE_LEG_MM+CAR_STORAGE_DEPARTURE_EXTRA_MM
            +(!batch && !STN_FIXED_RING_ARM_ENABLED ? CAR_STORAGE_FIRST_DEPARTURE_EXTRA_MM : 0);
        initial_field_heading=HEAD_Y_POS;field_x=planned_x=CAR_MAP_CENTER_X_MM-departure;field_y=planned_y=1200;
        CHECK(run_plan_road(run.edge,&road) && road.from==NODE_H && road.to==NODE_E && road.length==ROAD_STORAGE_LEG_MM);
        CHECK(road_plan(NODE_E,NODE_H,&road) && road.length==ROAD_STORAGE_LEG_MM);
        run.heading=run.target_heading=HEAD_Y_POS;run.turn_rpm=CAR_TURN_CLOSED_LOOP_RPM;
        run.state=RUN_STATION;run.station_phase=1;
        CHECK(heading_control_start(&run.heading_control,heading_snapshot(),0,clock_ms));
        run_resume_after_station();
        checked_job=FULL_ROUTE_JOBS;
        CHECK(run_active && run.station_departing && run_plan_road(run.edge,&road));
        CHECK(road.from==NODE_H && road.to==NODE_E && road.length==departure && road.segments[0].mm==-departure);
        deadline=clock_ms+15000u;
        while(run_active && clock_ms<deadline && run.road.from!=NODE_E)plant_step();
        CHECK(run_active && clock_ms<deadline && run.road.from==NODE_E);
        CHECK(!run.station_departure_edge);
        if(batch) {
            /* E在同轴不停的途中，实际位置与编码器交接时刻相差一帧。检查名义入段原点和真正停稳点。 */
            CHECK(run.road.to==NODE_B && run.encoder_offset_mm==departure);
            deadline=clock_ms+15000u;
            while(run_active && clock_ms<deadline && run.road.from!=NODE_B)plant_step();
            CHECK(run_active && clock_ms<deadline && run.road.from==NODE_B && fabs(field_x-2190.0)<6.0);
            CHECK(fabs(field_y-1200.0)<6.0);
        } else CHECK(fabs(field_x-CAR_MAP_CENTER_X_MM)<6.0);
        stop_request();
    }
    puts("PASS H departure: two start zones and batches with avoidance on/off; map distance, batch-specific departure compensation, E centerline");
}

static void scanned_return_midpoint_tests(void)
{
    for(unsigned plan=0;plan<2;plan++) {
        RoadPlan road;
        prepare_arm_auto(plan);send_line("task 134+123+314+231");
        CHECK(station_task_ready());
        run.plan=(uint8_t)plan;run.job=7;run.station_enabled=1;
        for(unsigned avoid=0;avoid<2;avoid++) {
            RoadPoint start,goal;
            uint8_t home=plan ? NODE_C : NODE_A;
            unsigned approach=avoid ? 0u : FULL_ROUTE_NODES-3u;
            run.obstacle_enabled=(uint8_t)avoid;run.edge=(uint8_t)approach;
            obstacle_nodes[0]=NODE_E;obstacle_nodes[1]=NODE_B;obstacle_nodes[2]=home;obstacle_node_count=3;
            CHECK(run_plan_road((uint8_t)approach,&road) && road.from==NODE_E && road.to==NODE_B);
            CHECK(road.count==1 && road.length==ROAD_B_E_MM);
            if(avoid) {
                obstacle_road_endpoint(&road,(uint8_t)approach,&start);
                CHECK(start.x==2190 && start.y==1200);
            }
            CHECK(run_plan_road((uint8_t)(approach+1u),&road) && road.from==NODE_B && road.to==home);
            CHECK(road.count==2 && road.length==1030u+CAR_START_INSET_MM);
            CHECK(road.segments[0].axis==BODY_FORWARD && road.segments[1].mm==-CAR_START_INSET_MM);
            CHECK(road_point(home,&goal) && road_point(NODE_B,&start));
            CHECK(start.y+road.segments[0].mm==goal.y);
            if(avoid) {
                obstacle_road_endpoint(&road,(uint8_t)(approach+1u),&start);
                CHECK(start.y==goal.y);
            }
        }
        stop_request();
    }
    puts("PASS scanned return uses B midpoint from E and 1030mm to either home, avoidance on/off");
}

int main(void)
{
    node_stop_gate_tests();departure_feedback_reuse_tests();work_turn_policy_tests();road_turn_handoff_feedback_tests();road_turn_handoff_tests();start_and_b_corner_tests();raw_station_departure_tests();storage_station_departure_tests();
    scanned_return_midpoint_tests();
    printf("PASS stop at direction nodes: %u assertions; host model only\n",assertions);
    return 0;
}
