/* 验证真实入口及连续路径中的横移分档；不把软件限速当作实车跟随验收。 */
#define STATION_TEST_MAIN retained_lateral_tier_station_suite
#include "verify_station_vision.c"

static unsigned tier_observe(unsigned maximum,unsigned limit)
{
    unsigned i;
    for(i=0;i<4;i++) {
        unsigned value=(unsigned)abs(simulated_wheel_rpm[i]);
        CHECK(value<=limit);if(value>maximum)maximum=value;
    }
    return maximum;
}

static void tier_inputs_and_position(void)
{
    static const unsigned lengths[]={1000,1200,1201,2000};unsigned i,axis,sign;
    CHECK(car_lateral_limit_rpm(1200)==250 && car_lateral_limit_rpm(1201)==200);
    for(i=0;i<4;i++)for(axis=0;axis<2;axis++)for(sign=0;sign<2;sign++) {
        unsigned wheel,limit=axis ? car_lateral_limit_rpm(lengths[i]) : 250;
        MotionParameters p={(BodyAxis)axis,(int16_t)(sign ? -(int)lengths[i] : (int)lengths[i]),250,CAR_MOTION_ACCELERATION};
        reset_plant(0,1.15);CHECK(motion_start(&drive,p,clock_ms));
        CHECK(drive.parameters.rpm==limit && position_count==4);
        for(wheel=0;wheel<4;wheel++)CHECK(sent[wheel].rpm<=limit);
        if(axis) {
            reset_plant(0,1.15);CHECK(start_heading_shift(p.mm,250));
            CHECK(run.lateral_rpm==limit);stop_request();
            reset_plant(0,1.15);CHECK(start_heading_shift(p.mm,150));
            CHECK(run.lateral_rpm==150);stop_request();
        }
    }
    reset_plant(0,1.15);send_line("test");send_line("rpm 250");CHECK(test_rpm==250);
    send_line("rpm 251");CHECK(strstr(output,"[rejected") && test_rpm==250);
    reset_plant(0,1.15);send_line("nav");send_line("rpm 250");CHECK(speed_rpm==250);
    send_line("rpm 251");CHECK(strstr(output,"[rejected"));send_line("rpm 0");CHECK(!speed_rpm);
    reset_plant(0,1.15);send_line("turn");send_line("rpm 300");CHECK(turn_rpm==300);
    send_line("rpm 301");CHECK(strstr(output,"[rejected"));
    puts("PASS tier boundary1200/1201, both directions, position-mode axis caps, lower requests, translation/nav250 and existing turn300");
}

static void tier_runtime(void)
{
    static const unsigned lengths[]={1000,1200,1201,2000};unsigned i,direction,axis;
    for(i=0;i<4;i++)for(direction=0;direction<2;direction++) {
        char command[32];uint32_t deadline;unsigned maximum=0,limit=car_lateral_limit_rpm(lengths[i]);
        reset_plant(0,1.15);translation_yaw_disturbance=0;
        send_line("test");snprintf(command,sizeof(command),"dist %u",lengths[i]);send_line(command);send_line("rpm 250");send_line(direction ? "r" : "l");
        CHECK(run_active && run.lateral_rpm==limit);deadline=clock_ms+30000;
        while(run_active && clock_ms<deadline){plant_step();maximum=tier_observe(maximum,limit);}
        CHECK(clock_ms<deadline && run.state==RUN_DONE && abs(run.progress_mm-(int)lengths[i])<=1);
        printf("TIER_LATERAL right=%u distance_mm=%u configured_limit=%u max_wheel_command=%u\n",direction,lengths[i],limit,maximum);
    }
    for(axis=0;axis<2;axis++)for(direction=0;direction<2;direction++) {
        uint32_t deadline;unsigned maximum=0;int distance=axis ? 2000 : 800;
        reset_plant(0,1.15);translation_yaw_disturbance=0;
        CHECK(start_heading_shift(direction ? -distance : distance,250));run.travel_axis=BODY_FORWARD;
        CHECK(run_wheel_limit_rpm()==250);deadline=clock_ms+30000;
        while(run_active && clock_ms<deadline){plant_step();maximum=tier_observe(maximum,250);}
        CHECK(clock_ms<deadline && run.state==RUN_DONE);
        if(axis)CHECK(maximum>200);
        printf("TIER_FORWARD backward=%u distance_mm=%d limit=250 max_wheel_command=%u\n",direction,distance,maximum);
    }
}

static void tier_mix_limits(void)
{
    static const int16_t values[]={-250,-60,0,60,250};unsigned mode,f,l,t;
    for(mode=0;mode<4;mode++)for(f=0;f<5;f++)for(l=0;l<5;l++)for(t=0;t<5;t++) {
        unsigned limit=mode==0 ? 200 : mode==3 ? 300 : 250;
        reset_plant(0,1.15);run_active=run.imu_mode=1;
        run.travel_axis=mode<2 ? BODY_LEFT : BODY_FORWARD;run.lateral_rpm=(uint16_t)limit;
        if(mode==3)run.turn_only=1;
        CHECK(run_wheel_limit_rpm()==limit && run_velocity_send(values[f],values[l],values[t]));
        (void)tier_observe(0,limit);
    }
    puts("PASS mixed wheel saturation:500 combinations; long lateral200, short lateral/forward250, turn300");
}

static void tier_route_continuity(void)
{
    unsigned plan,i;
    for(plan=0;plan<2;plan++) {
        reset_plant(0,1.15);run_active=run.imu_mode=1;run.station_enabled=1;run.plan=(uint8_t)plan;
        run.job=2;run.edge=3;run.heading=run.target_heading=HEAD_X_NEG;
        CHECK(road_plan(NODE_F,NODE_E,&run.road));CHECK(run_segment());
        CHECK(run.lateral_rpm==200 && run.remaining_mm+run.continuation_mm==1620);
        run.action_mm=810;run.progress_mm=815;CHECK(run_pass_straight());
        CHECK(run.edge==4 && run.progress_mm==5 && run.lateral_rpm==200);
        run.turning=0;run.progress_mm=805;CHECK(run_wheel_limit_rpm()==200);
        if(!CAR_RUN_CORNER_ENABLED)continue;
        reset_plant(0,1.15);run.plan=(uint8_t)plan;run.edge=13;run.job=7;run.heading=HEAD_Y_POS;run.station_enabled=1;
        CHECK(run_corner_plan());run.corner.active=1;
        for(i=0;i<2;i++) {
            run.corner.s_mm=i ? run.corner.legs[0].end_s+100 : 100;
            CHECK(run_wheel_limit_rpm()==200);
        }
        run.corner.s_mm=run.corner.total_mm-10;CHECK(run_wheel_limit_rpm()==250);
        printf("TIER_HOME plan=%u H-E-B long lateral200; B-home forward/backward250\n",plan);
        reset_plant(0,1.15);run.plan=(uint8_t)plan;run.edge=0;run.job=0;run.heading=HEAD_X_NEG;run.station_enabled=1;
        CHECK(run_corner_plan());run.corner.active=1;run.corner.s_mm=100;CHECK(run_wheel_limit_rpm()==250);
        printf("TIER_START plan=%u A/C-B short lateral250\n",plan);
    }
    puts("PASS automatic F-E-D1620 holds long tier across E and near destination; A/C short start and home direction switch");
}

int main(void)
{
    CHECK(CAR_RUN_RPM==250 && CAR_LATERAL_SHORT_RPM==250 && CAR_LATERAL_LONG_RPM==200 && CAR_LATERAL_SHORT_MM==1200);
    CHECK(LEFT_FORWARD_MM_PER_M==30 && RIGHT_FORWARD_MM_PER_M==10 && CAR_DRIVE_DECEL_RPM_PER_S==200);
    tier_inputs_and_position();tier_runtime();tier_mix_limits();tier_route_continuity();
    printf("PASS lateral speed tiers: %u assertions; software limits/model completion only, not physical acceptance\n",assertions);
    return 0;
}
