/* 底盘速度输入边界、混控叠加及真实运动入口；不代替实车跟随验证。 */
#define STATION_TEST_MAIN retained_rpm_limit_station_suite
#include "verify_station_vision.c"

static unsigned observe_wheels(unsigned maximum)
{
    unsigned i;
    for(i=0;i<4;i++) {
        unsigned rpm=(unsigned)abs(simulated_wheel_rpm[i]);
        CHECK(rpm<=200);
        if(rpm>maximum)maximum=rpm;
    }
    return maximum;
}

static void input_and_position_tests(void)
{
    static const char *modes[]={"test","turn","nav"};
    unsigned mode,direction,i;
    for(mode=0;mode<3;mode++) {
        reset_plant(0,1.15);send_line(modes[mode]);send_line("rpm 200");
        CHECK(mode==0 ? test_rpm==200 : mode==1 ? turn_rpm==200 : speed_rpm==200);
        send_line("rpm 201");CHECK(strstr(output,"[rejected"));
        send_line("rpm 250");CHECK(strstr(output,"[rejected"));
        CHECK(mode==0 ? test_rpm==200 : mode==1 ? turn_rpm==200 : speed_rpm==200);
        send_line("rpm 150");CHECK(strstr(output,"[accepted"));
        if(mode==2){send_line("rpm 0");CHECK(speed_rpm==0);}
    }
    for(direction=0;direction<2;direction++) {
        MotionParameters p={BODY_FORWARD,100,201,CAR_MOTION_ACCELERATION};
        reset_plant(0,1.15);CHECK(!motion_start(&drive,p,clock_ms) && !position_count);
        CHECK(!nav_start(&nav,NODE_E,201));CHECK(nav_start(&nav,NODE_E,200) && nav.rpm==200);
        reset_plant(0,1.15);send_line("test");send_line("dist 2000");send_line("rpm 200");send_line(direction ? "b" : "f");
        CHECK(position_count==4);
        for(i=0;i<position_count;i++)CHECK(sent[i].rpm<=200);
        CHECK(sent[0].rpm==200);
    }
    puts("PASS speed inputs: test/turn/nav accept200 and lower, reject201/250; nav0 retained; position f/b <=200");
}

static void mixed_output_tests(void)
{
    static const int16_t components[]={-200,-60,0,60,200};
    unsigned f,l,t,i;int16_t wheels[4];
    for(f=0;f<5;f++)for(l=0;l<5;l++)for(t=0;t<5;t++) {
        reset_plant(0,1.15);run_active=run.imu_mode=1;
        CHECK(run_velocity_send(components[f],components[l],components[t]));
        heading_control_translate_wheels(components[f],components[l],components[t],wheels);
        for(i=0;i<4;i++)CHECK(abs(wheels[i])<=200 && simulated_wheel_rpm[i]==wheels[i]);
    }
    puts("PASS mixed outputs:125 forward/lateral/yaw combinations, including simultaneous200RPM components, final wheel commands <=200");
}

static void moving_entry_tests(void)
{
    unsigned direction,plan;
    for(direction=0;direction<2;direction++) {
        uint32_t deadline;unsigned maximum=0;
        reset_plant(0,1.15);send_line("test");send_line("dist 2000");send_line("rpm 200");send_line(direction ? "r" : "l");
        CHECK(run_active && run.lateral_rpm==200);deadline=clock_ms+30000;
        while(run_active && clock_ms<deadline){plant_step();maximum=observe_wheels(maximum);}
        CHECK(clock_ms<deadline && run.state==RUN_DONE && abs(run.progress_mm-2000)<=CAR_DISTANCE_TOLERANCE_MM);
        CHECK(maximum==200);
        printf("PASS lateral %s200: commanded max=%u; encoder/IMU completion only\n",direction ? "right" : "left",maximum);
        reset_plant(0,1.15);send_line("turn");send_line("rpm 200");send_line(direction ? "right180" : "left180");
        CHECK(run_active && run.turn_rpm==200);deadline=clock_ms+30000;maximum=0;
        while(run_active && clock_ms<deadline){plant_step();maximum=observe_wheels(maximum);}
        CHECK(clock_ms<deadline && run.state==RUN_DONE && maximum==200);
    }
    for(plan=0;plan<2;plan++) {
        unsigned offset=0,maximum=0;uint32_t deadline;
        prepare_arm_auto(plan);send_line("task 134+123+314+231");send_line(plan ? "runimu C" : "runimu A");
        CHECK(run_active && run.turn_rpm==200);deadline=clock_ms+4000;
        while(clock_ms<deadline){serve_peer(&offset);plant_step();maximum=observe_wheels(maximum);}
        CHECK(run_active && velocity_batches && maximum==200);stop_request();
        printf("PASS automatic %c launch: max wheel command=%u, manual ! retained\n",plan ? 'C' : 'A',maximum);
    }
}

int main(void)
{
    CHECK(CAR_RUN_RPM==200 && CAR_LATERAL_MAX_RPM==200 && CAR_TURN_CLOSED_LOOP_RPM==200 && CAR_WHEEL_MAX_RPM==200);
    CHECK(CAR_DRIVE_DECEL_RPM_PER_S==200 && CAR_DRIVE_BRAKE_LOOKAHEAD_MS==200 && CAR_ROUGH_BRAKE_LOOKAHEAD_MS==200);
    CHECK(LEFT_FORWARD_MM_PER_M==43 && RIGHT_FORWARD_MM_PER_M==10);
    input_and_position_tests();mixed_output_tests();moving_entry_tests();
    printf("PASS chassis RPM limit: %u assertions; software command limits only, no physical tracking acceptance\n",assertions);
    return 0;
}
