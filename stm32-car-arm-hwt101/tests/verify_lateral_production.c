/* 比赛横移与test l/r的标定一致性；真实入口/编码器反算，主机模型。 */
#define CORNER_ROUTE_TEST_MAIN retained_lateral_corner_suite
#include "verify_corner_route.c"

static int16_t lateral_profile(unsigned diagnostic,int direction)
{
    reset_fixture();clock_ms=40; /* 保持在原50ms单次斜坡时间上限内。 */
    run.travel_axis=BODY_LEFT;run.translation_only=(uint8_t)diagnostic;
    return run_forward_slew((int16_t)(direction*100));
}

static void segment_profile_tests(void)
{
    for(int direction=-1;direction<=1;direction+=2) {
        int16_t test=lateral_profile(1,direction),production=lateral_profile(0,direction);
        CHECK(test==direction*6 && production==test);
    }
    reset_fixture();clock_ms=40;run.travel_axis=BODY_FORWARD;
    CHECK(run_forward_slew(100)==9); /* 9.6RPM按原分段接口截断，前后道路仍240RPM/s。 */
    reset_fixture();clock_ms=40;run.travel_axis=BODY_LEFT;
    run.forward_mrpm=100000;run.profile_sign=1;
    CHECK(run_forward_slew(0)==95); /* 横移使用合入的120RPM/s，40ms从100降到95.2RPM。 */
    puts("PASS segmented: left/right production matches test150RPM/s; forward240 retained, lateral braking120");
}

static void straight_fixture(int yaw,int direction,int lateral)
{
    float angle=yaw*0.0001745329252f;
    reset_fixture();clock_ms=40;
    run.state=RUN_MOVING;run_active=run.imu_mode=run.corner.active=1;
    run.corner.span_count=run.corner.leg_count=1;
    run.corner.spans[0].ux=run.corner.legs[0].ux=direction*(lateral ? cosf(angle) : sinf(angle));
    run.corner.spans[0].uy=run.corner.legs[0].uy=direction*(lateral ? sinf(angle) : -cosf(angle));
    run.corner.spans[0].yaw=(float)yaw;
    run.corner.spans[0].length=run.corner.legs[0].length=run.corner.legs[0].end_s=4000;
    run.corner.total_mm=4000;run.corner.s_mm=100;
}

static void continuous_profile_tests(void)
{
    const int yaws[]={0,9000,-9000,18000};
    for(unsigned h=0;h<4;h++)for(int direction=-1;direction<=1;direction+=2) {
        float desired[3]={0,(float)(direction*100),0};int16_t command[3];
        straight_fixture(yaws[h],direction,1);
        run_corner_slew(desired,command);
        CHECK(command[1]==direction*6 && !command[0] && !command[2]);
        straight_fixture(yaws[h],direction,0);desired[0]=(float)(direction*100);desired[1]=0;
        run_corner_slew(desired,command);CHECK(command[0]==direction*10);
    }
    {
        float desired[3]={0,100,0};int16_t command[3];
        straight_fixture(0,1,1);run.corner.spans[0].blend_length=600;
        run_corner_slew(desired,command);CHECK(command[1]==10);
        straight_fixture(0,1,1);run.corner.spans[0].radius=150;run.corner.spans[0].dyaw=9000;
        run_corner_slew(desired,command);CHECK(command[1]==12);
        straight_fixture(0,1,1);run.corner.reversing=1;
        run_corner_slew(desired,command);CHECK(command[1]==8);
        straight_fixture(0,1,1);desired[1]=0;desired[2]=100;
        run_corner_slew(desired,command);CHECK(command[2]==10); /* 终点纯纠角沿用原240RPM/s。 */
        straight_fixture(0,1,1);desired[1]=100;desired[2]=0;
        run.corner.spans[1]=run.corner.spans[0];run.corner.spans[1].s=100;
        memset(&run.corner.spans[0],0,sizeof(run.corner.spans[0]));
        run.corner.spans[0].length=100;run.corner.spans[0].radius=150;
        run.corner.spans[0].yaw=-9000;run.corner.spans[0].dyaw=9000;
        run.corner.span_count=2;run.corner.s_mm=200;
        run_corner_slew(desired,command);CHECK(command[1]==6); /* 纯横移不被出弯450覆盖。 */
    }
    puts("PASS continuous: four headings, both lateral signs150; forward240, blend240, rotating arc300 and return200 retained");
}

static void continuous_trim_tests(void)
{
    const int yaws[]={0,9000,-9000,18000};
    const int fs[4]={-1,1,1,-1},ls[4]={-1,-1,1,1};
    for(unsigned h=0;h<4;h++)for(int direction=-1;direction<=1;direction+=2) {
        float left=(float)(direction*2000),forward=0;
        float per_m=(float)(direction>0 ? LEFT_FORWARD_MM_PER_M : RIGHT_FORWARD_MM_PER_M);
        float expected_pulses=2*per_m*CALIBRATED_PPM_Y;
        CornerPoint point;
        straight_fixture(yaws[h],direction,1);point=run_corner_point(100);
        run_corner_apply_straight_trim(point,1,&forward,&left);
        CHECK(fabsf(forward*car_road_ppm_y(forward)-expected_pulses)<0.001f);
        run_corner_apply_straight_trim(point,-1,&forward,&left);
        CHECK(fabsf(forward)<0.001f && left==direction*2000);
        {
            const float corrections[]={-80,-1,1,80};
            for(unsigned j=0;j<4;j++) {
                float base=corrections[j],pulses=base*car_road_ppm_y(base)+expected_pulses;
                forward=base;
                run_corner_apply_straight_trim(point,1,&forward,&left);
                CHECK(fabsf(forward*car_road_ppm_y(forward)-pulses)<0.001f);
                run_corner_apply_straight_trim(point,-1,&forward,&left);
                CHECK(fabsf(forward-base)<0.001f);
            }
        }
        /* 注入与test补偿相同的真实轮脉冲；经过生产CPOS反算后仍是纯横移。 */
        clock_ms=120;run.corner.polarity_ready=1;run.corner.integrated_ms=100;
        run.corner.integrated_yaw_cdeg=(float)yaws[h];
        run.corner.measured_s_mm=run.corner.s_mm=2000;
        run.corner.yaw_count=run.corner.yaw_head=2;
        run.corner.yaw_ms[0]=100;run.corner.yaw_ms[1]=120;
        run.corner.yaw[0]=run.corner.yaw[1]=(float)yaws[h];
        for(unsigned wheel=0;wheel<4;wheel++) {
            double pulses=expected_pulses*fs[wheel]+direction*2000.0*CALIBRATED_PPM_X*ls[wheel];
            int64_t counts=(int64_t)llround(pulses*CAR_ENCODER_COUNTS_PER_REV/CAR_POSITION_PULSES_PER_REV);
            run.corner.polarity[wheel]=1;run.corner.previous_ms[wheel]=100;
            run.corner.latest_ms[wheel]=motor_feedback[wheel].position_ms=120;
            run.corner.latest[wheel]=motor_feedback[wheel].position=counts;
        }
        CHECK(run_corner_observe());
        CHECK(fabsf(run.corner.x_mm-run.corner.spans[0].ux*2000)<0.02f);
        CHECK(fabsf(run.corner.y_mm-run.corner.spans[0].uy*2000)<0.02f);
    }
    puts("PASS lateral trim: test9.68 conversion, four headings and both signs; real wheel counts inverse without cancelling feed-forward");
}

static void limits_and_runtime_tests(void)
{
    const unsigned lengths[]={1000,1200,1201,2000};
    for(unsigned i=0;i<4;i++)for(int direction=-1;direction<=1;direction+=2) {
        uint32_t deadline;unsigned maximum=0,limit=i<2 ? 250u : 200u;
        reset_plant(0,1.15);CHECK(start_heading_shift(direction*(int)lengths[i],250));
        CHECK(run.lateral_rpm==limit);deadline=clock_ms+30000;
        while(run_active && clock_ms<deadline) {
            plant_step();
            for(unsigned wheel=0;wheel<4;wheel++) {
                unsigned speed=(unsigned)abs(simulated_wheel_rpm[wheel]);
                CHECK(speed<=limit);if(speed>maximum)maximum=speed;
            }
        }
        CHECK(clock_ms<deadline && run.state==RUN_DONE && abs(run.progress_mm-(int)lengths[i])<=1);
        CHECK(maximum>60); /* 比赛分档没有被测试默认60覆盖。 */
        straight_fixture(0,direction,1);
        run.corner.legs[0].length=run.corner.legs[0].end_s=(float)lengths[i];
        CHECK(run_wheel_limit_rpm()==limit);
    }
    puts("PASS unchanged race speeds:1200mm inclusive250, longer200; actual test l/r segment completion and production path caps");
}

int main(void)
{
#ifdef TEST_TRIM_FIRST
    continuous_trim_tests();
#endif
    segment_profile_tests();continuous_profile_tests();continuous_trim_tests();continuous_lateral_trim_tests();limits_and_runtime_tests();
    printf("PASS lateral production: %u assertions; host model only\n",assertions);
    return 0;
}
