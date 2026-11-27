/* 原启停摆位：A/C->B内完成60mm内移及1030mm横移，随后990mm到E。 */
#define STATION_TEST_MAIN retained_start_station_suite
#include "verify_station_vision.c"

/* 逻辑过点依据较早的共同CPOS时刻；独立车辆已经继续前行，不能把这段时间差当偏航。 */
static double current_position_latency_mm(void)
{
    double f=(-actual_rpm[0]+actual_rpm[1]+actual_rpm[2]-actual_rpm[3])/4;
    double l=(-actual_rpm[0]-actual_rpm[1]+actual_rpm[2]+actual_rpm[3])/4;
    double t=(uint32_t)(clock_ms-run.corner.integrated_ms)/1000.0;
    double speed=hypot(f*3200/(60*CALIBRATED_PPM_Y),l*3200/(60*CALIBRATED_PPM_X));
    return speed*t+0.5*CAR_ROUTE_ACCEL_RPM_PER_S*3200/(60*CALIBRATED_PPM_Y)*t*t;
}

/* 用独立场地坐标核对偏移发生在发车段，且进入B圆角前已经完成。 */
static void start_blend_geometry_test(void)
{
    unsigned plan,j;
    if(!CAR_RUN_CORNER_ENABLED)return;
    for(plan=0;plan<2;plan++) {
        uint32_t deadline;
        double previous_x=2250;
        prepare_arm_auto(plan);send_line(plan ? "runimu C" : "runimu A");deadline=clock_ms+10000;
        while(!run.corner.active && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && run.corner.start_edge==0);
        CHECK(run.corner.spans[0].blend_x==0 && run.corner.spans[0].blend_y==-60);
        CHECK(run.corner.spans[0].blend_length==600 && run.corner.legs[0].length==1030);
        for(j=0;j<=120;j++) {
            float s=600.0f*j/120;
            CornerPoint p=run_corner_point(s);
            double x=2250+p.y;
            double y=(plan ? 2230 : 170)+p.x;
            CHECK(x<=previous_x+0.001 && x>=2190-0.001);
            CHECK(fabs(y-((plan ? 2230 : 170)+(plan ? -s : s)))<0.001);
            CHECK(fabs(p.yaw_cdeg)<0.001);
            if(!j || j==120)CHECK(fabs(p.dy)<0.001);
            if(j==120)CHECK(fabs(x-2190)<0.001);
            previous_x=x;
        }
        CHECK(run.corner.legs[0].y==-60 && run.corner.legs[1].length==990);
        stop_request();
    }
    puts("PASS start blend geometry: both original starts shift X2250->2190 within first600 of1030mm; monotone, zero endpoint lateral slope, fixed yaw, B-E990 retained");
}

static void start_geometry_and_scan_tests(void)
{
    unsigned plan,mode,baud;
    for(plan=0;plan<2;plan++)for(mode=0;mode<3;mode++)for(baud=0;baud<2;baud++) {
        uint32_t deadline,reached_b=0,departed=0;unsigned sent=0,scan_wait=0;
        prepare_arm_auto(plan);imu_period_ms=20;scanner.baud=baud ? 115200u : 9600u;
        physical_yaw=origin_yaw=plan ? -17900 : 17900;sample_yaw(wrapped_yaw(physical_yaw));
        /* init之前的半包不得与本次任务拼接。 */
        scanner.state=SCANNER_READY;scanner_receive((const uint8_t *)"314+231+",8);
        if(mode==2)send_line("task 134+123+314+231");
        send_line(plan ? "runimu C" : "runimu A");deadline=clock_ms+30000;
        CHECK(scanner_init_calls==1 && !scanner.length && scanner.baud==(baud ? 115200u : 9600u));
        CHECK(run.initial_heading==HEAD_X_NEG && run.scan_turn_stage && run.road.count==0);
        while(run.edge<2 && run.state!=RUN_FAULT && clock_ms<deadline) {
            if(mode!=1)scanner_peer_ack(); /* mode1同时覆盖配置超时后接任务。 */
            if(!run.edge && run.velocity_phase==VELOCITY_FORWARD) {
                if(CAR_RUN_CORNER_ENABLED || run.segment==1) {
                    CHECK(run.segment==(CAR_RUN_CORNER_ENABLED ? 0 : 1) && run.action_mm==1030);
                    CHECK(run.travel_axis==BODY_LEFT && run.travel_sign==(plan ? -1 : 1));
                } else CHECK(run.segment==0 && run.action_mm==60 && run.travel_axis==BODY_FORWARD && run.travel_sign==1);
                CHECK(!rotation_starts && run.target_heading==HEAD_X_NEG && !arm_positions);
                if(mode==0 && !sent && run.progress_mm>=150) {
                    scanner_receive((const uint8_t *)"bad-task\r",9);CHECK(!station_task_ready());
                    scanner_receive((const uint8_t *)"134+123+314+231\r",8);CHECK(!station_task_ready());
                    scanner_receive((const uint8_t *)"134+123+314+231\r"+8,8);
                    CHECK(station_task_ready() && !run.job && !run.edge);sent=1;
                }
            }
            plant_step();
            if(strstr(output,"[scan station] B reached;")) {
                reached_b=clock_ms;
                CHECK(run.edge==1);
                CHECK(!CAR_RUN_CORNER_ENABLED || fabs(field_x-2190)<8);
                if(!CAR_RUN_CORNER_ENABLED)CHECK(planned_x==2190 && planned_y==1200);
                if(fabs(field_x-2190-start_forward_bias_x)>=80 || fabs(field_y-1200-start_forward_bias_y)>=4)
                    fprintf(stderr,"START_B continuous=%u plan=%u mode=%u actual=%.3f,%.3f bias=%.3f,%.3f progress=%ld\n",(unsigned)CAR_RUN_CORNER_ENABLED,plan,mode,field_x,field_y,start_forward_bias_x,start_forward_bias_y,(long)run.progress_mm);
                /* 分段执行器没有二维绝对位置闭环；正交漂移沿用整圈80mm模型预算，
                 * 主移动轴仍4mm。连续过点沿用专项圆弧35mm路径预算并区分采样时延。 */
                CHECK(fabs(field_x-2190-start_forward_bias_x)<80 && fabs(field_y-1200-start_forward_bias_y)<4);
            }
            if(strstr(output,"[scan station] B bypassed;")) {
                reached_b=clock_ms;
                CHECK(CAR_RUN_CORNER_ENABLED && mode!=1 && run.edge==1);
                printf("START_CURVE_B plan=%u mode=%u actual=%.3f,%.3f age_ms=%lu latency_mm=%.3f\n",plan,mode,field_x,field_y,(unsigned long)(clock_ms-run.corner.integrated_ms),current_position_latency_mm());
                CHECK(fabs(field_y-1200)<35 && field_x<=2075 && field_x>=2040-current_position_latency_mm()-35);
                CHECK(run.corner.active && (run.forward_cmd_rpm || run.left_cmd_rpm));
            }
            if(run.state==RUN_STATION && !run.job) {
                CHECK(mode==1 && !run_active && !arm_positions);
                if(++scan_wait==1000) {
                    CHECK(scanner.state==SCANNER_FAULT);
                    scanner_receive((const uint8_t *)"134+123+314+231\r",16);CHECK(station_task_ready());
                }
            }
            if(run.job==1 && run.edge==1 && run.velocity_phase==VELOCITY_FORWARD && run.forward_cmd_rpm) {
                if(!departed)departed=clock_ms;
                CHECK(run.action_mm==990 && run.travel_axis==BODY_FORWARD && run.travel_sign==1);
                CHECK(run.target_heading==HEAD_X_NEG && !rotation_starts);
            }
        }
        CHECK(clock_ms<deadline && run.edge==2 && run.state!=RUN_FAULT && reached_b && departed);
        if(CAR_RUN_CORNER_ENABLED) {
            CHECK(run.corner.active && run.distance_mm==2890);
            printf("START_CURVE_E plan=%u mode=%u actual=%.3f,%.3f latency_mm=%.3f\n",plan,mode,field_x,field_y,current_position_latency_mm());
            CHECK(fabs(field_x-1200-start_forward_bias_x)<35);
            CHECK(field_y-start_forward_bias_y<=1085 && field_y-start_forward_bias_y>=1050-current_position_latency_mm()-35);
            CHECK(run.forward_cmd_rpm || run.left_cmd_rpm);
        } else {
            CHECK(planned_x==1200 && planned_y==1200 && run.distance_mm==2080);
            printf("START_SEGMENTED_ERROR plan=%u mode=%u dx_mm=%.3f dy_mm=%.3f; independent plant minus original empirical trim\n",
                plan,mode,field_x-1200-start_forward_bias_x,field_y-1200-start_forward_bias_y);
            CHECK(hypot(field_x-1200-start_forward_bias_x,field_y-1200-start_forward_bias_y)<80);
        }
        CHECK(scanner_init_calls==1 && (mode==1 ? scan_wait==1000 : !scan_wait && departed-reached_b<200));
        printf("START_SCAN continuous=%u plan=%u mode=%u baud=%lu b_ms=%lu depart_ms=%lu gap_ms=%lu no_rotation=1\n",
            (unsigned)CAR_RUN_CORNER_ENABLED,plan,mode,(unsigned long)scanner.baud,(unsigned long)reached_b,(unsigned long)departed,(unsigned long)(departed-reached_b));
        stop_request();
    }
    puts("PASS start geometry/scan: original A/C2250 x; start-road60mm reaches B2190/1200, forward990 reaches E1200/1200; early/late/preloaded QR, two bauds, yaw wrap, no duplicate init or extra B scan wait");
}

static void approach_cancellation_tests(void)
{
    unsigned variant;
    for(variant=0;variant<3;variant++) {
        uint32_t deadline;
        prepare_arm_auto(variant&1u);send_line(variant&1u ? "runimu C" : "runimu A");deadline=clock_ms+10000;
        while(run.progress_mm<100 && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && !run.edge && !run.job && run_active);
        if(variant==0)send_line("!");
        if(variant==1){bus_off=1;plant_step();}
        if(variant==2){motor_flags[2]=11;tick(700);}
        CHECK(!run_active && !station_active() && !station_task_ready());
        scanner_receive((const uint8_t *)"134+123+314+231\r",16);tick(50);
        CHECK(!station_task_ready() && !run.job && !arm_positions);
        stop_request();
    }
    puts("PASS start cancellation: emergency/bus-off/driver fault while scanning cannot advance B/E or accept late task");
}

#ifndef START_SCAN_TEST_MAIN
#define START_SCAN_TEST_MAIN main
#endif
int START_SCAN_TEST_MAIN(void)
{
    start_blend_geometry_test();start_geometry_and_scan_tests();approach_cancellation_tests();scan_during_travel_tests();scan_handoff_tests();
    automatic_obstacle_return_test();automatic_route_tests();
    printf("PASS start scan integration: %u assertions; software simulation only\n",assertions);
    return 0;
}
