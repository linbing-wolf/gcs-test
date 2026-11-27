/* 冷启动只按PC9，复用真实CAN/机构/IMU/扫码和整圈模型。 */
#define STATION_TEST_MAIN station_vision_regression_main
#include "verify_station_vision.c"

static void button_gesture(unsigned hold_ms)
{
    simulated_start_button=1;tick(hold_ms);
    CHECK(!run_active && !start_button.preparing);
    simulated_start_button=0;tick(CAR_START_BUTTON_DEBOUNCE_MS+1u);
}

static void selection_fixture(void)
{
    reset_plant(0,1.15);station_init();diagnostic_auto_reply=0;
}

static void button_selection_tests(void)
{
    unsigned variant;
    selection_fixture();
    simulated_start_button=1;tick(100);
    CHECK(!arm_lock.state && !run_active); /* 按住期间不能先发C。 */
    simulated_start_button=0;tick(CAR_START_BUTTON_DEBOUNCE_MS+1u);
    CHECK(arm_lock.state==1 && strstr(output,"preparing runimu C"));

    selection_fixture();
    simulated_start_button=1;tick(1200);
    CHECK(!arm_lock.state && !run_active);
    simulated_start_button=0;tick(CAR_START_BUTTON_DEBOUNCE_MS+1u);
    CHECK(arm_lock.state==1 && strstr(output,"preparing runimu A"));

    for(variant=0;variant<3;variant++) {
        selection_fixture();
        if(variant==2)clock_ms=UINT32_MAX-500u;
        button_gesture(CAR_START_BUTTON_LONG_PRESS_MS-(variant==0 ? 1u : 0u));
        CHECK(strstr(output,variant==0 ? "preparing runimu C" : "preparing runimu A"));
        tick(100);CHECK(!run_active);
    }
    /* 松开抖动不得重复选择；短于消抖的按下不构成手势。 */
    selection_fixture();simulated_start_button=1;tick(CAR_START_BUTTON_DEBOUNCE_MS-1u);
    simulated_start_button=0;tick(40);CHECK(!arm_lock.state);
    simulated_start_button=1;tick(100);
    simulated_start_button=0;tick(CAR_START_BUTTON_DEBOUNCE_MS-1u);
    CHECK(!arm_lock.state);
    simulated_start_button=1;tick(5);simulated_start_button=0;tick(31);
    CHECK(arm_lock.state==1 && strstr(output,"preparing runimu C"));
    /* !在持续按住和松开消抖时都取消整次手势，之后必须重新按。 */
    for(variant=0;variant<4;variant++) {
        selection_fixture();simulated_start_button=1;tick(variant>=2 ? 5u : 1200u);
        if(variant==1){simulated_start_button=0;tick(10);}
        queue_byte('!');tick(1);if(variant!=3)tick(100);
        simulated_start_button=0;tick(1000);
        CHECK(!arm_lock.state && !run_active && !start_button.preparing);
        CHECK(!strstr(output,"[button] preparing") && !strstr(output,"[button] requesting"));
        button_gesture(100);CHECK(arm_lock.state==1);
    }
    puts("PASS button selection: release-only C/A, threshold-1/threshold, wrap, debounce, bounce and held/release emergency cancellation");
}

static void cold_start_fixture(unsigned plan)
{
    RoadPoint start;
    peer_pending_command=peer_pending_color=0;
    peer_session_active=0;peer_next_ms=0;peer_stack_count=peer_stack_hold=0;
    memset(peer_area_count,0,sizeof(peer_area_count));peer_plate_dx=peer_plate_dy=0;
    reset_plant(0,1.15);station_init();CHECK(station_auto_enabled());
    CHECK(road_point(plan?NODE_C:NODE_A,&start));planned_x=field_x=start.x;planned_y=field_y=start.y;
    initial_field_heading=HEAD_X_NEG;
    CHECK(!arm_enabled() && !arm_pose_known() && !arm_platform_known());
}

static void cold_button_tests(void)
{
    unsigned variant,plan;
    for(plan=0;plan<2;plan++)for(variant=0;variant<6;variant++) {
        uint32_t deadline;
        cold_start_fixture(plan);
        if(variant==1){simulated_start_button=1;start_button_init();tick(500);CHECK(!arm_enabled() && !run_active);
            simulated_start_button=0;tick(CAR_START_BUTTON_DEBOUNCE_MS+1u);}
        if(variant==2){frozen_imu=1;heading_sensor_invalidate(&heading_sensor);}
        if(variant==3)diagnostic_auto_reply=0;
        if(variant==4)clock_ms=UINT32_MAX-100u;
        button_gesture(plan?100u:CAR_START_BUTTON_LONG_PRESS_MS);
        CHECK(arm_lock.state==1 && !arm_enabled() && !run_active);
        if(variant==5){queue_byte('!');tick(1);tick(1000);CHECK(!run_active && !arm_enabled());continue;}
        tick(600);
        if(variant==3){CHECK(!arm_enabled() && !arm_pose_known() && !run_active);diagnostic_auto_reply=1;}
        if(variant==2){CHECK(arm_ready() && arm_parked() && !run_active);frozen_imu=0;}
        deadline=clock_ms+2000u;
        while(!run_active && (int32_t)(clock_ms-deadline)<0)plant_step();
        CHECK(run_active && run.plan==plan && run.station_enabled && arm_ready() && arm_parked());
        CHECK(arm_platform_slot()==1 && arm_enables==2 && !arm_positions && !vision_tx_count);
        CHECK(scanner_init_calls==1);
        tick(600);CHECK(scanner_init_calls==1 && arm_enables==2);
        stop_request();tick(500);CHECK(!run_active && scanner_init_calls==1);
    }
    puts("PASS cold PC9: automatic session/declared setup/A and C routes; boot-held, delayed IMU/wheel feedback, timer wrap and emergency cancellation");
}

static void preparation_cancel_tests(void)
{
    unsigned variant;
    for(variant=0;variant<4;variant++) {
        cold_start_fixture(variant%2u);frozen_imu=1;heading_sensor_invalidate(&heading_sensor);
        button_gesture(variant%2u?100u:CAR_START_BUTTON_LONG_PRESS_MS);tick(600);
        CHECK(arm_ready() && !run_active && start_button.preparing==2);
        if(!variant)stop_request();
        else send_line(variant==1 ? "s" : variant==2 ? "arm stop" : "arm off");
        tick(10);CHECK(!start_button.preparing && !run_active);
        frozen_imu=0;tick(1000);CHECK(!run_active && !scanner_init_calls);
    }
    /* 人工姿态、正在作业、故障后的未知姿态不得被开机约定覆盖。 */
    prepare_arm_auto(1);send_line("arm pose 0 130 0 45");
    button_gesture(100);tick(600);
    {int lift;arm_pose_now(NULL,&lift,NULL,NULL);CHECK(lift==130 && !run_active && !start_button.preparing);}
    cold_start_fixture(1);simulated_start_button=1;tick(CAR_START_BUTTON_DEBOUNCE_MS);
    queue_byte('!');tick(601);simulated_start_button=0;tick(100);
    CHECK(!run_active && !arm_enabled() && !start_button.preparing);
    puts("PASS startup cancellation: !, serial s, arm stop/off cancel IMU-wait; manual pose is retained; cold emergency wins");
}

static void prepared_origin_tests(void)
{
    unsigned plan;
    for(plan=0;plan<2;plan++) {
        uint32_t started;
        prepare_arm_auto(plan);
        button_gesture(plan?100u:CAR_START_BUTTON_LONG_PRESS_MS);
        CHECK(run_active && run.plan==plan && scanner_init_calls==1);
        started=screen_clock.start_ms;
        /* 行驶中再次选择另一启停区不能重启或切换当前路线。 */
        simulated_start_button=1;tick(plan?CAR_START_BUTTON_LONG_PRESS_MS:100u);
        simulated_start_button=0;tick(CAR_START_BUTTON_DEBOUNCE_MS+1u);
        CHECK(run_active && run.plan==plan && scanner_init_calls==1 && screen_clock.start_ms==started);
        stop_request();
    }
    /* 等反馈期间再按另一种手势，不能覆盖已选起点。 */
    for(plan=0;plan<2;plan++) {
        cold_start_fixture(plan);diagnostic_auto_reply=0;
        button_gesture(plan?100u:CAR_START_BUTTON_LONG_PRESS_MS);
        simulated_start_button=1;tick(plan?CAR_START_BUTTON_LONG_PRESS_MS:100u);
        simulated_start_button=0;tick(CAR_START_BUTTON_DEBOUNCE_MS+1u);
        diagnostic_auto_reply=1;tick(600);
        CHECK(run_active && run.plan==plan && scanner_init_calls==1);
        stop_request();
    }
    puts("PASS prepared PC9 A/C: shared runimu entry, active route unchanged, pending origin retained");
}

static void cold_full_route_test(unsigned plan)
{
    uint32_t start,deadline;unsigned offset=0,last_job=0;
    cold_start_fixture(plan);peer_geometry=peer_disc_count=peer_move_count=0;
    start=clock_ms;deadline=clock_ms+300000u;
    button_gesture(plan?100u:CAR_START_BUTTON_LONG_PRESS_MS);
    while(!run_active && clock_ms<deadline)plant_step();
    CHECK(run_active && run.plan==plan && scanner_init_calls==1);
    scanner_receive((const uint8_t *)"134+123+314+231\r\n",17);
    while(run.state!=RUN_DONE && clock_ms<deadline) {
        serve_peer(&offset);scanner_peer_ack();plant_step();
        if(run.job!=last_job){last_job=run.job;arm_positions=0;}
    }
    CHECK(clock_ms<deadline && run.job==8 && run.distance_mm==EXPECTED_FULL_ROUTE_MM);
    CHECK(peer_stack_count==3 && arm_ready() && arm_parked() && arm_platform_slot()==1);
    CHECK(scanner_init_calls==1 && !start_button.preparing);
    CHECK(peer_area_count[1]==1 && peer_area_count[2]==1 && peer_area_count[3]==1 && peer_area_count[4]==1 && peer_area_count[5]==1);
    tick(1000);CHECK(run.state==RUN_DONE && scanner_init_calls==1);
    printf("COLD_PC9_FULL_ROUTE plan=%c elapsed_ms=%lu jobs=%u distance_mm=%lu max_endpoint_model_mm=%.3f\n",
        plan?'C':'A',(unsigned long)(clock_ms-start),run.job,(unsigned long)run.distance_mm,checkpoint_max_error_mm);
}

int main(void)
{
    reset_fixture();
    CHECK(entry_host_start_button_level()==1 && !start_button_pressed());
    simulated_start_button=1;
    CHECK(entry_host_start_button_level()==0 && start_button_pressed());
    puts("PASS PC9 electrical levels: released HIGH, grounded press LOW");
    button_selection_tests();cold_button_tests();preparation_cancel_tests();prepared_origin_tests();pc9_and_scan_init_tests();
    cold_full_route_test(0);cold_full_route_test(1);
    printf("PASS button startup: %lu assertions; host model only\n",(unsigned long)assertions);return 0;
}
