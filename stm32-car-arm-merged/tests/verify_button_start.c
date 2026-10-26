/* 冷启动只按PC9，复用真实CAN/机构/IMU/扫码和整圈模型。 */
#define STATION_TEST_MAIN station_vision_regression_main
#include "verify_station_vision.c"

static void cold_start_fixture(void)
{
    RoadPoint start;
    peer_pending_command=peer_pending_color=0;
    peer_session_active=0;peer_next_ms=0;peer_stack_count=peer_stack_hold=0;
    memset(peer_area_count,0,sizeof(peer_area_count));peer_plate_dx=peer_plate_dy=0;
    reset_plant(0,1.15);station_init();CHECK(station_auto_enabled());
    CHECK(road_point(NODE_C,&start));planned_x=field_x=start.x;planned_y=field_y=start.y;
    initial_field_heading=HEAD_X_NEG;
    CHECK(!arm_enabled() && !arm_pose_known() && !arm_platform_known());
}

static void cold_button_tests(void)
{
    unsigned variant;
    for(variant=0;variant<6;variant++) {
        uint32_t deadline;
        cold_start_fixture();
        if(variant==1){simulated_start_button=1;start_button_init();tick(500);CHECK(!arm_enabled() && !run_active);
            simulated_start_button=0;tick(CAR_START_BUTTON_DEBOUNCE_MS+1u);}
        if(variant==2){frozen_imu=1;heading_sensor_invalidate(&heading_sensor);}
        if(variant==3)diagnostic_auto_reply=0;
        if(variant==4)clock_ms=UINT32_MAX-100u;
        simulated_start_button=1;tick(CAR_START_BUTTON_DEBOUNCE_MS+1u);
        CHECK(arm_lock.state==1 && !arm_enabled() && !run_active);
        if(variant==5){queue_byte('!');tick(1);tick(1000);CHECK(!run_active && !arm_enabled());continue;}
        tick(600);
        if(variant==3){CHECK(!arm_enabled() && !arm_pose_known() && !run_active);diagnostic_auto_reply=1;}
        if(variant==2){CHECK(arm_ready() && arm_parked() && !run_active);frozen_imu=0;}
        deadline=clock_ms+2000u;
        while(!run_active && (int32_t)(clock_ms-deadline)<0)plant_step();
        CHECK(run_active && run.plan==1 && run.station_enabled && arm_ready() && arm_parked());
        CHECK(arm_platform_slot()==1 && arm_enables==2 && !arm_positions && !vision_tx_count);
        CHECK(scanner_init_calls==1);
        tick(600);CHECK(scanner_init_calls==1 && arm_enables==2);
        stop_request();tick(500);CHECK(!run_active && scanner_init_calls==1);
    }
    puts("PASS cold PC9: automatic session/declared setup/C route; boot-held, hold, delayed IMU/wheel feedback, timer wrap and emergency cancellation");
}

static void preparation_cancel_tests(void)
{
    unsigned variant;
    for(variant=0;variant<4;variant++) {
        cold_start_fixture();frozen_imu=1;heading_sensor_invalidate(&heading_sensor);
        simulated_start_button=1;tick(CAR_START_BUTTON_DEBOUNCE_MS+601u);
        CHECK(arm_ready() && !run_active && start_button.preparing==2);
        if(!variant)stop_request();
        else send_line(variant==1 ? "s" : variant==2 ? "arm stop" : "arm off");
        tick(10);CHECK(!start_button.preparing && !run_active);
        frozen_imu=0;tick(1000);CHECK(!run_active && !scanner_init_calls);
    }
    /* 人工姿态、正在作业、故障后的未知姿态不得被开机约定覆盖。 */
    prepare_arm_auto(1);send_line("arm pose 0 130 0 45");
    simulated_start_button=1;tick(CAR_START_BUTTON_DEBOUNCE_MS+601u);
    {int lift;arm_pose_now(NULL,&lift,NULL,NULL);CHECK(lift==130 && !run_active && !start_button.preparing);}
    cold_start_fixture();simulated_start_button=1;tick(CAR_START_BUTTON_DEBOUNCE_MS);
    queue_byte('!');tick(601);
    CHECK(!run_active && !arm_enabled() && !start_button.preparing);
    puts("PASS startup cancellation: !, serial s, arm stop/off cancel IMU-wait; manual pose is retained; cold emergency wins");
}

static void cold_full_route_test(void)
{
    uint32_t start,deadline;unsigned offset=0,last_job=0;
    cold_start_fixture();peer_geometry=peer_disc_count=peer_move_count=0;
    start=clock_ms;deadline=clock_ms+300000u;
    simulated_start_button=1;tick(CAR_START_BUTTON_DEBOUNCE_MS+1u);
    while(!run_active && clock_ms<deadline)plant_step();
    CHECK(run_active && scanner_init_calls==1);
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
    printf("COLD_PC9_FULL_ROUTE plan=C elapsed_ms=%lu jobs=%u distance_mm=%lu max_endpoint_model_mm=%.3f\n",
        (unsigned long)(clock_ms-start),run.job,(unsigned long)run.distance_mm,checkpoint_max_error_mm);
}

int main(void)
{
    reset_fixture();
    CHECK(entry_host_start_button_level()==1 && !start_button_pressed());
    simulated_start_button=1;
    CHECK(entry_host_start_button_level()==0 && start_button_pressed());
    puts("PASS PC9 electrical levels: released HIGH, grounded press LOW");
    cold_button_tests();preparation_cancel_tests();pc9_and_scan_init_tests();cold_full_route_test();
    printf("PASS button startup: %lu assertions; host model only\n",(unsigned long)assertions);return 0;
}
