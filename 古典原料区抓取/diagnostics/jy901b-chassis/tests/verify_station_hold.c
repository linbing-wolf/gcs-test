/* 工位航向保持：真实命令/工位/机构控制，CAN和底盘为主机模型。 */
#define RAW_PLATE_TEST_MAIN retained_station_hold_raw_suite
#include "verify_raw_plate.c"

static void hold_fresh(void)
{
    for(unsigned i=0;i<4;i++) {
        motor_feedback[i].speed_valid=1;motor_feedback[i].speed_rpm=0;
        motor_feedback[i].status_flags=MOTOR_FLAG_ENABLED;
        motor_feedback[i].speed_ms=motor_feedback[i].status_ms=clock_ms;
        arm_lock.seen[i]=3;
    }
}
static void hold_fixture(int start,RoadHeading heading)
{
    reset_plant(0,1.15);translation_yaw_disturbance=0;
    run.state=RUN_STATION;run.imu_mode=run.station_enabled=run.station_phase=run.start_reference_valid=1;
    run.job=2;run.heading=run.target_heading=heading;run.initial_heading=HEAD_X_NEG;
    run.yaw_zero_cdeg=heading_wrap_cdeg(start-CAR_MAP_YAW_SIGN*(int32_t)run.initial_heading*9000);
    arm_lock.state=2;hold_fresh();station_hold_enabled=1;
}
static void hold_tick(int yaw)
{
    clock_ms+=25;sample_yaw((int16_t)yaw);hold_fresh();arm_lock_update();
}
static void reference_and_output_tests(void)
{
    for(unsigned plan=0;plan<2;plan++)for(unsigned h=0;h<4;h++)for(int sign=-1;sign<=1;sign+=2) {
        int start=plan ? 17950 : -17300,target,measured;
        hold_fixture(start,(RoadHeading)h);run.plan=(uint8_t)plan;
        target=heading_wrap_cdeg(start+CAR_MAP_YAW_SIGN*((int)h-HEAD_X_NEG)*9000);
        measured=heading_wrap_cdeg(target+sign*300);
        for(unsigned n=0;n<12;n++) {
            hold_tick(measured);
            CHECK(station_hold.target_cdeg==target && abs(station_hold.command_rpm)<=5);
            CHECK(station_hold.command_rpm*sign<=0);
        }
        CHECK(station_hold.active && station_hold.command_rpm*sign<0 && !run_active);
        CHECK(run.yaw_zero_cdeg==heading_wrap_cdeg(start-CAR_MAP_YAW_SIGN*(int32_t)HEAD_X_NEG*9000));
        for(unsigned i=1;i<4;i++)CHECK(simulated_wheel_rpm[i]==simulated_wheel_rpm[0]);
        CHECK(!position_count && !arm_stops && !host_tx_cancels[5] && !host_tx_cancels[7]);
    }
    hold_fixture(1200,HEAD_X_NEG);
    for(unsigned n=0;n<8;n++)hold_tick(1210);
    CHECK(station_hold.active && !station_hold.command_rpm);
    {
        HmiSnapshot display;screen_snapshot(&display);
        CHECK(display.value[HMI_YAW_T]==station_hold.target_cdeg/10);
    }
    puts("PASS hold reference: A/C, four station headings, both errors and yaw wrap; fixed startup reference, pure rotation<=5RPM, quiet zone");
}
static void ownership_and_fault_tests(void)
{
    hold_fixture(0,HEAD_X_NEG);send_line("loop gyro off");send_line("loop encoder off");
    for(unsigned n=0;n<12;n++)hold_tick(300);
    CHECK(station_hold.command_rpm<0); /* 行驶开关不影响工位保持。 */
    CHECK(!station_hold_release());
    for(unsigned n=0;n<30 && station_hold.active;n++)hold_tick(300);
    CHECK(!station_hold.active && !simulated_wheel_rpm[0] && station_hold_release());
    hold_fixture(0,HEAD_X_NEG);for(unsigned n=0;n<12;n++)hold_tick(300);
    station_hold.command_mrpm=station_hold.command_rpm=0;station_hold.waiting=0;
    station_hold.velocity_ms=clock_ms;motor_feedback[0].speed_ms=clock_ms-1;
    CHECK(!station_hold_release());hold_fresh();motor_feedback[3].speed_rpm=2;
    CHECK(!station_hold_release());motor_feedback[3].speed_rpm=0;CHECK(station_hold_release());
    hold_fixture(0,HEAD_X_NEG);station_hold_enabled=0;send_line("loop station off");hold_tick(300);
    CHECK(!station_hold.active && !velocity_batches);
    send_line("loop station on");for(unsigned n=0;n<12;n++)hold_tick(300);
    send_line("loop station off");for(unsigned n=0;n<30 && station_hold.active;n++)hold_tick(300);
    CHECK(!station_hold.active && !simulated_wheel_rpm[0]);
    for(unsigned blocked=0;blocked<5;blocked++) {
        hold_fixture(0,HEAD_X_NEG);
        if(blocked==0)run.start_reference_valid=0;
        if(blocked==1)run.vision_debug=1;
        if(blocked==2)run_active=1;
        if(blocked==3)run.station_relock=1;
        if(blocked==4)run.job=0;
        hold_tick(300);CHECK(!station_hold.active && !velocity_batches);
    }
    hold_fixture(0,HEAD_X_NEG);host_tx_stats.queued=12;
    for(unsigned n=0;n<8;n++)hold_tick(300);
    CHECK(!velocity_batches && diagnostic_queries>0);host_tx_stats.queued=0;
    for(unsigned n=0;n<12;n++)hold_tick(300);
    CHECK(velocity_batches && station_hold.command_rpm<0);
    {
        unsigned batches=velocity_batches;clock_ms+=CAR_IMU_MAX_AGE_MS+1;hold_fresh();arm_lock_update();
        CHECK(velocity_batches==batches && !arm_stops); /* 反馈缺失沿用告警等待，不中止机构。 */
    }
    hold_fixture(0,HEAD_X_NEG);clock_ms=UINT32_MAX-80u;
    for(unsigned n=0;n<12;n++)hold_tick(300);
    CHECK(station_hold.active && station_hold.command_rpm<0);
    hold_fixture(0,HEAD_X_NEG);velocity_ack_enabled=0;
    for(unsigned n=0;n<30;n++)hold_tick(300);
    CHECK(station_hold.waiting && !arm_stops && !run_active);
    hold_fixture(0,HEAD_X_NEG);for(unsigned n=0;n<12;n++)hold_tick(300);
    queue_byte('!');firmware_step();firmware_step();
    CHECK(!station_hold.active && !run.start_reference_valid && !run_active);
    for(unsigned i=0;i<4;i++)CHECK(!simulated_wheel_rpm[i]);
    puts("PASS ownership: independent switch, zero/ACK/fresh-stop release, manual/debug/route exclusions, CAN headroom, stale IMU, missing ACK and explicit stop");
}
static void concurrent_job_test(unsigned job)
{
    unsigned offset=0,disturbed=0,nonzero=0,arm_at_disturb=0,arm_after=0;uint32_t deadline;
    if(job==1u || job==4u) {
        prepare_raw_plate_task(job==4u,"task 134+312+314+312");offset=vision_tx_count;
        peer_reply("HDISC,320,240,0,3,1,1",0);
    } else if(job==3u || job==6u)prepare_storage(job==6u,"task 134+312+314+312");
    else prepare_rough(job==5u,"task 134+312+314+312");
    translation_yaw_disturbance=0;
    run.start_reference_valid=1;station_hold_enabled=1;deadline=clock_ms+180000;
    while(run.job==job && clock_ms<deadline) {
        serve_peer(&offset);plant_step();
        if(!disturbed && arm_busy() && arm_positions) {
            physical_yaw+=300;disturbed=1;arm_at_disturb=arm_positions;
        }
        if(disturbed && station_hold.active && station_hold.command_rpm) {
            nonzero=1;if(arm_positions>arm_at_disturb)arm_after=1;
        }
        CHECK(!arm_failed() && run.state!=RUN_FAULT && !arm_stops);
        CHECK(!host_tx_cancels[5] && !host_tx_cancels[7]);
        if(station_hold.active)CHECK(abs(station_hold.command_rpm)<=5 && !run_active);
        if(run_active && run.station_shift)CHECK(!station_hold.active);
    }
    CHECK(clock_ms<deadline && disturbed && nonzero && arm_after);
    CHECK(run.job==job+1u && !station_hold.active);
    CHECK(fabs(physical_yaw)<=80);stop_request();
    printf("PASS station job%u: real arm progresses during nonzero hold correction, correct target recovered, micro-move/departure transfers ownership\n",job);
}
static void startup_capture_tests(void)
{
    for(unsigned plan=0;plan<2;plan++) {
        prepare_arm_auto(plan);sample_yaw(17300);
        send_line(plan ? "runimu C" : "runimu A");
        CHECK(run_active && run.start_reference_valid);
        CHECK(heading_wrap_cdeg(run.yaw_zero_cdeg+CAR_MAP_YAW_SIGN*(int32_t)run.initial_heading*9000)==17300);
        stop_request();CHECK(!run.start_reference_valid && !station_hold.active);
    }
    reset_fixture();CHECK(station_hold_enabled==CAR_STATION_HOLD_DEFAULT_ON);
}
int main(void)
{
    startup_capture_tests();reference_and_output_tests();ownership_and_fault_tests();
    for(unsigned job=1;job<=6;job++)concurrent_job_test(job);
    printf("PASS station hold: %u assertions; host model only, no physical acceptance\n",assertions);
    return 0;
}
