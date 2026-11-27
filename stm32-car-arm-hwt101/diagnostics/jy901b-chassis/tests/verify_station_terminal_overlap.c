/* 真实全流程验证：以机构运动提交/PWM变化对齐非零底盘指令，不以busy或日志代替动作。 */
#define APPROACH_TEST_MAIN retained_approach_suite
#include "verify_approach_overlap.c"

static void complete_flow_overlap_tests(void)
{
    unsigned plan,phase;
    /* H新增修正会改变后续控制周期相位。5mm末段允许提前准备，
     * 但不保证每次PWM预检完成时车辆仍在动；跨相位保留各区真实运动重叠覆盖。 */
    for(plan=0;plan<2u;plan++)for(phase=0;phase<3u;phase++) {
        unsigned offset=0,index_motor_mask=0,ground_motor_mask=0;
        uint32_t deadline;unsigned positions=0,last_job=0;
        float hold_claw=0;
        prepare_arm_auto(plan);
        peer_plate_dx=phase==1 ? 4 : phase==2 ? -4 : 0;
        peer_plate_dy=phase==1 ? -3 : phase==2 ? 3 : 0;
        send_line("task 134+123+314+231");send_line(plan ? "runimu C" : "runimu A");
        deadline=clock_ms+2000000;
        while(run.state!=RUN_DONE && run.state!=RUN_FAULT && clock_ms<deadline) {
            uint8_t approach_before=run.station_approach_overlap;
            uint16_t old_yaw=arm_yaw_us;unsigned old_positions=arm_positions,tx=vision_tx_count;
            int on_road=run_active && !run.station_shift && !run.departure_overlap;
            serve_peer(&offset);plant_step();
            if(on_road && !run.station_shift)
                CHECK(arm_yaw_us==old_yaw && arm_positions==old_positions && vision_tx_count==tx && !station_active());
            if(run.station_approach_overlap && !approach_before) {
                positions=old_positions;arm_pose_now(NULL,NULL,NULL,&hold_claw);
            }
            if(run.station_approach_overlap) {
                float claw;arm_pose_now(NULL,NULL,NULL,&claw);
                CHECK(claw==hold_claw && arm_positions<=positions+2u);
                if(arm_positions>old_positions && (run.forward_cmd_rpm || run.left_cmd_rpm)) {
                    float target_yaw;
                    arm_target_now(NULL,NULL,&target_yaw,NULL);
                    index_motor_mask|=1u<<run.job;
                    if(target_yaw==STN_ROUGH_YAW_DEG)ground_motor_mask|=1u<<run.job;
                }
            }
            if(run.job!=last_job){last_job=run.job;arm_positions=0;}
        }
        if(run.state!=RUN_DONE)fprintf(stderr,"FLOW_OVERLAP state=%u job=%u %s\n",run.state,run.job,output);
        printf("OVERLAP_FLOW plan=%u index_axes=%02x ground_axes=%02x\n",plan,index_motor_mask,ground_motor_mask);
        CHECK(clock_ms<deadline && run.state==RUN_DONE && run.job==8);
        CHECK((index_motor_mask&0x24u)==0x24u); /* 粗加工地面靠近末段并行；车载搬运由pipeline专项覆盖。 */
        CHECK((ground_motor_mask&0x24u)==0x24u); /* 两次粗加工地面抓回：下降也确实提前。 */
        CHECK(run.distance_mm==EXPECTED_FULL_ROUTE_MM && arm_ready() && arm_parked());
        stop_request();
    }
    puts("PASS all areas: A/C routes wait for entry arrival; in-station index and grip timing checked for all H corrections");
}

static void index_overlap_delay_and_cancel_tests(void)
{
    unsigned variant;
    for(variant=0;variant<6u;variant++) {
        unsigned offset=0;uint32_t deadline;float initial_claw;
        prepare_rough(0,"task 134+123+314+231");deadline=clock_ms+30000;
        while(!(run.station_approach_overlap && run.remaining_mm==150) && run.state!=RUN_FAULT && clock_ms<deadline) {
            serve_peer(&offset);plant_step();
        }
        CHECK(clock_ms<deadline && run.station_approach_overlap && !run.vector_move && run.remaining_mm==150);
        arm_pose_now(NULL,NULL,NULL,&initial_claw);
        if(variant==0 || variant==1) {
            float claw;int slide,lift;
            if(variant==0)missing_feedback_motor=4;
            else {frozen_imu=1;heading_sensor_invalidate(&heading_sensor);}
            tick(4000);arm_pose_now(&slide,&lift,NULL,&claw);
            CHECK(run_active && station_active() && !arm_busy() && claw==initial_claw);
            CHECK(slide==STN_ROUGH_SLIDE_MM && lift==STN_ROUGH_LIFT_MM);
            missing_feedback_motor=0;frozen_imu=0;
            while(run_active && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
            CHECK(clock_ms<deadline && run.state==RUN_STATION && !run.station_relock && arm_lock.state==2);
            stop_request();continue;
        }
        if(variant==2)send_line("!");
        if(variant==3){bus_off=1;plant_step();}
        if(variant==4)send_line("obstacle");
        if(variant==5)send_line("arm stop");
        tick(100);
        CHECK(!run_active && !station_active() && arm_failed() && arm_lock.state==3);
        CHECK(!simulated_wheel_rpm[0] && !simulated_wheel_rpm[1] && !simulated_wheel_rpm[2] && !simulated_wheel_rpm[3]);
    }
    puts("PASS index overlap: missing wheel/IMU can let approach finish first without closing claw; feedback resumes; emergency/bus-off/obstacle/operator arm stop cancel both");
}

static void entry_delay_and_cancel_tests(void)
{
    unsigned variant;
    for(variant=0;variant<4u;variant++) {
        uint32_t deadline;unsigned tx,positions;
        uint16_t idle_yaw;
        prepare_arm_auto(0);send_line("task 134+123+314+231");send_line("runimu A");deadline=clock_ms+120000;
        while(!(run_active && run.job==1 && run.road.to==NODE_F) && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && !station_active() && !arm_busy());
        tx=vision_tx_count;idle_yaw=arm_yaw_us;positions=arm_positions;
        if(variant==0) {
            missing_feedback_motor=4;tick(4000);
            CHECK(!station_active() && !arm_busy() && arm_yaw_us==idle_yaw && arm_positions==positions && vision_tx_count==tx);
            missing_feedback_motor=0;
            while(vision_tx_count<tx+6u && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
            CHECK(clock_ms<deadline && run.state==RUN_STATION && vision_align_state()==VISION_PICK_WAITING);
            CHECK(vision_tx_count==tx+6u && vision_tx_bytes[tx+3u]=='H' && vision_pick_state()==VISION_PICK_IDLE);
            stop_request();continue;
        }
        if(variant==1)send_line("!");
        if(variant==2){bus_off=1;plant_step();}
        if(variant==3)send_line("arm stop");
        tick(100);
        if(variant==1)CHECK(!run_active && arm_failed());
        CHECK(!station_active() && arm_yaw_us==idle_yaw && arm_positions==positions && vision_tx_count==tx);
        stop_request();
    }
    puts("PASS entry wait: delayed feedback, CAN interruption and operator stops cannot start arm motion or vision during approach");
}

int main(void)
{
    overlap_tests();eligibility_tests();complete_flow_overlap_tests();
    index_overlap_delay_and_cancel_tests();entry_delay_and_cancel_tests();
    printf("PASS station terminal overlap: %u assertions; command/PWM timing simulation only, not physical acceptance\n",assertions);
    return 0;
}
