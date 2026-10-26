/* 从最后松爪记录首个非零底盘指令，不能以载物台仍在动冒充云台并行。 */
#define STATION_TEST_MAIN retained_station_suite
#include "verify_station_vision.c"

static void storage_start_tests(void)
{
    for(unsigned batch=0;batch<2;batch++)for(unsigned variant=0;variant<5;variant++) {
        unsigned offset=0,placed=0,corrected_baseline=0,drift_injected=0;uint32_t deadline,released;float previous=ARM_CLAW_OPEN_DEG;
        int64_t lift_position;int source=batch ? 6 : 3;
        prepare_storage(batch,"task 134+123+314+231");deadline=clock_ms+100000;
        while(placed<3 && clock_ms<deadline) {
            float yaw,claw;serve_peer(&offset);plant_step();arm_pose_now(NULL,NULL,&yaw,&claw);
            if(claw!=previous && claw==ARM_CLAW_OPEN_DEG && yaw==STN_STORAGE_YAW_DEG)placed++;
            previous=claw;
        }
        CHECK(clock_ms<deadline && run.job==source && !run_active);
        released=clock_ms;lift_position=arm_encoder[1];arm_freeze=(uint8_t)(variant==1);
        if(variant==2)run.zero_ms=run.stopped_ms=0; /* 没有旧零速计时，也不能重新等300ms。 */
        if(variant==3) {
            missing_feedback_motor=4;tick(180);
            CHECK(!run.forward_cmd_rpm && !run.left_cmd_rpm);missing_feedback_motor=0;
        }
        deadline=clock_ms+1500;
        while(!(run.forward_cmd_rpm || run.left_cmd_rpm) && clock_ms<deadline) {
            plant_step();
            if(variant==4 && !drift_injected && run.job==source+1) {
                physical_yaw+=300;sample_yaw(wrapped_yaw(physical_yaw));drift_injected=1;
            }
            if(variant==4 && run.velocity_phase==VELOCITY_BASELINE) {
                CHECK((int32_t)(run.baseline_after_ms-released)>0);corrected_baseline=1;
            }
        }
        printf("STORAGE_START batch=%u variant=%u release_to_drive_ms=%lu yaw_us=%u home_us=1975 state=%u\n",
            batch,variant,(unsigned long)(clock_ms-released),arm_yaw_us,run.state);
        CHECK(clock_ms<deadline && run.job==source+1 && run_active);
        if(variant<3)CHECK(arm_busy() && arm_yaw_us!=1975 && clock_ms-released<CAR_HEADING_STABLE_MS);
        if(variant==1)CHECK(arm_encoder[1]==lift_position);
        if(variant==4)CHECK(corrected_baseline);
        stop_request();CHECK(!run_active && !station_active());
    }
}

static void actual_turn_start_tests(void)
{
    for(unsigned plan=0;plan<2;plan++) {
        unsigned offset=0,last_job=0,turns=0;uint32_t deadline;
        prepare_arm_auto(plan);send_line("task 134+123+314+231");send_line(plan ? "runimu C" : "runimu A");
        deadline=clock_ms+300000;
        while(run.state!=RUN_DONE && clock_ms<deadline) {
            int old_phase=run.velocity_phase;unsigned old_job=run.job;
            serve_peer(&offset);plant_step();
            if(strstr(output,"[station turn overlap]") && (old_job==2 || old_job==5)) {
                uint32_t entered=clock_ms;int residual=(int)run.action_mm-run.progress_mm;double translation=0;
                CHECK(old_phase==VELOCITY_FORWARD && abs(run.heading_control.target_cdeg)>=17900);
                for(unsigned i=0;i<4;i++)translation+=fabs(actual_rpm[i]);
                printf("TURN_180_ENTRY plan=%u job=%u remaining_mm=%d actual_abs_rpm_sum=%.2f\n",plan,old_job,residual,translation);
                CHECK(residual>CAR_DISTANCE_TOLERANCE_MM && residual<=CAR_SETTLE_DISTANCE_MM && translation>4);
                while((run.forward_cmd_rpm || run.left_cmd_rpm || !run.turn_cmd_rpm
                    || (int32_t)(run.velocity_ms-entered)<0) && clock_ms<entered+200) {
                    CHECK(!run.zero_ms);plant_step();
                }
                CHECK(!run.forward_cmd_rpm && !run.left_cmd_rpm && run.turn_cmd_rpm
                    && clock_ms-entered<=CAR_VELOCITY_CONTROL_MS+20u && !run.zero_ms);
                translation=0;for(unsigned i=0;i<4;i++)translation+=fabs(actual_rpm[i]);CHECK(translation>4);
                printf("TURN_180_COMMAND plan=%u job=%u delay_ms=%lu turn_rpm=%d actual_abs_rpm_sum=%.2f\n",plan,old_job,
                    (unsigned long)(clock_ms-entered),run.turn_cmd_rpm,translation);turns++;
            }
            if(run.job!=last_job){last_job=run.job;arm_positions=0;}
        }
        CHECK(clock_ms<deadline && turns==2 && run.job==8 && run.distance_mm==EXPECTED_FULL_ROUTE_MM);
    }
}

int main(void)
{
    storage_start_tests();actual_turn_start_tests();
    printf("PASS storage/180 start: %lu assertions; actual command timing in host model only\n",(unsigned long)assertions);return 0;
}
