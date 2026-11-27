/* 只跑带物升顶时序专项，不执行被引入的整套路线回归。 */
#define STATION_TEST_MAIN station_full_suite_not_run
#include "verify_station_vision.c"
int main(void)
{
    unsigned area;
    for(area=0;area<5;area++) {
        unsigned offset=0,grips=0,placed=0,trace=0,carrying=0,lifted=0,lowered=0,parallel_slide=0,raw_started=0,cup_preselect_parallel=0;
        unsigned total=(area==0 || area==4) ? 6u : 3u;
        uint32_t deadline,closed_ms=0,top_ms=0;
        uint16_t cup_pwm=0,last_platform_pwm;
        float last_claw=ARM_CLAW_OPEN_DEG,place_yaw=0;
        int source_lift=0,place_lift=0,place_slide=0;
        if(area==3) {
            prepare_arm_auto(0);send_line("arm pose 0 135 131 45");
        } else if(area==4)prepare_rough(1,"task 134+231+314+123");
        else if(area)prepare_storage(area==2,"task 134+231+314+123");
        else prepare_rough(0,"task 134+231+314+123");
        last_platform_pwm=arm_platform_us;
        deadline=clock_ms+300000;
        while(placed<total && run.state!=RUN_FAULT && clock_ms<deadline) {
            int slide,lift;float yaw,claw;
            if(area==3 && !arm_busy() && raw_started<3) {
                send_line(raw_started==2 ? "arm load1 last" : "arm load1");
                CHECK(arm_busy());raw_started++;
            }
            if(area!=3)serve_peer(&offset);
            plant_step();arm_pose_now(&slide,&lift,&yaw,&claw);
            if((area==0 || area==4) && grips>=3 && !carrying && arm_platform_us!=last_platform_pwm) {
                int target_slide,target_lift;float target_yaw;
                CHECK(yaw==STN_ROUGH_YAW_DEG && claw==ARM_CLAW_OPEN_DEG);
                arm_target_now(&target_slide,&target_lift,&target_yaw,NULL);
                CHECK(target_slide==STN_ROUGH_SLIDE_MM);
                CHECK(target_lift==STN_ROUGH_LIFT_MM && target_yaw==STN_ROUGH_YAW_DEG);
                cup_preselect_parallel++;
            }
            last_platform_pwm=arm_platform_us;
            if(!closed_ms && arm_claw_us==604)closed_ms=clock_ms;
            if(!carrying && claw==ARM_CLAW_CLOSE_DEG && last_claw!=ARM_CLAW_CLOSE_DEG) {
                carrying=1;grips++;lifted=lowered=0;cup_pwm=arm_platform_us;top_ms=0;source_lift=lift;
                if(area==3 || ((area==0 || area==4) && grips>3)) {
                    CHECK(lift==(area==3 ? ARM_PICK_LIFT_MM : STN_ROUGH_LIFT_MM));
                    CHECK(yaw==(area==3 ? ARM_PICK_YAW_DEG : STN_ROUGH_YAW_DEG));
                    place_slide=ARM_PLACE_SLIDE_MM;place_lift=ARM_PLACE_LIFT_MM;place_yaw=ARM_PLACE_YAW_DEG;
                    /* 回杯前必须已选好原杯；不能等带物转回后才旋转载物台。 */
                    if(area!=3)CHECK(arm_platform_slot()==(int)(grips-3u));
                } else {
                    CHECK(lift==STN_ROUGH_PLATFORM_TAKE_LIFT_MM && yaw==ARM_PLACE_YAW_DEG);
                    place_slide=STN_ROUGH_SLIDE_MM;
                    place_lift=area==2 ? STN_STORAGE_LIFT_MM+STN_STORAGE_LAYER_LIFT_MM : STN_ROUGH_PLACE_LIFT_MM;
                    place_yaw=STN_ROUGH_YAW_DEG;
                }
            }
            if(carrying && lifted && lift==ARM_LIFT_HOME_MM && !top_ms)top_ms=clock_ms;
            if(carrying && (!lowered || ((area==0 || area==4) && grips>3)))CHECK(arm_platform_us==cup_pwm);
            while(trace<arm_positions) {
                unsigned n=trace++;
                if(!carrying)continue;
                if(arm_trace_address[n]==5 && arm_trace_dir[n]==ARM_LIFT_DIR_INVERT) {
                    CHECK(!lifted && arm_trace_ms[n]-closed_ms>=ARM_CLAW_CLOSE_HOLD_MS);
                    CHECK(arm_trace_pulses[n]==(ARM_LIFT_HOME_MM-source_lift)*ARM_LIFT_PULSES_PER_MM);
                    lifted=1;
                } else if(arm_trace_address[n]==5) {
                    CHECK(lifted && !lowered);
#if ARM_CUP_EXIT_DESCENT_DELAY_MS > 0
                    CHECK(top_ms && arm_trace_ms[n]-top_ms>=ARM_CUP_EXIT_DESCENT_DELAY_MS);
                    CHECK(arm_trace_ms[n]-top_ms<=ARM_CUP_EXIT_DESCENT_DELAY_MS+50u);
#endif
                    CHECK(top_ms);
                    CHECK(arm_trace_pulses[n]==(ARM_LIFT_HOME_MM-place_lift)*ARM_LIFT_PULSES_PER_MM);
                    lowered=1;
                } else if(arm_trace_address[n]==7) {
                    CHECK(lifted && !lowered && arm_platform_us==cup_pwm);
                    CHECK(arm_trace_pulses[n]==(uint32_t)((STN_ROUGH_SLIDE_MM-ARM_PLACE_SLIDE_MM)*ARM_SLIDE_PULSES_PER_MM+0.5f));
                    parallel_slide++;
                }
            }
            if(carrying && claw==ARM_CLAW_OPEN_DEG) {
                CHECK(lifted && lowered && lift==place_lift && slide==place_slide && yaw==place_yaw);
                placed++;carrying=0;closed_ms=0;
            }
            last_claw=claw;
        }
        CHECK(clock_ms<deadline && run.state!=RUN_FAULT && grips==total && placed==total && parallel_slide==total);
        if(area!=3)CHECK(peer_area_count[area==4 ? 5 : area==0 ? 2 : 3]==(area==2 ? 0u : 1u));
        if(area==0 || area==4)CHECK(cup_preselect_parallel>0);
        stop_request();
    }
    printf("PASS loaded rise sequencing: %u assertions; raw load1 all3, both rough batches outbound3/retrieval3 with target cup preselection parallel with side approach, complete before grip and held through return, first/second storage all3; claw hold then continuous source5/100/105->135 lift, yaw/slide release at actual110, delay only after top feedback, then135->5/65/110 with original final poses and vision requests; host only\n",assertions);
    return 0;
}
