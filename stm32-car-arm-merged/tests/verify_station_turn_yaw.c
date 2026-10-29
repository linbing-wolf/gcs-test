/* 确认真正的云台PWM变化与进站转向重叠，禁止用到站后平台仍忙冒充并行。 */
#define STATION_TEST_MAIN retained_station_suite
#include "verify_station_vision.c"

static void rough_turn_yaw_tests(void)
{
    for(unsigned plan=0;plan<2;plan++)for(unsigned batch=0;batch<2;batch++) {
        unsigned offset=0,last_job=0,positions;uint32_t deadline,turn_ms;int source=batch ? 5 : 2;
        prepare_arm_auto(plan);send_line("task 134+123+314+231");send_line(plan ? "runimu C" : "runimu A");
        deadline=clock_ms+200000;
        while(clock_ms<deadline && !(run.job==source && run.station_align_pending && run.velocity_phase==VELOCITY_TURN)) {
            serve_peer(&offset);plant_step();if(run.job!=last_job){last_job=run.job;arm_positions=0;}
        }
        CHECK(clock_ms<deadline && abs(run.heading_control.target_cdeg)>=17900);turn_ms=clock_ms;positions=arm_positions;
        while(arm_yaw_us==1975 && clock_ms<deadline)plant_step();
        printf("ROUGH_TURN_YAW plan=%u batch=%u delay_ms=%lu chassis_progress_cdeg=%ld yaw_us=%u\n",
            plan,batch,(unsigned long)(clock_ms-turn_ms),(long)run.heading_control.progress_cdeg,arm_yaw_us);
        CHECK(clock_ms<deadline && run_active && run.station_entry_overlap && run.turning);
        CHECK(abs(run.heading_control.progress_cdeg)<9000 && run.turn_cmd_rpm && arm_yaw_us!=1975 && arm_positions==positions);
        stop_request();CHECK(!run_active && !station_active() && arm_failed());
    }
}

static void d_after_entry_tests(void)
{
    for(unsigned plan=0;plan<2;plan++)for(unsigned batch=0;batch<2;batch++) {
        unsigned offset=0,last_job=0,positions;uint32_t deadline;
        unsigned job=batch ? 5u : 2u;
        prepare_arm_auto(plan);send_line("task 134+123+314+231");send_line(plan ? "runimu C" : "runimu A");
        deadline=clock_ms+200000;
        while(clock_ms<deadline) {
            if(run.job==job && vision_tx_count>=offset+6u)break;
            if(run.job!=job)serve_peer(&offset);
            plant_step();if(run.job!=last_job){last_job=run.job;arm_positions=0;}
        }
        CHECK(clock_ms<deadline && vision_tx_bytes[offset+3u]=='D');
        CHECK(!run_active && !run.station_entry_overlap && !station_chassis_busy() && run.state==RUN_STATION);
        positions=arm_positions;
        peer_reply("MOVEXY,15,7",0);peer_reply("DISC,320,240,0,1",0);
        while(run.station_entry_overlap && clock_ms<deadline) {
            int slide,lift;arm_pose_now(&slide,&lift,NULL,NULL);
            CHECK(arm_positions==positions && slide==ARM_SLIDE_HOME_MM && lift==ARM_LIFT_HOME_MM);
            plant_step();
        }
        while(!run.station_visual_shift && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && run.vector_target_mm[0]==-15 && run.vector_target_mm[1]==7);
        CHECK(vision_tx_count==offset+6u);stop_request();
        peer_reply("MOVEXY,90,90",0);tick(200);CHECK(!run_active && !station_active());
    }
    puts("PASS D after arrival: both A/C and both rough visits wait for chassis completion; MOVE/terminal executes once; ! cancels");
}

static void curved_entry_and_full_routes(void)
{
    for(unsigned plan=0;plan<2;plan++) {
        unsigned offset=0,last_job=0,mask=0;uint32_t start,deadline;
        prepare_arm_auto(plan);send_line("task 134+123+314+231");send_line(plan ? "runimu C" : "runimu A");
        start=clock_ms;deadline=clock_ms+300000;
        while(run.state!=RUN_DONE && clock_ms<deadline) {
            uint16_t yaw=arm_yaw_us;
            serve_peer(&offset);plant_step();
            if(run.station_entry_overlap && run_active) {
                int slide,lift;arm_pose_now(&slide,&lift,NULL,NULL);
                CHECK(slide==ARM_SLIDE_HOME_MM && lift==ARM_LIFT_HOME_MM);
                if(run.corner.active && run_corner_rotating_span() && yaw!=arm_yaw_us && run.turn_cmd_rpm) {
                    HeadingSample sample=heading_snapshot();
                    int error=abs(heading_wrap_cdeg(CAR_MAP_YAW_SIGN*(int32_t)run.corner.end_heading*9000
                        - ((int32_t)sample.yaw_cdeg-run.yaw_zero_cdeg)));
                    if(error>STN_APPROACH_OVERLAP_YAW_CDEG)mask|=1u<<run.job;
                }
            }
            if(run.job!=last_job){last_job=run.job;arm_positions=0;}
        }
        CHECK(clock_ms<deadline && run.job==8 && mask==((1u<<3)|(1u<<4)|(1u<<6)));
        CHECK(run.distance_mm==EXPECTED_FULL_ROUTE_MM && peer_stack_count==3 && arm_ready() && arm_parked());
        CHECK(peer_area_count[1]==1 && peer_area_count[2]==1 && peer_area_count[3]==1 && peer_area_count[4]==1 && peer_area_count[5]==1);
        printf("STATION_YAW_ROUTE plan=%u elapsed_ms=%lu curved_entry_mask=%u jobs=%u max_endpoint_model_mm=%.3f\n",
            plan,(unsigned long)(clock_ms-start),mask,run.job,checkpoint_max_error_mm);
    }
}

int main(void)
{
    rough_turn_yaw_tests();d_after_entry_tests();curved_entry_and_full_routes();
    printf("PASS station turn/yaw: %lu assertions; host model only\n",(unsigned long)assertions);return 0;
}
