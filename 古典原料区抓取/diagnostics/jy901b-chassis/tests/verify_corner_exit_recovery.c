/* 测量出弯后真实路径速度恢复，不能只验证参考斜坡有增加。 */
#define STATION_TEST_MAIN retained_station_suite
#include "verify_station_vision.c"

static void exit_recovery_models(void)
{
    for(unsigned area=0;area<3;area++)for(unsigned response=0;response<2;response++) {
        uint32_t deadline,start,exit_ms=0,recovery_ms=0;float exit_s=0;
        double min_speed=100000,max_yaw=0,max_error=0;unsigned samples=0;
        if(area==2)prepare_storage(0,"task 134+123+314+231");else prepare_rough(area,"task 134+123+314+231");
        yaw_gain=response ? 0.6 : 1.15;imu_period_ms=20;
        for(unsigned i=0;i<4;i++)wheel_acceleration[i]=response ? 0.18 : 0.30;
        CHECK(heading_control_start(&run.heading_control,heading_snapshot(),0,clock_ms));
        run_resume_after_station();checked_job=run.job;start=clock_ms;deadline=clock_ms+60000;
        while(run_active && clock_ms<deadline) {
            plant_step();
            if(!run.corner.active)continue;
            for(unsigned i=0;i<run.corner.span_count;i++)if(run.corner.spans[i].dyaw)
                exit_s=run.corner.spans[i].s+run.corner.spans[i].length;
            if(run.corner.s_mm>=exit_s && !exit_ms)exit_ms=clock_ms;
            if(exit_ms && run.corner.s_mm<=exit_s+200 && !run.corner.visual_terminal) {
                double forward=(-actual_rpm[0]+actual_rpm[1]+actual_rpm[2]-actual_rpm[3])/4.0;
                double left=(-actual_rpm[0]-actual_rpm[1]+actual_rpm[2]+actual_rpm[3])/4.0;
                double speed=hypot(forward*CAR_POSITION_PULSES_PER_REV/(60*CALIBRATED_PPM_Y),
                    left*CAR_POSITION_PULSES_PER_REV/(60*CALIBRATED_PPM_X));
                double yaw=abs(run.heading_control.error_cdeg)/100.0;
                CornerPoint p=run_corner_point(run.corner.s_mm);
                double error=hypot(run.corner.x_mm-p.x,run.corner.y_mm-p.y);
                if(speed<min_speed)min_speed=speed;
                if(yaw>max_yaw)max_yaw=yaw;
                if(error>max_error)max_error=error;
                if(!recovery_ms && speed>=200)recovery_ms=clock_ms-exit_ms;
                samples++;
            }
        }
        CHECK(clock_ms<deadline && exit_ms && samples && run.state==RUN_STATION);
        printf("EXIT_RECOVERY area=%u response=%u total_ms=%lu recover200_ms=%lu min_mm_s=%.3f max_yaw_deg=%.3f path_mm=%.3f\n",
            area,response,(unsigned long)(clock_ms-start),(unsigned long)recovery_ms,min_speed,max_yaw,max_error);
        stop_request();
    }
}

static void exit_scope_and_rate_tests(void)
{
    float desired[3]={100,0,0};int16_t output[3];float error=0;
    reset_plant(0,1.15);memset(&run,0,sizeof(run));
    run.corner.active=1;run.corner.span_count=1;
    run.corner.spans[0].radius=150;run.corner.spans[0].length=150*1.570796327f;
    run.corner.spans[0].dyaw=-9000;run.corner.spans[0].turn=1;
    run.corner.s_mm=run.corner.spans[0].length*0.9f;run.heading_control.progress_cdeg=-8500;
    CHECK(run_corner_exit_pending(&error) && error==500); /* 几何弯尚未结束，已可温和恢复。 */
    run.corner.s_mm=run.corner.spans[0].length+1;run.corner.command_ms=clock_ms;clock_ms+=25;
    run_corner_slew(desired,output);
    CHECK(fabsf(run.corner.command[0]-CAR_CORNER_EXIT_MAX_ACCEL_RPM_PER_S*0.025f)<0.001f);
    run.corner.command[0]=40;desired[0]=0;clock_ms+=25;run_corner_slew(desired,output);
    CHECK(fabsf(run.corner.command[0]-(40-CAR_DRIVE_DECEL_RPM_PER_S*0.025f))<0.001f);
    puts("PASS exit scope: geometric last5deg may recover early, aligned wheel acceleration uses exit maximum; zero request retains200RPM/s");
}

int main(void)
{
    exit_scope_and_rate_tests();exit_recovery_models();printf("PASS exit recovery: %lu assertions; independent wheel model only\n",(unsigned long)assertions);return 0;
}
