#define HEADING_TEST_MAIN retained_braking_suite
#include "verify_heading.c"

/* 执行延迟和制动能力独立于固件参数；不是对某块实物驱动器的拟合。 */
typedef struct {uint32_t ms;int16_t rpm[4];} DelayedCommand;
static DelayedCommand history[2048];
static unsigned history_count;
static uint32_t driver_delay_ms;
static double driver_decel;

static void delayed_step(void)
{
    uint32_t elapsed;double turn=0;
    if(!history_count || memcmp(history[history_count-1].rpm,simulated_wheel_rpm,sizeof(history[0].rpm))) {
        CHECK(history_count<2048);
        history[history_count].ms=clock_ms;
        memcpy(history[history_count++].rpm,simulated_wheel_rpm,sizeof(history[0].rpm));
    }
    clock_ms++;elapsed=clock_ms-plant_ms;plant_ms=clock_ms;
    for(unsigned i=0;i<4;i++) {
        double target=0,difference,change;
        for(unsigned n=0;n<history_count;n++)
            if((uint32_t)(clock_ms-history[n].ms)>=driver_delay_ms+i*5u)target=history[n].rpm[i];
        difference=target-actual_rpm[i];
        change=((target*actual_rpm[i]>=0 && fabs(target)>fabs(actual_rpm[i]))
            ? 0.18 : driver_decel*(1.0+i*0.03))*elapsed;
        if(difference>change)difference=change;
        if(difference< -change)difference=-change;
        actual_rpm[i]+=difference;
        encoder_value[i]+=encoder_polarity[i]*actual_rpm[i]*65536.0*elapsed/60000.0;
        simulated_encoder[i]=(int64_t)llround(encoder_value[i]);
        reported_wheel_rpm[i]=(int16_t)lround(encoder_polarity[i]*actual_rpm[i]);
        turn+=actual_rpm[i]/4.0;
    }
    physical_yaw+=turn*yaw_gain*0.1*elapsed;
    if(clock_ms-imu_ms>=10u) {
        imu_ms=clock_ms;sample_yaw(wrapped_yaw(physical_yaw));
        heading_sensor_publish_rate(&heading_sensor,(int32_t)lround(turn*yaw_gain*100),clock_ms);
    }
    output_size=0;output[0]=0;firmware_step();
}

static void braking_test(unsigned right,unsigned rpm,unsigned variant)
{
    uint32_t deadline;int32_t peak=0;unsigned max_request=0;double peak_yaw=0;
    reset_plant(0,1.15);diagnostic_init();translation_yaw_disturbance=0;
    for(unsigned i=0;i<4;i++)encoder_polarity[i]=-1;
    driver_delay_ms=variant ? 220u : 120u;
    driver_decel=variant ? 0.12 : 0.18;history_count=0;
    send_line("loop encoder off");send_line("loop forward off");send_line("loop gyro on");
    send_line("test");send_line("dist 2000");send_line(rpm==200 ? "rpm 200" : "rpm 120");send_line(right ? "r" : "l");
    deadline=clock_ms+30000;
    while(run_active && clock_ms<deadline) {
        delayed_step();
        if(run.progress_mm>peak)peak=run.progress_mm;
        if(abs(run.left_cmd_rpm)>max_request)max_request=abs(run.left_cmd_rpm);
        if(fabs(physical_yaw)>peak_yaw)peak_yaw=fabs(physical_yaw);
        CHECK(!arm_positions && arm_claw_us==826 && arm_yaw_us==1975 && arm_platform_us==648);
    }
    printf("BRAKING right=%u rpm=%u delay=%ums decel=%.0fRPM/s peak_mm=%ld max_request=%u yaw_peak=%.2fdeg final_mm=%ld state=%u\n",
        right,rpm,driver_delay_ms,driver_decel*1000,(long)peak,max_request,peak_yaw/100,(long)run.progress_mm,run.state);
    CHECK(!run_active && run.state==RUN_DONE && clock_ms<deadline);
    CHECK(abs(run.progress_mm-2000)<=CAR_DISTANCE_TOLERANCE_MM+2);
    CHECK(peak<=2010); /* 保留相同验收断言，先记录旧程序的过冲失败。 */
    if(rpm==200)CHECK(max_request>=190); /* 不能通过把巡航上限降到120来让测试通过。 */
}
int main(int argc,char **argv)
{
    unsigned mode=argc>1 ? (unsigned)atoi(argv[1]) : 0;
    braking_test(mode&1u,(mode&2u) ? 120u : 200u,(mode>>2)&1u);
    printf("PASS delayed lateral braking: %u assertions; model only\n",assertions);return 0;
}
