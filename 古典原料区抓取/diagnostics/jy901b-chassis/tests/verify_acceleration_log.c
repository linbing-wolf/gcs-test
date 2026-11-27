#define HEADING_TEST_MAIN retained_acceleration_log_suite
#include "verify_heading.c"

static void acceleration_frame(int16_t x,int16_t y,int16_t z)
{
    uint8_t frame[11]={0x55,0x51,0,0,0,0,0,0,0,0,0};int16_t raw[3]={x,y,z};
    for(unsigned i=0;i<3;i++) {frame[2+i*2]=(uint8_t)raw[i];frame[3+i*2]=(uint8_t)((uint16_t)raw[i]>>8);}
    for(unsigned i=0;i<10;i++)frame[10]=(uint8_t)(frame[10]+frame[i]);
    for(unsigned i=0;i<11;i++)heading_sensor_byte(&heading_sensor,frame[i],clock_ms);
}
static void log_now(void)
{
    output_size=0;output[0]=0;run.log_ms=clock_ms-CAR_CONTROL_LOG_MS;run_velocity_log();
}
int main(void)
{
    reset_plant(0,1.15);diagnostic_init();clock_ms=1000;
    run.velocity_phase=VELOCITY_FORWARD;run.travel_axis=BODY_LEFT;run.action_mm=2000;
    log_now();CHECK(strstr(output,"acc_fresh=0 acc_age_ms=-1 acc_mg=NA acc_n=0"));
    acceleration_frame(1024,-512,2048); /* 500,-250,1000mg */
    run.log_ms=clock_ms;run_velocity_log();
    for(unsigned i=0;i<20;i++)run_velocity_log(); /* 同一帧不能重复计数。 */
    clock_ms+=100;acceleration_frame(-2048,512,3072); /* -1000,250,1500mg */
    run_velocity_log();clock_ms+=20;log_now();
    CHECK(strstr(output,"acc_fresh=1 acc_age_ms=20 acc_mg=-1000,250,1500 acc_n=2"));
    CHECK(strstr(output,"acc_win=1000,1100,-1000,500,-250,250,1000,1500"));
    CHECK(strstr(output,"acc_mean_mg=-250,0,1250"));
    CHECK(strstr(output,"tilt_cdeg=0,0"));
    log_now();CHECK(strstr(output,"acc_n=0") && !strstr(output,"acc_win="));
    clock_ms+=CAR_IMU_MAX_AGE_MS+1;log_now();
    CHECK(strstr(output,"acc_fresh=0 acc_age_ms=271 acc_mg=-1000,250,1500 acc_n=0"));
    clock_ms+=10;acceleration_frame(256,0,2048);log_now();
    CHECK(strstr(output,"acc_fresh=1 acc_age_ms=0 acc_mg=125,0,1000 acc_n=1"));
    CHECK(strstr(output,"acc_mean_mg=125,0,1000"));
    CHECK(!velocity_count && !arm_positions && !arm_queries && arm_claw_us==826 && arm_yaw_us==1975 && arm_platform_us==648);
    puts("PASS acceleration log: XYZ mg, freshness, unique frames, min/max windows, reset and fixed actuators; observation only");return 0;
}
