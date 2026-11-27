/* 独立底盘入口、JY901B标准角度帧与固定机构PWM；不连接硬件。 */
#define HEADING_TEST_MAIN retained_jy901b_heading_suite
#include "verify_heading.c"

static void diagnostic_fixture(void)
{
    reset_plant(0,1.15);translation_yaw_disturbance=0;diagnostic_init();
    CHECK(CAR_IMU_UART_BAUD==115200 && diagnostic_imu_baud==115200);
    CHECK(test_mode==MODE_TRANSLATION_TEST && arm_enables==2 && !arm_positions && !arm_queries);
    CHECK(arm_claw_us==826 && arm_yaw_us==1975 && arm_platform_us==648);
    CHECK(!arm_enabled() && !arm_busy());
}
static void imu_and_command_tests(void)
{
    const char *blocked[]={"arm on","arm home","arm yaw 133","arm slide 55","claw close","runimu A","runimu C",
        "nav","go H","pose A","task 134","vision debug D","scan C","raw run 1","loop station on"};
    unsigned enables,pwm_claw,pwm_yaw,pwm_platform;
    diagnostic_fixture();enables=arm_enables;pwm_claw=arm_claw_us;pwm_yaw=arm_yaw_us;pwm_platform=arm_platform_us;
    heading_sensor_reset(&heading_sensor);
    wit_packet(&heading_sensor,0x53,16384,clock_ms,0);
    CHECK(heading_snapshot().valid && heading_snapshot().yaw_cdeg==9000);
    wit_packet(&heading_sensor,0x51,0,clock_ms,0);CHECK(heading_snapshot().sequence==1);
    wit_packet(&heading_sensor,0x53,0,clock_ms,1);CHECK(heading_sensor.checksum_errors==1);
    send_line("imu");CHECK(strstr(output,"fresh=1") && strstr(output,"baud=115200"));
    CHECK(strstr(output,"rate_fresh=0") && strstr(output,"tilt_warn=") && strstr(output,"max_tilt_cdeg=3000"));
    for(unsigned i=0;i<sizeof(blocked)/sizeof(blocked[0]);i++) {
        send_line(blocked[i]);CHECK(strstr(output,"rejected") && !run_active);
        CHECK(!position_count && !velocity_batches && !arm_positions && !arm_queries && arm_enables==enables);
    }
    send_line("arm status");CHECK(strstr(output,"MANUALLY PLACED"));
    send_line("imu baud 9600");CHECK(diagnostic_imu_baud==9600 && !heading_snapshot().valid && !heading_sensor.rate.valid);
    send_line("l");CHECK(!run_active && strstr(output,"rejected"));
    send_line("imu baud 12345");CHECK(diagnostic_imu_baud==9600 && strstr(output,"baud rejected"));
    send_line("imu baud 230400");CHECK(diagnostic_imu_baud==230400);
    send_line("imu baud 115200");sample_yaw(0);
    send_line("loop encoder off");send_line("loop gyro on");send_line("loop status");
    CHECK(strstr(output,"encoder=OFF gyro=ON") && strstr(output,"DISABLED_IN_DIAGNOSTIC"));
    simulated_start_button=1;for(unsigned i=0;i<1500;i++)plant_step();
    simulated_start_button=0;for(unsigned i=0;i<50;i++)plant_step();
    CHECK(!run_active && !velocity_batches && !scanner_init_calls && !arm_positions);
    CHECK(arm_claw_us==pwm_claw && arm_yaw_us==pwm_yaw && arm_platform_us==pwm_platform);
    puts("PASS JY901B: 55/53 angle/checksum, baud selection/invalid/stale guards; route/arm/vision/scanner/PC9 blocked; fixed PWM unchanged");
}
/* 完整0x53帧：横滚/俯仰/yaw原始值，用于倾斜诊断校验。 */
static void wit_attitude(HeadingSensor *sensor,int16_t roll,int16_t pitch,int16_t yaw,uint32_t now)
{
    uint8_t frame[11]={0x55,0x53,0,0,0,0,0,0,0,0,0};
    unsigned i;
    frame[2]=(uint8_t)roll;frame[3]=(uint8_t)((uint16_t)roll>>8);
    frame[4]=(uint8_t)pitch;frame[5]=(uint8_t)((uint16_t)pitch>>8);
    frame[6]=(uint8_t)yaw;frame[7]=(uint8_t)((uint16_t)yaw>>8);
    for(i=0;i<10;i++)frame[10]=(uint8_t)(frame[10]+frame[i]);
    for(i=0;i<11;i++)heading_sensor_byte(sensor,frame[i],now);
}
static void stability_loop_tests(void)
{
    HeadingSample sample;
    HeadingRate rate;
    uint32_t before;
    diagnostic_fixture();heading_sensor_reset(&heading_sensor);
    /* 0x52角速度帧：±2000度/秒满量程，按0.01度/秒存放，不刷新角度样本。 */
    wit_packet(&heading_sensor,0x52,1024,1000,0);
    CHECK(heading_sensor.rate.valid && heading_sensor.rate.rate_cdeg_s==6250);
    CHECK(heading_sensor.rate.sequence==1 && heading_sensor.rate.received_ms==1000);
    CHECK(heading_sensor.sample.sequence==0 && !heading_sensor.sample.valid);
    wit_packet(&heading_sensor,0x52,32767,1010,0);CHECK(heading_sensor.rate.rate_cdeg_s==199993);
    wit_packet(&heading_sensor,0x52,-32768,1020,0);CHECK(heading_sensor.rate.rate_cdeg_s==-200000);
    wit_packet(&heading_sensor,0x52,1024,1030,1);
    CHECK(heading_sensor.checksum_errors==1 && heading_sensor.rate.rate_cdeg_s==-200000);
    /* 角度与角速度两条通道的时效互相独立。 */
    wit_packet(&heading_sensor,0x52,512,1100,0);
    wit_packet(&heading_sensor,0x53,0,1200,0);
    rate=heading_sensor.rate;sample=heading_sensor.sample;
    CHECK(heading_rate_fresh(&rate,1350,CAR_IMU_MAX_AGE_MS));
    CHECK(!heading_rate_fresh(&rate,1351,CAR_IMU_MAX_AGE_MS));
    CHECK(heading_sample_fresh(&sample,1450,CAR_IMU_MAX_AGE_MS));
    CHECK(!heading_sample_fresh(&sample,1451,CAR_IMU_MAX_AGE_MS));
    CHECK(heading_sensor.sample.sequence==1 && heading_sensor.rate.sequence==4);
    /* 0x53横滚/俯仰解析；倾斜只计数，不使有效Yaw失效。 */
    before=heading_tilt_warnings;
    wit_attitude(&heading_sensor,16384,-16384,9000,1300);
    CHECK(heading_sensor.sample.valid && heading_sensor.sample.roll_cdeg==9000
        && heading_sensor.sample.pitch_cdeg==-9000);
    sample=heading_snapshot();
    CHECK(sample.valid && heading_tilt_warnings==before+1);
    (void)heading_snapshot();
    CHECK(heading_tilt_warnings==before+1);
    CHECK(heading_sample_fresh(&sample,1300,CAR_IMU_MAX_AGE_MS));
    wit_attitude(&heading_sensor,1000,-1000,0,1320);
    sample=heading_snapshot();
    CHECK(sample.valid && sample.yaw_cdeg==0 && heading_tilt_warnings==before+1);
    CHECK(heading_sensor.sample.roll_cdeg==549 && heading_sensor.sample.pitch_cdeg==-549);
    /* 新鲜角速度帧替代角度差分作为D项；过期后回退角度差分。 */
    clock_ms=2000;heading_sensor_reset(&heading_sensor);
    sample_yaw(0);
    CHECK(heading_control_start(&run.heading_control,heading_snapshot(),0,clock_ms));
    clock_ms=2020;sample_yaw(0);
    wit_packet(&heading_sensor,0x52,1024,clock_ms,0); /* 原始1024=62.5度/秒=6250(0.01度/秒) */
    CHECK(heading_update(&run.heading_control,heading_snapshot())!=HEADING_FAULT);
    CHECK(run.heading_control.rate_cdeg_s==2500 && run.heading_control.rate_source==1);
    CHECK(heading_control_drive_rpm(&run.heading_control,60)==-8);
    clock_ms=2300;sample_yaw(0);rate=heading_sensor.rate;
    CHECK(!heading_rate_fresh(&rate,clock_ms,CAR_IMU_MAX_AGE_MS));
    CHECK(heading_update(&run.heading_control,heading_snapshot())!=HEADING_FAULT);
    CHECK(run.heading_control.rate_cdeg_s>0 && run.heading_control.rate_cdeg_s<2500);
    CHECK(!run.heading_control.rate_source);
    for(unsigned n=0;n<12;n++) {clock_ms+=20;sample_yaw(0);(void)heading_update(&run.heading_control,heading_snapshot());}
    CHECK(abs(run.heading_control.rate_cdeg_s)<5 && heading_control_drive_rpm(&run.heading_control,60)==0);
    clock_ms+=20;sample_yaw(100);
    CHECK(heading_update(&run.heading_control,heading_snapshot())!=HEADING_FAULT);
    CHECK(run.heading_control.rate_cdeg_s>=2000 && run.heading_control.rate_cdeg_s<=2002);
    puts("PASS sensor feedback: full-range gyro, independent freshness, tilt diagnostic, filtered rate and continuous fallback");
}
static void translation_tests(void)
{
    const char *directions[]={"f","b","l","r"};
    for(unsigned gyro=0;gyro<2;gyro++)for(unsigned d=0;d<4;d++) {
        uint32_t deadline;unsigned disturbed=0,corrected=0;
        diagnostic_fixture();send_line("loop encoder off");send_line(gyro ? "loop gyro on" : "loop gyro off");
        send_line("test");send_line("dist 2000");send_line("rpm 200");send_line(directions[d]);
        CHECK(run_active && run.action_mm==0 && !run.station_enabled && !run.start_reference_valid);
        send_line("imu baud 9600");CHECK(strstr(output,"baud rejected") && diagnostic_imu_baud==115200);
        deadline=clock_ms+30000;
        while(run_active && clock_ms<deadline) {
            if(!disturbed && run.progress_mm>200) {physical_yaw+=300;disturbed=1;}
            plant_step();
            if(run.velocity_phase==VELOCITY_FORWARD || run.velocity_phase==VELOCITY_STOP_FORWARD) {
                if(gyro && run.turn_cmd_rpm<0)corrected=1;
                if(!gyro)CHECK(!run.turn_cmd_rpm);
            }
            CHECK(!arm_positions && !arm_queries && arm_enables==2);
            CHECK(arm_claw_us==826 && arm_yaw_us==1975 && arm_platform_us==648);
        }
        CHECK(!run_active && clock_ms<deadline && run.state==RUN_DONE && disturbed);
        CHECK(abs(run.progress_mm-2000)<=CAR_DISTANCE_TOLERANCE_MM+4);
        if(gyro)CHECK(corrected && fabs(physical_yaw)<=90);
        else CHECK(fabs(physical_yaw-300)<1);
        {
            unsigned batches=velocity_batches;for(unsigned i=0;i<1000;i++)plant_step();
            CHECK(!run_active && batches==velocity_batches && !arm_positions);
        }
        printf("PASS chassis %s2000mm200RPM gyro=%u: original controller/encoder stop, heading policy and fixed arm; model only\n",directions[d],gyro);
    }
}
static void turn_and_stop_tests(void)
{
    for(unsigned right=0;right<2;right++) {
        uint32_t deadline;
        diagnostic_fixture();send_line("turn");send_line("rpm 60");send_line(right ? "right90" : "left90");
        deadline=clock_ms+22000;while(run_active && clock_ms<deadline)plant_step();
        CHECK(!run_active && run.state==RUN_DONE && fabs(physical_yaw-(right ? -9000 : 9000))<=90);
        CHECK(!arm_positions && !arm_queries && arm_claw_us==826 && arm_yaw_us==1975 && arm_platform_us==648);
    }
    diagnostic_fixture();send_line("dist 2000");send_line("rpm 200");send_line("l");
    while(run_active && run.progress_mm<100)plant_step();
    queue_byte('!');plant_step();CHECK(!run_active && run.state==RUN_STOPPED && arm_stops==2);
    for(unsigned i=0;i<4;i++)CHECK(!simulated_wheel_rpm[i]);
    CHECK(arm_claw_us==826 && arm_yaw_us==1975 && arm_platform_us==648 && !arm_positions);
    for(unsigned i=0;i<1000;i++)plant_step();
    CHECK(!run_active);
    puts("PASS turns and explicit stop: original IMU turns, all motors stop request, fixed servo PWM/axis enable retained; no restart");
}
int main(void)
{
    sensor_tests();imu_and_command_tests();stability_loop_tests();translation_tests();turn_and_stop_tests();
    printf("PASS JY901B chassis diagnostic: %u assertions; no hardware accessed\n",assertions);
    return 0;
}
