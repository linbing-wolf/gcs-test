#define HEADING_TEST_MAIN retained_dynamics_suite
#include "verify_heading.c"

/* IMU两种输出频率独立；角速度来自轮惯性模型，串口量化与噪声独立于控制器。 */
static void dynamics_test(unsigned direction)
{
    const char *commands[]={"f","b","l","r"};
    uint32_t deadline,gyro_ms=0;unsigned disturbed=0,gyro_seen=0,fallback_seen=0;
    double max_error=0;
    reset_plant(0,1.15);diagnostic_init();imu_period_ms=100;
    send_line("test");send_line("dist 2000");send_line("rpm 200");send_line(commands[direction]);
    deadline=clock_ms+30000;
    while(run_active && clock_ms<deadline) {
        double turn=(actual_rpm[0]+actual_rpm[1]+actual_rpm[2]+actual_rpm[3])/4.0;
        if(clock_ms-gyro_ms>=10 && !(run.progress_mm>700 && run.progress_mm<1000)) {
            double rate=turn*yaw_gain*100.0;
            if(fabs(actual_rpm[0])>1)rate+=translation_yaw_disturbance*1000;
            rate+=(clock_ms/10%2 ? 60 : -60); /* ±0.6度/秒交替噪声 */
            wit_packet(&heading_sensor,0x52,(int16_t)lround(rate*1024.0/6250.0),clock_ms,0);
            gyro_ms=clock_ms;
        }
        if(!disturbed && run.progress_mm>250) {physical_yaw+=300;disturbed=1;}
        plant_step();
        if(run.velocity_phase==VELOCITY_FORWARD) {
            if(run.heading_control.rate_source)gyro_seen=1;
            if(run.progress_mm>800 && !run.heading_control.rate_source)fallback_seen=1;
            if(fabs(physical_yaw)>max_error)max_error=fabs(physical_yaw);
        }
        CHECK(abs(run.turn_cmd_rpm)<=CAR_HEADING_DRIVE_TRIM_RPM);
        CHECK(!arm_positions && arm_claw_us==826 && arm_yaw_us==1975 && arm_platform_us==648);
    }
    CHECK(!run_active && run.state==RUN_DONE && clock_ms<deadline);
    CHECK(gyro_seen && fallback_seen && disturbed && max_error<500 && fabs(physical_yaw)<=90);
    CHECK(abs(run.progress_mm-2000)<=CAR_DISTANCE_TOLERANCE_MM+4);
    printf("PASS dynamics %s: yaw10Hz gyro100Hz, noise/dropout, max_yaw=%.2fdeg end=%.2fdeg; model only\n",
        commands[direction],max_error/100,physical_yaw/100);
}
static void acceleration_test(void)
{
    uint8_t frame[11]={0x55,0x51,0xff,0x7f,0,0x80,0,8,0,0,0};
    reset_plant(0,1.15);
    for(unsigned i=0;i<10;i++)frame[10]=(uint8_t)(frame[10]+frame[i]);
    for(unsigned i=0;i<11;i++)heading_sensor_byte(&heading_sensor,frame[i],clock_ms);
    CHECK(heading_sensor.acceleration.valid && heading_sensor.acceleration.x_mg==15999
        && heading_sensor.acceleration.y_mg==-16000 && heading_sensor.acceleration.z_mg==1000);
    CHECK(heading_sensor.sample.sequence==1 && !heading_sensor.rate.valid);
    frame[10]++;for(unsigned i=0;i<11;i++)heading_sensor_byte(&heading_sensor,frame[i],clock_ms);
    CHECK(heading_sensor.acceleration.sequence==1 && heading_sensor.checksum_errors==1);
    heading_sensor_invalidate(&heading_sensor);CHECK(!heading_sensor.acceleration.valid);
}
int main(void)
{
    acceleration_test();for(unsigned d=0;d<4;d++)dynamics_test(d);
    printf("PASS asynchronous sensor dynamics: %u assertions; no hardware accessed\n",assertions);return 0;
}
