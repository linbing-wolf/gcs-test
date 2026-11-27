#define HEADING_TEST_MAIN retained_nine_axis_suite
#include "verify_heading.c"

static void sensor_frame(uint8_t kind,int16_t a,int16_t b,int16_t c,int16_t d,uint8_t bad)
{
    int16_t values[4]={a,b,c,d};uint8_t frame[11]={0x55,0,0,0,0,0,0,0,0,0,0};frame[1]=kind;
    for(unsigned i=0;i<4;i++) {frame[2+i*2]=(uint8_t)values[i];frame[3+i*2]=(uint8_t)((uint16_t)values[i]>>8);}
    for(unsigned i=0;i<10;i++)frame[10]=(uint8_t)(frame[10]+frame[i]);
    frame[10]+=bad;
    for(unsigned i=0;i<11;i++)heading_sensor_byte(&heading_sensor,frame[i],clock_ms);
}
int main(void)
{
    reset_plant(0,1.15);diagnostic_init();send_line("imu");
    CHECK(strstr(output,"gyro_fresh=0 gyro_age_ms=-1 gyro_axes=0 gyro_sensor_cdeg_s=NA"));
    CHECK(strstr(output,"mag_fresh=0 mag_age_ms=-1 mag_lsb=NA"));
    CHECK(strstr(output,"temp_fresh=0 temp_age_ms=-1 temp_cdeg=NA"));
    CHECK(strstr(output,"quat_fresh=0 quat_age_ms=-1 quat_q15=NA"));
    CHECK(strstr(output,"+X=forward +Y=left +Z=up; verified by f/r/l direction tests"));
    clock_ms=1000;sensor_frame(0x52,32767,-32768,1024,0,0);
    CHECK(heading_sensor.rate.axes_valid==7 && heading_sensor.rate.x_cdeg_s==199993
        && heading_sensor.rate.y_cdeg_s==-200000 && heading_sensor.rate.rate_cdeg_s==6250);
    sensor_frame(0x52,0,0,0,0,1);
    CHECK(heading_sensor.rate.sequence==1 && heading_sensor.checksum_errors==1);
    sensor_frame(0x51,1024,-512,2048,-1234,0);
    CHECK(heading_sensor.acceleration.x_mg==500 && heading_sensor.acceleration.y_mg==-250
        && heading_sensor.acceleration.z_mg==1000 && heading_sensor.temperature.cdeg==-1234);
    sensor_frame(0x54,3,-4,12,3000,0);
    CHECK(heading_sensor.magnetic.valid && heading_sensor.magnetic.x_lsb==3
        && heading_sensor.magnetic.y_lsb==-4 && heading_sensor.magnetic.z_lsb==12);
    CHECK(heading_sensor.temperature.cdeg==-1234 && heading_sensor.temperature.sequence==1);
    sensor_frame(0x59,16384,-16384,0,23170,0);
    CHECK(heading_sensor.quaternion.valid && heading_sensor.quaternion.q15[0]==16384
        && heading_sensor.quaternion.q15[1]==-16384 && heading_sensor.quaternion.q15[3]==23170);
    sensor_frame(0x54,0,0,0,0,1);sensor_frame(0x59,0,0,0,0,1);
    CHECK(heading_sensor.magnetic.sequence==1 && heading_sensor.quaternion.sequence==1 && heading_sensor.checksum_errors==3);
    send_line("imu");
    CHECK(strstr(output,"gyro_fresh=1 gyro_age_ms=0 gyro_axes=7 gyro_sensor_cdeg_s=199993,-200000,6250"));
    CHECK(strstr(output,"mag_fresh=1 mag_age_ms=0 mag_lsb=3,-4,12 mag_norm_lsb=13"));
    CHECK(strstr(output,"temp_fresh=1 temp_age_ms=0 temp_cdeg=-1234"));
    CHECK(strstr(output,"quat_fresh=1 quat_age_ms=0 quat_q15=16384,-16384,0,23170"));
    clock_ms=1101;sample_yaw(500);send_line("imu");
    CHECK(strstr(output,"gyro_fresh=0 gyro_age_ms=101") && strstr(output,"mag_fresh=1 mag_age_ms=101"));
    clock_ms=1251;sample_yaw(500);sensor_frame(0x52,0,0,1,0,0);send_line("imu");
    CHECK(strstr(output,"mag_fresh=0 mag_age_ms=251") && strstr(output,"temp_fresh=0 temp_age_ms=251")
        && strstr(output,"quat_fresh=0 quat_age_ms=251"));
    CHECK(heading_sensor.magnetic.received_ms==1000 && heading_sensor.quaternion.received_ms==1000);
    heading_sensor_publish_rate(&heading_sensor,100,clock_ms);send_line("imu");
    CHECK(strstr(output,"gyro_axes=4 gyro_sensor_cdeg_s=NA,NA,100"));
    sensor_frame(0x54,-32768,-32768,-32768,0,0);send_line("imu");
    CHECK(strstr(output,"mag_norm_lsb=56756")); /* 三轴平方和超过int32，不能整型溢出。 */
    heading_sensor_invalidate(&heading_sensor);send_line("imu");
    CHECK(!heading_sensor.magnetic.valid && !heading_sensor.temperature.valid && !heading_sensor.quaternion.valid);
    CHECK(strstr(output,"mag_fresh=0 mag_age_ms=-1 mag_lsb=NA"));
    sensor_frame(0x51,0,0,2048,2500,0);sensor_frame(0x54,3,4,0,0,0);
    run.velocity_phase=VELOCITY_FORWARD;run.log_ms=clock_ms-CAR_CONTROL_LOG_MS;
    output_size=0;output[0]=0;run_velocity_log();
    CHECK(strstr(output,"acc_mg=0,0,1000") && strstr(output,"mag_lsb=3,4,0") && strstr(output,"temp_cdeg=2500"));
    CHECK(!velocity_count && !arm_positions && !arm_queries && arm_claw_us==826 && arm_yaw_us==1975 && arm_platform_us==648);
    heading_sensor_reset(&heading_sensor);
    CHECK(!heading_sensor.magnetic.valid && !heading_sensor.temperature.valid && !heading_sensor.quaternion.valid);
    printf("PASS nine-axis observation: %u assertions; full gyro/magnetometer/temperature/quaternion, independent freshness, no motion effect\n",assertions);return 0;
}
