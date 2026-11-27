#define HEADING_TEST_MAIN retained_async_heading_suite
#include "verify_heading.c"

static void async_rate_test(void)
{
    HeadingControl c;HeadingSample s;
    reset_plant(0,1.15);clock_ms=1000;sample_yaw(0);s=heading_snapshot();
    CHECK(heading_control_start(&c,s,0,clock_ms));
    clock_ms=1020;sample_yaw(0);CHECK(heading_control_update(&c,heading_snapshot(),clock_ms)!=HEADING_FAULT);
    /* 角度还没更新，但陀螺仪已经报告旋转；D项必须及时响应。 */
    clock_ms=1045;heading_control_measured_rate(&c,5000,1);
    CHECK(heading_control_update(&c,heading_snapshot(),clock_ms)==HEADING_WAIT);
    CHECK(c.rate_cdeg_s>0 && heading_control_drive_rpm(&c,20)<0);
}
static void tilt_test(void)
{
    reset_plant(0,1.15);clock_ms=1000;
    heading_sensor_publish_attitude(&heading_sensor,3100,0,500,clock_ms);
    CHECK(heading_snapshot().valid); /* 倾斜不等于串口角度帧失效。 */
}
static void full_rate_test(void)
{
    reset_plant(0,1.15);wit_packet(&heading_sensor,0x52,32767,clock_ms,0);
    CHECK(heading_sensor.rate.rate_cdeg_s==199993);
}
int main(int argc,char **argv)
{
    unsigned mode=argc>1 ? (unsigned)atoi(argv[1]) : 0;
    if(mode==0)async_rate_test();else if(mode==1)tilt_test();else full_rate_test();
    printf("PASS async feedback regression%u: %u assertions\n",mode,assertions);return 0;
}
