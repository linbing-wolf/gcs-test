/* 真实入口与独立车辆模型；只测同步转头提速及其作用边界，不连接车辆。 */
#ifdef TURNING_SPEED_BASELINE
#include "../src/car_config.h"
#undef CAR_CORNER_SPEED_MM_S
#undef CAR_CORNER_WHEEL_ACCEL_RPM_PER_S
#undef CAR_CORNER_WHEEL_DECEL_RPM_PER_S
#define CAR_CORNER_SPEED_MM_S 150
#define CAR_CORNER_WHEEL_ACCEL_RPM_PER_S 240u
#define CAR_CORNER_WHEEL_DECEL_RPM_PER_S 200u
#endif
#define STATION_TEST_MAIN retained_turning_speed_station_suite
#include "verify_station_vision.c"

static void turning_slew_scope_tests(void)
{
    float desired[3]={100,0,0};int16_t command[3];unsigned mode;
    for(mode=0;mode<5;mode++) {
        float expected;
        reset_plant(0,1.15);memset(&run,0,sizeof(run));
        run.corner.active=1;run.corner.span_count=1;run.corner.spans[0].length=200;
        run.corner.spans[0].dyaw=mode==1 ? 0 : -9000;
        run.corner.s_mm=mode==2 ? 201 : 100;
        run.corner.reversing=(uint8_t)(mode==3);run.corner.visual_terminal=(uint8_t)(mode==4);
        run.corner.command_ms=clock_ms;clock_ms+=25;desired[0]=100;
        run_corner_slew(desired,command);
        expected=(mode==0 ? CAR_CORNER_WHEEL_ACCEL_RPM_PER_S
            : mode==3 ? CAR_DRIVE_DECEL_RPM_PER_S : CAR_ROUTE_ACCEL_RPM_PER_S)*0.025f;
        CHECK(fabsf(run.corner.command[0]-expected)<0.001f);
        run.corner.command[0]=40;desired[0]=20;clock_ms+=25;
        run_corner_slew(desired,command);
        expected=40-(mode==0 ? CAR_CORNER_WHEEL_DECEL_RPM_PER_S : CAR_DRIVE_DECEL_RPM_PER_S)*0.025f;
        CHECK(fabsf(run.corner.command[0]-expected)<0.001f);
        /* 整体请求零速仍使用原200，不能把提速参数用于停车。 */
        run.corner.command[0]=40;desired[0]=0;clock_ms+=25;
        run_corner_slew(desired,command);
        CHECK(fabsf(run.corner.command[0]-(40-CAR_DRIVE_DECEL_RPM_PER_S*0.025f))<0.001f);
    }
    puts("PASS turning slew scope: rotating arc only; fixed yaw, straight, return, visual terminal and zero request retain ordinary rates");
}

static void turning_motion_tests(void)
{
    static const double gains[]={0.6,1.15,1.8},responses[]={0.18,0.30,0.50};
    unsigned plan,k,g,r,variant,cases=0;double worstyaw=0,worstradial=0;
    for(plan=0;plan<2;plan++)for(k=0;k<3;k++)for(g=0;g<3;g++)for(r=0;r<3;r++)for(variant=0;variant<2;variant++) {
        uint32_t start,deadline,arc_ms=0;double maxyaw=0,maxradial=0;unsigned i,seen=0;
        RoadPoint target;double cx=1050,cy=k==2 ? 1050 : 1350;
        if(k==2)prepare_storage(0,"task 134+123+314+231");
        else prepare_rough(k,"task 134+123+314+231");
        run.plan=(uint8_t)plan;yaw_gain=gains[g];imu_period_ms=variant ? 40 : 20;
        for(i=0;i<4;i++)wheel_acceleration[i]=responses[r];
        if(variant){wheel_speed_scale[0]=0.93;wheel_speed_scale[2]=1.05;}
        CHECK(heading_control_start(&run.heading_control,heading_snapshot(),0,clock_ms));
        run_resume_after_station();checked_job=run.job;start=clock_ms;deadline=clock_ms+60000;
        CHECK(road_point(k==2 ? NODE_F : NODE_H,&target));
        while(run_active && run.state!=RUN_FAULT && clock_ms<deadline) {
            plant_step();
            if(run.corner.active) {
                for(i=0;i<4;i++)CHECK(abs(simulated_wheel_rpm[i])<=run_wheel_limit_rpm());
                for(i=0;i<run.corner.span_count;i++) {
                    CornerSpan *span=&run.corner.spans[i];
                    if(span->dyaw && run.corner.s_mm>=span->s && run.corner.s_mm<span->s+span->length) {
                        double yaw=abs(run.heading_control.error_cdeg)/100.0;
                        double radial=fabs(hypot(field_x-cx,field_y-cy)-span->radius);
                        if(yaw>maxyaw)maxyaw=yaw;
                        if(radial>maxradial)maxradial=radial;
                        seen=1;arc_ms++;
                    }
                }
            }
        }
        printf("TURNING_CASE plan=%u leg=%u gain=%.2f response=%.0f variant=%u total_ms=%lu arc_ms=%lu yaw_deg=%.3f radial_mm=%.3f endpoint_mm=%.3f\n",
            plan,k,gains[g],responses[r]*1000,variant,(unsigned long)(clock_ms-start),
            (unsigned long)arc_ms,maxyaw,maxradial,hypot(field_x-target.x,field_y-target.y));
        CHECK(seen && clock_ms<deadline && run.state==RUN_STATION && !run_active);
        CHECK(hypot(field_x-target.x,field_y-target.y)<CAR_ROUTE_VISUAL_WINDOW_MM+2);
        CHECK(abs(run.heading_control.error_cdeg)<=CAR_HEADING_TOLERANCE_CDEG);
        if(maxyaw>worstyaw)worstyaw=maxyaw;
        if(maxradial>worstradial)worstradial=maxradial;
        cases++;stop_request();
    }
    printf("TURNING_SUMMARY cases=%u worst_yaw_deg=%.3f worst_radial_mm=%.3f\n",cases,worstyaw,worstradial);
    CHECK(worstyaw<15 && worstradial<35);
}

static void turning_whole_route_tests(void)
{
    unsigned plan;
    for(plan=0;plan<2;plan++) {
        unsigned offset=0,last_job=0,mask=0;uint32_t start,deadline;
        prepare_arm_auto(plan);imu_period_ms=20;
        send_line("task 134+123+314+231");send_line(plan ? "runimu C" : "runimu A");
        start=clock_ms;deadline=clock_ms+2000000;
        while(run.state!=RUN_DONE && run.state!=RUN_FAULT && clock_ms<deadline) {
            serve_peer(&offset);plant_step();
            if(run_corner_rotating_span())mask|=1u<<run.corner.start_edge;
            if(run.job!=last_job){last_job=run.job;arm_positions=0;}
        }
        CHECK(clock_ms<deadline && run.state==RUN_DONE && run.job==8 && arm_ready() && arm_parked());
        CHECK(mask==((1u<<5)|(1u<<7)|(1u<<11)) && run.distance_mm==EXPECTED_FULL_ROUTE_MM);
        printf("TURNING_FULL_ROUTE plan=%u total_ms=%lu distance_mm=%u jobs=%u model_max_endpoint_mm=%.3f\n",
            plan,(unsigned long)(clock_ms-start),run.distance_mm,run.job,checkpoint_max_error_mm);
        stop_request();
    }
}

int main(int argc,char **argv)
{
    if(argc>1 && !strcmp(argv[1],"full-route")){turning_whole_route_tests();return 0;}
    CHECK(CAR_RUN_RPM==250 && CAR_LATERAL_SHORT_RPM==250 && CAR_LATERAL_LONG_RPM==200);
    CHECK(CAR_DRIVE_DECEL_RPM_PER_S==200 && CAR_DRIVE_BRAKE_LOOKAHEAD_MS==200 && CAR_ROUGH_BRAKE_LOOKAHEAD_MS==200);
    turning_slew_scope_tests();turning_motion_tests();
    printf("PASS turning corner speed: %u assertions; host model only, no hardware\n",assertions);
    return 0;
}
