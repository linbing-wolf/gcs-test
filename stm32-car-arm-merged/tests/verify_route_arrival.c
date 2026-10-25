/* 独立车辆积分验证：C前中段侧移、到站残差交视觉，以及实车分段计时。 */
#define STATION_TEST_MAIN retained_arrival_suite
#include "verify_station_vision.c"

static void timing_tests(void)
{
    unsigned i;uint32_t sum=0;
    reset_plant(0,1.15);memset(&route_timing,0,sizeof(route_timing));
    route_timing.active=1;route_timing.last_ms=clock_ms;run_active=1;run.state=RUN_MOVING;
    clock_ms+=100;run_timing_poll();CHECK(route_timing.total[0]==100);
    run_active=0;run.state=RUN_STATION;run_timing_poll();clock_ms+=200;run_timing_poll();
    CHECK(route_timing.total[1]==200);
    run.job=1;vision_request_pick(1,clock_ms);run_timing_poll();clock_ms+=300;run_timing_poll();
    CHECK(route_timing.total[4]==300);
    station_parent=run;run.station_shift=1;run_active=1;run_timing_poll();clock_ms+=400;run_timing_poll();
    CHECK(route_timing.total[2]==400);
    run.station_shift=0;run_active=0;run.state=RUN_DONE;run_timing_poll();
    for(i=0;i<6;i++)sum+=route_timing.total[i];
    CHECK(!route_timing.active && sum==1000 && strstr(output,"[run timing] total"));
    memset(&route_timing,0,sizeof(route_timing));route_timing.active=1;route_timing.category=2;route_timing.last_ms=clock_ms;
    run.station_shift=1;run.state=RUN_STOPPED;clock_ms+=10;run_timing_poll();
    CHECK(!route_timing.active && route_timing.total[2]==10);
    memset(&route_timing,0,sizeof(route_timing));route_timing.active=1;route_timing.last_ms=UINT32_MAX-5u;
    run.station_shift=0;run_active=1;run.state=RUN_MOVING;clock_ms=10;run_timing_poll();
    CHECK(route_timing.total[0]==16);route_timing.active=0;
    puts("PASS route wall-clock timing: road/scan/vision/station child intervals count once across job changes; terminal summary closes without motion or task advancement");
}

static void c_blend_tests(void)
{
    unsigned variant;
    for(variant=0;variant<3;variant++) {
        uint32_t deadline,start;unsigned seen=0,mixed=0;double peak_x=0,b_peak=0;
        prepare_storage(1,"task 134+123+314+231");imu_period_ms=variant ? 50 : 20;
        if(variant==2){encoder_polarity[0]=-1;wheel_speed_scale[0]=0.96;wheel_speed_scale[2]=1.03;}
        CHECK(heading_control_start(&run.heading_control,heading_snapshot(),0,clock_ms));
        run_resume_after_station();checked_job=run.job;start=clock_ms;deadline=clock_ms+60000;
        while(run_active && run.state!=RUN_FAULT && clock_ms<deadline) {
            plant_step();if(field_x>peak_x)peak_x=field_x;
            if(field_y<1400 && field_x>b_peak)b_peak=field_x;
            if(run.corner.active && !seen) {
                CornerSpan *span=&run.corner.spans[run.corner.span_count-1u];unsigned j;
                CHECK(run.corner.legs[1].length==990 && span->blend_x==60 && span->blend_y==0 && span->blend_length==600);
                for(j=0;j<=120;j++) {
                    float d=span->length*j/120.0f;
                    CornerPoint p=run_corner_point(span->s+d);
                    CHECK(p.x>=1770-0.001f && p.x<=1830+0.001f && p.dx>=-0.001f);
                    if(d>=600)CHECK(fabs(p.x-1830)<0.001 && fabs(p.dx)<0.001);
                }
                seen=1;
            }
            if(run.edge==15 && run.corner.active) {
                CHECK(run.segment==0 && run.heading==HEAD_Y_POS);
                if(run.forward_cmd_rpm>0 && run.left_cmd_rpm>0)mixed=1;
            }
        }
        if(run.state!=RUN_DONE)fprintf(stderr,"C_BLEND variant=%u %s\n",variant,output);
        CHECK(clock_ms<deadline && run.state==RUN_DONE && seen && mixed && run.job==8);
        printf("C_BLEND_METRIC variant=%u B_peak=%.3f peak=%.3f endpoint=%.3f,%.3f estimate=%.3f,%.3f\n",variant,b_peak,peak_x,field_x,field_y,run.corner.x_mm,run.corner.y_mm);
        CHECK(hypot(field_x-2250,field_y-2230)<8 && peak_x<2258 && b_peak<2230);
        CHECK(fabs(run.corner.x_mm-1830)<=1 && fabs(run.corner.y_mm+1030)<=1);
        printf("C_BLEND variant=%u elapsed_s=%.3f B_region_peak_x=%.3f whole_peak_x=%.3f endpoint_mm=%.3f,%.3f; fixed heading, no final lateral segment\n",
            variant,(clock_ms-start)/1000.0,b_peak,peak_x,field_x,field_y);
        stop_request();
    }
}

static void visual_arrival_tests(void)
{
    unsigned mode;
    for(mode=0;mode<2;mode++) {
        uint32_t deadline;unsigned injected=0,latched=0,latched_batch=0;
        if(!mode) {
            prepare_storage(0,"task 134+123+314+231");
            run.edge=3;run.job=1;run.plan=0;run.heading=run.target_heading=HEAD_X_NEG;run.yaw_zero_cdeg=9000;
            field_x=planned_x=1200;field_y=planned_y=390;initial_field_heading=HEAD_X_NEG;checked_job=1;
            CHECK(road_plan(NODE_E,NODE_F,&run.road));
        } else prepare_rough(0,"task 134+123+314+231");
        imu_period_ms=20;CHECK(heading_control_start(&run.heading_control,heading_snapshot(),0,clock_ms));
        run_resume_after_station();checked_job=run.job;deadline=clock_ms+60000;
        while(run_active && run.state!=RUN_FAULT && clock_ms<deadline) {
            unsigned i;
            if(!injected && ((!mode && run.edge==4 && run.velocity_phase==VELOCITY_FORWARD && run.progress_mm>run.action_mm-10)
                || (mode && run.corner.active && run.corner.s_mm>run.corner.total_mm-10))) {
                if(!mode) {
                    static const int8_t left[4]={-1,-1,1,1};
                    for(i=0;i<4;i++){encoder_value[i]+=encoder_polarity[i]*left[i]*20*CALIBRATED_PPM_X*CAR_ENCODER_COUNTS_PER_REV/CAR_POSITION_PULSES_PER_REV;simulated_encoder[i]=(int64_t)llround(encoder_value[i]);}
                    field_y+=20;
                } else {field_x-=20;run.corner.y_mm+=20;}
                injected=1;
            }
            plant_step();
            if(run.route_terminal || run.corner.visual_terminal) {
                if(!latched){latched=1;latched_batch=velocity_batches;}
                if(!mode && run.velocity_phase==VELOCITY_STOP_FORWARD)CHECK(run.left_cmd_rpm>=0);
                if(mode && velocity_batches>latched_batch)CHECK(!run.forward_ref_rpm && !run.left_ref_rpm);
            }
        }
        if(!mode) {
            CHECK(clock_ms<deadline && run.state==RUN_STATION && injected && !latched && !run_active);
            CHECK(abs(run.progress_mm-(int32_t)run.action_mm)<=CAR_DISTANCE_TOLERANCE_MM);
        } else {
            CHECK(clock_ms<deadline && run.state==RUN_STATION && injected && latched && !run_active);
            CHECK(field_x<415 && abs(run.heading_control.error_cdeg)<=CAR_HEADING_TOLERANCE_CDEG);
        }
        CHECK(!run.vision_debug); /* Route arrival remains an automatic station handoff. */
        printf("VISUAL_ARRIVAL mode=%u actual_mm=%.3f,%.3f; D retains position correction without fault stop, H keeps visual residual policy\n",mode,field_x,field_y);
        stop_request();
    }
    reset_heading_fixture();run.station_enabled=1;run.job=7;run.plan=1;run.segment=0;
    CHECK(road_plan(NODE_B,NODE_C,&run.road));CHECK(!run_visual_arrival());
    run.job=2;CHECK(road_plan(NODE_E,NODE_D,&run.road));CHECK(!run_visual_arrival());
    run.station_shift=1;CHECK(!run_visual_arrival());run.station_shift=0;run.station_enabled=0;CHECK(!run_visual_arrival());
    puts("PASS visual handoff scope: road arrival only; C home, station MOVE/MOVEXY, non-auto and in-route waypoints retain position completion");
}

int main(void)
{
    timing_tests();c_blend_tests();visual_arrival_tests();
    printf("PASS route arrival integration: %u assertions; observed encoder motion/independent plant only, not ground-slip or vehicle acceptance\n",assertions);
    return 0;
}
