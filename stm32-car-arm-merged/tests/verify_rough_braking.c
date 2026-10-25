/* 以首次到位峰值验收，不能用超程后的回补冒充不越界。 */
#define STATION_TEST_MAIN retained_rough_brake_station_suite
#include "verify_station_vision.c"

static void incoming(unsigned plan,unsigned batch,double response)
{
    RoadPoint f;unsigned i;
    prepare_rough(batch,"task 134+123+314+231");
    CHECK(road_point(NODE_F,&f));run.plan=(uint8_t)plan;
    run.edge=(uint8_t)(batch ? 9 : 3);run.job=(uint8_t)(batch ? 4 : 1);
    run.heading=run.target_heading=HEAD_X_NEG;run.yaw_zero_cdeg=9000;
    field_x=planned_x=f.x;field_y=planned_y=f.y;initial_field_heading=HEAD_X_NEG;checked_job=run.job;
    CHECK(road_plan(NODE_E,NODE_F,&run.road));
    for(i=0;i<4;i++)wheel_acceleration[i]=response;
    CHECK(heading_control_start(&run.heading_control,heading_snapshot(),0,clock_ms));
    run_resume_after_station();checked_job=run.job;
}

static void first_arrival_tests(void)
{
    unsigned plan,batch,r,t;
    const double responses[]={0.18,0.30};
    const unsigned periods[]={10,20,50,100};
    for(plan=0;plan<2;plan++)for(batch=0;batch<2;batch++)for(r=0;r<2;r++)for(t=0;t<4;t++) {
        RoadPoint d;uint32_t start,deadline;double peak=-10000;unsigned reversed=0;
        incoming(plan,batch,responses[r]);imu_period_ms=periods[t];CHECK(road_point(NODE_D,&d));
        start=clock_ms;deadline=clock_ms+60000;
        while(run.state!=RUN_FAULT && !(run.state==RUN_STATION && !run_active) && clock_ms<deadline) {
            plant_step();if(field_y-d.y>peak)peak=field_y-d.y;
            if(run.edge==(batch ? 10u : 4u) && run.left_cmd_rpm<0)reversed=1;
            CHECK(!vision_tx_count);
        }
        printf("ROUGH_FIRST plan=%u batch=%u response=%.2f imu_ms=%u elapsed_ms=%lu peak_mm=%.3f end_mm=%.3f reverse=%u\n",
            plan,batch,responses[r],periods[t],(unsigned long)(clock_ms-start),peak,field_y-d.y,reversed);
        CHECK(clock_ms<deadline && run.state==RUN_STATION && !run_active && !reversed);
        /* 使用既有1mm位置预算；这不是禁入边界的零越界保证。 */
        CHECK(peak<=CAR_DISTANCE_TOLERANCE_MM && fabs(field_y-d.y)<3
            && abs(run.progress_mm-(int32_t)run.action_mm)<=CAR_DISTANCE_TOLERANCE_MM);
        CHECK(!run.route_terminal && !run.corner.visual_terminal);
        deadline=clock_ms+10000;
        while(vision_tx_count<6 && clock_ms<deadline) {
            plant_step();CHECK(run.state!=RUN_FAULT);
        }
        CHECK(clock_ms<deadline && vision_tx_bytes[3]=='D');stop_request();
    }
}

static void overrun_must_not_pass_tests(void)
{
    unsigned variant,i;
    for(variant=0;variant<3;variant++) {
        RoadPoint d;uint32_t deadline;unsigned injected=0,reversed=0;double peak=-10000;
        incoming(0,0,variant==0 ? 0.12 : 0.18);CHECK(road_point(NODE_D,&d));deadline=clock_ms+60000;
        while(run.state!=RUN_FAULT && !(run.state==RUN_STATION && !run_active) && clock_ms<deadline) {
            if(variant && !injected && run.edge==4 && run.progress_mm>=run.action_mm-4) {
                static const int8_t signs[4]={-1,-1,1,1};
                if(variant==1)for(i=0;i<4;i++) {
                    encoder_value[i]+=encoder_polarity[i]*signs[i]*100.0*CALIBRATED_PPM_X*CAR_ENCODER_COUNTS_PER_REV/CAR_POSITION_PULSES_PER_REV;
                    simulated_encoder[i]=(int64_t)llround(encoder_value[i]);
                }
                field_y+=100;injected=1;
            }
            plant_step();if(field_y-d.y>peak)peak=field_y-d.y;
            if(run.edge==4 && run.left_cmd_rpm<0)reversed=1;
        }
        CHECK(clock_ms<deadline);
        if(variant<2) {
            CHECK(run.state==RUN_STATION && reversed && peak>90);
            printf("ROUGH_LIMIT variant=%u peak_before_correction_mm=%.3f; no automatic fault stop; NOT a successful no-entry result\n",variant,peak);
        } else {
            CHECK(run.state==RUN_STATION && !reversed && field_y-d.y>90);
            printf("ROUGH_UNOBSERVABLE ground_only100mm remains=%.3f; cannot establish physical no-entry guarantee\n",field_y-d.y);
        }
        stop_request();
    }
}

static void scope_tests(void)
{
    CHECK(CAR_DRIVE_DECEL_RPM_PER_S==240 && CAR_ROUGH_BRAKE_LOOKAHEAD_MS==200);
    reset_heading_fixture();run.station_enabled=1;run.job=2;run.plan=0;run.segment=0;
    CHECK(road_plan(NODE_E,NODE_D,&run.road));CHECK(run_rough_arrival() && !run_visual_arrival());
    run.station_shift=1;CHECK(!run_rough_arrival());run.station_shift=0;
    run.translation_only=1;CHECK(!run_rough_arrival());run.translation_only=0;
    run.obstacle_phase=1;CHECK(!run_rough_arrival());run.obstacle_phase=0;
    run.station_enabled=0;CHECK(!run_rough_arrival());run.station_enabled=1;
    run.job=1;CHECK(road_plan(NODE_E,NODE_F,&run.road));CHECK(!run_rough_arrival() && run_visual_arrival());
    run.job=3;CHECK(road_plan(NODE_E,NODE_H,&run.road));CHECK(!run_rough_arrival() && run_visual_arrival());
}

int main(void)
{
    scope_tests();first_arrival_tests();overrun_must_not_pass_tests();
    printf("PASS rough braking: %u assertions; first-stop model limits and negative controls are separate from hardware acceptance\n",assertions);
    return 0;
}
