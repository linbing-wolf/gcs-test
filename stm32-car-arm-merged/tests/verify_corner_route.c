/* 复用真实入口、CAN替身及独立车辆积分；当前模式与编译期r72策略对照，不连接硬件。 */
#define STATION_TEST_MAIN retained_station_suite
#include "verify_station_vision.c"

static void launch_from_d(unsigned plan,unsigned batch,double gain,unsigned polarity)
{
    unsigned i;
    prepare_rough(batch,"task 134+123+314+231");run.plan=(uint8_t)plan;
    yaw_gain=gain;imu_period_ms=20;translation_yaw_disturbance=0;
    for(i=0;i<4u;i++)encoder_polarity[i]=(polarity&(1u<<i)) ? -1 : 1;
    CHECK(heading_control_start(&run.heading_control,heading_snapshot(),0,clock_ms));
    run_resume_after_station();checked_job=run.job;
    CHECK(run_active && run.job==(batch ? 6u : 3u));
}

static void corner_transit_tests(void)
{
    const double gains[]={0.6,1.15,1.8};
    unsigned plan,batch,g,p;
    for(plan=0;plan<2;plan++)for(batch=0;batch<2;batch++)
    for(g=0;g<3;g++)for(p=0;p<6;p++) {
        uint32_t start,deadline,last_batch_ms=0,last_batch=0;
        int16_t previous[4]={0};unsigned seen=0,rotating_moving=0,arc_samples=0;
        double max_yaw_error=0,max_arc_error=0;
        RoadPoint h;
        launch_from_d(plan,batch,gains[g],p==5 ? 10 : p ? 1u<<(p-1u) : 0);
        if(p==5){wheel_speed_scale[0]=0.93;wheel_speed_scale[2]=1.05;}
        start=clock_ms;deadline=clock_ms+60000;last_batch=velocity_batches;
        while(run_active && run.state!=RUN_FAULT && clock_ms<deadline) {
            unsigned i;
            plant_step();
            if(run.corner.active) {
                double error=abs(run.heading_control.error_cdeg)/100.0;
                seen=1;if(error>max_yaw_error)max_yaw_error=error;
                if(velocity_batches!=last_batch) {
                    if(last_batch_ms)for(i=0;i<4u;i++) {
                        double elapsed=clock_ms-last_batch_ms;
                        if(elapsed>CAR_DRIVE_SLEW_MAX_DT_MS)elapsed=CAR_DRIVE_SLEW_MAX_DT_MS;
                        CHECK(abs(simulated_wheel_rpm[i]-previous[i])<=ceil(CAR_DRIVE_DECEL_RPM_PER_S*elapsed/1000.0)+2);
                    }
                    memcpy(previous,simulated_wheel_rpm,sizeof(previous));last_batch_ms=clock_ms;last_batch=velocity_batches;
                }
                for(i=0;i<4u;i++)CHECK(abs(simulated_wheel_rpm[i])<=CAR_WHEEL_MAX_RPM);
                if(run.corner.s_mm>660 && run.corner.s_mm<895) {
                    /* 独立地图圆心(1050,1350)、半径150；按实际坐标核对，不能只看控制器自身里程。 */
                    double deviation=fabs(hypot(field_x-1050,field_y-1350)-150);
                    double fw=(-actual_rpm[0]+actual_rpm[1]+actual_rpm[2]-actual_rpm[3])/4;
                    double le=(-actual_rpm[0]-actual_rpm[1]+actual_rpm[2]+actual_rpm[3])/4;
                    if(deviation>max_arc_error)max_arc_error=deviation;
                    CHECK(hypot(fw,le)>0.5);arc_samples++;
                    if(fabs(physical_yaw)>500 && run.turn_cmd_rpm && (run.forward_cmd_rpm || run.left_cmd_rpm))rotating_moving=1;
                }
            }
        }
        if(run.state!=RUN_STATION)fprintf(stderr,"CORNER transit plan=%u batch=%u gain=%.2f p=%u state=%u %s\n",plan,batch,gains[g],p,run.state,output);
        CHECK(clock_ms<deadline && run.state==RUN_STATION && !run_active && run.heading==HEAD_Y_POS);
        CHECK(road_point(NODE_H,&h));
        if(hypot(field_x-h.x,field_y-h.y)>=(CAR_RUN_CORNER_ENABLED ? 5.0 : 80.0))
            fprintf(stderr,"CORNER endpoint enabled=%u plan=%u batch=%u gain=%.2f polarity=%u actual=%f,%f estimated=%f,%f\n",
                (unsigned)CAR_RUN_CORNER_ENABLED,plan,batch,gains[g],p,field_x,field_y,run.corner.x_mm,run.corner.y_mm);
        CHECK(hypot(field_x-h.x,field_y-h.y)<(CAR_RUN_CORNER_ENABLED ? 5.0 : 80.0));
        CHECK(abs(heading_wrap_cdeg(wrapped_yaw(physical_yaw)+9000))<=CAR_HEADING_TOLERANCE_CDEG+2);
        if(CAR_RUN_CORNER_ENABLED) {
            CHECK(seen && rotating_moving && arc_samples>100 && run.corner.passed && run.corner.arriving);
            CHECK(max_yaw_error<15 && max_arc_error<35);
            CHECK(fabs(run.corner.x_mm-810)<=CAR_DISTANCE_TOLERANCE_MM);
            CHECK(fabs(run.corner.y_mm-780)<=CAR_DISTANCE_TOLERANCE_MM);
        } else CHECK(!seen);
        if(!p)printf("CORNER_TIME enabled=%u plan=%u batch=%u gain=%.2f total_s=%.3f model_error_mm=%.3f max_track_yaw_deg=%.2f max_arc_error_mm=%.2f\n",
            (unsigned)CAR_RUN_CORNER_ENABLED,plan,batch,gains[g],(clock_ms-start)/1000.0,
            hypot(field_x-h.x,field_y-h.y),max_yaw_error,max_arc_error);
        stop_request();
    }
    puts("PASS corner transit: A/C, both storage arrivals, three yaw responses, encoder polarity and unequal wheel speeds; independently measured map endpoint/yaw and nonzero translation through turn");
}

static void wait_in_corner(unsigned stage)
{
    uint32_t deadline;
    launch_from_d(0,0,1.15,0);deadline=clock_ms+20000;
    while(run_active && clock_ms<deadline) {
        plant_step();
        if(run.corner.active && run.corner.polarity_ready
            && run.corner.s_mm>(stage==0 ? 250 : stage==1 ? 750 : 1000))break;
    }
    CHECK(clock_ms<deadline && run.corner.active && !run.corner.arriving);
}

static void corner_recovery_tests(void)
{
    unsigned variant,stage;
    if(!CAR_RUN_CORNER_ENABLED)return;
    for(variant=0;variant<6;variant++) {
        uint32_t deadline;unsigned job;
        wait_in_corner(1);job=run.job;
        if(variant==0 || variant==1) {
            if(variant==0)missing_feedback_motor=4;
            else {frozen_imu=1;heading_sensor_invalidate(&heading_sensor);}
            tick(1800);CHECK(run_active && run.corner.active && run.state!=RUN_FAULT && run.job==job);
            CHECK(!simulated_wheel_rpm[0] && !simulated_wheel_rpm[1] && !simulated_wheel_rpm[2] && !simulated_wheel_rpm[3]);
            missing_feedback_motor=0;frozen_imu=0;deadline=clock_ms+30000;
            while(run_active && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
            if(run.state!=RUN_STATION)fprintf(stderr,"CORNER resume variant=%u s=%f xy=%f,%f target=%ld progress=%ld %s\n",variant,run.corner.s_mm,run.corner.x_mm,run.corner.y_mm,(long)run.heading_control.target_cdeg,(long)run.heading_control.progress_cdeg,output);
            CHECK(clock_ms<deadline && run.state==RUN_STATION && !run.corner.active && run.job==job);
            CHECK(hypot(field_x-420,field_y-1200)<5.0);
        } else {
            if(variant==2)send_line("!");
            if(variant==3){bus_off=1;plant_step();}
            if(variant==4){motor_flags[2]=5;tick(200);}
            if(variant==5){frozen_encoder=1;tick(2000);}
            CHECK(!run_active && (run.state==RUN_STOPPED || run.state==RUN_FAULT));
            CHECK(!simulated_wheel_rpm[0] && !simulated_wheel_rpm[1] && !simulated_wheel_rpm[2] && !simulated_wheel_rpm[3]);
            tick(500);CHECK(!run_active && run.job==job);
        }
        stop_request();
    }
    for(stage=0;stage<3;stage++) {
        uint32_t deadline;unsigned job;
        wait_in_corner(stage);job=run.job;send_line("obstacle");deadline=clock_ms+60000;
        CHECK(run.obstacle_node==NODE_D && run.state==RUN_OBSTACLE_STOP);
        while(run_active && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        if(run.state!=RUN_WAIT_OBSTACLE)fprintf(stderr,"CORNER return stage=%u s=%f xy=%f,%f err=%ld %s\n",stage,run.corner.s_mm,run.corner.x_mm,run.corner.y_mm,(long)run.heading_control.error_cdeg,output);
        CHECK(clock_ms<deadline && run.state==RUN_WAIT_OBSTACLE && !run_active && !run.corner.active);
        CHECK(run.edge==5 && run.job==job && run.heading==HEAD_X_POS && arm_ready() && arm_parked());
        CHECK(hypot(field_x-1200,field_y-2010)<5.0 && abs(wrapped_yaw(physical_yaw))<=82);
        tick(500);CHECK(!run_active && run.state==RUN_WAIT_OBSTACLE);
        stop_request();
    }
    puts("PASS corner recovery: lost wheel/IMU feedback stops and resumes same target; emergency/bus-off/driver stall/encoder stall stop; external obstacle at entry/arc/exit returns along curved path to D and waits without resuming");
}

static void corner_delay_tests(void)
{
    unsigned variant;
    if(!CAR_RUN_CORNER_ENABLED)return;
    for(variant=0;variant<2u;variant++) {
        uint32_t deadline;
        wait_in_corner(1);
        if(!variant){missing_velocity_ack=3;tick(500);missing_velocity_ack=0;}
        else simulate_uart_cost=1;
        deadline=clock_ms+30000;
        while(run_active && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        if(run.state!=RUN_STATION)fprintf(stderr,"CORNER delay variant=%u %s\n",variant,output);
        CHECK(clock_ms<deadline && run.state==RUN_STATION && !run.corner.active);
        CHECK(hypot(field_x-420,field_y-1200)<5.0);
        simulate_uart_cost=0;stop_request();
    }
    puts("PASS corner delays: lost velocity ACK for500ms and115200baud log cost retain endpoint/yaw completion");
}

static void corner_whole_route_tests(void)
{
    unsigned plan;
    for(plan=0;plan<2;plan++) {
        unsigned offset=0,last_job=0,count=0,was_active=0;uint32_t deadline;
        prepare_arm_auto(plan);imu_period_ms=20;
        send_line("task 134+123+314+231");send_line(plan ? "runimu C" : "runimu A");deadline=clock_ms+2000000;
        while(run.state!=RUN_DONE && run.state!=RUN_FAULT && clock_ms<deadline) {
            serve_peer(&offset);plant_step();
            if(run.corner.active) {
                CHECK(run.job==3 || run.job==6);
                if(!was_active)count++;
            }
            was_active=run.corner.active;
            if(run.job!=last_job){last_job=run.job;arm_positions=0;}
        }
        if(run.state!=RUN_DONE)fprintf(stderr,"CORNER full plan=%u %s\n",plan,output);
        CHECK(clock_ms<deadline && run.state==RUN_DONE && run.job==8 && arm_ready() && arm_parked());
        CHECK(count==(CAR_RUN_CORNER_ENABLED ? 2u : 0u) && run.distance_mm==EXPECTED_FULL_ROUTE_MM);
        printf("PASS corner full flow plan=%u enabled=%u corners=%u logical_distance_mm=%u; eight jobs unchanged\n",plan,(unsigned)CAR_RUN_CORNER_ENABLED,count,run.distance_mm);
        stop_request();
    }
}

static void corner_yaw_wrap_tests(void)
{
    static const int16_t origins[]={-15000,17950};
    unsigned i;
    if(!CAR_RUN_CORNER_ENABLED)return;
    for(i=0;i<2u;i++) {
        uint32_t deadline;
        launch_from_d(0,0,1.15,0);
        physical_yaw=origin_yaw=origins[i];sample_yaw(origins[i]);
        run.yaw_zero_cdeg=heading_wrap_cdeg(origins[i]-CAR_MAP_YAW_SIGN*(int32_t)HEAD_X_POS*9000);
        CHECK(heading_control_start(&run.heading_control,heading_snapshot(),0,clock_ms));
        deadline=clock_ms+30000;
        while(run_active && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && run.state==RUN_STATION && run.heading==HEAD_Y_POS);
        CHECK(hypot(field_x-420,field_y-1200)<5.0);
        CHECK(abs(heading_wrap_cdeg(wrapped_yaw(physical_yaw)-origins[i]+9000))<=82);
        stop_request();
    }
    puts("PASS corner yaw wrap: absolute IMU origins near+/-180 degrees preserve map endpoint and clockwise90-degree target");
}

static void corner_threshold_recovery_tests(void)
{
    unsigned axis,direction;
    if(!CAR_RUN_CORNER_ENABLED)return;
    for(direction=0;direction<2u;direction++) {
        uint32_t deadline;
        wait_in_corner(1);
        physical_yaw+=direction ? 2000 : -2000;sample_yaw(wrapped_yaw(physical_yaw));
        plant_step();
        CHECK(abs(run.heading_control.error_cdeg)>CAR_HEADING_MAX_DRIFT_CDEG);
        CHECK(run_active && run.corner.active && run.state!=RUN_FAULT);
        deadline=clock_ms+60000;
        while(run_active && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && run.state==RUN_STATION && !run.corner.active);
        CHECK(hypot(field_x-420,field_y-1200)<5.0);
        stop_request();
    }
    for(axis=0;axis<2u;axis++)for(direction=0;direction<2u;direction++) {
        uint32_t deadline;
        float goal=direction ? (axis ? 780.0f : 810.0f)+600.0f : -600.0f;
        wait_in_corner(1);
        /* 只验证已观测位置越界后的控制行为；同时平移独立场地位置以保持模型一致，
         * 不用它声称编码器能检测真实无轮转侧滑。 */
        if(axis){field_x-=goal-run.corner.y_mm;run.corner.y_mm=goal;}
        else {field_y-=goal-run.corner.x_mm;run.corner.x_mm=goal;}
        plant_step();CHECK(run_active && run.corner.active && run.state!=RUN_FAULT);
        deadline=clock_ms+90000;
        while(run_active && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        if(run.state!=RUN_STATION)fprintf(stderr,"CORNER threshold axis=%u direction=%u xy=%f,%f %s\n",axis,direction,run.corner.x_mm,run.corner.y_mm,output);
        CHECK(clock_ms<deadline && run.state==RUN_STATION && !run.corner.active);
        CHECK(hypot(field_x-420,field_y-1200)<5.0);
        stop_request();
    }
    puts("PASS corner threshold recovery: both20-degree yaw disturbances and all four600mm outside-position boundaries continue closed-loop to H; no threshold fault or task cancellation");
}

int main(void)
{
    corner_transit_tests();corner_recovery_tests();corner_delay_tests();corner_yaw_wrap_tests();corner_threshold_recovery_tests();corner_whole_route_tests();
    printf("PASS corner route enabled=%u assertions=%u; closed-loop host model only, no hardware\n",(unsigned)CAR_RUN_CORNER_ENABLED,assertions);
    return 0;
}
