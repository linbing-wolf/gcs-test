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
                        CHECK(abs(simulated_wheel_rpm[i]-previous[i])<=ceil(CAR_ROUTE_ACCEL_RPM_PER_S*elapsed/1000.0)+2);
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
        if(hypot(field_x-h.x,field_y-h.y)>=(CAR_RUN_CORNER_ENABLED ? CAR_ROUTE_VISUAL_WINDOW_MM+2.0 : 80.0))
            fprintf(stderr,"CORNER endpoint enabled=%u plan=%u batch=%u gain=%.2f polarity=%u actual=%f,%f estimated=%f,%f\n",
                (unsigned)CAR_RUN_CORNER_ENABLED,plan,batch,gains[g],p,field_x,field_y,run.corner.x_mm,run.corner.y_mm);
        CHECK(hypot(field_x-h.x,field_y-h.y)<(CAR_RUN_CORNER_ENABLED ? CAR_ROUTE_VISUAL_WINDOW_MM+2.0 : 80.0));
        CHECK(abs(heading_wrap_cdeg(wrapped_yaw(physical_yaw)+9000))<=CAR_HEADING_TOLERANCE_CDEG+2);
        if(CAR_RUN_CORNER_ENABLED) {
            CHECK(seen && rotating_moving && arc_samples>100 && run.corner.passed && run.corner.arriving);
            if(max_yaw_error>=15 || max_arc_error>=35)fprintf(stderr,"CORNER tracking gain=%.2f polarity=%u yaw_deg=%.3f arc_mm=%.3f\n",gains[g],p,max_yaw_error,max_arc_error);
            CHECK(max_yaw_error<15 && max_arc_error<35);
            CHECK(run.corner.visual_terminal && hypot(run.corner.x_mm-810,run.corner.y_mm-780)<=CAR_ROUTE_VISUAL_WINDOW_MM+2);
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
            CHECK(hypot(field_x-420,field_y-1200)<CAR_ROUTE_VISUAL_WINDOW_MM+2.0);
        } else {
            if(variant==2)send_line("!");
            if(variant==3){bus_off=1;plant_step();}
            if(variant==4){motor_flags[2]=9;tick(200);}
            if(variant==5){frozen_encoder=1;tick(2000);}
            CHECK(!run_active && (run.state==RUN_STOPPED || run.state==RUN_FAULT));
            CHECK(!simulated_wheel_rpm[0] && !simulated_wheel_rpm[1] && !simulated_wheel_rpm[2] && !simulated_wheel_rpm[3]);
            tick(500);CHECK(!run_active && run.job==job);
        }
        stop_request();
    }
    for(stage=0;stage<3;stage++) {
        uint32_t deadline;unsigned job;int preparing;
        wait_in_corner(stage);job=run.job;preparing=run.station_entry_overlap;
        send_line("obstacle");deadline=clock_ms+60000;
        if(preparing) {
            CHECK(run.state==RUN_FAULT && !run_active && arm_failed() && !station_active());
            tick(500);CHECK(!run_active && !run.obstacle_phase);stop_request();continue;
        }
        CHECK(run.obstacle_node==NODE_D && run.state==RUN_OBSTACLE_STOP);
        while(run_active && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        if(run.state!=RUN_WAIT_OBSTACLE)fprintf(stderr,"CORNER return stage=%u s=%f xy=%f,%f err=%ld %s\n",stage,run.corner.s_mm,run.corner.x_mm,run.corner.y_mm,(long)run.heading_control.error_cdeg,output);
        CHECK(clock_ms<deadline && run.state==RUN_WAIT_OBSTACLE && !run_active && !run.corner.active);
        CHECK(run.edge==5 && run.job==job && run.heading==HEAD_X_POS && arm_ready() && arm_parked());
        CHECK(hypot(field_x-1200,field_y-2010)<5.0 && abs(wrapped_yaw(physical_yaw))<=82);
        tick(500);CHECK(!run_active && run.state==RUN_WAIT_OBSTACLE);
        stop_request();
    }
    puts("PASS corner recovery: lost wheel/IMU resumes same target; emergency/actual faults stop; obstacle before arm preparation returns along curve, after preparation retains existing station-work stop policy");
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
        CHECK(hypot(field_x-420,field_y-1200)<CAR_ROUTE_VISUAL_WINDOW_MM+2.0);
        simulate_uart_cost=0;stop_request();
    }
    puts("PASS corner delays: lost velocity ACK for500ms and115200baud log cost retain endpoint/yaw completion");
}

static void corner_whole_route_tests(void)
{
    unsigned plan;
    for(plan=0;plan<2;plan++) {
        unsigned offset=0,last_job=0,count=0,was_active=0;uint32_t deadline,start,travel_ms=0;
        prepare_arm_auto(plan);imu_period_ms=20;
        send_line("task 134+123+314+231");send_line(plan ? "runimu C" : "runimu A");start=clock_ms;deadline=clock_ms+2000000;
        while(run.state!=RUN_DONE && run.state!=RUN_FAULT && clock_ms<deadline) {
            if(run_active && !run.station_shift)travel_ms++;
            serve_peer(&offset);plant_step();
            if(run.corner.active) {
                CHECK(run.corner.start_edge==0 || run.corner.start_edge==1 || run.corner.start_edge==5
                    || run.corner.start_edge==7 || run.corner.start_edge==11 || run.corner.start_edge==13);
                if(!was_active)count++;
            }
            was_active=run.corner.active;
            if(run.job!=last_job){last_job=run.job;arm_positions=0;}
        }
        if(run.state!=RUN_DONE)fprintf(stderr,"CORNER full plan=%u %s\n",plan,output);
        CHECK(clock_ms<deadline && run.state==RUN_DONE && run.job==8 && arm_ready() && arm_parked());
        CHECK(count==(CAR_RUN_CORNER_ENABLED ? 5u : 0u) && run.distance_mm==EXPECTED_FULL_ROUTE_MM);
        plant_step();CHECK(!route_timing.active);
        printf("ROUTE_BUDGET enabled=%u plan=%u road_ms=%lu scan_wait_ms=%lu station_move_ms=%lu arm_ms=%lu vision_wait_ms=%lu other_ms=%lu\n",
            (unsigned)CAR_RUN_CORNER_ENABLED,plan,(unsigned long)route_timing.total[0],(unsigned long)route_timing.total[1],
            (unsigned long)route_timing.total[2],(unsigned long)route_timing.total[3],(unsigned long)route_timing.total[4],(unsigned long)route_timing.total[5]);
        printf("FULL_ROUTE_TIME enabled=%u plan=%u total_s=%.3f chassis_ms=%lu\n",
            (unsigned)CAR_RUN_CORNER_ENABLED,plan,(clock_ms-start)/1000.0,(unsigned long)travel_ms);
        printf("PASS corner full flow plan=%u enabled=%u corners=%u logical_distance_mm=%u; absolute_model_max_mm=%.3f initial_forward_bias_mm=%.3f budget_mm=%.3f; eight jobs unchanged\n",
            plan,(unsigned)CAR_RUN_CORNER_ENABLED,count,run.distance_mm,checkpoint_max_error_mm,
            hypot(start_forward_bias_x,start_forward_bias_y),80+hypot(start_forward_bias_x,start_forward_bias_y));
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
        CHECK(hypot(field_x-420,field_y-1200)<CAR_ROUTE_VISUAL_WINDOW_MM+2.0);
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
        /* 直接构造相对当前目标的±20度，不能让原跟踪误差抵消注入量。 */
        physical_yaw+=run.heading_control.error_cdeg+(direction ? 2000 : -2000);sample_yaw(wrapped_yaw(physical_yaw));
        tick(imu_period_ms+1u); /* 同毫秒的注入不能伪造新的角速度采样间隔。 */
        CHECK(abs(run.heading_control.error_cdeg)>CAR_HEADING_MAX_DRIFT_CDEG);
        CHECK(run_active && run.corner.active && run.state!=RUN_FAULT);
        deadline=clock_ms+60000;
        while(run_active && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && run.state==RUN_STATION && !run.corner.active);
        CHECK(hypot(field_x-420,field_y-1200)<CAR_ROUTE_VISUAL_WINDOW_MM+2.0);
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
        plant_step();CHECK(run_active && run.corner.active && run.state!=RUN_FAULT && !run.corner.visual_terminal);
        deadline=clock_ms+90000;
        while(run_active && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        if(run.state!=RUN_STATION)fprintf(stderr,"CORNER threshold axis=%u direction=%u xy=%f,%f %s\n",axis,direction,run.corner.x_mm,run.corner.y_mm,output);
        CHECK(clock_ms<deadline && run.state==RUN_STATION && !run.corner.active);
        CHECK(hypot(field_x-420,field_y-1200)<CAR_ROUTE_VISUAL_WINDOW_MM+2.0);
        stop_request();
    }
    puts("PASS corner threshold recovery: both20-degree yaw disturbances and all four600mm outside-position boundaries continue closed-loop to H; no threshold fault or task cancellation");
}


/* 以场地图坐标独立检查所有拐角，包含不转车头的前后/横移切换。 */
static unsigned check_chain_geometry(void)
{
    static const unsigned starts[]={0,1,5,7,11,13},counts[]={2,1,1,1,1,2};
    unsigned i,j,k=0,arcs=0,rotations=0,expected;
    RoadPoint origin;
    double a=run.corner.start_heading*3.141592653589793/2;
    double centers[3][2],radii[3]={150,150,150};
    while(k<6 && starts[k]!=run.corner.start_edge)k++;
    CHECK(k<6 && road_point(full_route_nodes[run.plan][run.corner.start_edge],&origin));
    expected=(k==5 && run.plan) ? 1u : counts[k];

    if(k<=1) {
        if(!k){centers[0][0]=2040;centers[0][1]=run.plan ? 1350 : 1050;}
        centers[counts[k]-1u][0]=1350;centers[counts[k]-1u][1]=1050;
    } else if(k==2 || k==4) {centers[0][0]=1050;centers[0][1]=1350;}
    else if(k==3) {centers[0][0]=1050;centers[0][1]=1050;}
    else {
        centers[0][0]=2040;centers[0][1]=run.plan ? 1350 : 1050;
        centers[1][0]=2220;centers[1][1]=run.plan ? 2200 : 200;radii[1]=30;
    }
    if(run.corner.prefix_mm) {
        CHECK(k==2 || k==3 || k==4);
        centers[1][0]=centers[0][0];centers[1][1]=centers[0][1];
        if(k==3) {
            centers[0][0]=origin.x+75;centers[0][1]=origin.y-(run.corner.prefix_mm>0 ? 75 : -75);
            origin.y-=run.corner.prefix_mm;
        } else {
            centers[0][0]=1200-(run.corner.prefix_mm>0 ? 75 : -75);centers[0][1]=1935;
            origin.x-=run.corner.prefix_mm;
        }
        radii[0]=75;expected++;
    }
    for(i=0;i<run.corner.span_count;i++) {
        CornerSpan *span=&run.corner.spans[i];
        CornerPoint begin=run_corner_point(span->s),end=run_corner_point(span->s+span->length);
        CHECK(fabs(hypot(begin.dx,begin.dy)-1)<0.0001 && fabs(hypot(end.dx,end.dy)-1)<0.0001);
        if(i) {
            CornerPoint previous=run_corner_point(span->s-0.001f);
            CHECK(hypot(previous.x-begin.x,previous.y-begin.y)<0.01);
            CHECK(hypot(previous.dx-begin.dx,previous.dy-begin.dy)<0.001);
            CHECK(fabs(previous.yaw_cdeg-begin.yaw_cdeg)<0.01 && fabs(previous.yaw_per_mm-begin.yaw_per_mm)<0.01);
        }
        if(span->radius>0) {
            double x=origin.x+cos(a)*span->cx+sin(a)*span->cy;
            double y=origin.y+sin(a)*span->cx-cos(a)*span->cy;
            CHECK(arcs<expected && fabs(x-centers[arcs][0])<0.01 && fabs(y-centers[arcs][1])<0.01);
            CHECK(fabs(span->radius-radii[arcs])<0.001);arcs++;
        }
        if(span->dyaw) {CHECK(span->radius>0 && span->dyaw==-9000);rotations++;}
        for(j=0;j<5;j++) {
            CornerPoint point=run_corner_point(span->s+span->length*j/4);
            if(k==0 || k==1 || k==5)CHECK(point.yaw_cdeg==0 && point.yaw_per_mm==0);
        }
    }
    CHECK(arcs==expected && rotations==((k==2 || k==3 || k==4) ? 1u : 0u));
    return arcs;
}

static void corner_sample_time_tests(void)
{
    HeadingSample sample;float yaw;unsigned count;
    reset_plant(0,1.15);memset(&run.corner,0,sizeof(run.corner));
    memset(&sample,0,sizeof(sample));sample.valid=1;sample.yaw_cdeg=17950;sample.received_ms=UINT32_MAX-30u;
    run_corner_yaw_record(sample);count=run.corner.yaw_count;
    sample.received_ms-=20;run_corner_yaw_record(sample);CHECK(run.corner.yaw_count==count);
    sample.received_ms=9;sample.yaw_cdeg=-17950;run_corner_yaw_record(sample);
    CHECK(run.corner.yaw_count==2 && run_corner_yaw_at(UINT32_MAX-10u,&yaw) && fabs(yaw-18000)<0.1);
    CHECK(!run_corner_yaw_at(10,&yaw)); /* 不把最后一份航向外推至未来编码器。 */
    puts("PASS corner timebase: old/repeated IMU cannot create descending history; angle and millisecond wrap interpolate actual samples, no future extrapolation");
}

static void all_corner_motion_tests(void)
{
    unsigned plan;
    if(!CAR_RUN_CORNER_ENABLED)return;
    for(plan=0;plan<2;plan++) {
        unsigned offset=0,last_job=0,was_active=0,chains=0,arcs=0,mask=0,moving_mask=0,chain_arcs=0;
        uint32_t deadline,start,travel_ms=0;
        prepare_arm_auto(plan);imu_period_ms=20;
        send_line("task 134+123+314+231");send_line(plan ? "runimu C" : "runimu A");
        start=clock_ms;deadline=clock_ms+2000000;
        while(run.state!=RUN_DONE && run.state!=RUN_FAULT && clock_ms<deadline) {
            unsigned i;
            if(run_active && !run.station_shift)travel_ms++;
            serve_peer(&offset);plant_step();
            if(run.corner.active) {
                unsigned arc_index=0;
                if(!was_active){chain_arcs=check_chain_geometry();arcs+=chain_arcs;chains++;mask|=1u<<run.corner.start_edge;}
                for(i=0;i<run.corner.span_count;i++) {
                    CornerSpan *span=&run.corner.spans[i];
                    if(span->radius>0)arc_index++;
                    if(span->radius>0 && run.corner.s_mm>span->s+5 && run.corner.s_mm<span->s+span->length-5) {
                        double fw=(-actual_rpm[0]+actual_rpm[1]+actual_rpm[2]-actual_rpm[3])/4;
                        double le=(-actual_rpm[0]-actual_rpm[1]+actual_rpm[2]+actual_rpm[3])/4;
                        CHECK(run_active && hypot(fw,le)>0.5);
                        CHECK(run.forward_cmd_rpm || run.left_cmd_rpm);
                        moving_mask|=1u<<(arcs-chain_arcs+arc_index-1u);
                        if(run.corner.start_heading==run.corner.end_heading)CHECK(run.heading_control.target_cdeg==run.corner.heading_target);
                    }
                }
            }
            was_active=run.corner.active;
            if(run.job!=last_job){last_job=run.job;arm_positions=0;}
        }
        CHECK(clock_ms<deadline && run.state==RUN_DONE && run.job==8 && chains==5 && arcs==(plan ? 6u : 7u)+(STN_ROUGH_THROUGH_ENABLED ? 2u : 0u));
        CHECK(mask==((1u<<0)|(1u<<5)|(1u<<7)|(1u<<11)|(1u<<13)) && moving_mask==((1u<<arcs)-1u));
        CHECK(run.distance_mm==EXPECTED_FULL_ROUTE_MM && run.heading==HEAD_Y_POS);
        printf("ALL_CORNERS plan=%u chains=%u arcs=%u total_s=%.3f chassis_ms=%lu; independent circle centers, fixed/yaw-changing body modes, continuous nonzero translation and eight jobs verified\n",
            plan,chains,arcs,(clock_ms-start)/1000.0,(unsigned long)travel_ms);
        stop_request();
    }
}

static void general_corner_return_tests(void)
{
    static const unsigned edges[]={0,1,7,13};
    unsigned plan,k;
    if(!CAR_RUN_CORNER_ENABLED)return;
    for(plan=0;plan<2;plan++)for(k=0;k<4;k++) {
        unsigned offset=0,last_job=0;uint32_t deadline;RoadPoint start;
        prepare_arm_auto(plan);imu_period_ms=20;
        if(k!=1)send_line("task 134+123+314+231");
        send_line(plan ? "runimu C" : "runimu A");deadline=clock_ms+2000000;
        while(clock_ms<deadline && run.state!=RUN_FAULT) {
            unsigned i;int in_arc=0;
            if(k==1 && run.state==RUN_STATION && !run.job && !station_task_ready())send_line("task 134+123+314+231");
            serve_peer(&offset);plant_step();
            if(run.job!=last_job){last_job=run.job;arm_positions=0;}
            if(!run.corner.active || run.corner.start_edge!=edges[k])continue;
            for(i=0;i<run.corner.span_count;i++)if(run.corner.spans[i].radius>0
                && run.corner.s_mm>run.corner.spans[i].s+40 && run.corner.s_mm<run.corner.spans[i].s+run.corner.spans[i].length-40)in_arc=1;
            if(in_arc)break;
        }
        CHECK(clock_ms<deadline && run.corner.active && !run.corner.arriving);
        CHECK(road_point(full_route_nodes[plan][edges[k]],&start));
        {
            unsigned job=run.corner.start_job;RoadHeading heading=run.corner.start_heading;
            /* 初始/等待后两种符号、首个H离站及末次H离站，均回到本次连续路径起点。 */
            send_line("obstacle");deadline=clock_ms+120000;
            CHECK(run.obstacle_node==full_route_nodes[plan][edges[k]]);
            while(run_active && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
            if(run.state!=RUN_WAIT_OBSTACLE)fprintf(stderr,"GENERAL_RETURN plan=%u edge=%u %s\n",plan,edges[k],output);
            CHECK(clock_ms<deadline && run.state==RUN_WAIT_OBSTACLE && !run_active);
            CHECK(run.edge==edges[k] && run.job==job && run.heading==heading);
            CHECK(hypot(field_x-start.x,field_y-start.y)<80 && arm_ready() && arm_parked());
            tick(500);CHECK(run.state==RUN_WAIT_OBSTACLE && !run_active);
        }
        stop_request();
    }
    puts("PASS generalized corner return: A/C opposite initial lateral signs, B forward launch, H-to-F yaw turn and final home path reverse to their own start with original job/heading and no automatic restart");
}


/* 道路重新起步提速，停车和过零仍使用原制动能力，不能用新加速率缩短制动距离。 */
static void transition_profile_tests(void)
{
    int16_t output[3];float desired[3]={200,0,0};RoadPlan road;unsigned plan;
    reset_plant(0,1.15);memset(&run,0,sizeof(run));
    run.corner.command_ms=clock_ms;clock_ms+=25;
    run_corner_slew(desired,output);CHECK(fabs(run.corner.command[0]-6)<0.01);
    run.corner.command[0]=20;desired[0]=0;clock_ms+=25;
    run_corner_slew(desired,output);CHECK(fabs(run.corner.command[0]-14)<0.01);
    run.corner.command[0]=2;desired[0]=-200;clock_ms+=50;
    run_corner_slew(desired,output);CHECK(fabs(run.corner.command[0]+10)<0.01);
    run.corner.reversing=1;run.corner.command[0]=2;clock_ms+=50;
    run_corner_slew(desired,output);CHECK(fabs(run.corner.command[0]+10)<0.01);
    memset(&run,0,sizeof(run));run.corner.command[0]=20;run.corner.command_ms=clock_ms;
    desired[0]=0;desired[1]=20;clock_ms+=25;run_corner_slew(desired,output);
    CHECK(fabs(run.corner.command[0]-17.0f)<0.01 && fabs(run.corner.command[1]-3.0f)<0.01);
    memset(&run,0,sizeof(run));run.forward_profile_ms=clock_ms;clock_ms+=25;
    CHECK(run_forward_slew(200)==6);
    memset(&run,0,sizeof(run));run.station_shift=1;run.forward_profile_ms=clock_ms;clock_ms+=25;
    CHECK(run_forward_slew(200)==3); /* 已有工位150RPM/s不跟随道路240RPM/s提速。 */
    for(plan=0;plan<2;plan++) {
        run.plan=(uint8_t)plan;
        CHECK(run_plan_road(13,&road) && road.count==1 && road.length==780);
        CHECK(run_plan_road(14,&road) && road.count==1 && road.length==990);
        CHECK(run_plan_road(15,&road) && road.count==((plan && CAR_RUN_CORNER_ENABLED) ? 1 : 2)
            && road.length==((plan && CAR_RUN_CORNER_ENABLED) ? 1030 : 1090));
        CHECK(road.segments[0].axis==BODY_FORWARD && road.segments[0].mm==(plan ? 1030 : -1030));
        CHECK(road_plan(NODE_B,NODE_C,&road) && road.count==2 && road.length==1090);
    }
    puts("PASS transition profiles: road240 versus station150RPM/s, braking240, reversal decelerates through zero, shared wheel ratio; C retains E-B990 and blends final60 into B-C forward motion");
}

int main(void)
{
    transition_profile_tests();corner_sample_time_tests();all_corner_motion_tests();general_corner_return_tests();corner_transit_tests();corner_recovery_tests();corner_delay_tests();corner_yaw_wrap_tests();corner_threshold_recovery_tests();corner_whole_route_tests();
    printf("PASS corner route enabled=%u assertions=%u; closed-loop host model only, no hardware\n",(unsigned)CAR_RUN_CORNER_ENABLED,assertions);
    return 0;
}
