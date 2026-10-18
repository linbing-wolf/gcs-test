/* Reuse the existing firmware fixture and legacy regressions. */
#define main legacy_test_main
#include "verify_entry.c"
#undef main

static void reset_heading_fixture(void)
{
    reset_fixture();
    heading_sensor_reset(&heading_sensor);
    can_rx_fault = imu_uart_fault = 0;
    memset(motor_feedback, 0, sizeof(motor_feedback));
    invalid_can_replies = rejected_can_frames = 0;
}

static void sample_yaw(int16_t yaw)
{
    heading_sensor_publish(&heading_sensor, yaw, clock_ms);
}

static void wit_packet(HeadingSensor *sensor, uint8_t kind, int16_t raw, uint32_t now, int bad)
{
    uint8_t frame[11] = {0x55,0,0,0,0,0,0,0,0,0,0};
    unsigned i;
    frame[1] = kind;
    frame[6] = (uint8_t)raw; frame[7] = (uint8_t)((uint16_t)raw >> 8);
    for (i = 0; i < 10; i++) frame[10] = (uint8_t)(frame[10] + frame[i]);
    if (bad) frame[10]++;
    for (i = 0; i < 11; i++) heading_sensor_byte(sensor, frame[i], now);
}

static void sensor_tests(void)
{
    HeadingSensor sensor;
    HeadingSample sample;
    unsigned i;
    heading_sensor_reset(&sensor);
    CHECK(!heading_sample_fresh((const HeadingSample *)&sensor.sample, 0, 250));
    wit_packet(&sensor, 0x53, 16384, 100, 0);
    sample = sensor.sample;
    CHECK(sample.yaw_cdeg == 9000 && sample.sequence == 1);
    CHECK(heading_sample_fresh(&sample, 350, 250));
    CHECK(!heading_sample_fresh(&sample, 351, 250));
    wit_packet(&sensor, 0x51, 0, 400, 0);
    CHECK(sensor.sample.sequence == 1 && sensor.sample.received_ms == 100);
    wit_packet(&sensor, 0x53, -16384, 450, 1);
    CHECK(sensor.checksum_errors == 1 && sensor.sample.sequence == 1);
    /* An incomplete frame followed by a complete one must resynchronize. */
    heading_sensor_byte(&sensor, 0x55, 500);
    heading_sensor_byte(&sensor, 0x53, 500);
    for (i = 0; i < 2; i++) wit_packet(&sensor, 0x53, -16384, 510, 0);
    CHECK(sensor.sample.yaw_cdeg == -9000 && sensor.sample.received_ms == 510);
    wit_packet(&sensor, 0x53, -32768, 600, 0);
    CHECK(sensor.sample.yaw_cdeg == -18000);
    CHECK(heading_wrap_cdeg(17900 - (-17900)) == -200);
    CHECK(heading_wrap_cdeg(-17900 - 17900) == 200);
    heading_sensor_publish(&sensor, 10, UINT32_MAX - 40u);
    sample = sensor.sample;
    CHECK(heading_sample_fresh(&sample, 50, 100));
    heading_sensor_publish(&sensor, 20000, 900);
    CHECK(!sensor.sample.valid);
}

static void feedback_tests(void)
{
    unsigned i;
    const uint8_t arrived[3] = {0x3a,3,0x6b};
    /* Motor 4 replies but reports not-arrived: diagnostics distinguish this from no reply. */
    reset_heading_fixture();calibrate_turn_fixture();send_line("run A");run_to_first_segment();
    motor_flags[3] = 1;
    clock_ms = drive.started_ms + drive.estimate_ms;
    for (i = 0; i < 20; i++) {clock_ms += 25; firmware_step();}
    CHECK(run.state == RUN_MOVING && run.acks == 15 && run.ready == 7 && run.steps == 1);
    CHECK(motor_feedback[3].status_replies > 0 && motor_feedback[3].status_flags == 1);
    CHECK(motor_feedback[3].action_status_replies > 0);
    send_line("feedback");CHECK(strstr(output,"addr=4") && strstr(output,"arrived=0"));
    stop_request();

    reset_heading_fixture();calibrate_turn_fixture();send_line("run A");run_to_first_segment();
    CHECK(motor_feedback[3].status_replies==1 && motor_feedback[3].action_status_replies==0);
    status_auto_reply=0;clock_ms=drive.started_ms+drive.estimate_ms;
    run.ready=7;run.pending=4;run.query_ms=clock_ms;
    clock_ms+=CAR_RUN_STATUS_TIMEOUT_MS;firmware_step();
    CHECK(run.state==RUN_FAULT && motor_feedback[3].action_status_replies==0);

    reset_heading_fixture();calibrate_turn_fixture();send_line("run A");run_to_first_segment();
    clock_ms=drive.started_ms+drive.estimate_ms;
    run.ready=15;run.pending=0;
    for(i=0;i<4;i++)run.status_ms[i]=clock_ms-CAR_RUN_STATUS_MAX_AGE_MS-1u;
    firmware_step();CHECK(run.state==RUN_MOVING && run.steps==1);
    stop_request();

    /* Replies already queued before a request cannot satisfy that request. */
    reset_heading_fixture();calibrate_turn_fixture();status_auto_reply = 0;send_line("run A");
    CHECK(run.pending == 1);
    clock_ms += 20;
    route_test_can_rx(0x100,4,0,3,arrived);
    run.query_ms = clock_ms + 1;
    clock_ms += 2;firmware_step();
    CHECK(run.ready == 0 && run.pending == 1 && motor_feedback[0].status_replies == 1);
    stop_request();

    /* A timely reply processed just after the deadline is retained; a late reply is rejected. */
    for (i = 0; i < 2; i++) {
        reset_heading_fixture();calibrate_turn_fixture();send_line("run A");run_to_first_segment();
        firmware_step();CHECK(run.acks == 15);
        run.ready = 7;run.pending = 4;status_auto_reply = 0;
        clock_ms = drive.started_ms + drive.timeout_ms - 10;
        run.query_ms = clock_ms;
        run.status_ms[0] = run.status_ms[1] = run.status_ms[2] = clock_ms;
        if (i) clock_ms += 11;
        route_test_can_rx(0x400,4,0,3,arrived);
        if (!i) clock_ms += 11;
        firmware_step();
        if (!i) CHECK(run.state == RUN_MOVING && run.steps == 2);
        else CHECK(run.state == RUN_FAULT && run.steps == 1);
    }

    reset_heading_fixture();calibrate_turn_fixture();send_line("run A");
    route_test_can_error(1);firmware_step();
    CHECK(run.state == RUN_FAULT && !position_count && strstr(output,"FIFO overrun"));

    reset_heading_fixture();calibrate_turn_fixture();fail_can_at_position=1;send_line("run A");
    for(i=0;run_active && i<20;i++){clock_ms+=25;firmware_step();}
    CHECK(run.state==RUN_FAULT && position_count==1 && sync_count==0);
    CHECK(strstr(output,"mailbox full"));

    /* Arrival notification is logged, but is not silently substituted for a required ACK. */
    reset_heading_fixture();calibrate_turn_fixture();position_ack_enabled = 0;
    send_line("run A");
    for(i=0;run.state==RUN_PREFLIGHT && i<100;i++){clock_ms+=25;firmware_step();}
    CHECK(run.state==RUN_MOVING && position_count==4);
    {
        const uint8_t notice[3] = {0xfd,0x9f,0x6b};
        route_test_can_rx(0x400,4,0,3,notice);firmware_step();
        CHECK(run.acks == 0 && motor_feedback[3].arrival_notices == 1);
    }
    clock_ms = run.segment_ms + 1501;firmware_step();CHECK(run.state == RUN_FAULT);
}

static void motor_diagnostic_tests(void)
{
    unsigned i;
    const uint8_t error[7]={0x37,1,0x81,0x23,0x45,0x67,0x6b};
    const uint8_t speed[5]={0x35,1,0x05,0xdc,0x6b};
    const uint8_t bad_sign[7]={0x37,2,0,0,0,8,0x6b};
    const uint8_t bad_check[5]={0x35,0,0,0,0x6a};
    const uint8_t short_error[3]={0x37,0,0x6b};
    reset_heading_fixture();calibrate_turn_fixture();send_line("run A");run_to_first_segment();
    motor_flags[3]=1;
    clock_ms=drive.started_ms+drive.estimate_ms;
    for(i=0;i<80;i++){clock_ms+=25;firmware_step();}
    CHECK(run.state==RUN_MOVING && run.steps==1 && run.ready==7 && run.acks==15);
    CHECK(diagnostic_queries>=3 && diagnostic_queries<=5);
    CHECK(motor_feedback[3].error_valid && motor_feedback[3].error_negative);
    CHECK(motor_feedback[3].error_counts==8 && motor_feedback[3].speed_valid && !motor_feedback[3].speed_rpm);
    send_line("feedback");CHECK(strstr(output,"error_counts=-8") && strstr(output,"speed_valid=1 speed_rpm=0"));

    /* Unsolicited diagnostic replies cannot acknowledge or complete a status request. */
    diagnostic_auto_reply=0;status_auto_reply=0;run.pending=4;run.query_ms=clock_ms;
    route_test_can_rx(0x400,4,0,7,error);route_test_can_rx(0x400,4,0,5,speed);
    firmware_step();
    CHECK(run.pending==4 && run.ready==7 && run.acks==15 && run.steps==1);
    CHECK(motor_feedback[3].error_counts==0x81234567u && motor_feedback[3].error_negative);
    CHECK(motor_feedback[3].speed_rpm==-1500);
    route_test_can_rx(0x400,4,0,7,bad_sign);route_test_can_rx(0x400,4,0,5,bad_check);
    route_test_can_rx(0x400,4,0,3,short_error);
    firmware_step();CHECK(invalid_can_replies==3 && run.pending==4);
    CHECK(motor_feedback[3].error_counts==0x81234567u && motor_feedback[3].speed_rpm==-1500);
    route_test_can_rx(0x400,4,0,6,error);route_test_can_rx(0x400,4,0,8,error);
    CHECK(rejected_can_frames==2);
    /* No diagnostic response is required: ordinary four-wheel flags still decide completion. */
    run.pending=0;status_auto_reply=1;
    for(i=0;i<60;i++){clock_ms+=25;firmware_step();}
    CHECK(run.state==RUN_MOVING && run.ready==7 && run.steps==1);
    motor_flags[3]=3;
    for(i=0;run.steps==1 && i<20;i++){clock_ms+=25;firmware_step();}
    CHECK(run.steps==2 && !motor_feedback[3].error_valid && !motor_feedback[3].speed_valid);
    stop_request();
    puts("PASS motor diagnostics: error/speed signed decoding, malformed frames, missing replies, four-wheel/ACK isolation, per-action validity");
}


/* Independent plant: wheel inertia, encoder integration and IMU updates.
 * Gains are deliberately independent of controller parameters; no 3600-pulse turn model is used by firmware.
 */
#include <math.h>
static double actual_rpm[4], encoder_value[4], physical_yaw, yaw_gain;
static int encoder_polarity[4];
static uint32_t plant_ms, imu_ms;
static uint32_t imu_period_ms;
static int frozen_imu, frozen_encoder, inject_heading, injected_heading;
static unsigned forward_actions, max_turn_rpm, min_turn_rpm, reverse_seen;
static int last_step;
static unsigned corrected_during_drive, disturbance_step;
static unsigned reverse_actions, rotation_starts, lateral_actions;
static int last_turn_step, planned_x, planned_y;
static uint8_t checked_job;
static double field_x, field_y, origin_yaw, checkpoint_max_error_mm;
static int initial_field_heading;
static unsigned station_alignment_entries, last_station_edge;
static unsigned heading_reuses, baseline_transitions, moving_transitions;
static uint32_t aligned_ms, motion_after_align_ms, baseline_delay_sum, baseline_delay_max;
static uint32_t motion_delay_sum, motion_delay_max;

static int16_t wrapped_yaw(double yaw)
{
    while(yaw>=18000)yaw-=36000;
    while(yaw< -18000)yaw+=36000;
    return (int16_t)lround(yaw);
}

static void reset_plant(int16_t yaw,double gain)
{
    unsigned i;
    reset_heading_fixture();
    memset(actual_rpm,0,sizeof(actual_rpm));
    for(i=0;i<4;i++) {encoder_value[i]=(i&1) ? -1000000.0 : 1000000.0;encoder_polarity[i]=1;}
    for(i=0;i<4;i++)simulated_encoder[i]=(int64_t)encoder_value[i];
    memset(motor_flags,1,sizeof(motor_flags)); /* Healthy wheels with position-arrived unset. */
    physical_yaw=yaw;yaw_gain=gain;plant_ms=imu_ms=clock_ms;
    imu_period_ms=10;
    frozen_imu=frozen_encoder=inject_heading=injected_heading=0;
    forward_actions=max_turn_rpm=reverse_seen=0;min_turn_rpm=1000;last_step=-1;
    corrected_during_drive=disturbance_step=0;
    reverse_actions=rotation_starts=lateral_actions=0;last_turn_step=-1;
    planned_x=planned_y=0;checked_job=0;
    field_x=field_y=checkpoint_max_error_mm=0;origin_yaw=yaw;initial_field_heading=0;
    station_alignment_entries=0;last_station_edge=255;
    heading_reuses=baseline_transitions=moving_transitions=0;
    aligned_ms=motion_after_align_ms=baseline_delay_sum=baseline_delay_max=motion_delay_sum=motion_delay_max=0;
    sample_yaw(yaw);
}

static void plant_step(void)
{
    unsigned i;
    uint32_t elapsed;
    double forward,left,turn;
    clock_ms++;
    elapsed=clock_ms-plant_ms;plant_ms=clock_ms;
    for(i=0;i<4;i++) {
        double difference=simulated_wheel_rpm[i]-actual_rpm[i], change=0.18*elapsed;
        if(difference>change)difference=change;
        if(difference< -change)difference=-change;
        actual_rpm[i]+=difference;
        if(!frozen_encoder)encoder_value[i]+=encoder_polarity[i]*actual_rpm[i]*65536.0*elapsed/60000.0;
        simulated_encoder[i]=(int64_t)llround(encoder_value[i]);
        reported_wheel_rpm[i]=(int16_t)lround(actual_rpm[i]);
    }
    forward=(-actual_rpm[0]+actual_rpm[1]+actual_rpm[2]-actual_rpm[3])/4.0;
    left=(-actual_rpm[0]-actual_rpm[1]+actual_rpm[2]+actual_rpm[3])/4.0;
    turn=(actual_rpm[0]+actual_rpm[1]+actual_rpm[2]+actual_rpm[3])/4.0;
    {
        /* Independent field drawing: +X right, +Y down, positive physical yaw turns left. */
        double angle=(initial_field_heading*9000.0-(physical_yaw-origin_yaw))*3.141592653589793/18000.0;
        double mm=forward*3200.0*elapsed/(60000.0*CALIBRATED_PPM_Y);
        double lateral_mm=left*3200.0*elapsed/(60000.0*CALIBRATED_PPM_X);
        field_x-=sin(angle)*mm;field_y+=cos(angle)*mm;
        field_x+=cos(angle)*lateral_mm;field_y+=sin(angle)*lateral_mm;
    }
    physical_yaw+=turn*yaw_gain*0.1*elapsed;
    if(fabs(forward)>1 || fabs(left)>1)physical_yaw+=0.04*elapsed; /* Independent yaw disturbance on either translation axis. */
    if(inject_heading && !injected_heading && run.velocity_phase==VELOCITY_FORWARD && run.progress_mm>100
        && (inject_heading==1 || run.travel_sign<0)) {
        physical_yaw+=400;injected_heading=1;disturbance_step=run.steps;
    }
    if(!frozen_imu && (uint32_t)(clock_ms-imu_ms)>=imu_period_ms) {
        imu_ms=clock_ms;sample_yaw(wrapped_yaw(physical_yaw));
    }
    output_size=0;output[0]=0;firmware_step();
    if(strstr(output,"[heading reused; four wheels stopped]"))heading_reuses++;
    if(strstr(output,"[heading aligned; four wheels stopped]") || strstr(output,"[heading reused; four wheels stopped]"))
        aligned_ms=motion_after_align_ms=clock_ms;
    if(aligned_ms && run.velocity_phase==VELOCITY_FORWARD) {
        uint32_t elapsed=clock_ms-aligned_ms;
        baseline_transitions++;baseline_delay_sum+=elapsed;
        if(elapsed>baseline_delay_max)baseline_delay_max=elapsed;
        aligned_ms=0;
    }
    if(motion_after_align_ms && run.velocity_phase==VELOCITY_FORWARD && !run.zero_ms) {
        uint32_t elapsed=clock_ms-motion_after_align_ms;
        moving_transitions++;motion_delay_sum+=elapsed;
        if(elapsed>motion_delay_max)motion_delay_max=elapsed;
        motion_after_align_ms=0;
    }
    if(run.station_align_pending && last_station_edge!=run.edge) {
        station_alignment_entries++;last_station_edge=run.edge;
    }
    if(run_active && run.velocity_phase==VELOCITY_FORWARD && run.steps!=last_step) {
        int mm=run.action_mm*run.travel_sign;
        RoadHeading travel=(RoadHeading)(((unsigned)run.target_heading+(run.travel_axis==BODY_LEFT ? 3u : 0u))%4u);
        forward_actions++;last_step=run.steps;
        if(run.travel_axis==BODY_LEFT) {
            lateral_actions++;
            if(run.edge==FULL_ROUTE_NODES-2u && !run.translation_only) {
                CHECK(run.segment==1 && run.target_heading==run.initial_heading);
                CHECK(run.travel_sign==(run.plan ? -1 : 1) && run.action_mm==60);
            }
        }
        if(run.travel_sign<0 && run.travel_axis==BODY_FORWARD)reverse_actions++;
        planned_x+=travel==HEAD_X_NEG ? -mm : travel==HEAD_X_POS ? mm : 0;
        planned_y+=travel==HEAD_Y_POS ? mm : travel==HEAD_Y_NEG ? -mm : 0;
    }
    if(injected_heading && run.steps==disturbance_step && run.velocity_phase==VELOCITY_FORWARD
        && run.progress_mm>200 && abs(run.heading_control.error_cdeg)<100 && (fabs(forward)>1 || fabs(left)>1))
        corrected_during_drive=1;
    if(run_active && run.turning) {
        unsigned magnitude=(unsigned)abs(simulated_wheel_rpm[0]);
        if(magnitude>max_turn_rpm)max_turn_rpm=magnitude;
        if(magnitude && magnitude<min_turn_rpm)min_turn_rpm=magnitude;
        if(magnitude && last_turn_step!=run.steps) {rotation_starts++;last_turn_step=run.steps;}
        if(run.heading_control.target_cdeg>0 && simulated_wheel_rpm[0]<0)reverse_seen=1;
    }
    if(!run.turn_only && !run.translation_only && run.job>checked_job) {
        /* Task order is independently fixed from the official rule, not copied from full_route_jobs. */
        static const uint8_t expected[8]={NODE_B,NODE_F,NODE_D,NODE_H,NODE_F,NODE_D,NODE_H,NODE_A};
        RoadPoint point;
        uint8_t node=run.job==8 && run.plan ? NODE_C : expected[run.job-1u];
        CHECK(run.job==checked_job+1 && full_route_nodes[run.plan][run.edge]==node);
        CHECK(road_point(node,&point) && planned_x==point.x && planned_y==point.y);
        if(node==NODE_F || node==NODE_D || node==NODE_H) {
            unsigned i;
            int desired_heading=node==NODE_F ? 1 : node==NODE_D ? 3 : 0;
            double angle=(initial_field_heading*9000.0-(physical_yaw-origin_yaw))*3.141592653589793/18000.0;
            double right_x=-cos(angle),right_y=-sin(angle);
            double facing=node==NODE_F ? -right_y : node==NODE_D ? right_y : -right_x;
            CHECK(!run.station_align_pending && run.heading==(RoadHeading)desired_heading);
            CHECK(facing>0.999); /* Right-side outward normal, independently checked in drawing coordinates. */
            CHECK(strstr(output,"[station right-side ready]"));
            for(i=0;i<4;i++)CHECK(!simulated_wheel_rpm[i] && abs(reported_wheel_rpm[i])<=CAR_VELOCITY_STOP_RPM);
        }
        {
            double error=hypot(field_x-point.x,field_y-point.y);
            if(error>checkpoint_max_error_mm)checkpoint_max_error_mm=error;
            CHECK(error<80); /* Simulation bound only; catches mirrored/mislabelled stations. */
        }
        checked_job=run.job;
    }
}

static void finish_plant(uint32_t limit)
{
    uint32_t deadline=clock_ms+limit;
    while(run_active && clock_ms<deadline)plant_step();
    if(run.state!=RUN_DONE)fprintf(stderr,"Plant failure phase=%d progress=%ld error=%ld: %s\n",
        run.velocity_phase,(long)run.progress_mm,(long)run.heading_control.error_cdeg,output);
    CHECK(run.state==RUN_DONE && !run_active && clock_ms<deadline);
    CHECK(position_count==0 && velocity_batches>0);
    CHECK(abs(run.heading_control.error_cdeg)<=CAR_HEADING_TOLERANCE_CDEG);
    CHECK(!pose_set && !nav.position_confirmed && drive.state==MOTION_IDLE);
}

static void continuous_turn_tests(void)
{
    unsigned origin,command;
    const char *commands[]={"left90","right90","left180","right180"};
    const int16_t origins[]={17900,-17900,7420};
    for(origin=0;origin<3;origin++)for(command=0;command<4;command++) {
        int expected=command>=2 ? 18000 : 9000;
        int32_t measured;
        if(command&1)expected=-expected;
        reset_plant(origins[origin],origin==1 ? 0.85 : 1.3);
        imu_period_ms=origin==0 ? 10 : origin==1 ? 50 : 100;
        send_line("turn");send_line("rpm 60");send_line(commands[command]);
        CHECK(run_active && run.turn_only && run.heading_control.target_cdeg==expected);
        send_line("left90");CHECK(strstr(output,"[rejected"));
        send_line("rpm 90");CHECK(strstr(output,"[rejected") && turn_rpm==60);
        finish_plant(20000);
        measured=(int32_t)lround(physical_yaw)-origins[origin];
        CHECK(abs(measured-expected)<=CAR_HEADING_TOLERANCE_CDEG+2);
        CHECK(max_turn_rpm<=60 && min_turn_rpm<=CAR_HEADING_MIN_RPM && forward_actions==0);
        CHECK(run.steps==1 && run.distance_mm==0 && motor_feedback[3].status_flags==1);
    }
    /* A physical overshoot must reverse even with an arbitrary origin/wrap. */
    reset_plant(17900,1.3);send_line("turn");send_line("rpm 60");send_line("left90");
    while(run_active && run.heading_control.progress_cdeg<8000)plant_step();
    physical_yaw=17900+9500;sample_yaw(wrapped_yaw(physical_yaw));
    finish_plant(15000);CHECK(reverse_seen);
    /* Each new relative command uses the current measured yaw. */
    sample_yaw(wrapped_yaw(physical_yaw));send_line("right90");finish_plant(20000);
    CHECK(abs(heading_wrap_cdeg(wrapped_yaw(physical_yaw)-17900))<=2*CAR_HEADING_TOLERANCE_CDEG+2);
    puts("PASS continuous turn: 12 origin/direction/angle cases, +/-180 wrap, variable wheel gain, deceleration, overshoot reversal, sequential commands, no position pulses");
}

static void continuous_route_tests(void)
{
    unsigned plan;
    for(plan=0;plan<2;plan++) {
        RoadPoint start;
        reset_plant(17900,1.15);inject_heading=1;
        CHECK(road_point(plan ? NODE_C : NODE_A,&start));planned_x=start.x;planned_y=start.y;
        field_x=start.x;field_y=start.y;initial_field_heading=plan ? 2 : 0;
        imu_period_ms=plan ? 50 : 100;
        send_line(plan ? "runimu C" : "runimu A");
        while(run_active && run.state==RUN_PREFLIGHT)plant_step();
        CHECK(run.travel_axis==BODY_LEFT && run.travel_sign==(plan ? 1 : -1));
        CHECK(run.target_heading==run.initial_heading && !rotation_starts);
        finish_plant(300000);
        CHECK(run.job==8 && run.edge==16 && run.distance_mm==EXPECTED_FULL_ROUTE_MM);
        CHECK(forward_actions==18 && injected_heading && corrected_during_drive);
        CHECK(checked_job==8 && station_alignment_entries==6);
        CHECK(rotation_starts==18 && reverse_actions==1 && lateral_actions==2);
        CHECK(heading_reuses==6 && baseline_transitions==18 && baseline_delay_max<=5);
        CHECK(moving_transitions==18 && motion_delay_max<=CAR_VELOCITY_CONTROL_MS+4u);
        {
            unsigned edge;
            uint16_t expected_roads=0;
            for(edge=0;edge<FULL_ROUTE_NODES-1u;edge++)
                expected_roads |= (uint16_t)(1u << road_index(full_route_nodes[plan][edge],full_route_nodes[plan][edge+1u]));
            CHECK(run.completed_roads==expected_roads);
        }
        CHECK(abs(heading_wrap_cdeg(wrapped_yaw(physical_yaw)-17900))<=CAR_HEADING_TOLERANCE_CDEG+2);
        printf("PASS continuous route %c: %umm, 18 segments, %u reverse, %u turns; six right-side station orientations and B-F-D-H-F-D-H-start coordinates verified, physical-plant max checkpoint error %.1fmm (simulated)\n",plan?'C':'A',EXPECTED_FULL_ROUTE_MM,reverse_actions,rotation_starts,checkpoint_max_error_mm);
        printf("PASS transition timing %c: %u heading reuses, %u baselines avg %.1fms max %lums; first moving batch avg %.1fms max %lums (simulation only)\n",
            plan?'C':'A',heading_reuses,baseline_transitions,(double)baseline_delay_sum/baseline_transitions,(unsigned long)baseline_delay_max,
            (double)motion_delay_sum/moving_transitions,(unsigned long)motion_delay_max);
    }
}

static void continuous_lateral_tests(void)
{
    static const unsigned distances[]={50,60,500,1000,2000};
    static const unsigned speeds[]={30,60,90};
    unsigned direction,distance,speed,test;
    double max_lateral_error=0,max_unobserved_drift=0;
    double start_lateral_error=0,start_forward_drift=0;
    char buffer[32];
    for(direction=0;direction<2;direction++)for(distance=0;distance<5;distance++)for(speed=0;speed<3;speed++) {
        uint32_t deadline;
        int requested=(int)distances[distance];
        reset_plant(direction ? -17900 : 17900,1.15);
        imu_period_ms=speed==0 ? 10 : speed==1 ? 50 : 100;
        inject_heading=distances[distance]>=500;
        send_line("test");sprintf(buffer,"dist %u",distances[distance]);send_line(buffer);
        sprintf(buffer,"rpm %u",speeds[speed]);send_line(buffer);
        send_line(direction ? "r" : "l");
        CHECK(run_active && run.translation_only && run.travel_axis==BODY_LEFT);
        CHECK(run.lateral_rpm==(speeds[speed]>60 ? 60 : speeds[speed]));
        send_line("l");CHECK(strstr(output,"[rejected") && run_active);
        send_line("dist 100");CHECK(test_distance==distances[distance]);
        deadline=clock_ms+30000;
        while(run_active && clock_ms<deadline)plant_step();
        CHECK(run.state==RUN_DONE && !run_active && clock_ms<deadline);
        CHECK(!position_count && velocity_batches && !run.job && !run.completed_roads && !turn_postcheck.active);
        CHECK(abs(run.progress_mm-requested)<=CAR_DISTANCE_TOLERANCE_MM);
        CHECK(abs(run.heading_control.error_cdeg)<=CAR_HEADING_TOLERANCE_CDEG);
        {
            double error=fabs(field_x-(direction ? -requested : requested));
            CHECK(error<15);
            if(error>max_lateral_error)max_lateral_error=error;
            /* IMU无法观测前后平移；记录该偏移，不把它伪装成已闭环的验收指标。 */
            if(fabs(field_y)>max_unobserved_drift)max_unobserved_drift=fabs(field_y);
            if(requested==60) {
                if(error>start_lateral_error)start_lateral_error=error;
                if(fabs(field_y)>start_forward_drift)start_forward_drift=fabs(field_y);
            }
        }
        CHECK(strstr(output,"DONE_BY_IMU_ENCODER") && !pose_set && !nav.position_confirmed);
        if(inject_heading)CHECK(injected_heading && corrected_during_drive);
    }
    /* A small overshoot at stop must produce reverse correction and pass the final distance gate. */
    reset_plant(0,1.15);send_line("test");send_line("dist 60");send_line("l");
    {
        uint32_t deadline=clock_ms+15000;
        unsigned i,shifted=0,reversed=0;
        while(run_active && clock_ms<deadline) {
            if(!shifted && run.velocity_phase==VELOCITY_STOP_FORWARD) {
                for(i=0;i<4;i++)encoder_value[i]+=run.encoder_sign[i]*3.0*CALIBRATED_PPM_X*65536.0/3200.0;
                shifted=1;
            }
            plant_step();
            if(shifted && (-simulated_wheel_rpm[0]-simulated_wheel_rpm[1]+simulated_wheel_rpm[2]+simulated_wheel_rpm[3])<0)
                reversed=1;
        }
        CHECK(shifted && reversed && run.state==RUN_DONE && abs(run.progress_mm-60)<=1);
    }
    /* Missing fresh IMU cannot fall back to the old position-pulse command. */
    reset_heading_fixture();send_line("test");send_line("l");CHECK(!run_active && !position_count);
    for(direction=0;direction<2;direction++) {
        uint32_t deadline;
        reset_plant(0,1.15);encoder_polarity[1]=encoder_polarity[2]=-1;
        send_line("test");send_line("dist 60");send_line(direction ? "r" : "l");
        deadline=clock_ms+15000;
        while(run_active && clock_ms<deadline)plant_step();
        CHECK(run.state==RUN_DONE && abs(run.progress_mm-60)<=1);
        CHECK(fabs(field_x-(direction ? -60 : 60))<3);
    }
    for(test=0;test<8;test++) {
        uint32_t deadline;
        reset_plant(0,1.15);send_line("test");send_line("l");
        if(test==0)missing_feedback_motor=4;
        if(test==1)missing_velocity_ack=4;
        if(test==2)frozen_encoder=1;
        if(test==3)frozen_imu=1;
        if(test==4)moving_at_stop_motor=4;
        if(test==5)route_test_can_error(1);
        if(test==6)interrupt_at_velocity=velocity_count+1;
        if(test==7)fail_can_at_velocity=velocity_count+1;
        deadline=clock_ms+32000;
        while(run_active && clock_ms<deadline)plant_step();
        CHECK(!run_active && (run.state==RUN_FAULT || run.state==RUN_STOPPED));
        CHECK(run.job==0 && !run.completed_roads && !position_count && stop_count>=4);
        if(test>=6)CHECK(!velocity_batches);
    }
    reset_plant(0,1.15);clock_ms=UINT32_MAX-200;plant_ms=imu_ms=clock_ms;sample_yaw(0);
    send_line("test");send_line("dist 60");send_line("r");
    for(test=0;run_active && test<15000;test++)plant_step();
    CHECK(run.state==RUN_DONE && abs(run.progress_mm-60)<=1);
    send_line("info");CHECK(strstr(output,"[shift closed-loop] DONE_BY_IMU_ENCODER"));
    send_line("f");CHECK(!run_active && !run.translation_only && position_count==4 && drive.state==MOTION_RUNNING);
    stop_request();
    printf("Lateral plant observation (simulation only): max lateral coordinate error %.1fmm; unobserved forward/back displacement %.1fmm under persistent yaw disturbance and retained translation feedforward\n",max_lateral_error,max_unobserved_drift);
    printf("Start 60mm plant observation (simulation only): lateral error %.1fmm, forward/back displacement %.1fmm\n",start_lateral_error,start_forward_drift);
    puts("PASS lateral closed loop: 30 left/right-distance-speed cases; live yaw disturbance recovery; 60RPM cap, encoder X scale, terminal overshoot correction, four-wheel stop, stale IMU/telemetry/ACK, frozen encoder, CAN/stop priority, timer wrap; physical accuracy unverified");
}

static void prepare_reverse_station_leg(void)
{
    reset_plant(0,1.15);send_line("runimu A");
    while(run_active && run.state==RUN_PREFLIGHT)plant_step();
    CHECK(run_active && !velocity_batches);
    run.edge=2;run.job=checked_job=1;run.heading=HEAD_Y_POS;
    CHECK(road_plan(NODE_E,NODE_F,&run.road));run.segment=0;
    run.completed_roads=(uint16_t)(1u << run.road.road);
    planned_x=1200;planned_y=1200;field_x=1200;field_y=1200;
    CHECK(run_segment() && run.travel_sign==-1 && run.target_heading==HEAD_Y_POS);
}

static void station_gating_tests(void)
{
    uint32_t deadline;
    unsigned test;
    RoadHeading heading;
    CHECK(run_station_heading(NODE_F,&heading) && heading==HEAD_X_NEG);
    CHECK(run_station_heading(NODE_D,&heading) && heading==HEAD_X_POS);
    CHECK(run_station_heading(NODE_H,&heading) && heading==HEAD_Y_POS);
    CHECK(!run_station_heading(NODE_B,&heading) && !run_station_heading(NODE_E,&heading));
    prepare_reverse_station_leg();inject_heading=2;deadline=clock_ms+30000;
    while(run_active && run.job<2 && clock_ms<deadline)plant_step();
    CHECK(run.job==2 && reverse_actions==1 && corrected_during_drive && station_alignment_entries==1);
    CHECK(run.heading==HEAD_X_NEG && !run.station_align_pending);
    stop_request();
    for(test=0;test<3;test++) {
        prepare_reverse_station_leg();deadline=clock_ms+30000;
        while(run_active && !run.station_align_pending && clock_ms<deadline)plant_step();
        CHECK(run.station_align_pending && run.job==1 && run.edge==3);
        if(test==0) {moving_at_stop_motor=4;while(run_active && clock_ms<deadline)plant_step();}
        if(test==1) {heading_sensor_invalidate(&heading_sensor);firmware_step();}
        if(test==2)stop_request();
        CHECK(!run_active && run.job==1 && (run.state==RUN_FAULT || run.state==RUN_STOPPED));
        CHECK(!strstr(output,"[station right-side ready]"));
    }
    puts("PASS right-gripper gating: F west / D east / H south; reverse-approach disturbance recovery; no job advance on moving wheel-4, invalid IMU or stop during station alignment");
}

static void core_and_parser_tests(void)
{
    HeadingControl h;
    HeadingSample sample={0};
    int16_t wheels[4];
    const uint8_t position[7]={0x36,1,0xff,0xff,0xff,0xff,0x6b};
    const uint8_t malformed[7]={0x36,2,0,0,0,8,0x6b};
    RunCanFrame frame={0};
    sample.valid=1;sample.sequence=1;sample.received_ms=UINT32_MAX-20;sample.yaw_cdeg=17900;
    CHECK(heading_control_start(&h,sample,18000,UINT32_MAX-20));
    sample.sequence++;sample.received_ms=29;sample.yaw_cdeg=-17900;
    CHECK(heading_control_update(&h,sample,29)==HEADING_CORRECT && h.progress_cdeg==200);
    CHECK(heading_control_rpm(&h,60)==60);
    h.error_cdeg=200;h.rate_cdeg_s=0;CHECK(heading_control_rpm(&h,60)==3);
    h.rate_cdeg_s=10000;CHECK(heading_control_rpm(&h,60)==0);
    heading_control_wheels(90,10,wheels);CHECK(wheels[0]==-80 && wheels[1]==100 && wheels[2]==100 && wheels[3]==-80);
    heading_control_wheels(0,-30,wheels);CHECK(wheels[0]==-30 && wheels[3]==-30);
    heading_control_translate_wheels(0,60,0,wheels);
    CHECK(wheels[0]==-60 && wheels[1]==-60 && wheels[2]==60 && wheels[3]==60);
    heading_control_translate_wheels(1,-60,10,wheels);
    CHECK(wheels[0]==69 && wheels[1]==71 && wheels[2]==-49 && wheels[3]==-51);
    reset_plant(0,1);run_active=1;
    frame.id=0x400;frame.length=7;frame.received_ms=clock_ms;memcpy(frame.data,position,7);
    CHECK(run_record_frame(&frame) && motor_feedback[3].position==-(int64_t)UINT32_MAX);
    memcpy(frame.data,malformed,7);CHECK(!run_record_frame(&frame) && invalid_can_replies==1);
    run.state=RUN_STOPPED;stop_request();
    puts("PASS continuous core/parser: PD units, braking, wheel mapping, signed 32-bit position magnitudes, timestamp wrap");
}

static void continuous_failure_tests(void)
{
    unsigned test;
    reset_plant(0,1);heading_sensor_invalidate(&heading_sensor);send_line("runimu A");CHECK(!run_active && !position_count);
    for(test=0;test<13;test++) {
        uint32_t deadline;
        reset_plant(0,1.2);send_line("turn");send_line("rpm 60");send_line("left90");
        if(test==0)missing_feedback_motor=4;
        if(test==1)missing_velocity_ack=4;
        if(test==2)motor_flags[3]=5;
        if(test==3)bus_off=1;
        if(test==4)route_test_can_error(1);
        if(test==5)frozen_imu=1;
        if(test==6)frozen_encoder=1;
        if(test==7) {
            while(run_active && !velocity_batches)plant_step();
            moving_at_stop_motor=4;
        }
        if(test==8)position_auto_reply=0;
        if(test==9)speed_auto_reply=0;
        if(test==10)yaw_gain=-1.2;
        if(test==11)motor_flags[3]=0;
        if(test==12)motor_flags[3]=9;
        deadline=clock_ms+22000;
        while(run_active && clock_ms<deadline)plant_step();
        CHECK(run.state==RUN_FAULT && !run_active && clock_ms<deadline);
        CHECK(stop_count>=4 && !position_count && drive.state==MOTION_IDLE);
    }
    /* Stop during an active velocity run cancels it permanently. */
    reset_plant(0,1.2);planned_x=2250;planned_y=170;field_x=2250;field_y=170;send_line("runimu A");
    while(run_active && !velocity_batches)plant_step();
    stop_request();CHECK(run.state==RUN_STOPPED && !run_active);
    {uint32_t batches=velocity_batches;clock_ms+=60000;firmware_step();CHECK(velocity_batches==batches);}
    /* Fresh but frozen angle streams reach a bounded deadline, never a successful turn. */
    reset_plant(0,0);send_line("turn");sample_yaw(0);send_line("left90");
    {uint32_t deadline=clock_ms+22000;while(run_active && clock_ms<deadline)plant_step();CHECK(run.state==RUN_FAULT);}
    for(test=0;test<2;test++) {
        uint32_t before_sync;
        reset_plant(0,1.2);send_line("turn");send_line("left90");
        while(run_active && run.state==RUN_PREFLIGHT)plant_step();
        before_sync=sync_count;
        if(test)fail_can_at_velocity=velocity_count+1;
        else interrupt_at_velocity=velocity_count+1;
        while(run_active)plant_step();
        CHECK((run.state==RUN_FAULT || run.state==RUN_STOPPED) && sync_count==before_sync && velocity_batches==0);
    }
    reset_plant(0,1.2);planned_x=2250;planned_y=170;field_x=2250;field_y=170;send_line("runimu A");
    while(run_active && !(run.velocity_phase==VELOCITY_FORWARD && run.progress_mm>100))plant_step();
    {unsigned i;for(i=0;i<4;i++)encoder_value[i]+=run.encoder_sign[i]*500000.0;}
    while(run_active)plant_step();
    CHECK(run.state==RUN_FAULT && strstr(output,"overrun"));
    puts("PASS continuous failures: missing wheel-4 telemetry/ACK, disabled motor, bus off/FIFO, IMU stale/frozen/wrong direction, frozen encoder, stop, nonzero wheel speed");
}

static void adapter_tests(void)
{
    unsigned i;
    uint8_t frame[11]={0x55,0x53,0,0,0,0,0,0x40,0,0,0};
    reset_heading_fixture();
    for(i=0;i<10;i++)frame[10]=(uint8_t)(frame[10]+frame[i]);
    for(i=0;i<11;i++)CHECK(route_test_imu_rx(frame[i],0)==CAR_IMU_UART_ENABLED);
#if CAR_IMU_UART_ENABLED
    CHECK(heading_snapshot().yaw_cdeg==9000 && heading_snapshot().valid);
    send_line("runimu A");CHECK(run_active);CHECK(route_test_imu_rx(0,1));
    firmware_step();CHECK(run.state==RUN_FAULT && imu_uart_fault);
#else
    CHECK(!heading_snapshot().valid);send_line("runimu A");CHECK(!run_active);
#endif
    puts("PASS IMU adapter: configured UART switch, checksum/yaw byte path, UART error invalidation");
}

static void map_reverse_and_postcheck_tests(void)
{
    unsigned heading,travel,history,road;
    for(road=0;road<ROAD_COUNT;road++)for(history=0;history<3;history++)
    for(heading=0;heading<4;heading++)for(travel=0;travel<4;travel++) {
        unsigned reverse=history==1 && (travel+4u-heading)%4u==2u;
        int expected_heading=reverse ? (int)heading : (int)travel;
        reset_heading_fixture();sample_yaw(heading_wrap_cdeg(-(int32_t)heading*9000));
        run.imu_mode=1;run.heading=(RoadHeading)heading;run.yaw_zero_cdeg=0;
        run.road.count=1;run.road.road=(uint8_t)road;
        run.completed_roads=history==0 ? 0 : (uint16_t)(1u << (history==1 ? road : (road+1u)%ROAD_COUNT));
        run.road.segments[0].axis=travel==0 || travel==2 ? BODY_FORWARD : BODY_LEFT;
        run.road.segments[0].mm=travel==0 || travel==1 ? 1000 : -1000;
        CHECK(run_segment());
        CHECK(run.target_heading==(RoadHeading)expected_heading && run.travel_sign==(reverse ? -1 : 1));
        CHECK(run.heading_control.target_cdeg==heading_wrap_cdeg((int32_t)(heading-expected_heading)*9000));
    }
    reset_plant(0,1.2);send_line("turn");send_line("left90");finish_plant(20000);
    {
        uint32_t commands=velocity_count,batches=velocity_batches;
        int16_t done_yaw=turn_postcheck.done_yaw_cdeg;
        unsigned i;
        CHECK(turn_postcheck.active);
        for(i=0;i<3;i++) {
            clock_ms=turn_postcheck.done_ms+(500u<<i);
            sample_yaw(heading_wrap_cdeg((int32_t)done_yaw+300));
            output_size=0;output[0]=0;firmware_step();
            CHECK(strstr(output,"[turn postcheck]") && strstr(output,"change_since_done_cdeg=300"));
            CHECK(run.state==RUN_DONE && !run_active && velocity_count==commands && velocity_batches==batches);
        }
        CHECK(!turn_postcheck.active);
    }
    puts("PASS map/reverse/postcheck: 576 road/history/heading/travel cases, completed current road required for backward travel, drawing-to-IMU handedness, passive 0.5/1/2-second drift logs");
}

static void return_lateral_policy_tests(void)
{
    unsigned plan,history,heading,test;
    for(plan=0;plan<2;plan++)for(history=0;history<3;history++)for(heading=0;heading<4;heading++) {
        RoadHeading initial=plan ? HEAD_Y_NEG : HEAD_Y_POS;
        uint16_t road_bit;
        reset_heading_fixture();run.imu_mode=1;run.plan=(uint8_t)plan;run.edge=FULL_ROUTE_NODES-2u;
        run.initial_heading=initial;run.heading=(RoadHeading)heading;
        CHECK(road_plan(NODE_B,plan ? NODE_C : NODE_A,&run.road));
        road_bit=(uint16_t)(1u<<run.road.road);
        run.completed_roads=history==1 ? road_bit : history==2 ? (uint16_t)(1u<<11) : 0;
        sample_yaw(heading_wrap_cdeg(-(int32_t)heading*9000));
        CHECK(run_segment() && run.travel_axis==BODY_FORWARD);
        CHECK(run.travel_sign==(history==1 ? -1 : 1));
        CHECK(run.target_heading==(history==1 ? initial : plan ? HEAD_Y_POS : HEAD_Y_NEG));
        run.heading=run.target_heading;run.segment=1;
        sample_yaw(heading_wrap_cdeg(-(int32_t)run.heading*9000));
        CHECK(run_segment());
        if(history==1) {
            CHECK(run.travel_axis==BODY_LEFT && run.target_heading==initial);
            CHECK(run.travel_sign==(plan ? -1 : 1));
        } else CHECK(run.travel_axis==BODY_FORWARD && run.target_heading==HEAD_X_POS && run.travel_sign==1);
        CHECK(run.completed_roads==(history==1 ? road_bit : history==2 ? (uint16_t)(1u<<11) : 0));
    }
    /* Final return cannot report job 8 while the last lateral motion has not completed safely. */
    for(test=0;test<3;test++) {
        RoadPoint start;
        uint32_t deadline;
        reset_plant(0,1.15);CHECK(road_point(NODE_C,&start));
        planned_x=start.x;planned_y=start.y;field_x=start.x;field_y=start.y;initial_field_heading=2;
        send_line("runimu C");deadline=clock_ms+300000;
        while(run_active && !(run.edge==15 && run.segment==1 && run.velocity_phase==VELOCITY_FORWARD) && clock_ms<deadline)
            plant_step();
        CHECK(run_active && run.job==7 && run.travel_axis==BODY_LEFT && run.heading==run.initial_heading);
        if(test==0)moving_at_stop_motor=4;
        if(test==1) {heading_sensor_invalidate(&heading_sensor);firmware_step();}
        if(test==2)stop_request();
        while(run_active && clock_ms<deadline)plant_step();
        CHECK(!run_active && run.job==7 && (run.state==RUN_FAULT || run.state==RUN_STOPPED));
    }
    puts("PASS return lateral: 24 start/history/heading cases; reverse requires this completed road; A left / C right 60mm at initial heading; final wheel-4/IMU/stop failures cannot finish job 8");
}

/* 构造经过真实角度模块稳定判据的上一段完成状态，供衔接边界验证。 */
static void prepare_stopped_transition(void)
{
    unsigned i;
    reset_heading_fixture();
    run.imu_mode=1;run_active=1;run.heading=run.target_heading=HEAD_Y_POS;
    run.state=RUN_MOVING;run.velocity_phase=VELOCITY_STOP_FORWARD;
    run.travel_sign=1;run.remaining_mm=100;run.turn_rpm=60;run.transition_ready=1;
    clock_ms=1000;sample_yaw(-9000);
    CHECK(heading_control_start(&run.heading_control,heading_snapshot(),9000,clock_ms));
    clock_ms=1040;sample_yaw(-4500);heading_control_update(&run.heading_control,heading_snapshot(),clock_ms);
    clock_ms=1080;sample_yaw(0);heading_control_update(&run.heading_control,heading_snapshot(),clock_ms);
    clock_ms=1100;sample_yaw(0);heading_control_update(&run.heading_control,heading_snapshot(),clock_ms);
    clock_ms=1250;sample_yaw(0);heading_control_update(&run.heading_control,heading_snapshot(),clock_ms);
    clock_ms=1400;sample_yaw(0);
    CHECK(heading_control_update(&run.heading_control,heading_snapshot(),clock_ms)==HEADING_DONE);
    run.zero_ms=1000;run.stopped_ms=1100;
    for(i=0;i<4;i++) {
        MotorFeedback *f=&motor_feedback[i];
        f->position_valid=f->speed_valid=f->status_replies=1;f->status_flags=1;
        f->position_ms=clock_ms-60;f->speed_ms=f->status_ms=clock_ms;
        f->position=10000+(int)i;
    }
}

static void transition_reuse_tests(void)
{
    unsigned test,i;
    for(test=0;test<11;test++) {
        prepare_stopped_transition();
        if(test==0) {clock_ms+=50;sample_yaw(0);} /* 新样本仍稳定，保留原累计目标。 */
        if(test==1) {clock_ms+=50;sample_yaw(200);}
        if(test==2) {clock_ms+=25;sample_yaw(60);} /* 角差合格但角速度过高。 */
        if(test==3)heading_sensor_invalidate(&heading_sensor);
        if(test==4)motor_feedback[3].speed_rpm=2;
        if(test==5)motor_feedback[3].position_ms=clock_ms-CAR_VELOCITY_FEEDBACK_MS-1;
        if(test==6)run.velocity_waiting=1;
        if(test==7)run.transition_ready=0;
        if(test==8)run.target_heading=HEAD_X_NEG;
        if(test==9)run.station_align_pending=1;
        if(test==10)sample_yaw(40); /* 同毫秒但角度变化，角速度尚不能更新。 */
        run_begin_alignment();
        CHECK(!run.transition_ready && !position_count && !velocity_batches);
        if(test==0) {
            CHECK(run.velocity_phase==VELOCITY_BASELINE && run.baseline_after_ms==1100 && !run.turning);
            CHECK(run.heading_control.target_cdeg==9000 && run.heading_control.progress_cdeg==9000);
            CHECK(run.heading_control.stable_samples>=3);
        } else if(test==3)CHECK(run.state==RUN_FAULT && !run_active);
        else CHECK(run.velocity_phase==VELOCITY_TURN && run.turning && !run.heading_control.stable_samples);
    }
    /* 停稳窗口内、阶段开始前的位置可作为基线，旧旋转阶段的位置不能混入。 */
    for(test=0;test<2;test++) {
        prepare_stopped_transition();run_begin_alignment();
        CHECK(run.velocity_phase==VELOCITY_BASELINE);
        if(test)run.baseline_after_ms=clock_ms-50;
        clock_ms++;sample_yaw(0);run_velocity_update();
        if(test)CHECK(run.velocity_phase==VELOCITY_BASELINE && !velocity_batches);
        else {
            CHECK(run.velocity_phase==VELOCITY_FORWARD);
            for(i=0;i<4;i++)CHECK(run.encoder_start[i]==10000+(int)i);
        }
    }
    prepare_stopped_transition();run.finishing=1;run_begin_alignment();
    CHECK(run.state==RUN_DONE && !run_active && !run.transition_ready);
    /* 新旧采样跨毫秒计时回绕时，停稳窗口比较仍有效。 */
    prepare_stopped_transition();
    {
        uint32_t offset=UINT32_MAX-1379u;
        clock_ms+=offset;heading_sensor.sample.received_ms+=offset;
        run.heading_control.started_ms+=offset;run.heading_control.stable_ms+=offset;run.heading_control.sample_ms+=offset;
        run.zero_ms+=offset;run.stopped_ms+=offset;
        for(i=0;i<4;i++) {
            motor_feedback[i].position_ms+=offset;motor_feedback[i].speed_ms+=offset;motor_feedback[i].status_ms+=offset;
        }
        run_begin_alignment();CHECK(run.velocity_phase==VELOCITY_BASELINE);
        run_velocity_update();CHECK(run.velocity_phase==VELOCITY_FORWARD);
    }
    puts("PASS transition reuse: same-target completion carried once; latest IMU sample checked without rebasing; reject yaw/rate/stale IMU/wheel-4/missing ACK/unverified stop/changed heading/station alignment; cached positions fenced by stopped interval; final completion and timer wrap");
}

static void completed_road_history_tests(void)
{
    unsigned fault;
    for(fault=0;fault<2;fault++) {
        uint32_t deadline;
        RoadPoint start;
        uint16_t first_road=(uint16_t)(1u << road_index(NODE_A,NODE_B));
        uint16_t second_road=(uint16_t)(1u << road_index(NODE_B,NODE_E));
        uint16_t raw_road=(uint16_t)(1u << road_index(NODE_E,NODE_F));
        reset_plant(0,1.15);CHECK(road_point(NODE_A,&start));
        planned_x=start.x;planned_y=start.y;field_x=start.x;field_y=start.y;
        send_line("runimu A");deadline=clock_ms+100000;
        CHECK(!run.completed_roads);
        while(run_active && run.edge==0 && run.segment==0 && clock_ms<deadline)plant_step();
        CHECK(run_active && run.edge==0 && run.segment==1 && !run.completed_roads);
        while(run_active && !(run.edge==2 && !run.turning && run.progress_mm>=100) && clock_ms<deadline)plant_step();
        CHECK(run_active && run.edge==2 && run.progress_mm>=100);
        CHECK(run.completed_roads==(uint16_t)(first_road|second_road));
        CHECK(!(run.completed_roads&raw_road) && run.travel_sign==1);
        if(fault) {heading_sensor_invalidate(&heading_sensor);firmware_step();CHECK(run.state==RUN_FAULT);}
        else {stop_request();CHECK(run.state==RUN_STOPPED);}
        CHECK(!run_active && !(run.completed_roads&raw_road));
        send_line("stop");sample_yaw(wrapped_yaw(physical_yaw));send_line("runimu A");
        CHECK(run_active && !run.completed_roads);
        stop_request();
    }
    puts("PASS completed-road history: multi-segment road recorded only at endpoint, first raw approach forward, partial stop/fault not recorded, every new run clears history");
}

int main(void)
{
    CHECK(legacy_test_main()==0);
    sensor_tests();feedback_tests();motor_diagnostic_tests();core_and_parser_tests();
    continuous_turn_tests();continuous_lateral_tests();continuous_route_tests();station_gating_tests();continuous_failure_tests();adapter_tests();map_reverse_and_postcheck_tests();return_lateral_policy_tests();transition_reuse_tests();completed_road_history_tests();
    printf("PASS heading/feedback: %lu assertions; continuous IMU turn/drive/four-wheel feedback and legacy regressions\n",(unsigned long)assertions);
    return 0;
}
