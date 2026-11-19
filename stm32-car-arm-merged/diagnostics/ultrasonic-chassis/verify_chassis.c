#define main old_entry_tests
#include "../../tests/verify_entry.c"
#undef main

static double actual[4], encoder[4], field_x, field_y, yaw;
static int initial_field_heading;
static uint8_t checked_job;

static void reset_plant(void)
{
    unsigned i;
    reset_fixture();heading_sensor_reset(&heading_sensor);
    can_rx_fault=imu_uart_fault=0;
    memset(motor_feedback,0,sizeof(motor_feedback));
    memset(motor_flags,1,sizeof(motor_flags));
    memset(actual,0,sizeof(actual));
    for (i=0;i<4;i++) encoder[i]=(i&1u) ? -1000000.0 : 1000000.0;
    field_x=field_y=yaw=0.0;checked_job=0;
    heading_sensor_publish(&heading_sensor,0,clock_ms);
}

static void plant_step(void)
{
    static const uint8_t expected[FULL_ROUTE_JOBS]={NODE_B,NODE_F,NODE_D,NODE_H,NODE_F,NODE_D,NODE_H,NODE_A};
    double f,l,t,a;
    unsigned i;
    clock_ms++;
    for (i=0;i<4;i++) {
        double delta=simulated_wheel_rpm[i]-actual[i];
        actual[i]+=delta>0.18 ? 0.18 : delta< -0.18 ? -0.18 : delta;
        encoder[i]+=actual[i]*65536.0/60000.0;
        simulated_encoder[i]=(int64_t)llround(encoder[i]);
        reported_wheel_rpm[i]=(int16_t)lround(actual[i]);
    }
    f=(-actual[0]+actual[1]+actual[2]-actual[3])/4.0;
    l=(-actual[0]-actual[1]+actual[2]+actual[3])/4.0;
    t=(actual[0]+actual[1]+actual[2]+actual[3])/4.0;
    a=(initial_field_heading*9000.0-yaw)*3.141592653589793/18000.0;
    f*=3200.0/(60000.0*CALIBRATED_PPM_Y);l*=3200.0/(60000.0*CALIBRATED_PPM_X);
    field_x+=-sin(a)*f+cos(a)*l;field_y+=cos(a)*f+sin(a)*l;
    yaw+=t*0.115;
    if (clock_ms%10u==0) heading_sensor_publish(&heading_sensor,heading_wrap_cdeg((int32_t)lround(yaw)),clock_ms);
    output_size=0;output[0]=0;firmware_step();
    if (run.job>checked_job) {
        uint8_t node=run.job==FULL_ROUTE_JOBS && run.plan ? NODE_C : expected[run.job-1u];
        CHECK(run.job==checked_job+1u && RUN_NODES[run.edge]==node);
        if (node==NODE_B || node==NODE_F) CHECK(run.heading==HEAD_X_NEG);
        if (node==NODE_D) CHECK(run.heading==HEAD_X_POS);
        if (node==NODE_H) CHECK(run.heading==HEAD_Y_POS);
        checked_job=run.job;
    }
}

static uint8_t test_sensor;
static uint32_t shot_ms;
static unsigned max_wheel_rpm;

static void start_at(uint8_t node)
{
    reset_plant();
    initial_field_heading=node==NODE_C ? HEAD_Y_NEG : HEAD_X_NEG;
    test_sensor=0;shot_ms=clock_ms;max_wheel_rpm=0;
    CHECK(start_run(node));
    CHECK(!run.station_enabled && (int)run.initial_heading==initial_field_heading);
}

static void step_with_sonar(int front,int left)
{
    unsigned i;
    if (clock_ms-shot_ms>=70u) {
        obstacle_io_shot(test_sensor);
        obstacle_io_result(test_sensor,test_sensor ? left : front);
        test_sensor^=1u;shot_ms=clock_ms;
    }
    plant_step();
    for (i=0;i<4;i++) {
        unsigned rpm=(unsigned)abs(simulated_wheel_rpm[i]);
        if (rpm>max_wheel_rpm) max_wheel_rpm=rpm;
    }
}

static void finish_route(void)
{
    uint32_t deadline=clock_ms+240000u;
    while (run_active && clock_ms<deadline) step_with_sonar(-1,-1);
    if(run.state!=RUN_DONE) fprintf(stderr,"state=%u job=%u edge=%u %u->%u segment=%u phase=%u obstacle=%u progress=%ld heading=%u goal=%u log=%s\n",
        run.state,run.job,run.edge,run.road.from,run.road.to,run.segment,run.velocity_phase,run.obstacle_phase,(long)run.progress_mm,run.heading,run.target_heading,output);
    CHECK(run.state==RUN_DONE && run.job==FULL_ROUTE_JOBS);
    CHECK(max_wheel_rpm>0u && max_wheel_rpm<=OBSTACLE_WHEEL_RPM);
    CHECK(hypot(field_x,field_y)<100.0);
}

static void obstacle_on_launch(uint8_t node)
{
    int road=road_index(node,NODE_B);
    int sensor=node==NODE_C ? 0 : 1;
    uint32_t deadline;
    start_at(node);deadline=clock_ms+20000u;
    while (clock_ms<deadline && !(obstacle_sensor_for_leg()==sensor && run.segment==1u
            && run.progress_mm>=100)) step_with_sonar(-1,-1);
    CHECK(clock_ms<deadline && !run.blocked_roads);
    obstacle_io_shot((uint8_t)sensor);obstacle_io_result((uint8_t)sensor,420);
    CHECK(!run.blocked_roads);
    obstacle_io_shot((uint8_t)(sensor^1));obstacle_io_result((uint8_t)(sensor^1),50);
    CHECK(!run.blocked_roads);
    obstacle_io_shot((uint8_t)sensor);obstacle_io_result((uint8_t)sensor,-1);
    CHECK(!run.blocked_roads && obstacle_hits==0u);
    deadline=clock_ms+1000u;
    while (clock_ms<deadline && !run.blocked_roads)
        step_with_sonar(sensor ? 50 : 420,sensor ? 420 : 50);
    CHECK(run.blocked_roads==(1u<<road) && run.state==RUN_OBSTACLE_STOP);
    deadline=clock_ms+35000u;
    while (clock_ms<deadline && run_active && run.obstacle_phase) step_with_sonar(-1,-1);
    CHECK(clock_ms<deadline && run_active && !run.obstacle_phase && run.job==0u);
    CHECK(RUN_NODES[0]==node && RUN_NODES[1]==(node==NODE_C ? NODE_D : NODE_F));
    CHECK(hypot(field_x,field_y)<60.0);
    finish_route();
}

int main(void)
{
    start_at(NODE_A);finish_route();CHECK(!run.blocked_roads);
    start_at(NODE_C);finish_route();CHECK(!run.blocked_roads);
    obstacle_on_launch(NODE_A);obstacle_on_launch(NODE_C);
    start_at(NODE_C);
    {uint32_t deadline=clock_ms+20000u;
     while (clock_ms<deadline && !(obstacle_sensor_for_leg()==0 && run.segment==1u)) step_with_sonar(-1,-1);
     CHECK(clock_ms<deadline && run.progress_mm==0);
     while (clock_ms<deadline && !run.blocked_roads) step_with_sonar(300,-1);
     CHECK(clock_ms<deadline && run.state==RUN_OBSTACLE_STOP && abs((int)run.progress_mm)<2);}
    queue_byte('!');plant_step();CHECK(!run_active && run.state==RUN_STOPPED);
    start_at(NODE_A);
    run.obstacle_phase=2;run.obstacle_node=NODE_A;
    run.blocked_roads=(uint16_t)((1u<<road_index(NODE_A,NODE_B))|(1u<<road_index(NODE_A,NODE_F)));
    run_return_wait();CHECK(!run_active && run.state==RUN_WAIT_OBSTACLE);
    puts("PASS chassis obstacle: both full routes and detours, front/left association, isolated outlier and invalid echoes, pre-motion detection, blocked exit hold, manual stop and wheel speed bound");
    return 0;
}
