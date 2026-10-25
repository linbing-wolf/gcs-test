/* 注入实际入口/反馈异常，检查没有故障停车包、队列取消或任务中止。 */
#define STATION_TEST_MAIN retained_no_auto_stop_station_suite
#include "verify_station_vision.c"

static void moving_fixture(void)
{
    uint32_t deadline;
    reset_plant(0,1.15);CHECK(start_heading_shift(1800,60));
    run.travel_axis=BODY_LEFT;deadline=clock_ms+10000;
    while(run_active && (run.velocity_phase!=VELOCITY_FORWARD || run.progress_mm<50) && clock_ms<deadline)plant_step();
    CHECK(clock_ms<deadline && run_active && run.left_cmd_rpm>0);
}

static void automatic_diagnostics_tests(void)
{
    unsigned variant,i;
    for(variant=0;variant<9;variant++) {
        uint32_t stops,cancel;RunState state;
        moving_fixture();stops=stop_count;cancel=host_tx_cancels[0];state=run.state;
        if(variant==0)motor_flags[0]=0;
        if(variant==1)motor_flags[1]=MOTOR_FLAG_ENABLED|MOTOR_FLAG_PROTECTED;
        if(variant==2)frozen_encoder=1;
        if(variant==3){frozen_imu=1;heading_sensor_invalidate(&heading_sensor);}
        if(variant==4)missing_feedback_motor=4;
        if(variant==5)bus_off=1;
        if(variant==6)for(i=0;i<RX_SIZE+10u;i++)queue_byte('x');
        if(variant==7)run_rx_overflow=1;
        if(variant==8){fail_can_at_velocity=velocity_count+1;run.velocity_sent=0;}
        tick(variant==2 ? 1800 : 900);
        CHECK(run_active && run.state==state && !stop_pending);
        CHECK(stop_count==stops && host_tx_cancels[0]==cancel);
        CHECK(run.left_cmd_rpm>0 && simulated_wheel_rpm[0]);
        if(variant==3 || variant==4) {
            uint32_t batches=velocity_batches;int16_t command=run.left_cmd_rpm;
            tick(300);CHECK(run_active && run.left_cmd_rpm==command && velocity_batches==batches);
        }
        send_line("!");CHECK(!run_active && run.state==RUN_STOPPED && stop_count>stops);
        CHECK(!simulated_wheel_rpm[0] && !simulated_wheel_rpm[1] && !simulated_wheel_rpm[2] && !simulated_wheel_rpm[3]);
        printf("NO_AUTO_STOP variant=%u status/stall/stale/bus/RX warning retained motion; manual ! stopped\n",variant);
        stop_request();
    }
    moving_fixture();
    for(i=0;i<RX_SIZE+10u;i++)queue_byte('x');
    CHECK(rx_overflow && !stop_pending);queue_byte('!');CHECK(stop_pending);
    firmware_step();CHECK(!run_active && run.state==RUN_STOPPED);stop_request();
    puts("PASS ! priority: emergency remains accepted while the input ring is already overflowing");
}

static void numerical_overrun_tests(void)
{
    unsigned i;uint32_t stops;uint8_t state;
    moving_fixture();stops=stop_count;state=run.state;
    for(i=0;i<4;i++) {
        double counts=(run.action_mm+700.0)*CALIBRATED_PPM_X*CAR_ENCODER_COUNTS_PER_REV/CAR_POSITION_PULSES_PER_REV;
        motor_feedback[i].position=run.encoder_start[i]+(int64_t)llround(run.encoder_sign[i]*counts);
        motor_feedback[i].position_ms=clock_ms;
    }
    CHECK(run_velocity_distance() && run.progress_mm>run.action_mm+500);
    run_fault("injected automatic fault");CHECK(run_active && run.state==state && stop_count==stops);
    run_wait_feedback("injected missing feedback");CHECK(run_active && run.left_cmd_rpm>0 && stop_count==stops);
    send_line("!");CHECK(!run_active && stop_count>stops);stop_request();
    puts("PASS numerical overrun: actual count remains beyond target+500, no automatic cancellation/stop or fake completion");
}

static void mechanism_diagnostics_tests(void)
{
    uint32_t stops,deadline;uint16_t yaw;unsigned step;
    prepare_arm_auto(0);yaw=arm_yaw_us;send_line("arm yaw 160");deadline=clock_ms+3000;
    while(arm_yaw_us==yaw && clock_ms<deadline)plant_step();
    CHECK(clock_ms<deadline && arm_busy());yaw=arm_yaw_us;stops=arm_stops;
    arm_can_error(2);tick(40);
    CHECK(arm_stops==stops && !arm_failed() && arm_pose_known() && arm_yaw_us!=yaw);
    CHECK(station_start(NODE_D,1,clock_ms)<0 && station_failed());
    CHECK(arm_stops==stops && !arm_failed() && arm_busy());
    for(step=0;step<20 && arm_busy();step++)tick(100);
    CHECK(arm_stops==stops && !arm_failed());
    arm_lock_fault("injected interlock condition");CHECK(arm_lock.state==2 && arm_stops==stops);
    send_line("!");CHECK(arm_stops>stops && arm_failed());stop_request();
    puts("PASS mechanism diagnostics: CAN/station/interlock conditions do not abort active servo/axes; rejected program stays unresolved; ! still aborts");
}

static unsigned port_stops;
static int port_reject;
static int test_position(void *u,uint8_t a,uint8_t d,uint16_t r,uint8_t c,uint32_t p)
{(void)u;(void)a;(void)d;(void)r;(void)c;(void)p;return !port_reject;}
static int test_sync(void *u){(void)u;return 0;}
static void test_stop(void *u,uint8_t a){(void)u;(void)a;port_stops++;}
static int nav_reject(void *u,MotionParameters p,uint32_t now){(void)u;(void)p;(void)now;return 0;}
static NavMotionEvent nav_busy(void *u,uint32_t now){(void)u;(void)now;return NAV_MOTION_BUSY;}
static void nav_stop_counter(void *u){(void)u;port_stops++;}

static void submission_failure_tests(void)
{
    Motion motion;Navigation navigation;MotionIO io;NavMotionPort port;unsigned rejected;
    reset_plant(0,1.15);port_stops=0;
    io.position=test_position;io.synchronize=test_sync;io.stop=test_stop;io.user=0;
    for(rejected=0;rejected<2;rejected++) {
        MotionParameters p={BODY_FORWARD,100,60,200};port_reject=(int)rejected;
        CHECK(motion_init(&motion,&io,&drive.config));
        CHECK(!motion_start(&motion,p,clock_ms) && motion.state==MOTION_FAULT && !port_stops);
    }
    port.start=nav_reject;port.poll=nav_busy;port.stop=nav_stop_counter;port.user=0;
    CHECK(nav_init(&navigation,NODE_A,HEAD_X_NEG,&port));CHECK(nav_start(&navigation,NODE_B,60));
    nav_update(&navigation,clock_ms);CHECK(navigation.state==NAV_FAULT && !port_stops);
    CHECK(nav_init(&navigation,NODE_A,HEAD_X_NEG,&port));navigation.blocked=4095u;
    CHECK(nav_start(&navigation,NODE_B,60));nav_update(&navigation,clock_ms);
    CHECK(navigation.state==NAV_FAULT && navigation.error==NAV_NO_ROUTE && !port_stops);
    nav_stop(&navigation);CHECK(port_stops==1);
    motion_stop(&motion);CHECK(port_stops==5);
    puts("PASS submission failure: motion/navigation report unresolved failure without stop; explicit stop API retained");
}

static void normal_completion_tests(void)
{
    reset_plant(0,1.15);CHECK(start_heading_shift(100,60));finish_plant(20000);
    CHECK(!simulated_wheel_rpm[0] && !simulated_wheel_rpm[1] && !simulated_wheel_rpm[2] && !simulated_wheel_rpm[3]);
    stop_request();puts("PASS ordinary completion: planned zero velocity and done state retained");
}

int main(void)
{
    automatic_diagnostics_tests();numerical_overrun_tests();mechanism_diagnostics_tests();submission_failure_tests();normal_completion_tests();
    printf("PASS no-auto-stop policy: %u assertions; host model only, no physical fail-safe guarantee\n",assertions);
    return 0;
}
