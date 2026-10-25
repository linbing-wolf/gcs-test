/* CAN堵转位仅告警；其他故障与实际位置反馈仍参与控制。 */
#define STATION_TEST_MAIN retained_station_suite
#include "verify_station_vision.c"

static void stall_transition_tests(void)
{
    unsigned wheel;
    for(wheel=0;wheel<4;wheel++) {
        RunCanFrame frame;
        reset_heading_fixture();memset(&frame,0,sizeof(frame));
        frame.id=(wheel+1u)<<CAN_MOTOR_ADDRESS_SHIFT;frame.length=3;
        frame.data[0]=MOTOR_STATUS_COMMAND;frame.data[1]=5;frame.data[2]=MOTOR_REPLY_CHECK_BYTE;
        frame.received_ms=clock_ms;
        output_size=0;output[0]=0;
        CHECK(run_record_frame(&frame) && motor_feedback[wheel].status_flags==5);
        CHECK(strstr(output,"[wheel stall] addr=") && strstr(output,"reported; log only"));
        output_size=0;output[0]=0;CHECK(run_record_frame(&frame));CHECK(!output_size);
        frame.data[1]=1;CHECK(run_record_frame(&frame));CHECK(strstr(output,"; cleared"));
    }
    puts("PASS stall flags: raw5 retained on all four wheels; warning on transition only, recovery logged");
}

static void moving_stall_tests(void)
{
    unsigned wheel,variant,i;
    static const uint8_t unsafe_flags[]={0,9,13};
    for(wheel=0;wheel<4;wheel++)for(variant=0;variant<5;variant++) {
        uint32_t deadline;
        reset_plant(0,1.15);for(i=0;i<4;i++)motor_flags[i]=5;
        CHECK(start_heading_shift(1000,60));deadline=clock_ms+5000;
        while(run_active && run.velocity_phase!=VELOCITY_FORWARD && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && run_active);
        deadline=clock_ms+500;
        while(run_active && clock_ms<deadline)plant_step();
        CHECK(run_active && run.state!=RUN_FAULT && run.progress_mm>0);
        CHECK(motor_feedback[wheel].status_flags==5);
        if(variant<3) {
            motor_flags[wheel]=unsafe_flags[variant];deadline=clock_ms+2000;
            while(run_active && clock_ms<deadline)plant_step();
            CHECK(!run_active && run.state==RUN_FAULT && strstr(output,"motor disabled or protected"));
        } else if(variant==3) {
            send_line("!");CHECK(!run_active && run.state==RUN_STOPPED);
        } else {
            frozen_encoder=1;deadline=clock_ms+5000;
            while(run_active && clock_ms<deadline)plant_step();
            CHECK(!run_active && run.state==RUN_FAULT);
        }
        stop_request();
    }
    puts("PASS moving: stall5 permits real encoder motion; disabled0, protected9/13, emergency and frozen encoder still stop");
}

static void lock_and_station_stall_tests(void)
{
    unsigned wheel,i;
    for(wheel=0;wheel<4;wheel++) {
        reset_plant(0,1.15);for(i=0;i<4;i++)motor_flags[i]=5;
        send_line("arm on");tick(1000);
        CHECK(arm_enabled() && arm_lock.state==2 && !arm_failed());
        CHECK(motor_feedback[wheel].status_flags==5);
        motor_flags[wheel]=9;tick(600);CHECK(arm_lock.state==3 && arm_failed());
        stop_request();

        prepare_rough(0,"task 134+123+314+231");run.station_phase=1;
        for(i=0;i<4;i++) {
            motor_flags[i]=motor_feedback[i].status_flags=5;
            motor_feedback[i].speed_ms=motor_feedback[i].status_ms=clock_ms;
        }
        CHECK(station_chassis_start(150) && run_active && run.station_shift);
        stop_request();
        prepare_rough(0,"task 134+123+314+231");run.station_phase=1;
        motor_feedback[wheel].status_flags=9;
        CHECK(!station_chassis_start(150) && !run_active);stop_request();
    }
    puts("PASS handoff: stall5 allows fresh arm interlock and station child launch; protected9 still rejects/faults");
}

static void automatic_curve_stall_test(void)
{
    unsigned plan,i;uint32_t deadline;
    for(plan=0;plan<2;plan++) {
        prepare_arm_auto(plan);for(i=0;i<4;i++)motor_flags[i]=5;
        send_line(plan ? "runimu C" : "runimu A");deadline=clock_ms+15000;
        while(run_active && !(run.corner.active && run.corner.s_mm>400) && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && run_active && run.corner.active && !arm_failed());
        for(i=0;i<4;i++)CHECK(motor_feedback[i].status_flags==5);
        send_line("!");CHECK(!run_active && arm_failed());stop_request();
    }
    puts("PASS automatic A/C: all four stall5 flags survive real route preflight/curve; global emergency still cancels chassis and arm");
}

int main(void)
{
    stall_transition_tests();moving_stall_tests();lock_and_station_stall_tests();automatic_curve_stall_test();
    printf("PASS wheel stall log-only: %u assertions; host simulation only\n",assertions);
    return 0;
}
