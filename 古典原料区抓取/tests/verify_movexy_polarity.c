/* 方向反转后CPOS仍在原点一侧：不能把累计位移方向配给新的轮速方向。 */
#define STATION_TEST_MAIN retained_station_suite
#include "verify_station_vision.c"

static void reversal_learning_replay(uint32_t started)
{
    static const int8_t fs[4]={-1,1,1,-1},ls[4]={-1,-1,1,1};
    int64_t counts=(int64_t)(3.0*CALIBRATED_PPM_X*CAR_ENCODER_COUNTS_PER_REV/CAR_POSITION_PULSES_PER_REV);
    reset_plant(0,1.0);run.vector_move=1;run.vector_target_mm[0]=36;run.vector_target_mm[1]=-72;
    clock_ms=started;
    /* 第一段向正方向移动3mm，第二段开始反向但惯性/反馈仍向正方向增长。 */
    run_active=1;run.imu_mode=1;run.velocity_waiting=0;
    for(unsigned step=0;step<3;step++) {
        clock_ms+=60;
        for(unsigned i=0;i<4;i++) {
            run_vector_polarity_command((uint8_t)i,(int8_t)(step ? -1 : 1));
            run.wheel_rpm[i]=step ? -20 : 20;
            motor_feedback[i].position=(int64_t)(step+1)*counts;
            motor_feedback[i].position_ms=motor_feedback[i].speed_ms=clock_ms+1;
            motor_feedback[i].speed_rpm=step==2 ? -20 : 20;motor_feedback[i].speed_valid=1;
        }
        clock_ms++;run_vector_distance();
        if(step<2)CHECK(run.encoder_ready==0);
    }
    printf("REVERSAL_REPLAY mask=%u signs=%d,%d,%d,%d\n",run.encoder_ready,
        run.encoder_sign[0],run.encoder_sign[1],run.encoder_sign[2],run.encoder_sign[3]);
    CHECK(run.encoder_ready==0);
    for(unsigned step=0;step<3;step++) {
        clock_ms+=60;
        for(unsigned i=0;i<4;i++) {
            motor_feedback[i].position=(int64_t)(2-(int)step)*counts;
            motor_feedback[i].position_ms=motor_feedback[i].speed_ms=clock_ms;
        }
        run_vector_distance();
    }
    CHECK(run.encoder_ready==15);
    for(unsigned i=0;i<4;i++)CHECK(run.encoder_sign[i]==1);
    /* 用已确认极性解算用户这次36/-72目标，正反向回补必须保持车身坐标符号。 */
    clock_ms+=60;
    for(unsigned i=0;i<4;i++) {
        motor_feedback[i].position=(int64_t)((36.0*CALIBRATED_PPM_Y*fs[i]-72.0*CALIBRATED_PPM_X*ls[i])
            *CAR_ENCODER_COUNTS_PER_REV/CAR_POSITION_PULSES_PER_REV);
        motor_feedback[i].position_ms=motor_feedback[i].speed_ms=clock_ms;
    }
    run_vector_distance();CHECK(abs(run.vector_progress_mm[0]-36)<=1 && abs(run.vector_progress_mm[1]+72)<=1);
}

static void retained_polarity_tests(void)
{
    for(unsigned mask=0;mask<16;mask++)for(unsigned response=0;response<2;response++) {
        uint32_t deadline;double x,y;unsigned was_shift=0,shifts=0;
        prepare_vision_debug();send_line("vision debug E");wait_debug_request(0);
        for(unsigned i=0;i<4;i++) {
            encoder_polarity[i]=(mask&(1u<<i)) ? -1 : 1;
            wheel_acceleration[i]=(response ? 0.06 : 0.18)+i*0.02;
        }
        peer_reply("MOVE,-30",0);deadline=clock_ms+20000;
        while(clock_ms<deadline) {
            plant_step();if(run.station_shift)was_shift=1;
            if(was_shift && !station_chassis_busy())break;
        }
        CHECK(clock_ms<deadline && run.vision_debug);
        for(unsigned i=0;i<4;i++)CHECK(run.wheel_polarity[i]==encoder_polarity[i]);
        x=field_x;y=field_y;
        peer_reply("MOVEXY,-36,-72",0);peer_reply("MOVEXY,36,72",0);peer_reply("DISC,320,240,0,1",0);
        was_shift=0;deadline=clock_ms+60000;
        while(run.vision_debug && clock_ms<deadline) {
            plant_step();
            if(run.station_shift && !was_shift) {shifts++;CHECK(run.vector_move);}
            if(run.vector_move && run.velocity_phase==VELOCITY_FORWARD) {
                CHECK(run.encoder_ready==15);
                for(unsigned i=0;i<4;i++)CHECK(run.encoder_sign[i]==encoder_polarity[i]);
            }
            was_shift=run.station_shift;
        }
        CHECK(clock_ms<deadline && shifts==2 && run.state==RUN_IDLE);
        CHECK(fabs(field_x-x)<4 && fabs(field_y-y)<4);
        for(unsigned i=0;i<4;i++)CHECK(run.wheel_polarity[i]==encoder_polarity[i]);
    }
    puts("PASS retained polarity: single-axis measurement seeds both36/-72 and reverse MOVEXY before movement; all16 polarities/two wheel responses, FIFO terminal and independent endpoint");
}

static void cold_target_tests(void)
{
    for(unsigned direction=0;direction<4;direction++) {
        char payload[48];uint32_t deadline;double x,y;
        int forward=(direction&1) ? -36 : 36,left=(direction&2) ? -72 : 72;
        prepare_vision_debug();send_line("vision debug D");wait_debug_request(0);x=field_x;y=field_y;
        snprintf(payload,sizeof(payload),"MOVEXY,%d,%d",-forward,left);peer_reply(payload,0);peer_reply("DISC,320,240,0,1",0);
        deadline=clock_ms+30000;
        while(run.vision_debug && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && run.state==RUN_IDLE && fabs(field_x-x-left)<3 && fabs(field_y-y-forward)<3);
    }
    puts("PASS cold MOVEXY36/72: all four quadrants converge without prefilled polarity");
}

int main(void)
{
    reversal_learning_replay(0);reversal_learning_replay(UINT32_MAX-300u);retained_polarity_tests();cold_target_tests();
    printf("PASS MOVEXY polarity: %lu assertions; host model only\n",(unsigned long)assertions);return 0;
}
