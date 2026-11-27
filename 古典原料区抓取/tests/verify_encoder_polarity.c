/* 实际入口回归：缓存复用、混合转向冷启动、换向旧反馈，以及斜行可观测性。 */
#define HEADING_TEST_MAIN retained_encoder_polarity_suite
#include "verify_heading.c"

static void baseline(BodyAxis axis,int sign,unsigned known,unsigned polarity)
{
    HeadingSample yaw;
    reset_plant(1400,1.15);
    sample_yaw(1400);yaw=heading_snapshot();
    run.imu_mode=run.translation_only=1;run.travel_axis=axis;run.travel_sign=(int8_t)sign;
    run_active=1;run.state=RUN_MOVING;run.velocity_phase=VELOCITY_BASELINE;
    run.remaining_mm=860;run.lateral_rpm=run.test_forward_rpm=200;
    run.baseline_after_ms=clock_ms-1;run.phase_ms=clock_ms;
    CHECK(heading_control_start(&run.heading_control,yaw,0,clock_ms));
    for(unsigned i=0;i<4;i++) {
        MotorFeedback *f=&motor_feedback[i];
        f->position=1000000;f->position_valid=f->speed_valid=1;
        f->speed_rpm=0;f->position_ms=f->speed_ms=f->status_ms=clock_ms;
        f->status_replies=1;f->status_flags=MOTOR_FLAG_ENABLED;
        if(known&(1u<<i))run.wheel_polarity[i]=(polarity&(1u<<i)) ? 1 : -1;
    }
    run_velocity_update();
    CHECK(run.velocity_phase==VELOCITY_FORWARD);
}

static void advance_feedback(const int32_t delta[4],const int16_t speed[4],int fresh_speed)
{
    clock_ms+=20;sample_yaw(1400);run_lateral_yaw_sample(heading_snapshot());
    for(unsigned i=0;i<4;i++) {
        motor_feedback[i].position=run.encoder_start[i]+delta[i];
        motor_feedback[i].position_ms=clock_ms;
        if(fresh_speed){motor_feedback[i].speed_rpm=speed[i];motor_feedback[i].speed_ms=clock_ms;}
    }
    run.velocity_waiting=0;run.acks=15;
    CHECK(run_velocity_distance());
}

static void reuse_test(void)
{
    static const int8_t signs[2][4]={{-1,1,1,-1},{-1,-1,1,1}};
    for(unsigned axis=0;axis<2;axis++)for(unsigned reverse=0;reverse<2;reverse++)
        for(unsigned mask=0;mask<16;mask++) {
            int sign=reverse ? -1 : 1;
            baseline((BodyAxis)axis,sign,15,mask);
            CHECK(run.encoder_ready==15);
            for(unsigned i=0;i<4;i++)
                CHECK(run.encoder_sign[i]==((mask&(1u<<i)) ? 1 : -1)*signs[axis][i]*sign);
        }
    /* 已知物理极性后，即使新CPOS初期与主轴相反，也不得重新猜或覆盖缓存。 */
    baseline(BODY_LEFT,1,15,0);
    {const int32_t delta[4]={-500,-500,-500,-500};const int16_t speed[4]={-12,-12,-28,-28};
     advance_feedback(delta,speed,1);advance_feedback(delta,speed,1);}
    CHECK(run.encoder_sign[0]==1 && run.encoder_sign[1]==1 && run.encoder_sign[2]==-1 && run.encoder_sign[3]==-1);
    for(unsigned i=0;i<4;i++)CHECK(run.wheel_polarity[i]==-1);
    puts("PASS cached physical polarity: all16 polarities x four directions; startup samples cannot overwrite it");
}

static void mixed_cold_start_test(void)
{
    static const unsigned known_masks[4]={0,5,10,15};
    for(unsigned mask=0;mask<16;mask++)for(unsigned known=0;known<4;known++) {
        int32_t delta[4];int16_t speed[4];
        baseline(BODY_LEFT,1,known_masks[known],mask);
        /* 轴项8RPM，转向20RPM；前两轮实际指令与纯左移的预期符号相反。 */
        CHECK(run_velocity_send(0,8,20));
        for(unsigned n=1;n<=3;n++) {
            for(unsigned i=0;i<4;i++) {
                int polarity=mask&(1u<<i) ? 1 : -1;
                delta[i]=polarity*run.wheel_rpm[i]*(int32_t)n*100;
                speed[i]=(int16_t)(polarity*run.wheel_rpm[i]);
            }
            advance_feedback(delta,speed,1);
        }
        CHECK(run.encoder_ready==15);
        for(unsigned i=0;i<4;i++) {
            int polarity=mask&(1u<<i) ? 1 : -1;
            CHECK(run.wheel_polarity[i]==polarity);
            CHECK(run.encoder_sign[i]==polarity*(i<2 ? -1 : 1));
        }
    }
    puts("PASS cold mixed turn: all16 polarities x four known masks follow actual wheel command");
}

static void stale_and_reversal_test(void)
{
    int32_t delta[4];int16_t speed[4];
    baseline(BODY_LEFT,1,0,0);CHECK(run_velocity_send(0,8,20));
    for(unsigned n=1;n<=2;n++) {
        for(unsigned i=0;i<4;i++){delta[i]=-run.wheel_rpm[i]*(int32_t)n*100;speed[i]=(int16_t)-run.wheel_rpm[i];}
        advance_feedback(delta,speed,0);
    }
    CHECK(!run.encoder_ready); /* 新CPOS不能配命令之前的速度来确认极性。 */
    for(unsigned i=0;i<4;i++){motor_feedback[i].speed_rpm=speed[i];motor_feedback[i].speed_ms=clock_ms;}
    run.velocity_waiting=0;CHECK(run_velocity_send(0,-8,-20));
    for(unsigned n=3;n<=4;n++) {
        for(unsigned i=0;i<4;i++)delta[i]=run.wheel_rpm[i]*(int32_t)n*100;
        advance_feedback(delta,speed,1);CHECK(!run.encoder_ready); /* 旧方向仍在转，不能落锁。 */
    }
    for(unsigned i=0;i<4;i++)speed[i]=0;
    advance_feedback(delta,speed,1);CHECK(!run.encoder_ready);
    for(unsigned n=1;n<=4;n++) {
        for(unsigned i=0;i<4;i++){delta[i]=run.wheel_rpm[i]*400-run.wheel_rpm[i]*(int32_t)n*100;speed[i]=(int16_t)-run.wheel_rpm[i];}
        advance_feedback(delta,speed,1);
    }
    CHECK(run.encoder_ready==15);
    for(unsigned i=0;i<4;i++)CHECK(run.wheel_polarity[i]==-1);
    puts("PASS learning interval: reject stale speed and old-direction movement; confirm only after reversal feedback");
}

static void diagonal_observability_test(void)
{
    const int32_t delta[4]={3200,0,-3200,0};const int16_t speed[4]={16,0,-16,0};
    baseline(BODY_LEFT,1,15,0);
    advance_feedback(delta,speed,1);
    CHECK(run.encoder_ready==15 && run.lateral_pose.valid);
    CHECK(run.lateral_pose.left_mm>5 && run.lateral_pose.forward_mm>5);
    /* 原来单轮符号学反时，这组真实斜行增量在along/cross中同时抵消为0。 */
    CHECK(run.progress_mm>5 && run.straight_hold.lateral_mm>5);
    puts("PASS diagonal observability: forward8/left8 motion accumulates both encoder pose components");
}

static void zero_resume_test(void)
{
    for(unsigned vector=0;vector<2;vector++) {
        int32_t delta[4];int16_t speed[4];
        baseline(BODY_LEFT,1,0,0);run.vector_move=(uint8_t)vector;CHECK(run_velocity_send(0,20,0));
        for(unsigned i=0;i<4;i++){delta[i]=(i<2 ? 100 : -100);speed[i]=(i<2 ? 20 : -20);}
        advance_feedback(delta,speed,1);CHECK(!run.encoder_ready);
        run.velocity_waiting=0;CHECK(run_velocity_send(0,0,0));
        for(unsigned n=2;n<=4;n++) {
            for(unsigned i=0;i<4;i++)delta[i]=(i<2 ? 1 : -1)*(int32_t)n*200;
            advance_feedback(delta,speed,1);CHECK(!run.encoder_ready); /* 零指令期间不确认物理极性。 */
        }
        run.velocity_waiting=0;CHECK(run_velocity_send(0,20,0));
        for(unsigned n=5;n<=6;n++) {
            for(unsigned i=0;i<4;i++)delta[i]=(i<2 ? 1 : -1)*(int32_t)n*200;
            advance_feedback(delta,speed,1);
        }
        CHECK(run.encoder_ready==15); /* 同方向恢复，不要求再次物理过零。 */
        for(unsigned i=0;i<4;i++)CHECK(run.wheel_polarity[i]==-1);
    }
    puts("PASS zero/resume: do not learn at zero command; same-direction coast resumes without re-crossing zero");
}

int main(int argc,char **argv)
{
    unsigned mode=argc>1 ? (unsigned)atoi(argv[1]) : 0;
    if(mode==0)reuse_test();
    else if(mode==1)mixed_cold_start_test();
    else if(mode==2)stale_and_reversal_test();
    else if(mode==3)diagonal_observability_test();
    else zero_resume_test();
    printf("PASS encoder polarity mode%u: %u assertions; model only\n",mode,assertions);return 0;
}
