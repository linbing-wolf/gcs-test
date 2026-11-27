/* 观察实际PWM提交时机和升降反馈，保持全部现场标定值。 */
#define STATION_TEST_MAIN retained_platform_station_suite
#include "verify_station_vision.c"

static void arm_ticks(unsigned ms)
{
    while(ms--){clock_ms++;firmware_step();}
}
static void lift_at(int source,int height)
{
    arm_encoder[1]=(ARM_LIFT_DIR_INVERT ? -1 : 1)*(int64_t)(height-source)
        *ARM_LIFT_PULSES_PER_MM*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV;
}
static uint16_t cup_pwm(unsigned slot)
{
    const float degrees[]={ARM_PLATFORM_FIRST_DEG,ARM_PLATFORM_SECOND_DEG,ARM_PLATFORM_THIRD_DEG};
    return (uint16_t)(500.0f+degrees[slot-1u]*2000.0f/270.0f+0.5f);
}
static uint16_t yaw_pwm(float yaw)
{
    float angle=yaw*ARM_YAW_DIRECTION+ARM_YAW_ZERO_DEG-ARM_YAW_OFFSET_DEG;
    return (uint16_t)(500.0f+fminf(360.0f,fmaxf(0.0f,angle))*2000.0f/360.0f+0.5f);
}

static void loaded_preselect_test(int yaw_first)
{
    char pose[80];unsigned moves;uint16_t start;
    uint32_t deadline;
    prepare_arm_auto(0);
    snprintf(pose,sizeof(pose),"arm pose %d %d %.1f %d",ARM_PLACE_SLIDE_MM,
        STN_ROUGH_PLATFORM_TAKE_LIFT_MM,(double)ARM_PLACE_YAW_DEG,ARM_CLAW_CLOSE_DEG);
    send_line(pose);arm_freeze=1;start=arm_platform_us;
    CHECK(arm_transfer_to(STN_STORAGE_SLIDE_MM,STN_STORAGE_LIFT_MM,STN_STORAGE_YAW_DEG,2,clock_ms));
    arm_ticks(90);CHECK(arm_positions==1 && arm_platform_us==start);
    lift_at(STN_ROUGH_PLATFORM_TAKE_LIFT_MM,ARM_PLATFORM_TURN_START_LIFT_MM-1);arm_ticks(100);
    CHECK(arm_platform_us==start && arm_yaw_us==yaw_pwm(ARM_PLACE_YAW_DEG));
    lift_at(STN_ROUGH_PLATFORM_TAKE_LIFT_MM,yaw_first ? ARM_CLEARANCE_MM-1 : ARM_CLEARANCE_MM);
    deadline=clock_ms+ARM_YAW_TIME_MS+200u;
    while(arm_yaw_us!=yaw_pwm(STN_STORAGE_YAW_DEG) && clock_ms<deadline) {
        arm_ticks(1);CHECK(arm_platform_us==start);
    }
    CHECK(clock_ms<deadline);
#if ARM_YAW_SETTLE_MS > 0
    arm_ticks(ARM_YAW_SETTLE_MS-1u);CHECK(arm_platform_us==start);
#endif
    if(yaw_first) {
        arm_ticks(ARM_YAW_SETTLE_MS+ARM_SERVO_TIME_MS+60u);
        CHECK(arm_platform_us==start);
        lift_at(STN_ROUGH_PLATFORM_TAKE_LIFT_MM,ARM_CLEARANCE_MM);
    }
    arm_ticks(ARM_SERVO_TIME_MS+60u);
    CHECK(arm_platform_us==cup_pwm(2) && arm_busy());
    moves=arm_positions;
    lift_at(STN_ROUGH_PLATFORM_TAKE_LIFT_MM,ARM_LIFT_HOME_MM);arm_ticks(100);
#if ARM_CUP_EXIT_DESCENT_DELAY_MS > 100
    CHECK(arm_positions==moves); /* 转杯不再等待升顶后的下降延时。 */
#else
    (void)moves;
#endif
    CHECK(arm_platform_us==cup_pwm(2));
    stop_request();arm_ticks(250);CHECK(arm_failed());
    puts("PASS loaded preselection: target yaw PWM and hold before cup motion, existing clearance and emergency stop retained");
}

static void fixed_station_test(unsigned storage,unsigned batch)
{
    const float storage_yaws[]={STN_STORAGE_RING3_YAW_DEG,STN_STORAGE_RING1_YAW_DEG,STN_STORAGE_YAW_DEG};
    const float rough_yaws[]={STN_ROUGH_RING3_YAW_DEG,STN_ROUGH_RING1_YAW_DEG,STN_ROUGH_YAW_DEG};
    const float *place_yaws=storage ? storage_yaws : rough_yaws;
    const int storage_fixed[]={STN_STORAGE_RING3_SLIDE_MM,STN_STORAGE_RING1_SLIDE_MM,STN_STORAGE_SLIDE_MM};
    const int rough_fixed[]={STN_ROUGH_RING3_SLIDE_MM,STN_ROUGH_RING1_SLIDE_MM,STN_ROUGH_SLIDE_MM};
    const int storage_indexed[]={STN_STORAGE_INDEX_RING3_SLIDE_MM,STN_STORAGE_INDEX_RING1_SLIDE_MM,STN_STORAGE_INDEX_RING2_SLIDE_MM};
    const int rough_indexed[]={STN_ROUGH_INDEX_RING3_SLIDE_MM,STN_ROUGH_INDEX_RING1_SLIDE_MM,STN_ROUGH_INDEX_RING2_SLIDE_MM};
    const int *place_slides=STN_FIXED_RING_ARM_ENABLED ? (storage ? storage_fixed : rough_fixed)
                                                      : (storage ? storage_indexed : rough_indexed);
    unsigned offset=0,taken=0,placed=0,retrieved=0,preselected_out=0,preselected_in=0,trace=0,extensions=0;
    float previous=ARM_CLAW_OPEN_DEG;uint16_t last_pwm;uint32_t deadline;
    uint16_t returning_pwm=0;unsigned returning=0;
    unsigned job=storage ? 3u : batch ? 5u : 2u;
    if(storage)prepare_storage(0,"task 134+312+314+312");
    else prepare_rough(batch,"task 134+312+314+312");
    last_pwm=arm_platform_us;deadline=clock_ms+180000;
    while(run.job==job && clock_ms<deadline) {
        int slide,lift;float yaw,claw;
        float placement_yaw=STN_FIXED_RING_ARM_ENABLED && placed<3 ? place_yaws[placed]
            : (storage ? STN_STORAGE_YAW_DEG : STN_ROUGH_YAW_DEG);
        serve_peer(&offset);plant_step();arm_pose_now(&slide,&lift,&yaw,&claw);
        while(trace<arm_positions) {
            unsigned command=trace++;
            if(taken<=placed || arm_trace_address[command]!=ARM_SLIDE_ADDRESS)continue;
            CHECK(placed<3 && slide==ARM_PLACE_SLIDE_MM);
            CHECK(arm_trace_yaw_us[command]==yaw_pwm(placement_yaw));
            CHECK(arm_trace_pulses[command]==(uint32_t)(abs(place_slides[placed]-ARM_PLACE_SLIDE_MM)*ARM_SLIDE_PULSES_PER_MM+0.5f));
            extensions|=1u<<placed;
        }
        if(returning && ARM_STATION_PRESELECT_ENABLED)CHECK(arm_platform_us==returning_pwm);
        if(arm_platform_us!=last_pwm && taken>placed && taken<3 && claw==ARM_CLAW_CLOSE_DEG) {
            double actual_lift=ARM_LIFT_HOME_MM+(ARM_LIFT_DIR_INVERT ? -1.0 : 1.0)*arm_encoder[1]
                *ARM_PULSES_PER_REV/(ARM_LIFT_PULSES_PER_MM*(double)ARM_ENCODER_COUNTS_PER_REV);
            CHECK(actual_lift+0.01>=ARM_CLEARANCE_MM);
            CHECK(arm_yaw_us==yaw_pwm(placement_yaw));
            preselected_out|=1u<<taken;
        }
        if(arm_platform_us!=last_pwm && !storage && placed==3 && retrieved<3
            && claw==ARM_CLAW_OPEN_DEG && yaw!=ARM_PLACE_YAW_DEG)preselected_in|=1u<<retrieved;
        last_pwm=arm_platform_us;
        if(arm_ready() && previous!=claw) {
            if(claw==ARM_CLAW_CLOSE_DEG && yaw==ARM_PLACE_YAW_DEG) {
                CHECK(taken<3 && arm_platform_slot()==(int)taken+1);taken++;
            } else if(claw==ARM_CLAW_OPEN_DEG && yaw!=ARM_PLACE_YAW_DEG) {
                CHECK(placed<taken);placed++;
                if(taken<3 && ARM_STATION_PRESELECT_ENABLED) {
                    CHECK((preselected_out&(1u<<taken)) && arm_platform_slot()==(int)taken+1);
                }
            } else if(claw==ARM_CLAW_CLOSE_DEG && yaw!=ARM_PLACE_YAW_DEG) {
                CHECK(!storage && retrieved<3);
                if(ARM_STATION_PRESELECT_ENABLED)CHECK(arm_platform_slot()==(int)retrieved+1);
                retrieved++;returning=1;returning_pwm=arm_platform_us;
            } else if(claw==ARM_CLAW_OPEN_DEG && yaw==ARM_PLACE_YAW_DEG) {
                returning=0;
            }
            previous=claw;
        }
    }
    CHECK(clock_ms<deadline && run.job==job+1 && taken==3 && placed==3 && retrieved==(storage ? 0u : 3u));
    if(ARM_STATION_PRESELECT_ENABLED)CHECK((preselected_out&6u)==6u);
    if(!storage && ARM_STATION_PRESELECT_ENABLED)CHECK(preselected_in);
    for(unsigned i=0;i<3;i++)
        CHECK(!!(extensions&(1u<<i))==(place_slides[i]!=ARM_PLACE_SLIDE_MM));
    CHECK(peer_disc_count==1 && !peer_stack_count);
    stop_request();
    printf("PASS %s station: storage=%u batch=%u keeps cup slide through turn and selects next cup at placement yaw\n",
        STN_FIXED_RING_ARM_ENABLED ? "fixed" : "indexed",storage,batch);
}

static void second_storage_test(void)
{
    unsigned offset=0,taken=0,placed=0,early=0;float previous=ARM_CLAW_OPEN_DEG;
    uint16_t last_pwm;uint32_t deadline;
    prepare_storage(1,"task 134+312+314+312");last_pwm=arm_platform_us;deadline=clock_ms+240000;
    while(run.job==6u && clock_ms<deadline) {
        int slide,lift;float yaw,claw;
        serve_peer(&offset);plant_step();arm_pose_now(&slide,&lift,&yaw,&claw);
        if(arm_platform_us!=last_pwm && taken>placed && taken<3 && claw==ARM_CLAW_CLOSE_DEG) {
            CHECK(arm_yaw_us==yaw_pwm(STN_STORAGE_YAW_DEG));
            early|=1u<<taken;
        }
        last_pwm=arm_platform_us;
        if(arm_ready() && previous!=claw) {
            if(claw==ARM_CLAW_CLOSE_DEG && yaw==ARM_PLACE_YAW_DEG) {
                CHECK(taken<3 && arm_platform_slot()==(int)taken+1);taken++;
            } else if(claw==ARM_CLAW_OPEN_DEG && yaw==STN_STORAGE_YAW_DEG) {
                placed++;
                if(taken<3 && ARM_STATION_PRESELECT_ENABLED)CHECK((early&(1u<<taken)) && arm_platform_slot()==(int)taken+1);
            }
            previous=claw;
        }
    }
    CHECK(clock_ms<deadline && run.job==7u && taken==3 && placed==3 && peer_stack_count==3);
    if(ARM_STATION_PRESELECT_ENABLED)CHECK((early&6u)==6u);
    stop_request();puts("PASS second storage: per-target F and chassis indexing retained; next cup waits for the placement yaw PWM");
}

static void raw_loading_test(void)
{
    unsigned slot;
    prepare_arm_auto(0);
    for(slot=1;slot<=3;slot++) {
        unsigned next=slot==3 ? 1 : slot+1,saw_release=0,turned=0;float previous=ARM_CLAW_OPEN_DEG;
        uint16_t last_pwm=arm_platform_us;uint32_t deadline=clock_ms+30000;
        CHECK(arm_load_one_at(ARM_PICK_SLIDE_MM,ARM_PICK_LIFT_MM,ARM_PICK_YAW_DEG,slot,next,slot==3,clock_ms));
        while(arm_busy() && clock_ms<deadline) {
            int slide,lift;float yaw,claw;
            plant_step();arm_pose_now(&slide,&lift,&yaw,&claw);
            if(previous==ARM_CLAW_CLOSE_DEG && claw==ARM_CLAW_OPEN_DEG && yaw==ARM_PLACE_YAW_DEG)saw_release=1;
            if(arm_platform_us!=last_pwm) {
                double actual=ARM_LIFT_HOME_MM+(ARM_LIFT_DIR_INVERT ? -1.0 : 1.0)*arm_encoder[1]
                    *ARM_PULSES_PER_REV/(ARM_LIFT_PULSES_PER_MM*(double)ARM_ENCODER_COUNTS_PER_REV);
                CHECK(saw_release && actual+0.01>=ARM_CLEARANCE_MM);turned=1;
            }
            last_pwm=arm_platform_us;previous=claw;
        }
        CHECK(arm_ready() && saw_release && turned && arm_platform_slot()==(int)next);
    }
    stop_request();puts("PASS raw: three cups in order, next cup turns only after release and measured clearance; last returns slot1");
}

static void stack_order_test(unsigned alternate)
{
    unsigned offset=0,taken=0,placed=0,trace=0;
    unsigned extended=0,retracted=0;
    uint32_t deadline,retract_ms=0;
    uint16_t ring_yaw_pwm=0;
    int64_t retract_encoder=0;
    float previous=ARM_CLAW_OPEN_DEG;
    prepare_storage(1,alternate ? "task 134+231+314+312" : "task 134+123+314+312");
    deadline=clock_ms+240000;
    while((run.job==6u || (run.job==7u && station_active())) && clock_ms<deadline && !station_failed()) {
        int slide,lift;
        float yaw,claw;
        int64_t slide_before=arm_encoder[0];
        serve_peer(&offset);
        plant_step();
        arm_pose_now(&slide,&lift,&yaw,&claw);
        while(trace<arm_positions) {
            unsigned command=trace++;
            if(arm_trace_address[command]!=ARM_SLIDE_ADDRESS)continue;
            if(taken==placed+1u) {
                CHECK(yaw==STN_STORAGE_YAW_DEG && arm_trace_yaw_us[command]==yaw_pwm(STN_STORAGE_YAW_DEG)
                    && slide==ARM_PLACE_SLIDE_MM);
                CHECK(arm_trace_dir[command]==ARM_SLIDE_DIR_INVERT);
                CHECK(arm_trace_pulses[command]==(uint32_t)((STN_STORAGE_STACK_SLIDE_MM-ARM_PLACE_SLIDE_MM)*ARM_SLIDE_PULSES_PER_MM+0.5f));
                CHECK(extended==placed);
                extended++;
            }
            if(placed>retracted && arm_trace_dir[command]!=ARM_SLIDE_DIR_INVERT && !retract_ms) {
                CHECK(yaw==STN_STORAGE_YAW_DEG && slide==STN_STORAGE_STACK_SLIDE_MM);
                retract_ms=clock_ms;
                retract_encoder=arm_encoder[0];
                arm_encoder[0]=slide_before;
            }
        }
        if(retract_ms) {
            CHECK(arm_yaw_us==ring_yaw_pwm);
            if(clock_ms-retract_ms>=ARM_YAW_TIME_MS+ARM_YAW_SETTLE_MS+60u) {
                arm_encoder[0]=retract_encoder;
                retracted++;
                retract_ms=0;
            }
        }
        if(arm_ready() && previous!=claw) {
            if(claw==ARM_CLAW_CLOSE_DEG && yaw==ARM_PLACE_YAW_DEG) {
                CHECK(arm_platform_slot()==(int)taken+1);
                taken++;
            } else if(claw==ARM_CLAW_OPEN_DEG && yaw==STN_STORAGE_YAW_DEG) {
                CHECK(lift==STN_STORAGE_LIFT_MM+STN_STORAGE_LAYER_LIFT_MM);
                placed++;
                ring_yaw_pwm=arm_yaw_us;
            }
            previous=claw;
        }
    }
    CHECK(extended==3 && retracted==3 && placed==3 && taken==3 && peer_stack_count==3);
    CHECK(run.job==7u && clock_ms<deadline && !station_failed() && arm_parked());
    stop_request();
    printf("PASS second storage order %s: all three rings turn before extension, wait for retract feedback before yaw, and use F/STACK\n",
        alternate ? "3-2-1" : "2-1-3");
}

static void release_stop_test(void)
{
    unsigned area;
    for(area=0;area<3;area++) {
        unsigned offset=0;
        uint32_t deadline;
        float work_yaw=area ? STN_STORAGE_YAW_DEG : STN_ROUGH_YAW_DEG;
        int place_lift=area ? STN_STORAGE_LIFT_MM+(area==2u ? STN_STORAGE_LAYER_LIFT_MM : 0)
                            : STN_ROUGH_PLACE_LIFT_MM;
        int64_t lift_feedback;
        uint16_t closed_pwm;
        if(area)prepare_storage(area==2u,"task 134+213+314+213");
        else prepare_rough(0,"task 134+213+314+213");
        deadline=clock_ms+180000;
        while(clock_ms<deadline && !station_failed()) {
            int slide,lift,target_lift;
            float yaw,claw;
            int64_t previous_feedback=arm_encoder[1];
            serve_peer(&offset);plant_step();
            arm_pose_now(&slide,&lift,&yaw,&claw);
            arm_target_now(NULL,&target_lift,NULL,NULL);
            if(yaw==work_yaw && claw==ARM_CLAW_CLOSE_DEG && lift!=place_lift
                && target_lift==place_lift && arm_encoder[1]!=previous_feedback) {
                if(ARM_LIFT_PARALLEL_ENABLED) {
                    /* 下降指令必须在平推到位确认前发出。 */
                    CHECK(slide==ARM_PLACE_SLIDE_MM);
                }
                lift_feedback=arm_encoder[1];
                arm_encoder[1]=previous_feedback;
                break;
            }
        }
        CHECK(clock_ms<deadline && !station_failed() && !station_chassis_busy());
        closed_pwm=arm_claw_us;
        actual_rpm[3]=CAR_VELOCITY_STOP_RPM+5;
        wheel_acceleration[3]=0;
        if(!area) {
            run.start_reference_valid=1;
            station_hold_enabled=1;
            physical_yaw+=300;
        }
        tick(8u*ARM_POLL_MS+20u);
        CHECK(motor_feedback[3].speed_valid && motor_feedback[3].speed_rpm>CAR_VELOCITY_STOP_RPM);
        if(!area)CHECK(station_hold.active && station_hold.command_rpm!=0);
        arm_encoder[1]=lift_feedback;
        deadline=clock_ms+ARM_FEEDBACK_TIMEOUT_MS+ARM_CLAW_TIME_MS;
        while(clock_ms<deadline) {
            plant_step();
            CHECK(arm_claw_us==closed_pwm && !run_active && !station_failed());
        }
        if(!area)CHECK(station_hold.active && station_hold.sent && !station_hold.waiting && !station_hold.command_rpm);
        actual_rpm[3]=0;
        deadline=clock_ms+ARM_FEEDBACK_TIMEOUT_MS+ARM_CLAW_TIME_MS;
        while(arm_claw_us==closed_pwm && clock_ms<deadline)plant_step();
        CHECK(arm_claw_us!=closed_pwm && !run_active && !station_failed());
        stop_request();
    }
    puts("PASS material release: ground approach uses configured axis overlap, claw waits for wheel stop");
}

static void platform_release_while_moving_test(void)
{
    unsigned offset=0;
    int64_t lift_feedback=0;
    uint16_t closed_pwm;
    uint32_t deadline;
    prepare_rough(0,"task 134+213+314+213");
    deadline=clock_ms+180000;
    while(clock_ms<deadline && !station_failed()) {
        int target_lift;
        float target_yaw,claw;
        int64_t previous_feedback=arm_encoder[1];
        serve_peer(&offset);plant_step();
        arm_pose_now(NULL,NULL,NULL,&claw);
        arm_target_now(NULL,&target_lift,&target_yaw,NULL);
        if(claw==ARM_CLAW_CLOSE_DEG && target_yaw==ARM_PLACE_YAW_DEG
            && target_lift==ARM_PLACE_LIFT_MM && arm_encoder[1]!=previous_feedback) {
            lift_feedback=arm_encoder[1];
            arm_encoder[1]=previous_feedback;
            break;
        }
    }
    CHECK(clock_ms<deadline && !station_failed());
    while(station_chassis_busy() && clock_ms<deadline && !station_failed())plant_step();
    CHECK(clock_ms<deadline && !station_failed() && station_active());
    closed_pwm=arm_claw_us;
    actual_rpm[3]=CAR_VELOCITY_STOP_RPM+5;
    wheel_acceleration[3]=0;
    tick(8u*ARM_POLL_MS+20u);
    CHECK(motor_feedback[3].speed_valid && motor_feedback[3].speed_rpm>CAR_VELOCITY_STOP_RPM);
    arm_encoder[1]=lift_feedback;
    deadline=clock_ms+ARM_YAW_TIME_MS+ARM_YAW_SETTLE_MS+ARM_FEEDBACK_TIMEOUT_MS;
    while(arm_claw_us==closed_pwm && clock_ms<deadline && !station_failed())plant_step();
    CHECK(arm_claw_us!=closed_pwm && !station_failed()
        && motor_feedback[3].speed_rpm>CAR_VELOCITY_STOP_RPM);
    stop_request();
    puts("PASS rough retrieval: releases into the cup without the ground-placement wheel-stop gate");
}

static void retain_cup_slide_test(unsigned storage,unsigned batch)
{
    const int rough_slides[]={STN_ROUGH_INDEX_RING3_SLIDE_MM,STN_ROUGH_INDEX_RING1_SLIDE_MM,STN_ROUGH_INDEX_RING2_SLIDE_MM};
    const int storage_slides[]={STN_STORAGE_INDEX_RING3_SLIDE_MM,STN_STORAGE_INDEX_RING1_SLIDE_MM,STN_STORAGE_INDEX_RING2_SLIDE_MM};
    const int *slides=storage ? storage_slides : rough_slides;
    int retain=slides[0]==ARM_PLACE_SLIDE_MM && slides[1]==ARM_PLACE_SLIDE_MM && slides[2]==ARM_PLACE_SLIDE_MM;
    unsigned offset=0,trace=0,moves=0,placed=0,returned=0;
    unsigned job=storage ? 3u : batch ? 5u : 2u;
    float previous=ARM_CLAW_OPEN_DEG;
    uint32_t deadline;
    if(storage)prepare_storage(0,"task 134+312+314+312");
    else prepare_rough(batch,"task 134+312+314+312");
    deadline=clock_ms+150000;
    while((run.job==job || station_active()) && clock_ms<deadline && !station_failed() && run.state!=RUN_FAULT) {
        int slide;
        float yaw,claw;
        serve_peer(&offset);plant_step();arm_pose_now(&slide,NULL,&yaw,&claw);
        if(claw==ARM_CLAW_OPEN_DEG && previous==ARM_CLAW_CLOSE_DEG) {
            if(yaw==ARM_PLACE_YAW_DEG) {
                CHECK(!storage && returned<3u && slide==ARM_PLACE_SLIDE_MM);
                returned++;
            } else {
                CHECK(placed<3u && slide==slides[placed] && !station_chassis_busy());
                if(!placed && retain && CAR_ARM_TRANSLATION_OVERLAP_ENABLED && ARM_LIFT_PARALLEL_ENABLED) {
                    unsigned raise=arm_positions;
                    uint16_t yaw_before=arm_yaw_us,platform_before=arm_platform_us;
                    int64_t start=arm_encoder[1],delta;
                    arm_freeze=1;tick(250);
                    CHECK(arm_positions==raise+1u && arm_trace_address[raise]==ARM_LIFT_ADDRESS);
                    CHECK(!station_chassis_busy() && !station_index_overlap_active());
                    CHECK(arm_yaw_us==yaw_before && arm_platform_us==platform_before);
                    delta=(int64_t)arm_trace_pulses[raise]*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV;
                    arm_encoder[1]=start+(arm_trace_dir[raise] ? -delta : delta);
                    arm_freeze=0;
                }
                placed++;
            }
        }
        previous=claw;
        while(trace<arm_positions) {
            unsigned command=trace++;
            if(arm_trace_address[command]!=ARM_SLIDE_ADDRESS)continue;
            moves++;
            if(retain) {
                CHECK(moves<=2u);
                CHECK(arm_trace_pulses[command]==(uint32_t)(abs(ARM_PLACE_SLIDE_MM-ARM_SLIDE_HOME_MM)*ARM_SLIDE_PULSES_PER_MM+0.5f));
                CHECK(arm_trace_dir[command]==(unsigned)((moves==2u)^ARM_SLIDE_DIR_INVERT));
                if(moves==2u)CHECK(placed==3u && returned==(storage ? 0u : 3u));
            }
        }
    }
    CHECK(clock_ms<deadline && !station_failed() && run.state!=RUN_FAULT && run.job==job+1u);
    if(placed!=3u || returned!=(storage ? 0u : 3u))
        fprintf(stderr,"job=%u placed=%u returned=%u position_commands=%u slide_commands=%u\n",job,placed,returned,arm_positions,moves);
    CHECK(placed==3u && returned==(storage ? 0u : 3u));
    CHECK(arm_ready() && arm_parked() && arm_platform_slot()==1);
    CHECK(retain ? moves==2u : moves>2u);
    printf("PASS indexed cup slide: job=%u, retained=%d, slide_commands=%u, three ground releases and verified parking\n",
        job,retain,moves);
}

int main(int argc,char **argv)
{
    if(argc>1 && !strcmp(argv[1],"retain-slide")) {
        retain_cup_slide_test(0,0);retain_cup_slide_test(0,1);retain_cup_slide_test(1,0);
        return 0;
    }
    if(argc>1 && !strcmp(argv[1],"release-stop")) {
        release_stop_test();platform_release_while_moving_test();return 0;
    }
    if(argc>1 && !strcmp(argv[1],"stack-order")) {
        stack_order_test(0);stack_order_test(1);return 0;
    }
    if(argc>1 && !strcmp(argv[1],"fixed")){fixed_station_test(0,0);return 0;}
    loaded_preselect_test(0);loaded_preselect_test(1);
    release_stop_test();platform_release_while_moving_test();
    fixed_station_test(0,0);fixed_station_test(0,1);fixed_station_test(1,0);
    second_storage_test();raw_loading_test();
    stack_order_test(0);stack_order_test(1);
    printf("PASS platform preselection: %u assertions; PWM/feedback model only\n",assertions);return 0;
}
