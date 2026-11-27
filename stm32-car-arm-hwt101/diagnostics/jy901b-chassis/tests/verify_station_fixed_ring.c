#define STATION_TEST_MAIN legacy_station_vision_main
#include "verify_station_vision.c"

#ifdef FIXED_RING_TEST_CALIBRATED
static void fixed_job_test(int storage,unsigned batch)
{
    /* 目标顺序 3、1、2，用不相同的两侧姿态检查环位映射。 */
    static const unsigned target[]={2,0,1};
    const int rough_slide[]={STN_ROUGH_RING1_SLIDE_MM,STN_ROUGH_SLIDE_MM,STN_ROUGH_RING3_SLIDE_MM};
    const float rough_yaw[]={STN_ROUGH_RING1_YAW_DEG,133.0f,STN_ROUGH_RING3_YAW_DEG};
    const int storage_slide[]={STN_STORAGE_RING1_SLIDE_MM,STN_STORAGE_SLIDE_MM,STN_STORAGE_RING3_SLIDE_MM};
    const float storage_yaw[]={STN_STORAGE_RING1_YAW_DEG,133.0f,STN_STORAGE_RING3_YAW_DEG};
    const int *slide_by_ring=storage ? storage_slide : rough_slide;
    const float *yaw_by_ring=storage ? storage_yaw : rough_yaw;
    unsigned offset=0,placed=0,picked=0,aligned=0;
    float previous=ARM_CLAW_OPEN_DEG;
    double stop_x=0.0,stop_y=0.0;
    uint32_t deadline;
    uint8_t job=(uint8_t)(storage ? 3 : batch ? 5 : 2);
    if(storage)prepare_storage(0,"task 134+312+314+312");
    else prepare_rough(batch,"task 134+312+314+312");
    if(storage)field_y+=9.0;else field_x+=9.0;
    peer_geometry=1;
    deadline=clock_ms+180000u;
    while(run.job==job && run.state!=RUN_FAULT && clock_ms<deadline) {
        int slide,lift;
        float yaw,claw;
        serve_peer(&offset);
        plant_step();
        if(run.job!=job)break;
        if(!aligned && strstr(output,"[station aligned]")) {
            aligned=1;stop_x=field_x;stop_y=field_y;
        }
        if(aligned) {
            CHECK(!run_active && !run.station_shift && !station_index_overlap_active());
            CHECK(fabs(field_x-stop_x)<0.01 && fabs(field_y-stop_y)<0.01);
        }
        if(!arm_ready())continue;
        arm_pose_now(&slide,&lift,&yaw,&claw);
        if(lift==(storage ? STN_STORAGE_LIFT_MM : STN_ROUGH_PLACE_LIFT_MM)
            && previous==ARM_CLAW_CLOSE_DEG && claw==ARM_CLAW_OPEN_DEG) {
            CHECK(aligned && placed<3 && slide==slide_by_ring[target[placed]] && yaw==yaw_by_ring[target[placed]]);
            ++placed;
        }
        if(!storage && lift==STN_ROUGH_LIFT_MM && previous==ARM_CLAW_OPEN_DEG && claw==ARM_CLAW_CLOSE_DEG) {
            CHECK(aligned && placed==3 && picked<3 && slide==slide_by_ring[target[picked]] && yaw==yaw_by_ring[target[picked]]);
            ++picked;
        }
        previous=claw;
    }
    CHECK(run.job==job+1u && clock_ms<deadline && run.state!=RUN_FAULT);
    CHECK(placed==3 && picked==(storage ? 0u : 3u));
    CHECK(peer_disc_count==1 && peer_move_count>0 && peer_stack_count==0);
    CHECK(!strstr(output,"[station index]") && !run.station_return_mm && arm_parked());
    stop_request();
}

static void stacking_keeps_chassis_correction(void)
{
    unsigned offset=0,move_sent=0,correction_seen=0;
    uint32_t deadline;
    prepare_storage(1,"task 134+123+314+231");
    peer_stack_hold=1;
    deadline=clock_ms+30000u;
    while(run.job==6u && !correction_seen && run.state!=RUN_FAULT && clock_ms<deadline) {
        serve_peer(&offset);
        plant_step();
        if(peer_stack_count==1u && !move_sent && !station_chassis_busy()) {
            peer_reply("MOVE,8",0);
            peer_reply("STACK,320,240,0,1",0);
            move_sent=1;
        }
        correction_seen=strstr(output,"[station stack correction]")!=NULL;
    }
    CHECK(run.state!=RUN_FAULT && move_sent && correction_seen && !peer_disc_count);
    stop_request();
}
#else
static void missing_calibration_test(void)
{
    prepare_arm_auto(0);
    CHECK(station_fixed_ring_calibration_error()!=NULL);
    send_line("runimu A");
    CHECK(strstr(output,"fixed rough ring poses") && !run_active && !arm_positions && !velocity_batches);
}
#endif

int main(void)
{
    CHECK(ARM_YAW_MIN_DEG==-94.5f && ARM_YAW_MAX_DEG==265.5f);
    CHECK(ARM_PICK_YAW_DEG==133.0f && ARM_RAW_CURRENT_YAW_DEG==133.0f);
    CHECK(STN_ROUGH_YAW_DEG==133.0f && STN_STORAGE_YAW_DEG==133.0f && STN_STORAGE_ALIGN_YAW_DEG==133.0f);
    CHECK(ARM_PLACE_SLIDE_MM==40 && ARM_PLACE_LIFT_MM==100 && ARM_PLACE_YAW_DEG==263.0f);
#ifdef FIXED_RING_TEST_CALIBRATED
    CHECK(station_fixed_ring_calibration_error()==NULL);
    fixed_job_test(0,0);
    fixed_job_test(0,1);
    fixed_job_test(1,0);
    stacking_keeps_chassis_correction();
    puts("PASS fixed-ring rough (two batches), first storage, and second storage F/chassis correction");
#else
    missing_calibration_test();
    puts("PASS missing calibration rejects runimu before motion");
#endif
    return 0;
}
