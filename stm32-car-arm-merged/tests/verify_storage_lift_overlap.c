/* 两次暂存每次松爪后的实际位置反馈放行；固定在125mm，证明不要求135mm到顶。 */
#define STATION_TEST_MAIN retained_storage_lift_station_suite
#include "verify_station_vision.c"

int main(void)
{
    unsigned plan,batch,release,variant,cases=0;
    CHECK(CAR_ARM_TRANSLATION_OVERLAP_ENABLED && STN_STORAGE_FIRST_THROUGH_ENABLED && STN_STORAGE_SECOND_THROUGH_ENABLED);
    for(plan=0;plan<2;plan++)for(batch=0;batch<2;batch++)for(release=1;release<=3;release++)for(variant=0;variant<3;variant++) {
        uint32_t deadline;unsigned offset=0,placed=0,positions,job=3+3*batch;int64_t base,counts,one;
        float previous=ARM_CLAW_OPEN_DEG;
        int place_lift=STN_STORAGE_LIFT_MM+(batch ? STN_STORAGE_LAYER_LIFT_MM : 0);
        prepare_storage(batch,"task 134+123+314+231");run.plan=(uint8_t)plan;deadline=clock_ms+120000;
        while(placed<release && clock_ms<deadline) {
            float yaw,claw;serve_peer(&offset);plant_step();arm_pose_now(NULL,NULL,&yaw,&claw);
            if(claw!=previous && claw==ARM_CLAW_OPEN_DEG && yaw==STN_STORAGE_YAW_DEG)placed++;
            previous=claw;
        }
        CHECK(placed==release && run.job==job && !run_active);
        base=arm_encoder[1];positions=arm_positions;arm_freeze=1;
        while(arm_positions==positions && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && arm_trace_address[positions]==ARM_LIFT_ADDRESS);
        counts=(int64_t)((ARM_CLEARANCE_MM-place_lift)*ARM_LIFT_PULSES_PER_MM)
            *ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV;
        if(ARM_LIFT_DIR_INVERT)counts=-counts;
        counts*=ARM_LIFT_ENCODER_SIGN;
        one=(int64_t)(ARM_LIFT_PULSES_PER_MM*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV);
        arm_encoder[1]=base+counts-(counts>0 ? one : -one);
        tick(600);CHECK(run.job==job && !run_active && !station_through_departure(NULL));
        if(variant==2) {
            stop_request();tick(100);CHECK(!run_active && !station_active() && arm_failed());cases++;continue;
        }
        if(variant==1) {
            arm_missing_motor=ARM_LIFT_ADDRESS;arm_encoder[1]=base+counts;
            tick(ARM_FEEDBACK_TIMEOUT_MS+200);CHECK(run.job==job && !run_active);
            arm_missing_motor=0;
        }
        arm_encoder[1]=base+counts;
        while(!(run.forward_cmd_rpm || run.left_cmd_rpm) && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && run_active && arm_busy() && arm_encoder[1]==base+counts);
        CHECK(ARM_CLEARANCE_MM<ARM_LIFT_HOME_MM);
        if(release==3)CHECK(run.job==job+1 && run.departure_overlap==2);
        else CHECK(run.job==job && run.station_shift && station_index_overlap_active());
        printf("LIFT_OVERLAP plan=%u batch=%u release=%u fresh_restore=%u lift_mm=%u highest_mm=%u wheel_rpm=%d,%d,%d,%d\n",
            plan,batch,release,variant,(unsigned)ARM_CLEARANCE_MM,(unsigned)ARM_LIFT_HOME_MM,
            run.wheel_rpm[0],run.wheel_rpm[1],run.wheel_rpm[2],run.wheel_rpm[3]);
        cases++;stop_request();
    }
    printf("PASS storage lift overlap: cases=%u assertions=%u; both batches, all3 releases, A/C,124hold/125move below135, missing feedback and !; controlled host model only\n",cases,assertions);
    return 0;
}
