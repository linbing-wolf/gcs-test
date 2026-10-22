/* 使用真实工位编排函数，验证被删的是重复目标而非抓放/移车/视觉动作。 */
#define STATION_TEST_MAIN unused_full_station_suite_main
#include "verify_station_vision.c"
#include "../src/station_job.c"

int main(void)
{
    static const StationStep expected[]={
        {SS_YAW,1265,0,0,0,0,0},{SS_CLAW,450,0,0,0,0,0},{SS_LIFT,135,0,0,0,0,0},{SS_SLIDE,0,0,0,0,0,0},
        {SS_CHASSIS,300,3,0,0,0,0},{SS_ALIGN,2,0,0,0,0,0},{SS_STACK,3,0,0,0,0,0},
        {SS_PLATFORM_YAW,1265,0,1,0,0,0},{SS_PLATFORM_YAW,1265,0,2,0,0,0},
        {SS_YAW,2560,0,0,0,0,0},{SS_PLATFORM_YAW,1265,0,2,0,0,0},
        {SS_CLAW,150,0,0,0,0,0},{SS_CLAW,450,0,0,0,0,0},{SS_CLAW,150,0,0,0,0,0},
        {SS_LOAD1,0,0,0,0,0,0},{SS_YAW,1265,0,0,0,0,0},{SS_CLAW,150,0,0,0,0,0}
    };
    unsigned i;uint8_t before;
    prepare_arm_auto(0);CHECK(arm_command("arm pose 0 135 126.5 45",clock_ms));
    stn.count=0;
    push(SS_YAW,1265);push(SS_YAW,1265);
    push(SS_CLAW,450);push(SS_CLAW,450);
    push(SS_LIFT,135);push(SS_LIFT,135);
    push(SS_SLIDE,0);push(SS_SLIDE,0);
    push(SS_CHASSIS,300);program[stn.count-1u].ring=3;
    push(SS_YAW,1265);push(SS_ALIGN,2);push(SS_STACK,3);
    push_platform_yaw(1,126.5f);push_platform_yaw(1,126.5f);
    push_platform_yaw(2,126.5f);push(SS_YAW,2560);push_platform_yaw(2,126.5f);
    push(SS_CLAW,150);push(SS_CLAW,450);push(SS_CLAW,150);
    push(SS_LOAD1,0);push(SS_YAW,1265);push(SS_CLAW,150);push(SS_CLAW,150);
    before=stn.count;CHECK(before==24 && compact_station_program()==7);
    CHECK(stn.count==sizeof(expected)/sizeof(expected[0]));
    for(i=0;i<stn.count;i++) {
        CHECK(program[i].kind==expected[i].kind && program[i].value==expected[i].value);
        CHECK(program[i].ring==expected[i].ring && program[i].platform_slot==expected[i].platform_slot);
    }
    CHECK(compact_station_program()==0 && stn.count==17);
    /* 相同云台但杯位首次未知不能跳过；只换盘、只转云台都必须保留。 */
    CHECK(program[0].kind==SS_YAW && program[1].kind==SS_CLAW);
    CHECK(program[7].platform_slot==1 && program[8].platform_slot==2);
    CHECK(program[9].value==2560 && program[10].value==1265);
    /* 并行步骤改变的各目标必须进入缓存；下降/合爪仍是后续独立动作。 */
    stn.count=0;
    push(SS_SLIDE,0);push(SS_YAW,2560);push(SS_CLAW,150);
    push_approach(55,ARM_CLEARANCE_MM,126.5f,2,1);
    push(SS_SLIDE,55);push(SS_YAW,1265);push(SS_CLAW,450);
    push(SS_LIFT,5);push(SS_CLAW,150);push(SS_SLIDE,0);push(SS_YAW,2560);
    CHECK(compact_station_program()==3 && stn.count==8);
    CHECK(program[3].kind==SS_APPROACH && program[3].slide_mm==55
        && program[3].value==1265 && program[3].platform_slot==2 && program[3].open_claw);
    CHECK(program[4].kind==SS_LIFT && program[4].value==5 && program[5].kind==SS_CLAW && program[5].value==150);
    CHECK(program[6].kind==SS_SLIDE && !program[6].value && program[7].kind==SS_YAW && program[7].value==2560);
    CHECK(compact_station_program()==0);
    stn.count=0;
    push(SS_LIFT,5);push(SS_SLIDE,55);push(SS_YAW,1265);
    push_retract_work(256,2);
    push(SS_LIFT,135);push(SS_SLIDE,0);push(SS_YAW,2560);push_platform_yaw(2,256);
    CHECK(compact_station_program()==4 && stn.count==4);
    CHECK(program[3].kind==SS_RETRACT && program[3].value==2560 && program[3].platform_slot==2);
    CHECK(compact_station_program()==0);
    /* 车右侧复合步骤直接包含最终lift，不能保留110mm中间点或误删其他高度。 */
    stn.count=0;
    push_move_to(55,5,1265);push(SS_CLAW,450);
    CHECK(compact_station_program()==1 && stn.count==2);
    CHECK(program[0].kind==SS_APPROACH && program[0].lift_mm==5 && program[1].kind==SS_CLAW);
    CHECK(compact_station_program()==0);
    stn.count=0;
    push_move_to(55,65,1265);push(SS_CLAW,450);
    CHECK(compact_station_program()==1 && stn.count==2 && program[0].lift_mm==65);
    stn.count=0;
    for(i=0;i<224u;i++)push(SS_YAW,(i&1u) ? 2560 : 1265);
    CHECK(compact_station_program()==0 && stn.count==224u);
    stn.count=0;for(i=0;i<224u;i++)push(SS_YAW,1265);
    CHECK(compact_station_program()==223u && stn.count==1u && program[0].value==1265);
    stn.count=0;
    for(i=0;i<2;i++) {push(SS_CHASSIS,300);push(SS_ALIGN,2);push(SS_STACK,3);push(SS_VISION,2);push(SS_LOAD1,0);}
    CHECK(compact_station_program()==0 && stn.count==10u);
    printf("PASS station compaction: %u assertions; first targets kept despite declared pose, repeated scalar targets removed, all grab/release changes and D/E/F/motion barriers retained, unknown/changed platform and macro invalidation preserved, idempotent; no hardware\n",assertions);
    return 0;
}
