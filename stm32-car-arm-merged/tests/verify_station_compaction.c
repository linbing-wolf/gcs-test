/* 使用真实工位编排函数，验证被删的是重复目标而非抓放/移车/视觉动作。 */
#define STATION_TEST_MAIN unused_full_station_suite_main
#include "verify_station_vision.c"
#include "../src/station_job.c"

int main(void)
{
    static const StationStep expected[]={
        {SS_YAW,1265,0,0},{SS_CLAW,450,0,0},{SS_LIFT,135,0,0},{SS_SLIDE,0,0,0},
        {SS_CHASSIS,300,3,0},{SS_ALIGN,2,0,0},{SS_STACK,3,0,0},{SS_STACK_RETURN,0,0,0},
        {SS_PLATFORM_YAW,1265,0,1},{SS_PLATFORM_YAW,1265,0,2},
        {SS_YAW,2560,0,0},{SS_PLATFORM_YAW,1265,0,2},
        {SS_CLAW,150,0,0},{SS_CLAW,450,0,0},{SS_CLAW,150,0,0},
        {SS_LOAD1,0,0,0},{SS_YAW,1265,0,0},{SS_CLAW,150,0,0}
    };
    unsigned i;uint8_t before;
    prepare_arm_auto(0);CHECK(arm_command("arm pose 0 135 126.5 45",clock_ms));
    stn.count=0;
    push(SS_YAW,1265);push(SS_YAW,1265);
    push(SS_CLAW,450);push(SS_CLAW,450);
    push(SS_LIFT,135);push(SS_LIFT,135);
    push(SS_SLIDE,0);push(SS_SLIDE,0);
    push(SS_CHASSIS,300);program[stn.count-1u].ring=3;
    push(SS_YAW,1265);push(SS_ALIGN,2);push(SS_STACK,3);push(SS_STACK_RETURN,0);
    push_platform_yaw(1,126.5f);push_platform_yaw(1,126.5f);
    push_platform_yaw(2,126.5f);push(SS_YAW,2560);push_platform_yaw(2,126.5f);
    push(SS_CLAW,150);push(SS_CLAW,450);push(SS_CLAW,150);
    push(SS_LOAD1,0);push(SS_YAW,1265);push(SS_CLAW,150);push(SS_CLAW,150);
    before=stn.count;CHECK(before==25 && compact_station_program()==7);
    CHECK(stn.count==sizeof(expected)/sizeof(expected[0]));
    for(i=0;i<stn.count;i++) {
        CHECK(program[i].kind==expected[i].kind && program[i].value==expected[i].value);
        CHECK(program[i].ring==expected[i].ring && program[i].platform_slot==expected[i].platform_slot);
    }
    CHECK(compact_station_program()==0 && stn.count==18);
    /* 相同云台但杯位首次未知不能跳过；只换盘、只转云台都必须保留。 */
    CHECK(program[0].kind==SS_YAW && program[1].kind==SS_CLAW);
    CHECK(program[8].platform_slot==1 && program[9].platform_slot==2);
    CHECK(program[10].value==2560 && program[11].value==1265);
    stn.count=0;
    for(i=0;i<224u;i++)push(SS_YAW,(i&1u) ? 2560 : 1265);
    CHECK(compact_station_program()==0 && stn.count==224u);
    stn.count=0;for(i=0;i<224u;i++)push(SS_YAW,1265);
    CHECK(compact_station_program()==223u && stn.count==1u && program[0].value==1265);
    stn.count=0;
    for(i=0;i<2;i++) {push(SS_CHASSIS,300);push(SS_ALIGN,2);push(SS_STACK,3);push(SS_STACK_RETURN,0);push(SS_VISION,2);push(SS_LOAD1,0);}
    CHECK(compact_station_program()==0 && stn.count==12u);
    printf("PASS station compaction: %u assertions; first targets kept despite declared pose, repeated scalar targets removed, all grab/release changes and D/E/F/motion barriers retained, unknown/changed platform and macro invalidation preserved, idempotent; no hardware\n",assertions);
    return 0;
}
