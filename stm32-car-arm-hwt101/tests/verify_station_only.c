#define STATION_TEST_MAIN retained_station_suite
#include "verify_station_vision.c"

static const char *commands[3][2]={
    {"runraw","runraw 2"},{"runrough 1","runrough 2"},{"runstorage 1","runstorage 2"}
};
static const uint8_t nodes[3]={NODE_F,NODE_D,NODE_H};
static const RoadHeading headings[3]={CAR_STATION_RAW_HEADING,CAR_STATION_ROUGH_HEADING,CAR_STATION_STORAGE_HEADING};

static void place_at_station(unsigned area)
{
    RoadPoint point;
    CHECK(road_point(nodes[area],&point));
    field_x=planned_x=point.x+(area==1u ? 18 : 0);
    field_y=planned_y=point.y+(area==2u ? 18 : 0);
    physical_yaw=origin_yaw=1230;sample_yaw(1230);
    initial_field_heading=headings[area];
    peer_geometry=1;peer_disc_count=peer_move_count=0;
    peer_plate_dx=18;peer_storage_offset_mm=16;peer_rough_offset_mm=0;peer_storage_first_extra_mm=0;
    checked_job=FULL_ROUTE_JOBS;
}

static void check_station_only(unsigned area,unsigned batch)
{
    unsigned offset=0,vision_frames[128]={0},saw_move=0,saw_yaw=0,attempted_start=0;
    uint32_t deadline;
    uint8_t job=(uint8_t)(area+1u+batch*3u);
    prepare_arm_auto(0);
    send_line("task 134+123+314+231");
    send_line("station auto off");
    place_at_station(area);
    send_line(commands[area][batch]);
    CHECK(run.station_only && run.state==RUN_STATION && run.job==job && !run_active);
    CHECK(RUN_NODES[run.edge]==nodes[area] && run.heading==headings[area]);
    CHECK(run.start_reference_valid && run_target_yaw(&run,run.heading)==1230);
    deadline=clock_ms+150000u;
    while(run.station_only && clock_ms<deadline && !station_failed()) {
        unsigned frame=offset;
        if(!attempted_start && arm_busy()) {
            send_line("runstorage 2");
            CHECK(strstr(output,"station-only rejected") && run.job==job);
            attempted_start=1;
        }
        serve_peer(&offset);
        while(frame<offset) {
            uint8_t command_byte=vision_tx_bytes[frame+3u];
            CHECK(command_byte<sizeof(vision_frames)/sizeof(*vision_frames));
            vision_frames[command_byte]++;
            frame+=vision_tx_bytes[frame+2u]+5u;
        }
        plant_step();
        CHECK(!run_active || (run.station_shift && run.station_only));
        CHECK(!run.departure_overlap && !run.station_departure_edge);
        if(run.station_only)CHECK(run.job==job && run_target_yaw(&run,run.heading)==1230);
        if(run.station_shift && (run.forward_cmd_rpm || run.left_cmd_rpm))saw_move=1;
        if(run.station_shift && run.corner.active)CHECK(!run.corner.prefix_mm);
        if(arm_yaw_us)saw_yaw=1;
    }
    if(clock_ms>=deadline || station_failed())fprintf(stderr,"station-only area=%u batch=%u state=%u %s\n",area,batch,run.state,output);
    CHECK(clock_ms<deadline && !station_failed() && !run.station_only && !station_active() && !run_active);
    CHECK(run.state==RUN_IDLE && arm_parked() && arm_platform_slot()==1 && station_task_ready() && !station_auto_enabled());
    CHECK(peer_wheels_stopped() && saw_move && saw_yaw && attempted_start && arm_positions>0);
    CHECK(!vision_frames['R']);
    if(area==0u) {
        CHECK(vision_frames['H']==1u && !vision_frames['D'] && !vision_frames['E'] && !vision_frames['F']);
#ifdef EXPECT_RAW_C3
        CHECK(vision_frames['T']==1u && vision_frames['C']==3u && !vision_frames['J']);
#else
        CHECK(!vision_frames['T'] && !vision_frames['C'] && vision_frames['J']==3u);
#endif
    } else if(area==1u) {
        RoadPoint point;
        CHECK(vision_frames['D']==1u && !vision_frames['E'] && !vision_frames['F'] && !vision_frames['H']);
        CHECK(road_point(NODE_D,&point) && fabs(field_x-point.x)<12.0);
    } else {
        CHECK(!vision_frames['D'] && !vision_frames['H']);
        CHECK(vision_frames['E']==(batch ? 0u : 1u) && vision_frames['F']==(batch ? 3u : 0u));
    }
    printf("PASS station-only %c batch %u: vision and work complete; parked, task retained, no road\n",'A'+nodes[area],batch+1u);
    stop_request();
}

static void check_rejections_and_stop(void)
{
    unsigned offset=0;
    uint32_t deadline;
    prepare_arm_auto(0);place_at_station(0);
    send_line("runraw");CHECK(!run.station_only && strstr(output,"set task code"));
    send_line("task 134");send_line("runraw 2");CHECK(!run.station_only && strstr(output,"set task code"));
    send_line("runstorage 3");CHECK(!run.station_only && strstr(output,"station-only usage"));
    send_line("task 134+123+314+231");
    motor_feedback[2].speed_valid=0;
    send_line("runraw 1");CHECK(!run.station_only && strstr(output,"fresh stopped wheel feedback"));
    tick(80);send_line("runraw 1");CHECK(run.station_only);
    deadline=clock_ms+18000u;
    while(!run.station_shift && clock_ms<deadline) {serve_peer(&offset);plant_step();}
    CHECK(run.station_only && run.station_shift && run_active);
    stop_request();
    CHECK(!run_active && !run.station_only && !station_active() && !station_task_ready() && run.state==RUN_STOPPED);
    tick(250);CHECK(!run_active && !station_active());
    puts("PASS station-only guards: missing task/batch, invalid command, wheel feedback, ! during correction");
}

int main(void)
{
    for(unsigned area=0;area<3u;area++)for(unsigned batch=0;batch<2u;batch++)check_station_only(area,batch);
    check_rejections_and_stop();
    printf("PASS standalone stations: %u assertions; host model only\n",assertions);
    return 0;
}
