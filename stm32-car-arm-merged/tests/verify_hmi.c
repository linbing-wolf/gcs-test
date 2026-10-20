#define HEADING_TEST_MAIN heading_regression_main
#include "verify_heading.c"
#include <limits.h>

static char display_text[4096];
static void drain_display(void)
{
    uint8_t byte;unsigned n=0,terminators=0,bytes=0;
    while(hmi_tx_next(&byte)) {
        CHECK(++bytes<=1024);
        if(byte==0xff) {
            if(++terminators==3){display_text[n++]='\n';terminators=0;}
        } else {
            CHECK(!terminators && byte>=32 && byte<=126);
            display_text[n++]=(char)byte;
        }
    }
    CHECK(!terminators && !hmi_busy());display_text[n]=0;
}
static void serializer_tests(void)
{
    HmiSnapshot snapshot;unsigned i;
    hmi_init(0);memset(&snapshot,0,sizeof(snapshot));
    snapshot.value[HMI_TASKA]=134123;snapshot.value[HMI_TASKB]=314231;
    snapshot.value[HMI_TASKOK]=1;snapshot.value[HMI_PHASE]=3;
    snapshot.value[HMI_PICK]=snapshot.value[HMI_PLACE]=-1;
    hmi_update(&snapshot,999);CHECK(!hmi_busy());
    hmi_update(&snapshot,1000);CHECK(hmi_busy());
    /* 数据正在发送时，页面请求和新状态不能插入半条旧指令。 */
    snapshot.value[HMI_PHASE]=6;hmi_select(1,2);hmi_update(&snapshot,1200);
    drain_display();CHECK(strstr(display_text,"bkcmd=0\n") && strstr(display_text,"phase=3\n"));
    CHECK(strstr(display_text,"taska=134123\ntaskb=314231\ntaskok=1\n"));
    CHECK(strstr(display_text,"pick=-1\nplace=-1\n") && strstr(display_text,"page 0\n"));
    hmi_update(&snapshot,1201);CHECK(hmi_busy());drain_display();
    CHECK(strstr(display_text,"phase=6\n") && strstr(display_text,"group=2\npage 1\n"));
    hmi_update(&snapshot,1701);CHECK(!hmi_busy());
    snapshot.value[HMI_PHASE]=7;hmi_update(&snapshot,2201);drain_display();
    CHECK(!strcmp(display_text,"phase=7\nclick b0,1\n"));
    hmi_update(&snapshot,6201);drain_display();CHECK(strstr(display_text,"taska=134123"));
    for(i=0;i<HMI_FIELDS;i++)snapshot.value[i]=INT_MIN;
    hmi_resync();hmi_update(&snapshot,6202);drain_display();
    CHECK(strstr(display_text,"vision_age=-2147483648\n") && strstr(display_text,"comm=-2147483648\n"));
    hmi_init(UINT32_MAX-499u);hmi_update(&snapshot,499);CHECK(!hmi_busy());
    hmi_update(&snapshot,500);CHECK(hmi_busy());drain_display();
    puts("PASS HMI serializer: ASCII/three FF, full task transaction, atomic batch, busy transport/page changes, delta refresh, periodic resync, signed32 bounds, boot and timer wrap");
}
static void display_peer(const char *text)
{
    uint8_t frame[64],crc=0;unsigned i,bit,n=(unsigned)strlen(text);
    frame[0]=0xaa;frame[1]=0x55;frame[2]=(uint8_t)n;memcpy(frame+3,text,n);
    for(i=2;i<n+3;i++){crc^=frame[i];for(bit=0;bit<8;bit++)crc=(uint8_t)((crc<<1)^((crc&128)?7:0));}
    frame[n+3]=crc;frame[n+4]=13;
    for(i=0;i<n+5;i++)vision_receive_byte(frame[i],0);
    vision_update(clock_ms);
}
static void hmi_adapter_tests(void)
{
    HmiSnapshot s;FullRouteState original;unsigned i,positions;int16_t offset;uint32_t received;
    reset_plant(0,1.15);screen_snapshot(&s);
    CHECK(s.value[HMI_PHASE]==0 && s.value[HMI_TASKOK]==0 && !s.value[HMI_TASKA]);
    CHECK(s.value[HMI_PICK]==-1 && s.value[HMI_PLACE]==-1 && s.value[HMI_VOLTAGE]==-1);
    CHECK(s.value[HMI_LIFT]==HMI_UNKNOWN && s.value[HMI_MECHANISM]==0);
    send_line("task 134+123+314+231");screen_snapshot(&s);
    CHECK(s.value[HMI_TASKA]==134123 && s.value[HMI_TASKB]==314231 && s.value[HMI_TASKOK]==1);
    send_line("runimu A");screen_snapshot(&s);CHECK(s.value[HMI_PHASE]==1);
    for(i=1;i<8;i++) {run.job=(uint8_t)i;screen_snapshot(&s);CHECK(s.value[HMI_PHASE]==(int32_t)i+1);}
    run.job=1;clock_ms=screen_clock.start_ms+3500;screen_snapshot(&s);CHECK(s.value[HMI_ELAPSED]==3);
    run.state=RUN_DONE;screen_snapshot(&s);CHECK(s.value[HMI_PHASE]==9);
    clock_ms+=5000;screen_snapshot(&s);CHECK(s.value[HMI_ELAPSED]==3);
    run.state=RUN_FAULT;screen_snapshot(&s);CHECK(s.value[HMI_PHASE]==10);
    reset_plant(0,1.15);sample_yaw(-9050);clock_ms+=10;
    motor_feedback[0].speed_valid=1;motor_feedback[0].speed_rpm=-37;motor_feedback[0].speed_ms=clock_ms;
    screen_snapshot(&s);CHECK(s.value[HMI_YAW]==-905 && s.value[HMI_IMU_AGE]==10);
#if HMI_LF_ADDRESS == 1
    CHECK(s.value[HMI_LF_R]==-37);
    clock_ms+=501;screen_snapshot(&s);CHECK(s.value[HMI_LF_R]==HMI_UNKNOWN);
#else
    CHECK(s.value[HMI_LF_R]==HMI_UNKNOWN);
    clock_ms+=501;screen_snapshot(&s);
#endif
    CHECK(s.value[HMI_YAW]==HMI_UNKNOWN && s.value[HMI_COMM]==2);
    reset_plant(0,1.15);send_line("arm on");
    for(i=0;i<500;i++)plant_step();
    send_line("arm pose 0 135 0 45");send_line("arm platform pose 20");
    send_line("arm lift 100");screen_snapshot(&s);
    CHECK(s.value[HMI_LIFT]==100 && s.value[HMI_MECHANISM]==1);
    CHECK(s.value[HMI_PICK]==-1 && s.value[HMI_PLACE]==-1);
    original=run;positions=arm_positions;
    screen_snapshot(&s);send_line("screen sensors");send_line("screen refresh");
    CHECK(!memcmp(&original,&run,sizeof(run)) && arm_positions==positions);
    /* 屏幕不取走MOVE，不确认视觉成功，也不重复发送D。 */
    vision_request_align('D',2,clock_ms);
    for(i=0;i<12;i++)vision_update(clock_ms);
    display_peer("MOVE,27");screen_snapshot(&s);
    CHECK(s.value[HMI_DX]==27 && s.value[HMI_COLOR]==0 && s.value[HMI_VISION_AGE]==0);
    CHECK(vision_align_state()==VISION_PICK_WAITING && vision_align_take_move(&offset,&received) && offset==27);
    vision_cancel_pick();vision_request_pick(2,clock_ms);for(i=0;i<12;i++)vision_update(clock_ms);
    display_peer("PICK,2,351,253");screen_snapshot(&s);
    CHECK(s.value[HMI_COLOR]==2 && s.value[HMI_DX]==31 && s.value[HMI_DY]==13);
    clock_ms+=1001;screen_snapshot(&s);CHECK(!s.value[HMI_COLOR] && s.value[HMI_VISION_AGE]==1001);
    vision_cancel_pick();vision_request_pick(2,clock_ms);for(i=0;i<12;i++)vision_update(clock_ms);
    display_peer("PICK,2");screen_snapshot(&s);
    CHECK(vision_pick_state()==VISION_PICK_READY && !s.value[HMI_COLOR]); /* Legacy PICK still accepted, coordinates are not invented. */
    /* 不消费任何屏幕TX，急停仍走原始优先路径。 */
    hmi_resync();screen_update();CHECK(hmi_busy());stop_request();
    CHECK(!run_active && arm_failed() && arm_lock.state==3 && hmi_busy());
    puts("PASS HMI adapter: four task groups, all phases/time freeze, unknown counts/voltage/servo arrival, fresh/stale feedback, readonly page commands, MOVE ownership, visual age, disconnected TX cannot block emergency");
}
int main(void)
{
    serializer_tests();hmi_adapter_tests();
    printf("PASS HMI integration: %u assertions; no screen/vehicle accessed\n",assertions);return 0;
}
