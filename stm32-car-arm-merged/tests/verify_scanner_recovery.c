/* 运行真实扫码状态机、任务解析器及路线入口；中断事件由主机单独驱动。 */
#define STATION_TEST_MAIN retained_station_suite
#include "verify_station_vision.c"

static const uint8_t qr_ack[]={2,0,0,1,0,0x33,0x31};
static const uint8_t qr_task[]="134+123+314+231\r\n";
static const uint8_t qr_field_task[]="213+213+213+213\r"; /* 操作者日志中的实际16字节。 */

static void qr_deliver(const uint8_t *data,unsigned length)
{
    unsigned i;
    for(i=0;i<length;i++)CHECK(route_test_scanner_rx(data[i],0));
    do {scanner_update();} while(scanner_tail!=scanner_head);
}

static void qr_tx_finish(void)
{
    unsigned count=0;
    while(scanner.tx_active && count++<12) {clock_ms++;route_test_scanner_tx_irq();}
    CHECK(scanner.tx_complete && scanner.tx_offset==9 && !scanner.tx_active);
    scanner_update();CHECK(scanner.waiting_ack);
}

static void qr_at_b(void)
{
    run.station_enabled=1;run.job=0;run.edge=1;run.state=RUN_STATION;run_active=0;
}

static void irq_and_deadline_tests(void)
{
    unsigned i;
    uint32_t transmitted;
    reset_fixture();scanner_init(9600);clock_ms+=CAR_SCANNER_BOOT_MS;scanner_update();
    CHECK(scanner.tx_offset==1 && scanner.tx_active && !scanner.waiting_ack);
    /* 主循环暂停，TXE仍连续推进；发送用了800ms也不挤占发送完成后的ACK时间。 */
    for(i=1;i<9;i++) {clock_ms+=100;route_test_scanner_tx_irq();}
    CHECK(scanner.tx_offset==9 && !scanner.tx_complete && scanner.tx_active);
    clock_ms+=10;route_test_scanner_tx_irq();transmitted=clock_ms;
    clock_ms+=500;scanner_update();
    CHECK(scanner.state==SCANNER_SET_ENDING && scanner.waiting_ack && scanner.phase_ms==transmitted);
    clock_ms=transmitted+CAR_SCANNER_ACK_MS-1;scanner_update();CHECK(scanner.state==SCANNER_SET_ENDING);
    clock_ms++;scanner_update();
    CHECK(scanner.state==SCANNER_FAULT && scanner.fault_reason==SCANNER_ACK_TIMEOUT && scanner.fault_tx==9);
    CHECK(strstr(output,"stage=CRLF reason=ACK_TIMEOUT tx=9/9"));
    reset_fixture();scanner_init(9600);scanner_can_tx=0;clock_ms+=CAR_SCANNER_BOOT_MS;scanner_update();
    clock_ms+=CAR_SCANNER_TX_MS;scanner_update();
    CHECK(scanner.state==SCANNER_FAULT && scanner.fault_reason==SCANNER_TX_TIMEOUT && !scanner.fault_tx);
    reset_fixture();scanner_init(9600);clock_ms+=CAR_SCANNER_BOOT_MS;scanner_update();scanner_can_complete=0;
    for(i=0;i<10;i++)route_test_scanner_tx_irq();
    CHECK(scanner.tx_offset==9 && !scanner.tx_complete && !scanner.waiting_ack);
    clock_ms+=CAR_SCANNER_TX_MS;scanner_update();
    CHECK(scanner.state==SCANNER_FAULT && scanner.fault_reason==SCANNER_TX_TIMEOUT && scanner.fault_tx==9);
    puts("PASS scanner IRQ/deadlines: TX progresses without main loop; TC starts full ACK interval; blocked TX and missing TC distinguished from ACK timeout");
}

static void fault_receiver_tests(void)
{
    unsigned stage;
    for(stage=0;stage<2;stage++) {
        int32_t first,second,locked_first,locked_second;
        reset_fixture();scanner_init(9600);clock_ms+=CAR_SCANNER_BOOT_MS;scanner_update();qr_tx_finish();
        if(stage) {qr_deliver(qr_ack,7);qr_tx_finish();}
        clock_ms+=CAR_SCANNER_ACK_MS;scanner_update();
        CHECK(scanner.state==SCANNER_FAULT && scanner.fault_stage==(stage ? SCANNER_SET_CONTINUOUS : SCANNER_SET_ENDING));
        qr_at_b();
        qr_deliver((const uint8_t *)"111+123+111+123\r\n",17);CHECK(!station_task_ready());
        qr_deliver(qr_task,8);CHECK(!station_task_ready());
        qr_deliver(qr_task+8,9);CHECK(station_task_ready() && scanner.state==SCANNER_FAULT);
        station_task_words(&locked_first,&locked_second);
        qr_deliver((const uint8_t *)"314+231+134+123\r\n",17);
        station_task_words(&first,&second);CHECK(first==locked_first && second==locked_second);
        qr_deliver(qr_ack,7);CHECK(scanner.state==SCANNER_FAULT); /* 迟到ACK不伪造配置成功。 */
        scanner_status();CHECK(strstr(output,"text_rx=enabled") && strstr(output,"[scanner rx hex]"));
        CHECK(scanner.recent_count==32);
        {char init[]="qr init";command(init);}
        CHECK(scanner.state==SCANNER_BOOT && !scanner.fault_reason && !scanner.recent_count);
        CHECK(station_task_ready()); /* 手动重配置不清掉已锁定任务。 */
    }
    puts("PASS scanner FAULT receiver: either config stage can fail; next fragmented valid task accepted, invalid/duplicate task rejected, state remains FAULT, reinit preserves locked task");
}

static void binary_demux_tests(void)
{
    static const uint8_t wrong_ack[]={2,0,0,1,0,0x33,0x30};
    reset_fixture();scanner_init(9600);qr_deliver(qr_ack,3); /* BOOT期间的旧半包不进入新配置应答。 */
    clock_ms+=CAR_SCANNER_BOOT_MS;scanner_update();qr_tx_finish();qr_deliver(qr_ack,7);
    CHECK(scanner.state==SCANNER_SET_CONTINUOUS);
    qr_tx_finish();qr_deliver(qr_ack,3);clock_ms+=CAR_SCANNER_ACK_MS;scanner_update();
    CHECK(scanner.state==SCANNER_FAULT && scanner.fault_ack==3);
    qr_at_b();qr_deliver(qr_task,17);CHECK(station_task_ready());
    reset_fixture();scanner_init(9600);clock_ms+=CAR_SCANNER_BOOT_MS;scanner_update();qr_tx_finish();qr_at_b();
    qr_deliver(wrong_ack,7);CHECK(scanner.waiting_ack && !scanner.results && !scanner.length);
    qr_deliver(qr_task,17);CHECK(station_task_ready() && scanner.state==SCANNER_SET_ENDING);
    station_clear_task();
    qr_deliver((const uint8_t *)"134+",4);qr_deliver(qr_ack,7);
    CHECK(scanner.state==SCANNER_SET_CONTINUOUS && scanner.discard);
    qr_deliver(qr_task+4,13);CHECK(!station_task_ready());
    qr_deliver(qr_task,17);CHECK(station_task_ready());
    qr_tx_finish();qr_deliver(qr_ack,3);CHECK(scanner.ack_offset==3);
    qr_deliver(qr_ack+3,4);CHECK(scanner.state==SCANNER_READY && !scanner.length);
    /* 两份相邻ACK不能跳过第二条命令的真实发送完成。 */
    reset_fixture();scanner_init(9600);clock_ms+=CAR_SCANNER_BOOT_MS;scanner_update();qr_tx_finish();
    {uint8_t pair[14];memcpy(pair,qr_ack,7);memcpy(pair+7,qr_ack,7);qr_deliver(pair,14);}
    CHECK(scanner.state==SCANNER_SET_CONTINUOUS && !scanner.waiting_ack && scanner.tx_offset==1);
    qr_tx_finish();qr_deliver(qr_ack,7);CHECK(scanner.state==SCANNER_READY);
    reset_fixture();clock_ms=UINT32_MAX-500;scanner_init(9600);
    clock_ms+=CAR_SCANNER_BOOT_MS;scanner_update();qr_tx_finish();
    clock_ms+=CAR_SCANNER_ACK_MS;scanner_update();CHECK(scanner.fault_reason==SCANNER_ACK_TIMEOUT);
    puts("PASS scanner demux: bad/binary ACK never joins text, ACK inside partial task fences line, fragmented ACK and adjacent ACKs, millisecond wrap");
}

static void route_fault_recovery_tests(void)
{
    unsigned plan,phase,ending;
    for(plan=0;plan<2;plan++)for(phase=0;phase<2;phase++)for(ending=0;ending<2;ending++) {
        uint32_t deadline;
        unsigned init_calls;
        prepare_arm_auto(plan);send_line(plan ? "runimu C" : "runimu A");deadline=clock_ms+30000;
        while(!run.scan_turn_stage && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        CHECK(run.scan_turn_stage && scanner.state==SCANNER_BOOT);
        init_calls=scanner_init_calls;
        while(scanner.state!=SCANNER_FAULT && run.state!=RUN_FAULT && clock_ms<deadline) {
            if(phase && scanner.state==SCANNER_SET_ENDING)scanner_peer_ack();
            plant_step();
        }
        CHECK(scanner.state==SCANNER_FAULT && run.state!=RUN_FAULT && run.job==0);
        CHECK(scanner.fault_stage==(phase ? SCANNER_SET_CONTINUOUS : SCANNER_SET_ENDING));
        scanner_receive((const uint8_t *)"bad-task\r\n",10);CHECK(!station_task_ready() && !run.job);
        scanner_receive(ending ? qr_field_task : qr_task,ending ? 16 : 17);CHECK(station_task_ready());
        while(run.job==0 && run.state!=RUN_FAULT && clock_ms<deadline)plant_step();
        CHECK(run.job==1 && run.state!=RUN_FAULT && scanner.state==SCANNER_FAULT && scanner_init_calls==init_calls);
        CHECK(abs(run.heading_control.error_cdeg)<=CAR_HEADING_TOLERANCE_CDEG);
        stop_request();
    }
    /* 无B点窗口/急停后，仅显示文本，不重新填入已清除的任务。 */
    reset_fixture();scanner_init(9600);scanner.state=SCANNER_FAULT;
    qr_deliver(qr_task,17);CHECK(!station_task_ready());
    qr_at_b();run.job=1;qr_deliver(qr_task,17);CHECK(!station_task_ready());
    run.job=0;stop_pending=1;qr_deliver(qr_task,17);CHECK(!station_task_ready());
    puts("PASS scanner route: actual A/C first B with either ACK missing resumes on CR-only or CRLF task without qr init, retains heading completion and one-init/first-task/stop gates");
}

static void field_cr_framing_tests(void)
{
    unsigned state,i;
    int32_t first,second;
    uint8_t captured[32];
    memcpy(captured,qr_field_task,16);memcpy(captured+16,qr_field_task,16);
    for(state=SCANNER_BOOT;state<=SCANNER_FAULT;state++) {
        reset_fixture();scanner_init(9600);scanner.state=(ScannerState)state;qr_at_b();
        qr_deliver(captured,sizeof(captured));
        CHECK(scanner.rx_bytes==32 && scanner.results==2 && station_task_ready());
        CHECK(!scanner.length && !scanner.discard);
        station_task_words(&first,&second);CHECK(first==213213 && second==213213);
        for(i=0;i<10;i++)qr_deliver(qr_field_task,16);
        CHECK(scanner.results==12 && !scanner.length && !scanner.discard); /* 不再累积到超长。 */
    }
    reset_fixture();scanner_init(9600);scanner.state=SCANNER_FAULT;qr_at_b();
    qr_deliver(qr_field_task,15);CHECK(!scanner.results && !station_task_ready());
    qr_deliver(qr_field_task+15,1);CHECK(scanner.results==1 && station_task_ready());
    qr_deliver((const uint8_t *)"\n",1);CHECK(scanner.results==1); /* CRLF分次到达不重复计数。 */
    station_clear_task();
    qr_deliver((const uint8_t *)"111+123+111+123\r",16);CHECK(!station_task_ready());
    qr_deliver((const uint8_t *)"134+123+314+231\n",16);CHECK(!station_task_ready()); /* 未扩展为LF单独结束。 */
    qr_deliver(qr_ack,7);CHECK(scanner.state==SCANNER_FAULT && !station_task_ready());
    qr_deliver((const uint8_t *)"213+",4);qr_deliver(qr_ack,7);
    qr_deliver(qr_field_task+4,12);CHECK(!station_task_ready()); /* 二进制插入仍丢弃受损帧。 */
    qr_deliver(qr_field_task,16);CHECK(station_task_ready());
    qr_deliver((const uint8_t *)"134+123+314+231\r\n",17);
    station_task_words(&first,&second);CHECK(first==213213 && second==213213); /* 首码不覆盖。 */
    puts("PASS scanner field CR: exact32 captured bytes across all active config states, repeated CR frames, fragmented CRLF without duplicates, no delimiter/invalid/LF-only/binary rejection and first-task lock");
}

int main(void)
{
    irq_and_deadline_tests();fault_receiver_tests();binary_demux_tests();field_cr_framing_tests();route_fault_recovery_tests();
    printf("PASS scanner recovery: %lu assertions; host simulation only, no hardware accessed\n",(unsigned long)assertions);
    return 0;
}
