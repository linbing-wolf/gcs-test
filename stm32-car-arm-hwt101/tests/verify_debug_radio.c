/* 逐位驱动模拟串口状态机，接入比赛命令队列；不代表实测115200波形。 */
#define HEADING_TEST_MAIN retained_heading_suite
#include "verify_heading.c"
#define DEBUG_RADIO_HOST_TEST
#include "../src/debug_radio.c"

static uint8_t wire_rx=1,wire_tx=1,tx_timer,rx_timer,start_enabled=1;
static uint32_t fake_primask;
static unsigned tx_levels[50000],tx_count,received_count;
static unsigned fault_count;
static uint8_t received_bytes[1024];
static uint32_t irq_lock(void) {uint32_t previous=fake_primask;fake_primask=1;return previous;}
static void irq_unlock(uint32_t previous) {fake_primask=previous;}
static void tx_write(uint8_t high) {wire_tx=(uint8_t)(high!=0);CHECK(tx_count<50000);tx_levels[tx_count++]=wire_tx;}
static uint8_t rx_level(void) {return wire_rx;}
static void tx_timer_enable(uint8_t enabled) {tx_timer=enabled;}
static void rx_timer_start(void) {rx_timer=1;}
static void rx_timer_full_bit(void) {CHECK(rx_timer);}
static void rx_timer_disable(void) {rx_timer=0;}
static void rx_start_enable(uint8_t enabled) {start_enabled=enabled;}
static void hardware_init(void) {tx_write(1);}
static void receive_record(uint8_t byte) {CHECK(received_count<1024);received_bytes[received_count++]=byte;}
static void receive_error(void) {++fault_count;rx_overflow=1;}

static void receive_frame(uint8_t value,uint8_t stop_high)
{
    unsigned bit;
    wire_rx=0;CHECK(start_enabled);rx_start();CHECK(rx_timer && !start_enabled);
    rx_tick();
    for(bit=0;bit<8;bit++) {wire_rx=(uint8_t)((value>>bit)&1u);rx_tick();}
    wire_rx=stop_high;rx_tick();CHECK(!rx_timer && start_enabled && !rx.busy);
}

static void all_bytes_test(void)
{
    unsigned value,bit;
    DebugRadioStats stats;
    debug_radio_init(receive_record,receive_error);tx_count=received_count=fault_count=0;
    for(value=0;value<256;value++)debug_radio_send((uint8_t)value);
    while(tx_timer)tx_tick();
    CHECK(tx_count==2560 && wire_tx==1 && !tx.busy);
    for(value=0;value<256;value++) {
        CHECK(tx_levels[value*10u]==0 && tx_levels[value*10u+9u]==1);
        for(bit=0;bit<8;bit++)CHECK(tx_levels[value*10u+1u+bit]==((value>>bit)&1u));
        receive_frame((uint8_t)value,1);
        CHECK(received_bytes[value]==value);
    }
    debug_radio_stats(&stats);CHECK(stats.received==256 && !stats.tx_dropped && !stats.framing_errors);
    receive_frame('!',0);CHECK(received_count==256 && fault_count==1);
    wire_rx=1;rx_start();rx_tick();
    debug_radio_stats(&stats);CHECK(stats.framing_errors==1 && stats.false_starts==1 && !rx_timer && start_enabled);
}

static void buffer_and_command_test(void)
{
    unsigned i;
    DebugRadioStats stats;
    reset_plant(0,1.15);debug_radio_init(queue_byte,receive_error);tx_count=0;
    for(i=0;i<RADIO_TX_SIZE+3u;i++)debug_radio_send((uint8_t)i);
    debug_radio_stats(&stats);CHECK(stats.tx_dropped==3);
    CHECK(tx.busy && tx_timer);
    /* 日志满队列时，无需前台解析或换行即可登记人工停止。 */
    receive_frame('!',1);CHECK(stop_pending);
    firmware_step();CHECK(!stop_pending && run.state!=RUN_MOVING);
    while(tx_timer)tx_tick();
    CHECK(tx_count==RADIO_TX_SIZE*10u && !tx.busy);
    fake_primask=1;debug_radio_send('A');CHECK(fake_primask==1);fake_primask=0;
    while(tx_timer)tx_tick();
    tx_count=0;
    for(i=0;i<5000;i++) {debug_radio_send((uint8_t)i);while(tx_timer)tx_tick();if(i%1000==999)tx_count=0;}
    CHECK(!tx.busy && tx.head==tx.tail);
    receive_frame('i',1);firmware_step();
    receive_frame('m',1);firmware_step();
    receive_frame('u',1);firmware_step();
    receive_frame('\r',1);firmware_step();
    CHECK(strstr(output,"[imu]"));
    receive_frame('t',1);firmware_step();
    receive_frame('e',0);CHECK(rx_overflow);firmware_step();
    CHECK(!line_length && !rx_overflow && rx_head==rx_tail);
}

int main(void)
{
    all_bytes_test();buffer_and_command_test();
    straight_test_command_heading();
    printf("PASS debug radio: 256 RX/TX values, framing/false-start handling, nonblocking buffer overflow, wrap, masked IRQ preservation, command and emergency input; %lu assertions\n",(unsigned long)assertions);
    return 0;
}
