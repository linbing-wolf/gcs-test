#include "vision_link.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t tx[32];
static unsigned tx_count;
static int saw_movexy, saw_offsets;

void arm_output(const char *message)
{
    if (strstr(message,"MOVEXY,-40,25 (px)")) saw_movexy=1;
    if (strstr(message,"offset_px=-40 offset_y_px=25")) saw_offsets=1;
    fputs(message,stdout);
}
void entry_host_vision_init(uint32_t baud) {assert(baud==115200u);}
int entry_host_vision_tx_ready(void) {return 1;}
void entry_host_vision_send_byte(uint8_t byte)
{
    assert(tx_count<sizeof(tx));
    tx[tx_count++]=byte;
}

static uint8_t crc_byte(uint8_t crc,uint8_t value)
{
    unsigned bit;
    crc^=value;
    for (bit=0;bit<8u;bit++) crc=(uint8_t)((crc<<1)^((crc&128u)?7u:0u));
    return crc;
}

static void reply(const char *body,uint32_t now)
{
    size_t i,n=strlen(body);
    uint8_t crc=crc_byte(0,(uint8_t)n);
    vision_receive_byte(0xAAu,0);vision_receive_byte(0x55u,0);
    vision_receive_byte((uint8_t)n,0);
    for (i=0;i<n;i++) {
        crc=crc_byte(crc,(uint8_t)body[i]);
        vision_receive_byte((uint8_t)body[i],0);
    }
    vision_receive_byte(crc,0);vision_receive_byte(0x0Du,0);
    vision_update(now);
}

static void finish_tx(uint32_t now)
{
    unsigned i;
    for (i=0;i<6u;i++) vision_update(now);
    assert(tx_count==6u && tx[0]==0xAAu && tx[1]==0x55u && tx[2]==1u);
    assert(tx[3]=='H' && tx[4]==crc_byte(crc_byte(0,1),'H') && tx[5]==0x0Du);
    tx_count=0;
}

int main(void)
{
    uint8_t first=0,second=0;
    int16_t offset=0;
    uint32_t received_ms=0;
    vision_init();
    reply("COLORS,3,1",0);
    assert(!vision_h_colors(&first,&second));
    assert(vision_command("vision h",0));
    assert(vision_h_state()==VISION_PICK_WAITING);
    assert(!vision_request_h(0));
    finish_tx(0);
    reply("DISC,320,240,0,1",6);
    reply("COLORS,2,2",7);
    reply("COLORS,3,5",8);
    assert(!vision_h_colors(&first,&second));
    assert(vision_h_state()==VISION_PICK_WAITING);
    reply("COLORS,3,1",9);
    reply("COLORS,4,2",10);
    assert(vision_h_colors(&first,&second) && first==3u && second==1u);
    assert(vision_h_state()==VISION_PICK_WAITING);
    reply("MOVE,-20",11);
    assert(vision_h_take_move(&offset,&received_ms) && offset==-20 && received_ms==11u);
    assert(!vision_h_take_move(&offset,&received_ms));
    reply("MOVE,20",11);
    reply("MOVEXY,-40,25",11);
    reply("MOVEXY,4,5,6",11);
    assert(!vision_h_take_move(&offset,&received_ms));
    assert(vision_h_state()==VISION_PICK_WAITING);
    assert(vision_command("vision status",11));
    assert(saw_movexy && saw_offsets);
    reply("DISC,320,240,0,1",12);
    reply("DISC,320,240,0,1,3",12);
    reply("DISC,320,240,0,3,3",12);
    reply("DISC,320,240,0,3,5",12);
    reply("DISC,320,240,0,3,1,4",12);
    reply("DISC,640,240,0,3,1",12);
    assert(vision_h_state()==VISION_PICK_WAITING);
    assert(vision_h_colors(&first,&second) && first==3u && second==1u);
    reply("DISC,320,240,0,3,1",13);
    assert(vision_h_state()==VISION_PICK_READY);
    assert(vision_h_colors(&first,&second) && first==3u && second==1u);
    reply("MOVE,20",14);
    assert(!vision_h_take_move(&offset,&received_ms));
    assert(vision_request_h(15));
    finish_tx(15);
    assert(!vision_h_colors(&first,&second));
    reply("DISC,321,239,-7,1,4",16);
    assert(vision_h_state()==VISION_PICK_READY);
    assert(vision_h_colors(&first,&second) && first==1u && second==4u);
    assert(vision_request_h(18));
    finish_tx(18);
    reply("COLORS,1,4",19);
    reply("NONE,H",20);
    assert(vision_h_state()==VISION_PICK_FAULT);
    assert(vision_h_colors(&first,&second) && first==1u && second==4u);
    assert(vision_request_h(21));
    finish_tx(21);
    vision_update(25021u);
    assert(vision_h_state()==VISION_PICK_TIMEOUT);
    vision_cancel_pick();
    assert(vision_h_state()==VISION_PICK_IDLE && !vision_h_colors(&first,&second));
    puts("PASS H transport: framing, ordered color latch, MOVE, terminal DISC, faults and timeout");
    return 0;
}
