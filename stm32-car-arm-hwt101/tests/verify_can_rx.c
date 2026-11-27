#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#define __CAN_H
#define __IO volatile
static struct {uint32_t ESR,TSR;} hardware;
#define CAN1 (&hardware)
#define CAN_FIFO0 0
#define CAN_FLAG_FOV0 1
#define CAN_Id_Extended 4
#define CAN_RTR_Data 0
#define CAN_TxStatus_NoMailBox 4
#define CAN_TxStatus_Pending 2
#define CAN_TxStatus_Ok 1
#define RESET 0
typedef struct {
    uint32_t StdId,ExtId;
    uint8_t Data[8],IDE,RTR,DLC;
} CanRxMsg;
typedef CanRxMsg CanTxMsg;
typedef struct {volatile CanRxMsg CAN_RxMsg;volatile CanTxMsg CAN_TxMsg;volatile bool rxFrameFlag;} CAN_t;
static CanRxMsg fifo[3];
static unsigned pending, cursor, received, errors, clears;
static uint8_t overrun;
static unsigned transmit_calls, transmit_failure,hold_mailboxes,submitted_count,aborted;
static uint8_t mailbox_busy[3];
static CanTxMsg submitted[4096];
static int CAN_GetFlagStatus(void *bus,int flag) {(void)bus;(void)flag;return overrun;}
static void CAN_ClearFlag(void *bus,int flag) {(void)bus;(void)flag;overrun=0;clears++;}
static unsigned CAN_MessagePending(void *bus,int which) {(void)bus;(void)which;return pending;}
static void CAN_Receive(void *bus,int which,CanRxMsg *frame)
{(void)bus;(void)which;if(!pending)exit(2);*frame=fifo[cursor++];pending--;}
static unsigned CAN_Transmit(void *bus,CanTxMsg *frame)
{
    unsigned i;(void)bus;transmit_calls++;
    if(transmit_failure)return CAN_TxStatus_NoMailBox;
    for(i=0;i<3;i++)if(!mailbox_busy[i]) {
        if(submitted_count>=4096)exit(5);
        submitted[submitted_count++]=*frame;mailbox_busy[i]=(uint8_t)hold_mailboxes;return i;
    }
    return CAN_TxStatus_NoMailBox;
}
static unsigned CAN_TransmitStatus(void *bus,uint8_t mailbox)
{(void)bus;return mailbox_busy[mailbox] ? CAN_TxStatus_Pending : CAN_TxStatus_Ok;}
static void CAN_CancelTransmit(void *bus,uint8_t mailbox)
{(void)bus;mailbox_busy[mailbox]=0;aborted++;}
void route_test_can_rx(uint32_t id,uint8_t ide,uint8_t rtr,uint8_t length,const uint8_t *data)
{
    if(id!=(received+1u)*256u || ide!=4 || rtr || length!=3 || data[1]!=3)exit(3);
    received++;
}
void route_test_can_error(uint8_t flags) {errors|=flags;}
#include "../vendor/stm32/can_ZDT/can.c"
#define CHECK(x) do{if(!(x)){fprintf(stderr,"FAIL CAN TX line%d: %s\n",__LINE__,#x);exit(1);}}while(0)
static void reset_tx(void)
{
    memset(tx_queue,0,sizeof(tx_queue));tx_head=tx_count=0;memset(&tx_stats,0,sizeof(tx_stats));
    memset(tx_mailbox_known,0,sizeof(tx_mailbox_known));memset(mailbox_busy,0,sizeof(mailbox_busy));
    transmit_calls=transmit_failure=hold_mailboxes=submitted_count=aborted=errors=0;
    hardware.ESR=hardware.TSR=0;
}
static void frame_check(unsigned index,uint8_t addr,uint8_t packet,uint8_t function,const uint8_t *data,uint8_t size)
{
    CanTxMsg *f=&submitted[index];
    CHECK(f->ExtId==((uint32_t)addr<<8|packet) && f->IDE==4 && !f->RTR && f->DLC==size+1);
    CHECK(f->Data[0]==function && !memcmp(f->Data+1,data,size));
}
static void queue_tests(void)
{
    uint8_t position[13]={7,0xfd,1,0,60,200,0,0,0,100,0,0,0x6b};
    uint8_t query[3]={5,0x3a,0x6b};
    CanTxSnapshot s;unsigned i;
    reset_tx();transmit_failure=1;hardware.ESR=0x1080;hardware.TSR=7;
    can_SendCmd(position,sizeof(position));can_SendCmd(query,sizeof(query));
    can_tx_snapshot(&s);CHECK(!errors && s.queued==3 && s.blocked && !submitted_count);
    CHECK(s.head_id==0x700 && s.head_command==0xfd && s.esr==0x1080 && s.tsr==7);
    /* command arrays are stack-owned by callers and must be copied before returning. */
    memset(position+2,99,11);can_tx_service();CHECK(!errors && tx_count==3);
    transmit_failure=0;can_tx_service();can_tx_snapshot(&s);
    CHECK(!s.queued && !s.blocked && submitted_count==3 && !errors);
    {uint8_t a[]={1,0,60,200,0,0,0},b[]={100,0,0,0x6b};
     frame_check(0,7,0,0xfd,a,7);frame_check(1,7,1,0xfd,b,4);frame_check(2,5,0,0x3a,query+2,1);}
    reset_tx();hold_mailboxes=1;
    for(i=0;i<9;i++){query[0]=(uint8_t)(7-i%7);can_SendCmd(query,3);}
    CHECK(submitted_count==3 && tx_count==6 && !errors);
    mailbox_busy[1]=0;can_tx_service();CHECK(submitted_count==4 && submitted[3].ExtId==(4u<<8) && tx_count==5);
    memset(mailbox_busy,0,3);hold_mailboxes=0;can_tx_service();CHECK(submitted_count==9 && !tx_count && !errors);
    for(i=0;i<9;i++)CHECK(submitted[i].ExtId==((uint32_t)(7-i%7)<<8));
    /* Atomic reservation: one free queue slot cannot accept a two-fragment position. */
    reset_tx();transmit_failure=1;
    for(i=0;i<CAN_TX_QUEUE_CAPACITY-1;i++)can_SendCmd(query,3);
    can_SendCmd(position,sizeof(position));can_tx_snapshot(&s);
    CHECK(errors==2 && s.queued==CAN_TX_QUEUE_CAPACITY-1 && s.queue_overflows==1 && !submitted_count);
    CHECK(s.head_id==0x700 && s.head_command==0xfd);
    can_tx_cancel(0);can_SendCmd(query,3);can_tx_snapshot(&s);
    CHECK(s.rejected_id==0x700 && s.rejected_command==0xfd); /* later stop/service cannot erase rejection */
    /* Cancel all stale work, including occupied hardware mailboxes, before stop frames. */
    reset_tx();hold_mailboxes=1;
    for(i=0;i<9;i++){query[0]=(uint8_t)(i%3+1);can_SendCmd(query,3);}
    can_tx_cancel(0);CHECK(!tx_count && aborted==3);
    {uint8_t stop[]={5,0xfe,0x98,0,0x6b};can_SendCmd(stop,5);CHECK(submitted_count==4 && submitted[3].Data[0]==0xfe);}
    /* Per-motor stop must not erase other motor/arm stop commands. */
    reset_tx();hold_mailboxes=1;
    {uint8_t addresses[]={7,5,1,7,5,1,7};
     for(i=0;i<sizeof(addresses);i++){query[0]=addresses[i];can_SendCmd(query,3);}}
    can_tx_cancel(7);CHECK(tx_count==2 && aborted==1);
    CHECK(tx_queue[tx_head].ExtId==0x500 && tx_queue[(tx_head+1)%CAN_TX_QUEUE_CAPACITY].ExtId==0x100);
    can_tx_cancel(5);CHECK(tx_count==1 && aborted==2 && mailbox_busy[2]);
    /* Multiple ring wraparounds with temporary mailboxes held; exact FIFO and copy integrity. */
    reset_tx();
    for(i=0;i<300;i++) {
        unsigned start=submitted_count,j;transmit_failure=1;
        for(j=0;j<5;j++){query[0]=(uint8_t)(1+j);can_SendCmd(query,3);}
        CHECK(tx_count==5 && !errors);transmit_failure=0;can_tx_service();
        for(j=0;j<5;j++)CHECK(submitted[start+j].ExtId==((j+1)<<8));
    }
    CHECK(!errors && !tx_count && submitted_count==1500);
    puts("PASS CAN TX queue: mailbox saturation retains copied complete commands, exact split/FIFO, all3 hardware mailboxes and independent slot release, recovery without new command, atomic software-full rejection, cancellation of stale software/hardware work, per-address stop isolation,1500 frames across ring wraps, ESR/TSR/head diagnostics");
}
int main(void)
{
    unsigned i;
    for(i=0;i<3;i++) {
        fifo[i].ExtId=(i+1u)*256u;fifo[i].IDE=4;fifo[i].DLC=3;
        fifo[i].Data[0]=0x3a;fifo[i].Data[1]=3;fifo[i].Data[2]=0x6b;
    }
    pending=3;overrun=1;CAN1_RX0_IRQHandler();
    if(pending || received!=3 || errors!=1 || clears!=1 || !can.rxFrameFlag)return 1;
    CAN1_RX0_IRQHandler();if(received!=3)return 1;
    queue_tests();
    puts("PASS CAN driver: burst RX FIFO drain, overrun, empty FIFO, valid interrupt release; temporary TX mailbox full is backpressure without fault");
    return 0;
}
