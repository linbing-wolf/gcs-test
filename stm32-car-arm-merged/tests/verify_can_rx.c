#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#define __CAN_H
#define __IO volatile
#define CAN1 1
#define CAN_FIFO0 0
#define CAN_FLAG_FOV0 1
#define CAN_Id_Extended 4
#define CAN_RTR_Data 0
#define CAN_TxStatus_NoMailBox 4
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
static unsigned transmit_calls, transmit_failure;
static int CAN_GetFlagStatus(int bus,int flag) {(void)bus;(void)flag;return overrun;}
static void CAN_ClearFlag(int bus,int flag) {(void)bus;(void)flag;overrun=0;clears++;}
static unsigned CAN_MessagePending(int bus,int which) {(void)bus;(void)which;return pending;}
static void CAN_Receive(int bus,int which,CanRxMsg *frame)
{(void)bus;(void)which;if(!pending)exit(2);*frame=fifo[cursor++];pending--;}
static unsigned CAN_Transmit(int bus,CanTxMsg *frame)
{(void)bus;(void)frame;transmit_calls++;return transmit_failure?CAN_TxStatus_NoMailBox:0;}
void route_test_can_rx(uint32_t id,uint8_t ide,uint8_t rtr,uint8_t length,const uint8_t *data)
{
    if(id!=(received+1u)*256u || ide!=4 || rtr || length!=3 || data[1]!=3)exit(3);
    received++;
}
void route_test_can_error(uint8_t flags) {errors|=flags;}
#include "../vendor/stm32/can_ZDT/can.c"
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
    {
        uint8_t command[13]={1,0xfd,1,0,60,150,0,0,0,100,0,1,0x6b};
        errors=0;transmit_failure=1;can_SendCmd(command,13);
        if(errors!=2 || transmit_calls!=1)return 1;
    }
    puts("PASS CAN driver: burst FIFO drain, overrun, empty FIFO, valid interrupt release, TX mailbox failure");
    return 0;
}
