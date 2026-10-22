#include "can.h"
#include "can_tx.h"
#include <string.h>
extern void route_test_can_rx(uint32_t id,uint8_t ide,uint8_t rtr,uint8_t length,const uint8_t *data);
extern void route_test_can_error(uint8_t flags);

/**********************************************************
***	ZDT_X57步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/

uint8_t can_RxFlag = 0;
uint8_t test4 = 0;

__IO CAN_t can = {0};

/**
	* @brief   CAN1_RX0接收中断
	* @param   无
	* @retval  无
	*/
void CAN1_RX0_IRQHandler (void)
{
    /* FMP0 is released by CAN_Receive, not CAN_ClearITPendingBit. */
    if (CAN_GetFlagStatus(CAN1, CAN_FLAG_FOV0) != RESET) {
        route_test_can_error(1u);
        CAN_ClearFlag(CAN1, CAN_FLAG_FOV0);
    }
    while (CAN_MessagePending(CAN1, CAN_FIFO0) != 0u) {
        CAN_Receive(CAN1, CAN_FIFO0, (CanRxMsg *)(&can.CAN_RxMsg));
        can.rxFrameFlag = true;
        route_test_can_rx(can.CAN_RxMsg.ExtId, can.CAN_RxMsg.IDE,
                          can.CAN_RxMsg.RTR, can.CAN_RxMsg.DLC,
                          (const uint8_t *)can.CAN_RxMsg.Data);
    }
    if (CAN_GetFlagStatus(CAN1, CAN_FLAG_FOV0) != RESET) {
        route_test_can_error(1u);
        CAN_ClearFlag(CAN1, CAN_FLAG_FOV0);
    }
}

/**
	* @brief   CAN发送多个字节
	* @param   无
	* @retval  无
	*/
/* Foreground only. TXFP in board.c preserves submission order in the three mailboxes. */
static CanTxMsg tx_queue[CAN_TX_QUEUE_CAPACITY];
static uint16_t tx_head, tx_count;
static uint32_t tx_mailbox_id[3];
static uint8_t tx_mailbox_known[3];
static CanTxSnapshot tx_stats;

void can_tx_snapshot(CanTxSnapshot *snapshot)
{
    *snapshot=tx_stats;snapshot->queued=tx_count;
}

void can_tx_service(void)
{
    while(tx_count) {
        uint8_t mailbox=CAN_Transmit(CAN1,&tx_queue[tx_head]);
        if(mailbox==CAN_TxStatus_NoMailBox) {
            tx_stats.blocked=1;tx_stats.mailbox_waits++;
            tx_stats.esr=CAN1->ESR;tx_stats.tsr=CAN1->TSR;
            tx_stats.head_id=tx_queue[tx_head].ExtId;
            tx_stats.head_command=tx_queue[tx_head].Data[0];
            return;
        }
        tx_mailbox_id[mailbox]=tx_queue[tx_head].ExtId;tx_mailbox_known[mailbox]=1;
        tx_head=(uint16_t)((tx_head+1u)%CAN_TX_QUEUE_CAPACITY);tx_count--;
    }
    tx_stats.blocked=0;
}

void can_tx_cancel(uint8_t address)
{
    uint16_t read,keep=0;
    uint8_t mailbox;
    for(read=0;read<tx_count;read++) {
        uint16_t index=(uint16_t)((tx_head+read)%CAN_TX_QUEUE_CAPACITY);
        if(!address || (tx_queue[index].ExtId>>8)==address) {tx_stats.cancelled++;continue;}
        tx_queue[(tx_head+keep)%CAN_TX_QUEUE_CAPACITY]=tx_queue[index];keep++;
    }
    tx_count=keep;
    for(mailbox=0;mailbox<3u;mailbox++) {
        if(tx_mailbox_known[mailbox] && (!address || (tx_mailbox_id[mailbox]>>8)==address)) {
            if(CAN_TransmitStatus(CAN1,mailbox)==CAN_TxStatus_Pending) {
                CAN_CancelTransmit(CAN1,mailbox);tx_stats.cancelled++;
            }
            tx_mailbox_known[mailbox]=0;
        }
    }
    if(!tx_count)tx_stats.blocked=0;
}

void can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
    uint16_t needed,offset=0,remaining;
    uint8_t packet=0;
    can_tx_service();
    if(len<3u) {route_test_can_error(2u);return;}
    remaining=(uint16_t)(len-2u);needed=(uint16_t)((remaining+6u)/7u);
    /* Reserve all fragments before writing any: never submit half a command. */
    if(needed>CAN_TX_QUEUE_CAPACITY-tx_count) {
        tx_stats.queue_overflows++;tx_stats.esr=CAN1->ESR;tx_stats.tsr=CAN1->TSR;
        tx_stats.head_id=(uint32_t)cmd[0]<<8;tx_stats.head_command=cmd[1];
        tx_stats.rejected_id=tx_stats.head_id;tx_stats.rejected_command=cmd[1];
        tx_stats.rejected_esr=tx_stats.esr;tx_stats.rejected_tsr=tx_stats.tsr;
        route_test_can_error(2u);return;
    }
    while(remaining) {
        uint8_t size=(uint8_t)(remaining>7u ? 7u : remaining),i;
        CanTxMsg *frame=&tx_queue[(tx_head+tx_count)%CAN_TX_QUEUE_CAPACITY];
        memset(frame,0,sizeof(*frame));frame->ExtId=((uint32_t)cmd[0]<<8)|packet++;
        frame->IDE=CAN_Id_Extended;frame->RTR=CAN_RTR_Data;frame->DLC=(uint8_t)(size+1u);
        frame->Data[0]=cmd[1];
        for(i=0;i<size;i++)frame->Data[i+1u]=cmd[offset+i+2u];
        offset+=size;remaining-=size;tx_count++;
    }
    if(tx_count>tx_stats.high_water)tx_stats.high_water=tx_count;
    can_tx_service();
}

uint8_t can_GetRxFlag(void)
{
	if (can_RxFlag == 1)			//如果标志位为1
	{
		can_RxFlag = 0;
		return 1;					//则返回1，并自动清零标志位
	}
	return 0;						//如果标志位为0，则返回0
}
