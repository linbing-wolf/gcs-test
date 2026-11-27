#ifndef CAN_TX_H
#define CAN_TX_H
#include <stdint.h>

/* 底盘与5/7共用一个发送FIFO；容量以CAN帧计，多分包命令整体入队。 */
#define CAN_TX_QUEUE_CAPACITY 64u
typedef struct {
    uint32_t mailbox_waits, queue_overflows, cancelled, esr, tsr, head_id;
    uint32_t rejected_id, rejected_esr, rejected_tsr;
    uint16_t queued, high_water;
    uint8_t blocked, head_command, rejected_command;
} CanTxSnapshot;

/* 仅主循环/前台调用，不阻塞；硬件邮箱满时保留队首。 */
void can_tx_service(void);
void can_tx_snapshot(CanTxSnapshot *snapshot);
/* address=0清全部；非零仅撤销该电机积压和硬件待发帧，用于停车替代旧动作。 */
void can_tx_cancel(uint8_t address);
#endif
