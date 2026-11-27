#ifndef HMI_STATUS_H
#define HMI_STATUS_H
#include <stdint.h>

/* robot-status.tft只读协议；显示故障/掉线不得改变控制状态。 */
#define HMI_UNKNOWN (-32768)
#define HMI_REFRESH_MS 500u
#define HMI_RESYNC_MS 5000u
#define HMI_BOOT_MS 1000u
/* 旧代码只定义地址和运动符号，未标注物理轮位；0表示尚未确认。 */
#ifndef HMI_LF_ADDRESS
#define HMI_LF_ADDRESS 0u
#endif
#ifndef HMI_RF_ADDRESS
#define HMI_RF_ADDRESS 0u
#endif
#ifndef HMI_LR_ADDRESS
#define HMI_LR_ADDRESS 0u
#endif
#ifndef HMI_RR_ADDRESS
#define HMI_RR_ADDRESS 0u
#endif
enum {
    HMI_TASKA,HMI_TASKB,HMI_TASKOK,HMI_PHASE,HMI_ELAPSED,HMI_PICK,HMI_PLACE,
    HMI_LF_T,HMI_LF_R,HMI_RF_T,HMI_RF_R,HMI_LR_T,HMI_LR_R,HMI_RR_T,HMI_RR_R,
    HMI_LIFT,HMI_PUSH,HMI_GRIP,HMI_TRAY,HMI_PAN,HMI_MECHANISM,
    HMI_YAW,HMI_YAW_T,HMI_COLOR,HMI_DX,HMI_DY,HMI_VISION_AGE,HMI_IMU_AGE,
    HMI_VOLTAGE,HMI_COMM,HMI_FIELDS
};
typedef struct {int32_t value[HMI_FIELDS];} HmiSnapshot;
void hmi_init(uint32_t now);
void hmi_update(const HmiSnapshot *snapshot,uint32_t now);
/* 仅更改显示页面；不转发屏幕返回数据为车辆命令。 */
void hmi_select(uint8_t page,uint8_t group);
void hmi_resync(void);
int hmi_busy(void);
/* USART6 TXE中断消费；电脑测试用同一接口验证逐字节和堵塞恢复。 */
int hmi_tx_next(uint8_t *byte);
#endif
