#ifndef STATION_JOB_H
#define STATION_JOB_H
#include <stdint.h>

/* 工位作业：到站停车后按任务码自动完成抓放，做完才允许继续开。
 *
 * 原料区：圆盘每把一块物料转到固定取料口，上位机识别到目标颜色后回 PICK，
 *         才执行一次已标定的单块装车动作（arm load1）。
 * 粗加工区：先视觉对准2号基准；前1中2后3，各目标位视觉微调后按任务抓放。
 *           平台取料高度100mm，落位/抓回70mm；作业内126.5/256度直转，离站回0。
 * 暂存区：首次126.5度E对准2号后查G2；第二次跳过E，反查G1/G2，在目标环位F定位已有物块并同色叠放。
 *
 * 粗加工/暂存姿态未标定时拒绝自动执行（见 station_config.h）。
 */
void station_init(void);
int station_auto_enabled(void);
/* 本轮有效任务是否已接收；扫码接收窗口由底盘首次抵达B控制。 */
int station_task_ready(void);
void station_task_words(int32_t *first,int32_t *second);
/* 返回 1 已启动作业，0 该节点无动作，-1 拒绝执行（已打印原因）。 */
int station_start(uint8_t node, uint8_t job_index, uint32_t now);
void station_update(uint32_t now);
int station_active(void);
int station_failed(void);
void station_abort(void);
/* 一轮含两批作业；仅整轮结束/取消时清除，普通工位交接不得调用。 */
void station_clear_task(void);
void station_status(void);
/* 粗加工/暂存使用：高位收回、云台0或粗加工作业角、四轮严格零速。
 * 支持150mm索引及上位机指定毫米距离；完成后重新取得工位互锁。 */
int station_chassis_start(int32_t delta_mm);
/* 工位横移：正值向左、负值向右；沿用同一编码器/IMU子动作及互锁。 */
int station_chassis_lateral_start(int32_t delta_mm);
int station_chassis_busy(void);
int station_chassis_failed(void);

/* task <12位任务码> 或 task <三个颜色>；颜色 1红 2黄 3蓝 4绿。 */
int station_command(const char *s);

#endif
