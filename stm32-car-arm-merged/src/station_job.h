#ifndef STATION_JOB_H
#define STATION_JOB_H
#include <stdint.h>

/* 工位作业：进站朝向匹配后提前转观察角；离物后固定索引与车载搬运并行。
 *
 * 原料区：圆盘每把一块物料转到固定取料口，上位机识别到目标颜色后回 PICK，
 *         才执行一次已标定的单块装车动作（arm load1）。
 * 粗加工区：先视觉对准2号基准一次；前1中2后3，随后按固定距离索引抓放，结束回2号。
 *           平台取料高度100mm，落位/抓回高度见station_config.h；作业内126.5/256度直转，离站回0。
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
/* 独立D/E/F纠偏，需已声明高位收回姿态；不要求任务码或取放标定表。 */
int station_start_vision_debug(char command,uint8_t color,uint32_t now);
void station_update(uint32_t now);
int station_active(void);
/* 最后仅剩高位收回后的云台/载物台回正，可提前开始离站预检和转弯。 */
int station_departure_ready(void);
/* 粗加工最后抓回/暂存最后放下并通过实际净空后，从当前环经过2环接入下一道路；mm为车身前向回2环距离。 */
int station_through_departure(int16_t *return_mm);
/* 已通过离物净空的固定环位索引；车载取放可继续，下一地面动作等待索引完成。 */
int station_index_overlap_active(void);
int station_failed(void);
void station_abort(void);
/* 一轮含两批作业；仅整轮结束/取消时清除，普通工位交接不得调用。 */
void station_clear_task(void);
void station_status(void);
/* 固定索引可由已通过离物净空的工位流水线调用；其他索引/视觉位移仍要求高位收回。
 * 最后低速几毫米可与下一靠近并行，完成后维持机构会话或重新取得工位互锁。 */
int station_chassis_start(int32_t delta_mm);
/* 工位横移：正值向左、负值向右；沿用同一编码器/IMU子动作及互锁。 */
int station_chassis_lateral_start(int32_t delta_mm);
/* 两轴同时纠偏：正forward向前、正left向左；任一轴0时沿用单轴执行器。 */
int station_chassis_xy_start(int32_t forward_mm,int32_t left_mm);
/* 视觉/固定索引末段：满足剩余距离、速度和新鲜反馈时登记靠近并行。 */
int station_chassis_approach_begin(void);
/* 进站平移或最后朝向修正期间：只允许准备姿态，不能请求视觉或抓取。 */
int station_chassis_entry_busy(void);
int station_chassis_busy(void);
int station_chassis_failed(void);

/* task <12位任务码> 或 task <三个颜色>；颜色 1红 2黄 3蓝 4绿。 */
int station_command(const char *s);

#endif
