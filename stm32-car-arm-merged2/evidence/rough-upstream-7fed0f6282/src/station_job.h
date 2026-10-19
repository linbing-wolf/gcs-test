#ifndef STATION_JOB_H
#define STATION_JOB_H
#include <stdint.h>

/* 工位作业：到站停车后按任务码自动完成抓放，做完才允许继续开。
 *
 * 任务码四组三位：G1 第一批颜色、G2 第一批靶位、G3 第二批颜色、G4 第二批靶位。
 * 颜色 1红 2黄 3蓝 4绿；靶位是 1/2/3 的一个排列。
 *
 * 原料区：圆盘每把一块物料转到固定取料口，上位机识别到目标颜色后回 PICK，
 *         才执行一次已标定的单块装车动作（arm load1）。
 * 粗加工区：车上第 i 块（盘位 i）→ G2[i]/G4[i] 号靶位。车先索引到靶位，
 *         再从载物台取料放下；三块放完后按同样顺序抓回载物台。
 * 暂存区：第一批盘位 i → G2[i]；第二批按颜色找第一批同色位置叠放。
 *
 * 车在工位内只在环位间走 150 mm（1↔3 经 2 拆两段），开始和结束都停在 2 号基准。
 * 粗加工/暂存姿态未标定时拒绝自动执行（见 station_config.h）。
 */
void station_init(void);
/* 返回 1 已启动作业，0 该节点无动作，-1 拒绝执行（已打印原因）。 */
int station_start(uint8_t node, uint8_t job_index, uint32_t now);
void station_update(uint32_t now);
int station_active(void);
int station_failed(void);
int station_task_ready(void);
uint8_t station_phase(void);
void station_abort(void);
void station_status(void);

/* task <12位任务码> 或 task <三个颜色>；颜色 1红 2黄 3蓝 4绿。 */
int station_command(const char *s);

/* 以下由 main_table_nav.c 实现：工位内短距离底盘索引。
 * delta_mm 正数朝车头，负数朝车尾；调用前机械臂必须已收臂停稳。 */
int station_chassis_start(int16_t delta_mm);
int station_chassis_busy(void);
int station_chassis_failed(void);

#endif
