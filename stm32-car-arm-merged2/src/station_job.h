#ifndef STATION_JOB_H
#define STATION_JOB_H
#include <stdint.h>

/* 工位作业：到站停车后按任务码自动完成抓放，做完才允许继续开。
 *
 * 原料区：圆盘每把一块物料转到固定取料口，上位机识别到目标颜色后回 PICK，
 *         才执行一次已标定的单块装车动作（arm load1）。
 * 粗加工区：以2号环为基准、底盘逐环索引；按任务码放3块，再按同序抓回。
 * 暂存区：从车载载物台取 3 块放到暂存位。
 *
 * 粗加工/暂存姿态未标定时拒绝自动执行（见 station_config.h）。
 */
void station_init(void);
int station_auto_enabled(void);
/* 返回 1 已启动作业，0 该节点无动作，-1 拒绝执行（已打印原因）。 */
int station_start(uint8_t node, uint8_t job_index, uint32_t now);
void station_update(uint32_t now);
int station_active(void);
int station_failed(void);
void station_abort(void);
/* 一轮含两批作业；仅整轮结束/取消时清除，普通工位交接不得调用。 */
void station_clear_task(void);
void station_status(void);
/* 仅粗加工工位使用；调用前已收臂且四轮停稳。每次一环间距，完成后重取互锁。 */
int station_chassis_start(int16_t delta_mm);
int station_chassis_busy(void);
int station_chassis_failed(void);

/* task <12位任务码> 或 task <三个颜色>；颜色 1红 2黄 3蓝 4绿。 */
int station_command(const char *s);

#endif
