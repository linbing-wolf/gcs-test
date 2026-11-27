#ifndef STATION_JOB_H
#define STATION_JOB_H
#include <stdint.h>
int station_raw_set_region(unsigned pose,int xmin,int xmax,int ymin,int ymax);
void station_raw_region_status(void);

/* 工位作业：到站、朝向与四轮反馈交接完成后转观察角。
 *
 * 原料区：每批 T/TASK 确认后发送一次 H，完成 HDISC2 和底盘纠偏后按 G1/G3 串行发送三次 C。
 *         每件按当次同色 PICK 坐标选择固定 ARM_RAW_* 姿态，装杯完成才发下一次 C。
 * 粗加工区：D对准2号后固定车身，按任务环位伸臂放下、抓回，离站回正。
 * 暂存区：首次E对准2号后固定车身放料；第二次按同色环位移车并使用F视觉补移叠放。
 *
 * 粗加工/暂存姿态未标定时拒绝自动执行（见 station_config.h）。
 */
void station_init(void);
void station_set_through_enabled(int enabled);
int station_auto_enabled(void);
/* 固定车身取放的两侧环位需要完成标定才能发车。返回NULL表示通过。 */
const char *station_fixed_ring_calibration_error(void);
/* 本轮有效任务是否已接收；扫码接收窗口由底盘首次抵达B控制。 */
int station_task_ready(void);
void station_task_words(int32_t *first,int32_t *second);
/* 返回 1 已启动作业，0 该节点无动作，-1 拒绝执行（已打印原因）。 */
int station_start(uint8_t node, uint8_t job_index, uint32_t now);
/* 独立工位编排保留视觉、索引和取放，末尾只回工位基准。 */
int station_start_local(uint8_t node,uint8_t job,uint32_t now);
/* 独立D/E/F纠偏，需已声明高位收回姿态；不要求任务码或取放标定表。 */
int station_start_vision_debug(char command,uint8_t color,uint32_t now);
void station_update(uint32_t now);
int station_active(void);
/* 地面环位或码垛的松爪步骤正在等待停稳或正在打开夹爪。 */
int station_ground_release_active(void);
int station_raw_capture_active(void);
/* 原料末块在平台松爪后可并行收臂离站；其他作业保留末尾回正放行。 */
int station_departure_ready(void);
/* 粗加工最后抓回/暂存最后放下并通过实际净空后，从当前环经过2环接入下一道路；mm为车身前向回2环距离。 */
int station_through_departure(int16_t *return_mm);
/* 已通过离物净空的固定环位索引；车载取放可继续，下一地面动作等待索引完成。 */
int station_index_overlap_active(void);
int station_index_cup_slide_ready(void);
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
/* 工位内纠偏或索引到末段时，可先发后续视觉请求。 */
int station_chassis_vision_early(void);
int station_chassis_busy(void);
int station_chassis_failed(void);
int station_chassis_raw_pick_ready(void);

/* task <12位任务码> 或 task <三个颜色>；颜色 1红 2黄 3蓝 4绿。
 * G1/G3 固定定义原料夹取顺序及杯 1/2/3 颜色。 */
int station_command(const char *s);

#endif
