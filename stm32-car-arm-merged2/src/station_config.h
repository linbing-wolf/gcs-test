#ifndef STATION_CONFIG_H
#define STATION_CONFIG_H
/* 粗加工区 / 暂存区的取放姿态与工位内底盘索引参数。
 *
 * 方案：爪子的 yaw/slide 固定，停在“爪子朝向车侧”的姿态；三个环位由车体
 * 沿前后方向移动 150 mm 索引，让对应环号正好落在爪子正下方。这样不用为
 * 1/2/3 号环各标一组角度，只需标一组姿态 + 一个索引方向和步距。
 *
 * 没填 READY=1 之前，自动流程走到对应工位会报
 * [station fault] ... not calibrated 并原地停车，不会猜角度乱动。
 *
 * 每项含义：slide=平推轴伸出量(mm)，lift=升降高度(mm)，yaw=云台角(°)。
 * 也就是“车停在 2 号环基准时，爪子正好在 2 号环正上方能夹住物块”的那组数。
 * 标定方法：手动 arm on → arm pose → arm platform pose 后，用
 *   arm slide N / arm lift N / arm yaw D / claw angle D
 * 一点一点试到位，记下四个数，再回填到下面，把 READY 改成 1。
 */
typedef struct {
    int slide_mm;
    int lift_mm;
    float yaw_deg;
} StationPose;

/* 环位中心距：1-2-3 相邻两环相差这一段，车按它索引。
 * 第一次上车前用卷尺量环心距，不一致就改这里。 */
#define STN_INDEX_MM 150

/* ===== 粗加工区 =====
 * 车到站停在 2 号环基准。任务码 G2（第一批）/G4（第二批）给车上第 1/2/3 块
 * 的目标环号，程序按 (环号-2)*STN_INDEX_MM*SIGN 让车前后走。
 *
 * INDEX_SIGN：+1 = 3 号环在车头方向、1 号环在车尾方向；
 *             -1 = 反过来。两区朝向可能相反，所以分开配。
 * 300 mm（1↔3）会强制经 2 号基准拆成两段 150 mm。 */
#define STN_ROUGH_PLACE_READY   0
#define STN_ROUGH_INDEX_SIGN    1
static const StationPose stn_rough_place = {0, 0, 0.0f};

/* ===== 暂存区 =====
 * 第一批：盘位 i → G2[i] 号暂存位；
 * 第二批：盘位 i 的颜色 → 找第一批同色的那个暂存位，叠放在上面，
 *         高度增加 STN_STORAGE_LAYER_LIFT_MM（一层物块高度）。 */
#define STN_STORAGE_PLACE_READY 0
#define STN_STORAGE_INDEX_SIGN  1
#define STN_STORAGE_LAYER_LIFT_MM 0
static const StationPose stn_storage_place = {0, 0, 0.0f};

/* 从车上载物台把物块夹起来的姿态，沿用已标定的载物台放置参数即可；
 * 若实测发现爪子下不到杯里，只改这三项。 */
#define STN_PLATFORM_TAKE_READY 0
#define STN_PLATFORM_TAKE_SLIDE 30
#define STN_PLATFORM_TAKE_LIFT 110
#define STN_PLATFORM_TAKE_YAW  254.0f

#endif
