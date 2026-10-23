#ifndef ARM_CONFIG_H
#define ARM_CONFIG_H
/* 机械机构参数入口；分类说明见docs/参数修改索引.md。坐标均为绝对目标，改后重编译下载。
 * 原始参数来源：gcs-test/test-code/stm32/main_armclaw.c，96f65d8；后续修改见工程交接记录。 */
#define ARM_DEBUG_UART_BAUD       115200u /* 调试串口波特率，bit/s；需与串口终端一致。 */
#define ARM_LIFT_ADDRESS               5u /* 升降轴CAN地址；必须与驱动器设置一致。 */
#define ARM_SLIDE_ADDRESS              7u /* 平推轴CAN地址；必须与驱动器设置一致。 */
#define ARM_LIFT_PULSES_PER_MM        80.0f /* 升降位移标定，脉冲/mm；增大则同一毫米指令发出更多脉冲。 */
#define ARM_SLIDE_PULSES_PER_MM       25.465f /* 平推位移标定，脉冲/mm；增大则同一毫米指令发出更多脉冲。 */
#define ARM_LIFT_MIN_MM                0 /* 升降最低绝对坐标，mm；不是一次下降量。 */
#define ARM_LIFT_MAX_MM              135 /* 升降允许最高绝对坐标，mm；必须对应实际行程。 */
#define ARM_LIFT_HOME_MM             135 /* 行驶/收拢目标高度，mm；135最高、0最低，上电须声明真实位置。 */
#ifndef ARM_CLEARANCE_MM
#define ARM_CLEARANCE_MM             125 /* 通用离杯净空高度，mm；保留操作者当前125；车载出杯起转另用ARM_PLATFORM_TURN_START_LIFT_MM。 */
#endif
#ifndef ARM_LIFT_PARALLEL_ENABLED
#define ARM_LIFT_PARALLEL_ENABLED      1 /* 1=升降与平推/旋转联动；直接搬运由ARM_DIRECT_TRANSFER_ENABLED选择；0=原顺序。 */
#endif
#ifndef ARM_STATION_PRESELECT_ENABLED
#define ARM_STATION_PRESELECT_ENABLED  1 /* 1=粗加工/暂存离杯到净空后提前选下一杯；0=下一次取放时选杯。 */
#endif
#ifndef ARM_DIRECT_TRANSFER_ENABLED
#define ARM_DIRECT_TRANSFER_ENABLED   1 /* 1=抓稳离杯后直转下一姿态，升降/平推同时到最终值；需升降联动=1；0=原收回路径。 */
#endif
#ifndef ARM_PLATFORM_TURN_START_LIFT_MM
#define ARM_PLATFORM_TURN_START_LIFT_MM 110 /* 原料/粗加工抓回/车载杯夹紧后实际升到此高度才开始云台/平推，mm；继续升至135。 */
#endif
#ifndef ARM_CUP_EXIT_DESCENT_DELAY_MS
#define ARM_CUP_EXIT_DESCENT_DELAY_MS 300u /* 各区夹紧后实际升到最高位再延迟下降/换杯，ms；当前300，0=到顶即下降，不等转向结束。 */
#endif
#if ARM_PLATFORM_TURN_START_LIFT_MM < ARM_LIFT_MIN_MM || ARM_PLATFORM_TURN_START_LIFT_MM > ARM_LIFT_HOME_MM
#error Invalid_platform_turn_start_height
#endif
#if ARM_CLEARANCE_MM < ARM_LIFT_MIN_MM || ARM_CLEARANCE_MM > ARM_LIFT_HOME_MM
#error Invalid_arm_clearance_height
#endif
#define ARM_SLIDE_MIN_MM               0 /* 齿条完全收回的人工坐标原点。 */
#define ARM_SLIDE_MAX_MM             187 /* 队友程序的理论行程上限，尚未实测全行程。 */
#define ARM_SLIDE_HOME_MM              0 /* 收拢平推绝对坐标，mm；0=完全收回。 */
#define ARM_LIFT_DIR_INVERT            1 /* 沿用队友调试结果：反转位置命令的方向位。 */
#define ARM_SLIDE_DIR_INVERT           1 /* 平推方向位反转开关，0/1；只随电机安装方向标定修改。 */
#define ARM_LIFT_RPM                 4000u /* 升降位置模式目标转速，RPM；增大更快，实际速度受负载/驱动限制。 */
#define ARM_SLIDE_RPM                4000u /* 平推位置模式目标转速，RPM；增大更快，实际到位仍看反馈。 */
#define ARM_AXIS_ACCEL               255u /* 升降/平推驱动器加速度档位，0~255；非RPM/s，不改底盘档位200。 */
#define ARM_ENCODER_COUNTS_PER_REV 65536u /* 编码器计数/圈；须匹配驱动器CPOS协议，不能用来调速度。 */
#define ARM_PULSES_PER_REV          3200u /* 位置指令脉冲/圈；对应16细分，须与驱动器设置一致。 */
#define ARM_POSITION_TOLERANCE_MM      0.5f /* 到位误差窗口，不能把1mm指令的零位移算完成。 */
#define ARM_LIFT_ENCODER_SIGN          1  /* dir=0时CPOS增加为+1，相反为-1。 */
#define ARM_SLIDE_ENCODER_SIGN         1 /* dir=0时CPOS增加为+1，相反为-1；仅影响反馈坐标换算。 */
#define ARM_YAW_MIN_DEG            (-94.5f) /* 云台可接受的最小逻辑角，度；不是PWM最小值。 */
#define ARM_YAW_MAX_DEG              265.5f /* 云台可接受的最大逻辑角，度；与零点/方向/PWM映射配合。 */
#define ARM_YAW_ZERO_DEG             273.4f /* 云台PWM映射零点，度；硬件标定，工作取放角不改此项。 */
#define ARM_YAW_DIRECTION             (-1.0f) /* 云台逻辑角到PWM的方向，+1/-1；硬件标定。 */
#define ARM_YAW_HOME_DEG                0.0f /* 1975us，远程当前标定的初始角。 */
#define ARM_YAW_OFFSET_DEG             7.9f /* 云台PWM映射补偿角，度；修改会整体改变云台落点。 */
#define ARM_CLAW_MIN_DEG               0.0f /* 夹爪可接受最小逻辑角，度；与PWM映射配合。 */
#define ARM_CLAW_MAX_DEG             270.0f /* 夹爪可接受最大逻辑角，度；操作者确认为270度款。 */
#define ARM_CLAW_OFFSET_DEG            1.0f /* 夹爪PWM映射补偿角，度；会整体改变夹爪开合位置。 */
#define ARM_CLAW_OPEN_DEG             45 /* 张爪目标角，度；当前45，增减影响开口，不是转速。 */
#define ARM_CLAW_CLOSE_DEG            15 /* 合爪目标角，度；当前15，须按物块夹持情况标定。 */
#define ARM_PICK_SLIDE_MM             55 /* 原料区抓取平推目标，mm；绝对伸出坐标。 */
#define ARM_PICK_LIFT_MM             105 /* 原料区抓取升降目标，mm；绝对高度，增大更高。 */
#define ARM_PICK_YAW_DEG             131.0f /* 约1247us，操作者提供的原料抓取角。 */
#define ARM_PLACE_SLIDE_MM            35 /* 车载杯位取/放共用平推目标，mm；绝对伸出坐标。 */
#define ARM_PLACE_LIFT_MM            110 /* 放回车载杯位时松爪高度，mm；取出高度100另见station_config.h。 */
#define ARM_PLACE_YAW_DEG            256.0f /* 约553us，操作者提供的车上放置角。 */
#define ARM_PLATFORM_FIRST_DEG        20.0f /* PA2，沿用随包PWM.c三个档位。 */
#define ARM_PLATFORM_SECOND_DEG      139.0f /* 载物台2号杯对准夹爪的目标角，度；不是环位2。 */
#define ARM_PLATFORM_THIRD_DEG       256.0f /* 载物台3号杯对准夹爪的目标角，度；不是环位3。 */
#define ARM_CHASSIS_STOP_HOLD_MS     200u /* 机构会话获取底盘状态的保持时间，ms；非每个抓放步骤的延时。 */
#define ARM_CHASSIS_LOCK_TIMEOUT_MS 2000u /* 底盘互锁等待诊断时限，ms；具体等待/日志行为见主控。 */
#define ARM_SERVO_TIME_MS           300u /* 载物台插值时长，ms；减小会更快发完目标，舵机无到位反馈。 */
#define ARM_YAW_TIME_MS             150u /* 云台插值时长，ms；减小加快指令变化，与末端保持40ms独立。 */
#define ARM_YAW_SETTLE_MS            40u /* 最终云台PWM写入后的保持时间，ms；减小更快衔接，稳定性须实测。 */
#define ARM_CLAW_TIME_MS             50u /* 夹爪开合插值时长，ms；与合爪后的抓稳保持120ms独立。 */
#define ARM_CLAW_CLOSE_HOLD_MS      120u /* 最终合爪PWM写入后抓稳保持，ms；减小可能未抓稳就上升，张爪不加此等待。 */
#define ARM_YAW_TEST_TIME_MS        8000u /* 全行程诊断单次插值时间，不改变夹取动作速度。 */
#define ARM_POLL_MS                   10u /* 机构反馈查询间隔，ms；减小增加CAN流量，净空阶段穿插升降位置查询。 */
#define ARM_FEEDBACK_TIMEOUT_MS      500u /* 机构反馈新鲜度/查询等待窗口，ms；不是动作间隔。 */
#define ARM_ACK_TIMEOUT_MS          1500u /* 位置命令ACK告警等待，ms；超时日志不替代实际位置/到位。 */
#define ARM_ACTION_TIMEOUT_MS      15000u /* 动作等待告警周期，ms；到时记录并继续等实际反馈。 */
#endif
