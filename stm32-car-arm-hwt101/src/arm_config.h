#ifndef ARM_CONFIG_H
#define ARM_CONFIG_H

/* 比赛机械机构参数入口。原始参考为 gcs-test/test-code/stm32/main_armclaw.c（96f65d8），
 * 以下为现用配置。改动后重新构建并下载。#ifndef 包住的默认值可由已有构建选项覆盖。
 */

/* 1. 原料区三点抓取
 * 升降、平推均为绝对坐标（mm），云台为逻辑角（度）。PAST/CURRENT/NEXT 分别对应已过去/面前/尚未到的姿态，按云台 133° 观察时对应右/中/左；J 像素分区见 station_config.h。两侧默认值须现场核对净空与落点。
 */
#define ARM_PICK_SLIDE_MM        7 /* 中间抓点的基础平推坐标，mm；增大向外伸。 */
#define ARM_PICK_LIFT_MM         90 /* 原料三点默认共用的抓料高度，mm；增大向上。 */
#define ARM_PICK_YAW_DEG         133.0f /* 车右侧原料观察和中间抓点的基础云台角，度。 */
#ifndef ARM_RAW_PAST_SLIDE_MM
#define ARM_RAW_PAST_SLIDE_MM    165 /* 右侧抓点绝对平推，mm；增大向外伸。 */
#endif
#ifndef ARM_RAW_PAST_LIFT_MM
#define ARM_RAW_PAST_LIFT_MM     ARM_PICK_LIFT_MM /* 右侧抓点绝对高度，mm；默认随 ARM_PICK_LIFT_MM，现场可单独标定。 */
#endif
#ifndef ARM_RAW_PAST_YAW_DEG
#define ARM_RAW_PAST_YAW_DEG     150.0f /* 右侧抓点云台目标角，度。 */
#endif
#ifndef ARM_RAW_CURRENT_SLIDE_MM
#define ARM_RAW_CURRENT_SLIDE_MM ARM_PICK_SLIDE_MM /* 中间抓点绝对平推，mm；默认随 ARM_PICK_SLIDE_MM。 */
#endif
#ifndef ARM_RAW_CURRENT_LIFT_MM
#define ARM_RAW_CURRENT_LIFT_MM  ARM_PICK_LIFT_MM /* 中间抓点绝对高度，mm；默认随 ARM_PICK_LIFT_MM。 */
#endif
#ifndef ARM_RAW_CURRENT_YAW_DEG
#define ARM_RAW_CURRENT_YAW_DEG  ARM_PICK_YAW_DEG /* 原料观察与中间抓点的云台角，度；默认随 ARM_PICK_YAW_DEG。 */
#endif
#ifndef ARM_RAW_NEXT_SLIDE_MM
#define ARM_RAW_NEXT_SLIDE_MM    165 /* 左侧抓点绝对平推，mm；增大向外伸。 */
#endif
#ifndef ARM_RAW_NEXT_LIFT_MM
#define ARM_RAW_NEXT_LIFT_MM     ARM_PICK_LIFT_MM /* 左侧抓点绝对高度，mm；默认随 ARM_PICK_LIFT_MM，现场可单独标定。 */
#endif
#ifndef ARM_RAW_NEXT_YAW_DEG
#define ARM_RAW_NEXT_YAW_DEG     117.0f /* 左侧抓点云台目标角，度。 */
#endif

/* 2. 车载杯、夹爪与收拢姿态 */
#define ARM_PLACE_SLIDE_MM      35 /* 车载杯取出与放回共用的平推坐标，mm；增大向外伸。 */
#define ARM_PLACE_LIFT_MM       110 /* 放回车载杯时的松爪高度，mm；粗加工和暂存的取出高度见 STN_ROUGH_PLATFORM_TAKE_LIFT_MM。 */
#define ARM_PLACE_YAW_DEG       263.0f /* 车载杯取出与放回共用的云台目标角，度。 */
#define ARM_PLATFORM_FIRST_DEG  20.0f /* 载物台 1 号杯对准夹爪的角度，度；冷启动 PC9 声明也用此值。 */
#define ARM_PLATFORM_SECOND_DEG 139.0f /* 载物台 2 号杯对准夹爪的角度，度；对应车载杯编号。 */
#define ARM_PLATFORM_THIRD_DEG  256.0f /* 载物台 3 号杯对准夹爪的角度，度；对应车载杯编号。 */
#define ARM_CLAW_OPEN_DEG       40 /* 张爪目标角，度；改变开口位置，也用于冷启动 PC9 姿态声明。 */
#define ARM_CLAW_CLOSE_DEG      0 /* 合爪目标角，度；按实际夹持深度和压力标定。 */
#define ARM_LIFT_HOME_MM        135 /* 行驶/收拢的升降目标高度，mm；也用于冷启动 PC9 姿态声明。 */
#define ARM_SLIDE_HOME_MM       0 /* 行驶/收拢和 PC9 初始化的平推目标，mm；0 为完全收回。 */
#ifndef ARM_PC9_START_SLIDE_MM
#define ARM_PC9_START_SLIDE_MM  113 /* 冷启动按 PC9 前人工摆放的实际平推坐标，mm；首段起步时从此位置收回。 */
#endif
#define ARM_YAW_HOME_DEG        0.0f /* 行驶/收拢的云台目标角，度；也用于冷启动 PC9 姿态声明。 */

/* 3. 速度、舵机插值与保持
 * 舵机时间控制 PWM 指令和动作衔接。确认实际到位、抓稳和净空需要观察机构。
 */
#define ARM_LIFT_RPM              4000u /* 升降位置模式目标转速，RPM；提高后须确认负载下的跟随。 */
#define ARM_SLIDE_RPM             4000u /* 平推位置模式目标转速，RPM；提高后须确认负载下的跟随。 */
#define ARM_AXIS_ACCEL            245u /* 升降与平推共用的驱动器加速度档位，0～255；软件斜坡的 RPM/s 是底盘另一类参数。 */
#define ARM_POSITION_TOLERANCE_MM 0.5f /* 位置反馈到位误差窗口，mm；增大会放宽到位要求。 */
#define ARM_SERVO_TIME_MS         40u /* 载物台插值时长，ms；减小会更快发完目标 PWM。 */
#define ARM_YAW_TIME_MS           100u /* 云台插值时长，ms；减小会让云台指令变化更快。 */
#define ARM_YAW_SETTLE_MS         40u /* 需要保持的云台动作在最终 PWM 写入后的等待，ms；部分动作按现有衔接规则跳过。 */
#define ARM_CLAW_TIME_MS          30u /* 夹爪开合插值时长，ms；减小会更快发完目标 PWM。 */
#define ARM_CLAW_CLOSE_HOLD_MS    50u /* 最终合爪 PWM 写入后给物块抓稳的保持时间，ms；张爪不加此等待。 */
#define ARM_YAW_TEST_TIME_MS      8000u /* arm yawtest 的全行程诊断插值时长，ms；正常作业使用 ARM_YAW_TIME_MS。 */

/* 4. 净空、搬运与底盘交接
 * 并行阈值以升降实际反馈判断。降低净空或提前下降后，须重查夹爪、物块、杯壁与车架的间隙。
 */
#ifndef ARM_CLEARANCE_MM
#define ARM_CLEARANCE_MM                125 /* 通用离杯净空高度，mm；增大会推迟基于此阈值的收回、选杯或索引交接。 */
#endif
#ifndef ARM_PLATFORM_TURN_START_LIFT_MM
#define ARM_PLATFORM_TURN_START_LIFT_MM 110 /* 带物转移开始时的实际升降门槛，mm；车载杯到地面仅放行云台，平推等放料角。 */
#endif
#ifndef ARM_CUP_EXIT_DESCENT_DELAY_MS
#define ARM_CUP_EXIT_DESCENT_DELAY_MS   100u /* 夹紧后实际到收拢高度再允许下降/换杯的延迟，ms；0 表示到位即可衔接。 */
#endif
#ifndef ARM_LIFT_PARALLEL_ENABLED
#define ARM_LIFT_PARALLEL_ENABLED       1 /* 1 允许升降与平推/旋转按步骤并行，0 使用顺序流程。 */
#endif
#ifndef ARM_DIRECT_TRANSFER_ENABLED
#define ARM_DIRECT_TRANSFER_ENABLED     1 /* 1 升降联动开启时允许升至净空即转；0 工位出杯先升到收拢高度，向杯内放料用串行路径。 */
#endif
#ifndef ARM_STATION_PRESELECT_ENABLED
#define ARM_STATION_PRESELECT_ENABLED   1 /* 1 允许粗加工/暂存从车载杯取料后，到放料角且满足离杯净空再选下一杯，0 随下一次取放选杯。 */
#endif
#define ARM_CHASSIS_STOP_HOLD_MS        0u /* 机构接管时底盘额外稳定保持时间，ms；0 仍检查四轮新反馈。 */
#define ARM_CHASSIS_LOCK_TIMEOUT_MS     2000u /* 底盘互锁等待告警阈值，ms；到时输出诊断，接管仍须满足反馈条件。 */

/* 5. 轴、舵机与电气标定 */
#define ARM_LIFT_MIN_MM            -20 /* 升降最低软件坐标，mm；负值允许低于原 0 点，须确认实物下方净空。 */
#define ARM_LIFT_MAX_MM            135 /* 升降最高软件坐标，mm；按实物行程核对，收拢高度须可达。 */
#define ARM_SLIDE_MIN_MM           0 /* 平推最小软件坐标，mm；以完全收回为坐标零点。 */
#define ARM_SLIDE_MAX_MM           187 /* 平推最大软件坐标，mm；默认值基于理论行程，须测实物可用范围。 */
#define ARM_LIFT_PULSES_PER_MM     80.0f /* 升降距离标定，脉冲/mm；同一距离指令下，调大会增加命令脉冲。 */
#if ARM_PC9_START_SLIDE_MM < ARM_SLIDE_HOME_MM || ARM_PC9_START_SLIDE_MM > ARM_SLIDE_MAX_MM
#error ARM_PC9_START_SLIDE_MM must be between slide home and maximum travel
#endif
#define ARM_SLIDE_PULSES_PER_MM    25.465f /* 平推距离标定，脉冲/mm；同一距离指令下，调大会增加命令脉冲。 */
#define ARM_LIFT_DIR_INVERT        1 /* 升降位置命令方向位，1 反转/0 不反转；以升为正复测。 */
#define ARM_SLIDE_DIR_INVERT       1 /* 平推位置命令方向位，1 反转/0 不反转；以伸出为正复测。 */
#define ARM_LIFT_ENCODER_SIGN      1 /* 升降 CPOS 极性，方向位为 0 时计数递增填 +1，递减填 -1。 */
#define ARM_SLIDE_ENCODER_SIGN     1 /* 平推 CPOS 极性，方向位为 0 时计数递增填 +1，递减填 -1。 */
#define ARM_ENCODER_COUNTS_PER_REV 65536u /* 轴编码器反馈计数/圈；须匹配驱动器 CPOS 协议。 */
#define ARM_PULSES_PER_REV         3200u /* 位置指令脉冲/圈；当前对应驱动器 16 细分。 */
#define ARM_YAW_MIN_DEG            (-94.5f) /* 云台最小逻辑角，度；须配合 PWM 映射和实物行程复核。 */
#define ARM_YAW_MAX_DEG            265.5f /* 云台最大逻辑角，度；须配合 PWM 映射和实物行程复核。 */
#define ARM_YAW_ZERO_DEG           273.4f /* 云台逻辑角到 PWM 的零点项，度；调整会改变整组云台落点。 */
#define ARM_YAW_DIRECTION          (-1.0f) /* 云台逻辑角到 PWM 的方向，+1 或 -1；实测映射正反方向后填写。 */
#define ARM_YAW_OFFSET_DEG         7.9f /* 云台逻辑角到 PWM 的补偿项，度；调整后复测所有工作角。 */
#define ARM_CLAW_MIN_DEG           0.0f /* 夹爪最小逻辑角，度；须与舵机和连杆行程相符。 */
#define ARM_CLAW_MAX_DEG           270.0f /* 夹爪最大逻辑角，度；须与舵机和连杆行程相符。 */
#define ARM_CLAW_OFFSET_DEG        1.0f /* 夹爪 PWM 映射补偿角，度；同时改变张爪、合爪的实际位置。 */
#define ARM_LIFT_ADDRESS           5u /* 升降轴 CAN 地址，须与驱动器地址一致，并与其他轴区分。 */
#define ARM_SLIDE_ADDRESS          7u /* 平推轴 CAN 地址，须与驱动器地址一致，并与其他轴区分。 */

/* 6. 反馈周期与告警 */
#define ARM_POLL_MS             10u /* 机构反馈查询间隔，ms；减小会增加 CAN 流量。 */
#define ARM_FEEDBACK_TIMEOUT_MS 500u /* 机构反馈查询等待和样本新鲜度窗口，ms。 */
#define ARM_ACK_TIMEOUT_MS      1500u /* 位置命令缺 ACK 的告警阈值，ms；完成仍看实际位置和到位反馈。 */
#define ARM_ACTION_TIMEOUT_MS   15000u /* 动作等待告警周期，ms；到时提示并继续等待实际反馈。 */
#define ARM_DEBUG_UART_BAUD     115200u /* 保留的机构调试串口值，bit/s；当前合并入口未引用，实际无线调试用 CAR_DEBUG_UART_BAUD。 */

/* 编译期净空约束 */
#if ARM_PLATFORM_TURN_START_LIFT_MM < ARM_LIFT_MIN_MM || ARM_PLATFORM_TURN_START_LIFT_MM > ARM_LIFT_HOME_MM
#error Invalid_platform_turn_start_height
#endif

#if ARM_CLEARANCE_MM < ARM_LIFT_MIN_MM || ARM_CLEARANCE_MM > ARM_LIFT_HOME_MM
#error Invalid_arm_clearance_height
#endif

/*
 * 需要联动的调整
 *
 * 1. 手动标定与 PC9 冷启动
 *    先把机构置于已知、可检查的姿态。串口 arm on 后，等待四轮互锁交接完成，再发
 *    arm pose SLIDE_MM LIFT_MM YAW_DEG CLAW_DEG 声明实际坐标；用 arm platform pose DEG
 *    声明平台实际角度，命令会提交该角度的 PWM。随后用 arm slide N、arm lift N、
 *    arm yaw D、claw angle D 逐轴小步试位，记录绝对目标，检查放下、夹紧、抬高和收回。
 *    原料三点和车载杯位回填本文件；粗加工/暂存地面位置回填 station_config.h。
 *    PC9 冷启动按 ARM_PC9_START_SLIDE_MM 和 ARM_LIFT_HOME_MM 建立两轴坐标。
 *    按键前须人工摆到这两个实际位置，空爪且云台、载物台具备回正净空。
 *    首段起步时平推收回，同时输出云台 HOME、夹爪 OPEN 和载物台 FIRST 的目标 PWM。
 *
 * 2. CAN 驱动器与舵机硬件
 *    换轴或驱动器时，在对应驱动器配置界面先核对地址、细分、方向与控制参数。
 *    回填本文件的地址、方向与计数换算，再作单轴短行程；核对实际方向、距离、CPOS，
 *    并复测边界和净空。驱动器内部闭环设置以对应型号为准，当前固件下发位置、速度
 *    和加速度指令。升降/平推动作顺序和轴反馈处理见 arm_control.c。
 *    改舵机型号、PWM 量程或引脚时，核对 arm_control.c 的 arm_pwm_init、
 *    servo_pose、platform_pose 和 yaw_of_pulse；现有 TIM2 PA1/PA2/PA3 分别用于夹爪、
 *    平台、云台。实测脉宽与机械角再校零、方向和限位，最后复测每个工作角。
 */

#endif
