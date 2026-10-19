# 底盘与机械机构合并工程

- 本目录是独立合并工程，唯一入口为 `keil/TableNavigation.uvprojx`。保持 `../stm32-car-test`、`../stm32-arm-test` 和 `../gcs-test` 原样。
- 底盘使用本地当前runimu控制；merged2仅合入机械机构、任务码、UART5视觉与到站作业。操作者已确认默认接入自动到站作业；`station auto off`仅跑底盘。未标定的粗加工/暂存/平台取料表必须拒绝执行，不能用占位数值放行。
- 自动行驶要求机械臂已声明且收拢，行驶与工位互斥地消费四轮CAN队列；工位完成须收拢并重新采集四轮反馈后才可发车。不导入独立测试程序的全部main或电脑端烧录脚本。
- `!`始终是全系统急停请求，不得复用为夹取。机构使用 `arm ...` / `claw ...` 文本命令。
- 底盘CAN地址1～4，升降5，平推7；PA1夹爪、PA2载物台、PA3云台使用TIM2，底盘毫秒计时使用TIM5。机构不发送广播同步命令。
- 当前参数来源和操作见 `docs/合并与串口操作.md`；当前验证以 `evidence/current-validation.json` 的源码哈希和结果为准。
- `docs`中其他文档和 `evidence/chassis-history` 属于继承的底盘资料，不能当成本合并版本的实车验收。
- 修改运动、CAN或互锁时运行 `tools/verify-heading.ps1`、`tools/verify-arm-integration.ps1`，并完整ARM构建；本地测试不代表烧录或真实动作已通过。
