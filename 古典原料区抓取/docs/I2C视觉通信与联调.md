# I2C视觉通信与联调

当前整车版本`20261030-merge2-r117`（IIC实现沿用r115），依据操作者提供的`I2C_TEST.md`，对接MaixCAM的`3.9.89-i2c-test`。操作者确认PF1/PF0被占用，改用PG15/PD7模拟I2C，取代r112硬件I2C2、r111 UART4及未交付的模拟串口草案。相机原为400kHz，操作者选择先降至10kHz联调。本次没有访问相机、修改Linux适配器、烧录STM32或验证真实总线。

## 接线

| STM32 | MaixCAM2 |
|---|---|
| PG15，模拟SCL | A1，SCL |
| PD7，模拟SDA | A0，SDA |
| GND | GND |

r115中GPIO为普通开漏输出并启用内部上拉（GPIO_PuPd_UP），释放为高、拉低为低，同时读IDR采样总线。**操作者2026-10-30确认：没有外接上拉电阻，只使用STM32内部上拉到3.3V。** 以前“外接上拉保留”的文字是错误假设，已更正；内部上拉不代表现场上升沿或通信已验证，相机端内部上拉配置未取证。不是UART交叉接法，也不使用5V信号上拉。相机原供电方案保留。视觉不再初始化PF0/PF1及I2C2，也不改占用这两脚的设备。详细现状、黄金数据和分阶段验收见[IIC交接文档](IIC交接文档_2026-10-30.md)。

EXTI15映射GPIOG、EXTI7映射GPIOD，两个共享IRQ只处理这两条线。无TIM、硬件UART或I2C中断。软件模拟从机的时序限制参见[ST AN4655](https://www.st.com/resource/en/application_note/an4655-virtually-increasing-the-number-of-serial-communication-peripherals-in-stm32-applications-stmicroelectronics.pdf)。

## 主从与邮箱契约

MaixCAM是`/dev/i2c-6`主机，STM32是7位地址`0x42`从机；线上地址字节为写`0x84`、读`0x85`。主机程序使用7位`0x42`，不能把`0x84`当作Linux设备地址。本版不用硬件OwnAddress寄存器。

每次主机read/write都恰好32字节，无寄存器地址字节，无额外SMBus长度字段。

| 偏移 | 含义 |
|---|---|
| 0 | 有效流字节数N，0..31 |
| 1..N | 原`AA 55 LEN payload CRC8 0D`协议的N个字节 |
| N+1..31 | 填0；接收方忽略填充区 |

主机write传视觉回复，read取STM32命令。空读返回32个0。帧边界可与邮箱边界不同，最大53字节帧可拆成31+22。IRQ仅搬运邮箱，不等待图像识别、不打印或解析视觉帧。收到完整、合法写事务后才把有效字节提交原解析器。

发送使用128字节环形流缓存。识别读地址后，在第9个ACK时钟的低电平准备完整邮箱；传输期间内容不变。完整32字节读并NACK后才消费其中N字节。START/STOP、地址、8位数据和ACK/NACK由双边沿EXTI状态机处理；在SCL下降后短暂拉低伸钟，设置SDA后释放SCL，不使用阻塞等待。短读、超长读不提交待发数据，下一次正常读取可继续。STOP及重复START按事务边界处理。错误地址不ACK；接收事务只在完整32字节且N合法时提交。

显式取消丢弃未发送数据；已经在主机读取中的邮箱不能撤回，通过代次隔离避免旧事务完成时误消费新命令。错误只记录并保留未完成请求，不新增底盘/机构自动故障停机。主机重试与链路重传不构成端到端exactly-once保证，原帧格式没有新增事务序号。

SCL频率由MaixCAM的Linux I2C适配器设置；文档里的115200不适用于I2C，10ms是轮询间隔。操作者报告当前400kHz，并选择先降至10kHz再联调：应在相机板对应的适配器/设备树配置中将目标设为10000Hz，并用逻辑分析仪核对实际SCL；原交接文档未提供该板修改入口，本项目也没有相机部署代码，不能编造命令或声称已经改频。

400kHz半周期仅1.25μs，本方案不作为该速率交付。两个EXTI采用现有分组下抢占优先级0，但仍会受到同优先级串口ISR和短暂全局关中断影响。SCL下降后的软件伸钟不能补救已经漏掉的边沿；检测到完整脉冲被合并时，仅标记接收错误、保留TX，等新START，不自动重发、不停底盘。10kHz的半周期50μs只提供更宽处理窗口，并非实测吞吐或可靠性保证。主机须支持时钟伸展；先看START、地址ACK及32字节ACK/NACK，再确认READY往返。

## r114的START修复与验收边界

旧实现要求START的SDA边沿单独处理。如果空闲START与第一个SCL下降沿一起挂起，就会仍处于IDLE并不应答地址；此时writes与bus_err也可能均为0。r114在phase=IDLE、上一SCL高且当前低、两线都有待处理边沿时先建立地址接收状态，再处理该下降沿；此时SDA可以仍低，也可以已经被主机设置成地址最高位。只恢复这一可定位的空闲起始条件，不将事务内的数据边沿猜成重复START，也不恢复丢失的完整时钟或承诺任意中断延迟下可用。

回归先在旧实现上复现失败，再验证两种IRQ先后顺序/地址位准备时机、0x84写入完整READY邮箱、0x85读出、错地址NACK，以及全部256种数据模式不误触发START。该结果证明指定软件路径已修复，不能证明实车1800多次写失败就由此引起。

## 初次联调

1. 先将相机Linux适配器由400kHz降至10kHz并核对实际波形，再烧录当前r117、按表接线并启动相机I2C测试版本。MCU开机应显示`soft I2C slave 0x42 PG15=SCL PD7=SDA; mailbox=32`。真实`Errno 121`是否消失需在相机日志核对，固件构建不能证明这一点。
2. 先确认开机`[drive] version=20261030-merge2-r117`；用逻辑分析仪检查0x84后的第9个SCL高电平时SDA确实被拉低。再让相机队列中的启动READY完成一次write，执行`vision status`，记录`frames`，确认`last="READY"`。
3. 执行现有`vision range`，它发送原始帧R。`queued R`仅表示排队；等`[vision i2c] reads`增加、`queued`归0，并在相机端看到RX R及新READY的`[I2C] TX transaction delivered`。随后再次`vision status`，应看到`frames`增加且`last="READY"`，不能把第2步的启动READY当作新应答。
4. 再按原流程测试视觉指令。`vision ping`原本发D，不能替代第3步的R测试。D仍在原到站交接后发送，H/E/F/C提前发送及其FIFO规则保持；PICK/MOVE/终包只在主机已取走相应请求后才推进。

`vision status`保留`addr/mailbox/reads/writes/queued/invalid/bus_err`。`reads/writes`是完整邮箱事务统计，包含空邮箱，不等于图像识别次数。短包/非法N计入invalid；本版bus_err是EXTI发现的脉冲合并计数，并非覆盖所有漏边沿的硬件检测器。正常读完的NACK不算错误。

`tools/verify-vision-i2c.ps1`执行真实软件从机分支的开漏线/EXTI逐边沿模型，覆盖空闲START合并边沿及256种数据模式、START/STOP、地址ACK、主机ACK/NACK、邮箱拆帧、填充、短读、超长、重复START、取消、脉冲合并及READY测试。模型在每条边沿后确定性调度ISR，不证明10kHz现场中断延迟，也不证明400kHz可用。高层A/C作业模型仍使用既有字节流替身。构建、源码哈希和未通过项以`evidence/current-validation.json`为准。
