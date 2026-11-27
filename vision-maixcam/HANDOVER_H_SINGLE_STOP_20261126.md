# H 单件停稳即时纠偏版交接

更新日期为 2026 年 11 月 26 日。

## 行为

本版替换当前 `H_MATERIAL_MODE` 的停稳流程，协议格式保持不变。

1. 在 `H_MATERIAL_ROI` 内等待任一物料停稳，不要求同时看到两种颜色。
2. 多个物料同时停稳时，选择画面中 `y` 最大的物料，即最下方物料。
3. 将该物料作为后到色 `c2`。先到色 `c1` 从其余颜色码中任选一个不同值补齐。
4. 依次发送 `HCOLORS,c1,c2,direction` 和一条 `MOVEXY,dx,dy`。
5. 无论动作前是否已经在容差内，都固定发送一次 `MOVEXY`。两轴误差都为 0 时发送 `MOVEXY,0,0`。
6. 当前默认仍发送 `HDISC,x,y,dx,c1,c2,direction`，不等待动作后的视觉复核，不因对齐精度改变成功语义。

## 实验版 HDISC2

`H_MATERIAL_EXTENDED_PACKET_ENABLED = True` 时，MOVEXY 后等待当前 `settle_ms`，再扫描完整视野并返回：

```text
HDISC2,arrival,n,color,x,y[,color,x,y,...]
```

- `arrival`：收到 H 后、首次发出校正前的停稳状态。`1` 表示未观察到运动即已停稳，`0` 表示观察到运动后才停稳。
- `n`：校准后完整视野内的有效颜色数量，最多 4 个。
- 后续按 `x` 从小到大排列，每个颜色只保留面积最大的一个目标。
- `x,y` 使用对外 640×480 坐标口径。

该包不重复发送旧 `HDISC` 的基准 `x,y,dx`、双色顺序和方向字段。

STM32 侧抓取流程见 `HANDOVER_STM32_HDISC2_20261126.md`。

当方向格式开关关闭时，仍兼容原格式 `COLORS,c1,c2` 和 `DISC,x,y,dx,c1,c2`。

## 当前参数

- H 停稳 ROI：`[140,160,360,320]`
- X 目标：`320`
- Y 目标：`318`
- 单次增益：板端 `h_material.json` 的 `gain`，当前默认 `0.65`
- 步长上限：板端 `h_material.json` 的 `step_max`，当前为 `0`，即不限
- 方向格式：`HCOLORS` / `HDISC`
- 方向兜底：`1`，即逆时针

## 配置开关

`H_MATERIAL_SINGLE_STOP_MODE = True`

关闭该开关后恢复原有等待双色、按到达顺序记录颜色的流程。

## 返回坐标

`x,y,dx` 都取发出唯一一次 `MOVEXY` 之前的当前物料坐标和水平误差。`dy` 只用于生成移动指令，不包含在终包中。
