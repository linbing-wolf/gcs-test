# MaixCAM J 与 H 顺逆时针现场发布

版本为 3.9.97-i2c-gpio。此目录为基于现有板端 v3.9.96 GPIO I²C 应用的直接文件覆盖层，运行文件已经部署到 `/maixapp/apps/yyb_mover/`。

保留设备原有模型、vision.py、precision.py、h_material_config.py、GPIO I²C 传输模块等依赖。config.py 是本次设备的参数快照，其他车辆应先核对自身标定和接线。

新增 J 无总超时停稳查询，MSTATE 带 done 和各颜色状态。H 开启 0/1 顺逆时针包，自动判向不足时采用配置兜底 1。H Y=318，D dy=40，E dx=0。

manifest.json 记录 7 个运行文件的 SHA256。deployment.json 记录实际备份路径、启动版本和验证范围。协议与流程详见仓库 vision-maixcam 目录内的 HANDOVER_J_20261122.md 与 HANDOVER_H_DIRECTION_20261122.md。

板端已启动并成功发送 READY，真实 J/H 现场任务仍需联调观察。恢复旧版可使用 deployment.json 指定的板端备份，恢复文件后重启视觉应用。
