#!/bin/sh
# run.sh —— 在 MaixCAM2 上跑视觉程序（不依赖 MaixVision，拔掉 USB 也能跑）
#
# 用法：
#   sh /root/maixcam_app/run.sh            # 前台运行，日志同时写 finder.log
#   sh /root/maixcam_app/run.sh &          # 后台运行
#   tail -f /root/maixcam_app/finder.log   # 看日志
#
# 为什么需要它：MaixVision 点"运行"时是把文件拷到 /tmp/maixpy_run 再执行的，
# 拔掉 USB 后 /tmp 里的文件就没了。装到 /root 下才能独立运行。

cd "$(dirname "$0")" || exit 1
echo "=== $(date) start ===" >> finder.log
# -u 关掉输出缓冲，日志能实时看到
python3 -u main.py 2>&1 | tee -a finder.log
echo "=== $(date) exit code: $? ===" >> finder.log
