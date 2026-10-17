#!/bin/sh
# run.sh —— 在 MaixCAM2 上跑视觉程序（不依赖 MaixVision，拔掉 USB 也能跑）
#
#   sh /root/maixcam_app_v2/run.sh          # 前台运行，日志写 finder.log
#   sh /root/maixcam_app_v2/run.sh &        # 后台运行
#   tail -f /root/maixcam_app_v2/finder.log # 看日志
cd "$(dirname "$0")" || exit 1
echo "=== $(date) start ===" >> finder.log
python3 -u main.py 2>&1 | tee -a finder.log
echo "=== $(date) exit code: $? ===" >> finder.log