#!/bin/sh
# Run on MaixCAM2. Follow finder.log from another terminal for live output.
cd "$(dirname "$0")" || exit 1
printf '=== %s start ===\n' "$(date)" >> finder.log || exit 1
python3 -u main.py >> finder.log 2>&1
status=$?
printf '=== %s exit code: %s ===\n' "$(date)" "$status" >> finder.log
exit "$status"
