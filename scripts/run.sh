#!/bin/bash
cd "$(dirname "$0")"
source ../install/setup.bash

LOG_DIR=~/sentry_logs/$(date +%m%d_%H%M%S)
mkdir -p "$LOG_DIR"
echo "=== logs: $LOG_DIR ==="

# запись топиков в фоне
ros2 bag record -o "$LOG_DIR/bag" \
    /odin1/odometry_highfreq /decision_to_planner /planner_to_driver &
BAG_PID=$!
trap 'kill -TERM $BAG_PID 2>/dev/null' EXIT

# основной запуск, лог в файл
bash chassis.sh 2>&1 | tee "$LOG_DIR/run.log"