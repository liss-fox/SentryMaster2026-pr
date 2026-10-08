# 保证libusb库的最新版可以被找到（没有该文件时跳过，方便在无机器人的电脑上运行）
if [ -f /usr/local/lib/libusb-1.0.so ]; then
    export LD_PRELOAD=/usr/local/lib/libusb-1.0.so
fi

# 获取livox消息类型（没有串口设备时跳过）
if [ -e /dev/ttyACM0 ]; then
    sudo chmod 777 /dev/ttyACM0
fi

cd ..
source install/setup.bash
ros2 launch sentry_launch chassis.launch.py