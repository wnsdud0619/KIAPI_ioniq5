#!/bin/bash

PASSWORD="1"

# --- 0. can0 상태 확인 및 설정 ---
state=$(ip link show can0 2>/dev/null | grep -o 'state [A-Z]*' | awk '{print $2}')
if [ "$state" != "UP" ]; then
  echo "[INFO] can0 is not UP. Enabling with bitrate 500000..."
  expect <<EOF
spawn sudo ip link set can0 up type can bitrate 500000
expect {
    "password" {
        send "$PASSWORD\r"
        exp_continue
    }
    eof
}
EOF
else
  echo "[INFO] can0 is already UP."
fi

# 1. velodyne # ros2 launch velodyne velodyne-all-nodes-VLS128-launch.py
cd ~/velodyne
xterm -iconic -hold -e bash -c "source install/setup.bash; ros2 launch velodyne velodyne-all-nodes-VLS128-composed-launch.py" &

# 2. novatel
cd ~/novatel_oem7_driver
xterm -iconic -hold -e bash -c '
DEVICE=/dev/ttyUSB0

expect <<EOF
spawn sudo chmod 777 $DEVICE
expect {
    "password" {
        send "'"$PASSWORD"'\r"
        exp_continue
    }
    eof
}
EOF

source install/setup.bash;
ros2 launch novatel_oem7_driver oem7_port.launch.py oem7_port_name:=$DEVICE
' &

## 3. autoware_convert_topic
#cd ~/KIAPI_ioniq5
#xterm -iconic -hold -e bash -c "source install/setup.bash; ros2 launch autoware_convert_topic convert.launch.py" &

## 4. autoware-launch-gui
cd ~/KIAPI_ioniq5
xterm -iconic -hold -e bash -c "source install/setup.bash; autoware-launch-gui" &

##5. GPS panel
#cd ~/gps_custom_panel
#xterm -iconic -hold -e bash -c "source install/setup.bash" &

##6. state pub
cd ~/KIAPI_ioniq5
xterm -iconic -hold -e bash -c 'source install/setup.bash; ros2 topic pub /system/operation_mode/state autoware_adapi_v1_msgs/msg/OperationModeState "{stamp: {sec: '"$(date +%s)"', nanosec: 0}, mode: 3, is_autonomous_mode_available: true, is_local_mode_available: false, is_remote_mode_available: false, is_stop_mode_available: false, is_autoware_control_enabled: true}"' &

##7. engage true
cd ~/KIAPI_ioniq5
xterm -iconic -hold -e bash -c 'source install/setup.bash; ros2 topic pub /autoware/engage autoware_vehicle_msgs/msg/Engage "engage: true"' &

#8. motion state 
cd ~/KIAPI_ioniq5
xterm -iconic -hold -e bash -c 'source install/setup.bash; ros2 topic pub /api/motion/state autoware_adapi_v1_msgs/msg/MotionState "{stamp: {sec: $(date +%s), nanosec: 0}, state: 3}"' &

#9. pub orientation 
#cd ~/GPS_orientation
#xterm -iconic -hold -e bash -c 'source install/setup.bash; ros2 run orientation_pub gps_imu_sync_node' &

exit 0

