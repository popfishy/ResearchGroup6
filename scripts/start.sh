#!/bin/bash

source  /opt/ros/noetic/setup.bash
source ~/ResearchGroup6/devel/setup.bash
source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware/ ~/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware/Tools/sitl_gazebo
clear
alias python="python3"

cd ~/ResearchGroup6/scripts
#echo 'dji' | sudo -S chmod 777 QGroundControl.AppImage

# 定义变量
UAV_NUM=7

# 启动仿真环境
gnome-terminal -- bash -c "cd ~/PX4_Firmware && roslaunch px4 multi_vehicle.launch"
sleep 10
# 启动QGC地面站
gnome-terminal -- bash -c "cd ~/ResearchGroup6/scripts && ./QGroundControl.AppImage"
sleep 2
# 启动无人机通信
gnome-terminal -- bash -c "cd ~/ResearchGroup6/src/fixed_wing_formation/scripts && bash multi_vehicle_communication.sh; exec bash"
sleep 1.5
# 启动无人机键盘控制，需手动解锁起飞，按o发送航点后切换至offboard模式。在未接收到公司数据前，无人机将在(0,0,200)处盘旋
gnome-terminal -- bash -c "cd ~/ResearchGroup6/src/fixed_wing_formation/scripts && python plane_keyboard_control.py $UAV_NUM "
sleep 1
gnome-terminal -- bash -c "cd ~/ResearchGroup6/src/fixed_wing_formation/scripts && python QGC_param_set.py plane $UAV_NUM "
sleep 0.5
# 发送无人机数据到公司
gnome-terminal -- bash -c "cd ~/ResearchGroup6/src/communication/scripts && python send_data_to_company.py plane $UAV_NUM "
sleep 1
# 等待接收公司数据
gnome-terminal -- bash -c "cd ~/ResearchGroup6/src/communication/scripts && python receive_company_data.py; exec bash"
# 对公司数据进行解析，生成无人机的航迹，并进行仿真
gnome-terminal -- bash -c "cd ~/ResearchGroup6/src/fixed_wing_formation/scripts && python GVF_ode_node.py; exec bash"
