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
UAV_NUM=1


# 启动仿真环境
gnome-terminal -- bash -c "cd ~/PX4_Firmware && roslaunch px4 outdoor2.launch"
sleep 5
# 启动QGC地面站
gnome-terminal -- bash -c "cd ~/ResearchGroup6/scripts && ./QGroundControl.AppImage"
sleep 2
# 启动无人机通信
gnome-terminal -- bash -c "cd ~/ResearchGroup6/src/fixed_wing_formation/scripts && python plane_communication.py 0; exec bash"
sleep 1.5
# 启动无人机键盘控制，需手动解锁起飞，按o发送航点后切换至offboard模式。在未接收到公司数据前，无人机将在(0,0,200)处盘旋
gnome-terminal -- bash -c "cd ~/ResearchGroup6/src/fixed_wing_formation/scripts && python plane_keyboard_control.py $UAV_NUM "
sleep 1
# 修改部分QGC参数
gnome-terminal -- bash -c "cd ~/ResearchGroup6/src/fixed_wing_formation/scripts && python QGC_param_set.py plane $UAV_NUM "
