#!/bin/bash

source  /opt/ros/noetic/setup.bash
source ~/ResearchGroup6/devel/setup.bash
source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware/ ~/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware/Tools/sitl_gazebo
clear
alias python="python3"
# 等待接收公司数据
gnome-terminal -- bash -c "cd ~/ResearchGroup6/src/communication/scripts && python receive_company_data.py; exec bash"
# 对公司数据进行解析，生成无人机的航迹，并进行仿真
gnome-terminal -- bash -c "cd ~/ResearchGroup6/src/fixed_wing_formation/scripts && python GVF_ode_node.py; exec bash"
