#!/bin/bash

source  /opt/ros/noetic/setup.bash
source ~/ResearchGroup6/devel/setup.bash
source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware/ ~/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware/Tools/sitl_gazebo
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src/ORB_SLAM2/Examples/ROS
clear
alias python="python3"

# 定义变量
UAV_NUM=3

gnome-terminal -- bash -c "cd ~/PX4_Firmware && roslaunch px4 multi_vehicle.launch"
sleep 10
gnome-terminal -- bash -c "cd ~/ResearchGroup6/src/fixed_wing_formation/scripts && bash multi_vehicle_communication.sh; exec bash"
sleep 1.5
gnome-terminal -- bash -c "cd ~/ResearchGroup6/src/fixed_wing_formation/scripts && python plane_keyboard_control.py $UAV_NUM "
sleep 1
gnome-terminal -- bash -c "cd ~/ResearchGroup6/src/communication/scripts && python send_data_to_company.py plane $UAV_NUM "
sleep 1
gnome-terminal -- bash -c "cd ~/ResearchGroup6/src/communication/scripts && python receive_company_data.py; exec bash"
gnome-terminal -- bash -c "cd ~/ResearchGroup6/src/fixed_wing_formation/scripts && python GVF_ode_node.py; exec bash"
