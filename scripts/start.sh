#!/bin/bash

source  /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware/ ~/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware/Tools/sitl_gazebo
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src/ORB_SLAM2/Examples/ROS
clear
alias python="python3"
# 设置需要启动和监视的ros命令
declare -a commands=("roslaunch px4 multi_vehicle_without_PX4.launch ")

# 定义函数来启动ros命令
start_commands() {
  for cmd in "${commands[@]}"
  do
	gnome-terminal -- bash -c " $cmd;exec bash;"
  done
}

start_commands
sleep 60
bash stop_gazebo_ode.sh
