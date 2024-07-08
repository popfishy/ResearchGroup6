#!/bin/bash

source  /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware/ ~/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware/Tools/sitl_gazebo
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src/ORB_SLAM2/Examples/ROS
clear
alias python="python3"
cd ../src/control/scripts/
declare -a commands=("python3 control_gazebo_vehicles.py iris 10 10"
		"python3 ui_interface.py"
		"python3 complex_networks.py")

start_commands() {
  for cmd in "${commands[@]}"
  do
	gnome-terminal -- bash -c " $cmd;exec bash;"
  done
}

start_commands
cd ~/catkin_ws/scripts/
