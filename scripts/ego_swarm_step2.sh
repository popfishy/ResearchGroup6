#!/bin/bash

source  /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware/ ~/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware/Tools/sitl_gazebo
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src/ORB_SLAM2/Examples/ROS
clear
alias python="python3"

#定义变量
UAV_NUM=9

gnome-terminal -- bash -c "cd ~/XTDrone/motion_planning/3d && python ego_swarm_transfer.py iris $UAV_NUM"
sleep 0.5
gnome-terminal -- bash -c "cd ~/XTDrone/motion_planning/3d && rviz -d ego_swarm_rviz_9.rviz"
sleep 0.5
# gnome-terminal -- bash -c "roslaunch ego_planner multi_uav.launch"
# gnome-terminal -- bash -c 'cd ~/catkin_ws/src/ego_planner"&& echo "bash ego_swarm2_goal.sh"; exec bash'
