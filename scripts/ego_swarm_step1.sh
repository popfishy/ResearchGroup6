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

gnome-terminal -- bash -c "cd ~/PX4_Firmware && roslaunch px4 multi_vehicle.launch"
sleep 8
gnome-terminal -- bash -c "cd ~/catkin_ws && roslaunch vins xtdrone_run_vio$UAV_NUM.launch"
sleep 1
# gnome-terminal -- bash -c "cd ~/XTDrone/sensing/slam/vio && python multi_vins_transfer.py iris $UAV_NUM"
gnome-terminal -- bash -c "cd ~/XTDrone/sensing/pose_ground_truth/ && python get_local_pose.py iris $UAV_NUM"
sleep 1
gnome-terminal -- bash -c "cd ~/XTDrone/communication && bash multi_vehicle_communication.sh; exec bash"
sleep 1.5
#gnome-terminal -- bash -c "cd ~/catkin_ws/src/ego_planner/scripts && python3 fly_to_regular_height.py iris $UAV_NUM vel; exec bash"
gnome-terminal -- bash -c "cd ~/XTDrone/control/keyboard && python multirotor_keyboard_control.py iris $UAV_NUM vel"
