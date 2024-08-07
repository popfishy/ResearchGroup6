##########################################
###           固定翼编队实现            ###
##########################################

### 0.修改参数
gedit ~/PX4_Firmware/ROMFS/px4fmu_common/init.d-posix/rcS
* 在123行左右加入以下
param set COM_RCL_EXCEPT 4 
param set NAV_DLL_ACT 0
param set NAV_RCL_ACT 0


### 1.启动仿真(启动后打开QGC)
cd ~/PX4_Firmware
roslaunch px4 multi_vehicle.launch
### 2.建立通信
cd ~/XTDrone/communication
bash multi_vehicle_communication.sh
### 3.提供位姿信息  
cd ~/XTDrone/sensing/pose_ground_truth
python get_local_pose.py plane 10          ### 十架固定翼
### 4.启动键盘控制
cd ~/XTDrone/control/keyboard
python plane_keyboard_control.py 10        ### 十架固定翼
* 操作：先按v解锁，再按t起飞，等飞行高度稳定在30m时，进行下一步操作
### 5.固定翼编队飞行
cd ~/XTDrone/coordination/formation_demo
bash run_GVF.sh plane 10                   ### 十架固定翼
* 此时再进行上一步的按键操作：按下o发送目标点，按b进入offboard模式










