1.启动固定翼的gazebo仿真环境后，打开QGC地面站
2.建立固定翼的通讯  使用fixed_wing_formation/script/multi_vehicle_communication.sh
3.使用键盘控制plane_keyboard_control.py（v -> t ->o（解锁发布航迹点) ->b（等固定翼稳定在（0,0,200）后，再往下运行    
  或者自己写的plane_without_keyboard_control.py
4.运行company_to_group6.py，模拟公司发布给课题六的消息 
5.运行receive_company_data.py接收公司的消息
6.运行GVF_ode_node.py开始执行任务


注意：4 5 6之间运行不能间隔过长。另外模拟使用的消息为test.json文件，里面有许多错误的航迹点数据。


需解决的问题：
1.QGC中如何设置固定翼飞机的速度为600m/s
2.如何解决固定翼飞机的local坐标系原点为px4上电位置 （获取固定翼飞机在gazebo中初始化的位置表？）
