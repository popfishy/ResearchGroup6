import rospy
from geometry_msgs.msg import Pose, PoseStamped    #订阅位置 四旋翼是速度角速度
import sys, select, os     #读键盘 
import tty, termios
from std_msgs.msg import String
import time

multi_plane_pose = [None] * int(sys.argv[1])
multi_plane_pose = [Pose() for i in range(int(sys.argv[1]))]

LIN_STEP_SIZE = 1    #键盘每按一下的setpoint增长量
ANG_STEP_SIZE = 0.01   #~~绕z轴~~


ctrl_leader = False    #编队
send_flag = False      #发目标点


def pose_callback(data, i):

    multi_plane_pose[i].position.x = data.pose.position.x
    multi_plane_pose[i].position.y = data.pose.position.y
    multi_plane_pose[i].position.z = data.pose.position.z

if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)   #获取键盘值

    plane_num = int(sys.argv[1])      
    rospy.init_node('plane_keyboard_control')

    multi_cmd_pose_enu_pub = [None]*plane_num
    multi_cmd_pub = [None]*plane_num
    
    for i in range(plane_num):
        sub = rospy.Subscriber('plane_'+str(i)+'/mavros/GVF_ode/pose' ,PoseStamped , pose_callback, i, queue_size=1)
        multi_cmd_pose_enu_pub[i] = rospy.Publisher('/xtdrone/plane_'+str(i)+'/cmd_pose_enu', Pose, queue_size=1)
        multi_cmd_pub[i] = rospy.Publisher('/xtdrone/plane_'+str(i)+'/cmd',String,queue_size=1)
    leader_pose_pub = rospy.Publisher("/xtdrone/leader/cmd_pose", Pose, queue_size=1)
    leader_cmd_pub = rospy.Publisher("/xtdrone/leader_cmd", String, queue_size=3)


    cmd= String()   #这两步消息实例化是给通信脚本发的指令

    pose = Pose()
    for i in range(plane_num):
        multi_plane_pose[i].position.x = 0
        multi_plane_pose[i].position.y = 0
        multi_plane_pose[i].position.z = 200

    forward  = 0.0
    leftward  = 0.0
    upward  = 0.0
    angular = 0.0


    # 自动控制部分
    auto_takeoff_time = time.time()
    arm_time = None
    waypoints_time = None
    flag = False
    offboard_time = None
    rate = rospy.Rate(20)
    # while not rospy.is_shutdown():
    while (1):
        current_time = time.time()

        # 自动起飞
        if current_time - auto_takeoff_time < 1.0:
            cmd = 'AUTO.TAKEOFF'
            print('AUTO.TAKEOFF')

        # ARM
        elif arm_time is None and current_time - auto_takeoff_time >= 3.0:
            arm_time = current_time
            cmd = 'ARM'
            print('Arming')

        # 发布航迹点
        elif waypoints_time is None and arm_time != None and current_time - arm_time >= 50.0:
            waypoints_time = current_time
            # 发布航迹点
            print('Publishing waypoints')
            flag = True

        # OFFBOARD
        elif flag:
            if offboard_time is None and current_time - waypoints_time >= 10.0:
                offboard_time = current_time
                cmd = 'OFFBOARD'
                print('Offboard')

        # 发布命令
        for i in range(plane_num):
            if ctrl_leader:
                leader_cmd_pub.publish(cmd)
            else:
                multi_cmd_pub[i].publish(cmd)
                multi_cmd_pose_enu_pub[i].publish(multi_plane_pose[i])
