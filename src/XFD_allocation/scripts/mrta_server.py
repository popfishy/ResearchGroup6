#!/usr/bin/env python3
import rospy
import random
import threading
from mrta_service.srv import TaskAllocation, TaskAllocationResponse
from mrta_service.msg import LS
from CBPA.lib.CBPA_REC import CBPA_REC  # 导入你的CBPA求解器类
from CBPA.lib.WorldInfo import WorldInfo  # 导入你的世界信息类
from CBPA.lib.Robot import Robot
from CBPA.lib.Task import Task
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from CBPA.lib.Task import Task
import numpy as np
from group6_interface.msg import AgentData, AgentsData, TargetData, TargetsData

# Global variable
path_list = []
idx_list = []
RobotList = []
TaskList = []
robot_lock = threading.Lock()
task_lock = threading.Lock()


# 回调函数，处理服务请求
def handle_task_allocation(req):
    rospy.loginfo("Received task allocation request")

    # 根据请求数据创建机器人和任务列表
    global RobotList, TaskList, path_list, idx_list
    global robot_lock, task_lock
    
    robot_lock.acquire()
    task_lock.acquire()

    # 执行任务分配
    path_list, idx_list, RobotList, TaskList = CBPA_solver.solve_centralized(RobotList, TaskList, WorldInfoTest)
    
    robot_lock.release()
    task_lock.release()

    # 构造并返回响应
    response = TaskAllocationResponse()
    for i in range(num_robots):
        response.path_list[i].ls = path_list[i]
    print("发送:",response)
         
    # response.path0 = path_list[0]
    return response

#RobotList: list, TaskList: list, 
# def plot_robot_schedules(path_list: list, robot_times_list: list, task_times_list: list, RobotList: list, TaskList: list):
def plot_robot_schedules(event):
    if path_list:
        # 清空之前的图表
        # plt.close('all')
        plt.clf()
        num_robots = len(RobotList)
        # print("num_robot:",num_robots)
        # 3D plot 
        fig_3d = plt.figure(1) #创建一个图形对象
        ax_3d = fig_3d.add_subplot(111, projection='3d')  #创建一个3D子图对象ax_3d，并将投影类型设置为’3d’
        # offset to plot text in 3D space
        offset = ( WorldInfoTest.limit_x[1]- WorldInfoTest.limit_x[0]) / 50 # 坐标单位长度

        # 绘制机器人
        color_list = []
        for n in range( num_robots):
            color_list.append(mcolors.to_hex([random.random(), random.random(), random.random()]))
            ax_3d.scatter( RobotList[n].x,  RobotList[n].y,  RobotList[n].z, marker='o', color=color_list[n])
            ax_3d.text( RobotList[n].x+offset,  RobotList[n].y+offset,  RobotList[n].z, "R"+str(n))
        
        # 绘制任务
        #color_str = "yellow"
        color_str = "#4B57A2"
        for m in range( num_tasks):
            
            ax_3d.scatter( TaskList[m].x,  TaskList[m].y,  TaskList[m].z, marker='^', s=30, color=color_str) #任务散点
            
            ax_3d.text( TaskList[m].x+offset,  TaskList[m].y+offset, TaskList[m].z, "T"+str(m))
            # if the path is not empty
            if path_list[m]:
                for j in range(len(path_list[m])-1):
                    idx = idx_list[m][j+1]
                    ax_3d.plot3D([ RobotList[idx].x,  TaskList[m].x], [ RobotList[idx].y,  TaskList[m].y],
                                [ RobotList[idx].z,  TaskList[m].z], linewidth=2, color=color_list[idx]) #绘制从Robot位置到第一个任务位置的路径
        
        plt.title('Robot Paths')
        ax_3d.set_xlabel("X")
        ax_3d.set_ylabel("Y")
        ax_3d.set_zlabel("Z")
        ax_3d.set_aspect('auto')

        # set legends
        colors = ["#7FABD1", "#F39865", "#963B79"]

        marker_list = ["solid", "solid", "solid"]
        labels = ["Reconnaissance robot", "Strike robot", "Reconnaissance & Strike robot"]
        def f(marker_type, color_type): return plt.plot([], [], linestyle=marker_type, color=color_type)[0]
        handles = [f(marker_list[i], colors[i]) for i in range(len(labels))]
        plt.legend(handles, labels, bbox_to_anchor=(1, 1), loc='upper left', framealpha=1)
        set_axes_equal_xy(ax_3d, flag_3d=True)

    def set_axes_equal_xy(ax, flag_3d: bool):
        """
        Make only x and y axes of 3D plot have equal scale. This is one possible solution to Matplotlib's
        ax.set_aspect('equal') and ax.axis('equal') not working for 3D.
        Reference: https://stackoverflow.com/questions/13685386/matplotlib-equal-unit-length-with-equal-aspect-ratio-z-axis-is-not-equal-to

        Input
        ax: a matplotlib axis, e.g., as output from plt.gca().
        flag_3d: boolean, True if it's 3D plot.
        """

        x_limits = WorldInfo.limit_x 
        y_limits = WorldInfo.limit_y

        x_range = abs(x_limits[1] - x_limits[0])
        x_middle = np.mean(x_limits)
        y_range = abs(y_limits[1] - y_limits[0])
        y_middle = np.mean(y_limits)

        # The plot bounding box is a sphere in the sense of the infinity
        # norm, hence I call half the max range the plot radius.
        plot_radius = 0.5*max([x_range, y_range])

        if flag_3d:
            ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
            ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
        else:
            ax.set_xlim([x_middle - plot_radius, x_middle + plot_radius])
            ax.set_ylim([y_middle - plot_radius, y_middle + plot_radius])

def robot_callback(Agentsmsg):
    """
    更新巡飞弹信息
    """
    rospy.loginfo("Update Agents Information")
    # 根据接收消息创建机器人列表
    global RobotList,robot_lock

    # 加载XFD默认信息
    robot_rs_default = Robot()
    robot_rs_default.robot_status = False
    robot_rs_default.nom_velocity = 0
    agents_data = Agentsmsg.agents_data
    num_robots = len(agents_data)

    robot_lock.acquire()
    RobotList = []
    for idx_robot in range(0, num_robots):
        RobotList.append(Robot(**robot_rs_default.__dict__))
        RobotList[idx_robot].robot_id = agents_data[idx_robot].missile_id
        RobotList[idx_robot].x = agents_data[idx_robot].x
        RobotList[idx_robot].y = agents_data[idx_robot].z
        RobotList[idx_robot].z = agents_data[idx_robot].y
        RobotList[idx_robot].robot_status = True
        RobotList[idx_robot].nom_velocity = agents_data[idx_robot].missile_flight_speed
    robot_lock.release()

def target_callback(Targetsmsg):
    """
    更新巡飞弹信息
    """
    rospy.loginfo("Update Agents Information")
    # 根据接收消息创建机器人列表
    global TaskList,task_lock

    # 加载XFD默认信息
    task_default = Task()
    task_default.task_status = False
    targets_data = Targetsmsg.targets_data
    num_tasks = len(targets_data)

    task_lock.acquire()
    TaskList = []
    for idx_task in range(0, num_tasks):
        TaskList.append(Task(**task_default.__dict__))
        TaskList[idx_task].task_id = targets_data[idx_task].target_id
        TaskList[idx_task].x = targets_data[idx_task].x
        TaskList[idx_task].y = targets_data[idx_task].y
        TaskList[idx_task].z = targets_data[idx_task].z
        TaskList[idx_task].task_type = targets_data[idx_task].target_type
        if TaskList[idx_task].task_type == 1:
            TaskList[idx_task].robot_need = 4

        elif TaskList[idx_task].task_type == 2:
            TaskList[idx_task].robot_need = 3

        elif TaskList[idx_task].task_type == 3:
            TaskList[idx_task].robot_need = 1

        elif TaskList[idx_task].task_type == 4:
            TaskList[idx_task].robot_need = 1

        elif TaskList[idx_task].task_type == 5:
            TaskList[idx_task].robot_need = 1

        elif TaskList[idx_task].task_type == 6:
            TaskList[idx_task].robot_need = 1

        elif TaskList[idx_task].task_type == 7:
            TaskList[idx_task].robot_need = 1

        elif TaskList[idx_task].task_type == 8:
            TaskList[idx_task].robot_need = 2

        else:
            TaskList[idx_task].robot_need = 0
    task_lock.release() 


# 主程序入口
if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('target_allocation_server')
    

    num_robots = 10
    num_tasks = 10

    # 创建一个世界，每个列表都是[x,y,z]轴的[min, max]坐标
    WorldInfoTest = WorldInfo([0.0, 10000.0], [0.0, 10000.0], [0.0, 1000.0])

    # 创建CBPA求解器
    CBPA_solver = CBPA_REC()

    # 订阅 robot/pos 话题
    rospy.Subscriber('/missile_data', AgentsData, robot_callback)
    rospy.Subscriber('/target_data', TargetsData, target_callback)

    # 注册服务
    task_allocation_service = rospy.Service('task_allocation', TaskAllocation, handle_task_allocation)
    rospy.loginfo("Ready to allocate tasks.")

    # 循环等待服务请求
    timer = rospy.Timer(rospy.Duration(1.0), plot_robot_schedules)
    rospy.spin()
