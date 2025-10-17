#!/usr/bin/env python3
import time
import json
import matplotlib.pyplot as plt
# import pathmagic
import os

from CBPA.lib.CBPA_REC import CBPA_REC
from CBPA.lib.WorldInfo import WorldInfo
import CBPA.lib.HelperLibrary as HelperLibrary


if __name__ == "__main__":
    # json 配置文件
    config_file_name = "CBPA/config_example_01.json"
    
    # 读取json文件中的配置信息
    json_file = open(config_file_name)
    config_data = json.load(json_file)

    # 创建一个世界，每个列表都是[x,y,z]轴的[min, max]坐标
    #WorldInfoTest = WorldInfo([-100.0, 100], [-50, 50], [0.0, 20.0])
    WorldInfoTest = WorldInfo([0.0, 2420.0], [0.0, 1380.0], [0.0, 1000.0])

    # 创建机器人和任务列表
    num_robots = 100
    num_tasks = 10
    max_depth = 1   #任务包深度 exp1: cbpa=4 paimai=5
    bid_times = 3*num_tasks
    global RobotList, TaskList
    RobotList, TaskList = HelperLibrary.create_robots_and_tasks_XFD(num_robots, num_tasks, WorldInfoTest, config_data)

    # 创建一个CBPA求解器
    CBPA_solver = CBPA_REC()
    t_start = time.time()

    # 任务分配
    # path_list, robot_times_list, task_times_list, robotNum_of_task, winner_strload_list, RobotList, TaskList = CBPA_solver.solve(RobotList, TaskList, WorldInfoTest, max_depth, bid_times, time_window_flag=True)
    path_list, idx_list, RobotList, TaskList = CBPA_solver.solve_centralized(RobotList, TaskList, WorldInfoTest)

    t_end = time.time()
    t_used = t_end - t_start
    print("分配用时 (秒): ", t_used)
    # 打印任务包、路径、时间、赢家、分数、出价、赢家出价信息
    # print("bundle_list")
    # print(CBPA_solver.bundle_list)
    print("path_list")
    print(path_list)

    # 绘图
    CBPA_solver.plot_assignment_XFD(path_list, idx_list)
    plt.show()
