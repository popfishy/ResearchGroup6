#!/usr/bin/env python3
import sys
import time
import json
import matplotlib.pyplot as plt
# sys.path.append("..")
# with pathmagic.context():
from CBPA.lib.CBPA import CBPA
from lib.WorldInfo import WorldInfo
import lib.HelperLibrary as HelperLibrary


if __name__ == "__main__":
    # json 配置文件
    config_file_name = "config_example_01.json"
    
    # 读取json文件中的配置信息
    json_file = open(config_file_name)
    config_data = json.load(json_file)

    # 创建一个世界，每个列表都是[x,y,z]轴的[min, max]坐标
    #WorldInfoTest = WorldInfo([-100.0, 100], [-50, 50], [0.0, 20.0])
    WorldInfoTest = WorldInfo([0.0, 2420.0], [0.0, 1380.0], [0.0, 0.0])

    # 创建机器人和任务列表
    num_robots = 6
    num_tasks = 10
    max_depth = 5   #任务包深度 exp1: cbpa=4 paimai=5
    bid_times = 3*num_tasks
    global RobotList, TaskList
    RobotList, TaskList = HelperLibrary.create_robots_and_tasks_exp2_4(num_robots, num_tasks, WorldInfoTest, config_data)

    # 创建一个CBPA求解器
    CBPA_solver = CBPA(config_data)
    t_start = time.time()

    # 任务分配
    path_list, times_list, RobotList, TaskList = CBPA_solver.solve(RobotList, TaskList, WorldInfoTest, max_depth, bid_times, time_window_flag=True)

    t_end = time.time()
    t_used = t_end - t_start导入上一级文件夹的包
    print("分配用时 (秒): ", t_used)
    # 打印任务包、路径、时间、赢家、分数、出价、赢家出价信息
    print("bundle_list")
    print(CBPA_solver.bundle_list)
    print("path_list")
    print(path_list)
    print("robot_times_list")
    print(times_list)
    print("winners_list")
    print(CBPA_solver.winners_list)
    # print("scores_list")
    # print(CBPA_solver.scores_list)
    print("bid_list")
    print(CBPA_solver.bid_list)
    # print("winner_bid_list")
    # print(CBPA_solver.winner_bid_list)
    print("task_times_list")
    print(CBPA_solver.task_times_list)
    sum_task_time = sum(CBPA_solver.task_times_list[0])
    print("sum_task_time  &  average_task_time")
    print(sum_task_time, sum_task_time/num_tasks)
    print("winner_recload_list")
    print(CBPA_solver.winner_recload_list)
    print("winner_strload_list")
    print(CBPA_solver.winner_strload_list)
    print("self.recload_list")
    print(CBPA_solver.recload_list)
    print("self.strload_list")
    print(CBPA_solver.strload_list)

    # 绘图
    CBPA_solver.plot_assignment()
    plt.show()
