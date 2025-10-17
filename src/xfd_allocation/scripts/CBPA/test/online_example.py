#!/usr/bin/env python3
import time
import json
import matplotlib.pyplot as plt
from CBPA import CBPA
from WorldInfo import WorldInfo
import HelperLibrary as HelperLibrary

if __name__ == "__main__":
    # json 配置文件
    config_file_name = "config_example_01.json"
    
    # 读取json文件中的配置信息
    json_file = open(config_file_name)
    config_data = json.load(json_file)
    # 创建一个世界，每个列表都是[x,y,z]轴的[min, max]坐标
    WorldInfoTest = WorldInfo([0.0, 2420.0], [0.0, 1380.0], [0.0, 0.0])

    # 创建机器人和任务列表
    num_robots = 6
    num_tasks = 10
    max_depth = 5   #任务包深度 exp1: cbpa=4 paimai=5
    bid_times = 3*num_tasks
    RobotList, TaskList = HelperLibrary.create_robots_and_tasks_exp2_4(num_robots, num_tasks, WorldInfoTest, config_data)
    # 创建CBPA求解器
    CBPA_solver = CBPA(config_data)
    while True:

        RobotList, TaskList = HelperLibrary.create_robots_and_tasks_exp2_4(num_robots, num_tasks, WorldInfoTest, config_data)
        # 执行任务分配
        path_list, times_list, RobotList, TaskList = CBPA_solver.solve(RobotList, TaskList, WorldInfoTest, max_depth, bid_times, time_window_flag=True)

        # 更新图表
        CBPA_solver.plot_robot_schedules()

        # 可选的等待时间，或者其他更新RobotList和TaskList的操作
        time.sleep(3)
    # plt.show()