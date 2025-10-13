#!/usr/bin/env python3
import sys
import math
import copy
import random
import heapq
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from mpl_toolkits.mplot3d import Axes3D
from CBPA.lib.Task import Task
from CBPA.lib.WorldInfo import WorldInfo
from scipy.optimize import linprog

# import HelperLibrary as HelperLibrary


class CBPA_REC(object):
    num_robots: int  # 机器人数量
    num_tasks: int  # 任务数量
    robot_types: list  # 记录每个机器人的类型
    task_types: list
    RobotList: list  # 一维列表，每个条目是一个数据类Robot
    TaskList: list
    max_depth: int  # 机器人任务包深度
    time_window_flag: bool  # 时间窗口标志
    duration_flag: bool  # 任务持续时间大于0时为真

    path_list: list  # 2D list    路径包：每一行是机器人任务执行顺序图
    MyWorldInfo: WorldInfo  # 世界信息

    def __init__(self):
        """
        初始化：记录每个机器的类型、任务数量、总的时间窗口、智能体与任务初始匹配矩阵
        """
        self.RobotList = []
        self.TaskList = []

    def settings_centralized(self, RobotList: list, TaskList: list, WorldInfoInput: WorldInfo):
        """
        初始化：机器人、任务、世界信息、机器人数量、任务数量、任务包深度等信息
        """
        self.RobotList = RobotList
        self.TaskList = TaskList
        self.num_robots = len(self.RobotList)
        self.num_tasks = len(self.TaskList)

        # 世界信息
        self.MyWorldInfo = WorldInfoInput
        self.space_limit_x = self.MyWorldInfo.limit_x
        self.space_limit_y = self.MyWorldInfo.limit_y
        self.space_limit_z = self.MyWorldInfo.limit_z

    def scoring_compute_score_cenralized(self, idx_robot: int, task_current: Task):
        """
        Compute marginal score of doing a task
        """
        # no time window for tasks
        dt_current = (
            math.sqrt(
                (self.RobotList[idx_robot].x - task_current.x) ** 2
                + (self.RobotList[idx_robot].y - task_current.y) ** 2
                + (self.RobotList[idx_robot].z - task_current.z) ** 2
            )
            / self.RobotList[idx_robot].nom_velocity
        )

        reward = task_current.task_value * math.exp((-task_current.discount) * dt_current)

        score = reward
        # raise Exception("Unknown robot type!")

        return score

    def solve_centralized(self, RobotList: list, TaskList: list, WorldInfoInput: WorldInfo):
        """
        主函数: CBPA_REC任务分配算法
        """
        # 初始化基本信息
        self.settings_centralized(RobotList, TaskList, WorldInfoInput)

        # 任务的分配矩阵、机器人出价、当前最小出价
        allocation_mat = [[-1] * 10 for _ in range(self.num_tasks)]
        idx_mat = [[-1] * 10 for _ in range(self.num_tasks)]

        score_sum = 0
        score_num = 0
        # CBPA_REC主循环,遍历每个任务
        for task_idx in range(self.num_tasks):
            # 初始化一个空堆
            min_heap = []
            max_bid_id = []
            # 判断任务是否未分配
            if self.TaskList[task_idx].task_status:
                for robot_idx in range(self.num_robots):
                    # 判断机器人状态是否正常
                    if self.RobotList[robot_idx].robot_status:
                        robot_bid = self.scoring_compute_score_cenralized(robot_idx, self.TaskList[task_idx])
                        # 当堆中的元素少于打击需求，直接添加新元素
                        if len(min_heap) < self.TaskList[task_idx].robot_need:
                            heapq.heappush(min_heap, (robot_bid, self.RobotList[robot_idx].robot_id, robot_idx))
                        else:
                            # 如果新元素比堆顶元素大，则替换堆顶元素
                            if robot_bid > min_heap[0][0]:
                                heapq.heapreplace(min_heap, (robot_bid, self.RobotList[robot_idx].robot_id, robot_idx))
                max_bid_id = sorted(min_heap, key=lambda x: x[0], reverse=True)

                allocation_mat[task_idx][0] = self.TaskList[task_idx].task_id
                idx_mat[task_idx][0] = task_idx
                for i in range(len(max_bid_id)):
                    score_num += 1
                    score_sum = score_sum + max_bid_id[i][0]
                    allocation_mat[task_idx][i + 1] = max_bid_id[i][1]
                    idx = max_bid_id[i][2]
                    idx_mat[task_idx][i + 1] = max_bid_id[i][2]
                    self.RobotList[idx].robot_status = False
                if len(idx_mat[task_idx]) > self.TaskList[task_idx].robot_need:
                    self.TaskList[task_idx].task_status = False
        average_score = score_sum / score_num

        # output the result path for each robot, delete all -1
        allocation_mat = [list(filter(lambda a: a != -1, allocation_mat[i])) for i in range(len(allocation_mat))]
        return allocation_mat, idx_mat, self.RobotList, self.TaskList, average_score

    def min_nonnegative_index(self, arr: list):
        arr = np.array(arr)
        nonnegative_vals = arr[arr >= 0]
        if len(nonnegative_vals) == 0:
            return -1
        min_nonnegative_index = np.argmin(nonnegative_vals)
        # 获取最小非负数在原数组中的索引
        indices = np.where(arr >= 0)[0]
        return indices[min_nonnegative_index]

    def min_positive(self, lst: list):
        positive_with_index = [(index, num) for index, num in enumerate(lst) if num > 0]
        if positive_with_index:
            return min(positive_with_index, key=lambda x: x[1])
        else:
            return [-1, -1]

    def max_positive(self, lst: list):
        positive_with_index = [(index, num) for index, num in enumerate(lst) if num > 0]
        if positive_with_index:
            return max(positive_with_index, key=lambda x: x[1])
        else:
            return [-1, -1]

    def min_non_negative(self, lst: list):
        positive_with_index = [(index, num) for index, num in enumerate(lst) if num >= 0]
        if positive_with_index:
            return min(positive_with_index, key=lambda x: x[1])
        else:
            return [-1, -1]

    def sum_of_non_negative(self, lst: list):
        non_negative = [num for num in lst if num >= 0]
        return sum(non_negative)

    def plot_assignment_XFD(self, path_list, idx_list):
        """
        Plots CBPA_REC outputs when there is time window for tasks.
        """

        # 3D plot
        fig_3d = plt.figure(1)  # 创建一个图形对象
        ax_3d = fig_3d.add_subplot(111, projection="3d")  # 创建一个3D子图对象ax_3d，并将投影类型设置为’3d’
        # offset to plot text in 3D space
        offset = (self.MyWorldInfo.limit_x[1] - self.MyWorldInfo.limit_x[0]) / 50  # 坐标单位长度

        # 绘制机器人
        color_list = []
        for n in range(self.num_robots):
            color_list.append(mcolors.to_hex([random.random(), random.random(), random.random()]))
            ax_3d.scatter(
                self.RobotList[n].x, self.RobotList[n].y, self.RobotList[n].z, marker="o", color=color_list[n]
            )
            ax_3d.text(self.RobotList[n].x + offset, self.RobotList[n].y + offset, self.RobotList[n].z, "R" + str(n))

        # 绘制任务
        # color_str = "yellow"
        color_str = "#4B57A2"
        for m in range(self.num_tasks):
            # ax_3d.scatter([self.TaskList[m].x]*2, [self.TaskList[m].y]*2,
            #               [self.TaskList[m].start_time, self.TaskList[m].end_time], marker='^', s=40, color=color_str) #任务开始和结束的散点
            ax_3d.scatter(
                self.TaskList[m].x, self.TaskList[m].y, self.TaskList[m].z, marker="^", s=30, color=color_str
            )  # 任务散点
            # ax_3d.plot3D([self.TaskList[m].x]*2, [self.TaskList[m].y]*2,[self.task_times_list[0][m],
            #                     self.task_times_list[0][m]+self.TaskList[m].duration],linestyle=':', color=color_str, linewidth=4) #任务开始和结束用虚线连接
            ax_3d.text(self.TaskList[m].x + offset, self.TaskList[m].y + offset, self.TaskList[m].z, "T" + str(m))
            # if the path is not empty
            if path_list[m]:
                for j in range(len(path_list[m]) - 1):
                    idx = idx_list[m][j + 1]
                    ax_3d.plot3D(
                        [self.RobotList[idx].x, self.TaskList[m].x],
                        [self.RobotList[idx].y, self.TaskList[m].y],
                        [self.RobotList[idx].z, self.TaskList[m].z],
                        linewidth=2,
                        color=color_list[idx],
                    )  # 绘制从Robot位置到第一个任务位置的路径

        plt.title("Robot Paths")
        ax_3d.set_xlabel("X")
        ax_3d.set_ylabel("Y")
        ax_3d.set_zlabel("Z")
        ax_3d.set_aspect("auto")

        # set legends
        colors = ["#7FABD1", "#F39865", "#963B79"]

        marker_list = ["solid", "solid", "solid"]
        labels = ["Reconnaissance robot", "Strike robot", "Reconnaissance & Strike robot"]

        def f(marker_type, color_type):
            return plt.plot([], [], linestyle=marker_type, color=color_type)[0]

        handles = [f(marker_list[i], colors[i]) for i in range(len(labels))]
        plt.legend(handles, labels, bbox_to_anchor=(1, 1), loc="upper left", framealpha=1)
        self.set_axes_equal_xy(ax_3d, flag_3d=True)

        plt.show(block=False)

    def set_axes_equal_xy(self, ax, flag_3d: bool):
        """
        Make only x and y axes of 3D plot have equal scale. This is one possible solution to Matplotlib's
        ax.set_aspect('equal') and ax.axis('equal') not working for 3D.
        Reference: https://stackoverflow.com/questions/13685386/matplotlib-equal-unit-length-with-equal-aspect-ratio-z-axis-is-not-equal-to

        Input
        ax: a matplotlib axis, e.g., as output from plt.gca().
        flag_3d: boolean, True if it's 3D plot.
        """

        x_limits = self.space_limit_x
        y_limits = self.space_limit_y

        x_range = abs(x_limits[1] - x_limits[0])
        x_middle = np.mean(x_limits)
        y_range = abs(y_limits[1] - y_limits[0])
        y_middle = np.mean(y_limits)

        # The plot bounding box is a sphere in the sense of the infinity
        # norm, hence I call half the max range the plot radius.
        plot_radius = 0.5 * max([x_range, y_range])

        if flag_3d:
            ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
            ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])

        else:
            ax.set_xlim([x_middle - plot_radius, x_middle + plot_radius])
            ax.set_ylim([y_middle - plot_radius, y_middle + plot_radius])
