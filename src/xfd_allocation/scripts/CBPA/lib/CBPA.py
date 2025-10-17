#!/usr/bin/env python3
import sys
import math
import copy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from CBPA.lib.Task import Task
from CBPA.lib.WorldInfo import WorldInfo
from scipy.optimize import linprog
#import HelperLibrary as HelperLibrary 

 
class CBPA(object):
    num_robots: int  # 机器人数量
    num_tasks: int  # 任务数量
    robot_types: list #记录每个机器人的类型
    task_types: list
    RobotList: list  # 一维列表，每个条目是一个数据类Robot
    TaskList: list  
    max_depth: int  # 机器人任务包深度
    time_window_flag: bool  # 时间窗口标志
    duration_flag: bool  # 任务持续时间大于0时为真
    space_limit_x: list  # x坐标范围 [min, max]（米）
    space_limit_y: list  # y坐标范围 [min, max]（米）
    space_limit_z: list  # z坐标范围 [min, max]（米）
    time_interval_list: list  # 总的时间窗口
    robot_index_list: list  # 1D，机器人的id, 从0开始 

    # [num_robots, num_robots]
    recload_list: list  # 2D list 载荷包：每个机器人视角下的侦察载荷剩余量
    strload_list: list  # 2D list 载荷包：每个机器人视角下的打击载荷剩余量
    

    # [num_robot, max_depth]
    bundle_list: list  # 2D list  任务包：每一行是机器人任务进包先后
    path_list: list  # 2D list    路径包：每一行是机器人任务执行顺序
    robot_times_list: list  #  list   时间包：
    scores_list: list  # 2D list  每个机器人所执行任务的分数

    # [num_robot, num_task]
    task_times_list: list  # 2D list   每个任务的时间
    bid_list: list  # 2D list  记录对每个机器人对每个任务的出价 [num_robot, num_task]
    bids_num_list:list  # 2D list 出价次数限制

    # [num_robot, num_task, num_robot+1]
    winners_list: list  # 3D list
    winner_bid_list: list  # 3D list  每个机器人对每个任务的出价 最后一项为每个任务的最低出价
    winner_recload_list: list  # 3D list 每个机器人对每个任务的侦察载荷 最后一项为每个任务的载荷差
    winner_strload_list: list  # 3D list 每个机器人对每个任务的打击载荷 最后一项为每个任务的载荷差
    graph: list  # 2D list  全连接图
    MyWorldInfo: WorldInfo  # 世界信息

    def __init__(self, config_data: dict):
        """
        初始化：记录每个机器的类型、任务数量、总的时间窗口、智能体与任务初始匹配矩阵
        """
        self.robot_types = config_data["ROBOT_TYPES"]
        self.task_types = config_data["TASK_TYPES"]

        self.time_interval_list = [int(config_data["TASK_DEFAULT"]["START_TIME"]), int(config_data["TASK_DEFAULT"]["END_TIME"])]
        self.duration_flag = (int(config_data["TASK_DEFAULT"]["DURATION"]) > 0)

    def settings(self, RobotList: list, TaskList: list, WorldInfoInput: WorldInfo, max_depth: int, bid_times: int, time_window_flag: bool):
        """
        初始化：机器人、任务、世界信息、机器人数量、任务数量、任务包深度等信息
        """
        self.RobotList = RobotList
        self.TaskList = TaskList
        self.num_robots = len(self.RobotList)
        self.num_tasks = len(self.TaskList)
        self.max_depth = max_depth
        self.time_window_flag = time_window_flag

        # 世界信息
        self.MyWorldInfo = WorldInfoInput
        self.space_limit_x = self.MyWorldInfo.limit_x 
        self.space_limit_y = self.MyWorldInfo.limit_y
        self.space_limit_z = self.MyWorldInfo.limit_z

        # 全连接图
        self.graph = np.logical_not(np.identity(self.num_robots)).tolist()

        # 初始化机器人属性[智能体数量，任务包深度]
        self.bundle_list = [[-1] * self.max_depth for _ in range(self.num_robots)]  #任务包
        self.path_list = [[-1] * self.max_depth for _ in range(self.num_robots)]
        self.robot_times_list = [[-1] * self.max_depth for _ in range(self.num_robots)] #机器人到达任务的时间
        self.scores_list = [[-1] * self.max_depth for _ in range(self.num_robots)]
        
        # 初始化获胜者列表，获胜者出价列表，获胜者消耗载荷列表，任务时间列表，机器人剩余载荷列表
        self.winners_list = np.array([[[0 for _ in range(self.num_robots)] for _ in range(self.num_tasks)] for _ in range(self.num_robots)])    #获胜联盟矩阵
        self.winner_bid_list = np.array([[[-1.0 for _ in range(self.num_robots+1)] for _ in range(self.num_tasks)] for _ in range(self.num_robots)])   #获胜联盟投标值矩阵
        self.winner_recload_list = np.array([[[0 for _ in range(self.num_robots+1)] for _ in range(self.num_tasks)] for _ in range(self.num_robots)])  #获胜联盟侦察载荷矩阵
        self.winner_strload_list = np.array([[[0 for _ in range(self.num_robots+1)] for _ in range(self.num_tasks)] for _ in range(self.num_robots)])  #获胜联盟打击载荷矩阵

        # [机器人数量，任务数量]
        self.task_times_list = [[0] * self.num_tasks for _ in range(self.num_robots)]  #任务时间列表
        self.bid_list = [[-1.0] * self.num_tasks for _ in range(self.num_robots)] #记录对每个机器人对每个任务的出价
        self.bids_num_list = np.array([[bid_times] * self.num_tasks for _ in range(self.num_robots)]) # 记录每个机器人对每个任务的出价次数
        
        self.recload_list = np.array([[0] * self.num_robots for _ in range(self.num_robots)]) #侦察剩余载荷列表
        self.strload_list = np.array([[0] * self.num_robots for _ in range(self.num_robots)]) #打击剩余载荷列表
        

        self.robot_index_list = []
        sum_of_robot_recload = 0
        sum_of_robot_strload = 0
        sum_of_task_recneed = 0
        sum_of_task_strneed = 0
        for i in range(0,len(RobotList)):
            for j in range (0,len(RobotList)):
                self.recload_list[i][j] = RobotList[j].rec_capability
                self.strload_list[i][j] = RobotList[j].str_capability 
                self.robot_index_list.append(self.RobotList[i].robot_id) #机器人的id, 从0开始
                sum_of_robot_recload += RobotList[j].rec_capability
                sum_of_robot_strload += RobotList[j].str_capability
            for n in range(self.num_tasks):
                self.winner_recload_list[i][n][self.num_robots] = self.TaskList[n].rec_need
                self.winner_strload_list[i][n][self.num_robots] = self.TaskList[n].str_need
                self.winner_bid_list[i][n][self.num_robots] = self.TaskList[n].end_time - self.TaskList[n].duration
                self.task_times_list[i][n] = self.TaskList[n].end_time - self.TaskList[n].duration
                sum_of_task_recneed += self.TaskList[n].rec_need
                sum_of_task_strneed += self.TaskList[n].str_need
            #if i == 0:
                #print("侦察载荷：Robot ", sum_of_robot_recload, "Task ", sum_of_task_recneed, "\n打击载荷：Robot ", sum_of_robot_strload, "Task ", sum_of_task_strneed)
    def solve(self, RobotList: list, TaskList: list, WorldInfoInput: WorldInfo, max_depth: int, bid_times: int, time_window_flag: bool):
        """
        主函数: CBPA任务分配算法
        """
        # 初始化基本信息
        self.settings(RobotList, TaskList, WorldInfoInput, max_depth, bid_times, time_window_flag)

        # 初始化迭代变量
        # 当前迭代次数
        iter_idx = 1
        # 机器人之间通讯的时间矩阵
        time_mat = [[0] * self.num_robots for _ in range(self.num_robots)] 
        iter_prev = 0
        done_flag = False

        # CBPA主循环 (循环至收敛)
        while not done_flag:

            # 1. 共识阶段
            # 就获胜者、出价、载荷消耗、时间进行共识
            time_mat = self.communicate(time_mat, iter_idx)

            # 2. 任务包构建阶段
            # 在每个机器人上运行CBPA算法 (分散但同步)
            for idx_robot in range(self.num_robots):
                new_bid_flag = self.bundle(idx_robot)
                #print(" ")
                # 如果有新的出价，更新迭代次数
                if new_bid_flag:
                    iter_prev = iter_idx

            # 3. 收敛性判定
            # 确定分配是否结束
            if (iter_idx - iter_prev) > self.num_robots:
                done_flag = True
            elif (iter_idx - iter_prev) > (2*self.num_robots):
                #print("因共识问题，算法未收敛")
                done_flag = True
            else:
                # 主循环继续
                iter_idx += 1
                #print("迭代次数：", iter_idx)
        # #print("task_time_list:", self.task_times_list)
        # #print("winner_bid_list_end", self.winner_bid_list[:, :, self.num_robots])

        # 将路径和任务包映射到实际任务id (实际和算法中相同)
        for n in range(self.num_robots):
            for m in range(self.max_depth):
                if self.bundle_list[n][m] == -1:
                    break
                else:
                    self.bundle_list[n][m] = self.TaskList[self.bundle_list[n][m]].task_id

                if self.path_list[n][m] == -1:
                    break
                else:
                    self.path_list[n][m] = self.TaskList[self.path_list[n][m]].task_id

        # 计算总收益
        score_total = 0
        for n in range(self.num_robots):
            for m in range(self.max_depth):
                if self.scores_list[n][m] > -1:
                    score_total += self.scores_list[n][m]
                else:
                    break

        # output the result path for each robot, delete all -1
        self.path_list = [list(filter(lambda a: a != -1, self.path_list[i]))
                          for i in range(len(self.path_list))]

        # delete redundant elements
        self.bundle_list = [list(filter(lambda a: a != -1, self.bundle_list[i]))
                            for i in range(len(self.bundle_list))]

        self.robot_times_list = [list(filter(lambda a: a != -1, self.robot_times_list[i]))
                           for i in range(len(self.robot_times_list))]

        self.scores_list = [list(filter(lambda a: a != -1, self.scores_list[i]))
                            for i in range(len(self.scores_list))]
        
        robotNum = [0]*self.num_tasks
        for j in range(self.num_tasks):
            # column_j = [row[j] for row in self.winners_list[0]]
            # #print(column_j)
            robotNum[j] = self.sum_of_non_negative(self.winners_list[0][j][:])

        return self.path_list, self.robot_times_list, self.task_times_list, robotNum, self.winner_strload_list[0], self.RobotList, self.TaskList

    def bundle(self, idx_robot: int):
        """
        任务包构建和更新 (在每个机器人上单独运行)
        """
        
        # 删除失价拍卖的包
        self.bundle_remove(idx_robot)
        # 对新任务出价并添加到任务包中
        new_bid_flag = self.bundle_add(idx_robot) 

        return new_bid_flag 

    def bundle_remove(self, idx_robot: int):
        """
        共识之后，释放任务包中的失价任务
        对于每个机器人，如果它的任务包中的任务被其他机器人获胜，则释放该任务
        """
        out_bid_for_task = False
        for idx in range(self.max_depth):
            # If bundle(j) < 0, it means that all tasks up to task j are
            # still valid and in paths, the rest (j to MAX_DEPTH) are released
            if self.bundle_list[idx_robot][idx] < 0:
                break
            else:
                # Test if robot has been outbid
                #  for a task.  If it has, release it and all subsequent tasks in its path.
                if self.winner_bid_list[idx_robot][self.bundle_list[idx_robot][idx]][idx_robot] == -1:
                    out_bid_for_task = True
                if out_bid_for_task:
                    path_current = copy.deepcopy(self.path_list[idx_robot])
                    idx_remove = path_current.index(self.bundle_list[idx_robot][idx])
                    # remove item from list at location specified by idx_remove, then append -1 at the end.
                    del self.path_list[idx_robot][idx_remove]
                    self.path_list[idx_robot].append(-1)
                    del self.robot_times_list[idx_robot][idx_remove]
                    self.robot_times_list[idx_robot].append(-1)
                    del self.scores_list[idx_robot][idx_remove]
                    self.scores_list[idx_robot].append(-1)
                    self.bundle_list[idx_robot][idx] = -1

    def bundle_add(self, idx_robot: int):
        """
        Create bundles for each robot
        """
        epsilon = 1e-5  #用于判断浮点数相等性
        best_find_flag = False
        new_bid_flag = False
        # 检查机器人任务包是否已满，如果未满则为false继续出价
        index_array = np.where(np.array(self.bundle_list[idx_robot]) == -1)[0] #找到智能体i的任务包为-1的索引并返回数组
        if len(index_array) > 0:
            bundle_full_flag = False
        else:
            bundle_full_flag = True
            #print("robot", idx_robot, "任务包已满")
        # 机器人剩余载荷
        for ri in range(self.num_robots):
            self.recload_list[idx_robot][ri] = self.RobotList[ri].rec_capability
            self.strload_list[idx_robot][ri] = self.RobotList[ri].str_capability
            for ti in range(self.num_tasks):
                #self.recload_list[idx_robot][ri] -= np.array(self.winner_recload_list[idx_robot][ti][ri])  # 机器人剩余侦察载荷
                self.strload_list[idx_robot][ri] -= np.array(self.winner_strload_list[idx_robot][ti][ri])  # 机器人剩余打击载荷
        # 检查机器人载荷是否还有剩余，如果没有则为true停止出价
        if self.recload_list[idx_robot][idx_robot] <= epsilon and self.strload_list[idx_robot][idx_robot] <= epsilon:
            robot_full_flag = True
            #print("robot", idx_robot, "载荷已满")
        else:
            robot_full_flag = False

        # 初始化可行性矩阵，确定任务包哪个位置可以插入任务
        # feasibility = np.ones((self.num_tasks, self.max_depth+1))
        feasibility = [[1] * (self.max_depth+1) for _ in range(self.num_tasks)]  #可行性矩阵 可行为1 

        # while not ( bundle_full_flag or robot_full_flag or best_find_flag):   #纯基于拍卖的方法
        while not (bundle_full_flag or robot_full_flag):
            
            # Update task values based on current assignment
            [best_indices, task_times, feasibility] = self.compute_bid(idx_robot, feasibility) #返回每个任务最佳插入索引、任务所需时间和更新后的可行性矩阵，函数还会更新bid_list
            
            # # 找到最佳任务索引
            bid_array = np.array(self.bid_list[idx_robot])  #当前机器人对每个任务的出价
            
            best_find_flag = False  #当前机器人分配了新任务的标志
            if len(np.where(bid_array > 0)[0]) == 0:
                #print("robot", idx_robot, "没有可执行任务break")
                break
            while not best_find_flag:
                #print("进入best_find循环, 构建任务包, 机器人", idx_robot)
                # #print("bid_array:", bid_array)
                
                #best_task = np.where(np.array(bid_array) >= 0)[0].min() if np.any(np.array(bid_array) >= 0) else -1  #找到bid_array中最小非负数的索引
                best_task = self.min_positive(bid_array)[0]  #找到bid_array中最小非负数的索引
                #print("best_task:", best_task, "best_indices:", best_indices, "task_times:", task_times)
                #best_task = self.min_nonnegative_index(bid_array)  #找到bid_array中最小非负数的索引

                # 寻找最佳任务
                value_max = bid_array[best_task]
                all_values = np.where(bid_array == value_max)[0] #找到所有出价等于最高值的任务索引
                if len(all_values) == 1:
                    best_task = all_values[0]
                else:
                    # Tie-break by which task starts first #根据任务的开始时间进行决策
                    #print("*******有相同出价")
                    earliest = sys.float_info.max  #初始化为最大的浮点数
                    for i in range(len(all_values)):
                        if self.TaskList[all_values[i]].start_time < earliest:  #最早开始的任务优先
                            earliest = self.TaskList[all_values[i]].start_time
                            best_task = all_values[i]
                best_time = task_times[best_task]  #当前任务的预计时间
                
                
                load_full = False #初始化任务未满载标志
                if self.winner_recload_list[idx_robot][best_task][self.num_robots] <= epsilon and \
                    self.winner_strload_list[idx_robot][best_task][self.num_robots] <= epsilon:    
                    load_full = True
                if (not load_full) and ((self.winner_recload_list[idx_robot][best_task][self.num_robots] > epsilon and self.recload_list[idx_robot][idx_robot] > 0) or
                                        (self.winner_strload_list[idx_robot][best_task][self.num_robots] > epsilon and self.strload_list[idx_robot][idx_robot] > 0)): #任务未满载
                    best_find_flag = True
                    #print("任务未满载")
                    # 更新获胜者列表、获胜者出价列表、任务分配载荷列表、机器人剩余载荷列表 (任务时间列表保持无穷大)
                    self.winners_list[idx_robot][best_task][idx_robot] = 1  #当前机器人获胜
                    self.winner_bid_list[idx_robot][best_task][idx_robot] = self.bid_list[idx_robot][best_task]  # 记录当前机器人出价
                    self.winner_bid_list[idx_robot][best_task][self.num_robots] = self.max_positive(self.winner_bid_list[idx_robot][best_task][0:self.num_robots])[1]  # 记录最晚时间
                    #self.recload_list[idx_robot] += np.array(self.winner_recload_list[idx_robot][best_task][0:self.num_robots])  # 恢复机器人侦察载荷等待重新分配
                    self.strload_list[idx_robot] += np.array(self.winner_strload_list[idx_robot][best_task][0:self.num_robots])  # 恢复机器人打击载荷等待重新分配
                    robot_recditribute = np.array(self.recload_list[idx_robot]) * np.array(self.winners_list[idx_robot][best_task])  # 当前参与任务的机器人侦察载荷值
                    robot_strditribute = np.array(self.strload_list[idx_robot]) * np.array(self.winners_list[idx_robot][best_task])  # 当前参与任务的机器人打击载荷值
                    recload_distribute = self.distribute_load(robot_recditribute, self.TaskList[best_task].rec_need)  # 分配侦察载荷
                    strload_distribute = self.distribute_load(robot_strditribute, self.TaskList[best_task].str_need)  # 分配打击载荷
                    self.winner_recload_list[idx_robot][best_task][0:self.num_robots] = recload_distribute
                    self.winner_strload_list[idx_robot][best_task][0:self.num_robots] = strload_distribute
                    self.winner_recload_list[idx_robot][best_task][self.num_robots] = self.TaskList[best_task].rec_need - self.sum_of_non_negative(recload_distribute)
                    self.winner_strload_list[idx_robot][best_task][self.num_robots] = self.TaskList[best_task].str_need - self.sum_of_non_negative(strload_distribute) # 获胜载荷包最后一位标志任务满载

                    #self.recload_list[idx_robot] = self.recload_list[idx_robot] - recload_distribute # 更新分配后的机器人侦察载荷

                    self.strload_list[idx_robot] = self.strload_list[idx_robot] - strload_distribute # 更新分配后的机器人打击载荷

                    # if best_time <= self.task_times_list[idx_robot][best_task]:
                    #     self.task_times_list[idx_robot][best_task] = best_time
                    if self.winner_recload_list[idx_robot][best_task][self.num_robots] <= epsilon and \
                    self.winner_strload_list[idx_robot][best_task][self.num_robots] <= epsilon:
                        load_full = True
                    if load_full:
                        #print("**********任务满载，设置时间")
                        self.task_times_list[idx_robot][best_task] = self.winner_bid_list[idx_robot][best_task][self.num_robots]

                    #将最优任务插入到智能体相关列表中, 更新任务路径、时间、 得分、 任务包列表 
                    # self.path_list[idx_robot].insert(best_indices[best_task], best_task)
                    # del self.path_list[idx_robot][-1]
                    # self.robot_times_list[idx_robot].insert(best_indices[best_task], task_times[best_task])
                    # del self.robot_times_list[idx_robot][-1]
                    # self.scores_list[idx_robot].insert(best_indices[best_task], self.bid_list[idx_robot][best_task])
                    # del self.scores_list[idx_robot][-1]

                    # length = len(np.where(np.array(self.bundle_list[idx_robot]) > -1)[0])
                    # self.bundle_list[idx_robot][length] = best_task        
                
                elif load_full:   #任务满载
                    current_winner_array = np.array(self.winners_list[idx_robot][best_task])  #当前任务的获胜者 0
                    current_bid_array = np.array(self.winner_bid_list[idx_robot][best_task][0:self.num_robots])
                    slow_index = current_bid_array.argmax() #找到最慢到达任务的机器人索引
                    fast_index = self.min_positive(current_bid_array)[0] #找到最快到达任务的机器人索引
                    slow_time = max(current_bid_array)
                    fast_time = current_bid_array[fast_index]
                    if best_time < slow_time and slow_time != fast_time:
                        # #print(".....value_max",value_max,".....best_time", best_time,  "....slow_time",slow_time,"......y[nr]",self.winner_bid_list[idx_robot][best_task][self.num_robots])
                    #if best_time < self.winner_bid_list[idx_robot][best_task][self.num_robots]: #当前机器人到达时间小于当前任务预计时间
                        # 任务满载，去掉最慢到达任务的机器人
                        while load_full:
                            #print("进入任务满载循环1，去掉最慢到达任务的机器人", best_time)
                            current_winner_array = np.array(self.winners_list[idx_robot][best_task])  #当前任务的获胜者 0
                            current_bid_array = np.array(self.winner_bid_list[idx_robot][best_task])
                            slow_index = current_bid_array.argmax() #找到最慢到达任务的机器人索引
                            
                            if slow_index != idx_robot:
                                #print("最慢机器人：",slow_index)
                                current_winner_array[slow_index] = 0
                                current_winner_array[idx_robot] = 1
                                #Sum_RecLoad_used = self.sum_of_non_negative(self.winner_recload_list[idx_robot][best_task][0:self.num_robots] * current_winner_array)
                                Sum_StrLoad_used = self.sum_of_non_negative(self.winner_strload_list[idx_robot][best_task][0:self.num_robots] * current_winner_array)
                                Sum_RecLoad = self.sum_of_non_negative(self.recload_list[idx_robot] * current_winner_array)
                                Sum_StrLoad = Sum_StrLoad_used + self.sum_of_non_negative(self.strload_list[idx_robot] * current_winner_array)
                                if (Sum_RecLoad - self.TaskList[best_task].rec_need) >= 0 and (Sum_StrLoad - self.TaskList[best_task].str_need) >= 0:
                                    #print("----进入最慢机器人：SumStr",Sum_StrLoad,"  SumRec",Sum_RecLoad)
                                    # 更新获胜者列表、获胜者出价列表
                                    self.winners_list[idx_robot][best_task][slow_index] = 0  # 最慢机器人失败
                                    self.winners_list[idx_robot][best_task][idx_robot] = 1  # 当前机器人获胜
                                    self.winner_bid_list[idx_robot][best_task][slow_index] = -1 # 最慢机器人出价清零
                                    self.winner_bid_list[idx_robot][best_task][idx_robot] = value_max   # 记录当前机器人出价
                                    self.winner_bid_list[idx_robot][best_task][self.num_robots] = self.max_positive(self.winner_bid_list[idx_robot][best_task][0:self.num_robots])[1]  # 记录最慢时间

                                    #self.recload_list[idx_robot] += np.array(self.winner_recload_list[idx_robot][best_task][0:self.num_robots])  # 恢复机器人侦察载荷等待重新分配
                                    self.strload_list[idx_robot] += np.array(self.winner_strload_list[idx_robot][best_task][0:self.num_robots])  # 恢复机器人打击载荷等待重新分配
                                    robot_recditribute = np.array(self.recload_list[idx_robot]) * np.array(self.winners_list[idx_robot][best_task])  # 当前参与任务的机器人侦察载荷值
                                    robot_strditribute = np.array(self.strload_list[idx_robot]) * np.array(self.winners_list[idx_robot][best_task])  # 当前参与任务的机器人打击载荷值
                                    recload_distribute = self.distribute_load(robot_recditribute, self.TaskList[best_task].rec_need)  # 分配侦察载荷
                                    strload_distribute = self.distribute_load(robot_strditribute, self.TaskList[best_task].str_need)  # 分配打击载荷
                                    self.winner_recload_list[idx_robot][best_task][0:self.num_robots] = recload_distribute
                                    self.winner_strload_list[idx_robot][best_task][0:self.num_robots] = strload_distribute
                                    self.winner_recload_list[idx_robot][best_task][self.num_robots] = self.TaskList[best_task].rec_need - self.sum_of_non_negative(recload_distribute)
                                    self.winner_strload_list[idx_robot][best_task][self.num_robots] = self.TaskList[best_task].str_need - self.sum_of_non_negative(strload_distribute) # 获胜载荷包最后一位标志任务满载
                                    # #print("Task", best_task, "满, 还需载荷: rec ", self.winner_recload_list[idx_robot][best_task][self.num_robots], "str", self.winner_strload_list[idx_robot][best_task][self.num_robots], \
                                        # "获胜者列表", self.winners_list)

                                    #self.recload_list[idx_robot] = self.recload_list[idx_robot] - recload_distribute # 更新分配后的机器人侦察载荷
                                    self.strload_list[idx_robot] = self.strload_list[idx_robot] - strload_distribute # 更新分配后的机器人打击载荷

                                    # 更新任务时间列表
                                    # if best_time <= self.task_times_list[idx_robot][best_task]:
                                    #     self.task_times_list[idx_robot][best_task] = best_time
                                    # else:
                                    #     #print("发生错误！！！满载1，机器人到达时间大于任务最大预计时间")
                                    #     #print("机器人", idx_robot, "任务", best_task, "时间", best_time, "最大时间", self.task_times_list[idx_robot][best_task])
                                    self.task_times_list[idx_robot][best_task] = self.winner_bid_list[idx_robot][best_task][self.num_robots]
                                    #print("----退出最慢机器人：SumStr",self.sum_of_non_negative(strload_distribute),"  SumRec",self.sum_of_non_negative(recload_distribute))
                                    if best_time > self.task_times_list[idx_robot][best_task]:
                                        print("发生错误！！！满载1，机器人到达时间大于任务最大预计时间")
                                        # #print("机器人", idx_robot, "任务", best_task, "时间", best_time, "最大时间", self.task_times_list[idx_robot][best_task])

                                    best_find_flag = True
                                    load_full = True
                                else:
                                    load_full = False                       
                                    #print("退出任务满载循环2:载荷不足")
                            else:
                                load_full = False
                                #print("退出任务满载循环1:索引重复")

                        # 任务满载，减少到达任务的机器人的等待
                        Sum_RecLoad = self.sum_of_non_negative(self.winner_recload_list[idx_robot][best_task][0:self.num_robots])
                        Sum_StrLoad = self.sum_of_non_negative(self.winner_strload_list[idx_robot][best_task][0:self.num_robots])
                        if (Sum_RecLoad - self.TaskList[best_task].rec_need) == 0 and (Sum_StrLoad - self.TaskList[best_task].str_need) == 0:
                            load_full = True
                        else:
                            load_full = False
                            #print("发生错误！！！去掉最快的机器人")
                        while load_full:
                            #print('进入循环2, 去掉最快到达的机器人')
                            current_winner_array = np.array(self.winners_list[idx_robot][best_task])
                            fast_index = self.min_positive(self.winner_bid_list[idx_robot][best_task][0:self.num_robots])[0]
                            fast_time = self.winner_bid_list[idx_robot][best_task][fast_index]
                            if slow_index != idx_robot and best_time > fast_time and fast_index != idx_robot:
                                current_winner_array[fast_index] = 0
                                current_winner_array[idx_robot] = 1
                                #Sum_RecLoad_used = self.sum_of_non_negative(self.winner_recload_list[idx_robot][best_task][0:self.num_robots] * current_winner_array)
                                Sum_StrLoad_used = self.sum_of_non_negative(self.winner_strload_list[idx_robot][best_task][0:self.num_robots] * current_winner_array)
                                Sum_RecLoad = self.sum_of_non_negative(self.recload_list[idx_robot] * current_winner_array)
                                Sum_StrLoad = Sum_StrLoad_used + self.sum_of_non_negative(self.strload_list[idx_robot] * current_winner_array)
                                if (Sum_RecLoad - self.TaskList[best_task].rec_need) >= 0 and (Sum_StrLoad - self.TaskList[best_task].str_need) >= 0:
                                    #print("最快机器人：",fast_index)
                                    # 更新获胜者列表、获胜者出价列表
                                    #print("----进入最快机器人：SumStr",Sum_StrLoad,"  SumRec",Sum_RecLoad)
                                    self.winners_list[idx_robot][best_task][fast_index] = 0    # 最快到达机器人失败
                                    self.winners_list[idx_robot][best_task][idx_robot] = 1     # 当前机器人获胜
                                    self.winner_bid_list[idx_robot][best_task][fast_index] = -1     # 最快到达机器人出价清零
                                    self.winner_bid_list[idx_robot][best_task][idx_robot] = value_max   # 记录当前机器人出价
                                    self.winner_bid_list[idx_robot][best_task][self.num_robots] = self.max_positive(self.winner_bid_list[idx_robot][best_task][0:self.num_robots])[1]  # 记录最快时间
                                    #current_bid_array = np.array(self.winner_bid_list[idx_robot][best_task])
                                    
                                    # 更新任务分配载荷列表、机器人剩余载荷列表
                                    #self.recload_list[idx_robot] += np.array(self.winner_recload_list[idx_robot][best_task][0:self.num_robots])  # 恢复机器人侦察载荷等待重新分配
                                    self.strload_list[idx_robot] += np.array(self.winner_strload_list[idx_robot][best_task][0:self.num_robots])  # 恢复机器人打击载荷等待重新分配
                                    robot_recditribute = np.array(self.recload_list[idx_robot]) * np.array(self.winners_list[idx_robot][best_task])  # 当前参与任务的机器人侦察载荷值
                                    robot_strditribute = np.array(self.strload_list[idx_robot]) * np.array(self.winners_list[idx_robot][best_task])  # 当前参与任务的机器人打击载荷值
                                    recload_distribute = self.distribute_load(robot_recditribute, self.TaskList[best_task].rec_need)  # 分配侦察载荷
                                    strload_distribute = self.distribute_load(robot_strditribute, self.TaskList[best_task].str_need)  # 分配打击载荷
                                    self.winner_recload_list[idx_robot][best_task][0:self.num_robots] = recload_distribute
                                    self.winner_strload_list[idx_robot][best_task][0:self.num_robots] = strload_distribute
                                    self.winner_recload_list[idx_robot][best_task][self.num_robots] = self.TaskList[best_task].rec_need - self.sum_of_non_negative(recload_distribute)
                                    self.winner_strload_list[idx_robot][best_task][self.num_robots] = self.TaskList[best_task].str_need - self.sum_of_non_negative(strload_distribute) # 获胜载荷包最后一位标志任务满载

                                    #self.recload_list[idx_robot] = self.recload_list[idx_robot] - recload_distribute # 更新分配后的机器人侦察载荷
                                    self.strload_list[idx_robot] = self.strload_list[idx_robot] - strload_distribute # 更新分配后的机器人打击载荷

                                    # 更新任务时间列表
                                    self.task_times_list[idx_robot][best_task] = self.winner_bid_list[idx_robot][best_task][self.num_robots]
                                    #print("----退出最快机器人：SumStr",self.sum_of_non_negative(strload_distribute),"  SumRec",self.sum_of_non_negative(recload_distribute))
                                    if best_time > self.task_times_list[idx_robot][best_task]:
                                        print("发生错误！！！满载1，机器人到达时间大于任务最大预计时间")
                                        # #print("机器人", idx_robot, "任务", best_task, "时间", best_time, "最大时间", self.task_times_list[idx_robot][best_task])

                                    best_find_flag = True
                                    load_full = True
                                else:
                                    load_full = False
                                    #print("退出减少机器人等待循环2")
                            else:
                                load_full = False
                                #print("退出减少机器人等待循环1")
                                #current_bid_array = np.array(self.winner_bid_list[idx_robot][best_task])
                        
                        # 任务满载，减少机器人的侦察载荷
                        if self.winner_recload_list[idx_robot][best_task][self.num_robots] == 0 and \
                            self.winner_strload_list[idx_robot][best_task][self.num_robots] == 0:
                            load_full = True
                        else:
                            load_full = False
                            #print("发生错误！！！减少机器人的侦察载荷")
                        while load_full:
                            #print('进入循环3，减少机器人的侦察载荷')
                            current_winner_array = np.array(self.winners_list[idx_robot][best_task])
                            minRecload_index = self.min_positive(self.winner_recload_list[idx_robot][best_task][0:self.num_robots])[0]
                            minRecload = self.recload_list[idx_robot][minRecload_index]
                            bestRecload = self.recload_list[idx_robot][idx_robot]
                            if minRecload_index != idx_robot and bestRecload > minRecload and current_winner_array[idx_robot] == 1:
                                current_winner_array[minRecload_index] = 0
                                current_winner_array[idx_robot] = 1
                                #Sum_RecLoad_used = self.sum_of_non_negative(self.winner_recload_list[idx_robot][best_task][0:self.num_robots] * current_winner_array)
                                Sum_StrLoad_used = self.sum_of_non_negative(self.winner_strload_list[idx_robot][best_task][0:self.num_robots] * current_winner_array)
                                Sum_RecLoad = self.sum_of_non_negative(self.recload_list[idx_robot] * current_winner_array)
                                Sum_StrLoad = Sum_StrLoad_used + self.sum_of_non_negative(self.strload_list[idx_robot] * current_winner_array)
                            
                                if (Sum_RecLoad - self.TaskList[best_task].rec_need) >= 0 and (Sum_StrLoad - self.TaskList[best_task].str_need) >= 0:
                                    # 更新获胜者列表、获胜者出价列表
                                    self.winners_list[idx_robot][best_task][minRecload_index] = 0    # 最快到达机器人失败
                                    self.winners_list[idx_robot][best_task][idx_robot] = 1     # 当前机器人获胜
                                    self.winner_bid_list[idx_robot][best_task][minRecload_index] = -1     # 最快到达机器人出价清零
                                    self.winner_bid_list[idx_robot][best_task][idx_robot] = value_max   # 记录当前机器人出价
                                    self.winner_bid_list[idx_robot][best_task][self.num_robots] = self.max_positive(self.winner_bid_list[idx_robot][best_task][0:self.num_robots])[1]  # 记录最慢时间

                                    # 更新任务分配载荷列表、机器人剩余载荷列表
                                    #self.recload_list[idx_robot] += np.array(self.winner_recload_list[idx_robot][best_task][0:self.num_robots])  # 恢复机器人侦察载荷等待重新分配
                                    self.strload_list[idx_robot] += np.array(self.winner_strload_list[idx_robot][best_task][0:self.num_robots])  # 恢复机器人打击载荷等待重新分配
                                    robot_recditribute = np.array(self.recload_list[idx_robot]) * np.array(self.winners_list[idx_robot][best_task])  # 当前参与任务的机器人侦察载荷值
                                    robot_strditribute = np.array(self.strload_list[idx_robot]) * np.array(self.winners_list[idx_robot][best_task])  # 当前参与任务的机器人打击载荷值
                                    recload_distribute = self.distribute_load(robot_recditribute, self.TaskList[best_task].rec_need)  # 分配侦察载荷
                                    strload_distribute = self.distribute_load(robot_strditribute, self.TaskList[best_task].str_need)  # 分配打击载荷
                                    self.winner_recload_list[idx_robot][best_task][0:self.num_robots] = recload_distribute
                                    self.winner_strload_list[idx_robot][best_task][0:self.num_robots] = strload_distribute
                                    self.winner_recload_list[idx_robot][best_task][self.num_robots] = self.TaskList[best_task].rec_need - self.sum_of_non_negative(recload_distribute)
                                    self.winner_strload_list[idx_robot][best_task][self.num_robots] = self.TaskList[best_task].str_need - self.sum_of_non_negative(strload_distribute) # 获胜载荷包最后一位标志任务满载

                                    #self.recload_list[idx_robot] = self.recload_list[idx_robot] - recload_distribute # 更新分配后的机器人侦察载荷
                                    self.strload_list[idx_robot] = self.strload_list[idx_robot] - strload_distribute # 更新分配后的机器人打击载荷

                                    #更新任务时间列表
                                    self.task_times_list[idx_robot][best_task] = self.winner_bid_list[idx_robot][best_task][self.num_robots]
                                    if best_time > self.task_times_list[idx_robot][best_task]:
                                        print("发生错误！！！满载1，机器人到达时间大于任务最大预计时间")
                                        # #print("机器人", idx_robot, "任务", best_task, "时间", best_time, "最大时间", self.task_times_list[idx_robot][best_task])

                                    best_find_flag = True
                                    load_full = True
                                    # Sum_RecLoad = Sum_RecLoad_Temp
                                    # Sum_StrLoad = Sum_StrLoad_Temp
                                else:
                                    #print("退出减少机器人侦察载荷循环2")
                                    load_full = False
                            else:       
                                load_full = False
                                #print("退出减少机器人侦察载荷循环1")
                                break
                        # 任务满载，减少机器人的打击载荷
                        if self.winner_recload_list[idx_robot][best_task][self.num_robots] >= 0 and \
                            self.winner_strload_list[idx_robot][best_task][self.num_robots] >= 0:
                            load_full = True
                        else:
                            load_full = False
                            #print("发生错误！！！减少机器人的打击载荷")
                        while load_full:
                            #print('进入循环4，减少机器人的打击载荷')
                            #current_strload_array = np.array(self.winner_strload_list[idx_robot][best_task])  #当前任务每个机器人投入的侦察载荷
                            current_winner_array = np.array(self.winners_list[idx_robot][best_task])
                            minStrload_index = self.min_positive(self.winner_strload_list[idx_robot][best_task][0:self.num_robots])[0]
                            minStrload = self.winner_strload_list[idx_robot][best_task][minStrload_index] + self.strload_list[idx_robot][minStrload_index]
                            bestStrload = self.winner_strload_list[idx_robot][best_task][idx_robot] + self.strload_list[idx_robot][idx_robot]
                            if minStrload_index != idx_robot and bestStrload > minStrload and current_winner_array[idx_robot] == 1:
                                current_winner_array[minStrload_index] = 0
                                current_winner_array[idx_robot] = 1
                                #Sum_RecLoad_used = self.sum_of_non_negative(self.winner_recload_list[idx_robot][best_task][0:self.num_robots] * current_winner_array)
                                Sum_StrLoad_used = self.sum_of_non_negative(self.winner_strload_list[idx_robot][best_task][0:self.num_robots] * current_winner_array)
                                Sum_RecLoad = self.sum_of_non_negative(self.recload_list[idx_robot] * current_winner_array)
                                Sum_StrLoad = Sum_StrLoad_used + self.sum_of_non_negative(self.strload_list[idx_robot] * current_winner_array)
                                #Sum_StrLoad_Temp = Sum_StrLoad_Temp - self.strload_list[idx_robot][minStrload_index]
                                if (Sum_RecLoad - self.TaskList[best_task].rec_need) >= 0 and (Sum_StrLoad - self.TaskList[best_task].str_need) >= 0:
                                    # 更新获胜者列表、获胜者出价列表
                                    self.winners_list[idx_robot][best_task][minStrload_index] = 0    # 最快到达机器人失败
                                    self.winners_list[idx_robot][best_task][idx_robot] = 1     # 当前机器人获胜
                                    self.winner_bid_list[idx_robot][best_task][minStrload_index] = -1     # 最快到达机器人出价清零
                                    self.winner_bid_list[idx_robot][best_task][idx_robot] = value_max   # 记录当前机器人出价
                                    self.winner_bid_list[idx_robot][best_task][self.num_robots] = self.max_positive(self.winner_bid_list[idx_robot][best_task][0:self.num_robots])[1]  # 记录最快时间
                                    if self.winner_bid_list[idx_robot][best_task][self.num_robots] != best_time:
                                        print("发生错误！！！")
                                    
                                    # 更新任务分配载荷列表、机器人剩余载荷列表
                                    #self.recload_list[idx_robot] += np.array(self.winner_recload_list[idx_robot][best_task][0:self.num_robots])  # 恢复机器人侦察载荷等待重新分配
                                    self.strload_list[idx_robot] += np.array(self.winner_strload_list[idx_robot][best_task][0:self.num_robots])  # 恢复机器人打击载荷等待重新分配
                                    robot_recditribute = np.array(self.recload_list[idx_robot]) * np.array(self.winners_list[idx_robot][best_task])  # 当前参与任务的机器人侦察载荷值
                                    robot_strditribute = np.array(self.strload_list[idx_robot]) * np.array(self.winners_list[idx_robot][best_task])  # 当前参与任务的机器人打击载荷值
                                    recload_distribute = self.distribute_load(robot_recditribute, self.TaskList[best_task].rec_need)  # 分配侦察载荷
                                    strload_distribute = self.distribute_load(robot_strditribute, self.TaskList[best_task].str_need)  # 分配打击载荷
                                    self.winner_recload_list[idx_robot][best_task][0:self.num_robots] = recload_distribute
                                    self.winner_strload_list[idx_robot][best_task][0:self.num_robots] = strload_distribute
                                    self.winner_recload_list[idx_robot][best_task][self.num_robots] = self.TaskList[best_task].rec_need - self.sum_of_non_negative(recload_distribute)
                                    self.winner_strload_list[idx_robot][best_task][self.num_robots] = self.TaskList[best_task].str_need - self.sum_of_non_negative(strload_distribute) # 获胜载荷包最后一位标志任务满载

                                    #self.recload_list[idx_robot] = self.recload_list[idx_robot] - recload_distribute # 更新分配后的机器人侦察载荷
                                    self.strload_list[idx_robot] = self.strload_list[idx_robot] - strload_distribute # 更新分配后的机器人打击载荷

                                    # 更新任务时间列表
                                    self.task_times_list[idx_robot][best_task] = self.winner_bid_list[idx_robot][best_task][self.num_robots]
                                    if best_time > self.task_times_list[idx_robot][best_task]:
                                        print("发生错误!!! 满载1, 机器人到达时间大于任务最大预计时间")
                                        ##print("机器人", idx_robot, "任务", best_task, "时间", best_time, "最大时间", self.task_times_list[idx_robot][best_task])

                                    best_find_flag = True
                                    load_full = True

                                else:
                                    load_full = False
                                    #print("退出减少机器人打击载荷循环2")
                            else:
                                load_full = False
                                #print("退出减少机器人打击载荷循环1")
                                break
                if best_find_flag:   # 更新任务包、路径、得分等     
                # Insert value into list at location specified by index, and delete the last one of original list. 
                    self.path_list[idx_robot].insert(best_indices[best_task], best_task)
                    del self.path_list[idx_robot][-1] 
                    self.robot_times_list[idx_robot].insert(best_indices[best_task], task_times[best_task])
                    del self.robot_times_list[idx_robot][-1]
                    self.scores_list[idx_robot].insert(best_indices[best_task], self.bid_list[idx_robot][best_task])
                    del self.scores_list[idx_robot][-1]
                    length = len(np.where(np.array(self.bundle_list[idx_robot]) > -1)[0])
                    self.bundle_list[idx_robot][length] = best_task
                    new_bid_flag = True

                else :
                    bid_array[best_task] = 0
                    feasibility[best_task][best_indices[best_task]] = 0
                    #print("满载,任务插不进去")
                    if len(np.where(bid_array > epsilon)[0]) == 0:
                        break

                for i in range(self.num_tasks):
                    # Insert value into list at location specified by index, and delete the last one of original list.
                    feasibility[i].insert(best_indices[best_task], feasibility[i][best_indices[best_task]])  
                    del feasibility[i][-1]
                

            #print("退出主循环", idx_robot)
                    
            # Check if bundle is full
            index_array = np.where(np.array(self.bundle_list[idx_robot]) == -1)[0]
            if len(index_array) > 0: #任务包未满
                bundle_full_flag = False
            else:
                bundle_full_flag = True
                #print("机器人", idx_robot, "任务包满了")
            # Check if robot is full
            if self.recload_list[idx_robot][idx_robot] <= epsilon and self.strload_list[idx_robot][idx_robot] <= epsilon:
                robot_full_flag = True
                #print("机器人", idx_robot, "载荷不足了")
            else:
                robot_full_flag = False
            if best_find_flag == False:
                return new_bid_flag
                break
        return new_bid_flag
    
    def min_nonnegative_index(self, arr: list):
        arr = np.array(arr)
        nonnegative_vals = arr[arr >= 0]  
        if len(nonnegative_vals) == 0:
            return -1  
        min_nonnegative_index = np.argmin(nonnegative_vals)
        # 获取最小非负数在原数组中的索引
        indices = np.where(arr >= 0)[0]
        return indices[min_nonnegative_index]

    def distribute_load(self, robot_loads: list, task_load: int):
        robot_loads = np.array(robot_loads)
        total_robot_load = np.sum(robot_loads)
        result = np.array([0] * len(robot_loads))
        if total_robot_load <= task_load:
            result = robot_loads
            return result
        n = len(robot_loads)
        while task_load > 0:
            
            average_load = task_load // n
            if average_load == 0:
                average_load = 1
            for i in range(len(robot_loads)):
                if robot_loads[i] <= average_load and result[i] < robot_loads[i]:
                    result[i] = robot_loads[i]
                    task_load -= robot_loads[i]
                    n -= 1
                elif robot_loads[i]-result[i] >= average_load and result[i]+average_load <= robot_loads[i]:
                    result[i] = result[i]+average_load
                    task_load -= average_load
                    if task_load == 0:
                        break
        return result
    
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

    def communicate(self, time_mat: list, iter_idx: int):
        """
        解决拍卖过程中的任务冲突，更新获胜者列表、获胜者出价列表、任务分配载荷列表、任务时间列表
        机器人i接收机器人k的任务分配信息, 对任务j的信息进行更新
        """ 
        #完全独立的更新时间矩阵副本，由迭代次数定义
        time_mat_new = copy.deepcopy(time_mat) 

        # 复制任务分配信息
        z = copy.deepcopy(self.winners_list)
        y = copy.deepcopy(self.winner_bid_list)
        r = copy.deepcopy(self.winner_recload_list)
        s = copy.deepcopy(self.winner_strload_list)
        t = copy.deepcopy(self.task_times_list)

        epsilon = 10e-6

        Nr = self.num_robots
        Nt = self.num_tasks
        for i in range(Nr):
            for k in range(Nr):
                if self.graph[i][k] == 1:
                    for j in range(Nt):
                        # 发送者k，接收者i，任务j （分布式算法集中写，通讯中的时间戳作用弱化）
                        # 一个侦察机器人和一个打击机器人之间的互斥消除
                        # if ((r[i][j][Nr] == self.TaskList[j].rec_need and r[k][j][Nr] == 0) and (s[i][j][Nr] == 0 and s[k][j][Nr] == self.TaskList[j].str_need)) or\
                        # ((r[i][j][Nr] == 0 and r[k][j][Nr] == self.TaskList[j].rec_need) and (s[i][j][Nr] == self.TaskList[j].str_need and s[k][j][Nr] == 0)):
                        #     z[i][j] = z[i][j] + z[k][j]
                        #     k_best = np.where(z[k][j] == 1)[0]
                        #     for i in range(len(k_best)):
                        #         y[i][j][k_best[i]] = y[k][j][k_best[i]]
                        #         r[i][j][k_best[i]] = r[k][j][k_best[i]]
                        #         s[i][j][k_best[i]] = s[k][j][k_best[i]]
                        #     y[i][j][Nr] = self.max_positive(y[i][j][0:Nr])[1]
                        #     r[i][j][Nr] = 0
                        #     s[i][j][Nr] = 0
                        #     t[i][j] = y[i][j][Nr]    
                        
                        #都认为任务j未满载: 1、载荷差小优先 2、到达时间快优先 3、数量少优先 4、等待时间少优先 5、索引小优先
                        if (r[i][j][Nr] > epsilon or s[i][j][Nr] > epsilon) and (r[k][j][Nr] > epsilon or s[k][j][Nr] > epsilon): # 都认为任务j未满载
                            if (r[i][j][Nr]+s[i][j][Nr] > r[k][j][Nr]+s[k][j][Nr]): # i认为的载荷差量大于k
                                z[i][j], y[i][j], r[i][j], s[i][j], t[i][j] = z[k][j], y[k][j], r[k][j], s[k][j], t[k][j]
                                #print("共识：载荷差量",i,"<---",j,"---",k)
                            elif (r[i][j][Nr]+s[i][j][Nr] == r[k][j][Nr]+s[k][j][Nr]):
                                # min_yi = self.min_positive(y[k][j][0:Nr])[1]
                                # min_yk = self.min_positive(y[k][j][0:Nr])[1]
                                # if (y[i][j][Nr]-min_yi > y[k][j][Nr]-min_yk):   # i认为的任务开始时间与最快机器人的时间差大于k
                                if (y[i][j][Nr] > y[k][j][Nr]):
                                    z[i][j], y[i][j], r[i][j], s[i][j], t[i][j] = z[k][j], y[k][j], r[k][j], s[k][j], t[k][j]
                                    #print("共识：未满载，任务时间",i,"<---",j,"---",k)
                                elif (y[i][j][Nr] == y[k][j][Nr]):    
                                    num_i = np.where(np.array(y[i][j][0:Nr]) != -1)[0].size
                                    num_k = np.where(np.array(y[k][j][0:Nr]) != -1)[0].size
                                    if (num_i > num_k):  # i认为的机器人数量多于k
                                        z[i][j], y[i][j], r[i][j], s[i][j], t[i][j] = z[k][j], y[k][j], r[k][j], s[k][j], t[k][j]
                                        #print("共识：未满载，机器人数量",i,"<---",j,"---",k)
                                    elif (num_i == num_k):
                                            if (i > k): # i的索引大于k
                                                z[i][j], y[i][j], r[i][j], s[i][j], t[i][j] = z[k][j], y[k][j], r[k][j], s[k][j], t[k][j]
                                                #print("共识：索引值",i,"<---",j,"---",k)

                        # 都认为任务j满载: 1、任务时间小优先 2、机器人数量少优先 3、等待时间小优先 4、索引小优先
                        elif (r[i][j][Nr] <= epsilon and s[i][j][Nr] <= epsilon) and (r[k][j][Nr] <= epsilon and s[k][j][Nr] <= epsilon): # 都认为任务j满载
                            if (y[i][j][Nr] > y[k][j][Nr]):
                                z[i][j], y[i][j], r[i][j], s[i][j], t[i][j] = z[k][j], y[k][j], r[k][j], s[k][j], t[k][j]
                                #print("满载共识阶段：时间，机器人i", i, "任务", j, "时间", t[i][j], "机器人k", k, "任务", j, "时间", t[k][j])
                            elif (y[i][j][Nr] == y[k][j][Nr]):    
                                min_yi = self.min_positive(y[i][j][0:Nr])[1]
                                min_yk = self.min_positive(y[k][j][0:Nr])[1]
                                if (y[i][j][Nr]-min_yi > y[k][j][Nr]-min_yk):   # i认为的任务开始时间与最快机器人的时间差大于k
                                    z[i][j], y[i][j], r[i][j], s[i][j], t[i][j] = z[k][j], y[k][j], r[k][j], s[k][j], t[k][j]
                                    #print("满载共识：时间差，机器人i", i, "任务", j, "时间", t[i][j], "机器人k", k, "任务", j, "时间", t[k][j])
                                elif (y[i][j][Nr]-min_yi == y[k][j][Nr]-min_yk):    
                                    # sum_yi = self.sum_of_non_negative(y[i][j][0:Nr])[1]
                                    # sum_yk = self.sum_of_non_negative(y[k][j][0:Nr])[1]
                                # num_i = np.where(np.array(y[i][j][0:Nr]) != -1)[0].size
                                # num_k = np.where(np.array(y[k][j][0:Nr]) != -1)[0].size
                                # if (num_i > num_k):     # i认为的机器人数量多于k
                                #     z[i][j], y[i][j], r[i][j], s[i][j], t[i][j] = z[k][j], y[k][j], r[k][j], s[k][j], t[k][j]
                                # elif (num_i == num_k):
                                #     # if (y[i][j][Nr] < y[k][j][Nr]):   # i认为的最快机器人等待时间大于k
                                #     #     z[i][j], y[i][j], r[i][j], s[i][j], t[i][j] = z[k][j], y[k][j], r[k][j], s[k][j], t[k][j]
                                #     # elif (y[i][Nr] == y[k][Nr]):
                                    if (i > k): # i的索引大于k
                                        z[i][j], y[i][j], r[i][j], s[i][j], t[i][j] = z[k][j], y[k][j], r[k][j], s[k][j], t[k][j]
                                        #print("满载共识：索引值，机器人i", i, "任务", j, "时间", t[i][j], "机器人k", k, "任务", j, "时间", t[k][j])

                        # i任务j未满载, k任务j满载: 更新i对任务j的信息
                        elif (r[i][j][Nr] >= epsilon or s[i][j][Nr] > epsilon) and (r[k][j][Nr] <= epsilon and s[k][j][Nr] <= epsilon): # i认为任务j未满载，k认为任务j满载
                            # Update i's information for task j
                            z[i][j], y[i][j], r[i][j], s[i][j], t[i][j] = z[k][j], y[k][j], r[k][j], s[k][j], t[k][j]
                            #print("未满载与满载共识：",i,"<---",j,"---",k)

                    # 更新时间矩阵
                    for n in range(self.num_robots):
                        if (n != i) and (time_mat_new[i][n] < time_mat[k][n]):
                            time_mat_new[i][n] = time_mat[k][n]
                    time_mat_new[i][k] = iter_idx

        # 更新任务分配信息
        self.winners_list = copy.deepcopy(z)
        self.winner_bid_list = copy.deepcopy(y)
        self.winner_recload_list = copy.deepcopy(r)
        self.winner_strload_list = copy.deepcopy(s)
        self.task_times_list = copy.deepcopy(t)

        return time_mat_new

    def compute_bid(self, idx_robot: int, feasibility: list):
        """
        Computes bids for each task. Returns bids, best index for task in
        the path, and times for the new path
        """
        # If the path is full then we cannot add any tasks to it
        empty_task_index_list = np.where(np.array(self.path_list[idx_robot]) == -1)[0] #智能体任然空闲位置的索引
        if len(empty_task_index_list) == 0:
            best_indices = []  
            task_times = []
            feasibility = [] #单指当前机器人对每个任务的可行性
            return best_indices, task_times, feasibility
        # Reset bids, best positions in path, and best times 
        self.bid_list[idx_robot] = [-1] * self.num_tasks
        best_indices = [-1] * self.num_tasks
        task_times = [-2] * self.num_tasks
        # For each task
        for idx_task in range(self.num_tasks):
            # Check for compatibility between robot and task 检查任务和智能体的兼容性
            # for floating precision
            #if self.compatibility_mat[self.RobotList[idx_robot].robot_type][self.TaskList[idx_task].task_type] > 0.5:
            best_time = -2
            best_bid = -1
            if self.bids_num_list[idx_robot][idx_task] >= 0:
                self.bids_num_list[idx_robot][idx_task] -= 1
                if (self.strload_list[idx_robot][idx_robot] > 0 and self.TaskList[idx_task].str_need > 0) or \
                    (self.recload_list[idx_robot][idx_robot] > 0 and self.TaskList[idx_task].rec_need > 0):
                    
                    # Check to make sure the path doesn't already contain task m
                    index_array = np.where(np.array(self.path_list[idx_robot][0:empty_task_index_list[0]]) == idx_task)[0]
                    if len(index_array) < 0.5:
                        # this task not in my bundle yet
                        # Find the best score attainable by inserting the score into the current path
                        
                        best_bid = float('inf')
                        best_index = -1
                        best_time = -2
                        # Try inserting task m in location j among other tasks and see if it generates a better new_path.
                        # 将新任务插入到现有的任务路径中
                        for j in range(empty_task_index_list[0]+1):
                            if feasibility[idx_task][j] == 1:
                                # Check new path feasibility, true to skip this iteration, false to be feasible
                                skip_flag = False

                                #确定插入前、后的任务和时间
                                if j == 0:
                                    # insert at the beginning
                                    task_prev = []
                                    time_prev = []
                                else:
                                    Task_temp = self.TaskList[self.path_list[idx_robot][j-1]]
                                    task_prev = Task(**Task_temp.__dict__)
                                    #time_prev = self.task_times_list[idx_robot][j-1]
                                    time_prev = self.task_times_list[idx_robot][self.path_list[idx_robot][j-1]]
                                
                                if j == (empty_task_index_list[0]):
                                    task_next = []
                                    time_next = []
                                else:
                                    Task_temp = self.TaskList[self.path_list[idx_robot][j]]
                                    task_next = Task(**Task_temp.__dict__)
                                    time_next = self.robot_times_list[idx_robot][j]

                                # Compute min and max start times and score
                                Task_temp = self.TaskList[idx_task]
                                [score, min_start, max_start] = self.scoring_compute_score(
                                    idx_robot, Task(**Task_temp.__dict__), task_prev, time_prev, task_next, time_next)
                                if self.time_window_flag:
                                    # if tasks have time window
                                    # if max_start < self.task_times_list[idx_robot][idx_task]: #最大到达时间不得超过任务开始时间
                                    #     skip_flag = True
                                    #     feasibility[idx_task][j] = 0

                                    if min_start > max_start:
                                        # Infeasible path
                                        skip_flag = True
                                        feasibility[idx_task][j] = 0

                                    # if min_start > self.TaskList[idx_task].end_time - self.TaskList[idx_task].duration: # 保证任务在截止时间前完成
                                    #     skip_flag = True
                                    #     feasibility[idx_task][j] = 0

                                    if not skip_flag:
                                        # Save the best score and task position， 选择最佳收益任务
                                        if score < best_bid: 
                                            best_bid = score
                                            best_index = j
                                            # Select min start time as optimal
                                            best_time = min_start
                                            # #print("best_bid: ", best_bid, "best_time: ", best_time)
                                else:
                                    # no time window for tasks
                                    # Save the best score and task position
                                    if score > best_bid:
                                        best_bid = score
                                        best_index = j
                                        # Select min start time as optimal  选择最佳启动时间
                                        best_time = 0.0

                                ##print("机器人", idx_robot, "任务", idx_task, "位置", j, "出价", best_bid, "最小时间", min_start, "最大时间", max_start)

                        # save best bid information
                        if best_bid >= 0 and best_bid != float('inf'):
                            self.bid_list[idx_robot][idx_task] = best_bid  #更新出价
                            best_indices[idx_task] = best_index
                            task_times[idx_task] = best_time

            # this task is incompatible with my type
        # end loop through tasks
        return best_indices, task_times, feasibility

    def scoring_compute_score(self, idx_robot: int, task_current: Task, task_prev: Task,
                              time_prev, task_next: Task, time_next):
        """
        Compute marginal score of doing a task and returns the expected start time for the task.
        """

        if (self.RobotList[idx_robot].robot_type == self.robot_types.index("rec_robot")) or \
                (self.RobotList[idx_robot].robot_type == self.robot_types.index("str_robot")) or \
                    (self.RobotList[idx_robot].robot_type == self.robot_types.index("rs_robot")):
            
            # if not task_prev:
            #     # First task in path
            #     # Compute start time of task
            #     dt = math.sqrt((self.RobotList[idx_robot].x-task_current.x)**2 +
            #                    (self.RobotList[idx_robot].y-task_current.y)**2 +
            #                    (self.RobotList[idx_robot].z-task_current.z)**2) / self.RobotList[idx_robot].nom_velocity  #路途时间
            #     min_start = max(task_current.start_time, self.RobotList[idx_robot].availability + dt)
            # else:
            #     # Not first task in path
            #     dt = math.sqrt((task_prev.x-task_current.x)**2 + (task_prev.y-task_current.y)**2 +
            #                    (task_prev.z-task_current.z)**2) / self.RobotList[idx_robot].nom_velocity
            #     # i have to have time to do task at j-1 and go to task m
            #     min_start = max(task_current.start_time, time_prev + task_prev.duration + dt)  #当前任务开始和“前一任务开始后加上执行和路途时间”的最大值

            # if not task_next:
            #     # Last task in path
            #     dt = 0.0
            #     max_start = task_current.end_time
            # else:
            #     # Not last task, check if we can still make promised task
            #     dt = math.sqrt((task_next.x-task_current.x)**2 + (task_next.y-task_current.y)**2 +
            #                    (task_next.z-task_current.z)**2) / self.RobotList[idx_robot].nom_velocity
            #     # i have to have time to do task m and fly to task at j+1
            #     max_start = min(task_current.end_time-task_current.duration, time_next - task_current.duration - dt) #当前任务结束后一任务开始减去执行时间的最小值
            
            if not task_prev:
                dt =  math.sqrt((self.RobotList[idx_robot].x-task_current.x)**2 + (self.RobotList[idx_robot].y-task_current.y)**2) / self.RobotList[idx_robot].nom_velocity
                min_reach = self.RobotList[idx_robot].availability + dt
            else:
                dt = math.sqrt((task_prev.x-task_current.x)**2 + (task_prev.y-task_current.y)**2) / self.RobotList[idx_robot].nom_velocity
                min_reach = time_prev + task_prev.duration + dt
            if not task_next:
                dt = 0
                # max_reach = task_current.end_time - task_current.duration
                max_reach = self.task_times_list[idx_robot][task_current.task_id]
            else:
                dt = math.sqrt((task_next.x-task_current.x)**2 + (task_next.y-task_current.y)**2) / self.RobotList[idx_robot].nom_velocity
                max_reach = time_next - task_current.duration - dt


            # # Compute score
            # if self.time_window_flag:
            #     # if tasks have time window
            #     reward = task_current.task_value * \
            #              math.exp((-task_current.discount) * (min_start-task_current.start_time))
                
            # else:
            #     # no time window for tasks
            #     dt_current = math.sqrt((self.RobotList[idx_robot].x-task_current.x)**2 +
            #                            (self.RobotList[idx_robot].y-task_current.y)**2 +
            #                            (self.RobotList[idx_robot].z-task_current.z)**2) / \
            #                  self.RobotList[idx_robot].nom_velocity

            #     reward = task_current.task_value * math.exp((-task_current.discount) * dt_current)

            # # Subtract fuel cost. Implement constant fuel to ensure DMG (diminishing marginal gain).
            # # This is a fake score since it double-counts fuel. Should not be used when comparing to optimal score.
            # # Need to compute real score of CBPA paths once CBPA algorithm has finished running.
            # penalty = self.RobotList[idx_robot].fuel * math.sqrt(
            #     (self.RobotList[idx_robot].x-task_current.x)**2 + (self.RobotList[idx_robot].y-task_current.y)**2 +
            #     (self.RobotList[idx_robot].z-task_current.z)**2)
            #
            # score = reward - penalty
            #score = reward
            # score = min_start
            score = min_reach
        else:
            # FOR USER TO DO:  Define score function for specialized robots, for example:
            # elseif(robot.type == CBPA_Params.ROBOT_TYPES.NEW_ROBOT), ...  
            # Need to define score, minStart and maxStart
            raise Exception("Unknown robot type!")
        return score, min_reach, max_reach
    
    def single_result_value(self):
        task_lode_list = [30]*self.num_tasks
        robot_lode_list = [100]*self.num_robots
        total_value = 0
        for idx_robot in range(self.num_robots):
            for j in range(len(self.path_list[idx_robot])):
                idx_task = self.path_list[idx_robot][j]
                # self.task_times_list[0][idx_task] = self.robot_times_list[idx_robot][j]
                if robot_lode_list[idx_robot] >= task_lode_list[idx_task]:
                    robot_lode_list[idx_robot] -= task_lode_list[idx_task]
                    task_lode_list[idx_task] = 0
                else:
                    task_lode_list[idx_task] -= robot_lode_list[idx_robot]
                    robot_lode_list[idx_robot] = 0
        #print("robot_load_list:",robot_lode_list)
        #print("task_load_list:",task_lode_list)
        #print("task_time:",self.task_times_list[0])
        for i in range(self.num_tasks):
            total_value += self.TaskList[i].task_value*((self.TaskList[i].str_need-task_lode_list[i])/self.TaskList[i].str_need)*math.exp(-0.07*self.task_times_list[0][i])
        #print(self.num_robots,"个机器人执行",self.num_tasks,"个任务的总收益为(CBPA):",total_value)

        str_value = str(total_value)
        with open("exp2_CBBA_value.txt", "a") as file:
            file.write(str_value+"\n")

    def plot_robot_schedules(self):
        # 重新进行任务分配
        #self.solve(RobotList, TaskList, self.world_info, self.max_depth, self.bid_times, time_window_flag=True)

        # 清空之前的图表
        plt.clf()

        # 绘制新的图表
        fig_schedule = plt.figure(1)
        fig_schedule.suptitle("Schedules for Robots")  # 添加整体标题

        for idx_robot in range(self.num_robots):
            ax = plt.subplot(self.num_robots, 1, idx_robot+1)  # 创建子图
            ax.set_title("Robot "+str(idx_robot+1), fontsize=10)  # 子图标题
            ax.title.set_position([-0.07, 0.2])
            if idx_robot == (self.num_robots - 1):
                ax.set_xlabel("Time")
            ax.set_xlim(self.time_interval_list)
            ax.set_yticks([])

            color_list = ["#91CCC0", "#7FABD1", "#F7AC53", "#EC6E66", "#B5CE4E", "#BD7795", "#C89736", "#52AADC", "#2D8875", "#C7C1DE"]
            if self.path_list[idx_robot]:
                for idx_path in range(len(self.path_list[idx_robot])):
                    color_str = color_list[self.path_list[idx_robot][idx_path]]
                    if self.path_list[idx_robot][idx_path] > -1:
                        task_current = self.lookup_task(self.path_list[idx_robot][idx_path])
                        ax.plot([self.robot_times_list[idx_robot][idx_path],
                                self.task_times_list[idx_robot][self.path_list[idx_robot][idx_path]] + task_current.duration],
                                [1, 1], linestyle='-', linewidth=10, color=color_str, alpha=0.5)
                        ax.plot([self.task_times_list[idx_robot][self.path_list[idx_robot][idx_path]],
                                self.task_times_list[idx_robot][self.path_list[idx_robot][idx_path]] + task_current.duration],
                                [1, 1], linestyle='-.', linewidth=2, color=color_str)
                        text_x = (self.robot_times_list[idx_robot][idx_path] + self.task_times_list[idx_robot][self.path_list[idx_robot][idx_path]] + task_current.duration) / 2
                        text_y = 1 + 0.02  # 在 y 轴上方稍微偏移
                        ax.text(text_x, text_y, "T"+str(self.path_list[idx_robot][idx_path]+1), ha='center', va='bottom', fontsize=9, color='black')
        
        plt.subplots_adjust(hspace=0.8)
        plt.pause(0.1)  # 显示更新后的图表
        #print("22222222")

    def plot_assignment(self):
        """
        Plots CBPA outputs when there is time window for tasks.
        """

        # 3D plot 
        fig_3d = plt.figure(1) #创建一个图形对象
        ax_3d = fig_3d.add_subplot(111, projection='3d')  #创建一个3D子图对象ax_3d，并将投影类型设置为’3d’
        # offset to plot text in 3D space
        offset = (self.MyWorldInfo.limit_x[1]-self.MyWorldInfo.limit_x[0]) / 50 # 坐标单位长度

        # 绘制任务
        #color_str = "yellow"
        color_str = "#4B57A2"
        for m in range(self.num_tasks):
            # ax_3d.scatter([self.TaskList[m].x]*2, [self.TaskList[m].y]*2,
            #               [self.TaskList[m].start_time, self.TaskList[m].end_time], marker='^', s=40, color=color_str) #任务开始和结束的散点
            ax_3d.scatter(self.TaskList[m].x, self.TaskList[m].y, self.task_times_list[0][m]+self.TaskList[m].duration, marker='^', s=30, color=color_str) #任务截止时间的散点
            ax_3d.plot3D([self.TaskList[m].x]*2, [self.TaskList[m].y]*2,[self.task_times_list[0][m], 
                                self.task_times_list[0][m]+self.TaskList[m].duration],linestyle=':', color=color_str, linewidth=4) #任务开始和结束用虚线连接
            ax_3d.text(self.TaskList[m].x+offset, self.TaskList[m].y+offset,self.task_times_list[0][m]+self.TaskList[m].duration, "T"+str(m))

        # 绘制机器人
        for n in range(self.num_robots):
            if self.RobotList[n].robot_type == 0:
                color_str = "#7FABD1"
                #color_str = 'red'
            elif self.RobotList[n].robot_type == 1:
                color_str = "#F39865"
                #color_str = 'green'
            else:
                color_str = "#963B79"
                #color_str = 'blue'
            ax_3d.scatter(self.RobotList[n].x, self.RobotList[n].y, 0, marker='o', color=color_str)
            ax_3d.text(self.RobotList[n].x+offset, self.RobotList[n].y+offset, 0.1, "R"+str(n))

            # if the path is not empty
            if self.path_list[n]:
                Task_prev = self.lookup_task(self.path_list[n][0])
                ax_3d.plot3D([self.RobotList[n].x, Task_prev.x], [self.RobotList[n].y, Task_prev.y],
                             [0, self.robot_times_list[n][0]], linewidth=2, color=color_str) #绘制从Robot位置到第一个任务位置的路径
                ax_3d.plot3D([Task_prev.x, Task_prev.x], [Task_prev.y, Task_prev.y],
                             [self.robot_times_list[n][0], self.task_times_list[n][self.path_list[n][0]]+Task_prev.duration],
                             linewidth=2, color=color_str) #绘制一个垂直于地面的线段，表示第一个任务的持续时间

                for m in range(1, len(self.path_list[n])):
                    if self.path_list[n][m] > -1:
                        Task_next = self.lookup_task(self.path_list[n][m])
                        ax_3d.plot3D([Task_prev.x, Task_next.x], [Task_prev.y, Task_next.y],
                                     [self.task_times_list[n][self.path_list[n][m-1]]+Task_prev.duration, self.robot_times_list[n][m]],
                                     linewidth=2, color=color_str) #绘制从一个任务位置到下一个任务位置的路径
                        ax_3d.plot3D([Task_next.x, Task_next.x], [Task_next.y, Task_next.y],
                                     [self.robot_times_list[n][m], self.task_times_list[n][self.path_list[n][m]]+Task_next.duration],
                                     linewidth=2, color=color_str) 
                        Task_prev = Task(**Task_next.__dict__)
        
        plt.title('Robot Paths with Time Windows')
        ax_3d.set_xlabel("X")
        ax_3d.set_ylabel("Y")
        ax_3d.set_zlabel("Time")
        ax_3d.set_aspect('auto')

        # set legends
        colors = ["#7FABD1", "#F39865", "#963B79"]

        marker_list = ["solid", "solid", "solid"]
        labels = ["Reconnaissance robot", "Strike robot", "Reconnaissance & Strike robot"]
        def f(marker_type, color_type): return plt.plot([], [], linestyle=marker_type, color=color_type)[0]
        handles = [f(marker_list[i], colors[i]) for i in range(len(labels))]
        plt.legend(handles, labels, bbox_to_anchor=(1, 1), loc='upper left', framealpha=1)
        # marker_handle = plt.plot([], [], marker = "^", color = "#4B57A2", ls = "none")[0]
        # plt.legend(marker_handle, ["Task"], bbox_to_anchor=(1, 1), loc='upper left', framealpha=1)
        self.set_axes_equal_xy(ax_3d, flag_3d=True)

        if self.duration_flag:
            # plot robot schedules
            fig_schedule = plt.figure(2)
            # fig_schedule.suptitle("Schedules for Robots") #标题
            for idx_robot in range(self.num_robots):
                ax = plt.subplot(self.num_robots, 1, idx_robot+1) # 创建子图
                ax.set_title("Robot "+str(idx_robot+1), fontsize=10) #子图标题
                ax.title.set_position([-0.07, 0.2]) 
                if idx_robot == (self.num_robots - 1):
                    ax.set_xlabel("Time") 
                ax.set_xlim(self.time_interval_list)
                #ax.set_ylim([0.95, 1.05])
                ax.set_yticks([])

                # quad robot is red
                # color_list = ["#ED1E24", "#387FB9", "#48AC42"]
                color_list = ["#91CCC0", "#7FABD1", "#F7AC53", "#EC6E66", "#B5CE4E", "#BD7795", "#C89736", "#52AADC", "#2D8875", "#C7C1DE"]
                # if self.RobotList[idx_robot].robot_type == 0:
                #     color_str = 'red'
                # # car robot is blue
                # else:
                #     color_str = '#EA8379'

                # if self.path_list[idx_robot]:
                #     for idx_path in range(len(self.path_list[idx_robot])):
                #         color_str = color_list[self.path_list[idx_robot][idx_path]]
                #         if self.path_list[idx_robot][idx_path] > -1:
                #             task_current = self.lookup_task(self.path_list[idx_robot][idx_path]) #当前任务
                #             ax.plot([task_current.start_time,
                #                      task_current.end_time], [1, 1],
                #                     linestyle='-', linewidth=10, color=color_str, alpha=0.5)
                #             ax.plot([task_current.start_time, task_current.end_time], [1, 1], linestyle='-.',
                #                     linewidth=2, color=color_str)
                if self.path_list[idx_robot]:
                    for idx_path in range(len(self.path_list[idx_robot])):
                        color_str = color_list[self.path_list[idx_robot][idx_path]]
                        if self.path_list[idx_robot][idx_path] > -1:
                            task_current = self.lookup_task(self.path_list[idx_robot][idx_path]) #当前任务
                            ax.plot([self.robot_times_list[idx_robot][idx_path],
                                     self.task_times_list[idx_robot][self.path_list[idx_robot][idx_path]]+task_current.duration], [1, 1],
                                    linestyle='-', linewidth=10, color=color_str, alpha=0.5)
                            ax.plot([self.task_times_list[idx_robot][self.path_list[idx_robot][idx_path]], self.task_times_list[idx_robot][self.path_list[idx_robot][idx_path]]+task_current.duration],
                                     [1, 1], linestyle='-.',linewidth=2, color=color_str)
                            text_x = (self.robot_times_list[idx_robot][idx_path] + self.task_times_list[idx_robot][self.path_list[idx_robot][idx_path]] + task_current.duration) / 2
                            text_y = 1 + 0.02  # 在 y 轴上方稍微偏移
                            ax.text(text_x, text_y, "T"+str(self.path_list[idx_robot][idx_path]+1), ha='center', va='bottom', fontsize=9, color='black')
            

            # # set legends
            # colors = ["#7FABD1", "#7FABD1"]
            # line_styles = ["-", "-."]
            # line_width_list = [8, 2]
            # labels = ["Assignment Time", "Task Time"]
            # def f(line_style, color_type, line_width): return plt.plot([], [], linestyle=line_style, color=color_type,
            #                                                            linewidth=line_width)[0]
            # handles = [f(line_styles[i], colors[i], line_width_list[i]) for i in range(len(labels))]
            # fig_schedule.legend(handles, labels, bbox_to_anchor=(1, 1), loc='upper right', framealpha=1)
        plt.subplots_adjust(hspace=0.8)
        plt.show(block=False)

    def plot_assignment_without_timewindow(self):
        """
        Plots CBPA outputs when there is no time window for tasks.
        """

        # 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111)
        # offset to plot text in 3D space
        offset = (self.WorldInfo.limit_x[1]-self.WorldInfo.limit_x[0]) / 100

        # plot tasks
        for m in range(self.num_tasks):
            # track task is red
            if self.TaskList[m].task_type == 0:
                color_str = 'red'
            # rescue task is blue
            else:
                color_str = 'blue'
            ax.scatter(self.TaskList[m].x, self.TaskList[m].y, marker='x', color=color_str)
            ax.text(self.TaskList[m].x+offset, self.TaskList[m].y+offset, "T"+str(m))

        # plot robots
        for n in range(self.num_robots):
            # quad robot is red
            if self.RobotList[n].robot_type == 0:
                color_str = 'red'
            # car robot is blue
            else:
                color_str = 'blue'
            ax.scatter(self.RobotList[n].x, self.RobotList[n].y, marker='o', color=color_str)
            ax.text(self.RobotList[n].x+offset, self.RobotList[n].y+offset, "A"+str(n))

            # if the path is not empty
            if self.path_list[n]:
                Task_prev = self.lookup_task(self.path_list[n][0])
                ax.plot([self.RobotList[n].x, Task_prev.x], [self.RobotList[n].y, Task_prev.y],
                        linewidth=2, color=color_str)
                for m in range(1, len(self.path_list[n])):
                    if self.path_list[n][m] > -1:
                        Task_next = self.lookup_task(self.path_list[n][m])
                        ax.plot([Task_prev.x, Task_next.x], [Task_prev.y, Task_next.y], linewidth=2, color=color_str)
                        Task_prev = Task(**Task_next.__dict__)
        
        plt.title('Robot Paths without Time Windows')
        ax.set_xlabel("X")
        ax.set_ylabel("Y")

        # set legends
        colors = ["red", "blue", "red", "blue"]
        marker_list = ["o", "o", "x", "x"]
        labels = ["Robot type 1", "Robot type 2", "Task type 1", "Task type 2"]
        def f(marker_type, color_type): return plt.plot([], [], marker=marker_type, color=color_type, ls="none")[0]
        handles = [f(marker_list[i], colors[i]) for i in range(len(labels))]
        plt.legend(handles, labels, bbox_to_anchor=(1, 1), loc='upper left', framealpha=1)

        self.set_axes_equal_xy(ax, flag_3d=False)
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
        plot_radius = 0.5*max([x_range, y_range])

        if flag_3d:
            ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
            ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
            if abs(self.time_interval_list[1]) >= 1e-3:
                ax.set_zlim3d(self.time_interval_list)
        else:
            ax.set_xlim([x_middle - plot_radius, x_middle + plot_radius])
            ax.set_ylim([y_middle - plot_radius, y_middle + plot_radius])

    def lookup_task(self, task_id: int):
        """
        Look up a Task given the task ID.
        """

        TaskOutput = []
        for m in range(self.num_tasks):
            if self.TaskList[m].task_id == task_id:
                Task_temp = self.TaskList[m]
                TaskOutput.append(Task(**Task_temp.__dict__))

        if not TaskOutput:
            raise Exception("Task " + str(task_id) + " not found!")

        return TaskOutput[0]
