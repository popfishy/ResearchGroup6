import numpy as np
import math

# 创建一个3x3x3的三维数组
array_3d: list
array_3d = np.array([
    [[1, 2, 3], [4, 5, 6], [7, 8, 9]],
    [[10, 11, 12], [13, 14, 15], [16, 17, 18]],
    [[19, 20, 21], [22, 23, 24], [25, 26, 27]]
])
array_2d = array_3d[0, 1]
#array_2d.append(2)
array_3d = [[[-1 for _ in range(2)] for _ in range(3)] for _ in range(4)]
#result：[[[0, 0], [0, 0], [0, 0]], [[0, 0], [0, 0], [0, 0]], [[0, 0], [0, 0], [0, 0]], [[0, 0], [0, 0], [0, 0]]]
#print(array_3d[3][1][:])


x = 10

def func():
    global x  # 使用global关键字声明x是全局变量
    x = 20  # 修改全局变量x的值
#    print(x)  # 输出全局变量x的值

func()  # 调用函数

feasibility = [[1] * (3) for _ in range(5)] 
#print(feasibility)  # 输出全局变量x的值

task_full_flag = np.array([False] * 5)
#print(task_full_flag[0])

## '测试np.argmax()函数'
# list = [[1, 2, 3, 4, 5], [6, 7, 8, 9, 10], [11, 12, 13, 14, 15]]
# a = np.array(list[0])
# b = a.argmax()
# print(b)
# a[b] = 0
# list[0] = a.tolist()
# print(list)

# # 线性规划 载荷分配函数
# from scipy.optimize import linprog
# def distribute_water(task_load,load_array):
#     load_array = np.array(load_array)
#     n = len(load_array)
#     c = [-1] * n
#     A_eq = [[1] * n]
#     b_eq = [task_load]
#     bounds = [(0, c) for c in load_array]
#     res = linprog(c, A_eq=A_eq, b_eq=b_eq, bounds=bounds, method='simplex')  #线性规划求解器
#     return np.array(res.x)
# total_volume = 22
# cup_capability = [20, 7, 14]
# res = distribute_water(total_volume, cup_capability)
# print(res)

def distribute_water(task_load, robot_loads):
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

# 示例输入
task_load = 6
robot_loads = [3,0,3,0,0,0,0,2,0,0]

# 调用函数进行水量分配
result = distribute_water(task_load, robot_loads)
# print(result) 


# # 数组最小正数索引提取
def min_positive(lst):
    positive_with_index = [(index, num) for index, num in enumerate(lst) if num > 0]
    if positive_with_index:
        return min(positive_with_index, key=lambda x: x[1])
    else:
        return None
list = np.array([[[[-1] for _ in range(5)] for _ in range(3)] for _ in range(4)])
n = 2
#print(min_positive(list[1][2][0:n]))  # 输出：(1, 1)

def min_nonnegative_index(arr):
    arr = np.array(arr)
    nonnegative_vals = arr[arr >= 0]  
    if len(nonnegative_vals) == 0:
        return -1  
    min_nonnegative_index = np.argmin(nonnegative_vals)
    # 获取最小非负数在原数组中的索引
    indices = np.where(arr >= 0)[0]
    return indices[min_nonnegative_index]

# 示例输入
arr = [-1, 39, 122]

# 调用函数查找最小非负数的索引
result_index = min_nonnegative_index(arr)

# 输出结果索引
#print("最小非负数的索引：", result_index)

path_list = [-1,-1,-1,-1,-1]
empty = np.where(np.array(path_list) == -1)[0]
print(empty)
print(path_list[0:1])


                        # # 任务满载，减少到达任务的机器人的等待
                        # Sum_RecLoad = self.sum_of_non_negative(self.winner_recload_list[idx_robot][best_task])
                        # Sum_StrLoad = self.sum_of_non_negative(self.winner_strload_list[idx_robot][best_task])
                        # if (Sum_RecLoad - self.TaskList[best_task].rec_need) >= 0 and (Sum_StrLoad - self.TaskList[best_task].str_need) >= 0:
                        #     load_full = True
                        # else:
                        #     load_full = False
                        #     print("发生错误！！！减少到达任务机器人的等待")
                        # while load_full:
                        #     print('进入循环，减少到达任务的机器人的等待')
                        #     current_winner_array = np.array(self.winners_list[idx_robot][best_task])
                        #     fast_index = self.min_positive(self.winner_bid_list[idx_robot][best_task][0:self.num_robots])[0]
                        #     fast_time = self.winner_bid_list[idx_robot][best_task][fast_index]
                        #     if fast_index != idx_robot and best_time > fast_time:
                        #         current_winner_array[fast_index] = 0
                        #         current_winner_array[idx_robot] = 1
                        #         Sum_RecLoad_used = self.sum_of_non_negative(self.winner_recload_list[idx_robot][best_task][0:self.num_robots] * current_winner_array)
                        #         Sum_StrLoad_used = self.sum_of_non_negative(self.winner_strload_list[idx_robot][best_task][0:self.num_robots] * current_winner_array)
                        #         Sum_RecLoad = Sum_RecLoad_used + self.sum_of_non_negative(self.recload_list[idx_robot] * current_winner_array)
                        #         Sum_StrLoad = Sum_StrLoad_used + self.sum_of_non_negative(self.strload_list[idx_robot] * current_winner_array)
                        #     else:
                        #         load_full = False
                        #         break
                        #     if (Sum_RecLoad - self.TaskList[best_task].rec_need) >= 0 and (Sum_StrLoad - self.TaskList[best_task].str_need) >= 0:
                        #         # 更新获胜者列表、获胜者出价列表
                        #         self.winners_list[idx_robot][best_task][fast_index] = 0    # 最快到达机器人失败
                        #         self.winners_list[idx_robot][best_task][idx_robot] = 1     # 当前机器人获胜
                        #         self.winner_bid_list[idx_robot][best_task][fast_index] = -1     # 最快到达机器人出价清零
                        #         self.winner_bid_list[idx_robot][best_task][idx_robot] = value_max   # 记录当前机器人出价
                        #         self.winner_bid_list[idx_robot][best_task][self.num_robots] = self.max_positive(self.winner_bid_list[idx_robot][best_task][0:self.num_robots])[1]  # 记录最快时间
                        #         #current_bid_array = np.array(self.winner_bid_list[idx_robot][best_task])
                                
                        #         # 更新任务分配载荷列表、机器人剩余载荷列表
                        #         self.recload_list[idx_robot] += np.array(self.winner_recload_list[idx_robot][best_task][0:self.num_robots])  # 恢复机器人侦察载荷等待重新分配
                        #         self.strload_list[idx_robot] += np.array(self.winner_strload_list[idx_robot][best_task][0:self.num_robots])  # 恢复机器人打击载荷等待重新分配
                        #         robot_recditribute = np.array(self.recload_list[idx_robot]) * np.array(self.winners_list[idx_robot][best_task])  # 当前参与任务的机器人侦察载荷值
                        #         robot_strditribute = np.array(self.strload_list[idx_robot]) * np.array(self.winners_list[idx_robot][best_task])  # 当前参与任务的机器人打击载荷值
                        #         recload_distribute = self.distribute_load(robot_recditribute, self.TaskList[best_task].rec_need)  # 分配侦察载荷
                        #         strload_distribute = self.distribute_load(robot_strditribute, self.TaskList[best_task].str_need)  # 分配打击载荷
                        #         self.winner_recload_list[idx_robot][best_task][0:self.num_robots] = recload_distribute
                        #         self.winner_strload_list[idx_robot][best_task][0:self.num_robots] = strload_distribute
                        #         self.winner_recload_list[idx_robot][best_task][self.num_robots] = self.TaskList[best_task].rec_need - self.sum_of_non_negative(recload_distribute)
                        #         self.winner_strload_list[idx_robot][best_task][self.num_robots] = self.TaskList[best_task].str_need - self.sum_of_non_negative(strload_distribute) # 获胜载荷包最后一位标志任务满载

                        #         self.recload_list[idx_robot] = self.recload_list[idx_robot] - recload_distribute # 更新分配后的机器人侦察载荷
                        #         self.strload_list[idx_robot] = self.strload_list[idx_robot] - strload_distribute # 更新分配后的机器人打击载荷


                        #         # 更新任务时间列表
                        #         if best_time <= self.task_times_list[idx_robot][best_task]:
                        #             self.task_times_list[idx_robot][best_task] = best_time
                        #         else:
                        #             print("发生错误！！！满载2，机器人到达时间大于任务最大预计时间")

                        #         best_find_flag = True
                        #         load_full = True
                        #     else:
                        #         load_full = False
                        #         #current_bid_array = np.array(self.winner_bid_list[idx_robot][best_task])

                        # # 任务满载，减少机器人的侦察载荷
                        # if self.winner_recload_list[idx_robot][best_task][self.num_robots] >= 0 and \
                        #     self.winner_strload_list[idx_robot][best_task][self.num_robots] >= 0:
                        #     load_full = True
                        # else:
                        #     load_full = False
                        #     print("发生错误！！！减少机器人的侦察载荷")
                        # while load_full:
                        #     print('进入循环，减少机器人的侦察载荷')
                        #     current_winner_array = np.array(self.winners_list[idx_robot][best_task])
                        #     minRecload_index = self.min_positive(self.winner_recload_list[idx_robot][best_task][0:self.num_robots])[0]
                        #     minRecload = self.winner_recload_list[idx_robot][best_task][minRecload_index] + self.recload_list[idx_robot][minRecload_index]
                        #     bestRecload = self.winner_recload_list[idx_robot][best_task][idx_robot] + self.recload_list[idx_robot][idx_robot]
                        #     if minRecload_index != idx_robot and bestRecload > minRecload:
                        #         current_winner_array[minRecload_index] = 0
                        #         current_winner_array[best_task] = 1
                        #         Sum_RecLoad_used = self.sum_of_non_negative(self.winner_recload_list[idx_robot][best_task][0:self.num_robots] * current_winner_array)
                        #         Sum_StrLoad_used = self.sum_of_non_negative(self.winner_strload_list[idx_robot][best_task][0:self.num_robots] * current_winner_array)
                        #         Sum_RecLoad = Sum_RecLoad_used + self.sum_of_non_negative(self.recload_list[idx_robot] * current_winner_array)
                        #         Sum_StrLoad = Sum_StrLoad_used + self.sum_of_non_negative(self.strload_list[idx_robot] * current_winner_array)
                        #     else:
                        #         load_full = False
                        #         break
                        #     if (Sum_RecLoad - self.TaskList[best_task].rec_need) >= 0 and (Sum_StrLoad - self.TaskList[best_task].str_need) >= 0:
                        #         # 更新获胜者列表、获胜者出价列表
                        #         self.winners_list[idx_robot][best_task][minRecload_index] = 0    # 最快到达机器人失败
                        #         self.winners_list[idx_robot][best_task][idx_robot] = 1     # 当前机器人获胜
                        #         self.winner_bid_list[idx_robot][best_task][minRecload_index] = -1     # 最快到达机器人出价清零
                        #         self.winner_bid_list[idx_robot][best_task][idx_robot] = value_max   # 记录当前机器人出价
                        #         self.winner_bid_list[idx_robot][best_task][self.num_robots] = self.max_positive(self.winner_bid_list[idx_robot][best_task][0:self.num_robots])[1]  # 记录最快时间
                        #         if self.winner_bid_list[idx_robot][best_task][self.num_robots] != best_time:
                        #             print("发生错误！！！")

                        #         # 更新任务分配载荷列表、机器人剩余载荷列表
                        #         self.recload_list[idx_robot] += np.array(self.winner_recload_list[idx_robot][best_task][0:self.num_robots])  # 恢复机器人侦察载荷等待重新分配
                        #         self.strload_list[idx_robot] += np.array(self.winner_strload_list[idx_robot][best_task][0:self.num_robots])  # 恢复机器人打击载荷等待重新分配
                        #         robot_recditribute = np.array(self.recload_list[idx_robot]) * np.array(self.winners_list[idx_robot][best_task])  # 当前参与任务的机器人侦察载荷值
                        #         robot_strditribute = np.array(self.strload_list[idx_robot]) * np.array(self.winners_list[idx_robot][best_task])  # 当前参与任务的机器人打击载荷值
                        #         recload_distribute = self.distribute_load(robot_recditribute, self.TaskList[best_task].rec_need)  # 分配侦察载荷
                        #         strload_distribute = self.distribute_load(robot_strditribute, self.TaskList[best_task].str_need)  # 分配打击载荷
                        #         self.winner_recload_list[idx_robot][best_task][0:self.num_robots] = recload_distribute
                        #         self.winner_strload_list[idx_robot][best_task][0:self.num_robots] = strload_distribute
                        #         self.winner_recload_list[idx_robot][best_task][self.num_robots] = self.TaskList[best_task].rec_need - self.sum_of_non_negative(recload_distribute)
                        #         self.winner_strload_list[idx_robot][best_task][self.num_robots] = self.TaskList[best_task].str_need - self.sum_of_non_negative(strload_distribute) # 获胜载荷包最后一位标志任务满载

                        #         self.recload_list[idx_robot] = self.recload_list[idx_robot] - recload_distribute # 更新分配后的机器人侦察载荷
                        #         self.strload_list[idx_robot] = self.strload_list[idx_robot] - strload_distribute # 更新分配后的机器人打击载荷

                        #         # 更新任务时间列表
                        #         if best_time <= self.task_times_list[idx_robot][best_task]:
                        #             self.task_times_list[idx_robot][best_task] = best_time
                        #         else:
                        #             print("发生错误！！！满载3，机器人到达时间大于任务最大预计时间")
                                
                        #         best_find_flag = True
                        #         load_full = True
                        #         # Sum_RecLoad = Sum_RecLoad_Temp
                        #         # Sum_StrLoad = Sum_StrLoad_Temp
                        #     else:
                        #         load_full = False
                        
                        # # 任务满载，减少机器人的打击载荷
                        # if self.winner_recload_list[idx_robot][best_task][self.num_robots] >= 0 and \
                        #     self.winner_strload_list[idx_robot][best_task][self.num_robots] >= 0:
                        #     load_full = True
                        # else:
                        #     load_full = False
                        #     print("发生错误！！！减少机器人的打击载荷")
                        # while load_full:
                        #     print('进入循环，减少机器人的打击载荷')
                        #     #current_strload_array = np.array(self.winner_strload_list[idx_robot][best_task])  #当前任务每个机器人投入的侦察载荷
                        #     current_winner_array = np.array(self.winners_list[idx_robot][best_task])
                        #     minStrload_index = self.min_positive(self.winner_strload_list[idx_robot][best_task][0:self.num_robots])[0]
                        #     minStrload = self.winner_strload_list[idx_robot][best_task][minStrload_index] + self.strload_list[idx_robot][minStrload_index]
                        #     bestStrload = self.winner_strload_list[idx_robot][best_task][idx_robot] + self.strload_list[idx_robot][idx_robot]
                        #     if minStrload_index != idx_robot and bestStrload > minStrload:
                        #         current_winner_array[minStrload_index] = 0
                        #         current_winner_array[idx_robot] = 1
                        #         Sum_RecLoad_used = self.sum_of_non_negative(self.winner_recload_list[idx_robot][best_task][0:self.num_robots] * current_winner_array)
                        #         Sum_StrLoad_used = self.sum_of_non_negative(self.winner_strload_list[idx_robot][best_task][0:self.num_robots] * current_winner_array)
                        #         Sum_RecLoad = Sum_RecLoad_used + self.sum_of_non_negative(self.recload_list[idx_robot] * current_winner_array)
                        #         Sum_StrLoad = Sum_StrLoad_used + self.sum_of_non_negative(self.strload_list[idx_robot] * current_winner_array)
                        #     else:
                        #         load_full = False
                        #         break
                        #     #Sum_StrLoad_Temp = Sum_StrLoad_Temp - self.strload_list[idx_robot][minStrload_index]
                        #     if (Sum_RecLoad - self.TaskList[best_task].rec_need) >= 0 and (Sum_StrLoad - self.TaskList[best_task].str_need) >= 0:
                        #         # 更新获胜者列表、获胜者出价列表
                        #         self.winners_list[idx_robot][best_task][minStrload_index] = 0    # 最快到达机器人失败
                        #         self.winners_list[idx_robot][best_task][idx_robot] = 1     # 当前机器人获胜
                        #         self.winner_bid_list[idx_robot][best_task][minStrload_index] = -1     # 最快到达机器人出价清零
                        #         self.winner_bid_list[idx_robot][best_task][idx_robot] = value_max   # 记录当前机器人出价
                        #         self.winner_bid_list[idx_robot][best_task][self.num_robots] = self.max_positive(self.winner_bid_list[idx_robot][best_task][0:self.num_robots])[1]  # 记录最快时间
                        #         if self.winner_bid_list[idx_robot][best_task][self.num_robots] != best_time:
                        #             print("发生错误！！！")
                                
                        #         # 更新任务分配载荷列表、机器人剩余载荷列表
                        #         self.recload_list[idx_robot] += np.array(self.winner_recload_list[idx_robot][best_task][0:self.num_robots])  # 恢复机器人侦察载荷等待重新分配
                        #         self.strload_list[idx_robot] += np.array(self.winner_strload_list[idx_robot][best_task][0:self.num_robots])  # 恢复机器人打击载荷等待重新分配
                        #         robot_recditribute = np.array(self.recload_list[idx_robot]) * np.array(self.winners_list[idx_robot][best_task])  # 当前参与任务的机器人侦察载荷值
                        #         robot_strditribute = np.array(self.strload_list[idx_robot]) * np.array(self.winners_list[idx_robot][best_task])  # 当前参与任务的机器人打击载荷值
                        #         recload_distribute = self.distribute_load(robot_recditribute, self.TaskList[best_task].rec_need)  # 分配侦察载荷
                        #         strload_distribute = self.distribute_load(robot_strditribute, self.TaskList[best_task].str_need)  # 分配打击载荷
                        #         self.winner_recload_list[idx_robot][best_task][0:self.num_robots] = recload_distribute
                        #         self.winner_strload_list[idx_robot][best_task][0:self.num_robots] = strload_distribute
                        #         self.winner_recload_list[idx_robot][best_task][self.num_robots] = self.TaskList[best_task].rec_need - self.sum_of_non_negative(recload_distribute)
                        #         self.winner_strload_list[idx_robot][best_task][self.num_robots] = self.TaskList[best_task].str_need - self.sum_of_non_negative(strload_distribute) # 获胜载荷包最后一位标志任务满载

                        #         self.recload_list[idx_robot] = self.recload_list[idx_robot] - recload_distribute # 更新分配后的机器人侦察载荷
                        #         self.strload_list[idx_robot] = self.strload_list[idx_robot] - strload_distribute # 更新分配后的机器人打击载荷

                        #         # 更新任务时间列表
                        #         if best_time <= self.task_times_list[idx_robot][best_task]:
                        #             self.task_times_list[idx_robot][best_task] = best_time
                        #         else:
                        #             print("发生错误！！！满载4，机器人到达时间大于任务最大预计时间")

                        #         best_find_flag = True
                        #         load_full = True

                            # else:
                             #     load_full = False


# # 主循环
# def main():
#     global target_pos
#     running = True
#     while running:
#         for event in pygame.event.get():
#             if event.type == pygame.QUIT:
#                 running = False
#             elif event.type == pygame.KEYDOWN:
#                 if event.key == pygame.K_ESCAPE:
#                     running = False
#             elif event.type == pygame.MOUSEBUTTONDOWN:
#                 set_target_pos(event.pos)
#         draw_objects()
#         robot_move()