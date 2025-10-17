""" 分配函数测试"""
# import numpy as np
# def distribute_load(robot_loads: list, task_load: int):
#         robot_loads = np.array(robot_loads)
#         total_robot_load = np.sum(robot_loads)
#         result = np.array([0] * len(robot_loads))
#         if total_robot_load <= task_load:
#             result = robot_loads
#             return result
#         n = len(robot_loads)
#         while task_load > 0:
            
#             average_load = task_load // n
#             if average_load == 0:
#                 average_load = 1
#             for i in range(len(robot_loads)):
#                 if robot_loads[i] <= average_load and result[i] < robot_loads[i]:
#                     result[i] = robot_loads[i]
#                     task_load -= robot_loads[i]
#                     n -= 1
#                 elif robot_loads[i]-result[i] >= average_load and result[i]+average_load <= robot_loads[i]:
#                     result[i] = result[i]+average_load
#                     task_load -= average_load
#                     if task_load == 0:
#                         break
#         return result

# robot_loads = [0, 20, 0]
# task_load = 30
# result = robot_loads
# result = distribute_load(robot_loads,task_load)
# print(result)

# a = [0,0,0]
# a = np.array(result)
# # a = result
# a[1] = 0
# print(robot_loads)

"""实时绘图"""
import matplotlib.pyplot as plt
from collections import deque
import numpy as np
import threading

# 全局变量用于存储键盘输入的数据
data_buffer = deque(maxlen=100)  # 只保留最后100个输入
lock = threading.Lock()

# 更新数据的线程
def input_listener():
    while True:
        user_input = input("Enter a number (or 'exit' to quit): ")
        if user_input.lower() == 'exit':
            break
        try:
            number = float(user_input)
            with lock:
                data_buffer.append(number)
        except ValueError:
            print("Invalid input. Please enter a valid number.")

# 绘制图表的函数
def plot_data():
    plt.ion()  # 开启交互模式
    fig, ax = plt.subplots()
    x = np.arange(len(data_buffer))
    line, = ax.plot(x, np.zeros_like(x))

    ax.set_xlim(0, len(data_buffer) - 1)
    ax.set_ylim(-10, 10)  # 根据需要调整y轴范围

    while True:
        with lock:
            y = np.array(data_buffer)
        x = np.arange(len(y))

        line.set_data(x, y)
        ax.set_xlim(0, len(y) - 1)

        plt.pause(0.1)  # 更新图表
        fig.canvas.flush_events()

if __name__ == "__main__":
    # 启动输入监听线程
    input_thread = threading.Thread(target=input_listener, daemon=True)
    input_thread.start()

    # 绘制图表
    plot_data()
