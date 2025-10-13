import matplotlib.pyplot as plt
import numpy as np

# 定义机器人类
class Robot:
    def __init__(self, id, pos):
        self.id = id
        self.pos = pos
        self.path = [pos]  # 记录机器人的路径

    def move(self, new_pos):
        self.pos = new_pos
        self.path.append(new_pos)

# 定义CBCA算法类
class CBCA:
    def __init__(self, robots, targets):
        self.robots = robots
        self.targets = targets
        self.assignments = {}  # 记录机器人与目标的分配关系

    def run(self):
        for t in range(len(self.targets)):
            self.bidding(t)
            self.assignment(t)
            self.update_paths(t)
            self.plot()

    def bidding(self, t):
        for robot in self.robots:
            if robot.id not in self.assignments.values():
                cost = self.calculate_cost(robot, t)
                if robot.id not in self.assignments.keys():
                    self.assignments[robot.id] = {'target': t, 'cost': cost}
                else:
                    if cost < self.assignments[robot.id]['cost']:
                        self.assignments[robot.id] = {'target': t, 'cost': cost}

    def assignment(self, t):
        for target in self.targets:
            robot_ids = [robot.id for robot in self.robots if robot.id in self.assignments.keys() and self.assignments[robot.id]['target'] == t]
            if len(robot_ids) > 0:
                min_cost = float('inf')
                min_robot_id = None
                for robot_id in robot_ids:
                    if self.assignments[robot_id]['cost'] < min_cost:
                        min_cost = self.assignments[robot_id]['cost']
                        min_robot_id = robot_id
                for robot_id in robot_ids:
                    if robot_id != min_robot_id:
                        del self.assignments[robot_id]
                
    def update_paths(self, t):
        for robot_id in self.assignments.keys():
            robot = [robot for robot in self.robots if robot.id == robot_id][0]
            robot.move(self.targets[self.assignments[robot_id]['target']])
            
    def calculate_cost(self, robot, t):
        return np.linalg.norm(robot.pos - self.targets[t])

    def plot(self):
        plt.figure()
        for robot in self.robots:
            x = [p[0] for p in robot.path]
            y = [p[1] for p in robot.path]
            plt.plot(x, y, 'o-')
        for target in self.targets:
            plt.plot(target[0], target[1], 'rx')
        plt.xlim(-10, 10)
        plt.ylim(-10, 10)
        plt.show()

# 创建机器人和目标
robots = [Robot(1, np.array([0, 0])), Robot(2, np.array([0, 0]))]
targets = [np.array([2, 2]), np.array([-2, -2])]

# 运行CBCA算法
CBCA = CBCA(robots, targets)
CBCA.run()