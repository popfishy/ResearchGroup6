"""
圆形巡逻路径生成模块

提供Dubins路径和圆形巡逻路径的生成功能。
"""

import numpy as np
import dubins
import matplotlib.pyplot as plt


class UavPath:
    """UAV路径生成器，支持Dubins路径和圆形巡逻"""

    def __init__(self, frequency: float = 30, uav_speed: float = 41.67):
        """
        初始化UAV路径生成器。

        Args:
            frequency: 控制频率 (Hz)
            uav_speed: 无人机速度 (m/s)
        """
        self.frequency = frequency
        self.uav_speed = uav_speed
        self.dubins_path = []
        self.circle_path = []

    def generate_dubins_path(self, start, end, radius):
        """
        生成Dubins路径。

        Args:
            start: 起始配置 (x, y, theta)
            end: 终点配置 (x, y, theta)
            radius: 最小转弯半径

        Returns:
            路径点配置列表
        """
        path = dubins.shortest_path(start, end, radius)
        num_points = self.calculate_num_points(self.frequency, self.uav_speed, path.path_length())
        configurations, _ = path.sample_many(path.path_length() / num_points)
        return configurations

    def generate_circular_path(self, target_x, target_y, target_theta, turning_radius):
        """
        生成圆形巡逻路径。

        Args:
            target_x: 目标点x坐标
            target_y: 目标点y坐标
            target_theta: 目标点朝向
            turning_radius: 转弯半径

        Returns:
            (path, path_x, path_y, path_theta): 路径元组
        """
        # 计算圆心坐标
        center_x = target_x - turning_radius * np.cos(np.pi / 2 - target_theta)
        center_y = target_y - turning_radius * np.sin(np.pi / 2 - target_theta)

        # 计算轨迹点的数量
        num_points = self.calculate_num_points(self.frequency, self.uav_speed, 2 * np.pi * turning_radius)

        # 生成轨迹点
        angles = np.linspace(target_theta, target_theta + 2 * np.pi, num_points, endpoint=True)
        path_x = center_x + turning_radius * np.cos(np.pi / 2 - angles)
        path_y = center_y + turning_radius * np.sin(np.pi / 2 - angles)
        path_theta = np.pi / 2 - angles  # 保持角度在 [0, 2*pi) 范围内

        # 创建包含 (x, y, theta) 的列表
        path = [(path_x[i], path_y[i], path_theta[i]) for i in range(num_points)]

        return path, path_x, path_y, path_theta

    def calculate_target_point(self, center_x, center_y, turning_radius, initial_theta):
        """
        计算圆上某一点的坐标和方向。

        Args:
            center_x: 圆心x坐标
            center_y: 圆心y坐标
            turning_radius: 转弯半径
            initial_theta: 初始角度

        Returns:
            (target_x, target_y, target_theta): 目标点坐标和朝向
        """
        target_x = center_x + turning_radius * np.cos(initial_theta)
        target_y = center_y + turning_radius * np.sin(initial_theta)
        target_theta = np.pi / 2 - initial_theta  # 切线方向
        return target_x, target_y, target_theta

    def calculate_num_points(self, frequency, speed, path_length):
        """
        根据路径长度和速度计算轨迹点数量。

        Args:
            frequency: 控制频率 (Hz)
            speed: 速度 (m/s)
            path_length: 路径长度 (m)

        Returns:
            轨迹点数量
        """
        delta_t = 1 / frequency
        delta_s = speed * delta_t
        num_points = int(path_length / delta_s)
        return num_points

    def calculate_path(self, start_x, start_y, start_theta, center_x, center_y, circle_radius, plt_figure: bool = False):
        """
        计算完整路径：Dubins接近路径 + 圆形巡逻路径。

        Args:
            start_x: 起始x坐标
            start_y: 起始y坐标
            start_theta: 起始朝向
            center_x: 圆心x坐标
            center_y: 圆心y坐标
            circle_radius: 圆巡逻半径
            plt_figure: 是否绘制图形
        """
        initial_theta = -1 / 3 * np.pi
        target_x, target_y, target_theta = self.calculate_target_point(
            center_x, center_y, circle_radius, initial_theta
        )

        start = (start_x, start_y, start_theta)
        end = (target_x, target_y, target_theta)

        self.dubins_path = self.generate_dubins_path(start, end, circle_radius)
        x_points = [point[0] for point in self.dubins_path]
        y_points = [point[1] for point in self.dubins_path]

        self.circle_path, path_x, path_y, path_theta = self.generate_circular_path(
            target_x, target_y, target_theta, circle_radius
        )

        if plt_figure:
            plt.figure(figsize=(8, 8))
            plt.plot(x_points, y_points, "b", label="Dubins Path")
            plt.plot(path_x, path_y, "r", label="Circular Path")
            plt.scatter(start_x, start_y, color="green", label="Start Point")
            plt.scatter(target_x, target_y, color="orange", label="Target Point")
            plt.legend()
            plt.xlabel("X-axis")
            plt.ylabel("Y-axis")
            plt.title("Combined Paths")
            plt.axis("equal")
            plt.grid(True)
            plt.show()

            for point in self.dubins_path:
                print(f"x: {point[0]:.2f}, y: {point[1]:.2f}, theta: {point[2]:.2f}")
            print("--------------------------------------")
            for i in range(len(path_x)):
                print(f"Point {i+1}: x={path_x[i]:.2f}, y={path_y[i]:.2f}, theta={path_theta[i]:.2f}")
