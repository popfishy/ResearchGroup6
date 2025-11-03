import numpy as np
import dubins
import matplotlib.pyplot as plt


class UavPath:
    def __init__(self):
        self.frequency = 30  # hz
        self.uav_speed = 30  # m/s

    def generate_dubins_path(self, start, end, radius):
        # Calculate the Dubins path
        path = dubins.shortest_path(start, end, radius)
        # 计算轨迹点数量
        num_points = self.calculate_num_points(self.frequency, self.uav_speed, path.path_length())
        # Sample the path at evenly spaced intervals
        configurations, _ = path.sample_many(path.path_length() / num_points)

        return configurations

    def generate_circular_path(self, target_x, target_y, target_theta, turning_radius):
        # 计算圆心坐标
        center_x = target_x - turning_radius * np.sin(target_theta)
        center_y = target_y + turning_radius * np.cos(target_theta)

        # 计算轨迹点的数量
        num_points = self.calculate_num_points(self.frequency, self.uav_speed, 2 * np.pi * turning_radius)

        # 生成轨迹点
        angles = np.linspace(target_theta, target_theta + 2 * np.pi, num_points, endpoint=True)
        path_x = center_x + turning_radius * np.cos(angles)
        path_y = center_y + turning_radius * np.sin(angles)
        path_theta = angles % (2 * np.pi)  # 保持角度在 [0, 2*pi) 范围内

        return path_x, path_y, path_theta

    def calculate_target_point(self, center_x, center_y, turning_radius, initial_theta):
        # 计算圆上某一点的坐标
        target_x = center_x + turning_radius * np.cos(initial_theta)
        target_y = center_y + turning_radius * np.sin(initial_theta)

        # 计算方向
        target_theta = initial_theta + np.pi / 2  # 切线方向

        return target_x, target_y, target_theta

    def calculate_num_points(self, frequency, speed, path_length):
        # 计算每个位置命令的时间间隔
        delta_t = 1 / frequency
        # 计算无人机在每个时间间隔内的位移
        delta_s = speed * delta_t
        # 计算轨迹点的数量
        num_points = int(path_length / delta_s)

        return num_points

    def plot_circular_path(self, start_x, start_y, start_theta, center_x, center_y, turning_radius):
        # 计算圆上某一点的坐标和方向
        initial_theta = -1 / 3 * np.pi  # 选择任意一个初始角度
        target_x, target_y, target_theta = self.calculate_target_point(
            center_x, center_y, turning_radius, initial_theta
        )

        # 定义起始和结束配置
        start = (start_x, start_y, start_theta)
        end = (target_x, target_y, target_theta)

        path = self.generate_dubins_path(start, end, turning_radius)
        x_points = [point[0] for point in path]
        y_points = [point[1] for point in path]

        # 生成圆形轨迹
        path_x, path_y, path_theta = self.generate_circular_path(target_x, target_y, target_theta, turning_radius)

        # 绘制轨迹
        plt_figure = True
        if plt_figure:
            plt.figure(figsize=(8, 8))
            plt.plot(x_points, y_points, "b", label="Dubins Path")  # 绘制杜宾斯路径
            plt.plot(path_x, path_y, "r", label="Circular Path")  # 绘制圆形路径

            # 绘制起点
            plt.scatter(start_x, start_y, color="green", label="Start Point")
            plt.scatter(target_x, target_y, color="orange", label="Target Point")

            # 添加图例和标签
            plt.legend()
            plt.xlabel("X-axis")
            plt.ylabel("Y-axis")
            plt.title("Combined Paths")
            plt.axis("equal")  # 保持比例一致
            plt.grid(True)
            plt.show()

            # 打印dubins轨迹
            for point in path:
                print(f"x: {point[0]:.2f}, y: {point[1]:.2f}, theta: {point[2]:.2f}")

            print("--------------------------------------")

            # 打印圆形轨迹
            for i in range(len(path_x)):
                print(f"Point {i+1}: x={path_x[i]:.2f}, y={path_y[i]:.2f}, theta={path_theta[i]:.2f}")


if __name__ == "__main__":

    start_x, start_y, start_theta = 0.0, 0.0, 0.0
    center_x, center_y, turning_radius = 100.0, 100.0, 50.0

    uav_path = UavPath()
    uav_path.plot_circular_path(start_x, start_y, start_theta, center_x, center_y, turning_radius)
