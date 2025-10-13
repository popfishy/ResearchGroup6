import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import dubins


def generate_circles_in_rectangle(
    circle_num,
    rect_width,
    rect_height,
    rect_center_x,
    rect_center_y,
    min_circle_radius: float = 100,
    max_circle_radius: float = 500,
):
    """
    参数:
    rect_width: float, 矩形宽度
    rect_height: float, 矩形高度
    rect_center_x: float, 矩形中心x坐标
    rect_center_y: float, 矩形中心y坐标
    min_circle_radius: float, 最小圆半径
    max_circle_radius: float, 最大圆半径

    返回:
    circle_centers: list of tuples, 所有有效圆形的中心坐标
    """
    # 首先确定封控半径
    rect_area = rect_height * rect_width
    expect_circle_area = rect_area / circle_num
    expect_circle_radius = np.sqrt(expect_circle_area / np.pi)
    if expect_circle_radius < min_circle_radius:
        circle_radius = min_circle_radius
    elif expect_circle_radius > max_circle_radius:
        circle_radius = max_circle_radius
    else:
        circle_radius = expect_circle_radius
    print(f"计算得到最佳圆半径为:{circle_radius}")

    # 首先根据面积比例确认覆盖模式
    circle_area = circle_num * np.pi * circle_radius**2
    ratio = circle_area / rect_area
    ratio_min = np.pi / 4 + 0.05
    ratio_medium = np.pi / 2 / np.sqrt(3)
    ratio_max = 1.4
    ratio_max_pro = 1.5

    # 计算矩形的边界
    x_min = rect_center_x - rect_width / 2
    x_max = rect_center_x + rect_width / 2
    y_min = rect_center_y - rect_height / 2
    y_max = rect_center_y + rect_height / 2

    circle_centers = []

    def add_circle(x, y):
        if len(circle_centers) < circle_num:
            circle_centers.append((x, y))

    def fill_circles(dx, dy, offset_x=0):
        cols = int(rect_width / dx) + 1
        rows = int(rect_height / dy) + 1
        start_x = x_min + circle_radius
        start_y = y_min + circle_radius

        for i in range(rows):
            for j in range(cols):
                x = start_x + j * dx + (offset_x if i % 2 else 0)
                y = start_y + i * dy
                if (
                    x + circle_radius > x_min
                    and x - circle_radius < x_max
                    and y + circle_radius > y_min
                    and y - circle_radius < y_max
                ):
                    add_circle(x, y)
                if len(circle_centers) >= circle_num:
                    return

    # 如果圆心数量不足，随机填充剩余的圆心
    def generate_non_intersecting_circle():
        for cx, cy in circle_centers:
            for angle in np.linspace(0, 2 * np.pi, 100):
                x = cx + 2 * circle_radius * np.cos(angle)
                y = cy + 2 * circle_radius * np.sin(angle)
                if is_valid_circle(x, y):
                    return x, y
        return None

    def is_valid_circle(x, y):
        for cx, cy in circle_centers:
            if np.sqrt((cx - x) ** 2 + (cy - y) ** 2) < 2 * circle_radius:
                return False
        return True

    ratio = (circle_num * np.pi * circle_radius**2) / rect_area
    ratio_min = np.pi / 4 + 0.05
    ratio_medium = np.pi / 2 / np.sqrt(3)
    ratio_max = 1.4
    ratio_max_pro = 1.5

    if ratio <= ratio_min:
        fill_circles(circle_radius * 2, circle_radius * 2)
    elif ratio <= ratio_medium:
        fill_circles(circle_radius * 2, circle_radius * np.sqrt(3), circle_radius)
    elif ratio <= ratio_max:
        fill_circles(circle_radius * 2, circle_radius * np.sqrt(3), -circle_radius)
    elif ratio <= ratio_max_pro:
        fill_circles(circle_radius * 3, circle_radius * np.sqrt(3) / 2, circle_radius * 1.5)
    else:
        fill_circles(circle_radius * 3, circle_radius * np.sqrt(3) / 2, circle_radius * 1.5)

    while len(circle_centers) < circle_num:
        new_circle = generate_non_intersecting_circle()
        if new_circle:
            add_circle(*new_circle)
        else:
            break

    return circle_radius, circle_centers


def plot_circles_and_rectangle(
    circle_num, rect_width, rect_height, rect_center_x, rect_center_y, is_plot: bool = False
):
    """
    绘制矩形和圆形密铺图案
    """
    # 获取圆心坐标
    circle_radius, circle_centers = generate_circles_in_rectangle(
        circle_num, rect_width, rect_height, rect_center_x, rect_center_y
    )

    if is_plot:
        # 创建图形
        fig, ax = plt.subplots(figsize=(10, 10))

        # 绘制矩形
        rect = Rectangle(
            (rect_center_x - rect_width / 2, rect_center_y - rect_height / 2),
            rect_width,
            rect_height,
            fill=False,
            color="blue",
            linewidth=2,
        )
        ax.add_patch(rect)

        # 绘制所有圆
        for center in circle_centers:
            circle = plt.Circle(center, circle_radius, fill=False, color="red")
            ax.add_patch(circle)

        # 设置坐标轴比例相等
        ax.set_aspect("equal")

        # 设置图形范围（添加边距）
        margin = max(rect_width, rect_height) * 0.1
        ax.set_xlim(rect_center_x - rect_width / 2 - margin, rect_center_x + rect_width / 2 + margin)
        ax.set_ylim(rect_center_y - rect_height / 2 - margin, rect_center_y + rect_height / 2 + margin)

        # 显示网格
        ax.grid(True, linestyle="--", alpha=0.3)

        # 显示坐标轴
        ax.axhline(y=0, color="k", linestyle="-", alpha=0.3)
        ax.axvline(x=0, color="k", linestyle="-", alpha=0.3)

        plt.savefig("circles_and_rectangle.png")  # Save the plot as a PNG file
        plt.show()

    return circle_centers


class UavPath:
    def __init__(self):
        self.frequency = 30  # hz
        self.uav_speed = 41.67  # m/s
        self.dubins_path = []
        self.circle_path = []

        # 计数使用
        self.dubins_cnt = -1
        self.is_dubins_finish = False
        self.circle_cnt = -1

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
        # 计算圆上某一点的坐标
        target_x = center_x + turning_radius * np.cos(initial_theta)
        target_y = center_y + turning_radius * np.sin(initial_theta)

        # 计算方向
        target_theta = np.pi / 2 - initial_theta  # 切线方向

        return target_x, target_y, target_theta

    def calculate_num_points(self, frequency, speed, path_length):
        # 计算每个位置命令的时间间隔
        delta_t = 1 / frequency
        # 计算无人机在每个时间间隔内的位移
        delta_s = speed * delta_t
        # 计算轨迹点的数量
        num_points = int(path_length / delta_s)

        return num_points

    def calculate_path(
        self, start_x, start_y, start_theta, center_x, center_y, circle_radius, plt_figure: bool = False
    ):
        # 计算圆上某一点的坐标和方向
        initial_theta = -1 / 3 * np.pi  # 选择任意一个初始角度
        target_x, target_y, target_theta = self.calculate_target_point(center_x, center_y, circle_radius, initial_theta)

        # 定义起始和结束配置
        start = (start_x, start_y, start_theta)
        end = (target_x, target_y, target_theta)

        self.dubins_path = self.generate_dubins_path(start, end, circle_radius)
        x_points = [point[0] for point in self.dubins_path]
        y_points = [point[1] for point in self.dubins_path]

        # 生成圆形轨迹
        self.circle_path, path_x, path_y, path_theta = self.generate_circular_path(
            target_x, target_y, target_theta, circle_radius
        )

        # 绘制轨迹
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
            for point in self.dubins_path:
                print(f"x: {point[0]:.2f}, y: {point[1]:.2f}, theta: {point[2]:.2f}")

            print("--------------------------------------")

            # 打印圆形轨迹
            for i in range(len(path_x)):
                print(f"Point {i+1}: x={path_x[i]:.2f}, y={path_y[i]:.2f}, theta={path_theta[i]:.2f}")


if __name__ == "__main__":
    # test1
    # cirecle_num = 150
    # rect_width = 5000
    # rect_height = 6500
    # rect_center_x = 0
    # rect_center_y = 0

    # # 调用函数并获取圆心坐标
    # centers = plot_circles_and_rectangle(
    #     cirecle_num, rect_width, rect_height, rect_center_x, rect_center_y, is_plot=True
    # )

    # # 打印圆心坐标（如果需要）
    # print(f"Total number of circles: {len(centers)}")

    # test2
    start_x, start_y, start_theta = 0.0, 0.0, 0.0
    center_x, center_y, turning_radius = 100.0, 100.0, 50.0
    circle_radius = 100

    uav_path = UavPath()
    uav_path.calculate_path(
        start_x, start_y, start_theta, center_x, center_y, circle_radius, turning_radius, plt_figure=True
    )
