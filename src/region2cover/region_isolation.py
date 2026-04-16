"""
区域隔离/封控模块

提供矩形区域内圆形封控区域规划功能。
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from .circle_path import UavPath


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
    在矩形区域内生成多个非重叠圆形封控区域的圆心坐标。

    Args:
        circle_num: 圆的数量
        rect_width: 矩形宽度
        rect_height: 矩形高度
        rect_center_x: 矩形中心x坐标
        rect_center_y: 矩形中心y坐标
        min_circle_radius: 最小圆半径
        max_circle_radius: 最大圆半径

    Returns:
        (circle_radius, circle_centers): 圆半径和圆心坐标列表
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

    # 根据面积比例确认覆盖模式
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

    def generate_non_intersecting_circle():
        """生成与非重叠圆不相交的新圆心"""
        for cx, cy in circle_centers:
            for angle in np.linspace(0, 2 * np.pi, 100):
                x = cx + 2 * circle_radius * np.cos(angle)
                y = cy + 2 * circle_radius * np.sin(angle)
                if is_valid_circle(x, y):
                    return x, y
        return None

    def is_valid_circle(x, y):
        """检查圆是否与现有圆重叠"""
        for cx, cy in circle_centers:
            if np.sqrt((cx - x) ** 2 + (cy - y) ** 2) < 2 * circle_radius:
                return False
        return True

    # 选择填充模式
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

    # 填充剩余的圆心
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
    绘制矩形和圆形密铺图案。

    Args:
        circle_num: 圆的数量
        rect_width: 矩形宽度
        rect_height: 矩形高度
        rect_center_x: 矩形中心x坐标
        rect_center_y: 矩形中心y坐标
        is_plot: 是否显示图形

    Returns:
        circle_centers: 圆心坐标列表
    """
    circle_radius, circle_centers = generate_circles_in_rectangle(
        circle_num, rect_width, rect_height, rect_center_x, rect_center_y
    )

    if is_plot:
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

        ax.set_aspect("equal")
        margin = max(rect_width, rect_height) * 0.1
        ax.set_xlim(rect_center_x - rect_width / 2 - margin, rect_center_x + rect_width / 2 + margin)
        ax.set_ylim(rect_center_y - rect_height / 2 - margin, rect_center_y + rect_height / 2 + margin)
        ax.grid(True, linestyle="--", alpha=0.3)
        ax.axhline(y=0, color="k", linestyle="-", alpha=0.3)
        ax.axvline(x=0, color="k", linestyle="-", alpha=0.3)

        plt.savefig("circles_and_rectangle.png")
        plt.show()

    return circle_centers
