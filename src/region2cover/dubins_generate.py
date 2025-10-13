import numpy as np
import dubins
import matplotlib.pyplot as plt


def generate_dubins_path(start, end, radius, num_points):
    # Calculate the Dubins path
    path = dubins.shortest_path(start, end, radius)

    # Sample the path at evenly spaced intervals
    configurations, _ = path.sample_many(path.path_length() / num_points)

    return configurations


def main():
    # Start and end coordinates with heading angles
    start_x, start_y, start_theta = 0.0, 0.0, 0.0
    end_x, end_y, end_theta = 100.0, 100.0, 2 * np.pi
    radius = 50.0
    num_points = 200

    # Define the start and end configuration
    start = (start_x, start_y, start_theta)
    end = (end_x, end_y, end_theta)

    # Generate the Dubins path
    path = generate_dubins_path(start, end, radius, num_points)

    # Print the path points
    for point in path:
        print(f"x: {point[0]:.2f}, y: {point[1]:.2f}, theta: {point[2]:.2f}")
        # 绘制路径
    x_points = [point[0] for point in path]
    y_points = [point[1] for point in path]

    plt.plot(x_points, y_points, marker="o")  # 绘制路径
    plt.title("Dubins Path")
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.axis("equal")  # 保持比例
    plt.grid()
    plt.show()  # 显示图形


if __name__ == "__main__":
    main()
