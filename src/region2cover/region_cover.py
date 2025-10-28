# =============================================================================
#     Copyright (C) 2021-2024 Wageningen University - All Rights Reserved
#                      Author: Gonzalo Mier
#                         BSD-3 License
# =============================================================================

import math
import fields2cover as f2c
import datetime


def vector_scale(v, scalar):
    return (v[0] * scalar, v[1] * scalar)


def vector_add(v1, v2):
    return (v1[0] + v2[0], v1[1] + v2[1])


def vector_subtract(v1, v2):  # v2->v1
    return (v1[0] - v2[0], v1[1] - v2[1])


class RegionCover:
    def __init__(self, num_rows, num_cols, pos1, pos2, pos3, pos4, is_plot: bool = True):
        self.is_plot = is_plot
        self.all_path = []
        self.x1, self.y1 = pos1[0], pos1[1]
        self.x2, self.y2 = pos4[0], pos4[1]
        self.x3, self.y3 = pos3[0], pos3[1]
        self.x4, self.y4 = pos2[0], pos2[1]
        self.num_rows = num_rows
        self.num_cols = num_cols
        # TODO:计数使用，为了逻辑清晰加的无用变量
        self.cnt = -1
        self.start_point_list: list = []
        # 区域覆盖任务是否结束
        self.is_finish_task: bool = False

    # a-------------b
    # |             |
    # d-------------c
    def divide_regions(self):
        A = (self.x1, self.y1)
        B = (self.x2, self.y2)
        D = (self.x4, self.y4)

        AB = vector_subtract(B, A)
        AD = vector_subtract(D, A)

        row_length = vector_scale(AB, 1.0 / self.num_cols)
        col_length = vector_scale(AD, 1.0 / self.num_rows)

        regions = []

        for i in range(self.num_rows):
            for j in range(self.num_cols):
                corner1 = vector_add(A, vector_add(vector_scale(row_length, j), vector_scale(col_length, i)))
                corner2 = vector_add(corner1, row_length)
                corner3 = vector_add(corner2, col_length)
                corner4 = vector_add(corner1, col_length)

                regions.append((corner1, corner2, corner3, corner4))

        return regions

    def pathcheck(self, path, dis):
        checked_path = []
        last_point = (path[0].point.getX(), path[0].point.getX(), path[0].point.getX())
        for point in path:
            checked_path.append(point)
        return checked_path

    def cover_run(self, uav_velocity:float, turning_radius:float, log_time:str, cov_width:float=200):
        self.all_path = []

        robot = f2c.Robot(2.0, cov_width)  # 宽度 覆盖宽度 
        robot.setCruiseVel(uav_velocity)  # 巡航速度
        robot.setMinTurningRadius(turning_radius)  # m
        robot.setMaxDiffCurv(0.1)  # 1/m^2 最大曲率变化率  TODO:0.2 as default

        const_hl = f2c.HG_Const_gen()
        bf = f2c.SG_BruteForce()
        snake_sorter = f2c.RP_Snake()
        path_planner = f2c.PP_PathPlanning()
        dubins_cc = f2c.PP_DubinsCurvesCC()

        regions = self.divide_regions()

        i = 0
        for region in regions:
            cells = f2c.Cells(
                f2c.Cell(
                    f2c.LinearRing(
                        f2c.VectorPoint(
                            [
                                f2c.Point(region[0][0], region[0][1]),
                                f2c.Point(region[1][0], region[1][1]),
                                f2c.Point(region[2][0], region[2][1]),
                                f2c.Point(region[3][0], region[3][1]),
                                f2c.Point(region[0][0], region[0][1]),
                            ]
                        )
                    )
                )
            )

            no_hl = const_hl.generateHeadlands(cells, 3.0 * robot.getWidth())
            swaths = bf.generateSwaths(math.pi, robot.getCovWidth(), no_hl.getGeometry(0))
            swaths = snake_sorter.genSortedSwaths(swaths, 1)
            swaths.at(0).getPath().exportToWkt()

            path_dubins_cc = path_planner.planPath(robot, swaths, dubins_cc)

            self.all_path.append(path_dubins_cc)
            self.start_point_list.append(path_dubins_cc[0])

            if self.is_plot:
                i += 1
                self.plot_path(i, cells, no_hl, path_dubins_cc, swaths, log_time)

    def plot_path(self, id, cells, no_hl, path_dubins_cc, swaths, log_time):
        f2c.Visualizer.figure()
        f2c.Visualizer.plot(cells)
        f2c.Visualizer.plot(no_hl)
        f2c.Visualizer.plot(path_dubins_cc)
        f2c.Visualizer.plot(swaths)
        f2c.Visualizer.save("../pic/" + log_time + "Region_Cover_Dubins_CC" + str(id) + ".png")


if __name__ == "__main__":
    # usage
    dis = 41.67 / 30
    region_cover = RegionCover(1, 1, [8081, 0, 3669], [8081, 0, 2636], [9519, 0, 2636], [9519, 0, 3669], is_plot=True)
    region_cover.cover_run(log_time='1')
    path_dubins_cc = region_cover.all_path[0]
    print("size:", path_dubins_cc.size())
    for i in range(path_dubins_cc.size() - 1):
        last_x = path_dubins_cc[i].point.getX()
        x = path_dubins_cc[i + 1].point.getX()
        if abs(last_x - x) > 10:
            print("i", i, "last_x", last_x, "x", x)