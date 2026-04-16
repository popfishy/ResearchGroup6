"""
RegionCover 模块测试脚本

用法:
    python -m region2cover.test_region_cover
"""

from region2cover.region_cover import RegionCover


if __name__ == "__main__":
    # 使用示例
    dis = 41.67 / 30
    region_cover = RegionCover(
        1, 1,
        [8081, 0, 3669],
        [8081, 0, 2636],
        [9519, 0, 2636],
        [9519, 0, 3669],
        is_plot=True
    )
    region_cover.cover_run(log_time='1')
    path_dubins_cc = region_cover.all_path[0]
    print("size:", path_dubins_cc.size())
    for i in range(path_dubins_cc.size() - 1):
        last_x = path_dubins_cc[i].point.getX()
        x = path_dubins_cc[i + 1].point.getX()
        if abs(last_x - x) > 10:
            print("i", i, "last_x", last_x, "x", x)
