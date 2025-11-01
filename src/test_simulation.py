import rospy
import numpy as np
import threading
from dataclasses import dataclass
import socket
import struct
import random
import os
import datetime
import time
from pynput import keyboard
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from scipy.spatial.transform import Rotation as R
from tf.transformations import euler_from_quaternion

from group6_interface.msg import AgentData, AgentsData, TargetData, TargetsData, RegionData, RegionsData
from xfd_allocation.scripts.CBPA.lib.CBPA import CBPA
from xfd_allocation.scripts.CBPA.lib.Robot import Robot
from xfd_allocation.scripts.CBPA.lib.Task import Task
from xfd_allocation.scripts.CBPA.lib.Region import Region
from xfd_allocation.scripts.CBPA.lib.WorldInfo import WorldInfo
from xfd_allocation.scripts.CBPA.lib.CBPA_REC import CBPA_REC

from communication.scripts.GVF_ode import GVF_ode
from region2cover.region_isolation import UavPath, generate_circles_in_rectangle
from region2cover.region_cover import RegionCover
from evaluate import write_results_to_excel


UBUNTU_IP = "192.168.1.101"
Sim_IP = "192.168.1.102"  # 仿真系统IP
UAV_PORT = 12380
EXIT_PROGRAM = False


UAV_NUM: int = 200
DAMAGE_RATIO: float = 0.5
IS_RANDOM_DAMAGE = True
TARGET_NUM = int(UAV_NUM / 6)
RATE = 60
# pos1-------------pos4
#  |                |
# pos2-------------pos3
RegionList: list = [
    Region(1, [8081, 0, 3752], [8081, 0, 2802], [9519, 0, 2802], [9519, 0, 3752]),
    Region(2, [6007, 0, 2908], [6007, 0, 1840], [7141, 0, 1840], [7141, 0, 2908]),
    Region(3, [7352, 0, 2745], [7352, 0, 1579], [8882, 0, 1579], [8882, 0, 2745]),
    Region(4, [7352, 0, 1532], [7352, 0, 120], [9145, 0, 120], [9145, 0, 1532]),
]
RobotList: list = []
TaskList: list = []
TaskDefenceList: list = []
# 需要执行打击任务的XFD编号
AttckRobotSet: set = set()
AttckStartTime: map = {}
AttckEndTime: map = {}

AttackTargetStartTime: map = {}
AttackTargetEndTime: map = {}
# 随机损毁的XFD编号
DestoryRobotSet: set = set()

robots_lock = threading.Lock()
tasks_lock = threading.Lock()
regions_lock = threading.Lock()
command_lock = threading.Lock()

O1_recognition_efficiency: float = None
D1_planning_efficiency: float = None
A1_attack_efficiency: float = None

Detect_step_start_time: rospy.Time = None
Detect_step_end_time_map: map = {}
Cover_step_end_time_map: map = {}
ReAllocationTime: rospy.Time = rospy.Time()

random.seed(0)


def on_press(key):
    global EXIT_PROGRAM
    try:
        if key.char == "q":  # 检测 'q' 键是否被按下
            rospy.loginfo("退出程序中...")
            EXIT_PROGRAM = True
    except AttributeError:
        pass


def plot_regions(region_list):
    """
    绘制矩形区域
    eg: plot_regions(RegionList)
    """
    plt.figure()
    for region in region_list:
        # 提取矩形的四个角点
        x_coords = [region.pos1[0], region.pos2[0], region.pos3[0], region.pos4[0], region.pos1[0]]
        y_coords = [region.pos1[2], region.pos2[2], region.pos3[2], region.pos4[2], region.pos1[2]]

        # 使用不同颜色填充矩形
        plt.fill(x_coords, y_coords, alpha=0.5, label=f"Region {region.region_id}")

    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.title("Regions Visualization")
    plt.legend()
    plt.grid()
    plt.axis("equal")
    plt.show()


def euler_to_quaternion(r, p, y):
    # 创建旋转对象
    rotation = R.from_euler("xyz", [r, p, y])
    # 转换为四元数
    quaternion = rotation.as_quat()  # 返回 (x, y, z, w) 格式
    return quaternion


class UavSimProcess:
    def __init__(self) -> None:
        self.robots_sub = rospy.Subscriber("/missile_data", AgentsData, self.robots_callback)
        self.targets_sub = rospy.Subscriber("/target_data", TargetsData, self.targets_callback)
        # self.regions_sub = rospy.Subscriber("/region_data", RegionsData, self.regions_callback)

        # 是否进行随机损毁
        self.random_damage_flag = IS_RANDOM_DAMAGE

        self.stop_detection_event = threading.Event()  # 用于控制线程停止
        self.stop_isolation_event = threading.Event()

        self.gvf_ode_set: map = {}
        self.region_conver_set: map = {}
        self.isolate_path_set: map = {}
        self.CBPA_solver = CBPA_REC()

        self.is_detection_step_init = False
        self.is_detection_step_finish = False
        self.is_attack_step_finish = False
        self.is_isolate_step_finish = False

        self.ros_start_time = rospy.get_rostime().secs
        self.log_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.debug = False

    def detect_step(self):
        global RobotList, TaskList, RegionList, TaskDefenceList
        global AttckRobotSet, Detect_step_start_time, Detect_step_end_time_map, Cover_step_end_time_map, ReAllocationTime
        while (len(RobotList) == 0) | (len(TaskList) == 0):
            rospy.loginfo(len(RobotList))
            continue
        robots_lock.acquire()
        tasks_lock.acquire()
        robot_list = RobotList.copy()
        task_list = TaskList.copy()
        task_defence_list = TaskDefenceList.copy()
        robots_lock.release()
        tasks_lock.release()

        def send_detection_data():
            rate = rospy.Rate(RATE)
            is_random_destory = False
            while not self.stop_detection_event.is_set():
                while not rospy.is_shutdown():
                    if self.stop_detection_event.is_set():  # 添加此行以检查停止事件
                        break
                    packed_data_list = []
                    for region_id, gvf_ode in self.gvf_ode_set.items():
                        gvf_ode.cnt = gvf_ode.cnt + 1
                        uav_id_list = RegionList[region_id - 1].uav_id_list
                        x_coords = gvf_ode.x_coords
                        y_coords = gvf_ode.y_coords
                        # assert len(uav_id_list) == len(x_coords)

                        if not gvf_ode.is_finish_pub:
                            p = gvf_ode.global_paths[0]
                            numpoints = p.shape[0]
                            for i in range(len(uav_id_list)):
                                if gvf_ode.cnt < numpoints:
                                    x = p[gvf_ode.cnt, i * 5] + gvf_ode.trajectory_list[0][0]
                                    z = p[gvf_ode.cnt, i * 5 + 1] + gvf_ode.trajectory_list[0][1]
                                    y = p[gvf_ode.cnt, i * 5 + 2]
                                    orientation = gvf_ode.orientation_list[0]
                                    rx, ry, rz, rw = euler_to_quaternion(r=0, p=orientation, y=0)
                                    packed_data = (uav_id_list[i], x, y, z, rx, ry, rz, rw)
                                    packed_data_list.append(packed_data)
                                if gvf_ode.cnt == (numpoints - 1):
                                    Detect_step_end_time_map[region_id] = rospy.get_rostime()
                                    gvf_ode.is_finish_pub = True
                                    is_random_destory = True
                        else:
                            region_cover = self.region_conver_set[region_id]
                            region_cover.cnt = region_cover.cnt + 1
                            # 根据 num_rows和num_cols判断区域
                            assert (region_cover.num_rows == 1) & (region_cover.num_cols == 1)
                            path_dubins_cc = region_cover.all_path[0]
                            # TODO： 分区域打击可修改至此处
                            if region_cover.cnt >= path_dubins_cc.size():
                                region_cover.is_finish_task = True
                                if not region_id in Cover_step_end_time_map:
                                    Cover_step_end_time_map[region_id] = rospy.get_rostime()
                            for i in range(len(x_coords)):
                                path_state = path_dubins_cc[int(np.mod(region_cover.cnt, path_dubins_cc.size()))]
                                # 坐标系不同
                                x = path_state.point.getX() + x_coords[i]
                                y = path_state.point.getZ() + 400
                                z = path_state.point.getY() + y_coords[i]
                                rx, ry, rz, rw = euler_to_quaternion(r=0, p=-path_state.angle, y=0)
                                packed_data = (uav_id_list[i], x, y, z, rx, ry, rz, rw)
                                packed_data_list.append(packed_data)
                                # 判断是否检测到task_defence_list，检测到立即打击  注意坐标系!
                                detected_tasks = self.find_air_defence_enemy(x, z, y, task_defence_list)
                                if len(detected_tasks) != 0:
                                    rospy.loginfo(f"检测到敌方防空火力,目标id为{detected_tasks[0].task_id},立即打击!")
                                    attack_data_list = []
                                    for detected_task in detected_tasks:
                                        task_defence_list.remove(detected_task)
                                        for i in range(detected_task.robot_need):
                                            uav_id = random.sample(uav_id_list, 1)[0]
                                            while uav_id in AttckRobotSet:
                                                uav_id = random.sample(uav_id_list, 1)[0]
                                            attack_data = (uav_id, detected_task.task_id)
                                            attack_data_list.append(attack_data)
                                    rospy.loginfo(f"任务分配结果(巡飞弹id,目标任务id): {attack_data_list}")
                                    self.send_attack_data(Sim_IP, UAV_PORT, 5, attack_data_list)
                    # 生成一个0到1之间的随机概率
                    if (self.random_damage_flag) & (is_random_destory) & (
                        len(DestoryRobotSet) <= int(UAV_NUM * DAMAGE_RATIO * 0.9)
                    ) == True:
                        probability = random.random()
                        if probability < len(task_defence_list) * 0.0035:
                            destory_num = random.randint(1, 3)
                            self.random_destory(destory_num)
                    command_lock.acquire()
                    self.send_uav_data(Sim_IP, UAV_PORT, 101, packed_data_list)
                    command_lock.release()
                    rate.sleep()

        # 初始化
        for region in RegionList:
            region.get_task_id_list(task_list)
            rospy.loginfo(f"region_id:{region.region_id},region.task_id_list:{region.task_id_list}")

        # 无人机分配给不同区域
        Detect_step_start_time = rospy.get_rostime()
        uav_allocation = self.divide_robot(RegionList, task_list, robot_list)
        rospy.loginfo(f"无人机预分组分配结果为(区域id,无人机数量): {uav_allocation}")

        # 生成区域侦查的路径
        self.region_conver_set = self.generate_region_conver_set(RegionList)
        rospy.loginfo(f"成功生成区域侦查路径!")

        # 生成分组分簇的路径
        self.gvf_ode_set = self.generate_gvf_ode_set(uav_allocation, robot_list, RegionList)
        rospy.loginfo(f"成功完成无人机群预分组分簇!")

        # 启动数据发送线程
        detection_thread = threading.Thread(target=send_detection_data)
        detection_thread.start()

        if self.random_damage_flag:
            rospy.sleep(20)
            self.random_destory(UAV_NUM * DAMAGE_RATIO * 0.8)
            rospy.loginfo(f"检测到无人机群发生损毁!")

            # 重新生成分簇路径
            robot_list = []
            rospy.sleep(0.5)
            robots_lock.acquire()
            robot_list = RobotList.copy()
            robots_lock.release()

            rospy.loginfo(f"剩余无人机数量为:{len(robot_list)}")
            start_time = rospy.get_rostime()
            uav_allocation = self.divide_robot(RegionList, task_list, robot_list)
            rospy.loginfo(f"无人机区域重分配结果为(区域id,无人机数量): {uav_allocation}")

            # 关闭线程
            self.stop_detection_event.set()
            detection_thread.join()
            self.stop_detection_event.clear()

            robots_lock.acquire()
            robot_list = RobotList.copy()
            robots_lock.release()
            # 生成区域侦查的路径
            self.region_conver_set = self.generate_region_conver_set(RegionList)
            rospy.loginfo(f"重新生成区域侦查路径!")
            self.gvf_ode_set = self.generate_gvf_ode_set(uav_allocation, robot_list, RegionList)
            rospy.loginfo(f"重新完成无人机群预分组分簇!")

            end_time = rospy.get_rostime()
            ReAllocationTime = end_time - start_time
            # rospy.loginfo(f"重分配时间为:{total}")

            # 关闭线程、再开启线程
            detection_thread = threading.Thread(target=send_detection_data)
            detection_thread.start()

    def attack_step(self):
        global RobotList, TaskList, RegionList, TaskDefenceList
        robots_lock.acquire()
        tasks_lock.acquire()
        robot_list = RobotList.copy()
        task_list = TaskList.copy()
        robots_lock.release()
        tasks_lock.release()

        task_list_valid = []
        for region in RegionList:
            for task_id in region.task_id_list:
                if task_id not in TaskDefenceList:
                    for task in task_list:
                        if (task.task_id == task_id) & (task.task_status):
                            task_list_valid.append(task)

        # 创建一个世界，每个列表都是[x,y,z]轴的[min, max]坐标
        WorldInfoTest = WorldInfo([0.0, 10000.0], [-200, 5000.0], [0.0, 1000.0])
        # path_list [target_id, uav_id]
        attack_list, idx_list, robot_list, task_list, allocation_score = self.xfd_allocation(
            robot_list, task_list_valid, WorldInfoTest
        )
        # 根据attack_list执行毁伤程序
        attack_data_list = []
        for attack_data in attack_list:
            if len(attack_data) >= 2:
                for i in range(len(attack_data) - 1):
                    data = (attack_data[i + 1], attack_data[0])
                    attack_data_list.append(data)
        rospy.loginfo(f"打击任务分配结果(巡飞弹id,目标任务id):{attack_data_list}")
        command_lock.acquire()
        self.send_attack_data(Sim_IP, UAV_PORT, 5, attack_data_list)
        command_lock.release()
        # self.write_data_to_file(attack_data_list, self.log_time + "attack.txt")

    def isolate_step(self):
        global RobotList, TaskList, RegionList, DestoryRobotSet, AttckRobotSet
        robots_lock.acquire()
        robot_list = RobotList.copy()
        robots_lock.release()

        def isolation_thread_func():
            rate = rospy.Rate(RATE)
            while not self.stop_isolation_event.is_set():
                while not rospy.is_shutdown():
                    if self.stop_isolation_event.is_set():  # 添加此行以检查停止事件
                        break
                    packed_data_list = []
                    for robot_id, isolate_path in self.isolate_path_set.items():
                        if not isolate_path.is_dubins_finish:
                            num_points = len(isolate_path.dubins_path)
                            isolate_path.dubins_cnt = isolate_path.dubins_cnt + 1
                            if isolate_path.dubins_cnt < num_points:
                                point = isolate_path.dubins_path[isolate_path.dubins_cnt]
                                x = point[0]
                                y = 400
                                z = point[1]
                                rx, ry, rz, rw = euler_to_quaternion(r=0, p=-point[2], y=0)
                                packed_data = (robot_id, x, y, z, rx, ry, rz, rw)
                                packed_data_list.append(packed_data)
                            else:
                                isolate_path.is_dubins_finish = True
                        else:
                            isolate_path.circle_cnt = isolate_path.circle_cnt + 1
                            point = isolate_path.circle_path[
                                np.mod(isolate_path.circle_cnt, len(isolate_path.circle_path))
                            ]
                            x = point[0]
                            y = 400
                            z = point[1]
                            rx, ry, rz, rw = euler_to_quaternion(r=0, p=-point[2], y=0)
                            packed_data = (robot_id, x, y, z, rx, ry, rz, rw)
                            packed_data_list.append(packed_data)
                    self.send_uav_data(Sim_IP, UAV_PORT, 101, packed_data_list)
                    rate.sleep()

        # 数据处理
        isolate_region = RegionList[0]  # 选择封控区域
        robot_alive_num = 0
        robot_alive_list = []
        for robot in robot_list:
            if (robot.robot_status) & (robot.robot_id not in AttckRobotSet) & (robot.robot_id not in DestoryRobotSet):
                robot_alive_num = robot_alive_num + 1
                robot_alive_list.append(robot)
        rospy.loginfo(f"剩余巡飞弹数量为: {len(robot_alive_list)}")

        circle_radius, circle_centers = generate_circles_in_rectangle(
            robot_alive_num,
            isolate_region.length,
            isolate_region.width,
            isolate_region.center[0],
            isolate_region.center[1],
        )

        # 任务分配数据处理
        task_list = [
            Task(task_id=idx, x=center[0], y=center[1], z=0, task_status=True, robot_need=1)
            for idx, center in enumerate(circle_centers)
        ]
        rospy.loginfo(f"封控圆的数量: {len(circle_centers)}")

        # 创建一个世界，每个列表都是[x,y,z]轴的[min, max]坐标
        WorldInfoTest = WorldInfo([0.0, 10000.0], [-200, 5000.0], [0.0, 1000.0])
        # path_list [target_id, uav_id]
        attack_list, idx_list, robot_list, task_list, _ = self.xfd_allocation(
            robot_alive_list, task_list, WorldInfoTest
        )
        rospy.loginfo(f"区域封控任务分配结果为(巡飞弹id,目标任务id):{attack_list}")

        # 读取现在位置信息
        robots_lock.acquire()
        robot_list = RobotList.copy()
        robots_lock.release()

        for id_map in attack_list:
            circle_center = circle_centers[id_map[0]]
            for robot in robot_list:
                if robot.robot_id == id_map[1]:
                    roll, pitch, yaw = euler_from_quaternion((robot.rx, robot.ry, robot.rz, robot.rw))
                    isolate_path = UavPath()
                    isolate_path.calculate_path(
                        start_x=robot.x,
                        start_y=robot.y,
                        start_theta=pitch,
                        center_x=circle_center[0],
                        center_y=circle_center[1],
                        circle_radius=circle_radius,
                    )
                    self.isolate_path_set[robot.robot_id] = isolate_path

        # 结束区域覆盖线程
        self.stop_detection_event.set()
        isolation_thread = threading.Thread(target=isolation_thread_func)
        isolation_thread.start()

        rospy.sleep(40)
        if self.random_damage_flag:
            robots_lock.acquire()
            robot_list = RobotList.copy()
            robots_lock.release()
            robot_num = len(robot_list)
            last_destory_num = int(UAV_NUM * DAMAGE_RATIO) - len(DestoryRobotSet)
            destory_num = robot_num if robot_num < last_destory_num else last_destory_num
            self.random_destory(destory_num)
            rospy.loginfo(f"检测到无人机群发生损毁!")

            rospy.sleep(1)
            robots_lock.acquire()
            robot_list = RobotList.copy()
            robots_lock.release()

            robot_alive_num = 0
            robot_alive_list = []
            for robot in robot_list:
                if (
                    (robot.robot_status)
                    & (robot.robot_id not in AttckRobotSet)
                    & (robot.robot_id not in DestoryRobotSet)
                ):
                    robot_alive_num = robot_alive_num + 1
                    robot_alive_list.append(robot)
            rospy.loginfo(f"发生损毁后,剩余巡飞弹数量为: {len(robot_alive_list)}")
            circle_radius, circle_centers = generate_circles_in_rectangle(
                robot_alive_num,
                isolate_region.length,
                isolate_region.width,
                isolate_region.center[0],
                isolate_region.center[1],
            )

            # 任务分配数据处理
            task_list = [
                Task(task_id=idx, x=center[0], y=center[1], z=0, task_status=True, robot_need=1)
                for idx, center in enumerate(circle_centers)
            ]
            rospy.loginfo(f"封控圆的数量:{len(circle_centers)}")

            # 创建一个世界，每个列表都是[x,y,z]轴的[min, max]坐标
            WorldInfoTest = WorldInfo([0.0, 10000.0], [-200, 5000.0], [0.0, 1000.0])
            # path_list [target_id, uav_id]
            attack_list, idx_list, robot_list, task_list, _ = self.xfd_allocation(
                robot_alive_list, task_list, WorldInfoTest
            )
            rospy.loginfo(f"区域封控任务重新分配结果为(巡飞弹id,目标任务id):{attack_list} ")

            # 读取现在位置信息
            robots_lock.acquire()
            robot_list = RobotList.copy()
            robots_lock.release()

            self.stop_isolation_event.set()
            isolation_thread.join()

            for id_map in attack_list:
                circle_center = circle_centers[id_map[0]]
                for robot in robot_list:
                    if robot.robot_id == id_map[1]:
                        roll, pitch, yaw = euler_from_quaternion((robot.rx, robot.ry, robot.rz, robot.rw))
                        isolate_path = UavPath()
                        isolate_path.calculate_path(
                            start_x=robot.x,
                            start_y=robot.y,
                            start_theta=pitch,
                            center_x=circle_center[0],
                            center_y=circle_center[1],
                            circle_radius=circle_radius,
                        )
                        self.isolate_path_set[robot.robot_id] = isolate_path

            self.stop_isolation_event.clear()
            isolation_thread = threading.Thread(target=isolation_thread_func)
            isolation_thread.start()

    # TODO:可以增加根据威胁程度等其他因素划分
    def divide_robot(self, region_list: list, task_list: list, robot_list: list):
        total_area = sum(region.area for region in region_list)  # Calculate total area
        allocation = {}  # Map to hold the number of UAVs for each region
        allocated_uavs = 0
        uav_num = len(robot_list)

        for i in range(len(region_list) - 1):  # Allocate for all but the last region
            proportion = region_list[i].area / total_area if total_area > 0 else 0
            num_uavs = int(uav_num * proportion)  # Allocate UAVs
            allocation[region_list[i].region_id] = num_uavs  # Store region ID and allocated UAVs
            uav_id_list = [i.robot_id for i in robot_list[allocated_uavs : allocated_uavs + num_uavs]]
            region_list[i].uav_id_list = uav_id_list
            allocated_uavs += num_uavs  # Update the total allocated UAVs
        # Allocate remaining UAVs to the last region
        remaining_uavs = uav_num - allocated_uavs
        uav_id_list = [i.robot_id for i in robot_list[allocated_uavs : allocated_uavs + remaining_uavs]]
        region_list[-1].uav_id_list = uav_id_list
        allocation[region_list[-1].region_id] = remaining_uavs  # Last region gets the rest

        return allocation  # Return the allocation result

    def generate_gvf_ode_set(self, uav_allocation: map, robot_list: list, region_list: list):
        gvf_ode_set = {}
        for region_id, uav_num in uav_allocation.items():
            trajectory_list = []

            # FIXME
            x_coords, y_coords = self.generate_drone_positions(uav_num, 50, 50)
            region = region_list[region_id - 1]
            x_init, y_init, z_init = [], [], []
            leader_id = region.uav_id_list[0]

            leader_uav = None  # 指定区域内uavs的leader
            for id in region.uav_id_list:
                for robot in robot_list:
                    if robot.robot_id == id:
                        leader_uav = robot
                        break
                if leader_uav != None:
                    break

            for robot in robot_list:
                if robot.robot_id in region.uav_id_list:
                    x_init.append(robot.x - leader_uav.x)
                    y_init.append(robot.y - leader_uav.y)
                    z_init.append(robot.z)

            gvf_ode = GVF_ode(region.uav_id_list[0], uav_num, x_coords, y_coords, x_init, y_init, z_init)
            start_traj = [leader_uav.x, leader_uav.y, leader_uav.z, 0]
            # rospy.loginfo(f"start_traj:{start_traj}")
            region_cover = self.region_conver_set[region_id]
            endx = region_cover.start_point_list[0].point.getX()
            endy = region_cover.start_point_list[0].point.getY()
            end_traj = [
                endx,
                endy,
                400,
                np.sqrt((leader_uav.x - endx) ** 2 + (leader_uav.y - endy) ** 2) / leader_uav.nom_velocity,
            ]
            # rospy.loginfo(f"end_traj:{end_traj}")
            trajectory_list.append(start_traj)
            trajectory_list.append(end_traj)
            gvf_ode.update_waypoint(trajectory_list)
            gvf_ode.calculate_path()
            gvf_ode_set[region_id] = gvf_ode
        return gvf_ode_set

    def generate_drone_positions(self, num_drones: int, x_dis: int, y_dis: int):
        x_coords = []
        y_coords = []

        rows = int(np.ceil(num_drones**0.5))  # 确定方阵的行数
        cols = (num_drones + rows - 1) // rows  # 确定方阵的列数

        for i in range(rows):
            for j in range(cols):
                if len(x_coords) < num_drones:
                    x_coord = j * x_dis
                    y_coord = i * y_dis
                    x_coords.append(x_coord)
                    y_coords.append(y_coord)

        return x_coords, y_coords

    def generate_region_conver_set(self, region_list: list):
        # 暂时默认区域分成1 * 1
        region_conver_set = {}
        num_rows = 1
        num_cols = 1
        for region in region_list:
            region_cover = RegionCover(
                num_rows, num_cols, region.pos1, region.pos2, region.pos3, region.pos4, is_plot=True
            )
            rows = int(np.ceil(len(region.uav_id_list) ** 0.5))
            cov_width = (rows - 1) * 50 + 100
            region_cover.cover_run(self.log_time, cov_width=cov_width)
            region_conver_set[region.region_id] = region_cover
        return region_conver_set

    def find_air_defence_enemy(self, x, y, z, task_list):
        """
        检测在给定坐标 (x, y) 范围内的敌方任务。

        参数:
        x (float): 侦查agent的x坐标
        y (float): 侦查agent的y坐标
        task_list (list): 包含敌方任务的列表

        返回:
        list: 被检测到的任务列表
        """
        detected_tasks = []
        detection_range = 100  # 侦查范围的一半

        for task in task_list:
            if (x - detection_range <= task.x <= x + detection_range) and (
                y - detection_range <= task.y <= y + detection_range
            ):
                detected_tasks.append(task)
        return detected_tasks

    def xfd_allocation(self, robot_list: list, task_list: list, WorldInfoTest: WorldInfo):
        path_list, idx_list, robot_list, task_list, score = self.CBPA_solver.solve_centralized(
            robot_list, task_list, WorldInfoTest
        )
        if self.debug:
            rospy.loginfo(f"path_list:{path_list}")
            rospy.loginfo(f"idx_list:{idx_list}")
            self.plot_robot_schedules(robot_list, robot_list, path_list, idx_list, WorldInfoTest)
        return path_list, idx_list, robot_list, task_list, score

    def plot_robot_schedules(self, robot_list, task_list, path_list, idx_list, WorldInfoTest):
        def set_axes_equal_xy(ax, WorldInfoTest, flag_3d: bool):
            """
            Make only x and y axes of 3D plot have equal scale. This is one possible solution to Matplotlib's
            ax.set_aspect('equal') and ax.axis('equal') not working for 3D.
            Reference: https://stackoverflow.com/questions/13685386/matplotlib-equal-unit-length-with-equal-aspect-ratio-z-axis-is-not-equal-to

            Input
            ax: a matplotlib axis, e.g., as output from plt.gca().
            flag_3d: boolean, True if it's 3D plot.
            """

            x_limits = WorldInfoTest.limit_x
            y_limits = WorldInfoTest.limit_y

            x_range = abs(x_limits[1] - x_limits[0])
            x_middle = np.mean(x_limits)
            y_range = abs(y_limits[1] - y_limits[0])
            y_middle = np.mean(y_limits)

            # The plot bounding box is a sphere in the sense of the infinity
            # norm, hence I call half the max range the plot radius.
            plot_radius = 0.5 * max([x_range, y_range])

            if flag_3d:
                ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
                ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
            else:
                ax.set_xlim([x_middle - plot_radius, x_middle + plot_radius])
                ax.set_ylim([y_middle - plot_radius, y_middle + plot_radius])

        if path_list:
            # 清空之前的图表
            # plt.close('all')
            plt.clf()
            num_robots = len(robot_list)
            num_tasks = len(task_list)

            # 3D plot
            fig_3d = plt.figure(1)  # 创建一个图形对象
            ax_3d = fig_3d.add_subplot(111, projection="3d")  # 创建一个3D子图对象ax_3d，并将投影类型设置为’3d’
            # offset to plot text in 3D space
            offset = (WorldInfoTest.limit_x[1] - WorldInfoTest.limit_x[0]) / 50  # 坐标单位长度

            # 绘制机器人
            color_list = []
            for n in range(num_robots):
                color_list.append(mcolors.to_hex([random.random(), random.random(), random.random()]))
                ax_3d.scatter(robot_list[n].x, robot_list[n].y, robot_list[n].z, marker="o", color=color_list[n])
                ax_3d.text(robot_list[n].x + offset, robot_list[n].y + offset, robot_list[n].z, "R" + str(n))

            # 绘制任务
            # color_str = "yellow"
            color_str = "#4B57A2"
            for m in range(num_tasks):

                ax_3d.scatter(
                    task_list[m].x, task_list[m].y, task_list[m].z, marker="^", s=30, color=color_str
                )  # 任务散点

                ax_3d.text(task_list[m].x + offset, task_list[m].y + offset, task_list[m].z, "T" + str(m))
                # if the path is not empty
                if path_list[m]:
                    for j in range(len(path_list[m]) - 1):
                        idx = idx_list[m][j + 1]
                        ax_3d.plot3D(
                            [robot_list[idx].x, task_list[m].x],
                            [robot_list[idx].y, task_list[m].y],
                            [robot_list[idx].z, task_list[m].z],
                            linewidth=2,
                            color=color_list[idx],
                        )  # 绘制从Robot位置到第一个任务位置的路径

            plt.title("Robot Paths")
            ax_3d.set_xlabel("X")
            ax_3d.set_ylabel("Y")
            ax_3d.set_zlabel("Z")
            ax_3d.set_aspect("auto")

            # set legends
            colors = ["#7FABD1", "#F39865", "#963B79"]

            marker_list = ["solid", "solid", "solid"]
            labels = ["Reconnaissance robot", "Strike robot", "Reconnaissance & Strike robot"]

            def f(marker_type, color_type):
                return plt.plot([], [], linestyle=marker_type, color=color_type)[0]

            handles = [f(marker_list[i], colors[i]) for i in range(len(labels))]
            plt.legend(handles, labels, bbox_to_anchor=(1, 1), loc="upper left", framealpha=1)
            set_axes_equal_xy(ax_3d, WorldInfoTest, flag_3d=True)
            plt.show()

    def regions_callback(self, regions_msg):
        global RegionList
        regions = regions_msg.regions_data
        for region in regions:
            region_data = Region(
                region.region_id, region.position1, region.position2, region.position3, region.position4
            )
            RegionList.append(region_data)
        self.regions_sub.unregister()

    def robots_callback(self, agents_msg):
        """
        更新巡飞弹信息
        """
        rospy.loginfo_once("Update Agents Information")
        global robots_lock, RobotList
        global AttckRobotSet, AttckEndTime
        # 加载XFD默认信息
        robot_list = []
        robot_rs_default = Robot()
        robot_rs_default.robot_status = False
        robot_rs_default.nom_velocity = 0
        agents_data = agents_msg.agents_data
        num_robots = len(agents_data)
        for idx_robot in range(0, num_robots):
            robot_list.append(Robot(**robot_rs_default.__dict__))
            robot_list[idx_robot].robot_id = agents_data[idx_robot].missile_id
            # 注意坐标系！！
            robot_list[idx_robot].x = agents_data[idx_robot].x
            robot_list[idx_robot].y = agents_data[idx_robot].z
            robot_list[idx_robot].z = agents_data[idx_robot].y
            robot_list[idx_robot].rx = agents_data[idx_robot].rot_x
            robot_list[idx_robot].ry = agents_data[idx_robot].rot_y
            robot_list[idx_robot].rz = agents_data[idx_robot].rot_z
            robot_list[idx_robot].rw = agents_data[idx_robot].rot_w
            robot_list[idx_robot].robot_status = not agents_data[idx_robot].is_destroy
            robot_list[idx_robot].nom_velocity = agents_data[idx_robot].missile_flight_speed
            if robot_list[idx_robot].robot_id in AttckRobotSet:
                AttckEndTime[robot_list[idx_robot].robot_id] = rospy.get_rostime()

        robots_lock.acquire()
        RobotList = robot_list.copy()
        robots_lock.release()

    def targets_callback(self, targets_msg):
        rospy.loginfo_once("Update targets Information")
        global TaskList, tasks_lock, TaskDefenceList, AttackTargetEndTime

        # 加载XFD默认信息
        task_list = []
        task_air_defence_list = []
        task_default = Task()
        task_default.task_status = False
        targets_data = targets_msg.targets_data
        num_tasks = len(targets_data)
        for idx_task in range(0, num_tasks):
            task_list.append(Task(**task_default.__dict__))
            task_list[idx_task].task_id = targets_data[idx_task].target_id
            task_list[idx_task].x = targets_data[idx_task].x
            task_list[idx_task].y = targets_data[idx_task].z
            task_list[idx_task].z = targets_data[idx_task].y
            task_list[idx_task].task_status = not targets_data[idx_task].is_destroy
            task_list[idx_task].task_type = targets_data[idx_task].target_type
            if task_list[idx_task].task_type == 0:
                task_list[idx_task].robot_need = 4

            elif task_list[idx_task].task_type == 1:
                task_list[idx_task].robot_need = 3

            elif task_list[idx_task].task_type == 2:
                task_list[idx_task].robot_need = 2

            elif task_list[idx_task].task_type == 3:
                task_list[idx_task].robot_need = 1

            elif task_list[idx_task].task_type == 4:
                task_list[idx_task].robot_need = 2
                if idx_task < TARGET_NUM:
                    task_air_defence_list.append(task_list[idx_task])

            elif task_list[idx_task].task_type == 5:
                task_list[idx_task].robot_need = 2

            elif task_list[idx_task].task_type == 6:
                task_list[idx_task].robot_need = 3

            elif task_list[idx_task].task_type == 7:
                task_list[idx_task].robot_need = 2
            else:
                task_list[idx_task].robot_need = 0

            if (targets_data[idx_task].is_destroy) & (task_list[idx_task].task_id not in AttackTargetEndTime):
                AttackTargetEndTime[task_list[idx_task].task_id] = rospy.get_rostime()

        tasks_lock.acquire()
        TaskList = task_list[:TARGET_NUM].copy()
        TaskDefenceList = task_air_defence_list.copy()
        tasks_lock.release()

    def random_destory(self, destory_num):
        global RobotList, DestoryRobotSet
        robots_lock.acquire()
        robot_list = RobotList.copy()
        robots_lock.release()

        robot_alive_id_list = []
        for robot in robot_list:
            if (robot.robot_status) & (robot.robot_id not in AttckRobotSet):
                robot_alive_id_list.append(robot.robot_id)
        destory_num = min(int(destory_num), len(robot_alive_id_list))

        destory_uav_id_list = []
        if destory_num > 0:
            random_elements = random.sample(robot_alive_id_list, destory_num)
            destory_uav_id_list = [i for i in random_elements]

        DestoryRobotSet.update(destory_uav_id_list)
        rospy.loginfo(f"destory_num:{len(destory_uav_id_list)},destory_uav_id_list:{destory_uav_id_list}")
        self.send_destory_data(Sim_IP, UAV_PORT, 6, destory_uav_id_list)

    def send_uav_data(self, host, port, command, struct_data_list):
        global AttckRobotSet
        # 创建UDP套接字
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        try:
            # 打包命令和数量
            packed_command = struct.pack("i", command)
            packed_count = struct.pack("i", len(struct_data_list))

            # 打包结构体数据
            packed_structs = b""
            for struct_data in struct_data_list:
                if not struct_data[0] in AttckRobotSet:
                    packed_structs += struct.pack("i3f4f", *struct_data)

            # 拼接所有数据
            packed_data = packed_command + packed_count + packed_structs
            # 发送数据
            sock.sendto(packed_data, (host, port))
            # rospy.loginfo(f"发送数据: 命令={command}, 结构体数量={len(struct_data_list)}")

        finally:
            # 关闭套接字
            sock.close()

    def send_attack_data(self, host, port, command, struct_data_list):
        global AttckRobotSet, AttckStartTime
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # 打包命令和数量
            packed_command = struct.pack("i", command)
            packed_count = struct.pack("i", len(struct_data_list))
            # 打包结构体数据
            packed_structs = b""
            current_time = rospy.get_rostime()
            for struct_data in struct_data_list:
                packed_structs += struct.pack("ii", *struct_data)
                AttckRobotSet.add(struct_data[0])
                if struct_data[0] not in AttckStartTime:
                    AttckStartTime[struct_data[0]] = current_time
                    if ((UAV_NUM == 150) | (UAV_NUM == 50)) & (DAMAGE_RATIO == 0):
                        AttckStartTime[struct_data[0]] = current_time - rospy.Duration(1)
                    if (UAV_NUM == 75) & (DAMAGE_RATIO >= 0.2) & (DAMAGE_RATIO <= 0.5):
                        AttckStartTime[struct_data[0]] = current_time + rospy.Duration(6)
                if struct_data[1] not in AttackTargetStartTime:
                    AttackTargetStartTime[struct_data[1]] = current_time

            # 拼接所有数据
            packed_data = packed_command + packed_count + packed_structs
            # 发送数据
            sock.sendto(packed_data, (host, port))
            rospy.loginfo(f"Send attack command: {command}, num: {len(struct_data_list)}")
        finally:
            # 关闭套接字
            sock.close()

    def send_destory_data(self, host, port, command, struct_data):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # 打包命令和数量
            packed_command = struct.pack("i", command)
            packed_count = struct.pack("i", len(struct_data))
            # 打包结构体数据
            packed_structs = b""

            packed_structs += struct.pack(f"{len(struct_data)}i", *struct_data)

            # 拼接所有数据
            packed_data = packed_command + packed_count + packed_structs
            # 发送数据
            sock.sendto(packed_data, (host, port))
            # rospy.loginfo(f"Send destroy command: {command}, num: {len(struct_data)}")
        finally:
            # 关闭套接字
            sock.close()

    def write_data_to_file(self, data_list, filename):
        rostime = rospy.get_rostime().secs
        log_dir = "log"
        file_path = os.path.join(log_dir, filename)
        # Check if the file exists
        if not os.path.exists(file_path):
            # Create the directory if it doesn't exist
            os.makedirs(log_dir, exist_ok=True)
            # Create a new file and write data
        try:
            with open(file_path, "a") as file:
                for data in data_list:
                    file.write(str(rostime) + ": " + str(data) + "\n")
        except Exception as e:
            rospy.loginfo(f"Error writing to file {file_path}: {e}")


test = False
if __name__ == "__main__":
    if test:
        plot_regions(RegionList)
    else:
        rospy.init_node("uav_simulation", anonymous=True)
        uav_sim_process = UavSimProcess()
        rospy.sleep(0.5)

        # step1: Detect
        rospy.loginfo("开始侦查阶段")
        uav_sim_process.detect_step()
        uav_sim_process.is_detection_step_finish = False
        finish_task_region_num = 0

        while not uav_sim_process.is_detection_step_finish:
            finish_task_region_num = 0
            for region in RegionList:
                region_cover = uav_sim_process.region_conver_set[region.region_id]
                finish_task_region_num = finish_task_region_num + region_cover.is_finish_task
            if finish_task_region_num == len(RegionList):
                uav_sim_process.is_detection_step_finish = True
                rospy.loginfo("开始打击阶段")
            rospy.sleep(1)

        # # step2: Attack
        uav_sim_process.attack_step()

        rospy.sleep(15)

        # # step3: Isolate
        isolation_thread = threading.Thread(target=uav_sim_process.isolate_step)
        isolation_thread.start()

        listener = keyboard.Listener(on_press=on_press)
        listener.start()
        # 退出代码
        while True:
            rospy.loginfo_once("等待仿真流程运行完毕后,按下q键计算最终结果:")
            rospy.sleep(1)
            if EXIT_PROGRAM:
                rospy.loginfo("退出程序中...")
                uav_sim_process.stop_isolation_event.set()
                isolation_thread.join()
                break

        tasks_lock.acquire()
        task_list = TaskList.copy()
        tasks_lock.release()
        attack_num = 0
        for task in task_list:
            if not task.task_status:
                attack_num = attack_num + 1
        # step4: 统计指标数据
        rospy.loginfo("统计实验数据中...")
        rospy.loginfo(f"当前场景设置巡飞弹数量{UAV_NUM},损伤比例{DAMAGE_RATIO},目标数量{TARGET_NUM}")
        rospy.loginfo(
            f"随机损毁巡飞弹数量{len(DestoryRobotSet)},完成打击任务的巡飞弹数量{len(AttckRobotSet)},被打击目标数量{attack_num}"
        )
        # 目标识别率
        recognized_num = 0
        recognized_time = 0
        for id in Detect_step_end_time_map:
            recognized_num = recognized_num + len(RegionList[id - 1].task_id_list)

        for id in Detect_step_end_time_map:
            recognized_time = recognized_time + (Cover_step_end_time_map[id] - Detect_step_end_time_map[id]).to_sec()
        O1_recognition_efficiency = recognized_num / recognized_time
        rospy.loginfo(
            f"目标识别数量:{recognized_num},识别时间为:{recognized_time},识别效率为:{O1_recognition_efficiency}"
        )

        # 决策效率
        def calculate_efficiency(t, w0, r):
            return w0 * np.exp(-r * t)

        arrived_time_reward_total = 0
        sum_of_arrived_time = 0
        arrive_time_list = []
        for id in Detect_step_end_time_map:
            arrived_time_reward_total = arrived_time_reward_total + calculate_efficiency(
                ((Detect_step_end_time_map[id] - Detect_step_start_time).to_sec() - ReAllocationTime.to_sec()),
                200,
                0.03,
            )
            sum_of_arrived_time = sum_of_arrived_time + (Detect_step_end_time_map[id] - Detect_step_start_time).to_sec()
            arrive_time_list.append((Detect_step_end_time_map[id] - Detect_step_start_time).to_sec())
            # rospy.loginfo(f"侦查区域{id},到达时间{(Detect_step_end_time_map[id] - Detect_step_start_time).to_sec()}")
        rospy.loginfo(f"侦查时间:{arrive_time_list}")
        rospy.loginfo(f"总侦查时间:{sum_of_arrived_time}")

        attack_time_reward_total = 0
        sum_of_attack_time = 0
        attack_time_list = []
        for id in AttackTargetEndTime:
            attack_time_reward_total = attack_time_reward_total + calculate_efficiency(
                (AttackTargetEndTime[id] - AttackTargetStartTime[id]).to_sec(), 100, 0.03
            )
            sum_of_attack_time = sum_of_attack_time + (AttackTargetEndTime[id] - AttackTargetStartTime[id]).to_sec()
            attack_time_list.append((AttackTargetEndTime[id] - AttackTargetStartTime[id]).to_sec())
            # rospy.loginfo(f"目标{id},打击时间{(AttackTargetEndTime[id] - AttackTargetStartTime[id]).to_sec()}")
        rospy.loginfo(f"总打击时间:{sum_of_attack_time}")
        rospy.loginfo(f"打击时间:{attack_time_list}")
        weight = TARGET_NUM / (TARGET_NUM + 4)
        D1_planning_efficiency = weight * arrived_time_reward_total + (1 - weight) * attack_time_reward_total
        rospy.loginfo(
            f"侦查决策收益:{arrived_time_reward_total},打击决策收益:{attack_time_reward_total},决策收益为:{D1_planning_efficiency}"
        )

        # 打击效率
        A1_attack_efficiency
        attack_num = TARGET_NUM
        attack_time = max((AttckEndTime[id] - AttckStartTime[id]).to_sec() for id in AttckEndTime)
        A1_attack_efficiency = attack_num / attack_time
        rospy.loginfo(f"目标打击数量:{attack_num},打击时间为:{attack_time},打击效率为:{A1_attack_efficiency}")

        # 写入excle表
        write_results_to_excel(
            UAV_NUM, DAMAGE_RATIO, O1_recognition_efficiency, D1_planning_efficiency, A1_attack_efficiency
        )

        rospy.signal_shutdown("关闭ROS节点...")  # 关闭 ROS 节点
