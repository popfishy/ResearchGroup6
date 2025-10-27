#!/usr/bin/env python3

from re import T
import numpy as np
import time
from typing import List, Tuple, Optional, Any, Dict
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import json
import os

from core.utils import *
from core.uav import UAV
from core.path_planner import PathPlanner
from core.scenario_parser import ScenarioParser
from communication.scripts.tcp_client_communication import TCPClient
from region2cover.region_cover import RegionCover

# ==================== 全局配置 ====================
# True:  运行本地Matplotlib可视化进行调试
# False: 运行无图形界面的仿真，并通过TCP/IP发送数据
VISUAL_DEBUG_MODE = True


# TODO: 任务分配
# import random
# from xfd_allocation.scripts.CBPA.lib.CBPA import CBPA
# from xfd_allocation.scripts.CBPA.lib.Robot import Robot
# from xfd_allocation.scripts.CBPA.lib.Task import Task
# from xfd_allocation.scripts.CBPA.lib.Region import Region
# from xfd_allocation.scripts.CBPA.lib.WorldInfo import WorldInfo
# from xfd_allocation.scripts.CBPA.lib.CBPA_REC import CBPA_REC


O1_recognition_efficiency: float = None
D1_planning_efficiency: float = None
A1_attack_efficiency: float = None


def on_press(key):
    global EXIT_PROGRAM
    try:
        if key.char == "q":  # 检测 'q' 键是否被按下
            print("退出程序中...")
            EXIT_PROGRAM = True
    except AttributeError:
        pass


class SwarmMissionManager:
    """50架无人机集群任务管理器"""
    
    def __init__(self):
        # 基本配置
        self.total_uavs = 50
        self.current_phase = MissionType.PREPARATION
        self.phase_start_time = time.time()
        self.mission_start_time = time.time()
        
        # 无人机管理
        self.uav_dict: Dict[int, UAV] = {}  # ID -> UAV对象
        self.group_assignments: Dict[int, List[int]] = {}  # group_id -> [uav_ids]
        
        # 起飞区域配置
        self.takeoff_zones = {}
        
        # 队形配置
        self.formation_configs = {}
        self._initialize_formations()
        
        # TODO：任务区域配置 - 需要您补充具体坐标
        self.reconnaissance_areas: List[TargetArea] = []  # 侦查区域列表
        self.attack_targets: List[EnemyTarget] = []        # 打击目标列表
        self.containment_zones: List[ContainmentZone] = []     # 封控区域列表
        
        # 任务状态跟踪
        self.phase_completion = {
            MissionType.PREPARATION: False,
            MissionType.RECONNAISSANCE: False,
            MissionType.ATTACK: False,
            MissionType.CONTAINMENT: False
        }
        
        # 区域覆盖规划器
        self.region_cover_planner: Optional[RegionCover] = None
        self.reconnaissance_sub_areas_corners: List[tuple] = [] # 存储子区域角点

        # 外部接口
        self.path_planner: Optional[PathPlanner] = None
        self.simulation_interface = None  # 仿真接口
        self.tcp_client: Optional[TCPClient] = None
        self.pos_converter = None
        
        # 初始化任务
        self._initialize_mission_areas()
    
    def _initialize_formations(self):
        """初始化队形配置"""
        square_5x5_positions = []
        spacing = 100.0
        for i in range(5):
            for j in range(5):
                x_offset = (i - 2) * spacing  # 中心对齐
                y_offset = (j - 2) * spacing
                square_5x5_positions.append((x_offset, y_offset))
        
        self.formation_configs["square_5x5"] = FormationConfig(
            name="square_5x5",
            formation_type="square",
            positions=square_5x5_positions,
            leader_index=12,  # 中心位置作为领机
            spacing=spacing
        )
        
        # 其他队形配置...
        # TODO: 添加其他需要的队形配置
    
    def _initialize_mission_areas(self):
        """初始化任务区域 - 根据给定范围生成覆盖路径和侦查区域"""
        print("--- 初始化任务区域 ---")
        # 1. 定义总侦察区域的角点 (East, North, Altitude)
        # region_cover.py 使用(x, y)进行2D计算，我们将East->x, North->y
        p1 = (4000, 0)
        p4 = (6500, 0)       # Top-Right
        p3 = (6500, -7000)   # Bottom-Right
        p2 = (4000, -7000)   # Bottom-Left
        
        # 2. 初始化RegionCover并将其划分为2x2=4个子区域
        num_groups = 4
        # pos1-4的顺序在region_cover中是 a-d-c-b (top-left, bottom-left, bottom-right, top-right)
        self.region_cover_planner = RegionCover(num_rows=2, num_cols=2, 
                                                pos1=p1, pos2=p2, pos3=p3, pos4=p4, 
                                                is_plot=False)
        
        # 3. 获取子区域边界并创建TargetArea对象
        self.reconnaissance_sub_areas_corners = self.region_cover_planner.divide_regions()

        if len(self.reconnaissance_sub_areas_corners) != num_groups:
            print(f"警告: 区域分割数量({len(self.reconnaissance_sub_areas_corners)})与预期无人机群数({num_groups})不符。")
            return

        self.reconnaissance_areas.clear() # 清空旧区域
        for i, corners in enumerate(self.reconnaissance_sub_areas_corners):
            # a(x,y), b(x,y), c(x,y), d(x,y)
            # a ---- b
            # |      |
            # d ---- c
            corner_a, corner_b, corner_c, corner_d = corners
            width = abs(corner_b[0] - corner_a[0])
            height = abs(corner_d[1] - corner_a[1])
            center_x = corner_a[0] + width / 2.0
            center_y = corner_a[1] - height / 2.0
            
            area = TargetArea(
                id=i + 1,
                center=(center_x, center_y, 100.0), # 假设侦查高度为100m
                width=width,
                height=height,
                priority=8,
                area_type=ZoneType.SEARCH_AREA,
                assigned_uavs=[]
            )
            self.reconnaissance_areas.append(area)
            print(f"创建侦查区域 {area.id}: 中心({center_x:.0f}, {center_y:.0f}), 宽 {width:.0f}, 高 {height:.0f}")

        # 4. 在区域定义后立即生成所有覆盖路径
        try:
            print("--- 正在为所有子区域生成覆盖路径 ---")
            self.region_cover_planner.cover_run(log_time='sim_run', cov_width=300)
            print(f"成功为 {len(self.region_cover_planner.all_path)} 个区域生成了覆盖路径。")
        except Exception as e:
            print(f"错误: 区域覆盖路径生成失败: {e}")

        # 5. 更新封控区域定义
        self.containment_zones = [
            ContainmentZone(
                id=1,
                center=(4000, 4000, 100),
                width=1600,
                height=1600,
                patrol_altitude=150,
                patrol_speed=25,
                assigned_uavs=[]
            )
        ]
    
    def set_external_interfaces(self, dubins_planner=None, coverage_planner=None, 
                               patrol_planner=None, simulation_interface=None):
        """
        设置外部接口，并用它们来配置核心的PathPlanner。
        """
        # 使用传入的底层规划器来实例化或配置总的PathPlanner
        if not self.path_planner:
            self.path_planner = PathPlanner() # 确保PathPlanner对象存在
        # 将具体的规划器实现注入到PathPlanner中
        if dubins_planner:
            self.path_planner.set_dubins_planner(dubins_planner)
        if coverage_planner:
            self.path_planner.set_coverage_planner(coverage_planner) 
        if patrol_planner:
            self.path_planner.set_patrol_planner(patrol_planner)
        self.simulation_interface = simulation_interface
        print("外部接口设置完毕，PathPlanner已配置。")

    # TODO:也可以提前设置好pathplanner，一步到位
    def set_path_planner(self, path_planner: PathPlanner):
        """设置路径规划器接口"""
        self.path_planner = path_planner

    def initialize_uavs_from_xml(self, xml_path: str):
        """【新增】从XML文件加载并初始化所有无人机和敌方目标"""
        print("\n --- 初始化无人机和敌方目标 ---")
        print(f"从 {xml_path} 加载场景")
        try:
            parser = ScenarioParser(xml_path)
            scenario_data = parser.parse()
            self.pos_converter = parser.converter
        except FileNotFoundError:
            print(f"错误: 场景文件 '{xml_path}' 未找到。无法初始化。")
            return False
        except Exception as e:
            print(f"解析XML文件时发生错误: {e}")
            return False

        uavs = scenario_data.get('uavs', [])
        if not uavs:
            print("错误: XML文件中未找到可用的我方无人机！")
            return False

        # 1. 填充无人机字典
        self.uav_dict = {uav.id: uav for uav in uavs}
        print(f"成功加载 {len(self.uav_dict)} 架无人机。")

        # 2. 填充敌方目标列表
        self.attack_targets = scenario_data.get('enemies', [])
        print(f"成功加载 {len(self.attack_targets)} 个敌方目标。")

        # TODO:打印敌方目标信息
        # for target in self.attack_targets:
        #     print(f"敌方目标: {target.id}, 类型: {target.target_type}, 位置: {target.position}")

        # 3. 动态配置编队信息
        self.group_assignments = scenario_data.get('uav_groups', {})
        self.total_uavs = len(self.uav_dict)
        print(f"识别出 {len(self.group_assignments)} 个无人机编队。")

        # 4. 动态分配侦察任务给编队
        # 简单策略：按编队ID从小到大，依次分配侦察区
        group_ids = sorted(self.group_assignments.keys())
        for i, area in enumerate(self.reconnaissance_areas):
            if i < len(group_ids):
                group_id = group_ids[i]
                area.assigned_uavs = self.group_assignments[group_id]
                print(f"编队 {group_id} (共 {len(area.assigned_uavs)} 架) 分配给侦察区域 {area.id}")
        return True

    
    # ! ==================== 阶段1: 准备阶段 ====================
    def execute_preparation_phase(self):
        """执行准备阶段：集结到侦查区域起点"""
        print("开始准备阶段：无人机集结...")
        self.current_phase = MissionType.PREPARATION
        self.phase_start_time = time.time()
        
        # 动态地为每个从XML加载的编队规划集结路径
        assigned_groups = set()
        for area in self.reconnaissance_areas:
            if not area.assigned_uavs: continue
            
            # 找到这组UAV属于哪个编队
            group_id_found = None
            for gid, uids in self.group_assignments.items():
                if area.assigned_uavs[0] in uids: # 检查第一个UAV的归属
                    group_id_found = gid
                    break
            
            if group_id_found and group_id_found not in assigned_groups:
                rally_point = self._calculate_rally_point(area)
                # 调用重写后的函数，注意参数变化
                self._plan_group_formation_movement(group_id_found, rally_point, "square_5x5")
                assigned_groups.add(group_id_found)
        
        # 等待所有无人机到达集结点（内部是仿真循环）
        self._wait_for_phase_completion("preparation")
        
        print("准备阶段完成，所有无人机已集结")
        self.phase_completion[MissionType.PREPARATION] = True

    def _calculate_rally_point(self, target_area: TargetArea) -> Tuple[float, float, float]:
        """使用区域的左上角作为集结点"""
        area_id = target_area.id
        if self.reconnaissance_sub_areas_corners and area_id <= len(self.reconnaissance_sub_areas_corners):
            # corners: (a, b, c, d), 'a' is top-left
            top_left_corner = self.reconnaissance_sub_areas_corners[area_id - 1][0]
            rally_point = (top_left_corner[0], top_left_corner[1], 100.0)
            print(f"\n --- 区域 {area_id} 的集结点设置为左上角: ({rally_point[0]:.0f}, {rally_point[1]:.0f}) ---")
            return rally_point
        else:
            # Fallback
            print(f"警告: 无法为区域 {area_id} 获取角点信息，使用备用集结点计算方法。")
            center = target_area.center
            width = target_area.width
            height = target_area.height
            return (center[0] - width / 2, center[1] + height / 2, center[2])
    
    def _plan_group_formation_movement(self, group_id: int, target_point: tuple, formation_name: str):
        """
        - 动态选择领航者。
        - 根据初始位置计算并设置跟随者的相对偏移。
        """
        if not self.path_planner:
            print("警告: PathPlanner 未设置"); return
        uav_ids = self.group_assignments.get(group_id)
        if not uav_ids:
            print(f"警告: 编队 {group_id} 中没有无人机。"); return
        # 1. 动态选择领航者 (例如，选择编队中ID最小的无人机)
        leader_id = min(uav_ids)
        leader_uav = self.uav_dict[leader_id]
        
        print(f"为编队 {group_id} 设置移动任务")
        print(f"UAV-{leader_id} 被选为领航者。")
        
        # 2. 为领航者设置角色并规划路径
        leader_uav.set_as_leader()
        self.path_planner.plan_leader_path(leader_uav, [target_point])
        # 3. 设置所有其他无人机为跟随者，并计算其相对领航者的初始偏移
        follower_count = 0
        leader_init_pos = leader_uav.init_global_position # 使用初始位置计算偏移
        for uav_id in uav_ids:
            if uav_id == leader_id:
                continue
            
            follower_uav = self.uav_dict[uav_id]
            follower_init_pos = follower_uav.init_global_position
            
            # 计算初始的相对偏移 (X, Y)，这将是他们在飞行中保持的队形
            relative_offset = (
                follower_init_pos[0] - leader_init_pos[0],
                follower_init_pos[1] - leader_init_pos[1]
            )
            
            # 设置跟随者角色，并告知其要跟随的领航者和自己的队形偏移
            follower_uav.set_as_follower(leader=leader_uav, offset=relative_offset)
            follower_count += 1
            
        print(f"{follower_count} 架无人机被设置为跟随者，将保持其初始相对位置。")
           
    def plan_preparation_phase(self):
        """为准备阶段规划所有编队的集结路径"""
        print("正在规划准备阶段路径...")
        assigned_groups = set()
        for area in self.reconnaissance_areas:
            if not area.assigned_uavs: continue
            
            group_id_found = None
            for gid, uids in self.group_assignments.items():
                if area.assigned_uavs[0] in uids:
                    group_id_found = gid
                    break
            
            if group_id_found and group_id_found not in assigned_groups:
                rally_point = self._calculate_rally_point(area)
                self._plan_group_formation_movement(group_id_found, rally_point, "square_5x5")
                assigned_groups.add(group_id_found)


# ==================== 阶段2: 侦查阶段 ====================
    def plan_reconnaissance_phase(self):
        """为侦察阶段规划并分配覆盖路径"""
        print("\n --- 进入侦查阶段 ---")
        for area in self.reconnaissance_areas:
            if not area.assigned_uavs:
                continue
            
            # 从region_cover_planner获取预先计算好的路径
            path_index = area.id - 1

            if self.region_cover_planner and path_index < len(self.region_cover_planner.all_path):
                coverage_path_raw = self.region_cover_planner.all_path[path_index]
                
                # 转换路径格式为PathPoint列表
                coverage_path = []
                
                # 【修改】使用 .size() 和索引来迭代，以匹配fields2cover对象的用法
                if not hasattr(coverage_path_raw, 'size'):
                    print(f"警告: 为区域 {area.id} 获取的路径对象无效，跳过。")
                    continue

                for i in range(coverage_path_raw.size()):
                    p_state = coverage_path_raw[i]
                    if hasattr(p_state, 'point') and hasattr(p_state, 'angle'):
                        point = PathPoint(p_state.point.getX(), p_state.point.getY(), area.center[2], p_state.angle)
                        coverage_path.append(point)
                    else:
                        print(f"警告: 路径点索引 {i} 的格式不正确，已跳过。")

                # 将这条路径分配给区域内的所有无人机（领机执行，随从跟随）
                self._assign_coverage_paths(area.assigned_uavs, coverage_path)
            else:
                print(f"警告: 找不到为区域 {area.id} 预计算的覆盖路径。")

    def execute_reconnaissance_phase(self):
        """执行侦查阶段：区域覆盖搜索"""
        print("开始侦查阶段：区域覆盖搜索...")
        self.current_phase = MissionType.RECONNAISSANCE
        self.phase_start_time = time.time()
        
        # 规划并分配路径
        self.plan_reconnaissance_phase()
        
        # 等待所有无人机完成其覆盖路径
        self._wait_for_phase_completion("reconnaissance")

        print("侦查阶段完成")
        self.phase_completion[MissionType.RECONNAISSANCE] = True
    
    def _assign_coverage_paths(self, uav_ids: List[int], path: List[PathPoint]):
        """
        将单条覆盖路径分配给一个无人机编队。
        - 领航者将获得完整的路径。
        - 跟随者将继续跟随领航者。
        """
        if not uav_ids: return
        
        # 假设编队的领航者是已经设置好的
        leader_uav = None
        for uid in uav_ids:
            uav = self.uav_dict.get(uid)
            if uav and uav.is_leader:
                leader_uav = uav
                break
        
        if leader_uav:
            # 将路径点列表转换为规划器可用的目标点列表
            target_points = [(p.x, p.y, p.z) for p in path]
            self.path_planner.plan_leader_path(leader_uav, target_points)
            print(f"为领航者 UAV-{leader_uav.id} 分配了包含 {len(path)} 个点的覆盖路径。")
        else:
            # 如果找不到领航者，则分配给第一个无人机（备用逻辑）
            leader_id = uav_ids[0]
            leader_uav = self.uav_dict[leader_id]
            leader_uav.set_as_leader() # 紧急设为领航者
            # 将路径点列表转换为规划器可用的目标点列表
            target_points = [(p.x, p.y, p.z) for p in path]
            self.path_planner.plan_leader_path(leader_uav, target_points)
            print(f"警告: 在编队中未找到预设领航者。已将 UAV-{leader_id} 设为领航者并分配覆盖路径。")



    # ==================== 阶段3: 打击阶段 ====================
    def execute_attack_phase(self):
        """执行打击阶段：攻击检测到的目标"""
        print("开始打击阶段：攻击检测目标...")
        self.current_phase = MissionType.ATTACK
        self.phase_start_time = time.time()
        
        if not self.attack_targets:
            print("未发现目标，跳过打击阶段")
            self.phase_completion[MissionType.ATTACK] = True
            return
        
        # 目标优先级排序
        sorted_targets = sorted(self.attack_targets, 
                               key=lambda t: (t.threat_level, t.estimated_value), 
                               reverse=True)
        
        # 分配攻击任务
        available_uavs = self._get_available_uavs_for_attack()
        attack_assignments = self._assign_attack_missions(sorted_targets, available_uavs)
        
        # 执行攻击
        self._execute_attack_missions(attack_assignments)
        
        print("打击阶段完成")
        self.phase_completion[MissionType.ATTACK] = True
    
    def _get_available_uavs_for_attack(self) -> List[int]:
        """获取可用于攻击的无人机"""
        available_uavs = []
        
        for uav_id, uav in self.uav_dict.items():
            # TODO: 检查无人机状态、燃料、武器等
            # if uav.status == UAVStatus.ACTIVE and uav.fuel > 30:
            #     available_uavs.append(uav_id)
            available_uavs.append(uav_id)  # 临时：假设所有无人机可用
        
        return available_uavs
    
    def _assign_attack_missions(self, targets: List[EnemyTarget], uav_ids: List[int]) -> Dict[int, int]:
        """分配攻击任务 - 返回 {uav_id: target_id} 映射"""
        assignments = {}
        
        # 简单分配策略：一对一分配
        for i, target in enumerate(targets):
            if i < len(uav_ids):
                uav_id = uav_ids[i]
                assignments[uav_id] = target.id
                target.assigned_uav_id = uav_id
                print(f"分配 UAV-{uav_id} 攻击目标 {target.id}")
        
        return assignments
    
    def _execute_attack_missions(self, assignments: Dict[int, int]):
        """执行攻击任务"""
        for uav_id, target_id in assignments.items():
            target = next((t for t in self.attack_targets if t.id == target_id), None)
            if target:
                # 规划攻击路径
                attack_waypoint = target.position
                
                # TODO: 为无人机设置攻击航路点
                # self.uav_dict[uav_id].set_waypoints([attack_waypoint])
                # self.uav_dict[uav_id].current_mission = MissionType.ATTACK
                
                print(f"UAV-{uav_id} 前往攻击目标 {target_id} at {attack_waypoint}")
                
                # 模拟攻击结果
                target.status = TargetStatus.ATTACKED
                # TODO: 根据实际攻击效果更新目标状态
    
    # ==================== 阶段4: 封控阶段 ====================
    def execute_containment_phase(self):
        """执行封控阶段：剩余无人机前往封控区域巡逻"""
        print("开始封控阶段：建立封控巡逻...")
        self.current_phase = MissionType.CONTAINMENT
        self.phase_start_time = time.time()
        
        # 获取剩余可用无人机
        remaining_uavs = self._get_remaining_uavs()
        
        if not remaining_uavs:
            print("无剩余无人机执行封控任务")
            self.phase_completion[MissionType.CONTAINMENT] = True
            return
        
        # 分配封控区域
        for zone in self.containment_zones:
            assigned_count = min(len(remaining_uavs), 10)  # 每个区域最多10架
            zone.assigned_uavs = remaining_uavs[:assigned_count]
            remaining_uavs = remaining_uavs[assigned_count:]
            
            # 规划巡逻路径
            self._plan_containment_patrol(zone)
            
            if not remaining_uavs:
                break
        
        print("封控阶段部署完成")
        self.phase_completion[MissionType.CONTAINMENT] = True
    
    def _get_remaining_uavs(self) -> List[int]:
        """获取剩余可用的无人机"""
        remaining = []
        
        for uav_id, uav in self.uav_dict.items():
            # TODO: 检查无人机状态
            # if uav.status == UAVStatus.ACTIVE and uav.fuel > 20:
            #     remaining.append(uav_id)
            remaining.append(uav_id)  # 临时：假设所有无人机可用
        
        return remaining
    
    def _plan_containment_patrol(self, zone: ContainmentZone):
        """规划封控区域巡逻路径"""
        if not self.patrol_planner:
            print("警告: 巡逻路径规划器未设置，使用简单圆形巡逻")
            self._simple_circle_patrol(zone)
            return
        
        try:
            # TODO: 调用专业巡逻路径规划库
            # patrol_paths = self.patrol_planner.plan_patrol_paths(
            #     zone.center, zone.radius, len(zone.assigned_uavs)
            # )
            
            self._simple_circle_patrol(zone)
            
        except Exception as e:
            print(f"巡逻路径规划失败: {e}")
            self._simple_circle_patrol(zone)
    
    def _simple_circle_patrol(self, zone: ContainmentZone):
        """简单圆形巡逻模式"""
        center = zone.center
        radius = zone.radius
        altitude = zone.patrol_altitude
        num_uavs = len(zone.assigned_uavs)
        
        for i, uav_id in enumerate(zone.assigned_uavs):
            # 计算起始角度
            start_angle = (2 * math.pi * i) / num_uavs
            
            # 生成圆形巡逻路径
            patrol_points = []
            for j in range(8):  # 8个点组成圆形
                angle = start_angle + (2 * math.pi * j) / 8
                x = center[0] + radius * math.cos(angle)
                y = center[1] + radius * math.sin(angle)
                patrol_points.append((x, y, altitude))
            
            # TODO: 为无人机设置巡逻路径
            # self.uav_dict[uav_id].set_waypoints(patrol_points)
            # self.uav_dict[uav_id].current_mission = MissionType.CONTAINMENT
            
            print(f"为 UAV-{uav_id} 设置圆形巡逻路径，半径 {radius}m")
    
    # ==================== 任务控制方法 ====================
    def start_mission(self):
        """启动完整任务流程"""
        print("=== 开始50架无人机集群任务 ===")
        
        # 初始化无人机
        self.initialize_uavs_from_simulation()
        
        try:
            # 执行各个阶段
            self.execute_preparation_phase()
            self.execute_reconnaissance_phase()
            self.execute_attack_phase()
            self.execute_containment_phase()
            
            print("=== 任务完成 ===")
            
        except Exception as e:
            print(f"任务执行失败: {e}")
            self._emergency_abort()

    def _wait_for_phase_completion(self, phase_name: str, max_steps=5000, dt=0.5):
        """
        等待阶段完成.
        - 在可视化模式下，此函数仅作为仿真核心。
        - 在TCP模式下，此函数会成为一个带实时速率控制的循环，并发布数据。
        """
        print(f"开始运行仿真，等待 {phase_name} 阶段完成...")
        
        start_time = time.time()
        for step in range(max_steps):
            loop_start_time = time.time()
            
            # 1. 为所有跟随者计算并更新其队形目标点
            for uav in self.uav_dict.values():
                if not uav.is_leader and uav.leader is not None:
                    # 获取领航者的当前状态
                    leader_pos = uav.leader.position
                    leader_heading = uav.leader.heading
                    
                    # 获取该跟随者的队形偏移量
                    offset_x = uav.formation_offset[0]
                    offset_y = uav.formation_offset[1]
                    
                    # 在领航者的坐标系下进行旋转，计算出世界坐标系下的偏移
                    rotated_offset_x = offset_x * np.cos(leader_heading) - offset_y * np.sin(leader_heading)
                    rotated_offset_y = offset_x * np.sin(leader_heading) + offset_y * np.cos(leader_heading)
                    
                    # 计算最终的目标位置
                    target_pos = (
                        leader_pos[0] + rotated_offset_x,
                        leader_pos[1] + rotated_offset_y,
                        leader_pos[2]  # 假设高度与领航者一致
                    )
                    
                    # 将计算出的目标点和航向设置给跟随者
                    uav.set_formation_target(target_pos, leader_heading)

            # 2. 更新所有无人机的位置
            for uav in self.uav_dict.values():
                if uav.status == UAVStatus.ACTIVE:
                    uav.update_position(dt)
            
            # 3. 如果是TCP模式，则发布数据
            if not VISUAL_DEBUG_MODE:
                self._publish_uav_data_to_tcp()
                    
            # 4. 检查所有领航者的路径是否完成
            leaders = [uav for uav in self.uav_dict.values() if uav.is_leader]
            if not leaders: 
                print("警告: 未找到领航者来判断阶段完成状态。")
                break
            # 检查path_planning_complete确保路径已生成
            active_leaders = [l for l in leaders if l.path_planning_complete]
            if not active_leaders: # 如果没有领航者完成规划，则继续等待
                 if step % 100 == 0:
                      print("等待领航者路径规划完成...")
                 continue

            all_leaders_done = all(leader.is_path_complete for leader in active_leaders)
            
            if all_leaders_done:
                duration = time.time() - start_time
                print(f"{phase_name} 阶段完成！耗时: {duration:.2f}s, 仿真步数: {step}")
                return
            
            # 5. 在TCP模式下，sleep以保持循环频率
            if not VISUAL_DEBUG_MODE:
                try:
                    elapsed = time.time() - loop_start_time
                    sleep_duration = (1.0 / 10.0) - elapsed
                    if sleep_duration > 0:
                        time.sleep(sleep_duration)
                except KeyboardInterrupt:
                    print("KeyboardInterrupt received. Exiting simulation loop.")
                    break
        
        duration = time.time() - start_time
        print(f"警告: {phase_name} 阶段在达到最大步数 {max_steps} 未完成。耗时: {duration:.2f}s")

    def report_destruction(self, destruction_events: Dict[int, int]):
        """
        报告并处理无人机或目标的损毁事件。
        - 构造并发送 "DestoryEntity" 格式的TCP消息。
        - 更新内部仿真状态，将被摧毁的UAV或目标标记为DESTROYED。

        :param destruction_events: 一个字典，键为攻击者ID(drone_id)，值为被摧毁者ID(target_id)。
        """
        if not destruction_events:
            print("警告: report_destruction 调用时未提供任何损毁事件。")
            return

        agents_list = []
        for attacker_id, target_id in destruction_events.items():
            # 1. 更新内部状态
            target_found = False
            # 检查被摧毁的是否为我方无人机
            if target_id in self.uav_dict:
                destroyed_uav = self.uav_dict[target_id]
                if destroyed_uav.status != UAVStatus.DESTROYED:
                    destroyed_uav.destroy()
                    print(f"UAV-{attacker_id} 摧毁了UAV-{target_id}。")
                target_found = True
            else:
                # 检查被摧毁的是否为敌方目标
                target_to_destroy = next((tgt for tgt in self.attack_targets if tgt.id == target_id), None)
                if target_to_destroy:
                    if target_to_destroy.status != TargetStatus.DESTROYED:
                        target_to_destroy.status = TargetStatus.DESTROYED
                        print(f"UAV-{attacker_id} 摧毁了敌方目标 {target_id}。")
                    target_found = True
            
            if not target_found:
                print(f"警告: 在损毁事件中未找到目标ID {target_id}。")

            # 2. 构造消息体
            agents_list.append({
                "drone_id": attacker_id,
                "target_id": target_id
            })

        # 3. 构造并发送完整的TCP消息
        destruction_payload = {
            "key": "DestoryEntity",
            "name": "ResearchGroup6",
            "timestamp": time.time(),
            "agents": agents_list
        }

        if self.tcp_client and self.tcp_client.connected:
            print("正在发送损毁信息...")
            self.tcp_client.send_json(destruction_payload)
            print("损毁信息发送成功。")
        else:
            print("警告: TCP客户端未连接，无法发送损毁信息。")

    def _emergency_abort(self):
        """紧急中止任务"""
        print("执行紧急中止程序...")
        
        # TODO: 让所有无人机返回基地
        for uav_id in self.uav_dict:
            # self.uav_dict[uav_id].return_to_base()
            pass
    
    def get_mission_status(self) -> Dict[str, Any]:
        """获取任务状态"""
        return {
            "current_phase": self.current_phase.value,
            "phase_duration": time.time() - self.phase_start_time,
            "total_duration": time.time() - self.mission_start_time,
            "phase_completion": {k.value: v for k, v in self.phase_completion.items()},
            "active_uavs": len([uav for uav in self.uav_dict.values()]),  # TODO: 检查实际状态
            "detected_targets": len(self.attack_targets),
            "containment_zones": len(self.containment_zones)
        }
    
    # ==================== TCP/IP通信方法 ====================
    def _initialize_tcp_client(self):
        """初始化TCP客户端"""
        SERVER_HOST = '10.66.1.93'
        SERVER_PORT = 13334
        CLIENT_IP = '10.66.1.192'
        self.tcp_client = TCPClient(host=SERVER_HOST, port=SERVER_PORT, client_ip=CLIENT_IP)
        if not self.tcp_client.connect():
            print(f"警告: TCP连接失败, 数据将不会被发布.")
            self.tcp_client = None

    def _publish_uav_data_to_tcp(self):
        """将所有无人机的状态数据通过TCP发布"""
        if not self.tcp_client or not self.pos_converter:
            return

        # 1. 构造JSON数据结构
        multi_path = []
        for uav in self.uav_dict.values():
            if uav.status != UAVStatus.ACTIVE: continue
            
            # 坐标转换: ENU -> LLA
            lon, lat, alt = self.pos_converter.ENUtoWGS84(uav.position[0], uav.position[1], uav.position[2])
            
            # 直接使用速度向量的分量
            vel_x = uav.velocity[0]
            vel_y = uav.velocity[1]
            
            dic = {
                "agentId": uav.id,
                "velocity_x": float(vel_x),
                "velocity_y": float(vel_y),
                "velocity_z": 0.0,  # 假设Z轴速度为0
                "latitude": float(lat),
                "longitude": float(lon),
                "altitude": float(alt),
                "roll": 0.0,      # 简易模型，横滚为0
                "pitch": 0.0,     # 简易模型，俯仰为0
                "yaw": float(uav.heading)
            }
            multi_path.append(dic)
            
        poses_data = {
            "key": "SwarmTrajectoryResults",
            "name": "ResearchGroup6",
            "timestamp": time.time(),
            "agents": multi_path
        }
        
        # 2. 发送数据
        self.tcp_client.send_json(poses_data)

    def disconnect(self):
        """Safely disconnects the TCP client if it's connected."""
        if self.tcp_client and self.tcp_client.connected:
            print("Disconnecting TCP client...")
            self.tcp_client.disconnect()

    def plot_results(self):
        """可视化所有无人机和目标的轨迹/位置"""
        import matplotlib.pyplot as plt
        plt.figure(figsize=(16, 12))
        
        # 绘制无人机轨迹
        colors = ['r', 'b', 'g', 'c', 'm', 'y']
        group_ids = sorted(self.group_assignments.keys())
        for i, group_id in enumerate(group_ids):
            color = colors[i % len(colors)]
            
            for uav_id in self.group_assignments[group_id]:
                uav = self.uav_dict[uav_id]
                history = np.array(uav.position_history)
                
                if uav.is_leader:
                    plt.plot(history[:, 0], history[:, 1], color=color, linewidth=2.5, label=f'Group {group_id} Leader')
                else:
                    plt.plot(history[:, 0], history[:, 1], color=color, linestyle='--', linewidth=1)

        # 绘制敌方目标位置
        if self.attack_targets:
            enemy_pos = np.array([tgt.position for tgt in self.attack_targets])
            plt.scatter(enemy_pos[:, 0], enemy_pos[:, 1], c='black', marker='x', s=100, label='Enemy Targets')

        # 绘制集结点
        for area in self.reconnaissance_areas:
            rp = self._calculate_rally_point(area)
            plt.plot(rp[0], rp[1], 'k*', markersize=15, mfc='yellow', label='Rally Point')

        plt.title('Swarm Formation Simulation from XML Scenario')
        plt.xlabel('East (m)')
        plt.ylabel('North (m)')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

    def animate_preparation_phase(self, max_steps=5000, dt=0.5, interval=20):
        """
        通过动态动画来可视化和执行准备阶段。
        这个函数会设置绘图，然后启动一个包含仿真循环的动画。
        """
        print("开始准备阶段的动态仿真与可视化...")
        # 1. ======== 动画设置 ========
        fig, ax = plt.subplots(figsize=(16, 12))
        ax.set_aspect('equal', adjustable='box')
        ax.set_title('Swarm Mission: Preparation Phase')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.grid(True)
        # 绘制静态目标点
        for area in self.reconnaissance_areas:
            rp = self._calculate_rally_point(area)
            ax.plot(rp[0], rp[1], 'kx', markersize=15, markeredgewidth=3, label='Rally Point')
        # 2. ======== 初始化绘图元素 ========
        # 为每个无人机创建线条(轨迹)和点(当前位置)对象
        # 使用字典来方便地通过 uav_id 访问
        uav_plots = {}
        colors = ['r', 'b', 'g', 'c', 'm', 'y']
        group_ids = sorted(self.group_assignments.keys())

        for i, group_id in enumerate(group_ids):
            color = colors[i % len(colors)]
            for uav_id in self.group_assignments[group_id]:
                uav = self.uav_dict[uav_id]
                # 创建一个空的轨迹线条和一个点
                line, = ax.plot([], [], color=color, linestyle='--', linewidth=0.8)
                point, = ax.plot(uav.position[0], uav.position[1], 'o', color=color, markersize=4)
                uav_plots[uav_id] = {'line': line, 'point': point, 'is_leader': uav.is_leader}

        # 为领航员的轨迹设置更粗的线条
        for uav_id, plot_L_info in uav_plots.items():
            if plot_L_info['is_leader']:
                plot_L_info['line'].set_linewidth(2.0)
                plot_L_info['line'].set_linestyle('-')

        # 添加一个显示时间的文本
        time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
        # 自动调整坐标轴范围
        ax.autoscale_view()
        ax.legend()
        # 3. ======== 定义动画更新函数 ========
        # 这是 FuncAnimation 在每一帧都会调用的核心函数
        def update(frame):
            # --- a. 仿真逻辑核心 ---
            # 检查是否所有领航者都已到达目的地
            leaders = [uav for uav in self.uav_dict.values() if uav.is_leader and uav.path_planning_complete]
            if leaders and all(l.is_path_complete for l in leaders):
                # 如果任务完成，可以选择停止动画，但这里我们让它继续运行完
                pass
            # 为所有跟随者计算并更新其队形目标点
            for uav in self.uav_dict.values():
                if not uav.is_leader and uav.leader is not None:
                    leader_pos = uav.leader.position
                    leader_heading = uav.leader.heading
                    
                    # 获取该跟随者的队形偏移量
                    offset_x = uav.formation_offset[0]
                    offset_y = uav.formation_offset[1]
                    
                    # 在领航者的坐标系下进行旋转，计算出世界坐标系下的偏移
                    rotated_offset_x = offset_x * np.cos(leader_heading) - offset_y * np.sin(leader_heading)
                    rotated_offset_y = offset_x * np.sin(leader_heading) + offset_y * np.cos(leader_heading)
                    
                    # 计算最终的目标位置
                    target_pos = (
                        leader_pos[0] + rotated_offset_x,
                        leader_pos[1] + rotated_offset_y,
                        leader_pos[2]  # 假设高度与领航者一致
                    )
                    
                    # 将计算出的目标点和航向设置给跟随者
                    uav.set_formation_target(target_pos, leader_heading)
            # 更新所有无人机的位置
            for uav in self.uav_dict.values():
                if uav.status == UAVStatus.ACTIVE:
                    uav.update_position(dt)
            
            # --- b. 更新绘图元素 ---
            artists = []
            for uav_id, uav in self.uav_dict.items():
                plots = uav_plots[uav_id]
                history = np.array(uav.position_history)
                
                # 更新轨迹
                plots['line'].set_data(history[:, 0], history[:, 1])
                
                # 更新当前位置点
                plots['point'].set_data(uav.position[0], uav.position[1])
                
                artists.append(plots['line'])
                artists.append(plots['point'])
            # 更新时间文本
            time_text.set_text(f'Time: {frame * dt:.1f}s')
            artists.append(time_text)
            # 动态调整视图
            if frame % 50 == 0:
                ax.relim()
                ax.autoscale_view()
            
            return artists
        # 4. ======== 创建并启动动画 ========
        # FuncAnimation 会自动调用 update 函数
        ani = animation.FuncAnimation(
            fig, update, frames=max_steps,
            interval=interval, blit=True, repeat=False
        )
        plt.show() # 显示动画窗口
        print("准备阶段动画播放完毕。")
        self.phase_completion[MissionType.PREPARATION] = True

    def animate_reconnaissance_phase(self, max_steps=5000, dt=0.5, interval=20):
        """通过动态动画来可视化和执行侦察阶段。"""
        print("开始侦察阶段的动态仿真与可视化...")
        # 1. ======== 动画设置 ========
        fig, ax = plt.subplots(figsize=(16, 12))
        ax.set_aspect('equal', adjustable='box')
        ax.set_title('Swarm Mission: Reconnaissance Phase')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.grid(True)
        # 绘制侦察区域边界
        from matplotlib.patches import Circle
        for area in self.reconnaissance_areas:
            circle = Circle(xy=(area.center[0], area.center[1]), radius=area.radius, 
                            color='blue', fill=False, linestyle='--', label='Recon Area')
            ax.add_patch(circle)

        # 2. ======== 初始化绘图元素 ========
        uav_plots = {}
        colors = ['r', 'b', 'g', 'c', 'm', 'y']
        group_ids = sorted(self.group_assignments.keys())

        for i, group_id in enumerate(group_ids):
            color = colors[i % len(colors)]
            for uav_id in self.group_assignments[group_id]:
                uav = self.uav_dict[uav_id]
                line, = ax.plot([], [], color=color, linestyle='--', linewidth=0.8)
                point, = ax.plot(uav.position[0], uav.position[1], 'o', color=color, markersize=4)
                uav_plots[uav_id] = {'line': line, 'point': point, 'is_leader': uav.is_leader}

        for uav_id, plot_info in uav_plots.items():
            if plot_info['is_leader']:
                plot_info['line'].set_linewidth(2.0)
                plot_info['line'].set_linestyle('-')

        time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
        ax.autoscale_view()
        
        # 3. ======== 定义动画更新函数 ========
        def update(frame):
            # 仿真逻辑核心
            leaders = [uav for uav in self.uav_dict.values() if uav.is_leader and uav.path_planning_complete]
            if leaders and all(l.is_path_complete for l in leaders):
                pass 
            
            for uav in self.uav_dict.values():
                if not uav.is_leader and uav.leader is not None:
                    leader_pos, leader_heading = uav.leader.position, uav.leader.heading
                    offset_x, offset_y = uav.formation_offset
                    
                    rotated_offset_x = offset_x * np.cos(leader_heading) - offset_y * np.sin(leader_heading)
                    rotated_offset_y = offset_x * np.sin(leader_heading) + offset_y * np.cos(leader_heading)
                    
                    target_pos = (leader_pos[0] + rotated_offset_x, leader_pos[1] + rotated_offset_y, leader_pos[2])
                    uav.set_formation_target(target_pos, leader_heading)
            
            for uav in self.uav_dict.values():
                if uav.status == UAVStatus.ACTIVE:
                    uav.update_position(dt)
            
            # 更新绘图元素
            artists = []
            for uav_id, uav in self.uav_dict.items():
                plots = uav_plots[uav_id]
                history = np.array(uav.position_history)
                plots['line'].set_data(history[:, 0], history[:, 1])
                plots['point'].set_data(uav.position[0], uav.position[1])
                artists.extend([plots['line'], plots['point']])
            
            time_text.set_text(f'Time: {frame * dt:.1f}s')
            artists.append(time_text)
            
            if frame % 50 == 0:
                ax.relim()
                ax.autoscale_view()
            
            return artists

        # 4. ======== 创建并启动动画 ========
        ani = animation.FuncAnimation(fig, update, frames=max_steps, interval=interval, blit=True, repeat=False)
        plt.show()
        print("侦察阶段动画播放完毕。")
        self.phase_completion[MissionType.RECONNAISSANCE] = True

# ==================== 使用示例 ====================
if __name__ == "__main__":
    # 为独立运行此文件而创建的 Mock Planner
    class SimpleDubinsPlanner:
        def plan(self, q0, q1, turning_radius, step_size):
            try:
                import dubins
                path = dubins.shortest_path(q0, q1, turning_radius)
                configurations, _ = path.sample_many(step_size)
                return configurations, _
            except ImportError:
                print("错误: 'dubins' 库未找到. 请运行 'pip install dubins'.")
                return [], None

    # --- 1. 初始化 ---
    mission_manager = SwarmMissionManager()
    
    try:
        # --- 2. 设置外部接口 (使用 Mock Planner) ---
        path_planner = PathPlanner(dubins_planner=SimpleDubinsPlanner())
        mission_manager.set_path_planner(path_planner)
        
        print("=== 开始无人机集群任务仿真（从XML加载） ===")
        
        # --- 3. 从XML文件初始化场景 ---
        xml_file_path = '山地丛林.xml'
        if not mission_manager.initialize_uavs_from_xml(xml_file_path):
            print("场景初始化失败，程序退出。")
            exit()
            
        # 激活所有已加载的无人机
        for uav in mission_manager.uav_dict.values():
            uav.activate()

        # --- 4. 根据模式执行不同流程 ---
        if VISUAL_DEBUG_MODE:
            # 【调试模式】: 规划路径并启动可视化动画
            
            # 阶段1: 准备阶段
            mission_manager.plan_preparation_phase()
            # mission_manager.animate_preparation_phase()
            
            # 阶段2: 侦察阶段
            mission_manager.plan_reconnaissance_phase()
            mission_manager.animate_reconnaissance_phase()

        else:
            # 【联调模式】: 初始化TCP并执行后台仿真
            print("--- 运行模式: TCP集成 ---")
            mission_manager._initialize_tcp_client()
            
            # 依次执行准备和侦察阶段
            mission_manager.execute_preparation_phase()
            mission_manager.execute_reconnaissance_phase()

        print("仿真流程结束。")

        # --- 5. 监控 ---
        status = mission_manager.get_mission_status()
        print(f"\n最终任务状态: {status}")

    except KeyboardInterrupt:
        print("\n程序被用户中断 (Ctrl+C)")
    finally:
        # 确保无论如何都断开TCP连接
        if mission_manager:
            mission_manager.disconnect()
        print("程序已安全退出。")



  