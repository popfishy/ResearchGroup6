#!/usr/bin/env python3

import numpy as np
import time
from typing import List, Tuple, Optional, Any, Dict
import math
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os
import rospy
import json

from core.utils import *
from core.uav import UAV
from core.path_planner import PathPlanner
from core.scenario_parser import ScenarioParser
from communication.scripts.tcp_client_communication import TCPClient
from region2cover.region_cover import RegionCover
from region2cover.region_isolation import generate_circles_in_rectangle

from xfd_allocation.scripts.CBPA.lib.CBPA import CBPA
from xfd_allocation.scripts.CBPA.lib.Robot import Robot
from xfd_allocation.scripts.CBPA.lib.Task import Task
from xfd_allocation.scripts.CBPA.lib.Region import Region
from xfd_allocation.scripts.CBPA.lib.WorldInfo import WorldInfo
from xfd_allocation.scripts.CBPA.lib.CBPA_REC import CBPA_REC

from evaluate import write_results_to_excel

# ==================== 全局配置 ====================
# True:  运行本地Matplotlib可视化进行调试
# False: 运行无图形界面的仿真，并通过TCP/IP发送数据
VISUAL_DEBUG_MODE = False

# 要使用的UAV数量（None表示使用XML文件中的所有UAV）
# 例如：20、50、100，只激活前N架UAV
NUM_UAVS_TO_USE = 50  # 可修改为 20, 50, 100 等进行测试

# 全局开关：是否启用准备/侦查阶段的随机损毁机制
RANDOM_DESTRUCTION_ENABLED = True
# 损毁比例
DAMAGE_RATIO = 0.0

SIMULATION_DT = 4  # 仿真时间步长（秒），对应4Hz更新频率
TCP_PUBLISH_RATE = 4.0  # TCP数据发布频率（Hz）

random.seed(0)

class SwarmMissionManager:
    """50架无人机集群任务管理器"""
    
    def __init__(self):
        # 性能指标数据收集
        self.detect_start_time: Optional[float] = None      # 识别开始时间
        self.detect_end_time_map: Dict[int, float] = {}     # 目标ID -> 识别完成时间
        self.attack_start_time_map: Dict[int, float] = {}   # 目标ID -> 攻击开始时间
        self.attack_end_time_map: Dict[int, float] = {}     # 目标ID -> 攻击完成时间
        self.total_target_num: int = 0                      # 总目标数
        self.attack_assigned_uav_ids: set = set()           # 曾被分配攻击任务的UAV

        # 性能指标存储
        self.O1_recognition_efficiency: float = 0.0
        self.D1_planning_efficiency: float = 0.0
        self.A1_attack_efficiency: float = 0.0

        # 基本配置
        self.total_uavs:int = NUM_UAVS_TO_USE
        self.current_phase = MissionType.PREPARATION
        self.phase_start_time = time.time()
        self.mission_start_time = time.time()
        
        # 无人机管理
        self.uav_dict: Dict[int, UAV] = {}  # ID -> UAV对象
        self.group_assignments: Dict[int, List[int]] = {}  # group_id -> [uav_ids]
        

        # TODO: 后续统一使用RegionList表示
        self.RegionList: list = [
            Region(1, [4000, 0, 100], [650, 0, 100], [6500, -7000, 100], [4000, -7000, 100]),
        ]
        
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
        
        # 跟踪已报告的损毁事件，避免重复报告
        self.reported_destructions = set()  # {(attacker_id, target_id)}
        
        # 存储初始化时待发送的销毁事件（用于TCP连接后发送）
        self.pending_initial_destructions: Optional[Dict[int, int]] = None
        
        # 区域覆盖规划器
        self.region_cover_planner: Optional[RegionCover] = None
        self.reconnaissance_sub_areas_corners: List[tuple] = [] # 存储子区域角点

        # 外部接口
        self.path_planner: Optional[PathPlanner] = None
        self.tcp_client: Optional[TCPClient] = None
        self.pos_converter = None
        
        # 任务分配求解器
        self.cbpa_solver = CBPA_REC()
        
        # 初始化任务
        self._initialize_mission_areas()
        
    # TODO: 2.无人机飞行高度设置
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
            self.region_cover_planner.cover_run(uav_velocity = 30.0, turning_radius = 200.0, log_time='sim_run', cov_width=400)
            print(f"成功为 {len(self.region_cover_planner.all_path)} 个区域生成了覆盖路径。")
        except Exception as e:
            print(f"错误: 区域覆盖路径生成失败: {e}")


        self.containment_zones = [
            ContainmentZone(
                id=1,
                center=(8000, 0, 100),
                width=2500,
                height=2000,
                patrol_altitude=100,
                patrol_speed=25,
                assigned_uavs=[]
            )
        ]

    # TODO:也可以提前设置好pathplanner，一步到位
    def set_path_planner(self, path_planner: PathPlanner):
        """设置路径规划器接口"""
        self.path_planner = path_planner

    def initialize_uavs_from_xml(self, xml_path: str, num_uavs_to_use: Optional[int] = None):
        """
        从XML文件加载并初始化所有无人机和敌方目标
        
        参数:
        xml_path: XML场景文件路径
        num_uavs_to_use: 要使用的UAV数量（None表示使用全部）
                         例如：20、50、100，只激活前N架UAV
        """
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

        all_uavs = scenario_data.get('uavs', [])
        if not all_uavs:
            print("错误: XML文件中未找到可用的我方无人机！")
            return False

        # 1. 确定要使用的UAV数量
        if num_uavs_to_use is None:
            num_uavs_to_use = len(all_uavs)
        else:
            num_uavs_to_use = min(num_uavs_to_use, len(all_uavs))
        
        # 只使用前N架UAV
        uavs_to_use = all_uavs[:num_uavs_to_use]
        self.uav_dict = {uav.id: uav for uav in uavs_to_use}
        self.total_uavs = len(self.uav_dict)
        print(f"XML文件中总共有 {len(all_uavs)} 架无人机，本次使用前 {num_uavs_to_use} 架。")
        print(f"成功加载 {len(self.uav_dict)} 架无人机。")

        # 2. 处理敌方目标：随机抽取 int(num_uavs_to_use / 6) 个目标
        all_enemies = scenario_data.get('enemies', [])
        num_targets_to_use = int(num_uavs_to_use / 6)
        num_targets_to_use = min(num_targets_to_use, len(all_enemies))
        
        if num_targets_to_use > 0 and len(all_enemies) > 0:
            # 随机抽取目标
            selected_enemies = random.sample(all_enemies, num_targets_to_use)
            self.attack_targets = selected_enemies
            print(f"XML文件中总共有 {len(all_enemies)} 个敌方目标，随机抽取 {num_targets_to_use} 个。")
            
            # 对于未被选中的目标，标记为已摧毁并在初始化时发送销毁消息
            unselected_enemies = [e for e in all_enemies if e not in selected_enemies]
            unselected_ids = [id.id for id in unselected_enemies]
            if unselected_enemies:
                print(f"初始化时销毁 {len(unselected_enemies)} 个未被选中的敌方目标: {unselected_ids}")
                existing_uav_ids = list(self.uav_dict.keys())
                destruction_events = {}
                for idx, enemy in enumerate(unselected_enemies):
                    enemy.status = TargetStatus.DESTROYED
                    destruction_events[existing_uav_ids[idx]] = enemy.id
                # 发送销毁消息（如果TCP客户端已初始化）
                if hasattr(self, 'tcp_client') and self.tcp_client and self.tcp_client.connected:
                    self.send_attack_data(destruction_events, init_flag=True)
                else:
                    # 如果TCP未连接，保存待发送的销毁事件，等待TCP连接后发送
                    self.pending_initial_destructions = destruction_events
                    print(f"注意: TCP客户端未连接，已标记 {len(unselected_enemies)} 个目标为已摧毁状态。")
                    print(f"待TCP连接后发送 {len(destruction_events)} 个初始化销毁消息。")
        else:
            self.attack_targets = []
            print(f"XML文件中总共有 {len(all_enemies)} 个敌方目标，但计算出的目标数量为 {num_targets_to_use}，不使用任何敌方目标。")

        # 3. 重新分组：将N架UAV分成4组（忽略XML中的分组）
        num_groups = len(self.reconnaissance_areas)  # 应该是4
        if num_groups == 0:
            print("错误: 未找到侦察区域，无法进行分组。")
            return False
        
        # 将UAV ID列表按顺序分成4组
        uav_ids = sorted([uav.id for uav in uavs_to_use])
        group_size = num_uavs_to_use // num_groups
        remainder = num_uavs_to_use % num_groups
        
        self.group_assignments = {}
        start_idx = 0
        for group_id in range(1, num_groups + 1):
            # 前 remainder 组多分配一个UAV
            current_group_size = group_size + (1 if group_id <= remainder else 0)
            group_uav_ids = uav_ids[start_idx:start_idx + current_group_size]
            self.group_assignments[group_id] = group_uav_ids
            start_idx += current_group_size
            print(f"编队 {group_id}: {len(group_uav_ids)} 架UAV (ID: {group_uav_ids})")

        # 4. 将编队分配给侦察区域
        for i, area in enumerate(self.reconnaissance_areas):
            group_id = i + 1
            if group_id in self.group_assignments:
                area.assigned_uavs = self.group_assignments[group_id]
                print(f"编队 {group_id} (共 {len(area.assigned_uavs)} 架) 分配给侦察区域 {area.id}")
        
        return True

    def generate_formation_offsets(self, num_followers: int, formation_type: str, parameter: float) -> List[Tuple[float, float]]:
        """Generate follower offsets around leader at origin (0,0), excluding (0,0).
        num_followers refers to the number of followers (leader not included).
        """
        if num_followers <= 0:
            return []

        offsets: List[Tuple[float, float]] = []

        if formation_type == "circle":
            radius = float(parameter)
            angle_step = 2.0 * np.pi / max(1, num_followers)
            for i in range(num_followers):
                angle = i * angle_step
                offsets.append((radius * np.cos(angle), radius * np.sin(angle)))

        elif formation_type == "square":
            spacing = float(parameter)
            # Total UAVs including leader at center
            total_uavs = num_followers + 1
            m = int(math.ceil(math.sqrt(total_uavs)))  # grid size m x m
            # Compute centered grid indices: (0..m-1) with center at cx
            cx = (m - 1) / 2.0
            offsets_seq: List[Tuple[float, float]] = []
            # Row-major order, skip the exact center (leader position)
            for iy in range(m):
                for ix in range(m):
                    dx = (ix - cx) * spacing
                    dy = (iy - cx) * spacing
                    # Treat floating center as exactly (0,0)
                    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
                        continue
                    offsets_seq.append((dx, dy))
            offsets = offsets_seq[:num_followers]

        else:
            # Default to circle if unknown formation
            radius = float(parameter)
            angle_step = 2.0 * np.pi / max(1, num_followers)
            for i in range(num_followers):
                angle = i * angle_step
                offsets.append((radius * np.cos(angle), radius * np.sin(angle)))

        return offsets


    # ! ==================== 阶段1: 准备阶段 ====================
    def execute_preparation_phase(self):
        """为准备阶段规划所有编队的集结路径"""
        print("\n=== 准备阶段开始 ===")
        print(f"总UAV数量: {len(self.uav_dict)}")
        print(f"编队数量: {len(self.group_assignments)}")
        print(f"侦查区域数量: {len(self.reconnaissance_areas)}")
        
        assigned_groups = set()
        for area in self.reconnaissance_areas:
            if not area.assigned_uavs: continue
            
            group_id_found = None
            for gid, uids in self.group_assignments.items():
                if area.assigned_uavs[0] in uids:
                    group_id_found = gid
                    break
            
            # 直接从region_cover_planner获取路径起点
            if group_id_found and group_id_found not in assigned_groups:
                area_id = area.id
                if self.region_cover_planner and area_id <= len(self.region_cover_planner.start_point_list):
                    start_point_state = self.region_cover_planner.start_point_list[area_id - 1]
                    rally_point = (start_point_state.point.getX(), start_point_state.point.getY(), 100.0)
                    print(f"\n--- 区域 {area_id} 的集结点设置为覆盖路径起点: ({rally_point[0]:.0f}, {rally_point[1]:.0f}) ---")

                    self._plan_group_formation_movement(group_id_found, rally_point, 'square', 200.0)
                    assigned_groups.add(group_id_found)
                else:
                    print(f"警告: 无法为区域 {area_id} 获取覆盖路径起点，跳过编队 {group_id_found} 的准备阶段规划。")
        
        # 【调试】检查编队设置结果
        print(f"\n准备阶段完成。已设置编队数: {len(assigned_groups)}")
        leader_count = sum(1 for uav in self.uav_dict.values() if uav.is_leader)
        follower_count = sum(1 for uav in self.uav_dict.values() if not uav.is_leader and uav.leader is not None)
        no_role_count = sum(1 for uav in self.uav_dict.values() if not uav.is_leader and uav.leader is None)
        print(f"UAV角色统计: {leader_count} 个leader, {follower_count} 个follower, {no_role_count} 个无角色")
        
        if no_role_count > 0:
            print(f"警告: 有 {no_role_count} 架UAV没有被分配编队角色！")
            # 找出这些UAV
            no_role_uavs = [uav.id for uav in self.uav_dict.values() if not uav.is_leader and uav.leader is None]
            print(f"无角色UAV ID: {no_role_uavs[:10]}{'...' if len(no_role_uavs) > 10 else ''}")

    def _plan_group_formation_movement(self, group_id: int, target_point: tuple, formation_name: str, parameter: float):
        """
        动态选择领航者并根据队形设置相应的偏移：
        - 领航者位于队形中心，其偏移为(0,0)
        - 其他无人机根据所选编队在领航者周围分布，且不重叠
        """
        if not self.path_planner:
            print("警告: PathPlanner 未设置")
            return

        uav_ids = self.group_assignments.get(group_id)
        if not uav_ids:
            print(f"警告: 编队 {group_id} 中没有无人机。")
            return

        group_uavs = [self.uav_dict[uid] for uid in uav_ids if uid in self.uav_dict]
        if not group_uavs:
            print(f"警告: 编队 {group_id} 中没有有效的无人机对象。")
            return

        positions = np.array([uav.init_global_position for uav in group_uavs])
        centroid = np.mean(positions, axis=0)
        distances = np.linalg.norm(positions - centroid, axis=1)
        leader_index = int(np.argmin(distances))
        leader_uav = group_uavs[leader_index]
        leader_id = leader_uav.id

        print(f"为编队 {group_id} 设置移动任务")
        print(f"UAV-{leader_id} (最靠近中心) 被选为领航者。")

        leader_uav.set_as_leader()  # leader at (0,0) offset implicitly

        # Deterministic assignment: followers in listed order excluding leader
        followers = [self.uav_dict[uid] for uid in uav_ids if uid != leader_id]
        num_followers = len(followers)
        if num_followers == 0:
            print("编队仅有领航者，无需设置跟随者。")
            return

        desired_offsets = self.generate_formation_offsets(num_followers, formation_name, parameter)
        if len(desired_offsets) != num_followers:
            print("警告: 生成的编队偏移数量与跟随者数量不匹配，调整为最小长度。")
        desired_offsets = desired_offsets[:num_followers]

        # Deterministic row-major assignment to match grid sequence
        for follower, assigned_offset in zip(followers, desired_offsets):
            follower.set_as_follower(leader=leader_uav, offset=assigned_offset)

        print("所有无人机的偏移已设置，领航者位于队形中心。")
        
    # ! ==================== 阶段2: 侦查阶段 ====================
    def execute_reconnaissance_phase(self):
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
                
                if not hasattr(coverage_path_raw, 'size'):
                    print(f"警告: 为区域 {area.id} 获取的路径对象无效，跳过。")
                    continue

                for i in range(coverage_path_raw.size()):
                    p_state = coverage_path_raw[i]
                    point = PathPoint(p_state.point.getX(), p_state.point.getY(), area.center[2], p_state.angle)
                    coverage_path.append(point)

                # 将这条路径分配给区域内的所有无人机（领机执行，随从跟随）
                self._assign_coverage_paths(area.assigned_uavs, coverage_path)
            else:
                print(f"警告: 找不到为区域 {area.id} 预计算的覆盖路径。")

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
            # 合并准备阶段路径和侦察阶段路径
            # 1. 规划准备阶段的Dubins路径
            rally_point = (path[0].x, path[0].y, 100.0) # 侦察路径的起点即为集结点
            prep_path = self.path_planner.plan_leader_path(
                start_pos=(leader_uav.init_global_position[0], leader_uav.init_global_position[1], leader_uav.heading),
                waypoints=[rally_point]
            )

            # 2. 转换RegionCover路径为numpy数组格式
            recon_path = [np.array([p.x, p.y, p.heading]) for p in path]
            
            # 3. 合并两条路径
            full_path = prep_path + recon_path
            
            # 4. 将完整的路径直接设置给领航者
            leader_uav.set_planned_path(full_path)
            
        else:
            print(f"警告: 在为区域分配覆盖路径时未找到领航者 (uav_ids: {uav_ids})。")

    def _detect_high_threat_targets_in_rect(self, center_xy: Tuple[float, float], half_width: float = 150.0, half_height: float = 150.0) -> List[EnemyTarget]:
        """在以center_xy为中心的矩形范围内检测高威胁敌方目标（未被摧毁，threat_level>=3）。"""
        cx, cy = center_xy
        x_min, x_max = cx - half_width, cx + half_width
        y_min, y_max = cy - half_height, cy + half_height
        results = []
        for tgt in self.attack_targets:
            if tgt.status == TargetStatus.DESTROYED:
                continue
            if getattr(tgt, 'threat_level', 1) < 3:
                continue
            tx, ty = float(tgt.position[0]), float(tgt.position[1])
            if x_min <= tx <= x_max and y_min <= ty <= y_max:
                if tgt.id not in self.detect_end_time_map:
                    if self.detect_start_time is None:
                        self.detect_start_time = time.time()
                    self.detect_end_time_map[tgt.id] = time.time()
                    print(f"【目标识别】目标 {tgt.id} 在 {self.detect_end_time_map[tgt.id]:.2f}s 被识别")
                results.append(tgt)
        return results

    def _assign_attackers_for_targets(self, targets: List[EnemyTarget], candidate_uavs: List[UAV]):
        """为给定目标分配最近的候选UAV执行攻击，所需数量由target.robot_need决定。"""
        # 过滤候选UAV：必须ACTIVE、非leader、且未在ATTACK任务
        available = [u for u in candidate_uavs if (u.status == UAVStatus.ACTIVE and not u.is_leader and u.current_mission != MissionType.ATTACK)]
        if not available or not targets:
            return
        used: set = set()
        for tgt in targets:
            # 若目标已被摧毁则跳过
            if tgt.status == TargetStatus.DESTROYED:
                continue
            need = int(getattr(tgt, 'robot_need', 1))

            if tgt.id not in self.attack_start_time_map:
                self.attack_start_time_map[tgt.id] = time.time()
                print(f"【攻击开始】目标 {tgt.id} 在 {self.attack_start_time_map[tgt.id]:.2f}s 被攻击")

            if need <= 0:
                continue
            # 选择距离目标最近的need架可用UAV
            distances = []
            txy = np.array([tgt.position[0], tgt.position[1]], dtype=float)
            for u in available:
                if u.id in used:
                    continue
                d = np.linalg.norm(np.array([u.position[0], u.position[1]]) - txy)
                distances.append((d, u))
            if not distances:
                continue
            distances.sort(key=lambda x: x[0])
            selected = [u for _, u in distances[:need]]
            # 分配攻击并锁定目标，避免重复分配
            if selected:
                for u in selected:
                    u.set_attack_target(target_position=np.array(tgt.position, dtype=float), use_dubins=False)
                    u.current_mission = MissionType.ATTACK
                    used.add(u.id)
                # 立即将目标标记为摧毁，防止后续重复指派
                tgt.status = TargetStatus.DESTROYED

        self.total_target_num = len(self.attack_targets)

    # ! ==================== 阶段3: 打击阶段 ====================
    def execute_attack_phase(self):
        """执行打击阶段：解散编队，重新分配任务"""
        print("\n--- 开始打击阶段：解散编队并分配攻击任务 ---")
        self.current_phase = MissionType.ATTACK
        self.phase_start_time = time.time()
        
        # 1. 检查是否所有编队都完成了侦察任务
        all_leaders_completed = True
        for area in self.reconnaissance_areas:
            if not area.assigned_uavs:
                continue
            # 找到该区域的领航者
            leader_uav = None
            for uid in area.assigned_uavs:
                uav = self.uav_dict.get(uid)
                if uav and uav.is_leader:
                    leader_uav = uav
                    break
            
            if leader_uav and not leader_uav.is_path_complete:
                all_leaders_completed = False
                break
        
        if not all_leaders_completed:
            print("警告: 不是所有编队都完成了侦察任务，但强制进入打击阶段。")
        
        # 2. 解散所有编队，重置UAV状态
        print("正在解散所有编队...")
        for uav in self.uav_dict.values():
            uav.unset_formation_roles()  # 清除编队角色
            uav.current_mission = MissionType.UNSET  # 重置任务状态
            # 清空路径，准备接受新任务
            uav.waypoints = []
            uav.planned_path = []
            uav.path_index = 0
            uav.is_path_complete = False
            uav.path_planning_complete = False
        
        if not self.attack_targets:
            print("未发现敌方目标，所有UAV转入封控状态。")
            for uav in self.uav_dict.values():
                uav.current_mission = MissionType.CONTAINMENT
            self.phase_completion[MissionType.ATTACK] = True
            return
        # 3. 准备任务分配所需的数据
        available_uav_ids = self._get_available_uavs_for_attack()
        available_uavs_as_robots = [self._convert_uav_to_robot(self.uav_dict[uid]) for uid in available_uav_ids]
        
        active_targets_as_tasks = [self._convert_enemy_to_task(tgt) for tgt in self.attack_targets if tgt.status != TargetStatus.DESTROYED]
        
        if not available_uavs_as_robots or not active_targets_as_tasks:
            print("没有可用的无人机或活动目标，所有UAV转入封控状态。")
            for uav in self.uav_dict.values():
                uav.current_mission = MissionType.CONTAINMENT
            self.phase_completion[MissionType.ATTACK] = True
            return
            
        # 定义仿真世界边界
        world_info = WorldInfo(limit_x=[-2000, 8000], limit_y=[-8000, 2000], limit_z=[0, 500])
        
        # 4. 调用CBPA求解器进行任务分配
        print(f"为 {len(available_uavs_as_robots)} 架无人机和 {len(active_targets_as_tasks)} 个目标进行任务分配...")
        try:
            assignment_results, _, _, _, _ = self.cbpa_solver.solve_centralized(
                available_uavs_as_robots, active_targets_as_tasks, world_info
            )
            
        except Exception as e:
            print(f"任务分配时发生错误: {e}")
            # 分配失败，所有UAV转入封控状态
            for uav in self.uav_dict.values():
                uav.current_mission = MissionType.CONTAINMENT
            self.phase_completion[MissionType.ATTACK] = True
            return
            
        # 5. 解析分配结果并分配任务
        attack_assignments = {}  # uav_id -> target_id
        assigned_uav_ids = set()
        
        print(f"任务分配结果: {assignment_results}")
        for assignment in assignment_results:
            if len(assignment) >= 2:
                target_id = assignment[0]
                for uav_id in assignment[1:]:
                    attack_assignments[uav_id] = target_id
                    assigned_uav_ids.add(uav_id)
        
        # 6. 设置UAV任务状态和目标
        attack_count = 0
        containment_count = 0
        
        for uav_id, uav in self.uav_dict.items():
            if uav.status != UAVStatus.ACTIVE:
                continue
                
            if uav_id in attack_assignments:
                # 分配攻击任务
                target_id = attack_assignments[uav_id]
                target = next((t for t in self.attack_targets if t.id == target_id), None)
                
                if target:
                    uav.set_attack_target(
                        target_position=np.array(target.position, dtype=float),
                        use_dubins=False
                    )
                    attack_count += 1
                else:
                    print(f"警告: 未找到目标 {target_id}，UAV-{uav_id} 转入封控状态")
                    uav.current_mission = MissionType.CONTAINMENT
                    containment_count += 1
            else:
                # 未分配攻击任务，转入封控状态
                uav.current_mission = MissionType.CONTAINMENT
                containment_count += 1
        
        print(f"任务分配完成: {attack_count} 架UAV执行攻击任务, {containment_count} 架UAV执行封控任务")
        self.phase_completion[MissionType.ATTACK] = True

        # 记录所有被分配攻击任务的UAV
        for uav_id in attack_assignments.keys():
            self.attack_assigned_uav_ids.add(uav_id)

    def _convert_uav_to_robot(self, uav: UAV) -> Robot:
        """将UAV对象转换为xfd_allocation库所需的Robot对象"""
        robot = Robot()
        robot.robot_id = uav.id
        robot.x = uav.position[0]
        robot.y = uav.position[1]
        robot.z = uav.position[2]
        robot.robot_status = (uav.status == UAVStatus.ACTIVE)
        robot.nom_velocity = uav.current_speed
        return robot

    def _convert_enemy_to_task(self, enemy: EnemyTarget) -> Task:
        """将EnemyTarget对象转换为xfd_allocation库所需的Task对象"""
        task = Task()
        task.task_id = enemy.id
        task.x = enemy.position[0]
        task.y = enemy.position[1]
        task.z = enemy.position[2]
        task.task_status = (enemy.status != TargetStatus.DESTROYED)
        task.str_need = enemy.threat_level
        task.robot_need = enemy.robot_need
        return task

    def _get_available_uavs_for_attack(self) -> List[int]:
        """获取可用于攻击的无人机"""
        available_uavs = []
        
        for uav_id, uav in self.uav_dict.items():
            # TODO: 检查无人机状态、燃料、武器等
            if uav.status == UAVStatus.ACTIVE:
                available_uavs.append(uav_id)
        
        return available_uavs
    
    # ! ==================== 阶段4: 封控阶段 ====================
    def execute_containment_phase(self):
        """执行封控阶段：在封控区域内生成圆形巡逻区域并分配UAV"""
        print("\n--- 开始封控阶段：分配圆形巡逻任务 ---")
        self.current_phase = MissionType.CONTAINMENT
        self.phase_start_time = time.time()
        
        # 获取需要执行封控任务的UAV：
        all_active_uavs = [uav for uav in self.uav_dict.values() if uav.status == UAVStatus.ACTIVE]
        
        # 先确保所有非攻击任务的UAV都明确设置为封控状态
        for uav in all_active_uavs:
            if uav.current_mission != MissionType.ATTACK:
                uav.current_mission = MissionType.CONTAINMENT
        
        # 然后筛选出所有需要执行封控任务的UAV
        containment_uavs = [
            uav for uav in all_active_uavs 
            if uav.current_mission == MissionType.CONTAINMENT
        ]
        
        if not containment_uavs:
            print("无UAV需要执行封控任务")
            print(f"调试信息: 总UAV数={len(self.uav_dict)}, 存活UAV数={len(all_active_uavs)}")
            print(f"调试信息: 攻击任务UAV数={len([u for u in all_active_uavs if u.current_mission == MissionType.ATTACK])}")
            print(f"调试信息: 其他状态UAV={[(u.id, u.current_mission.value) for u in all_active_uavs if u.current_mission != MissionType.ATTACK and u.current_mission != MissionType.CONTAINMENT]}")
            self.phase_completion[MissionType.CONTAINMENT] = True
            return
        
        print(f"共 {len(containment_uavs)} 架UAV需要执行封控任务")
        print(f"封控UAV ID列表: {[uav.id for uav in containment_uavs]}")
        
        # 为每个封控区域分配UAV和规划巡逻路径
        for zone in self.containment_zones:
            if not containment_uavs:
                break
                
            # 计算该区域需要的UAV数量
            available_count = len(containment_uavs)
            zone_uav_count = min(available_count, 100)
            
            # 分配UAV到该区域
            zone_uavs = containment_uavs[:zone_uav_count]
            containment_uavs = containment_uavs[zone_uav_count:]
            
            print(f"为封控区域 {zone.id} 分配 {len(zone_uavs)} 架UAV")
            
            # 在封控区域内生成圆形巡逻区域
            patrol_assignments = self._plan_containment_patrol(zone, zone_uavs)
            
            # 为每个UAV设置巡逻任务
            for uav, (center, radius) in patrol_assignments.items():
                uav.set_patrol_assignment(center, radius, zone.patrol_altitude)
        
        # 检查是否还有未分配巡逻任务的UAV
        if containment_uavs:
            print(f"警告: 还有 {len(containment_uavs)} 架UAV未被分配到巡逻任务（封控区域已满）")
            print(f"未分配UAV ID: {[uav.id for uav in containment_uavs]}")
            # 为这些UAV也分配一个简单的巡逻任务（使用第一个封控区域的参数）
            if self.containment_zones:
                first_zone = self.containment_zones[0]
                # 在第一个封控区域附近生成额外的巡逻圆
                additional_patrol_assignments = self._plan_containment_patrol(first_zone, containment_uavs)
                for uav, (center, radius) in additional_patrol_assignments.items():
                    uav.set_patrol_assignment(center, radius, first_zone.patrol_altitude)
        
        # 最终统计
        assigned_count = len([uav for uav in self.uav_dict.values() 
                              if uav.current_mission == MissionType.CONTAINMENT and uav.patrol_center is not None])
        print(f"封控阶段巡逻任务分配完成: 共 {assigned_count} 架UAV已分配巡逻任务")
        self.phase_completion[MissionType.CONTAINMENT] = True
        

    def _plan_containment_patrol(self, zone: ContainmentZone, uavs: List[UAV]) -> Dict[UAV, Tuple[Tuple[float, float], float]]:
        """在封控区域内生成圆形巡逻区域"""
        
        # 调用region_isolation.py的函数生成圆形巡逻区域
        circle_num = len(uavs)
        rect_width = zone.width
        rect_height = zone.height
        rect_center_x = zone.center[0]
        rect_center_y = zone.center[1]
        
        # 计算最佳圆半径
        circle_radius, circle_centers = self._generate_patrol_circles(
            circle_num, rect_width, rect_height, rect_center_x, rect_center_y
        )
        
        print(f"封控区域 {zone.id}: 生成 {len(circle_centers)} 个巡逻圆，半径 {circle_radius:.1f}m")
        
        # 为每个UAV分配最近的巡逻圆
        assignments = {}
        used_centers = set()
        
        for uav in uavs:
            best_center = None
            min_distance = float('inf')
            
            # 找到距离UAV当前位置最近的未分配巡逻圆
            for i, center in enumerate(circle_centers):
                if i in used_centers:
                    continue
                    
                distance = np.linalg.norm(np.array([uav.position[0] - center[0], uav.position[1] - center[1]]))
                if distance < min_distance:
                    min_distance = distance
                    best_center = (i, center)
            
            if best_center:
                center_index, center = best_center
                used_centers.add(center_index)
                assignments[uav] = (center, circle_radius)
        
        return assignments

    def _generate_patrol_circles(self, circle_num: int, rect_width: float, rect_height: float, 
                           rect_center_x: float, rect_center_y: float) -> Tuple[float, List[Tuple[float, float]]]:
        """生成圆形巡逻区域"""
        
        # 设置圆半径范围
        min_circle_radius = 50.0   # 最小50米
        max_circle_radius = 200.0  # 最大200米
        
        # 直接调用region_isolation.py的函数
        circle_radius, circle_centers = generate_circles_in_rectangle(
            circle_num=circle_num,
            rect_width=rect_width,
            rect_height=rect_height,
            rect_center_x=rect_center_x,
            rect_center_y=rect_center_y,
            min_circle_radius=min_circle_radius,
            max_circle_radius=max_circle_radius
        )
        
        print(f"成功生成 {len(circle_centers)} 个巡逻圆（目标 {circle_num} 个），半径 {circle_radius:.1f}m")
        
        return circle_radius, circle_centers
    
    # ! ==================== 任务控制方法 ====================
    def _check_random_destruction(self, phase: str, dt: float):
        """
        在准备和侦查阶段检查是否有无人机被随机损毁
        
        参数:
        phase: 当前阶段 ("PREPARATION" 或 "RECONNAISSANCE")
        dt: 时间步长
        
        返回:
        List[Tuple[int, int]]: [(attacker_id, target_id)] 损毁事件列表
        """
        if not RANDOM_DESTRUCTION_ENABLED:
            return []

        if DAMAGE_RATIO <= 0.0:
            return []

        if phase not in ["PREPARATION", "RECONNAISSANCE"]:
            return []
        
        destruction_events = []
        
        # 计算当前已损毁的UAV数量
        destroyed_uavs = [uav for uav in self.uav_dict.values() if uav.status == UAVStatus.DESTROYED]
        
        # 计算最大允许损毁数量（基于DAMAGE_RATIO）
        max_allowed_destructions = int(len(self.uav_dict) * DAMAGE_RATIO)
        
        # 如果已达到损毁上限，不再损毁
        if len(destroyed_uavs) >= max_allowed_destructions:
            return []
        
        # 为避免侦查进度受阻，准备/侦查阶段不摧毁领航者
        active_uavs = [
            uav for uav in self.uav_dict.values()
            if uav.status == UAVStatus.ACTIVE and not uav.is_leader
        ]
        
        if not active_uavs:
            return []
        
        # 获取高威胁敌方目标（威胁级别 >= 3）
        high_threat_targets = [tgt for tgt in self.attack_targets 
                            if tgt.status != TargetStatus.DESTROYED and tgt.threat_level >= 3]
        
        # 如果没有高威胁目标，使用所有活跃目标
        if not high_threat_targets:
            high_threat_targets = [tgt for tgt in self.attack_targets 
                                if tgt.status != TargetStatus.DESTROYED]
        
        # 计算威胁因子（0.5 到 3.0 之间）
        threat_factor = 0.5 + min(len(high_threat_targets) * 0.0035, 2.5)
        
        # 计算每帧损毁概率（上限为0.1，避免单帧损毁过多）
        destruction_probability_per_frame = min(DAMAGE_RATIO * threat_factor * dt * 0.1, 0.1)
        
        # 随机检查每个活跃无人机是否被损毁
        for uav in active_uavs:
            if np.random.random() < destruction_probability_per_frame:
                # 选择攻击该无人机的高威胁目标
                if high_threat_targets:
                    attacker_target = np.random.choice(high_threat_targets)
                    # 损毁无人机
                    uav.destroy()
                    print(f"【随机损毁】UAV-{uav.id} 在{phase}阶段被敌方目标 {attacker_target.id} (威胁级别{attacker_target.threat_level}) 击中并损毁")
                    destruction_events.append((attacker_target.id, uav.id))  # (attacker_id, target_id)
        
        return destruction_events
    
    def _check_attack_completion(self):
        """
        检查攻击任务完成情况，并发送损毁信息
        注意：UAV在update_position中到达目标后会自动destroy，所以这里检查DESTROYED状态的UAV
        
        返回:
        List[Tuple[int, int]]: [(uav_id, target_id)] 完成的攻击任务列表
        """
        attack_completions = []
        
        for uav in self.uav_dict.values():
            # 检查刚刚完成攻击的UAV（状态为DESTROYED，且任务为ATTACK，且有攻击目标）
            if (uav.status == UAVStatus.DESTROYED and
                uav.current_mission == MissionType.ATTACK and
                uav.attack_target_position is not None):
                
                # 找到对应的目标
                target = None
                min_dist = float('inf')
                for tgt in self.attack_targets:
                    dist = np.linalg.norm(np.array(uav.attack_target_position[:2]) - np.array(tgt.position[:2]))
                    if dist < min_dist:
                        min_dist = dist
                        target = tgt
                
                if target:
                    # 检查是否已经报告过
                    event_key = (uav.id, target.id)
                    if event_key not in self.reported_destructions:
                        # 标记目标为已摧毁
                        target.status = TargetStatus.DESTROYED

                        # 记录攻击完成时间（首次完成时）
                        if target.id not in self.attack_end_time_map:
                            self.attack_end_time_map[target.id] = time.time()

                        attack_completions.append((uav.id, target.id))
                        self.reported_destructions.add(event_key)
                        print(f"【攻击完成】UAV-{uav.id} 成功摧毁目标 {target.id}，并发送损毁信息")
        
        return attack_completions
    
    def send_attack_data(self, destruction_events: Dict[int, int], init_flag = False):
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
            drone_is_death = 0  # 默认值
            
            # 判断攻击者类型，确定droneIsDeath
            if attacker_id < 0:
                # 虚拟攻击者ID（负数）表示初始化销毁，droneIsDeath=0
                drone_is_death = 0
            elif attacker_id in self.uav_dict:
                # 攻击者是UAV
                if target_id in self.uav_dict:
                    # UAV攻击UAV（这种情况较少见）
                    drone_is_death = 0  # 攻击者UAV不会死亡（假设）
                else:
                    # UAV攻击敌方目标，UAV会死亡
                    drone_is_death = 1
            else:
                # 攻击者是敌方目标（在attack_targets中）
                attacker_target = next((tgt for tgt in self.attack_targets if tgt.id == attacker_id), None)
                if attacker_target:
                    # 敌方目标攻击UAV，敌方目标不会死亡
                    drone_is_death = 0
                else:
                    # 未知攻击者类型，默认不死亡
                    drone_is_death = 0
            
            # 检查被摧毁的是否为我方无人机
            if target_id in self.uav_dict:
                destroyed_uav = self.uav_dict[target_id]
                if destroyed_uav.status != UAVStatus.DESTROYED:
                    destroyed_uav.destroy()
                    print(f"UAV-{attacker_id} 摧毁了UAV-{target_id}。")
                target_found = True
            else:
                # 检查被摧毁的是否为敌方目标（在attack_targets中）
                target_to_destroy = next((tgt for tgt in self.attack_targets if tgt.id == target_id), None)
                if target_to_destroy:
                    if target_to_destroy.status != TargetStatus.DESTROYED:
                        target_to_destroy.status = TargetStatus.DESTROYED
                        print(f"UAV-{attacker_id} 摧毁了敌方目标 {target_id}。")
                    target_found = True
                else:
                    # 目标不在attack_targets中（可能是初始化时未被选中的目标，已标记为DESTROYED）
                    # 这种情况是正常的，只发送消息，不更新状态
                    if attacker_id < 0:  # 虚拟攻击者ID（负数）表示初始化销毁
                        print(f"系统初始化时销毁目标 {target_id}（不在当前任务列表中）。")
                        target_found = True  # 视为已处理，不打印警告
            
            # if not target_found:
            #     print(f"初始化时，销毁未使用的目标 {target_id}")

            if init_flag:
                drone_is_death = 0

            # 2. 构造消息体（无论是否找到目标，都发送消息）
            agents_list.append({
                "drone_id": attacker_id,
                "target_id": target_id,
                "droneIsDeath": drone_is_death
            })

        # 3. 构造并发送完整的TCP消息
        destruction_payload = {
            "key": "DestoryEntity",
            "name": "ResearchGroup6",
            "timestamp": time.time(),
            "agents": agents_list
        }

        if self.tcp_client and self.tcp_client.connected:
            self.tcp_client.send_json(destruction_payload)
            # TODO  後面刪除
            print(f"损毁信息发送成功: {destruction_payload}")
        else:
            print("警告: TCP客户端未连接，无法发送损毁信息。")
    
    def get_mission_status(self) -> Dict[str, Any]:
        """获取任务状态"""
        return {
            "current_phase": self.current_phase.value,
            "phase_duration": time.time() - self.phase_start_time,
            "total_duration": time.time() - self.mission_start_time,
            "phase_completion": {k.value: v for k, v in self.phase_completion.items()},
            "active_uavs": len(self.uav_dict),
            "detected_targets": len(self.attack_targets),
            "containment_zones": len(self.containment_zones)
        }
    
    # ! ==================== TCP/IP通信方法 ====================
    def _initialize_tcp_client(self):
        """初始化TCP客户端"""
        # SERVER_HOST = '10.66.1.93'  中电普信
        SERVER_HOST = '10.66.1.86'   # 湖大
        SERVER_PORT = 13334
        CLIENT_IP = '10.66.1.192'
        self.tcp_client = TCPClient(host=SERVER_HOST, port=SERVER_PORT, client_ip=CLIENT_IP)
        if not self.tcp_client.connect():
            print(f"警告: TCP连接失败, 数据将不会被发布.")
            self.tcp_client = None
        else:
            # TCP连接成功后，发送初始化时待发送的销毁消息
            if self.pending_initial_destructions:
                print(f"TCP连接成功，发送 {len(self.pending_initial_destructions)} 个初始化销毁消息。")
                self.send_attack_data(self.pending_initial_destructions)
                self.pending_initial_destructions = None  # 清空待发送列表

    # TODO：注意，uav的速度不能为0,否则上层仿真不会更新uav位置
    def _publish_uav_data_to_tcp(self):
        """将所有无人机的状态数据通过TCP发布"""
        # if not self.tcp_client or not self.pos_converter:
        #     return

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

        filename = f"communication/json/ResearchGroup6ResultTest.json"
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(poses_data, f, ensure_ascii=False, indent=4)
        except Exception as e:
            print(f"保存数据到文件 {filename} 时发生错误: {e}")

    def run_tcp_simulation(self):
        """
        运行完整的四阶段TCP仿真循环。
        自动进行阶段切换：准备->侦察->打击->封控
        """
        frequency = TCP_PUBLISH_RATE  # 使用统一的发布频率
        dt = SIMULATION_DT  # 使用统一的仿真步长
        
        print(f"--- 开始完整四阶段TCP仿真循环 (频率: {frequency}Hz, dt: {dt}s) ---")
        
        # 阶段控制变量
        current_phase = "PREPARATION"
        reconnaissance_started = False
        attack_started = False
        containment_started = False
        
        # TCP发布频率控制
        tcp_publish_interval = 1.0 / frequency
        last_tcp_publish_time = time.time()
        
        try:
            frame_count = 0
            while not rospy.is_shutdown():
                loop_start_time = time.time()
            
                # === 阶段切换逻辑 ===
                if current_phase == "PREPARATION":
                    leaders = [uav for uav in self.uav_dict.values() if uav.is_leader]
                    if leaders and any(l.path_planning_complete for l in leaders):
                        if not reconnaissance_started:
                            current_phase = "RECONNAISSANCE"
                            reconnaissance_started = True
                            self.current_phase = MissionType.RECONNAISSANCE
                            print(f"\n=== 切换到侦察阶段 ===")
                
                elif current_phase == "RECONNAISSANCE":
                    leaders = [uav for uav in self.uav_dict.values() if uav.is_leader and uav.path_planning_complete]
                    
                    # 非leader UAV执行本地扫描，发现高威胁目标则立即指派打击
                    recon_targets_collected = []
                    for uav in self.uav_dict.values():
                        if uav.status != UAVStatus.ACTIVE or uav.is_leader:
                            continue
                        detections = self._detect_high_threat_targets_in_rect(
                            center_xy=(uav.position[0], uav.position[1]), half_width=100.0, half_height=100.0
                        )
                        if detections:
                            recon_targets_collected.extend(detections)
                    
                    if recon_targets_collected:
                        # 去重：按target id
                        unique_targets = {t.id: t for t in recon_targets_collected}
                        self._assign_attackers_for_targets(list(unique_targets.values()), list(self.uav_dict.values()))
                        # 立即检查是否已有完成的攻击并上报
                        completions = self._check_attack_completion()
                        if completions:
                            destruction_dict = {uav_id: target_id for uav_id, target_id in completions}
                            self.send_attack_data(destruction_dict)
                    
                    # 检查是否所有领航者完成侦察路径
                    if leaders and all(l.is_path_complete for l in leaders):
                        if not attack_started:
                            current_phase = "ATTACK"
                            attack_started = True
                            self.current_phase = MissionType.ATTACK
                            print(f"\n=== 切换到打击阶段 ===")
                            self.execute_attack_phase()
                
                elif current_phase == "ATTACK":
                    # 检查是否有可用于封控的UAV
                    containment_eligible_uavs = [uav for uav in self.uav_dict.values() 
                                                if uav.status == UAVStatus.ACTIVE and uav.current_mission != MissionType.ATTACK]
                    
                    # 当有可用于封控的UAV时，立即切换到封控阶段
                    if containment_eligible_uavs and not containment_started:
                        current_phase = "CONTAINMENT"
                        containment_started = True
                        self.current_phase = MissionType.CONTAINMENT
                        print(f"\n=== 切换到封控阶段 ===")
                        print(f"发现 {len(containment_eligible_uavs)} 架UAV可用于封控任务")
                        self.execute_containment_phase()
                
                # === 位置更新逻辑 ===
                if current_phase in ["PREPARATION", "RECONNAISSANCE"]:
                    for uav in self.uav_dict.values():
                        if not uav.is_leader and uav.leader is not None:
                            leader_pos, leader_heading = uav.leader.position, uav.leader.heading
                            offset_x, offset_y = uav.formation_offset
                            
                            rotated_offset_x = offset_x * np.cos(leader_heading) - offset_y * np.sin(leader_heading)
                            rotated_offset_y = offset_x * np.sin(leader_heading) + offset_y * np.cos(leader_heading)
                            
                            target_pos = (leader_pos[0] + rotated_offset_x, leader_pos[1] + rotated_offset_y, leader_pos[2])
                            uav.set_formation_target(target_pos, leader_heading)
                
                # === 随机损毁检查 ===
                if current_phase in ["PREPARATION", "RECONNAISSANCE"]:
                    random_destructions = self._check_random_destruction(current_phase, dt)
                    if random_destructions:
                        destruction_dict = {attacker_id: uav_id for attacker_id, uav_id in random_destructions}
                        print("随机损毁事件：", destruction_dict)
                        self.send_attack_data(destruction_dict)
                
                # 更新所有存活的无人机位置
                for uav in self.uav_dict.values():
                    if uav.status == UAVStatus.ACTIVE:
                        uav.update_position(dt)

                # === 攻击任务完成检查 ===
                if current_phase == "CONTAINMENT":
                    attack_completions = self._check_attack_completion()
                    if attack_completions:
                        destruction_dict = {uav_id: target_id for uav_id, target_id in attack_completions}
                        self.send_attack_data(destruction_dict)

                    if self.attack_assigned_uav_ids:
                        all_attack_done = all(
                            self.uav_dict[uav_id].status == UAVStatus.DESTROYED
                            for uav_id in self.attack_assigned_uav_ids
                        )
                    else:
                        all_attack_done = True   # 没有攻击任务，直接算

                    if all_attack_done and not hasattr(self, '_metrics_already_computed'):
                        self._metrics_already_computed = True
                        print("\n=== 所有攻击 UAV 已摧毁，计算最终指标 ===")
                        o1, d1, a1 = self.compute_metrics_reference()
                        write_results_to_excel(
                            uav_num=self.total_uavs,
                            damage_ratio=DAMAGE_RATIO,
                            o1_result=o1,
                            d1_result=d1,
                            a1_result=a1,
                            filename="./result.xlsx"
                        )
                
                # === TCP数据发布（按固定频率）===
                current_time = time.time()
                if current_time - last_tcp_publish_time >= tcp_publish_interval:
                    self._publish_uav_data_to_tcp()
                    last_tcp_publish_time = current_time
                
                # 控制循环频率
                elapsed = time.time() - loop_start_time
                sleep_duration = 0.25 - elapsed
                if sleep_duration > 0:
                    time.sleep(sleep_duration)
                
                frame_count += 1
                
        except KeyboardInterrupt:
            print("\nTCP仿真循环被用户中断。")
        finally:
            print("--- TCP仿真循环结束 ---")

    def disconnect(self):
        """Safely disconnects the TCP client if it's connected."""
        if self.tcp_client and self.tcp_client.connected:
            print("Disconnecting TCP client...")
            self.tcp_client.disconnect()

    def animate_complete_mission(self, max_steps=20000):
        """可视化完整的四阶段任务：准备->侦察->打击->封控"""
        print("开始完整任务的动态仿真与可视化...")
        global TCP_PUBLISH_RATE, TCP_PUBLISH_RATE, SIMULATION_DT
        # 使用统一的仿真参数
        interval = 250
        dt = TCP_PUBLISH_RATE * SIMULATION_DT / (1000 / interval)
            
        # 1. ======== 动画设置 ========
        fig, ax = plt.subplots(figsize=(18, 14))
        ax.set_aspect('equal', adjustable='box')
        ax.set_title('Complete Swarm Mission: Preparation → Reconnaissance → Attack → Containment', fontsize=16, fontweight='bold')
        ax.set_xlabel('East (m)', fontsize=12)
        ax.set_ylabel('North (m)', fontsize=12)
        ax.grid(True, alpha=0.3)
        
        # 设置固定的坐标轴范围，避免自动缩放导致的显示问题
        if self.uav_dict and self.attack_targets:
            all_x = []
            all_y = []
            
            # UAV初始位置
            for uav in self.uav_dict.values():
                all_x.append(uav.init_global_position[0])
                all_y.append(uav.init_global_position[1])
            
            # 目标位置
            for target in self.attack_targets:
                all_x.append(target.position[0])
                all_y.append(target.position[1])
            
            # 侦察区域
            for area in self.reconnaissance_areas:
                all_x.extend([area.center[0] - area.width/2, area.center[0] + area.width/2])
                all_y.extend([area.center[1] - area.height/2, area.center[1] + area.height/2])
            
            # 【新增】封控区域
            for zone in self.containment_zones:
                all_x.extend([zone.center[0] - zone.width/2, zone.center[0] + zone.width/2])
                all_y.extend([zone.center[1] - zone.height/2, zone.center[1] + zone.height/2])
            
            # 设置固定范围，添加边距
            margin = 300
            x_min, x_max = min(all_x) - margin, max(all_x) + margin
            y_min, y_max = min(all_y) - margin, max(all_y) + margin
            
            ax.set_xlim(x_min, x_max)
            ax.set_ylim(y_min, y_max)
            
            print(f"设置固定坐标轴范围: X[{x_min:.0f}, {x_max:.0f}], Y[{y_min:.0f}, {y_max:.0f}]")
        
        # 2. ======== 绘制静态元素 ========
        # 绘制侦察区域边界
        from matplotlib.patches import Rectangle, Circle
        for i, area in enumerate(self.reconnaissance_areas):
            bottom_left_x = area.center[0] - area.width / 2
            bottom_left_y = area.center[1] - area.height / 2
            rect = Rectangle(xy=(bottom_left_x, bottom_left_y), 
                            width=area.width, height=area.height,
                            color='blue', fill=False, linestyle='--', linewidth=2,
                            label='Reconnaissance Area' if i == 0 else None)
            ax.add_patch(rect)
            ax.text(area.center[0], area.center[1], f'Recon-{area.id}', 
                    fontsize=10, ha='center', va='center', 
                    bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
        
        # 【新增】绘制封控区域边界
        for i, zone in enumerate(self.containment_zones):
            bottom_left_x = zone.center[0] - zone.width / 2
            bottom_left_y = zone.center[1] - zone.height / 2
            rect = Rectangle(xy=(bottom_left_x, bottom_left_y), 
                            width=zone.width, height=zone.height,
                            color='green', fill=False, linestyle='-.', linewidth=2,
                            label='Containment Zone' if i == 0 else None)
            ax.add_patch(rect)
            ax.text(zone.center[0], zone.center[1], f'Containment-{zone.id}', 
                    fontsize=10, ha='center', va='center', 
                    bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.5))
        
        # 绘制敌方目标
        if self.attack_targets:
            for tgt in self.attack_targets:
                target_scatter = ax.scatter(tgt.position[0], tgt.position[1], 
                        c='red', marker='X', s=200, linewidths=2,
                        edgecolors='darkred', zorder=15)
                
                # 添加目标圆圈显示攻击范围
                target_circle = Circle((tgt.position[0], tgt.position[1]), 2.0, 
                                    fill=False, color='red', linestyle=':', 
                                    linewidth=1, alpha=0.6, zorder=14)
                ax.add_patch(target_circle)
                
                ax.text(tgt.position[0] + 150, tgt.position[1] + 150, 
                        f'T-{tgt.id}', fontsize=10, color='darkred', fontweight='bold',
                        bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.8),
                        zorder=16)
            
            # 添加图例
            ax.scatter([], [], c='red', marker='X', s=200, linewidths=2,
                    edgecolors='darkred', label='Enemy Targets')
        
        # 3. ======== 初始化绘图元素 ========
        uav_plots = {}
        patrol_circles = {}  # 存储巡逻圆的绘图对象
        colors = ['r', 'b', 'g', 'c', 'm', 'y', 'orange', 'purple']
        group_ids = sorted(self.group_assignments.keys())
        for i, group_id in enumerate(group_ids):
            color = colors[i % len(colors)]
            for uav_id in self.group_assignments[group_id]:
                uav = self.uav_dict[uav_id]
                line, = ax.plot([], [], color=color, linestyle='--', linewidth=1.0, alpha=0.7, zorder=5)
                point, = ax.plot(uav.position[0], uav.position[1], 'o', color=color, 
                                markersize=6, zorder=10)
                uav_plots[uav_id] = {
                    'line': line, 
                    'point': point, 
                    'is_leader': uav.is_leader,
                    'original_color': color
                }
        # 设置领航者样式
        for uav_id, plot_info in uav_plots.items():
            if plot_info['is_leader']:
                plot_info['line'].set_linewidth(2.5)
                plot_info['line'].set_linestyle('-')
                plot_info['point'].set_markersize(8)
                plot_info['point'].set_zorder(12)
        # 4. ======== 添加状态显示 ========
        time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, 
                            fontsize=12, verticalalignment='top',
                            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        phase_text = ax.text(0.02, 0.92, '', transform=ax.transAxes,
                            fontsize=14, verticalalignment='top', fontweight='bold',
                            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
        status_text = ax.text(0.02, 0.85, '', transform=ax.transAxes,
                            fontsize=10, verticalalignment='top',
                            bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))
        
        ax.legend(loc='upper right', fontsize=10)
        
        # 5. ======== 任务阶段控制变量 ========
        mission_phase = "PREPARATION"
        phase_start_frame = 0
        reconnaissance_started = False
        attack_started = False
        containment_started = False

        last_tcp_publish_time = time.time()
        frame_count = 0
        
        # 6. ======== 定义动画更新函数 ========
        def update(frame):
            nonlocal mission_phase, phase_start_frame, reconnaissance_started, attack_started, containment_started
            nonlocal last_tcp_publish_time, frame_count
            
            # === 阶段切换逻辑 ===
            if mission_phase == "PREPARATION":
                leaders = [uav for uav in self.uav_dict.values() if uav.is_leader]
                # 准备与侦察路径已合并：当任一领航者完成路径规划即可进入侦察阶段
                if leaders and any(l.path_planning_complete for l in leaders):
                    if not reconnaissance_started:
                        mission_phase = "RECONNAISSANCE"
                        phase_start_frame = frame
                        reconnaissance_started = True
                        print(f"\n=== 切换到侦察阶段 ===")
                        for uav in self.uav_dict.values():
                            if len(uav.position_history) > 100:
                                uav.position_history = uav.position_history[-100:]
            
            elif mission_phase == "RECONNAISSANCE":
                leaders = [uav for uav in self.uav_dict.values() if uav.is_leader and uav.path_planning_complete]
                # 在侦查阶段，非leader UAV执行本地扫描，发现高威胁目标则立即指派打击
                recon_targets_collected = []
                for uav in self.uav_dict.values():
                    if uav.status != UAVStatus.ACTIVE or uav.is_leader:
                        continue
                    detections = self._detect_high_threat_targets_in_rect(
                        center_xy=(uav.position[0], uav.position[1]), half_width=100.0, half_height=100.0
                    )
                    if detections:
                        recon_targets_collected.extend(detections)

                if recon_targets_collected:
                    unique_targets = {t.id: t for t in recon_targets_collected}
                    self._assign_attackers_for_targets(list(unique_targets.values()), list(self.uav_dict.values()))
                    completions = self._check_attack_completion()
                    if completions:
                        destruction_dict = {uav_id: target_id for uav_id, target_id in completions}
                        # self.send_attack_data(destruction_dict)  # 动画模式下不需要发送
                if leaders and all(l.is_path_complete for l in leaders):
                    if not attack_started:
                        mission_phase = "ATTACK"
                        phase_start_frame = frame
                        attack_started = True
                        print(f"\n=== 切换到打击阶段 ===")
                        self.execute_attack_phase()

            elif mission_phase == "ATTACK":
                # 检查是否有可用于封控的UAV（存活且不是攻击任务）
                containment_eligible_uavs = [uav for uav in self.uav_dict.values() 
                                                if uav.status == UAVStatus.ACTIVE and uav.current_mission != MissionType.ATTACK]
                    
                # 当有可用于封控的UAV时，立即切换到封控阶段
                if containment_eligible_uavs and not containment_started:
                    mission_phase = "CONTAINMENT"
                    phase_start_frame = frame
                    containment_started = True
                    print(f"\n=== 切换到封控阶段 ===")
                    print(f"发现 {len(containment_eligible_uavs)} 架UAV可用于封控任务")
                    
                    # 执行封控阶段初始化（会为所有符合条件的UAV分配巡逻任务）
                    self.execute_containment_phase()
                    
                    # 清理历史轨迹并更新绘图样式
                    for uav in self.uav_dict.values():
                        if uav.status == UAVStatus.ACTIVE:
                            if len(uav.position_history) > 50:
                                uav.position_history = uav.position_history[-50:]
                    
                    # 更新封控UAV的绘图样式
                    for uav_id, uav in self.uav_dict.items():
                        if (uav_id in uav_plots and 
                            uav.current_mission == MissionType.CONTAINMENT and 
                            uav.status == UAVStatus.ACTIVE):
                            plot_info = uav_plots[uav_id]
                            plot_info['point'].set_color('darkgreen')
                            plot_info['line'].set_color('darkgreen')
                            plot_info['point'].set_markersize(6)
                            plot_info['point'].set_marker('s')  # 方形标记
                    
                    # 绘制巡逻圆
                    for uav in self.uav_dict.values():
                        if (uav.current_mission == MissionType.CONTAINMENT and 
                            uav.patrol_center and uav.patrol_radius and 
                            uav.status == UAVStatus.ACTIVE):
                            circle = Circle(uav.patrol_center, uav.patrol_radius, 
                                        fill=False, color='darkgreen', linestyle=':', 
                                        alpha=0.7, linewidth=1.5)
                            ax.add_patch(circle)
                            patrol_circles[uav.id] = circle
            
            # === 位置更新逻辑 ===
            if mission_phase in ["PREPARATION", "RECONNAISSANCE"]:
                for uav in self.uav_dict.values():
                    if not uav.is_leader and uav.leader is not None:
                        leader_pos, leader_heading = uav.leader.position, uav.leader.heading
                        offset_x, offset_y = uav.formation_offset
                        
                        rotated_offset_x = offset_x * np.cos(leader_heading) - offset_y * np.sin(leader_heading)
                        rotated_offset_y = offset_x * np.sin(leader_heading) + offset_y * np.cos(leader_heading)
                        
                        target_pos = (leader_pos[0] + rotated_offset_x, leader_pos[1] + rotated_offset_y, leader_pos[2])
                        uav.set_formation_target(target_pos, leader_heading)
            
            # === 随机损毁检查（仅在准备和侦查阶段） ===
            if mission_phase in ["PREPARATION", "RECONNAISSANCE"]:
                random_destructions = self._check_random_destruction(mission_phase, dt)
                if random_destructions:
                    # 转换为字典格式并发送
                    destruction_dict = {attacker_id: uav_id for attacker_id, uav_id in random_destructions}
                    # self.send_attack_data(destruction_dict)   # 动画模式下不需要发送
            
            # 更新所有存活的无人机位置
            for uav in self.uav_dict.values():
                if uav.status == UAVStatus.ACTIVE:
                    uav.update_position(dt)
            
            # === 攻击任务完成检查 ===
            if mission_phase == "ATTACK":
                attack_completions = self._check_attack_completion()
                if attack_completions:
                    # 转换为字典格式并发送
                    destruction_dict = {uav_id: target_id for uav_id, target_id in attack_completions}
                    # self.send_attack_data(destruction_dict)  # 动画模式下不需要发送
            
            # === 绘图更新 ===
            artists = []
            for uav_id, uav in self.uav_dict.items():
                if uav_id in uav_plots and uav.status == UAVStatus.ACTIVE:
                    plots = uav_plots[uav_id]
                    history = np.array(uav.position_history)
                    if len(history) > 0:
                        plots['line'].set_data(history[:, 0], history[:, 1])
                        plots['point'].set_data([uav.position[0]], [uav.position[1]])
                        
                        # 【新增】为已到达目标的攻击UAV特殊标记
                        if (mission_phase == "ATTACK" and 
                            uav.current_mission == MissionType.ATTACK and 
                            uav.attack_target_position is not None):
                            
                            distance_to_target = np.linalg.norm(uav.position[:2] - uav.attack_target_position[:2])
                            if distance_to_target <= 2.0:
                                plots['point'].set_marker('s')
                                plots['point'].set_markersize(8)
                                plots['point'].set_markeredgewidth(2)
                                plots['point'].set_markeredgecolor('black')
                        
                        artists.extend([plots['line'], plots['point']])
                elif uav_id in uav_plots and uav.status == UAVStatus.DESTROYED:
                    # 被摧毁的UAV显示为X标记
                    plots = uav_plots[uav_id]
                    plots['point'].set_data([uav.position[0]], [uav.position[1]])
                    plots['point'].set_marker('x')
                    plots['point'].set_color('black')
                    plots['point'].set_markersize(10)
                    artists.append(plots['point'])
            
            # 更新文本信息
            phase_duration = (frame - phase_start_frame) * dt
            time_text.set_text(f'Simulation Time: {frame * dt:.1f}s')
            phase_text.set_text(f'Current Phase: {mission_phase}\n(Duration: {phase_duration:.1f}s)')
            
            # 统计信息显示
            if mission_phase == "ATTACK":
                attacking_uavs = [uav for uav in self.uav_dict.values() if uav.current_mission == MissionType.ATTACK]
                containment_uavs = [uav for uav in self.uav_dict.values() if uav.current_mission == MissionType.CONTAINMENT]
                destroyed_uavs = [uav for uav in self.uav_dict.values() if uav.status == UAVStatus.DESTROYED]
                
                arrived_count = 0
                total_distance = 0
                for uav in attacking_uavs:
                    if uav.attack_target_position is not None and uav.status == UAVStatus.ACTIVE:
                        distance = np.linalg.norm(uav.position[:2] - uav.atta_publish_uav_data_to_tcpck_target_position[:2])
                        total_distance += distance
                        if distance <= 2.0:
                            arrived_count += 1
                
                avg_distance = total_distance / max(1, len([u for u in attacking_uavs if u.status == UAVStatus.ACTIVE]))
                status_text.set_text(f'Attacking: {len([u for u in attacking_uavs if u.status == UAVStatus.ACTIVE])} ({arrived_count} arrived)\n'
                                    f'Containment: {len(containment_uavs)}\n'
                                    f'Destroyed: {len(destroyed_uavs)}\n'
                                    f'Avg Distance: {avg_distance:.1f}m')
            
            elif mission_phase == "CONTAINMENT":
                containment_uavs = [uav for uav in self.uav_dict.values() 
                                if uav.current_mission == MissionType.CONTAINMENT and uav.status == UAVStatus.ACTIVE]
                destroyed_uavs = [uav for uav in self.uav_dict.values() if uav.status == UAVStatus.DESTROYED]
                
                approaching_count = sum(1 for uav in containment_uavs if uav.patrol_phase == "approaching")
                patrolling_count = sum(1 for uav in containment_uavs if uav.patrol_phase == "patrolling")
                
                status_text.set_text(f'Containment UAVs: {len(containment_uavs)}\n'
                                    f'  Approaching: {approaching_count}\n'
                                    f'  Patrolling: {patrolling_count}\n'
                                    f'Destroyed: {len(destroyed_uavs)}')
            
            else:
                active_leaders = [uav for uav in self.uav_dict.values() if uav.is_leader and uav.path_planning_complete]
                completed_leaders = [uav for uav in active_leaders if uav.is_path_complete]
                status_text.set_text(f'Active Groups: {len(active_leaders)}\nCompleted: {len(completed_leaders)}')
            
            artists.extend([time_text, phase_text, status_text])
            
            return artists
        # 7. ======== 创建并启动动画 ========
        ani = animation.FuncAnimation(fig, update, frames=max_steps, 
                                    interval=interval, blit=True, repeat=False)
        plt.show()
        print("完整任务动画播放完毕。")

    # ! ==================== 指标计算 ====================
    def compute_metrics_reference(
        self,
        det_reward_weight: float = 200.0,
        atk_reward_weight: float = 100.0,
        decay_rate: float = 0.03,
    ) -> Tuple[float, float, float]:
        """计算并更新三个性能指标"""
        
        # 计算O1: 目标识别效率
        if self.detect_start_time is not None and self.detect_end_time_map:
            recognized_num = len(self.detect_end_time_map)
            # 总识别时间 = 所有目标识别时间之和
            total_recognition_time = sum(
                end - self.detect_start_time for end in self.detect_end_time_map.values()
            )
            self.O1_recognition_efficiency = recognized_num / total_recognition_time if recognized_num > 0 else 0.0
        else:
            self.O1_recognition_efficiency = 0.0
        
        # 计算D1: 任务决策效能
        def calculate_efficiency(t: float, w0: float, r: float) -> float:
            return w0 * math.exp(-r * max(0.0, t))
        
        # 识别部分奖励
        det_reward_total = 0.0
        if self.detect_start_time is not None and self.detect_end_time_map:
            for end_ts in self.detect_end_time_map.values():
                response_time = end_ts - self.detect_start_time
                det_reward_total += calculate_efficiency(response_time, det_reward_weight, decay_rate)
        
        # 打击部分奖励
        atk_reward_total = 0.0
        for tid in self.attack_start_time_map:
            if tid in self.attack_end_time_map:
                response_time = self.attack_end_time_map[tid] - self.attack_start_time_map[tid]
                atk_reward_total += calculate_efficiency(response_time, atk_reward_weight, decay_rate)
        
        # 目标总数
        total_targets = max(1, self.total_target_num)
        alpha = total_targets / (total_targets + 4.0)
        self.D1_planning_efficiency = alpha * det_reward_total + (1.0 - alpha) * atk_reward_total
        
        # 计算A1: 打击效率
        completed_attacks = [tid for tid in self.attack_end_time_map.keys() 
                            if tid in self.attack_start_time_map]
        if completed_attacks:
            min_start = min(self.attack_start_time_map[tid] for tid in completed_attacks)
            max_end = max(self.attack_end_time_map[tid] for tid in completed_attacks)
            total_attack_time = max(0.0, max_end - min_start)
            self.A1_attack_efficiency = len(completed_attacks) / total_attack_time if total_attack_time > 0 else 0.0
        else:
            self.A1_attack_efficiency = 0.0
        
        # 打印结果
        print("\n" + "="*50)
        print("性能指标计算结果:")
        print(f"O1 目标识别效率: {self.O1_recognition_efficiency:.4f} (目标/秒)")
        print(f"D1 任务决策效能: {self.D1_planning_efficiency:.2f} (综合评分)")
        print(f"A1 打击效率: {self.A1_attack_efficiency:.4f} (目标/秒)")
        print("="*50)
        
        return self.O1_recognition_efficiency, self.D1_planning_efficiency, self.A1_attack_efficiency


# ==================== 使用示例 ====================
if __name__ == "__main__":
    # 为独立运行此文件而创建的 Mock Planner
    class SimpleDubinsPlanner:
        turning_radius = 100.0
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
    if not VISUAL_DEBUG_MODE:
        mission_manager._initialize_tcp_client()
    
    try:
        # --- 2. 设置外部接口 (使用 Mock Planner) ---
        path_planner = PathPlanner(dubins_planner=SimpleDubinsPlanner())
        mission_manager.set_path_planner(path_planner)
        
        print("=== 开始无人机集群任务仿真（从XML加载） ===")
        print(f"配置: 使用 {NUM_UAVS_TO_USE if NUM_UAVS_TO_USE else '全部'} 架UAV")
        
        # --- 3. 从XML文件初始化场景 ---
        # xml_file_path = '山地丛林-第二版.xml'
        xml_file_path = '山地丛林.xml'
        if not mission_manager.initialize_uavs_from_xml(xml_file_path, num_uavs_to_use=NUM_UAVS_TO_USE):
            print("场景初始化失败，程序退出。")
            exit()
            
        # 激活所有已加载的无人机
        for uav in mission_manager.uav_dict.values():
            uav.activate()

        # --- 4. 根据模式执行不同流程 ---
        if VISUAL_DEBUG_MODE:
            # 阶段1: 准备阶段 - 仅设置领航者和随从角色
            mission_manager.execute_preparation_phase()

            # 阶段2: 侦察阶段 - 规划并合并准备+侦察路径
            mission_manager.execute_reconnaissance_phase()
            
            # mission_manager.animate_reconnaissance_phase()  # 可视化侦察阶段
            mission_manager.animate_complete_mission(max_steps=10000)

        else:
            # 【联调模式】: 初始化TCP并执行后台仿真
            print("--- 运行模式: TCP集成 ---")
            # 1. 规划所有路径
            mission_manager.execute_preparation_phase()
            mission_manager.execute_reconnaissance_phase()

            # 2. 运行统一的TCP仿真循环
            mission_manager.run_tcp_simulation()

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