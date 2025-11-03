#!/usr/bin/env python3

from re import T
import numpy as np
import time
from typing import List, Tuple, Optional, Any, Dict
import math
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import json
import os
import rospy

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
from xfd_allocation.scripts.CBPA.lib.CBPA import CBPA
from xfd_allocation.scripts.CBPA.lib.Robot import Robot
from xfd_allocation.scripts.CBPA.lib.Task import Task
from xfd_allocation.scripts.CBPA.lib.Region import Region
from xfd_allocation.scripts.CBPA.lib.WorldInfo import WorldInfo
from xfd_allocation.scripts.CBPA.lib.CBPA_REC import CBPA_REC


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
        self.total_uavs:int = 50
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

        # TODO: 后续统一使用RegionList表示
        self.RegionList: list = [
            Region(1, [4000, 0, 100], [650, 0, 100], [6500, -7000, 100], [4000, 0-7000, 100]),
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
    
    # TODO: 1.添加其他需要的队形配置  
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
            self.region_cover_planner.cover_run(uav_velocity = 30.0, turning_radius = 50.0, log_time='sim_run', cov_width=400)
            print(f"成功为 {len(self.region_cover_planner.all_path)} 个区域生成了覆盖路径。")
        except Exception as e:
            print(f"错误: 区域覆盖路径生成失败: {e}")

        # 5. 更新封控区域定义
        self.containment_zones = [
            ContainmentZone(
                id=1,
                center=(5250, -3500, 100),
                width=2500,
                height=7000,
                patrol_altitude=150,
                patrol_speed=25,
                assigned_uavs=[]
            )
        ]

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

                    self._plan_group_formation_movement(group_id_found, rally_point, "square_5x5")
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
        
        # 选择最靠近编队中心的无人机作为领航者
        # 1. 获取编队所有无人机的初始位置
        group_uavs = [self.uav_dict[uid] for uid in uav_ids if uid in self.uav_dict]
        if not group_uavs:
            print(f"警告: 编队 {group_id} 中没有有效的无人机对象。"); return
        
        positions = np.array([uav.init_global_position for uav in group_uavs])
        
        # 2. 计算几何中心
        centroid = np.mean(positions, axis=0)
        
        # 3. 找到离中心最近的无人机
        distances = np.linalg.norm(positions - centroid, axis=1)
        leader_index = np.argmin(distances)
        leader_uav = group_uavs[leader_index]
        leader_id = leader_uav.id
        
        print(f"为编队 {group_id} 设置移动任务")
        print(f"UAV-{leader_id} (最靠近中心) 被选为领航者。")
        
        # 2. 为领航者设置角色并规划路径
        leader_uav.set_as_leader()
        
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
            
        # print(f"{follower_count} 架无人机被设置为跟随者，将保持其初始相对位置。")


# ==================== 阶段2: 侦查阶段 ====================
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


    # ==================== 阶段3: 打击阶段 ====================
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
        
        print(f"已解散编队，共 {len(self.uav_dict)} 架UAV等待新任务分配。")
        
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
                    print(f"UAV-{uav_id} at ({uav.position[0]:.0f}, {uav.position[1]:.0f}) 被分配攻击目标 {target_id} at ({target.position[0]:.0f}, {target.position[1]:.0f})")
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
    
    def _execute_attack_missions(self, assignments: Dict[int, int]):
        """为分配了攻击任务的无人机规划并设置路径"""
        print(f"--- 正在为 {len(assignments)} 架无人机规划攻击路径 ---")
        for uav_id, target_id in assignments.items():
            uav = self.uav_dict.get(uav_id)
            target = next((t for t in self.attack_targets if t.id == target_id), None)
            
            if uav and target:
                print(f"UAV-{uav_id} 前往攻击目标 {target_id} at {target.position}")
                
                # 规划从无人机当前位置到目标上方的简单路径
                attack_path = self.path_planner.plan_leader_path(
                    start_pos=(uav.position[0], uav.position[1], uav.heading),
                    waypoints=[target.position]
                )
                
                if attack_path:
                    # 【修复】将UAV设为领航者，这样它才能沿着规划的路径移动
                    uav.set_as_leader()
                    uav.set_planned_path(attack_path)
                    uav.current_mission = MissionType.ATTACK # 更新无人机当前任务状态
                else:
                    print(f"警告: 无法为 UAV-{uav_id} 规划攻击路径。")

                # 模拟攻击结果 (简单模型)
                target.status = TargetStatus.ATTACKED
    
    # ==================== 阶段4: 封控阶段 ====================
    def execute_containment_phase(self):
        """执行封控阶段：在封控区域内生成圆形巡逻区域并分配UAV"""
        print("\n--- 开始封控阶段：分配圆形巡逻任务 ---")
        self.current_phase = MissionType.CONTAINMENT
        self.phase_start_time = time.time()
        
        # 获取需要执行封控任务的UAV
        containment_uavs = [
            uav for uav in self.uav_dict.values() 
            if uav.current_mission == MissionType.CONTAINMENT and uav.status == UAVStatus.ACTIVE
        ]
        
        if not containment_uavs:
            print("无UAV需要执行封控任务")
            self.phase_completion[MissionType.CONTAINMENT] = True
            return
        
        print(f"共 {len(containment_uavs)} 架UAV需要执行封控任务")
        
        # 为每个封控区域分配UAV和规划巡逻路径
        for zone in self.containment_zones:
            if not containment_uavs:
                break
                
            # 计算该区域需要的UAV数量
            available_count = len(containment_uavs)
            zone_uav_count = min(available_count, 20)  # 每个区域最多20架UAV
            
            # 分配UAV到该区域
            zone_uavs = containment_uavs[:zone_uav_count]
            containment_uavs = containment_uavs[zone_uav_count:]
            
            print(f"为封控区域 {zone.id} 分配 {len(zone_uavs)} 架UAV")
            
            # 在封控区域内生成圆形巡逻区域
            patrol_assignments = self._plan_containment_patrol(zone, zone_uavs)
            
            # 为每个UAV设置巡逻任务
            for uav, (center, radius) in patrol_assignments.items():
                uav.set_patrol_assignment(center, radius, zone.patrol_altitude)
        
        print("封控阶段巡逻任务分配完成")
        self.phase_completion[MissionType.CONTAINMENT] = True

    def _plan_containment_patrol(self, zone: ContainmentZone, uavs: List[UAV]) -> Dict[UAV, Tuple[Tuple[float, float], float]]:
        """基于region_isolation.py的逻辑，在封控区域内生成圆形巡逻区域"""
        
        # 使用region_isolation.py的逻辑生成圆形巡逻区域
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
                print(f"UAV-{uav.id} 分配到巡逻圆 {center}，距离 {min_distance:.1f}m")
        
        return assignments

    def _generate_patrol_circles(self, circle_num: int, rect_width: float, rect_height: float, 
                           rect_center_x: float, rect_center_y: float) -> Tuple[float, List[Tuple[float, float]]]:
        """基于region_isolation.py的算法生成圆形巡逻区域"""
        
        # 设置圆半径范围
        min_circle_radius = 50.0   # 最小50米
        max_circle_radius = 200.0  # 最大200米
        
        # 计算最佳圆半径
        rect_area = rect_height * rect_width
        expect_circle_area = rect_area / circle_num
        expect_circle_radius = np.sqrt(expect_circle_area / np.pi)
        
        if expect_circle_radius < min_circle_radius:
            circle_radius = min_circle_radius
        elif expect_circle_radius > max_circle_radius:
            circle_radius = max_circle_radius
        else:
            circle_radius = expect_circle_radius
        
        print(f"计算得到最佳巡逻圆半径: {circle_radius:.1f}m")
        
        # 计算矩形边界
        x_min = rect_center_x - rect_width / 2
        x_max = rect_center_x + rect_width / 2
        y_min = rect_center_y - rect_height / 2
        y_max = rect_center_y + rect_height / 2
        
        circle_centers = []
        
        def add_circle(x, y):
            if len(circle_centers) < circle_num:
                circle_centers.append((x, y))
        
        def is_valid_circle(x, y):
            # 检查是否在矩形范围内
            if (x - circle_radius < x_min or x + circle_radius > x_max or
                y - circle_radius < y_min or y + circle_radius > y_max):
                return False
            
            # 检查是否与已有圆重叠
            for cx, cy in circle_centers:
                if np.sqrt((cx - x) ** 2 + (cy - y) ** 2) < 1.8 * circle_radius:  # 允许轻微重叠
                    return False
            return True
        
        # 采用网格布局生成圆心
        spacing = circle_radius * 1.8  # 圆心间距
        cols = max(1, int(rect_width / spacing))
        rows = max(1, int(rect_height / spacing))
        
        start_x = x_min + circle_radius
        start_y = y_min + circle_radius
        
        # 生成网格状圆心
        for i in range(rows):
            for j in range(cols):
                if len(circle_centers) >= circle_num:
                    break
                
                x = start_x + j * spacing
                y = start_y + i * spacing
                
                if is_valid_circle(x, y):
                    add_circle(x, y)
        
        # 如果圆心数量不足，尝试随机填充
        max_attempts = 100
        attempts = 0
        while len(circle_centers) < circle_num and attempts < max_attempts:
            x = np.random.uniform(x_min + circle_radius, x_max - circle_radius)
            y = np.random.uniform(y_min + circle_radius, y_max - circle_radius)
            
            if is_valid_circle(x, y):
                add_circle(x, y)
            
            attempts += 1
        
        print(f"成功生成 {len(circle_centers)} 个巡逻圆（目标 {circle_num} 个）")
        
        return circle_radius, circle_centers
    
    # ==================== 任务控制方法 ====================
    def send_attack_data(self, destruction_events: Dict[int, int]):
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

    def run_tcp_simulation(self, frequency=2.0):
        """
        运行完整的四阶段TCP仿真循环。
        自动进行阶段切换：准备->侦察->打击->封控
        """
        print(f"--- 开始完整四阶段TCP仿真循环 (频率: {frequency}Hz) ---")
        dt = 1.0 / frequency
        
        # 阶段控制变量
        current_phase = "PREPARATION"
        reconnaissance_started = False
        attack_started = False
        containment_started = False  # 【新增】
        
        try:
            frame_count = 0
            while not rospy.is_shutdown():
                loop_start_time = time.time()
                
                # === 阶段切换逻辑 ===
                if current_phase == "PREPARATION":
                    leaders = [uav for uav in self.uav_dict.values() if uav.is_leader and uav.path_planning_complete]
                    if leaders and all(l.is_path_complete for l in leaders):
                        if not reconnaissance_started:
                            current_phase = "RECONNAISSANCE"
                            reconnaissance_started = True
                            self.current_phase = MissionType.RECONNAISSANCE
                            
                elif current_phase == "RECONNAISSANCE":
                    leaders = [uav for uav in self.uav_dict.values() if uav.is_leader and uav.path_planning_complete]
                    if leaders and all(l.is_path_complete for l in leaders):
                        if not attack_started:
                            current_phase = "ATTACK"
                            attack_started = True
                            self.current_phase = MissionType.ATTACK
                            self.execute_attack_phase()
                            
                elif current_phase == "ATTACK":
                    attacking_uavs = [uav for uav in self.uav_dict.values() 
                                    if uav.current_mission == MissionType.ATTACK and uav.status == UAVStatus.ACTIVE]
                    
                    # 【修改】检查攻击是否完成
                    if not attacking_uavs or all(uav.status == UAVStatus.DESTROYED for uav in attacking_uavs):
                        if not containment_started:
                            current_phase = "CONTAINMENT"
                            containment_started = True
                            self.current_phase = MissionType.CONTAINMENT
                            print(f"\n=== TCP仿真：切换到封控阶段 ===")
                            self.execute_containment_phase()
                
                # 【新增】封控阶段逻辑
                elif current_phase == "CONTAINMENT":
                    containment_uavs = [uav for uav in self.uav_dict.values() 
                                    if uav.current_mission == MissionType.CONTAINMENT and uav.status == UAVStatus.ACTIVE]
                    
                    # 封控阶段可以持续运行，或设置特定的结束条件
                    # 这里可以添加封控阶段的结束条件，比如时间限制
                    if frame_count * dt > 3600:  # 1小时后结束
                        print(f"\n=== TCP仿真：封控阶段超时，任务结束 ===")
                        break
                
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
                
                # 更新所有存活的无人机位置
                for uav in self.uav_dict.values():
                    if uav.status == UAVStatus.ACTIVE:
                        uav.update_position(dt)
                
                # 通过TCP发布数据
                self._publish_uav_data_to_tcp()
                
                # 定期状态报告
                if frame_count % 100 == 0:
                    print(f"当前阶段: {current_phase}, 仿真时间: {frame_count * dt:.1f}s")
                    if current_phase in ["PREPARATION", "RECONNAISSANCE"]:
                        leaders = [uav for uav in self.uav_dict.values() if uav.is_leader]
                        completed = sum(1 for l in leaders if l.is_path_complete)
                        print(f"  编队进度: {completed}/{len(leaders)} 个编队完成")
                    elif current_phase == "ATTACK":
                        attacking = [uav for uav in self.uav_dict.values() 
                                if uav.current_mission == MissionType.ATTACK and uav.status == UAVStatus.ACTIVE]
                        destroyed = [uav for uav in self.uav_dict.values() if uav.status == UAVStatus.DESTROYED]
                        print(f"  攻击进度: {len(attacking)} 架UAV执行攻击任务, {len(destroyed)} 架已摧毁")
                    elif current_phase == "CONTAINMENT":
                        containment = [uav for uav in self.uav_dict.values() 
                                    if uav.current_mission == MissionType.CONTAINMENT and uav.status == UAVStatus.ACTIVE]
                        patrolling = sum(1 for uav in containment if uav.patrol_phase == "patrolling")
                        print(f"  封控进度: {len(containment)} 架UAV执行封控任务, {patrolling} 架正在巡逻")
                
                # 控制循环频率
                elapsed = time.time() - loop_start_time
                sleep_duration = dt - elapsed
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

    def plot_results(self):
        """可视化无人机初始位置、侦查区域和规划的侦查路径"""
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
        import numpy as np

        fig, ax = plt.subplots(figsize=(16, 12))
        ax.set_aspect('equal', adjustable='box')
        
        # 1. 绘制侦察区域
        for i, area in enumerate(self.reconnaissance_areas):
            bottom_left_x = area.center[0] - area.width / 2
            bottom_left_y = area.center[1] - area.height / 2
            rect = Rectangle(xy=(bottom_left_x, bottom_left_y), 
                             width=area.width, height=area.height,
                             color='blue', fill=False, linestyle='--', 
                             label='Recon Area' if i == 0 else "")
            ax.add_patch(rect)

        # 2. 绘制无人机初始位置和规划轨迹
        colors = plt.cm.jet(np.linspace(0, 1, len(self.group_assignments)))
        group_ids = sorted(self.group_assignments.keys())
        
        for i, group_id in enumerate(group_ids):
            color = colors[i]
            is_first_uav_in_group = True # 用于确保每组只添加一个 "Start" 标签
            
            for uav_id in self.group_assignments[group_id]:
                uav = self.uav_dict[uav_id]
                
                # 绘制初始位置
                init_pos = uav.init_global_position
                marker = 's' if uav.is_leader else 'o'
                label = f'Group {group_id} Start' if is_first_uav_in_group else None
                ax.scatter(init_pos[0], init_pos[1], color=color, marker=marker, s=50, label=label)
                is_first_uav_in_group = False

                # 绘制领航者规划路径
                if uav.is_leader and uav.waypoints:
                    # 准备阶段的路径（从初始位置到侦察路径起点）
                    prep_path_start = uav.init_global_position[:2]
                    # 【修改】使用索引访问numpy数组
                    recon_path_start = (uav.waypoints[0][0], uav.waypoints[0][1])
                    ax.plot([prep_path_start[0], recon_path_start[0]], 
                            [prep_path_start[1], recon_path_start[1]], 
                            color=color, linestyle=':', linewidth=1.2)

                    # 侦察阶段的覆盖路径
                    # 【修改】使用索引访问numpy数组
                    recon_path_points = np.array([(p[0], p[1]) for p in uav.waypoints])
                    ax.plot(recon_path_points[:, 0], recon_path_points[:, 1], color=color, 
                            linewidth=2.0, label=f'Group {group_id} Path')

        # 3. 绘制敌方目标位置
        if self.attack_targets:
            enemy_pos = np.array([tgt.position for tgt in self.attack_targets])
            ax.scatter(enemy_pos[:, 0], enemy_pos[:, 1], c='black', marker='x', s=100, label='Enemy Targets')

        ax.set_title('Planned Mission Trajectories Overview')
        ax.set_xlabel('East (m)')
        ax.set_ylabel('North (m)')
        ax.legend(loc='upper right')
        ax.grid(True)
        plt.show()

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
        from matplotlib.patches import Rectangle
        for i, area in enumerate(self.reconnaissance_areas):
            # 计算矩形左下角坐标
            bottom_left_x = area.center[0] - area.width / 2
            bottom_left_y = area.center[1] - area.height / 2
            rect = Rectangle(xy=(bottom_left_x, bottom_left_y), 
                             width=area.width, height=area.height,
                             color='blue', fill=False, linestyle='--', 
                             label='Recon Area' if i == 0 else None)
            ax.add_patch(rect)
 
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

    def animate_complete_mission(self, max_steps=20000, dt=0.5, interval=20):
        """可视化完整的四阶段任务：准备->侦察->打击->封控"""
        print("开始完整任务的动态仿真与可视化...")
        
        # 1. ======== 动画设置 ========
        fig, ax = plt.subplots(figsize=(18, 14))
        ax.set_aspect('equal', adjustable='box')
        ax.set_title('Complete Swarm Mission: Preparation → Reconnaissance → Attack → Containment', fontsize=16, fontweight='bold')
        ax.set_xlabel('East (m)', fontsize=12)
        ax.set_ylabel('North (m)', fontsize=12)
        ax.grid(True, alpha=0.3)
        
        # 【新增】设置固定的坐标轴范围，避免自动缩放导致的显示问题
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
        containment_started = False  # 【新增】封控阶段标志
        
        # 6. ======== 定义动画更新函数 ========
        def update(frame):
            nonlocal mission_phase, phase_start_frame, reconnaissance_started, attack_started, containment_started
            
            # === 阶段切换逻辑 ===
            if mission_phase == "PREPARATION":
                leaders = [uav for uav in self.uav_dict.values() if uav.is_leader and uav.path_planning_complete]
                if leaders and all(l.is_path_complete for l in leaders):
                    if not reconnaissance_started:
                        mission_phase = "RECONNAISSANCE"
                        phase_start_frame = frame
                        reconnaissance_started = True
                        for uav in self.uav_dict.values():
                            if len(uav.position_history) > 100:
                                uav.position_history = uav.position_history[-100:]
            
            elif mission_phase == "RECONNAISSANCE":
                leaders = [uav for uav in self.uav_dict.values() if uav.is_leader and uav.path_planning_complete]
                if leaders and all(l.is_path_complete for l in leaders):
                    if not attack_started:
                        mission_phase = "ATTACK"
                        phase_start_frame = frame
                        attack_started = True
                        self.execute_attack_phase()
                        
                        for uav in self.uav_dict.values():
                            if len(uav.position_history) > 100:
                                uav.position_history = uav.position_history[-100:]
                        
                        # 更新绘图样式
                        for uav_id, uav in self.uav_dict.items():
                            if uav_id in uav_plots:
                                plot_info = uav_plots[uav_id]
                                if uav.current_mission == MissionType.ATTACK:
                                    plot_info['point'].set_color('red')
                                    plot_info['line'].set_color('red')
                                    plot_info['point'].set_markersize(8)
                                elif uav.current_mission == MissionType.CONTAINMENT:
                                    plot_info['point'].set_color('gray')
                                    plot_info['line'].set_color('gray')
                                    plot_info['point'].set_markersize(6)
            
            # 【新增】打击阶段到封控阶段的切换
            elif mission_phase == "ATTACK":
                attacking_uavs = [uav for uav in self.uav_dict.values() 
                                if uav.current_mission == MissionType.ATTACK and uav.status == UAVStatus.ACTIVE]
                
                # 检查攻击是否完成（所有攻击UAV都被摧毁或到达目标）
                if not attacking_uavs or all(uav.status == UAVStatus.DESTROYED for uav in attacking_uavs):
                    if not containment_started:
                        mission_phase = "CONTAINMENT"
                        phase_start_frame = frame
                        containment_started = True
                        print(f"\n=== 切换到封控阶段 ===")
                        
                        # 执行封控阶段初始化
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
                        
                        # 【新增】绘制巡逻圆
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
            # 更新所有存活的无人机位置
            for uav in self.uav_dict.values():
                if uav.status == UAVStatus.ACTIVE:
                    uav.update_position(dt)
            
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
            
            # 【修改】统计信息显示
            if mission_phase == "ATTACK":
                attacking_uavs = [uav for uav in self.uav_dict.values() if uav.current_mission == MissionType.ATTACK]
                containment_uavs = [uav for uav in self.uav_dict.values() if uav.current_mission == MissionType.CONTAINMENT]
                destroyed_uavs = [uav for uav in self.uav_dict.values() if uav.status == UAVStatus.DESTROYED]
                
                arrived_count = 0
                total_distance = 0
                for uav in attacking_uavs:
                    if uav.attack_target_position is not None and uav.status == UAVStatus.ACTIVE:
                        distance = np.linalg.norm(uav.position[:2] - uav.attack_target_position[:2])
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


# ==================== 使用示例 ====================
if __name__ == "__main__":
    # 为独立运行此文件而创建的 Mock Planner
    class SimpleDubinsPlanner:
        turning_radius = 50.0
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
            # 阶段1: 准备阶段 - 仅设置领航者和随从角色
            mission_manager.execute_preparation_phase()

            # 阶段2: 侦察阶段 - 规划并合并准备+侦察路径
            mission_manager.execute_reconnaissance_phase()
            
            # mission_manager.animate_reconnaissance_phase()  # 可视化侦察阶段
            mission_manager.animate_complete_mission(max_steps=10000, dt=1.5, interval=50)

        else:
            # 【联调模式】: 初始化TCP并执行后台仿真
            print("--- 运行模式: TCP集成 ---")
            mission_manager._initialize_tcp_client()
            
            # 1. 规划所有路径
            mission_manager.execute_preparation_phase()
            mission_manager.execute_reconnaissance_phase()
            # mission_manager.execute_attack_phase()

            # 2. 运行统一的TCP仿真循环
            mission_manager.run_tcp_simulation(frequency=2.0)

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



  