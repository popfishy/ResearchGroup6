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

matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'WenQuanYi Zen Hei', 'DejaVu Sans', 'Arial Unicode MS']
matplotlib.rcParams['axes.unicode_minus'] = False

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
                center=(4000, 4000, 100),
                width=1600,
                height=1600,
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
        
        # 【修改】动态选择最靠近编队中心的无人机作为领航者
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
            
        print(f"{follower_count} 架无人机被设置为跟随者，将保持其初始相对位置。")


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
    
    def run_tcp_simulation(self, frequency=2.0):
        """
        【新增】在非调试模式下运行主仿真循环。
        - 以指定频率持续更新所有无人机位置。
        - 通过TCP发布数据。
        - 直到所有领航者完成其完整路径后结束。
        """
        print(f"--- 开始TCP仿真循环 (频率: {frequency}Hz) ---")
        dt = 1.0 / frequency
        
        try:
            while not rospy.is_shutdown():
                loop_start_time = time.time()

                # 1. 检查所有领航者是否都已完成路径
                leaders = [uav for uav in self.uav_dict.values() if uav.is_leader and uav.path_planning_complete]
                if not leaders:
                    print("警告: 未找到活动的领航者，仿真提前结束。")
                    break
                if all(l.is_path_complete for l in leaders):
                    print("所有领航者均已完成路径，仿真结束。")
                    break

                # 2. 更新所有无人机的位置（与动画逻辑完全相同）
                #   a. 更新跟随者的目标点
                for uav in self.uav_dict.values():
                    if not uav.is_leader and uav.leader is not None:
                        leader_pos, leader_heading = uav.leader.position, uav.leader.heading
                        offset_x, offset_y = uav.formation_offset
                        
                        rotated_offset_x = offset_x * np.cos(leader_heading) - offset_y * np.sin(leader_heading)
                        rotated_offset_y = offset_x * np.sin(leader_heading) + offset_y * np.cos(leader_heading)
                        
                        target_pos = (leader_pos[0] + rotated_offset_x, leader_pos[1] + rotated_offset_y, leader_pos[2])
                        uav.set_formation_target(target_pos, leader_heading)
                
                #   b. 更新所有无人机的位置
                for uav in self.uav_dict.values():
                    if uav.status == UAVStatus.ACTIVE:
                        uav.update_position(dt)

                # 3. 通过TCP发布数据
                self._publish_uav_data_to_tcp()

                # 4. 控制循环频率
                elapsed = time.time() - loop_start_time
                sleep_duration = dt - elapsed
                if sleep_duration > 0:
                    time.sleep(sleep_duration)

        except KeyboardInterrupt:
            print("\nTCP仿真循环被用户中断。")
        finally:
            print("--- TCP仿真循环结束 ---")

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
            # 【最终修正】合并准备阶段路径和侦察阶段路径
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
        """执行打击阶段：使用xfd_allocation库分配并攻击检测到的目标"""
        print("\n--- 开始打击阶段：解散编队并分配攻击任务 ---")
        self.current_phase = MissionType.ATTACK
        self.phase_start_time = time.time()
        
        if not self.attack_targets:
            print("未发现敌方目标，跳过打击阶段。")
            self.phase_completion[MissionType.ATTACK] = True
            return

        # 1. 解散所有编队，使无人机独立行动
        for uav in self.uav_dict.values():
            uav.unset_formation_roles()
            
        # 2. 准备任务分配所需的数据
        available_uav_ids = self._get_available_uavs_for_attack()
        available_uavs_as_robots = [self._convert_uav_to_robot(self.uav_dict[uid]) for uid in available_uav_ids]
        
        active_targets_as_tasks = [self._convert_enemy_to_task(tgt) for tgt in self.attack_targets if tgt.status != TargetStatus.DESTROYED]
        
        if not available_uavs_as_robots or not active_targets_as_tasks:
            print("没有可用的无人机或活动目标，无法进行攻击。")
            self.phase_completion[MissionType.ATTACK] = True
            return
            
        # 定义仿真世界边界 (用于任务分配计算)
        world_info = WorldInfo(limit_x=[-2000, 8000], limit_y=[-8000, 2000], limit_z=[0, 500])
        
        # 3. 调用CBPA求解器进行任务分配
        print(f"为 {len(available_uavs_as_robots)} 架无人机和 {len(active_targets_as_tasks)} 个目标进行任务分配...")
        try:
            assignment_results, _, _, _, _ = self.cbpa_solver.solve_centralized(
                available_uavs_as_robots, active_targets_as_tasks, world_info
            )
            
        except Exception as e:
            print(f"任务分配时发生错误: {e}")
            self.phase_completion[MissionType.ATTACK] = True
            return
            
        # 4. 解析分配结果并执行攻击
        # assignment_results 的格式: [[target_id, uav_id1, uav_id2, ...], ...]
        attack_assignments = {}
        for assignment in assignment_results:
            if len(assignment) >= 2:
                target_id = assignment[0]
                for uav_id in assignment[1:]:
                    attack_assignments[uav_id] = target_id
        
        if attack_assignments:
            self._execute_attack_missions(attack_assignments)
        else:
            print("任务分配未产生任何攻击指令。")

        # 注意：这里的完成是指令下达完成，实际攻击完成需要仿真循环继续
        print("打击阶段指令已下达。")
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

    def _simulate_reconnaissance_silently(self, dt=0.5, max_steps=10000):
        """静默模拟侦查阶段，不显示动画，仅更新UAV位置到侦查结束位置。"""
        
        # 【调试】检查编队信息
        leader_count = sum(1 for uav in self.uav_dict.values() if uav.is_leader)
        follower_count = sum(1 for uav in self.uav_dict.values() if not uav.is_leader and uav.leader is not None)
        print(f"开始静默模拟 - 编队统计: {leader_count} 个领航者, {follower_count} 个跟随者")
        
        if follower_count == 0 and leader_count < len(self.uav_dict):
            print(f"警告: 检测到有 {len(self.uav_dict) - leader_count} 架UAV没有编队角色！")
            print("这可能是因为编队信息被意外清除。")
        
        for step in range(max_steps):
            # 检查是否所有领航者都完成了路径
            leaders = [uav for uav in self.uav_dict.values() if uav.is_leader and uav.path_planning_complete]
            if not leaders:
                break
            if all(l.is_path_complete for l in leaders):
                break
            
            # 更新跟随者的目标点
            for uav in self.uav_dict.values():
                if not uav.is_leader and uav.leader is not None:
                    leader_pos, leader_heading = uav.leader.position, uav.leader.heading
                    offset_x, offset_y = uav.formation_offset
                    
                    rotated_offset_x = offset_x * np.cos(leader_heading) - offset_y * np.sin(leader_heading)
                    rotated_offset_y = offset_x * np.sin(leader_heading) + offset_y * np.cos(leader_heading)
                    
                    target_pos = (leader_pos[0] + rotated_offset_x, leader_pos[1] + rotated_offset_y, leader_pos[2])
                    uav.set_formation_target(target_pos, leader_heading)
            
            # 更新所有无人机位置
            for uav in self.uav_dict.values():
                if uav.status == UAVStatus.ACTIVE:
                    uav.update_position(dt)
        
        print(f"侦查阶段模拟完成，共执行 {step} 步。")
        
        # 【调试】打印最终位置信息
        sample_size = min(10, len(self.uav_dict))
        print(f"前 {sample_size} 架UAV的最终位置:")
        for uav_id, uav in sorted(self.uav_dict.items())[:sample_size]:
            role = "Leader" if uav.is_leader else ("Follower" if uav.leader is not None else "None")
            print(f"  UAV-{uav_id}({role}): ({uav.position[0]:.1f}, {uav.position[1]:.1f})")

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

    def animate_attack_phase(self, max_steps=3000, dt=0.5, interval=20, simulate_recon_first=True):
        """通过动态动画来可视化和执行打击阶段。
        
        显示内容：
        1. 侦查区域
        2. UAV在侦查阶段结束后的位置（打击阶段起点）
        3. 敌方目标
        4. 执行打击任务的无人机前往目标的动态路径
        
        参数：
        - simulate_recon_first: 是否先模拟执行侦查阶段（不显示动画）以获得侦查结束后的位置
        """
        print("开始打击阶段的动态仿真与可视化...")
        
        # 0. ======== 预处理：模拟侦查阶段飞行 ========
        if simulate_recon_first:
            print("正在模拟侦查阶段飞行以获取UAV结束位置...")
            self._simulate_reconnaissance_silently(dt=dt)
            print(f"侦查阶段模拟完成。UAV已到达侦查结束位置。")
            
            # 重新执行攻击阶段任务分配，因为现在UAV位置已经更新到侦查结束位置
            print("\n基于侦查结束位置，重新进行攻击任务分配和路径规划...")
            self.execute_attack_phase()
        
        # 0.1 ======== 清空历史轨迹，为打击阶段重新记录 ========
        print("清空历史轨迹，准备记录打击阶段路径...")
        for uav in self.uav_dict.values():
            uav.position_history = [uav.position.copy()]  # 只保留当前位置作为起点
        
        # 1. ======== 动画设置 ========
        fig, ax = plt.subplots(figsize=(16, 12))
        ax.set_aspect('equal', adjustable='box')
        ax.set_title('Swarm Mission: Attack Phase', fontsize=16, fontweight='bold')
        ax.set_xlabel('East (m)', fontsize=12)
        ax.set_ylabel('North (m)', fontsize=12)
        ax.grid(True, alpha=0.3)
        
        # 2. ======== 绘制侦察区域边界 ========
        from matplotlib.patches import Rectangle
        for i, area in enumerate(self.reconnaissance_areas):
            bottom_left_x = area.center[0] - area.width / 2
            bottom_left_y = area.center[1] - area.height / 2
            rect = Rectangle(xy=(bottom_left_x, bottom_left_y), 
                             width=area.width, height=area.height,
                             color='blue', fill=False, linestyle='--', linewidth=2,
                             label='Reconnaissance Area' if i == 0 else None)
            ax.add_patch(rect)
            # 添加区域标签
            ax.text(area.center[0], area.center[1], f'Area {area.id}', 
                    fontsize=10, ha='center', va='center', 
                    bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
        
        # 3. ======== 绘制敌方目标 ========
        if self.attack_targets:
            enemy_positions = []
            for tgt in self.attack_targets:
                enemy_positions.append(tgt.position[:2])
                # 绘制目标位置
                ax.scatter(tgt.position[0], tgt.position[1], 
                           c='red', marker='X', s=300, linewidths=2,
                           edgecolors='darkred', zorder=10)
                # 添加目标标签
                ax.text(tgt.position[0] + 100, tgt.position[1] + 100, 
                        f'Target-{tgt.id}\n({tgt.target_type})', 
                        fontsize=9, color='darkred', fontweight='bold',
                        bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
            
            # 添加图例项
            ax.scatter([], [], c='red', marker='X', s=300, linewidths=2,
                       edgecolors='darkred', label='Enemy Targets')
        
        # 4. ======== 绘制UAV的侦查结束位置（打击阶段起点） ========
        uav_start_positions = {}
        active_uav_count = 0
        for uav_id, uav in self.uav_dict.items():
            if uav.status == UAVStatus.ACTIVE:
                uav_start_positions[uav_id] = uav.position.copy()
                ax.scatter(uav.position[0], uav.position[1], 
                           c='green', marker='o', s=80, alpha=0.6,
                           edgecolors='darkgreen', linewidths=1.5, zorder=5)
                active_uav_count += 1
        
        print(f"绘制了 {active_uav_count} 架活动UAV的起始位置。")
        
        # 添加起点图例
        ax.scatter([], [], c='green', marker='o', s=80, alpha=0.6,
                   edgecolors='darkgreen', linewidths=1.5,
                   label='UAV Start (After Recon)')
        
        # 5. ======== 初始化UAV动态绘图元素 ========
        uav_plots = {}
        attack_uav_count = 0
        for uav_id, uav in self.uav_dict.items():
            if uav.status == UAVStatus.ACTIVE:
                # 创建轨迹线和当前位置点
                line, = ax.plot([], [], color='orange', linestyle='-', 
                                linewidth=2, alpha=0.7, zorder=3)
                point, = ax.plot(uav.position[0], uav.position[1], 'o', 
                                 color='blue', markersize=8, 
                                 markeredgecolor='navy', markeredgewidth=1.5, zorder=8)
                uav_plots[uav_id] = {'line': line, 'point': point}
                
                # 统计被分配攻击任务的UAV
                if uav.current_mission == MissionType.ATTACK and uav.waypoints:
                    attack_uav_count += 1
        
        print(f"共 {len(uav_plots)} 架UAV将参与动画，其中 {attack_uav_count} 架被分配了攻击任务。")
        
        # 添加当前位置图例
        ax.plot([], [], 'o', color='blue', markersize=8, 
                markeredgecolor='navy', markeredgewidth=1.5,
                label='UAV Current Position')
        ax.plot([], [], color='orange', linestyle='-', linewidth=2, alpha=0.7,
                label='Attack Trajectory')
        
        # 6. ======== 添加时间文本和状态信息 ========
        time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, 
                            fontsize=12, verticalalignment='top',
                            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        status_text = ax.text(0.02, 0.92, '', transform=ax.transAxes,
                              fontsize=10, verticalalignment='top',
                              bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
        
        ax.legend(loc='upper right', fontsize=10)
        ax.autoscale_view()
        
        # 7. ======== 定义动画更新函数 ========
        def update(frame):
            # 检查所有执行攻击任务的无人机是否完成
            attacking_uavs = [uav for uav in self.uav_dict.values() 
                              if uav.current_mission == MissionType.ATTACK 
                              and uav.status == UAVStatus.ACTIVE]
            
            if attacking_uavs:
                completed_count = sum(1 for uav in attacking_uavs if uav.is_path_complete)
            else:
                completed_count = 0
            
            # 更新所有无人机位置
            for uav in self.uav_dict.values():
                if uav.status == UAVStatus.ACTIVE:
                    uav.update_position(dt)
            
            # 更新绘图元素
            artists = []
            for uav_id, uav in self.uav_dict.items():
                if uav_id in uav_plots:
                    plots = uav_plots[uav_id]
                    history = np.array(uav.position_history)
                    if len(history) > 0:
                        plots['line'].set_data(history[:, 0], history[:, 1])
                        plots['point'].set_data([uav.position[0]], [uav.position[1]])
                        artists.extend([plots['line'], plots['point']])
            
            # 更新时间和状态信息
            time_text.set_text(f'Simulation Time: {frame * dt:.1f}s')
            if attacking_uavs:
                status_text.set_text(f'Attacking UAVs: {len(attacking_uavs)}\n'
                                     f'Completed: {completed_count}/{len(attacking_uavs)}')
            else:
                status_text.set_text('No attacking UAVs found')
            artists.extend([time_text, status_text])
            
            # 定期调整视图
            if frame % 50 == 0:
                ax.relim()
                ax.autoscale_view()
            
            return artists
        
        # 8. ======== 创建并启动动画 ========
        ani = animation.FuncAnimation(fig, update, frames=max_steps, 
                                       interval=interval, blit=True, repeat=False)
        plt.show()
        print("打击阶段动画播放完毕。")
        self.phase_completion[MissionType.ATTACK] = True

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
            # 【调试模式】: 规划路径并启动可视化动画
            
            # 阶段1: 准备阶段 - 仅设置领航者和随从角色
            mission_manager.execute_preparation_phase()
            
            # 阶段2: 侦察阶段 - 规划并合并准备+侦察路径
            mission_manager.execute_reconnaissance_phase()
            
            # 注意：打击阶段的任务分配会在 animate_attack_phase 中自动执行
            # 因为需要先模拟完侦查阶段，让UAV到达正确位置后再分配攻击任务
            
            # mission_manager.plot_results()
            
            # 可视化选项（取消注释以运行对应阶段的动画）：
            # mission_manager.animate_reconnaissance_phase()  # 可视化侦察阶段
            mission_manager.animate_attack_phase()  # 可视化打击阶段（会自动模拟侦查并分配攻击任务）

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



  