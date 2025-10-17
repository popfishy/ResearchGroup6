#!/usr/bin/env python3

import numpy as np
import rospy
from typing import List, Tuple, Optional, Any
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from core.utils import *
from core.uav import UAV
from core.path_planner import PathPlanner
from core.scenario_parser import ScenarioParser

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
            rospy.loginfo("退出程序中...")
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
        # self.takeoff_zones = {
        #     1: {"center": (0, 0, 0), "uav_ids": list(range(1, 26))},      # 区域1: UAV 1-25
        #     2: {"center": (2000, 0, 0), "uav_ids": list(range(26, 51))}   # 区域2: UAV 26-50
        # }

        self.takeoff_zones = {}
        
        # 队形配置
        self.formation_configs = {}
        self._initialize_formations()
        
        # TODO：任务区域配置 - 需要您补充具体坐标
        self.reconnaissance_areas = []  # 侦查区域列表
        self.attack_targets = []        # 打击目标列表
        self.containment_zones = []     # 封控区域列表
        
        # 任务状态跟踪
        self.phase_completion = {
            MissionType.PREPARATION: False,
            MissionType.RECONNAISSANCE: False,
            MissionType.ATTACK: False,
            MissionType.CONTAINMENT: False
        }
        
        # 外部接口
        self.path_planner: Optional[PathPlanner] = None
        self.simulation_interface = None  # 仿真接口
        
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
        """初始化任务区域 - 需要您补充具体区域信息"""
        # TODO: 侦查区域示例,修改坐标
        self.reconnaissance_areas = [
            TargetArea(
                id=1,
                center=(1000, 1000, 100),  # 需要您提供具体坐标
                radius=500,
                priority=8,
                area_type=ZoneType.SEARCH_AREA,
                assigned_uavs=list(range(1, 26))  # 区域1的无人机
            ),
            TargetArea(
                id=2,
                center=(3000, 1000, 100),  # 需要您提供具体坐标
                radius=500,
                priority=8,
                area_type=ZoneType.SEARCH_AREA,
                assigned_uavs=list(range(26, 51))  # 区域2的无人机
            )
        ]
        
        # TODO：封控区域示例，修改坐标
        self.containment_zones = [
            ContainmentZone(
                id=1,
                center=(4000, 4000, 100),  # 需要您提供具体坐标
                radius=800,
                patrol_altitude=150,
                patrol_speed=25,
                assigned_uavs=[]  # 将在封控阶段动态分配
            )
        ]
        
        # TODO: 请补充具体的侦查区域、打击目标、封控区域坐标
        print("警告: 需要补充具体的任务区域坐标信息")
    
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
        print(f"--- 从 {xml_path} 加载场景 ---")
        try:
            parser = ScenarioParser(xml_path)
            scenario_data = parser.parse()
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

    
    # ==================== 阶段1: 准备阶段 ====================
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
        """计算侦查区域的集结点"""
        # 在侦查区域边缘设置集结点
        center = target_area.center
        radius = target_area.radius
        
        # 简单示例：在区域南侧设置集结点
        rally_point = (center[0], center[1] - radius - 200, center[2])
        return rally_point
    
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
        
        print(f"\n--- 为编队 {group_id} 设置移动任务 ---")
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
           
    
    # ==================== 阶段2: 侦查阶段 ====================
    def execute_reconnaissance_phase(self):
        """执行侦查阶段：区域覆盖搜索"""
        print("开始侦查阶段：区域覆盖搜索...")
        self.current_phase = MissionType.RECONNAISSANCE
        self.phase_start_time = time.time()
        
        for area in self.reconnaissance_areas:
            assigned_uavs = area.assigned_uavs
            
            # 使用region2cover库规划覆盖路径
            coverage_paths = self._plan_area_coverage(area, assigned_uavs)
            
            # 分配路径给无人机
            self._assign_coverage_paths(assigned_uavs, coverage_paths)
        
        # 执行侦查并检测目标
        self._execute_reconnaissance_search()
        
        print("侦查阶段完成")
        self.phase_completion[MissionType.RECONNAISSANCE] = True
    
    def _plan_area_coverage(self, area: TargetArea, uav_ids: List[int]) -> List[List[PathPoint]]:
        """规划区域覆盖路径"""
        if not self.coverage_planner:
            print("警告: 区域覆盖规划器未设置，使用简单网格模式")
            return self._simple_grid_coverage(area, uav_ids)
        
        try:
            # TODO: 调用region2cover库
            # coverage_result = self.coverage_planner.plan_coverage(
            #     area.center, area.radius, len(uav_ids)
            # )
            # return coverage_result.paths
            
            print(f"为区域 {area.id} 规划覆盖路径，分配给 {len(uav_ids)} 架无人机")
            return self._simple_grid_coverage(area, uav_ids)
            
        except Exception as e:
            print(f"覆盖路径规划失败: {e}")
            return self._simple_grid_coverage(area, uav_ids)
    
    def _simple_grid_coverage(self, area: TargetArea, uav_ids: List[int]) -> List[List[PathPoint]]:
        """简单网格覆盖模式（备用方案）"""
        paths = []
        center = area.center
        radius = area.radius
        num_uavs = len(uav_ids)
        
        # 简单的平行线扫描模式
        strip_width = 2 * radius / num_uavs
        
        for i in range(num_uavs):
            path = []
            y_offset = -radius + i * strip_width
            
            # 来回扫描
            start_x = center[0] - radius
            end_x = center[0] + radius
            
            if i % 2 == 0:  # 偶数行从左到右
                path.append(PathPoint(start_x, center[1] + y_offset, center[2], 0))
                path.append(PathPoint(end_x, center[1] + y_offset, center[2], math.pi))
            else:  # 奇数行从右到左
                path.append(PathPoint(end_x, center[1] + y_offset, center[2], math.pi))
                path.append(PathPoint(start_x, center[1] + y_offset, center[2], 0))
            
            paths.append(path)
        
        return paths
    
    def _assign_coverage_paths(self, uav_ids: List[int], paths: List[List[PathPoint]]):
        """分配覆盖路径给无人机"""
        for i, uav_id in enumerate(uav_ids):
            if i < len(paths):
                # TODO: 将路径分配给对应的无人机
                # self.uav_dict[uav_id].set_waypoints(paths[i])
                print(f"为 UAV-{uav_id} 分配覆盖路径，包含 {len(paths[i])} 个路径点")
    
    def _execute_reconnaissance_search(self):
        """执行侦查搜索并检测目标"""
        print("执行侦查搜索...")
        
        # 模拟目标检测过程
        detected_targets = []
        
        # TODO: 实际的目标检测逻辑
        # 这里需要根据无人机传感器数据进行目标识别
        
        # 示例：模拟发现一些目标
        sample_targets = [
            EnemyTarget(1, (1200, 1200, 0), "vehicle", 3, TargetStatus.DETECTED),
            EnemyTarget(2, (2800, 1100, 0), "building", 4, TargetStatus.DETECTED),
        ]
        
        for target in sample_targets:
            target.detection_time = time.time()
            detected_targets.append(target)
            print(f"检测到目标 {target.id}: {target.target_type} at {target.position}")
        
        self.attack_targets.extend(detected_targets)
    
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

    def _wait_for_phase_completion(self, phase_name: str, max_steps=5000, dt=0.1):
        """等待阶段完成"""
        print(f"开始运行仿真，等待 {phase_name} 阶段完成...")
        
        start_time = time.time()
        for step in range(max_steps):
            # 1. 【新增】为所有跟随者计算并更新其队形目标点
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
                    
            # 3. 检查所有领航者的路径是否完成 (这部分逻辑不变)
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
        
        duration = time.time() - start_time
        print(f"警告: {phase_name} 阶段在达到最大步数 {max_steps} 未完成。耗时: {duration:.2f}s")

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

    def animate_preparation_phase(self, max_steps=5000, dt=0.1, interval=20):
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
                    offset_x, offset_y = uav.formation_offset[0], uav.formation_offset[1]
                    
                    rotated_offset_x = offset_x * np.cos(leader_heading) - offset_y * np.sin(leader_heading)
                    rotated_offset_y = offset_x * np.sin(leader_heading) + offset_y * np.cos(leader_heading)
                    
                    target_pos = (leader_pos[0] + rotated_offset_x, leader_pos[1] + rotated_offset_y, leader_pos[2])
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
    
    # --- 2. 设置外部接口 (使用 Mock Planner) ---
    path_planner = PathPlanner(dubins_planner=SimpleDubinsPlanner())
    # mission_manager.set_external_interfaces(path_planner) # 修正：函数名已改变
    mission_manager.set_path_planner(path_planner)
    
    print("=== 开始无人机集群任务仿真（从XML加载） ===")
    
    # --- 3. 从XML文件初始化场景 ---
    # 这个函数现在是启动的第一步
    xml_file_path = '山地丛林.xml' # 确保XML文件与脚本在同一目录或提供正确路径
    if not mission_manager.initialize_uavs_from_xml(xml_file_path):
        print("场景初始化失败，程序退出。")
        exit()
        
    # 激活所有已加载的无人机
    for uav in mission_manager.uav_dict.values():
        uav.activate()

    # --- 4. 执行准备阶段 ---
    # 此函数内部包含路径规划、角色设置
    # mission_manager.execute_preparation_phase()
    print("开始准备阶段：规划无人机集结路径...")
    mission_manager.current_phase = MissionType.PREPARATION
    mission_manager.phase_start_time = time.time()
    
    # 动态地为每个从XML加载的编队规划集结路径
    assigned_groups = set()
    for area in mission_manager.reconnaissance_areas:
        if not area.assigned_uavs: continue
        
        # 找到这组UAV属于哪个编队
        group_id_found = None
        for gid, uids in mission_manager.group_assignments.items():
            if area.assigned_uavs[0] in uids: # 检查第一个UAV的归属
                group_id_found = gid
                break
        
        if group_id_found and group_id_found not in assigned_groups:
            rally_point = mission_manager._calculate_rally_point(area)
            # 调用重写后的函数，注意参数变化
            mission_manager._plan_group_formation_movement(group_id_found, rally_point, "square_5x5")
            assigned_groups.add(group_id_found)

    # --- 5. 动态可视化准备阶段 ---
    mission_manager.animate_preparation_phase()

    print("仿真流程结束。")

    # --- 6. 监控 ---
    status = mission_manager.get_mission_status()
    print(f"\n最终任务状态: {status}")
    # print("生成轨迹图...")
    # mission_manager.plot_results()



  