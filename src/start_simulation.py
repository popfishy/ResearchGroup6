import numpy as np
import rospy
from typing import List, Tuple, Optional, Any
import math

from core.utils import *
from core.uav import UAV
from core.path_planner import PathPlanner


# pos1-------------pos4
#  |                |
# pos2-------------pos3
# RegionList: list = [
#     Region(1, [8081, 0, 3752], [8081, 0, 2802], [9519, 0, 2802], [9519, 0, 3752]),
#     Region(2, [6007, 0, 2908], [6007, 0, 1840], [7141, 0, 1840], [7141, 0, 2908]),
#     Region(3, [7352, 0, 2745], [7352, 0, 1579], [8882, 0, 1579], [8882, 0, 2745]),
#     Region(4, [7352, 0, 1532], [7352, 0, 120], [9145, 0, 120], [9145, 0, 1532]),
# ]


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
        self.uav_dict = {}  # ID -> UAV对象
        self.group_assignments = {}  # group_id -> [uav_ids]
        
        # 起飞区域配置
        self.takeoff_zones = {
            1: {"center": (0, 0, 0), "uav_ids": list(range(1, 26))},      # 区域1: UAV 1-25
            2: {"center": (2000, 0, 0), "uav_ids": list(range(26, 51))}   # 区域2: UAV 26-50
        }
        
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
        self.dubins_planner = None
        self.coverage_planner = None  # region2cover库
        self.patrol_planner = None
        self.simulation_interface = None  # 仿真接口
        
        # 初始化任务
        self._initialize_mission_areas()
    
    def _initialize_formations(self):
        """初始化队形配置"""
        # 5x5正方形队形
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
                center=(2000, 2000, 150),  # 需要您提供具体坐标
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
        """设置外部接口"""
        self.dubins_planner = dubins_planner
        self.coverage_planner = coverage_planner
        self.patrol_planner = patrol_planner
        self.simulation_interface = simulation_interface
    
    def initialize_uavs_from_simulation(self):
        """从仿真中读取无人机信息并初始化"""
        if not self.simulation_interface:
            print("警告: 仿真接口未设置，使用默认初始化")
            self._initialize_uavs_default()
            return
        
        try:
            # TODO: 从仿真接口读取无人机信息
            uav_data = self.simulation_interface.get_all_uav_status()
            
            for uav_info in uav_data:
                uav_id = uav_info['id']
                position = uav_info['position']
                # 创建UAV对象并添加到字典
                # self.uav_dict[uav_id] = UAV(uav_id, position)
                
        except Exception as e:
            print(f"从仿真读取无人机信息失败: {e}")
            self._initialize_uavs_default()
    
    def _initialize_uavs_default(self):
        """默认无人机初始化（用于测试）"""
        for zone_id, zone_info in self.takeoff_zones.items():
            center = zone_info["center"]
            uav_ids = zone_info["uav_ids"]
            
            # 在起飞区域周围分布无人机
            formation = self.formation_configs["square_5x5"]
            for i, uav_id in enumerate(uav_ids):
                if i < len(formation.positions):
                    offset = formation.positions[i]
                    position = (
                        center[0] + offset[0],
                        center[1] + offset[1],
                        center[2]
                    )
                else:
                    # 超出队形的无人机随机分布
                    position = (
                        center[0] + np.random.uniform(-200, 200),
                        center[1] + np.random.uniform(-200, 200),
                        center[2]
                    )
                
                # TODO: 创建UAV对象
                self.uav_dict[uav_id] = UAV(uav_id, position)
                print(f"初始化 UAV-{uav_id} 在位置 {position}")
    
    # ==================== 阶段1: 准备阶段 ====================
    def execute_preparation_phase(self):
        """执行准备阶段：集结到侦查区域起点"""
        print("开始准备阶段：无人机集结...")
        self.current_phase = MissionType.PREPARATION
        self.phase_start_time = time.time()
        
        # 为每个起飞区域的无人机规划到侦查区域的集结路径
        for zone_id, zone_info in self.takeoff_zones.items():
            uav_ids = zone_info["uav_ids"]
            target_area = self.reconnaissance_areas[zone_id - 1]  # 对应的侦查区域
            
            # 计算集结点（侦查区域边缘）
            rally_point = self._calculate_rally_point(target_area)
            
            # 为该组无人机规划队形移动路径
            self._plan_group_formation_movement(uav_ids, rally_point, "square_5x5")
        
        # 等待所有无人机到达集结点
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
    
    def _plan_group_formation_movement(self, uav_ids: List[int], target_point: Tuple[float, float, float], formation_name: str):
        """为一组无人机规划队形移动"""
        if not self.dubins_planner:
            print("警告: Dubins规划器未设置")
            return
        
        formation = self.formation_configs[formation_name]
        leader_id = uav_ids[formation.leader_index]
        
        # 领机航路点
        leader_waypoints = [target_point]
        
        # TODO: 调用PathPlanner.plan_formation_path()
        # formation_result = self.path_planner.plan_formation_path(
        #     leader_waypoints, formation, [self.uav_dict[uid] for uid in uav_ids], leader_id
        # )
        
        print(f"为组 {uav_ids} 规划队形移动到 {target_point}")
    
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
    
    def _wait_for_phase_completion(self, phase_name: str):
        """等待阶段完成"""
        print(f"等待{phase_name}阶段完成...")
        
        # TODO: 实际实现中需要检查所有无人机的任务状态
        # 这里使用简单的时间延迟模拟
        time.sleep(1)
        
        print(f"{phase_name}阶段完成")
    
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
# ==================== 使用示例 ====================
if __name__ == "__main__":
    # 创建任务管理器
    mission_manager = SwarmMissionManager()
    
    # TODO: 设置外部接口
    # mission_manager.set_external_interfaces(
    #     dubins_planner=your_dubins_planner,
    #     coverage_planner=your_region2cover_planner,
    #     patrol_planner=your_patrol_planner,
    #     simulation_interface=your_simulation_interface
    # )
    
    # 启动任务
    mission_manager.start_mission()
    
    # 监控任务状态
    status = mission_manager.get_mission_status()
    print(f"任务状态: {status}")
  