import numpy as np
from typing import List, Dict, Tuple, Optional, Any
from .utils import *

class PathPlanner:
    def __init__(self, dubins_planner=None):
        self.dubins_planner = dubins_planner
        self.default_turn_radius = 100.0
        self.default_speed = 30.0
        self.safety_margin = 50.0  # 安全间距
        
        # 外部库接口
        self.coverage_planner = None  # region2cover库接口
        self.patrol_planner = None    # 巡逻路径规划库接口
    
    def set_coverage_planner(self, coverage_planner):
        """设置区域覆盖路径规划器（region2cover库）"""
        self.coverage_planner = coverage_planner
    
    def set_patrol_planner(self, patrol_planner):
        """设置巡逻路径规划器"""
        self.patrol_planner = patrol_planner
    
    def plan_formation_path(self, 
                           leader_waypoints: List[Tuple[float, float, float]],
                           formation_config: FormationConfig,
                           uav_list: List,
                           leader_id: int,
                           initial_positions: Optional[Dict[int, Tuple[float, float, float]]] = None) -> FormationPathResult:
        """
        Leader-Follow队形路径规划
        
        Args:
            leader_waypoints: 领机航路点
            formation_config: 队形配置
            uav_list: 无人机列表
            leader_id: 领机ID
            initial_positions: 初始位置字典 {uav_id: (x,y,z)}
            
        Returns:
            FormationPathResult: 包含领机和跟随机路径
        """
        
        # 1. 获取领机和跟随机
        leader_uav = None
        follower_uavs = []
        
        for uav in uav_list:
            if uav.id == leader_id:
                leader_uav = uav
            else:
                follower_uavs.append(uav)
        
        if not leader_uav:
            raise ValueError(f"Leader UAV with ID {leader_id} not found")
        
        # 2. 为领机规划Dubins路径
        leader_path = self._plan_leader_path(leader_uav, leader_waypoints)
        
        # 3. 为跟随机规划路径
        follower_paths = self._plan_follower_paths(
            leader_path, 
            follower_uavs, 
            formation_config,
            initial_positions
        )
        
        # 4. 计算总距离和预估时间
        total_distance = self._calculate_path_distance(leader_path)
        estimated_time = total_distance / self.default_speed
        
        return FormationPathResult(
            leader_path=leader_path)
