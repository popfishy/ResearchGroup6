import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from typing import List, Dict, Tuple
from .utils import *
from .uav import UAV

class PathPlanner:
    def __init__(self, dubins_planner=None, coverage_planner=None, patrol_planner=None):
        self.dubins_planner = dubins_planner
        
        # 外部库接口
        self.coverage_planner = coverage_planner
        self.patrol_planner = patrol_planner
    
    def set_coverage_planner(self, coverage_planner):
        self.coverage_planner = coverage_planner
    
    def set_patrol_planner(self, patrol_planner):
        self.patrol_planner = patrol_planner
    
    def plan_leader_path(self, 
                         start_pos: Tuple[float, float, float],
                         waypoints: List[Tuple[float, float, float]]) -> List[np.ndarray]:
        """
        为领航者规划一条从起点依次经过所有航路点的Dubins路径。
        【修改】: 此函数现在返回路径，而不是直接设置给无人机。
        """
        if not self.dubins_planner:
            print("Error: Dubins planner is not set in PathPlanner.")
            return []
        
        full_path = []
        current_pos = start_pos

        for target_pos in waypoints:
            path_segment, _ = self.dubins_planner.plan(
                q0=(current_pos[0], current_pos[1], current_pos[2]),
                q1=(target_pos[0], target_pos[1], 0), # 假设目标航向为0，可优化
                turning_radius=self.dubins_planner.turning_radius,
                step_size=20.0
            )
            
            if path_segment:
                # 【修正】确保路径点是Numpy数组，以支持向量操作
                full_path.extend([np.array(p) for p in path_segment])
                # 更新当前位置为段的终点，用于下一段的起点
                current_pos = path_segment[-1]
        
        return full_path
    