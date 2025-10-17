import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from typing import List, Dict, Tuple
from .utils import *

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
                         leader_uav, # 直接传入UAV对象
                         leader_waypoints: List[Tuple[float, float, float]]):
        """
        为给定的领航者无人机规划路径。
        它直接调用领航者自己的方法来完成路径规划。
        """
        if not leader_uav:
            raise ValueError("Leader UAV object cannot be None")
            
        if not self.dubins_planner:
            print("Error: Dubins planner is not set in PathPlanner.")
            return
        # 将自身的dubins_planner实例赋给UAV，让UAV内部可以使用
        leader_uav.set_path_planner(self)
        
        # 直接调用UAV自己的路径规划方法
        leader_uav.set_waypoints(leader_waypoints)
    
        print(f"Path planning triggered for leader UAV-{leader_uav.id}")