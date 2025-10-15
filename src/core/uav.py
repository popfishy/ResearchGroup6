import numpy as np
from enum import Enum
from typing import List, Tuple, Optional
import time
from .utils import *


class UAV:
    def __init__(self, uav_id: int, initial_position: Tuple[float, float, float], initial_heading: float = 0.0):
        # 基本属性
        self.id = uav_id
        self.init_global_position = np.array((0, 0, 0), dtype=float)
        self.position = np.array(initial_position, dtype=float)
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.heading = initial_heading  # 初始航向角
        self.status = UAVStatus.STANDBY
        self.fuel = 100.0
        self.turn_radius = 100.0
        self.max_speed = 40.0
        self.min_speed = 15.0
        self.current_speed = 30.0
        
        # 任务相关
        self.current_mission = MissionType.IDLE
        self.group_id = None
        self.task_flag = None
        self.task_phase = None
        self.task_type = None
        
        # 路径规划相关
        self.waypoints = []  # 原始航路点
        self.planned_path = []  # Dubins规划后的详细路径点
        self.current_path_index = 0
        self.path_planner = None  # 外部路径规划器接口
        
        # 倍速控制相关
        self.speed_multiplier = 1.0
        self.base_update_interval = 0.1
        self.path_sampling_rate = 1
        self.last_update_time = time.time()
        
        # 仿真控制
        self.position_history = [self.position.copy()]
        self.is_path_complete = False
        self.path_planning_complete = False
    
    def set_path_planner(self, path_planner):
        """设置外部路径规划器"""
        self.path_planner = path_planner
    
    def set_waypoints(self, waypoints: List[Tuple[float, float, float]]):
        """设置航路点，触发Dubins路径规划"""
        self.waypoints = [np.array(wp, dtype=float) for wp in waypoints]
        self.current_path_index = 0
        self.is_path_complete = False
        self.path_planning_complete = False
        
        # 调用路径规划
        self._plan_dubins_path()
    
    def _plan_dubins_path(self):
        """使用Dubins模型规划路径"""
        if not self.waypoints or not self.path_planner:
            return
        
        try:
            # 构建起始状态 (x, y, heading)
            start_state = (self.position[0], self.position[1], self.heading)
            
            # 为每个航路点规划Dubins路径
            self.planned_path = []
            current_state = start_state
            
            for waypoint in self.waypoints:
                # 计算到达航路点的期望航向（可以根据需要调整）
                if len(self.waypoints) > 1:
                    wp_index = list(self.waypoints).index(waypoint)
                    if wp_index < len(self.waypoints) - 1:
                        next_wp = self.waypoints[wp_index + 1]
                        target_heading = np.arctan2(next_wp[1] - waypoint[1], 
                                                  next_wp[0] - waypoint[0])
                    else:
                        target_heading = current_state[2]  # 保持当前航向
                else:
                    target_heading = current_state[2]
                
                target_state = (waypoint[0], waypoint[1], target_heading)
                
                # 调用外部Dubins路径规划器
                segment_path = self.path_planner.plan_dubins_path(
                    start=current_state,
                    goal=target_state,
                    turning_radius=self.turn_radius
                )
                
                if segment_path:
                    # 添加路径段到总路径（避免重复起点）
                    if self.planned_path:
                        segment_path = segment_path[1:]  # 跳过起点
                    self.planned_path.extend(segment_path)
                    current_state = target_state
            
            self.path_planning_complete = True
            print(f"UAV-{self.id}: Dubins path planned with {len(self.planned_path)} points")
            
        except Exception as e:
            print(f"UAV-{self.id}: Path planning failed: {e}")
            # 降级到简单直线路径
            self._fallback_to_simple_path()
    
    def _fallback_to_simple_path(self):
        """降级到简单路径（直接连接航路点）"""
        self.planned_path = []
        current_pos = self.position[:2]
        
        for waypoint in self.waypoints:
            # 简单插值生成路径点
            target_pos = waypoint[:2]
            distance = np.linalg.norm(target_pos - current_pos)
            num_points = max(int(distance / 10), 1)  # 每10米一个点
            
            for i in range(1, num_points + 1):
                t = i / num_points
                interp_pos = current_pos + t * (target_pos - current_pos)
                # 计算航向
                direction = target_pos - current_pos
                heading = np.arctan2(direction[1], direction[0])
                self.planned_path.append((interp_pos[0], interp_pos[1], heading))
            
            current_pos = target_pos
        
        self.path_planning_complete = True
        print(f"UAV-{self.id}: Using fallback simple path with {len(self.planned_path)} points")
    
    def set_speed_multiplier(self, multiplier: float):
        """设置倍速系数"""
        self.speed_multiplier = max(0.1, multiplier)
        self.update_sampling_rate()
    
    def update_sampling_rate(self):
        """根据倍速系数更新路径采样率"""
        if self.speed_multiplier >= 2.0:
            self.path_sampling_rate = int(self.speed_multiplier)
        else:
            self.path_sampling_rate = 1
    
    def update_position(self, dt: float = None):
        """更新无人机位置 - 沿着规划好的Dubins路径"""
        if dt is None:
            current_time = time.time()
            dt = (current_time - self.last_update_time) * self.speed_multiplier
            self.last_update_time = current_time
        else:
            dt = dt * self.speed_multiplier
        
        if (self.status != UAVStatus.ACTIVE or 
            not self.path_planning_complete or 
            not self.planned_path):
            return
        
        # 检查是否完成路径
        if self.current_path_index >= len(self.planned_path):
            self.is_path_complete = True
            return
        
        # 根据采样率获取目标路径点
        target_index = min(
            self.current_path_index + self.path_sampling_rate - 1,
            len(self.planned_path) - 1
        )
        
        target_point = self.planned_path[target_index]
        target_pos = np.array([target_point[0], target_point[1], self.position[2]])
        target_heading = target_point[2]
        
        # 计算到目标点的距离
        distance_to_target = np.linalg.norm(target_pos[:2] - self.position[:2])
        
        # 如果接近目标点，移动到下一个路径点
        if distance_to_target < 2.0:  # 2米容差
            self.current_path_index += self.path_sampling_rate
            if self.current_path_index >= len(self.planned_path):
                self.is_path_complete = True
                return
        
        # 直接使用规划路径中的位置和航向（因为Dubins已经考虑了转弯约束）
        self.position = target_pos.copy()
        self.heading = target_heading
        
        # 更新速度向量
        self.velocity = np.array([
            self.current_speed * np.cos(self.heading),
            self.current_speed * np.sin(self.heading),
            0.0
        ])
        
        # 更新燃料消耗
        fuel_consumption_rate = 0.1
        self.fuel -= fuel_consumption_rate * dt * self.speed_multiplier
        self.fuel = max(0.0, self.fuel)
        
        # 记录位置历史
        self.position_history.append(self.position.copy())
        
        # 检查燃料状态
        if self.fuel <= 0:
            self.status = UAVStatus.DESTROYED
    
    def activate(self):
        """激活无人机"""
        if self.fuel > 0:
            self.status = UAVStatus.ACTIVE
    
    def destroy(self):
        """销毁无人机"""
        self.status = UAVStatus.DESTROYED
        self.velocity = np.array([0.0, 0.0, 0.0])
    
    def get_position_for_simulation(self) -> Tuple[float, float, float]:
        """获取位置信息用于发送给上层仿真软件"""
        return tuple(self.position)
    
    def get_status_info(self) -> dict:
        """获取状态信息"""
        return {
            'id': self.id,
            'position': self.position.tolist(),
            'velocity': self.velocity.tolist(),
            'heading': self.heading,
            'status': self.status.value,
            'fuel': self.fuel,
            'mission': self.current_mission.value,
            'group_id': self.group_id,
            'speed_multiplier': self.speed_multiplier,
            'path_progress': f"{self.current_path_index}/{len(self.planned_path)}",
            'is_path_complete': self.is_path_complete,
            'waypoints_count': len(self.waypoints),
            'planned_path_count': len(self.planned_path)
        }
    
    def __str__(self):
        if self.planned_path and self.current_path_index < len(self.planned_path):
            current_target = self.planned_path[self.current_path_index]
            return f"UAV-{self.id}: pos=({self.position[0]:.1f},{self.position[1]:.1f}), target=({current_target[0]:.1f},{current_target[1]:.1f}), progress={self.current_path_index}/{len(self.planned_path)}"
        else:
            return f"UAV-{self.id}: pos=({self.position[0]:.1f},{self.position[1]:.1f}), status={self.status.value}"
# 路径规划器接口示例（需要您提供具体的Dubins库实现）
class DubinsPathPlanner:
    """Dubins路径规划器接口"""
    
    def plan_dubins_path(self, start, goal, turning_radius):
        """
        规划Dubins路径
        
        Args:
            start: (x, y, heading) 起始状态
            goal: (x, y, heading) 目标状态  
            turning_radius: 转弯半径
            
        Returns:
            List of (x, y, heading) 路径点
        """
        # 这里需要调用您的专业Dubins库
        # 例如：dubins库、OMPL库等
        
        # 示例代码（需要替换为实际的库调用）
        try:
            import dubins  # 假设使用dubins库
            
            # 计算Dubins路径
            path = dubins.shortest_path(start, goal, turning_radius)
            
            # 采样路径点
            step_size = 5.0  # 每5米一个点
            configurations, _ = path.sample_many(step_size)
            
            return configurations
            
        except ImportError:
            print("Dubins library not available, using fallback")
            return None


import matplotlib.pyplot as plt

def plot_uav_path(uav: UAV):
    history = np.array(uav.position_history)
    plt.figure(figsize=(8, 8))
    plt.plot(history[:, 0], history[:, 1], 'b-', label='Trajectory')
    plt.scatter(history[0, 0], history[0, 1], c='green', s=100, label='Start')
    plt.scatter(history[-1, 0], history[-1, 1], c='red', s=100, label='End')
    
    # 绘制原始路径点
    if uav.waypoints:
        path = np.array(uav.waypoints)
        plt.plot(path[:, 0], path[:, 1], 'ro', label='Waypoints')
    
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title(f'UAV-{uav.id} Path')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# 使用示例
if __name__ == "__main__":

    planner = DubinsPathPlanner()

    # 创建无人机
    uav = UAV(uav_id=1, initial_position=(0, 0, 100), initial_heading=0.0)
    uav.set_path_planner(planner)
    
    # 设置路径
    path = [ (100, 0, 100), (100, 100, 100), (0, 100, 100)]
    uav.set_waypoints(path)
    
    # 激活无人机
    uav.activate()
    
    # 设置2倍速
    uav.set_speed_multiplier(2.0)
    
    # 仿真循环
    for i in range(400):
        uav.update_position(dt=1)
        print(f"Step {i}: {uav}")
        
        if uav.is_path_complete:
            print("Path completed!")
            break
        
    plot_uav_path(uav)
