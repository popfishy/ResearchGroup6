import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from enum import Enum
from typing import List, Tuple, Optional
import time
from utils import *

# 假设路径点之间的采样步长是固定的
PATH_STEP_SIZE = 5.0 

class UAV:
    def __init__(self, uav_id: int, initial_position: Tuple[float, float, float], initial_heading: float = 0.0):
        # 基本属性
        self.id = uav_id
        self.init_global_position = np.array(initial_position, dtype=float)
        self.position = np.array(initial_position, dtype=float)
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.heading = initial_heading
        self.status = UAVStatus.STANDBY
        self.fuel = 100.0
        self.turn_radius = 100.0
        self.max_speed = 60.0
        self.min_speed = 20.0
        self.current_speed = 30.0
        self.current_mission = MissionType.IDLE
        self.group_id = None
        self.waypoints = []
        self.planned_path = []
        self.current_path_index = 0
        self.path_planner = None
        self.speed_multiplier = 1.0
        self.position_history = [self.position.copy()]
        self.is_path_complete = False
        self.path_planning_complete = False
        
        # === 【修改点 1】: 为编队添加必要的内部属性 ===
        self.is_leader = False
        self.leader: Optional['UAV'] = None  # 指向领航者对象的引用
        self.formation_offset: Optional[Tuple[float, float]] = None # 相对于领航者的(dx, dy)偏移

        # 这两个属性现在由 _update_follower_position 内部计算并使用
        self.formation_target_pos = None
        self.formation_target_heading = 0.0
        
    def set_path_planner(self, path_planner):
        self.path_planner = path_planner
        
    def set_as_leader(self):
        self.is_leader = True
        self.leader = None
        self.formation_offset = None

    # === 【修改点 2】: 修改 set_as_follower 方法，接收领航者和偏移量 ===
    def set_as_follower(self, leader: 'UAV', offset: Tuple[float, float]):
        """
        将此无人机设置为跟随者。
        :param leader: 领航者UAV对象。
        :param offset: (dx, dy) 元组，表示在领航者坐标系中的相对位置。
        """
        self.is_leader = False
        self.leader = leader
        self.formation_offset = offset
        self.waypoints = []
        self.planned_path = []

    # set_formation_target 方法现在不再需要从外部调用，但保留以备将来扩展
    def set_formation_target(self, target_pos: Tuple[float, float, float], target_heading: float):
        if not self.is_leader:
            self.formation_target_pos = np.array(target_pos)
            self.formation_target_heading = target_heading
    
    def set_waypoints(self, waypoints: List[Tuple[float, float, float]]):
        self.waypoints = [np.array(wp, dtype=float) for wp in waypoints]
        self.current_path_index = 0
        self.is_path_complete = False
        self.path_planning_complete = False
        self._plan_dubins_path()
    
    def _plan_dubins_path(self):
        if not self.waypoints or not self.path_planner: return
        try:
            start_state = (self.position[0], self.position[1], self.heading)
            self.planned_path = []
            q0 = start_state
            for wp in self.waypoints:
                direction_vec = wp[:2] - np.array(q0[:2])
                target_heading = np.arctan2(direction_vec[1], direction_vec[0]) if np.linalg.norm(direction_vec) > 1e-6 else q0[2]
                q1 = (wp[0], wp[1], target_heading)
                segment_path, _ = self.path_planner.dubins_planner.plan(q0, q1, self.turn_radius, PATH_STEP_SIZE)
                if self.planned_path: self.planned_path.extend(segment_path[1:])
                else: self.planned_path.extend(segment_path)
                q0 = q1
            self.path_planning_complete = True
            print(f"UAV-{self.id} (Leader): Dubins path planned with {len(self.planned_path)} points")
        except Exception as e:
            print(f"UAV-{self.id}: Path planning failed: {e}")

    def update_position(self, dt: float):
        dt = dt * self.speed_multiplier
        if self.status != UAVStatus.ACTIVE: return

        if self.is_leader: self._update_leader_position(dt)
        else: self._update_follower_position(dt)
        
        fuel_consumption_rate = 0.1
        self.fuel = max(0.0, self.fuel - fuel_consumption_rate * dt)
        self.position_history.append(self.position.copy())
        
        if self.fuel <= 0 and self.status != UAVStatus.DESTROYED:
            print(f"UAV-{self.id} ran out of fuel and was destroyed.")
            self.destroy()

    def _update_leader_position(self, dt: float):
        if not self.path_planning_complete or not self.planned_path or self.is_path_complete: return
        distance_to_travel = self.current_speed * dt
        num_steps_to_advance = max(1, round(distance_to_travel / PATH_STEP_SIZE))
        self.current_path_index += num_steps_to_advance
        if self.current_path_index >= len(self.planned_path) - 1:
            self.current_path_index = len(self.planned_path) - 1
            self.is_path_complete = True
            
        new_point = self.planned_path[self.current_path_index]
        self.position[0], self.position[1] = new_point[0], new_point[1]
        self.heading = new_point[2]
        self.velocity = np.array([self.current_speed * np.cos(self.heading), self.current_speed * np.sin(self.heading), 0.0])
        
    def _update_follower_position(self, dt: float):
        if self.leader is None or self.formation_offset is None:
            self.velocity = np.array([0.0, 0.0, 0.0])
            return

        # 1. 计算理论目标点（这部分逻辑是正确的，保持不变）
        leader_pos, leader_heading = self.leader.position, self.leader.heading
        dx, dy = self.formation_offset
        cos_h, sin_h = np.cos(leader_heading), np.sin(leader_heading)
        
        rotated_dx = dx * cos_h - dy * sin_h
        rotated_dy = dx * sin_h + dy * cos_h
        
        self.formation_target_pos = np.array((
            leader_pos[0] + rotated_dx,
            leader_pos[1] + rotated_dy,
            leader_pos[2]
        ))
        self.formation_target_heading = leader_heading
        
        # 2. 计算与目标点的几何关系
        current_pos_2d = self.position[:2]
        target_pos_2d = self.formation_target_pos[:2]
        vector_to_target = target_pos_2d - current_pos_2d
        distance_to_target = np.linalg.norm(vector_to_target)

        if distance_to_target < 1.0:
            self.position[:2] = target_pos_2d
            self.heading = self.formation_target_heading
            self.velocity = np.array([0.0, 0.0, 0.0])
            return
            
        # === 【核心修正：动态速度控制器】 ===
        # 根据与目标点的距离来调整速度，赋予跟随者追赶能力
        leader_speed = self.leader.current_speed
        
        # 定义一个“舒适区半径”，在此范围内，速度与领航者匹配
        COMFORT_RADIUS = 5.0 
        # 定义一个“最大加速区半径”，超出此范围，就用最大速度追赶
        MAX_BOOST_RADIUS = 100.0 
        
        speed = 0.0
        if distance_to_target < COMFORT_RADIUS:
            # 在舒适区内，速度逐渐向领航者速度靠拢，以实现稳定跟随
            speed = leader_speed * (distance_to_target / COMFORT_RADIUS)
            speed = max(speed, self.min_speed * 0.5) # 保证最低速度
        elif distance_to_target > MAX_BOOST_RADIUS:
            # 距离太远，开足马力追
            speed = self.max_speed
        else:
            # 在舒适区和最大加速区之间，速度进行线性插值
            # 距离越远，速度越快
            ratio = (distance_to_target - COMFORT_RADIUS) / (MAX_BOOST_RADIUS - COMFORT_RADIUS)
            speed = leader_speed + ratio * (self.max_speed - leader_speed)
        
        # 确保速度不超过最大值
        speed = min(speed, self.max_speed)
        
        # 3. 比例导航制导律计算转弯（这部分逻辑是正确的，保持不变）
        desired_heading = np.arctan2(vector_to_target[1], vector_to_target[0])
        heading_error = (desired_heading - self.heading + np.pi) % (2 * np.pi) - np.pi
        
        PROPORTIONAL_GAIN = 4.0
        desired_turn_rate = PROPORTIONAL_GAIN * heading_error
        
        max_turn_rate = speed / self.turn_radius if self.turn_radius > 0 else float('inf')
        turn_rate = np.clip(desired_turn_rate, -max_turn_rate, max_turn_rate)
        
        # 4. 更新无人机状态
        self.heading += turn_rate * dt
        self.heading = (self.heading + np.pi) % (2 * np.pi) - np.pi
        
        self.position[0] += speed * np.cos(self.heading) * dt
        self.position[1] += speed * np.sin(self.heading) * dt
        
        self.velocity = np.array([speed * np.cos(self.heading), speed * np.sin(self.heading), 0.0])
    
    def set_speed_multiplier(self, multiplier: float):
        self.speed_multiplier = max(0.1, multiplier)
    def activate(self):
        if self.fuel > 0: self.status = UAVStatus.ACTIVE
    def destroy(self):
        self.status = UAVStatus.DESTROYED
        self.velocity = np.array([0.0, 0.0, 0.0])
    def __str__(self):
        role = "Leader" if self.is_leader else "Follower"
        progress = ""
        if self.is_leader:
            progress = f"path {self.current_path_index}/{len(self.planned_path)}" if self.planned_path else "no path"
        else:
            dist = np.linalg.norm(self.position[:2] - self.leader.position[:2]) if self.leader else float('inf')
            progress = f"dist_to_leader={dist:.1f}m"
        return f"UAV-{self.id} ({role}): pos=({self.position[0]:.1f},{self.position[1]:.1f}), {progress}, status={self.status.value}"

# ==================== 测试代码 (已同步修改) ====================
class SimpleDubinsPlanner:
    def plan(self, q0, q1, turning_radius, step_size):
        try:
            import dubins
            path = dubins.shortest_path(q0, q1, turning_radius)
            configurations, _ = path.sample_many(step_size)
            return configurations, _
        except ImportError:
            print("Error: 'dubins' library not found. Please install it with 'pip install dubins'.")
            return [], None
class MockPathPlanner:
    def __init__(self):
        self.dubins_planner = SimpleDubinsPlanner()

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    leader = UAV(uav_id=1, initial_position=(0, 0, 100), initial_heading=np.pi/4)
    follower1 = UAV(uav_id=2, initial_position=(-50, 50, 100), initial_heading=np.pi/4)
    follower2 = UAV(uav_id=3, initial_position=(50, -50, 100), initial_heading=np.pi/4)

    # === 【修改点 4】: 使用新的方式设置跟随者 ===
    leader.set_as_leader()
    # 定义队形偏移量：(dx, dy)
    # 领航者在(0,0)，前方是Y轴正向，右方是X轴正向
    follower1.set_as_follower(leader=leader, offset=(-20, 20)) # 左后方
    follower2.set_as_follower(leader=leader, offset=(20, -20))  # 右后方

    mock_planner = MockPathPlanner()
    leader.set_path_planner(mock_planner)
    
    leader.activate()
    follower1.activate()
    follower2.activate()

    waypoints = [(500, 200, 100), (500, 800, 100), (0, 500, 100)]
    leader.set_waypoints(waypoints)
    
    dt = 0.1
    # === 【修改点 5】: 仿真循环变得极其简单 ===
    for i in range(1500):
        # 只需要按顺序更新每个无人机即可
        leader.update_position(dt)
        follower1.update_position(dt) # follower1内部会自动计算目标并飞过去
        follower2.update_position(dt) # follower2内部会自动计算目标并飞过去
        
        if i % 100 == 0:
            print(f"--- Step {i} ---")
            print(f"  {leader}")
            print(f"  {follower1}")
            print(f"  {follower2}")
            
        if leader.is_path_complete:
            print(f"\nLeader has completed the path at step {i}.")
            break
            
    # 绘图部分保持不变
    plt.figure(figsize=(12, 12))
    history_leader = np.array(leader.position_history)
    history_f1 = np.array(follower1.position_history)
    history_f2 = np.array(follower2.position_history)
    plt.plot(history_leader[:, 0], history_leader[:, 1], 'r-', linewidth=2, label='Leader Trajectory')
    plt.plot(history_f1[:, 0], history_f1[:, 1], 'g--', label='Follower 1 Trajectory')
    plt.plot(history_f2[:, 0], history_f2[:, 1], 'b--', label='Follower 2 Trajectory')
    wp_array = np.array(waypoints)
    plt.plot(wp_array[:, 0], wp_array[:, 1], 'ko', markersize=10, label='Waypoints')
    for i in range(0, len(history_leader), 150):
        plt.plot([history_leader[i,0], history_f1[i,0]], [history_leader[i,1], history_f1[i,1]], 'k-', alpha=0.2)
        plt.plot([history_leader[i,0], history_f2[i,0]], [history_leader[i,1], history_f2[i,1]], 'k-', alpha=0.2)
        plt.plot([history_f1[i,0], history_f2[i,0]], [history_f1[i,1], history_f2[i,1]], 'k-', alpha=0.2)
    plt.title('Leader-Follower Formation Simulation (Internal Follower Logic)')
    plt.xlabel('X (m)'); plt.ylabel('Y (m)'); plt.legend(); plt.grid(True); plt.axis('equal'); plt.show()
