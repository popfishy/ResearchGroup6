import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from enum import Enum
from typing import List, Tuple, Optional
import time
from .utils import *

# 假设路径点之间的采样步长是固定的
PATH_STEP_SIZE = 5.0 

class UAV:
    def __init__(self, uav_id: int, initial_position: Tuple[float, float, float], initial_heading: float = 0.0):
        # 基本属性
        self.id = uav_id
        self.init_global_position = np.array(initial_position, dtype=float)
        self.position = np.array(initial_position, dtype=float)
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.heading = initial_heading  # 初始航向角 (rad)
        self.status = UAVStatus.STANDBY
        self.fuel = 100.0
        self.turn_radius = 50.0
        self.max_speed = 0.0
        self.min_speed = 0.0
        self.current_speed = 0.0
        
        # 任务相关
        self.current_mission = MissionType.IDLE
        self.group_id = None
        
        # 路径规划相关
        self.waypoints = []  # 原始航路点 (仅领航者使用)
        self.planned_path = []  # Dubins规划后的详细路径点 (仅领航者使用)
        self.current_path_index: int = 0
        self.path_planner = None  # 外部路径规划器接口
        self.path_index: int = 0

        # 编队控制相关属性
        self.is_leader = False
        self.formation_target_pos = None  # 跟随者的动态目标位置 [x, y, z]
        self.formation_target_heading = 0.0  # 跟随者的动态目标航向
        self.leader = None
        
        # 倍速控制相关
        self.speed_multiplier = 1.0
        
        # 仿真控制
        self.position_history = [self.position.copy()]
        self.is_path_complete = False
        self.path_planning_complete = False
    
    def set_path_planner(self, path_planner):
        """设置外部路径规划器"""
        self.path_planner = path_planner
        
    # ==================== 编队角色控制方法 ====================
    def set_speed_limits(self, min_speed:float, current_speed:float, max_speed:float):
        self.max_speed = max_speed
        self.current_speed = current_speed
        self.min_speed = min_speed

    def set_as_leader(self):
        """将此无人机设置为领航者"""
        self.is_leader = True
        self.formation_target_pos = None
        self.set_speed_limits(15, 30, 40)

    def unset_as_leader(self):
        """取消领航者角色"""
        self.is_leader = False

    def set_as_follower(self, leader, offset: Tuple[float, float]):
        """将此无人机设置为跟随者"""
        self.is_leader = False
        self.leader = leader
        self.formation_offset = np.array(offset) # <--- 存储偏移量
        self.waypoints = []
        self.planned_path = []
        self.set_speed_limits(15, 30, 60)
        print(f"UAV-{self.id} set as follower of UAV-{leader.id} with offset {self.formation_offset}")

    def set_formation_target(self, target_pos: Tuple[float, float, float], target_heading: float):
        """为跟随者设置其动态的队形目标点"""
        if not self.is_leader:
            self.formation_target_pos = np.array(target_pos)
            self.formation_target_heading = target_heading
    
    # ==================== 领航者专用方法 ====================
    def set_waypoints(self, waypoints: List[Tuple[float, float, float]]):
        """设置航路点，触发Dubins路径规划 (主要由领航者调用)"""
        self.waypoints = [np.array(wp, dtype=float) for wp in waypoints]
        self.current_path_index = 0
        self.is_path_complete = False
        self.path_planning_complete = False
        self._plan_dubins_path()
    
    def _plan_dubins_path(self):
        """使用Dubins模型规划路径 (领航者逻辑)"""
        if not self.waypoints or not self.path_planner:
            return
        
        try:
            start_state = (self.position[0], self.position[1], self.heading)
            self.planned_path = []
            q0 = start_state
            
            for wp in self.waypoints:
                direction_vec = wp[:2] - np.array(q0[:2])
                target_heading = np.arctan2(direction_vec[1], direction_vec[0]) if np.linalg.norm(direction_vec) > 1e-6 else q0[2]
                q1 = (wp[0], wp[1], target_heading)
                
                # 调用Dubins库，最后一个参数是步长
                segment_path, _ = self.path_planner.dubins_planner.plan(q0, q1, self.turn_radius, PATH_STEP_SIZE)
                
                if self.planned_path:
                    self.planned_path.extend(segment_path[1:])
                else:
                    self.planned_path.extend(segment_path)
                q0 = q1

            self.path_planning_complete = True
            print(f"UAV-{self.id} (Leader): Dubins path planned with {len(self.planned_path)} points")
            
        except Exception as e:
            print(f"UAV-{self.id}: Path planning failed: {e}")

    # ==================== 仿真核心更新方法 ====================
    def update_position(self, dt: float):
        """更新无人机位置 - 根据角色（领航者/跟随者）调用不同逻辑"""
        dt = dt * self.speed_multiplier
            
        if self.status != UAVStatus.ACTIVE:
            return

        if self.is_leader:
            self._update_leader_position(dt)
        else:
            self._update_follower_position(dt)
        
        # --- 通用逻辑 ---
        fuel_consumption_rate = 0.1
        self.fuel -= fuel_consumption_rate * dt
        self.fuel = max(0.0, self.fuel)
        
        self.position_history.append(self.position.copy())
        
        if self.fuel <= 0 and self.status != UAVStatus.DESTROYED:
            print(f"UAV-{self.id} ran out of fuel and was destroyed.")
            self.destroy()

    def _update_leader_position(self, dt: float):
        """
        【已修正】领航者沿着预先规划的路径移动。
        在每个时间步 dt 内，从当前位置沿着路径移动 'speed * dt' 的距离。
        速度根据位置变化反算得出，确保准确性。
        """
        if not self.path_planning_complete or self.is_path_complete:
            self.velocity = np.array([0.0, 0.0, 0.0])
            return

        pos_before_update = self.position.copy()
        distance_to_move = self.current_speed * dt
        
        while distance_to_move > 0 and not self.is_path_complete:
            if self.path_index >= len(self.planned_path) - 1:
                self.is_path_complete = True
                self.position[:2] = self.planned_path[-1][:2]
                break

            p_start_current = self.position[:2]
            p_end_target = self.planned_path[self.path_index][:2]
            
            vector_to_target = p_end_target - p_start_current
            distance_to_target = np.linalg.norm(vector_to_target)

            if distance_to_target < 1e-6:
                self.path_index += 1
                continue

            if distance_to_move >= distance_to_target:
                self.position[:2] = p_end_target
                distance_to_move -= distance_to_target
                self.path_index += 1
            else:
                move_vector = (vector_to_target / distance_to_target) * distance_to_move
                self.position[:2] += move_vector
                distance_to_move = 0
        
        # 根据实际位移反算速度
        if dt > 0:
            self.velocity = (self.position - pos_before_update) / dt
        
        # 更新航向
        if np.linalg.norm(self.velocity[:2]) > 0.1:
            self.heading = np.arctan2(self.velocity[1], self.velocity[0])

    def _update_follower_position(self, dt: float):
        """
        使用比例导航制导律和一个更平滑的速度控制器，以实现稳定追踪。
        """
        if self.formation_target_pos is None:
            # 没有目标，原地待命
            self.velocity = np.array([0.0, 0.0, 0.0])
            return

        # 1. 计算与目标点的几何关系
        target_pos_2d = self.formation_target_pos[:2]
        current_pos_2d = self.position[:2]
        vector_to_target = target_pos_2d - current_pos_2d
        distance_to_target = np.linalg.norm(vector_to_target)

        # 【新增】当领航者到达终点后，如果跟随者也到达了指定位置附近，则停止移动，避免转圈。
        ACCEPTANCE_RADIUS = 2.0  # 到达目标的接受半径（米）
        if self.leader and self.leader.is_path_complete and distance_to_target < ACCEPTANCE_RADIUS:
            self.position[:2] = target_pos_2d
            self.heading = self.formation_target_heading
            self.velocity = np.array([0.0, 0.0, 0.0])
            return

        # 【修正】移除过于激进的“吸附”逻辑，该逻辑会导致速度频繁归零。
        # 控制器本身和平滑减速逻辑足以处理逼近过程。
        # if distance_to_target < 1.0:
        #     self.position[:2] = target_pos_2d
        #     self.heading = self.formation_target_heading
        #     self.velocity = np.array([0.0, 0.0, 0.0])
        #     return

        # 2. 【改进】平滑的速度控制
        # 定义一个“减速区”，当无人机进入该区域时才开始减速
        # 例如，减速区半径为无人机2秒的飞行距离
        deceleration_radius = 2.0 * self.current_speed 
        if distance_to_target < deceleration_radius:
            # 在减速区内，速度与距离成正比
            speed = self.current_speed * (distance_to_target / deceleration_radius)
            # 设置一个最小速度，避免完全停下
            speed = max(speed, self.min_speed * 0.5) 
        else:
            # 在减速区外，保持全速
            speed = self.current_speed
        # 3. 【核心修正】使用比例导航制导律计算转弯角速度
        # 计算视线角（即期望的航向）
        desired_heading = np.arctan2(vector_to_target[1], vector_to_target[0])
        
        # 计算视线角与当前航向的误差
        heading_error = desired_heading - self.heading
        # 将误差归一化到 [-pi, pi]
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
        
        # 导航增益K，这是一个关键的可调参数。K=4通常效果不错。
        PROPORTIONAL_GAIN = 4.0 
        desired_turn_rate = PROPORTIONAL_GAIN * heading_error
        
        # 4. 施加物理约束（最大转弯角速度）
        # omega_max = v / R
        max_turn_rate = speed / self.turn_radius if self.turn_radius > 0 else float('inf')
        turn_rate = np.clip(desired_turn_rate, -max_turn_rate, max_turn_rate)
        
        # 5. 更新无人机的状态（位置和航向）
        self.heading += turn_rate * dt
        # 归一化航向
        self.heading = (self.heading + np.pi) % (2 * np.pi) - np.pi
        self.position[0] += speed * np.cos(self.heading) * dt
        self.position[1] += speed * np.sin(self.heading) * dt
        
        # 更新速度向量
        self.velocity = np.array([
            speed * np.cos(self.heading),
            speed * np.sin(self.heading),
            0.0
        ])
    
    # ==================== 其余方法 ====================
    def set_speed_multiplier(self, multiplier: float):
        self.speed_multiplier = max(0.1, multiplier)

    def activate(self):
        if self.fuel > 0:
            self.status = UAVStatus.ACTIVE
    
    def destroy(self):
        self.status = UAVStatus.DESTROYED
        self.velocity = np.array([0.0, 0.0, 0.0])
    
    def get_position_for_simulation(self) -> Tuple[float, float, float]:
        return tuple(self.position)
    
    def __str__(self):
        role = "Leader" if self.is_leader else "Follower"
        progress = ""
        if self.is_leader:
            progress = f"progress={self.current_path_index}/{len(self.planned_path)}"
        else:
            if self.formation_target_pos is not None:
                dist = np.linalg.norm(self.position[:2] - self.formation_target_pos[:2])
                progress = f"target_dist={dist:.1f}m"
            else:
                progress = "no target"
        return f"UAV-{self.id} ({role}): pos=({self.position[0]:.1f},{self.position[1]:.1f}), {progress}, status={self.status.value}"

# ==================== 测试代码 ====================
class SimpleDubinsPlanner:
    def plan(self, q0, q1, turning_radius, step_size):
        try:
            import dubins
            path = dubins.shortest_path(q0, q1, turning_radius)
            configurations, _ = path.sample_many(step_size)
            return configurations, _
        except ImportError:
            print("Error: 'dubins' library not found. Please install it using 'pip install dubins'.")
            return [], None

class MockPathPlanner:
    def __init__(self):
        self.dubins_planner = SimpleDubinsPlanner()

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # 创建一个领航者和两个跟随者
    leader = UAV(uav_id=1, initial_position=(0, 0, 100), initial_heading=np.pi/4)
    follower1 = UAV(uav_id=2, initial_position=(-50, 50, 100), initial_heading=np.pi/4)
    follower2 = UAV(uav_id=3, initial_position=(50, -50, 100), initial_heading=np.pi/4)

    leader.set_as_leader()
    follower1.set_as_follower()
    follower2.set_as_follower()

    mock_planner = MockPathPlanner()
    leader.set_path_planner(mock_planner)
    
    leader.activate()
    follower1.activate()
    follower2.activate()

    waypoints = [(500, 200, 100), (500, 800, 100), (0, 500, 100)]
    leader.set_waypoints(waypoints)
    
    dt = 0.1
    for i in range(1500):
        leader.update_position(dt)

        leader_pos = leader.position
        leader_heading = leader.heading
        
        dx1, dy1 = -50, -50 # 跟随者1的相对位置
        offset1_x = dx1 * np.cos(leader_heading) - dy1 * np.sin(leader_heading)
        offset1_y = dx1 * np.sin(leader_heading) + dy1 * np.cos(leader_heading)
        target1_pos = (leader_pos[0] + offset1_x, leader_pos[1] + offset1_y, leader_pos[2])
        follower1.set_formation_target(target1_pos, leader_heading)
        
        dx2, dy2 = 50, -50 # 跟随者2的相对位置
        offset2_x = dx2 * np.cos(leader_heading) - dy2 * np.sin(leader_heading)
        offset2_y = dx2 * np.sin(leader_heading) + dy2 * np.cos(leader_heading)
        target2_pos = (leader_pos[0] + offset2_x, leader_pos[1] + offset2_y, leader_pos[2])
        follower2.set_formation_target(target2_pos, leader_heading)
        
        follower1.update_position(dt)
        follower2.update_position(dt)
        
        if i % 100 == 0:
            print(f"--- Step {i} ---")
            print(f"  {leader}")
            print(f"  {follower1}")
            print(f"  {follower2}")
            
        if leader.is_path_complete:
            print(f"\nLeader has completed the path at step {i}.")
            break
            
    # 绘图
    plt.figure(figsize=(12, 12))
    history_leader = np.array(leader.position_history)
    history_f1 = np.array(follower1.position_history)
    history_f2 = np.array(follower2.position_history)
    
    plt.plot(history_leader[:, 0], history_leader[:, 1], 'r-', linewidth=2, label='Leader Trajectory')
    plt.plot(history_f1[:, 0], history_f1[:, 1], 'g--', label='Follower 1 Trajectory')
    plt.plot(history_f2[:, 0], history_f2[:, 1], 'b--', label='Follower 2 Trajectory')
    
    wp_array = np.array(waypoints)
    plt.plot(wp_array[:, 0], wp_array[:, 1], 'ko', markersize=10, label='Waypoints')

    # 绘制队形快照
    for i in range(0, len(history_leader), 150):
        plt.plot([history_leader[i,0], history_f1[i,0]], [history_leader[i,1], history_f1[i,1]], 'k-', alpha=0.2)
        plt.plot([history_leader[i,0], history_f2[i,0]], [history_leader[i,1], history_f2[i,1]], 'k-', alpha=0.2)
        plt.plot([history_f1[i,0], history_f2[i,0]], [history_f1[i,1], history_f2[i,1]], 'k-', alpha=0.2)

    plt.title('Leader-Follower Formation Simulation')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()
