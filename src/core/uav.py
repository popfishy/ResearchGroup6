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
        self.current_mission = MissionType.PREPARATION
        self.group_id = None
        self.attack_target_position = None  # 攻击目标位置 [x, y, z]
      
        # 路径规划相关
        self.waypoints = []  # 原始航路点 (仅领航者使用)
        self.planned_path = []  # Dubins规划后的详细路径点 (仅领航者使用)
        self.current_path_index: int = 0
        self.path_planner = None  # 外部路径规划器接口
        self.path_index: int = 0

        # 编队控制相关属性
        self.is_leader = False
        self.formation_offset = None      # 跟随者的相对偏移 [x, y]
        self.formation_target_pos = None  # 跟随者的动态目标位置 [x, y, z]
        self.formation_target_heading = 0.0  # 跟随者的动态目标航向
        self.leader = None
      
        # 封控巡逻相关属性
        self.patrol_center = None           # 巡逻圆心 (x, y)
        self.patrol_radius = None           # 巡逻半径
        self.patrol_altitude = None         # 巡逻高度
        self.patrol_path = []               # 完整巡逻路径（Dubins + 圆形巡逻）
        self.patrol_phase = "unassigned"    # 巡逻阶段: "approaching", "patrolling", "unassigned"
        self.circle_start_index = 0         # 圆形巡逻开始的路径索引
      
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
        self.set_speed_limits(15, 45, 60)

    def set_formation_target(self, target_pos: Tuple[float, float, float], target_heading: float):
        """为跟随者设置其动态的队形目标点"""
        if not self.is_leader:
            self.formation_target_pos = np.array(target_pos)
            self.formation_target_heading = target_heading
          
    def unset_formation_roles(self):
        """清除无人机的编队角色，使其成为独立单位。"""
        self.is_leader = False
        self.leader = None
        self.formation_offset = None
        self.formation_target_pos = None
        self.set_speed_limits(15, 30, 60) # min, current, max

    def set_attack_target(self, target_position: np.ndarray, use_dubins: bool = False):
        """设置攻击目标并规划攻击路径"""
        attack_height = self.position[2]  # 使用当前飞行高度
        self.attack_target_position = np.array([target_position[0], target_position[1], attack_height], dtype=float)
        self.current_mission = MissionType.ATTACK
        self._plan_attack_path(use_dubins=use_dubins)

    def set_patrol_assignment(self, center: Tuple[float, float], radius: float, altitude: float):
        """设置封控巡逻任务"""
        self.patrol_center = center
        self.patrol_radius = radius
        self.patrol_altitude = altitude
        self.current_mission = MissionType.CONTAINMENT
        self.patrol_phase = "approaching"
        # TODO：重置路径规划状态
        # self.planned_path = []
        # self.patrol_path = []
        # self.path_index = 0
        # self.is_path_complete = False
        # self.path_planning_complete = False
        self._plan_patrol_path()

    # ==================== 领航者专用方法 ====================
    def set_waypoints(self, waypoints: List[Tuple[float, float, float]]):
        """设置航路点，触发Dubins路径规划 (主要由领航者调用)"""
        self.waypoints = [np.array(wp, dtype=float) for wp in waypoints]
        self.current_path_index = 0
        self.is_path_complete = False
        self.path_planning_complete = False
        self._plan_dubins_path()
  
    def set_planned_path(self, path: List[np.ndarray]):
        """
        直接设置已经规划好的密集路径点。
        这将绕过内部的Dubins规划，用于接收来自外部规划器（如RegionCover）的路径。
        """
        self.planned_path = path
        self.current_path_index = 0
        self.path_index = 0
        self.is_path_complete = False
        self.path_planning_complete = True # 路径已提供，视为规划完成

    def _plan_dubins_path(self):
        """使用Dubins模型规划路径 (领航者逻辑)"""
        if not self.waypoints or not self.path_planner:
            return
      
        try:
            start_state = (self.position[0], self.position[1], self.heading)
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
            print(f"UAV-{self.id} (Leader): Dubins规划其共规划 {len(self.planned_path)} 个点")
          
        except Exception as e:
            print(f"UAV-{self.id}: Path planning failed: {e}")

    def _plan_attack_path(self, use_dubins: bool = False):
        """规划攻击路径：从当前位置到攻击目标位置"""
        if self.attack_target_position is None:
            print(f"UAV-{self.id}: 无法规划攻击路径，目标位置未设置")
            return
      
        current_pos = self.position.copy()
        target_pos = self.attack_target_position.copy()
      
        if use_dubins and self.path_planner:
            try:
                start_state = (current_pos[0], current_pos[1], self.heading)
              
                # 计算到达目标的合适航向
                direction_vec = target_pos[:2] - current_pos[:2]
                target_heading = np.arctan2(direction_vec[1], direction_vec[0]) if np.linalg.norm(direction_vec) > 1e-6 else self.heading
                target_state = (target_pos[0], target_pos[1], target_heading)
              
                # 调用Dubins规划器
                attack_path, _ = self.path_planner.dubins_planner.plan(start_state, target_state, self.turn_radius, PATH_STEP_SIZE)
              
                # 设置路径
                self.planned_path = [np.array([p[0], p[1], p[2]]) for p in attack_path]
                self.path_index = 0
                self.is_path_complete = False
                self.path_planning_complete = True
              
                print(f"UAV-{self.id}: 使用Dubins规划器生成攻击路径，共 {len(self.planned_path)} 个点")
              
            except Exception as e:
                print(f"UAV-{self.id}: Dubins攻击路径规划失败: {e}，使用直线路径")
                self._plan_simple_attack_path()
        else:
            # 使用简单直线路径（适用于旋翼无人机）
            self._plan_simple_attack_path()
  
    def _plan_simple_attack_path(self):
        """规划简单的直线攻击路径（质点模型，确保精确到达目标）"""
        current_pos = self.position.copy()
        target_pos = self.attack_target_position.copy()
      
        # 只计算XY平面的路径向量，Z轴保持不变
        path_vector_xy = target_pos[:2] - current_pos[:2]  # 只考虑XY方向
        path_distance = np.linalg.norm(path_vector_xy)
      
        if path_distance < 1e-6:
            # 已经在目标XY位置
            self.planned_path = [np.array([current_pos[0], current_pos[1], self.heading])]
            self.path_index = 0
            self.is_path_complete = True
            self.path_planning_complete = True
            print(f"UAV-{self.id}: 已在攻击目标位置")
            return
      
        # 减小路径规划步长，提高精度
        step_size = 10.0  # 从30米减小到10米
        num_steps = max(1, int(path_distance / step_size))
      
        self.planned_path = []
        attack_height = current_pos[2]  # 保持当前飞行高度
      
        for i in range(num_steps + 1):
            ratio = min(1.0, i / num_steps)
            # 只在XY平面内插值，Z轴保持固定
            point_x = current_pos[0] + ratio * path_vector_xy[0]
            point_y = current_pos[1] + ratio * path_vector_xy[1]
          
            # 计算该点的航向（朝向目标方向）
            heading = np.arctan2(path_vector_xy[1], path_vector_xy[0])
          
            self.planned_path.append(np.array([point_x, point_y, heading]))
      
        # 确保最后一个点是精确的目标位置
        final_heading = np.arctan2(path_vector_xy[1], path_vector_xy[0])
        self.planned_path[-1] = np.array([target_pos[0], target_pos[1], final_heading])
      
        self.path_index = 0
        self.is_path_complete = False
        self.path_planning_complete = True

    def _plan_patrol_path(self):
        """规划封控巡逻路径：Dubins路径 + 圆形巡逻"""
        if not self.patrol_center or not self.patrol_radius:
            return
      
        current_pos = self.position.copy()
        center_x, center_y = self.patrol_center
      
        # 1. 计算圆上的接入点（选择距离当前位置最近的圆周点）
        vector_to_center = np.array([center_x - current_pos[0], center_y - current_pos[1]])
        distance_to_center = np.linalg.norm(vector_to_center)
      
        if distance_to_center > 1e-6:
            # 计算最近的圆周接入点
            direction_to_center = vector_to_center / distance_to_center
            entry_point = np.array([center_x, center_y]) - direction_to_center * self.patrol_radius
          
            # 计算接入点的切线方向（逆时针）
            tangent_direction = np.array([-direction_to_center[1], direction_to_center[0]])
            entry_heading = np.arctan2(tangent_direction[1], tangent_direction[0])
        else:
            # 如果已经在圆心，选择任意接入点
            entry_point = np.array([center_x + self.patrol_radius, center_y])
            entry_heading = np.pi / 2  # 向北
      
        # 2. 规划Dubins路径（从当前位置到圆周接入点）
        if self.path_planner and self.path_planner.dubins_planner:
            try:
                start_state = (current_pos[0], current_pos[1], self.heading)
                end_state = (entry_point[0], entry_point[1], entry_heading)
              
                dubins_path, _ = self.path_planner.dubins_planner.plan(
                    start_state, end_state, self.turn_radius, PATH_STEP_SIZE
                )
              
                # 转换为numpy数组格式
                approach_path = [np.array([p[0], p[1], p[2]]) for p in dubins_path]
            except Exception as e:
                print(f"UAV-{self.id}: Dubins巡逻接近路径规划失败: {e}，使用直线路径")
                approach_path = self._plan_simple_approach_path(entry_point, entry_heading)
        else:
            approach_path = self._plan_simple_approach_path(entry_point, entry_heading)
      
        # 3. 生成圆形巡逻路径
        circle_path = self._generate_circle_patrol_path(entry_point, entry_heading)
      
        # 4. 合并路径
        self.patrol_path = approach_path + circle_path
        self.circle_start_index = len(approach_path)
        self.path_index = 0
        self.is_path_complete = False
        self.path_planning_complete = True
    
  
    def _plan_simple_approach_path(self, entry_point: np.ndarray, entry_heading: float):
        """规划简单的直线接近路径"""
        current_pos = self.position.copy()
      
        # 计算路径向量
        path_vector = entry_point - current_pos[:2]
        path_distance = np.linalg.norm(path_vector)
      
        if path_distance < 1e-6:
            return [np.array([current_pos[0], current_pos[1], entry_heading])]
      
        # 生成路径点
        step_size = 20.0  # 20米步长
        num_steps = max(1, int(path_distance / step_size))
      
        approach_path = []
        for i in range(num_steps + 1):
            ratio = min(1.0, i / num_steps)
            point_x = current_pos[0] + ratio * path_vector[0]
            point_y = current_pos[1] + ratio * path_vector[1]
          
            # 最后一点使用目标航向，其他点朝向目标方向
            if i == num_steps:
                heading = entry_heading
            else:
                heading = np.arctan2(path_vector[1], path_vector[0])
          
            approach_path.append(np.array([point_x, point_y, heading]))
      
        return approach_path
  
    def _generate_circle_patrol_path(self, entry_point: np.ndarray, entry_heading: float):
        """生成圆形巡逻路径"""
        center_x, center_y = self.patrol_center
        radius = self.patrol_radius
      
        # 计算一圈的路径点数（保证足够密集）
        circumference = 2 * np.pi * radius
        step_size = 15.0  # 15米步长
        num_points = max(8, int(circumference / step_size))
      
        # 计算起始角度（从接入点开始）
        start_vector = entry_point - np.array([center_x, center_y])
        start_angle = np.arctan2(start_vector[1], start_vector[0])
      
        circle_path = []
        for i in range(num_points):
            # 逆时针方向巡逻
            angle = start_angle + (2 * np.pi * i) / num_points
          
            # 计算路径点位置
            x = center_x + radius * np.cos(angle)
            y = center_y + radius * np.sin(angle)
          
            # 计算切线方向（逆时针）
            tangent_heading = angle + np.pi / 2
          
            circle_path.append(np.array([x, y, tangent_heading]))
      
        return circle_path

    # ==================== 仿真核心更新方法 ====================
    def update_position(self, dt: float):
        """更新无人机位置 - 根据任务类型调用不同逻辑"""
        if self.status != UAVStatus.ACTIVE:
            return
      
        # 根据当前任务类型选择更新逻辑
        if self.current_mission == MissionType.ATTACK:
            self._update_attack_position(dt)
        elif self.current_mission == MissionType.CONTAINMENT:
            self._update_containment_position(dt)
        elif self.current_mission == MissionType.PREPARATION or self.current_mission == MissionType.RECONNAISSANCE:
            if self.is_leader:
                self._update_leader_position(dt)
            else:
                self._update_follower_position(dt)
      
        MAX_HISTORY_LENGTH = 200  # 只保留最近500个点
        self.position_history.append(self.position.copy())
        if len(self.position_history) > MAX_HISTORY_LENGTH:
            self.position_history = self.position_history[-MAX_HISTORY_LENGTH:]
        
        # 燃料消耗（如果需要的话）
        # fuel_consumption_rate = 0.1
        # self.fuel -= fuel_consumption_rate * dt
        # self.fuel = max(0.0, self.fuel)
        
        # if self.fuel <= 0 and self.status != UAVStatus.DESTROYED:
        #     print(f"UAV-{self.id} ran out of fuel and was destroyed.")
        #     self.destroy()

    def _update_leader_position(self, dt: float):
        """领航者沿着预先规划的路径移动"""
        if not self.path_planning_complete or self.is_path_complete:
            self.velocity = np.array([0.0, 0.0, 0.0])
            return

        pos_before_update = self.position.copy()
        distance_to_move = self.current_speed * dt
      
        while distance_to_move > 0 and not self.is_path_complete:
            # 检查是否已在或超过路径的最后一个点
            if self.path_index >= len(self.planned_path):
                self.is_path_complete = True
                break

            # 获取当前目标点
            current_target = self.planned_path[self.path_index]
            vector_to_target = current_target[:2] - self.position[:2]  # 从当前位置到目标点
            distance_to_target = np.linalg.norm(vector_to_target)

            if distance_to_target < 1e-6:
                # 已到达当前目标点，移动到下一个点
                self.path_index += 1
                continue

            if distance_to_move >= distance_to_target:
                # 可以到达当前目标点
                self.position[:2] = current_target[:2]
                self.heading = current_target[2] if len(current_target) > 2 else self.heading
                distance_to_move -= distance_to_target
                self.path_index += 1
            else:
                # 只能部分移动
                move_vector = (vector_to_target / distance_to_target) * distance_to_move
                self.position[:2] += move_vector
                distance_to_move = 0
      
        # 根据实际位移反算速度
        if dt > 0:
            self.velocity = (self.position - pos_before_update) / dt
      
        # 更新航向
        if np.linalg.norm(self.velocity[:2]) > 0.1:
            self.heading = np.arctan2(self.velocity[1], self.velocity[0])

    # def _update_follower_position(self, dt: float):
    #     """旋翼无人机的简化跟踪控制 - 质点模型"""
    #     if self.formation_target_pos is None:
    #         self.velocity = np.array([0.0, 0.0, 0.0])
    #         return
    #     target_pos_2d = self.formation_target_pos[:2]
    #     current_pos_2d = self.position[:2]
    #     vector_to_target = target_pos_2d - current_pos_2d
    #     distance_to_target = np.linalg.norm(vector_to_target)
      
    #     # 到达判定
    #     ACCEPTANCE_RADIUS = 2.0
    #     if self.leader and self.leader.is_path_complete and distance_to_target < ACCEPTANCE_RADIUS:
    #         self.position[:2] = target_pos_2d
    #         self.heading = self.formation_target_heading
    #         self.velocity = np.array([0.0, 0.0, 0.0])
    #         return
    #     if distance_to_target < 1e-6:
    #         self.velocity = np.array([0.0, 0.0, 0.0])
    #         return
      
    #     # 质点模型：直接朝目标点移动，无转弯半径限制
    #     direction = vector_to_target / distance_to_target
      
    #     # 速度控制：距离远时全速，接近时减速
    #     if distance_to_target > 50.0:  # 50米外全速
    #         speed = self.current_speed
    #     else:  # 50米内线性减速
    #         speed = self.current_speed * (distance_to_target / 50.0)
    #         speed = max(speed, self.min_speed * 0.3)  # 保持最小速度
      
    #     # 直接设置速度向量
    #     self.velocity[:2] = direction * speed
    #     self.velocity[2] = 0.0
      
    #     # 更新位置
    #     self.position += self.velocity * dt
      
    #     # 更新航向（旋翼无人机可以瞬间转向）
    #     self.heading = np.arctan2(self.velocity[1], self.velocity[0])

    def _update_follower_position(self, dt: float):
        """Simplified follower control logic - directly compute positions."""
        # if self.formation_target_pos is None or not self.leader:
        #     self.velocity = np.array([0.0, 0.0, 0.0])
        #     return

        # # Get the leader's current position
        # leader_pos = self.leader.position
        # # Calculate the follower's desired position using the offset
        # desired_pos = leader_pos[:2] + self.formation_offset  # Assuming offset is in 2D

        # # Update the follower's position to the desired position directly
        # self.position[:2] = desired_pos
        # self.heading = self.leader.heading  # Follow the leader's heading

        # # Log for debug purposes
        # print(f"UAV-{self.id} position updated to {self.position}")

        # self.velocity = np.array([0.0, 0.0, 0.0])  # Stop any additional velocity since position is set
        if self.formation_target_pos is None or not self.leader:
            self.velocity = np.array([0.0, 0.0, 0.0])
            return
    
        # Get the leader's current position
        leader_pos = self.leader.position
        # Calculate the desired position
        desired_pos = leader_pos[:2] + self.formation_offset
    
        # Move towards the desired position gradually
        step_distance = self.current_speed * dt  # How far to move this step
        vector_to_target = desired_pos - self.position[:2]
        distance_to_target = np.linalg.norm(vector_to_target)

        if distance_to_target > step_distance:
            direction = vector_to_target / distance_to_target
            self.position[:2] += direction * step_distance  # Move toward the desired position
        else:
            self.position[:2] = desired_pos  # Snap to desired position if close enough
    
        self.heading = np.arctan2(vector_to_target[1], vector_to_target[0])

    def _update_attack_position(self, dt: float):
        """攻击任务的位置更新 - 基于路径规划，精确到达目标"""
        if self.attack_target_position is None:
            self.velocity = np.array([0.0, 0.0, 0.0])
            return
      
        # 检查是否有规划好的路径
        if not self.path_planning_complete or not self.planned_path:
            self.velocity = np.array([0.0, 0.0, 0.0])
            return
      
        # 使用与领航者相同的路径跟踪逻辑
        pos_before_update = self.position.copy()
        distance_to_move = self.current_speed * dt
      
        while distance_to_move > 0 and not self.is_path_complete:
            # 检查是否已到达路径末端
            if self.path_index >= len(self.planned_path):
                final_distance = np.linalg.norm(self.position[:2] - self.attack_target_position[:2])
                if final_distance <= 5.0:
                    self.position[:2] = self.attack_target_position[:2]
                    self.is_path_complete = True
                    self.destroy()  # 设置为DESTROYED状态
                    return
                else:
                    print(f"UAV-{self.id} 路径完成但未精确到达目标，距离目标 {final_distance:.1f}m，继续移动")
                    # 直接朝目标移动
                    vector_to_target = self.attack_target_position[:2] - self.position[:2]
                    direction = vector_to_target / np.linalg.norm(vector_to_target)
                    move_distance = min(distance_to_move, np.linalg.norm(vector_to_target))
                    self.position[:2] += direction * move_distance
                    distance_to_move -= move_distance
                break
          
            # 获取当前目标点
            current_target = self.planned_path[self.path_index]
            vector_to_target = current_target[:2] - self.position[:2]  # 只考虑XY方向
            distance_to_target = np.linalg.norm(vector_to_target)
          
            PATH_POINT_RADIUS = 5.0
            if distance_to_target < PATH_POINT_RADIUS:
                # 到达当前路径点
                self.position[:2] = current_target[:2]  # 只更新XY坐标
                self.heading = current_target[2] if len(current_target) > 2 else self.heading
                distance_to_move -= distance_to_target
                self.path_index += 1
            else:
                # 向当前目标点移动
                if distance_to_move >= distance_to_target:
                    self.position[:2] = current_target[:2]  # 只更新XY坐标
                    self.heading = current_target[2] if len(current_target) > 2 else self.heading
                    distance_to_move -= distance_to_target
                    self.path_index += 1
                else:
                    # 部分移动
                    move_vector = (vector_to_target / distance_to_target) * distance_to_move
                    self.position[:2] += move_vector  # 只更新XY坐标
                    distance_to_move = 0
        
        # 计算速度
        if dt > 0:
            velocity_3d = (self.position - pos_before_update) / dt
            self.velocity[:2] = velocity_3d[:2]  # 只更新XY速度
            self.velocity[2] = 0.0  # Z轴速度始终为0
        
        # 更新航向
        if np.linalg.norm(self.velocity[:2]) > 0.1:
            self.heading = np.arctan2(self.velocity[1], self.velocity[0])
        
        # 如果已完成攻击，停止移动
        if self.is_path_complete and self.status == UAVStatus.ACTIVE:
            self.velocity = np.array([0.0, 0.0, 0.0])
            final_distance = np.linalg.norm(self.position[:2] - self.attack_target_position[:2])
            if final_distance <= 2.0:
                print(f"UAV-{self.id} 到达攻击目标，执行攻击并被摧毁")
                self.destroy()

    def _update_containment_position(self, dt: float):
        """封控任务的位置更新 - 执行圆形巡逻"""
        if not self.patrol_path or not self.path_planning_complete:
            self.velocity = np.array([0.0, 0.0, 0.0])
            return
        
        pos_before_update = self.position.copy()
        distance_to_move = self.current_speed * dt
        
        while distance_to_move > 0:
            # 检查当前巡逻阶段
            if self.path_index < self.circle_start_index:
                # 接近阶段：沿路径移动到圆周
                self.patrol_phase = "approaching"
                if self._move_along_path(distance_to_move):
                    distance_to_move = 0
                else:
                    break
            else:
                # 巡逻阶段：在圆周上巡逻
                self.patrol_phase = "patrolling"
                if self._move_in_circle_patrol(distance_to_move):
                    distance_to_move = 0
                else:
                    break
        
        # 计算速度
        if dt > 0:
            velocity_3d = (self.position - pos_before_update) / dt
            self.velocity[:2] = velocity_3d[:2]
            self.velocity[2] = 0.0
        
        # 更新航向
        if np.linalg.norm(self.velocity[:2]) > 0.1:
            self.heading = np.arctan2(self.velocity[1], self.velocity[0])

    def _move_along_path(self, distance_to_move: float) -> bool:
        """沿路径移动（接近阶段）"""
        while distance_to_move > 0 and self.path_index < len(self.patrol_path):
            current_target = self.patrol_path[self.path_index]
            vector_to_target = current_target[:2] - self.position[:2]
            distance_to_target = np.linalg.norm(vector_to_target)
            
            if distance_to_target < 2.0:  # 到达当前路径点
                self.position[:2] = current_target[:2]
                self.heading = current_target[2]
                distance_to_move -= distance_to_target
                self.path_index += 1
            else:
                # 向目标移动
                if distance_to_move >= distance_to_target:
                    self.position[:2] = current_target[:2]
                    self.heading = current_target[2]
                    distance_to_move -= distance_to_target
                    self.path_index += 1
                else:
                    move_vector = (vector_to_target / distance_to_target) * distance_to_move
                    self.position[:2] += move_vector
                    distance_to_move = 0
                    return True
        
        return distance_to_move == 0

    def _move_in_circle_patrol(self, distance_to_move: float) -> bool:
        """在圆周上巡逻"""
        circle_path_length = len(self.patrol_path) - self.circle_start_index
        
        while distance_to_move > 0:
            # 计算在圆形路径中的当前索引
            circle_index = (self.path_index - self.circle_start_index) % circle_path_length
            current_path_index = self.circle_start_index + circle_index
            
            current_target = self.patrol_path[current_path_index]
            vector_to_target = current_target[:2] - self.position[:2]
            distance_to_target = np.linalg.norm(vector_to_target)
            
            if distance_to_target < 2.0:  # 到达当前路径点
                self.position[:2] = current_target[:2]
                self.heading = current_target[2]
                distance_to_move -= distance_to_target
                
                # 移动到下一个巡逻点（循环）
                self.path_index += 1
                if self.path_index >= len(self.patrol_path):
                    self.path_index = self.circle_start_index  # 重新开始圆形巡逻
            else:
                # 向目标移动
                if distance_to_move >= distance_to_target:
                    self.position[:2] = current_target[:2]
                    self.heading = current_target[2]
                    distance_to_move -= distance_to_target
                    
                    self.path_index += 1
                    if self.path_index >= len(self.patrol_path):
                        self.path_index = self.circle_start_index
                else:
                    move_vector = (vector_to_target / distance_to_target) * distance_to_move
                    self.position[:2] += move_vector
                    distance_to_move = 0
                    return True
        
        return True
    
    def activate(self):
        """激活无人机"""
        if self.fuel > 0:
            self.status = UAVStatus.ACTIVE
    
    def destroy(self):
        """摧毁无人机"""
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
