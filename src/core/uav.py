import numpy as np
from enum import Enum
from typing import List, Tuple, Optional
import time

class UAVStatus(Enum):
    ACTIVE = "active"
    DESTROYED = "destroyed"
    RETURNING = "returning"
    STANDBY = "standby"
    CIRCLE = "circle"

class MissionType(Enum):
    PREPARATION = "preparation"
    RECONNAISSANCE = "reconnaissance"
    ATTACK = "attack"
    CONTAINMENT = "containment"
    IDLE = "idle"

class UAV:
    def __init__(self, uav_id: int, initial_position: Tuple[float, float, float]):
        # 基本属性
        self.id = uav_id
        self.init_global_position = np.array((0, 0, 0),dtype=float) # (altitude, latitude, longitude)
        self.position = np.array(initial_position, dtype=float)  # (x, y, z)
        self.velocity = np.array([0.0, 0.0, 0.0])  # 速度向量 (vx, vy, vz)
        self.heading = 0.0  # 航向角，弧度制
        self.status = UAVStatus.STANDBY
        self.fuel = 100.0  # 剩余燃料百分比
        self.turn_radius = 100.0  # 最小转弯半径，米
        self.max_speed = 40.0  # 最大速度，m/s
        self.min_speed = 15.0  # 最小速度，m/s
        self.current_speed = 30.0  # 当前速度大小
        
        # 任务相关
        self.current_mission = MissionType.IDLE
        self.group_id = None
        # TODO：兼容其他课题，暂时不清楚有什么用处
        self.task_flag = None
        self.task_phase = None
        self.task_type = None
        
        # 路径和导航相关
        self.target_position = None  # 目标位置
        self.path_points = []  # 路径点列表 [(x,y,z), ...]
        self.current_path_index = 0  # 当前路径点索引
        
        # 倍速控制相关
        self.speed_multiplier = 1.0  # 倍速系数，1.0为正常速度
        self.base_update_interval = 0.1  # 基础更新间隔，秒
        self.path_sampling_rate = 1  # 路径采样率，1表示不跳过，2表示每2个点取1个
        self.last_update_time = time.time()
        
        # 仿真控制
        self.position_history = [self.position.copy()]  # 位置历史记录
        self.is_path_complete = False
    
    def set_speed_multiplier(self, multiplier: float):
        """设置倍速系数"""
        self.speed_multiplier = max(0.1, multiplier)  # 最小0.1倍速
        self.update_sampling_rate()
    
    def update_sampling_rate(self):
        """根据倍速系数更新路径采样率"""
        if self.speed_multiplier >= 2.0:
            # 高倍速时，增加采样率（跳过更多路径点）
            self.path_sampling_rate = int(self.speed_multiplier)
        else:
            # 低倍速时，正常采样
            self.path_sampling_rate = 1
    
    def set_path(self, path_points: List[Tuple[float, float, float]]):
        """设置路径点列表（不包含当前位置）"""
        if not path_points:
            self.path_points = []
            self.target_position = None
            return
        
        self.path_points = [np.array(p, dtype=float) for p in path_points]
        self.current_path_index = 0
        self.target_position = self.path_points[0]  # 第一个目标
        self.is_path_complete = False
    
    def get_next_waypoint(self) -> Optional[np.ndarray]:
        """获取下一个路径点，考虑倍速和采样"""
        self.current_path_index += self.path_sampling_rate
                
        if self.current_path_index >= len(self.path_points):
            self.is_path_complete = True
            return None
        
        waypoint = self.path_points[self.current_path_index]
        return waypoint
    
    def normalize_angle(self, angle):
        """将角度归一化到 [-π, π] 范围"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def update_position(self, dt: float = None):
        """更新无人机位置"""
        if dt is None:
            current_time = time.time()
            dt = (current_time - self.last_update_time) * self.speed_multiplier
            self.last_update_time = current_time
        else:
            dt = dt * self.speed_multiplier
        
        if self.status != UAVStatus.ACTIVE or self.target_position is None:
            return
        
        # 计算到目标的方向和距离
        direction_vector = self.target_position - self.position
        distance_to_target = np.linalg.norm(direction_vector[:2])
        
        # 如果接近当前目标点，获取下一个路径点
        if distance_to_target < 5.0:  # 5米容差
            next_waypoint = self.get_next_waypoint()
            if next_waypoint is not None:
                self.target_position = next_waypoint
                direction_vector = self.target_position - self.position
                distance_to_target = np.linalg.norm(direction_vector)
            else:
                # 路径完成
                self.is_path_complete = True
                return
        
        if distance_to_target > 0:
            # 计算期望航向角
            desired_heading = np.arctan2(direction_vector[1], direction_vector[0])
            
            # 计算航向角差值
            heading_error = self.normalize_angle(desired_heading - self.heading)
            
            # 应用转弯半径约束
            max_turn_rate = self.current_speed / self.turn_radius  # rad/s
            max_heading_change = max_turn_rate * dt
            
            # 限制航向变化率
            if abs(heading_error) > max_heading_change:
                heading_change = np.sign(heading_error) * max_heading_change
            else:
                heading_change = heading_error
            
            # 更新航向角
            self.heading += heading_change
            self.heading = self.normalize_angle(self.heading)
            
            # 更新速度向量（基于当前航向角）
            self.velocity = np.array([
                self.current_speed * np.cos(self.heading),
                self.current_speed * np.sin(self.heading),
                0.0  # 简化为2D运动
            ])
            
            # 更新位置
            displacement = self.velocity * dt
            self.position += displacement
            
            # 更新燃料消耗（简化模型）
            fuel_consumption_rate = 0.1  # 每秒消耗0.1%
            self.fuel -= fuel_consumption_rate * dt * self.speed_multiplier
            self.fuel = max(0.0, self.fuel)
            
            # 记录位置历史
            self.position_history.append(self.position.copy())
            
            # 检查燃料状态
            if self.fuel <= 0:
                self.status = UAVStatus.DESTROYED
    
    def set_mission(self, mission_type: MissionType):
        """设置当前任务"""
        self.current_mission = mission_type
    
    def set_group_cluster(self, group_id: int, cluster_id: int):
        """设置分组和分簇信息"""
        self.group_id = group_id
        self.cluster_id = cluster_id
    
    def activate(self):
        """激活无人机"""
        if self.fuel > 0:
            self.status = UAVStatus.ACTIVE
    
    def destroy(self):
        """销毁无人机"""
        self.status = UAVStatus.DESTROYED
        self.velocity = np.array([0.0, 0.0, 0.0])
    
    def get_current_update_interval(self) -> float:
        """获取当前更新间隔（用于外部调用控制）"""
        return self.base_update_interval / self.speed_multiplier
    
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
            'path_progress': f"{self.current_path_index}/{len(self.path_points)}",
            'is_path_complete': self.is_path_complete
        }
    
    def __str__(self):
        return f"UAV-{self.id}: pos={self.position}, status={self.status.value}, fuel={self.fuel:.1f}%"


import matplotlib.pyplot as plt

def plot_uav_path(uav: UAV):
    history = np.array(uav.position_history)
    plt.figure(figsize=(8, 8))
    plt.plot(history[:, 0], history[:, 1], 'b-', label='Trajectory')
    plt.scatter(history[0, 0], history[0, 1], c='green', s=100, label='Start')
    plt.scatter(history[-1, 0], history[-1, 1], c='red', s=100, label='End')
    
    # 绘制原始路径点
    if uav.path_points:
        path = np.array(uav.path_points)
        plt.plot(path[:, 0], path[:, 1], 'ro--', label='Waypoints')
    
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title(f'UAV-{uav.id} Path')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# 使用示例
if __name__ == "__main__":
    # 创建无人机
    uav = UAV(uav_id=1, initial_position=(0, 0, 100))
    
    # 设置路径
    path = [ (1000, 0, 100), (1000, 1000, 100), (0, 1000, 100)]
    uav.set_path(path)
    
    # 激活无人机
    uav.activate()
    
    # 设置2倍速
    uav.set_speed_multiplier(2.0)
    
    # 仿真循环
    for i in range(200):
        uav.update_position(dt=0.5)
        print(f"Step {i}: {uav}")
        
        if uav.is_path_complete:
            print("Path completed!")
            break
        
        time.sleep(uav.get_current_update_interval())
    plot_uav_path(uav)
