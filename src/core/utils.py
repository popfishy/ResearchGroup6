import numpy as np
from enum import Enum
from typing import List, Tuple, Optional, Dict
import time
from dataclasses import dataclass

class UAVStatus(Enum):
    ACTIVE = "active"
    DESTROYED = "destroyed"
    RETURNING = "returning"
    STANDBY = "standby"
    CIRCLE = "circle"

@dataclass 
class PathPoint:
    """路径点"""
    x: float
    y: float
    z: float
    heading: Optional[float] = None
    speed: float = 30.0
    timestamp: Optional[float] = None

@dataclass
class FormationConfig:
    """队形配置"""
    name: str
    formation_type: str  # "line", "wedge", "diamond", "circle"
    positions: List[Tuple[float, float]]  # 相对位置偏移
    leader_index: int = 0  # 队形领机索引
    spacing: float = 100.0  # 队形间距

@dataclass
class FormationPathResult:
    """队形路径规划结果"""
    leader_path: List[PathPoint]
    follower_paths: Dict[int, List[PathPoint]]  # uav_id -> path
    formation_config: FormationConfig
    total_distance: float
    estimated_time: float

class MissionType(Enum):
    PREPARATION = "preparation"
    RECONNAISSANCE = "reconnaissance"
    ATTACK = "attack"
    CONTAINMENT = "containment"
    IDLE = "idle"

class TargetStatus(Enum):
    UNDETECTED = "undetected"
    DETECTED = "detected"
    CONFIRMED = "confirmed"
    ATTACKED = "attacked"
    DESTROYED = "destroyed"

class ZoneType(Enum):
    SEARCH_AREA = "search_area"
    ATTACK_ZONE = "attack_zone"
    CONTAINMENT_ZONE = "containment_zone"
    NO_FLY_ZONE = "no_fly_zone"

@dataclass
class TargetArea:
    """目标区域"""
    id: int
    center: Tuple[float, float, float]
    radius: float
    priority: int  # 1-10, 10为最高优先级
    area_type: ZoneType
    assigned_uavs: List[int] = None  # 分配的无人机ID列表
    search_complete: bool = False

@dataclass
class EnemyTarget:
    """敌方目标"""
    id: int
    position: Tuple[float, float, float]
    target_type: str  # "vehicle", "building", "personnel"
    threat_level: int  # 1-5, 5为最高威胁
    status: TargetStatus = TargetStatus.UNDETECTED
    detection_time: Optional[float] = None
    assigned_uav_id: Optional[int] = None
    estimated_value: float = 1.0  # 目标价值评估

@dataclass
class ContainmentZone:
    """封控区域"""
    id: int
    center: Tuple[float, float, float]
    radius: float
    patrol_altitude: float
    patrol_speed: float
    assigned_uavs: List[int] = None