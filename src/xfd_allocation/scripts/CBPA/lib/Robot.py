#!/usr/bin/env python3
from dataclasses import dataclass
from turtle import heading


@dataclass
class Robot:
    robot_id: int = 0
    robot_type: int = 0
    availability: float = 0  # robot availability (expected time in sec)
    # clr_plotting: list = field([0, 0, 0]) # for plotting
    x: float = 0  # task position (meters)
    y: float = 0  # task position (meters)
    z: float = 0  # task position (meters)
    rx: float = 0
    ry: float = 0
    rz: float = 0
    rw: float = 0
    heading: float = 0
    nom_velocity: float = 0  # robot cruise velocity (m/s)
    fuel: float = 0  # robot fuel penalty (per meter)
    str_capability: float = 0  # 机器人打击能力
    rec_capability: float = 0  # 机器人侦察能力
    robot_status: bool = True  # 机器人状态（损毁与否）
