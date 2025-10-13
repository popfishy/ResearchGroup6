#!/usr/bin/env python3
from dataclasses import dataclass


@dataclass
class Task:
    task_id: int = 0
    task_type: int = 0
    task_value: float = 100  # task reward
    start_time: float = 0  # task start time (sec)
    end_time: float = 0  # task expiry time (sec)
    duration: float = 0  # task default duration (sec)
    discount: float = 0.1  # task exponential discount (lambda)
    x: float = 0  # task position (meters)
    y: float = 0  # task position (meters)
    z: float = 0  # task position (meters)
    str_need: float = 1  # 敌方威胁
    rec_need: float = 1  # 侦察需求
    robot_need: int = 0
    meet_need_flag: float = 0  # 任务需求满足标志
    task_status: bool = True  # 任务状态，完成与否
