# 项目开发者指南

本文档帮助新成员快速理解项目结构、代码架构和开发规范。

---

## 1. 项目概述

### 1.1 项目目标

实现一个基于OODA（Observe-Orient-Decide-Act）循环的无人机集群协同作战仿真系统：

- **准备阶段**: 无人机按编队分组
- **侦查阶段**: 对目标区域进行覆盖式侦查
- **打击阶段**: 对侦测到的目标实施攻击
- **封控阶段**: 对区域进行封控巡逻

### 1.2 技术栈

- **ROS Noetic**: 机器人操作系统
- **Python 3**: 主要开发语言
- **PX4 + Gazebo**: 无人机仿真
- **fields2cover**: 区域覆盖路径规划
- **dubins**: Dubins曲线路径规划

---

## 2. 代码结构导览

### 2.1 目录结构

```
src/
├── core/                    # ⭐ 核心业务逻辑
│   ├── scenario_parser.py   # XML场景解析 → UAV和EnemyTarget对象
│   ├── path_planner.py       # 路径规划器接口
│   ├── uav.py               # UAV实体类（状态、位置、任务）
│   └── utils.py            # 数据类型定义（枚举、数据类）
│
├── region2cover/            # ⭐ 区域规划算法
│   ├── region_cover.py      # 区域覆盖路径（fields2cover）
│   ├── region_isolation.py  # 封控区域划分
│   └── circle_path.py        # 圆形巡逻路径
│
├── communication/           # 外部通信接口
│   └── scripts/
│       ├── tcp_client_communication.py  # TCP通信
│       ├── receive_company_data.py     # 接收公司数据
│       ├── send_data_to_company.py     # 发送数据到公司
│       └── WGS84toCartesian.py         # 坐标系转换
│
├── fixed_wing_formation/     # 固定翼控制
├── xfd_allocation/           # 目标分配算法（第三方库，不修改）
│
├── start_simulation.py       # ⭐ 主仿真入口
└── evaluate.py               # 性能指标计算
```

### 2.2 核心模块依赖图

```
                    ┌─────────────────┐
                    │ start_simulation│
                    │   (主入口)      │
                    └────────┬────────┘
                             │
              ┌──────────────┼──────────────┐
              ▼              ▼              ▼
      ┌───────────┐  ┌───────────┐  ┌───────────┐
      │ scenario_  │  │  evaluate │  │communication│
      │ parser     │  │           │  │           │
      └─────┬─────┘  └───────────┘  └─────┬─────┘
            │                            │
            ▼                            ▼
      ┌───────────┐              ┌───────────┐
      │    UAV    │◄─────────────│TCPClient  │
      │ (实体类)  │              │           │
      └─────┬─────┘              └───────────┘
            │
     ┌──────┼──────┐
     ▼      ▼      ▼
┌────────┐ ┌─────┐ ┌────────────┐
│Path    │ │Region│ │ xfd_      │
│Planner │ │Cover│ │ allocation│
└────────┘ └─────┘ └────────────┘
```

---

## 3. 核心概念

### 3.1 UAV类（core/uav.py）

UAV是系统的核心实体，每架无人机对应一个UAV对象。

**关键属性**:
```python
uav.id              # 无人机ID
uav.position         # ENU坐标系位置 [x, y, z]
uav.velocity         # 速度向量 [vx, vy, vz]
uav.heading          # 航向角 (rad)
uav.status           # UAVStatus.ACTIVE/DESTROYED/STANDBY
uav.current_mission  # MissionType.PREPARATION/RECONNAISSANCE/ATTACK/CONTAINMENT
uav.is_leader        # 是否为编队领航者
uav.planned_path     # 规划好的路径点列表
```

**关键方法**:
- `update_position(dt)`: 根据当前任务更新位置（核心仿真逻辑）
- `set_as_leader()` / `set_as_follower()`: 设置编队角色
- `set_attack_target()`: 设置攻击目标并规划攻击路径
- `set_patrol_assignment()`: 设置封控巡逻任务

### 3.2 任务类型（MissionType）

| 阶段 | 枚举值 | UAV行为 |
|------|--------|--------|
| 准备 | PREPARATION | 按编队集结 |
| 侦查 | RECONNAISSANCE | 区域覆盖巡逻 |
| 打击 | ATTACK | 飞向目标并摧毁 |
| 封控 | CONTAINMENT | 圆形巡逻 |

### 3.3 位置更新模式

UAV的位置更新根据任务类型分发到不同的 `_update_*` 方法：

```python
def update_position(self, dt):
    if self.current_mission == ATTACK:
        self._update_attack_position(dt)
    elif self.current_mission == CONTAINMENT:
        self._update_containment_position(dt)
    elif self.is_leader:
        self._update_leader_position(dt)
    else:
        self._update_follower_position(dt)
```

---

## 4. 开发规范

### 4.1 代码优化原则

1. **不修改第三方库**: `xfd_allocation/scripts/CBPA/lib/` 和 `VINS-Fusion/` 保持原样
2. **适度优化**: 统一命名规范、消除重复代码、添加必要注释
3. **保持向后兼容**: 修改接口需确保调用方不受影响

### 4.2 模块边界

| 模块 | 职责 | 修改权限 |
|------|------|----------|
| core/ | 核心业务逻辑 | ✅ 可优化 |
| region2cover/ | 区域规划算法 | ✅ 可优化 |
| communication/ | 外部通信 | ⚠️ 谨慎修改 |
| fixed_wing_formation/ | 飞控接口 | ⚠️ 谨慎修改 |
| xfd_allocation/ | 分配算法 | ❌ 不修改 |

### 4.3 数据类型定义位置

- **枚举类型**: `core/utils.py` (UAVStatus, MissionType, TargetStatus, ZoneType)
- **数据类**: `core/utils.py` (PathPoint, EnemyTarget, TargetArea, ContainmentZone)
- **UAV实体**: `core/uav.py`

---

## 5. 调试技巧

### 5.1 本地可视化调试

```python
# start_simulation.py
VISUAL_DEBUG_MODE = True  # 启用Matplotlib可视化
```

### 5.2 ROS话题查看

```bash
# 查看所有话题
rostopic list

# 查看无人机位置话题
rostopic echo /uav_position

# 查看公司通信话题
rostopic echo /ResearchGroup6Result
```

### 5.3 常用命令

```bash
# 编译项目
cd ~/ResearchGroup6
catkin build

# 运行仿真
source devel/setup.bash
rosrun group6_bringup start.sh

# 单独运行仿真脚本
python src/start_simulation.py
```

---

## 6. 常见问题

### 6.1 坐标系混淆

- **WGS84**: 经纬度坐标系，用于与公司通信
- **ENU**: 东北天坐标系，MAVROS内部使用
- **NED**: 北东地坐标系，PX4飞控使用

**转换链**: WGS84 ↔ ENU ↔ NED

### 6.2 仿真频率

- 仿真步长: `SIMULATION_DT = 4` 秒（4Hz）
- TCP发布频率: `TCP_PUBLISH_RATE = 4.0` Hz
- 公司接收上限: 4Hz（公司端限制）

### 6.3 XML场景格式

参考 `src/core/xml/` 下的场景文件，格式为自定义XML。

---

## 7. 下一步

1. 阅读 `docs/INTERFACES.md` 了解详细接口定义
2. 阅读 `src/start_simulation.py` 了解主仿真流程
3. 阅读 `src/core/uav.py` 了解UAV实体实现
4. 查看现有XML场景文件了解数据格式
