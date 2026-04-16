# 模块接口文档

本文档描述项目各模块之间的接口关系、数据格式和ROS话题。

---

## 1. 项目模块结构

```
src/
├── core/                    # 核心模块
│   ├── scenario_parser.py   # XML场景解析
│   ├── path_planner.py      # 路径规划器
│   ├── uav.py               # UAV实体类
│   └── utils.py              # 数据类型和工具函数
│
├── region2cover/            # 区域覆盖模块
│   ├── region_cover.py       # 基于fields2cover的区域覆盖
│   ├── region_isolation.py   # 封控区域划分
│   ├── circle_path.py        # 圆形巡逻路径生成
│   └── dubins_generate.py    # Dubins路径生成（第三方示例）
│
├── communication/           # 通信模块
│   └── scripts/
│       ├── tcp_client_communication.py  # TCP客户端
│       ├── receive_company_data.py      # 接收公司数据
│       ├── send_data_to_company.py      # 发送数据到公司
│       └── WGS84toCartesian.py          # 坐标转换
│
├── fixed_wing_formation/     # 固定翼无人机控制
│   └── scripts/
│       ├── GVF_ode.py               # 编队控制
│       └── plane_communication.py   # 通信
│
├── xfd_allocation/           # 目标分配算法（第三方库，不修改）
│   └── scripts/CBPA/lib/     # CBPA算法实现
│
├── start_simulation.py       # 主仿真入口
└── evaluate.py               # 性能指标评估
```

---

## 2. 核心数据类型

### 2.1 枚举类型（core/utils.py）

```python
class UAVStatus(Enum):
    ACTIVE = "active"      # 活动中
    DESTROYED = "destroyed" # 已摧毁
    STANDBY = "standby"     # 待机

class MissionType(Enum):
    UNSET = "unset"
    PREPARATION = "preparation"     # 准备阶段
    RECONNAISSANCE = "reconnaissance" # 侦查阶段
    ATTACK = "attack"             # 打击阶段
    CONTAINMENT = "containment"   # 封控阶段

class TargetStatus(Enum):
    UNDETECTED = "undetected"
    DETECTED = "detected"
    CONFIRMED = "confirmed"
    ATTACKED = "attacked"
    DESTROYED = "destroyed"

class ZoneType(Enum):
    SEARCH_AREA = "search_area"       # 搜索区域
    ATTACK_ZONE = "attack_zone"       # 攻击区域
    CONTAINMENT_ZONE = "containment_zone" # 封控区域
    NO_FLY_ZONE = "no_fly_zone"       # 禁飞区
```

### 2.2 数据类（core/utils.py）

```python
@dataclass
class PathPoint:
    x: float
    y: float
    z: float
    heading: Optional[float] = None
    speed: float = 30.0
    timestamp: Optional[float] = None

@dataclass
class EnemyTarget:
    id: int
    position: Tuple[float, float, float]  # ENU坐标系 (x, y, z)
    target_type: str   # "vehicle", "building", "personnel"
    threat_level: int  # 1-4, 4为最高威胁
    robot_need: int   # 需要多少UAV进行攻击
    status: TargetStatus = TargetStatus.UNDETECTED
    detection_time: Optional[float] = None
    assigned_uav_id: Optional[int] = None

@dataclass
class TargetArea:
    id: int
    center: Tuple[float, float, float]
    width: float
    height: float
    priority: int = 1
    area_type: ZoneType = ZoneType.SEARCH_AREA
    assigned_uavs: List[int] = None
    search_complete: bool = False

@dataclass
class ContainmentZone:
    id: int
    center: Tuple[float, float, float]
    width: float
    height: float
    patrol_altitude: float
    patrol_speed: float
    assigned_uavs: List[int] = None
```

---

## 3. 核心类接口

### 3.1 UAV类（core/uav.py）

无人机实体类，管理单架无人机的状态和运动。

```python
class UAV:
    # ========== 初始化 ==========
    def __init__(self, uav_id: int, initial_position: Tuple[float, float, float],
                 initial_heading: float = 0.0)

    # ========== 状态属性 ==========
    id: int                           # 无人机ID
    position: np.ndarray              # ENU坐标系位置 [x, y, z]
    velocity: np.ndarray              # 速度向量 [vx, vy, vz]
    heading: float                    # 航向角 (rad)
    status: UAVStatus                 # 状态
    current_mission: MissionType      # 当前任务类型

    # ========== 编队控制 ==========
    def set_as_leader() -> None
    def set_as_follower(leader: 'UAV', offset: Tuple[float, float]) -> None
    def set_formation_target(target_pos, target_heading) -> None
    def unset_formation_roles() -> None

    # ========== 任务设置 ==========
    def set_waypoints(waypoints: List[Tuple[float, float, float]]) -> None
    def set_attack_target(target_position: np.ndarray, use_dubins: bool = False) -> None
    def set_patrol_assignment(center, radius, altitude) -> None

    # ========== 仿真更新 ==========
    def update_position(dt: float) -> None  # 根据任务类型更新位置
    def activate() -> None
    def destroy() -> None

    # ========== 辅助方法 ==========
    def get_position_for_simulation() -> Tuple[float, float, float]
```

### 3.2 PathPlanner类（core/path_planner.py）

路径规划器，管理Dubins路径生成。

```python
class PathPlanner:
    def __init__(self, dubins_planner=None, coverage_planner=None, patrol_planner=None)

    # 设置外部规划器
    def set_coverage_planner(coverage_planner) -> None
    def set_patrol_planner(patrol_planner) -> None

    # 规划路径
    def plan_leader_path(start_pos, waypoints) -> List[np.ndarray]
```

### 3.3 ScenarioParser类（core/scenario_parser.py）

解析XML格式的战场态势文件。

```python
class ScenarioParser:
    def __init__(self, xml_file_path: str)
    def parse() -> Dict[str, Any]
        # 返回: {'uavs': [UAV], 'enemies': [EnemyTarget], 'uav_groups': {group_id: [uav_ids]}}
```

### 3.4 RegionCover类（region2cover/region_cover.py）

基于fields2cover的区域覆盖路径规划。

```python
class RegionCover:
    def __init__(self, num_rows, num_cols, pos1, pos2, pos3, pos4, is_plot: bool = True)

    def divide_regions() -> List[Tuple]  # 划分区域
    def pathcheck(path) -> List          # 路径点去重
    def cover_run(uav_velocity, turning_radius, log_time, cov_width=200) -> None
    def plot_path(id, cells, no_hl, path_dubins_cc, swaths, log_time) -> None
```

### 3.5 generate_circles_in_rectangle（region2cover/region_isolation.py）

在矩形区域内生成圆形封控区域。

```python
def generate_circles_in_rectangle(
    circle_num,           # 圆的数量
    rect_width,           # 矩形宽度
    rect_height,          # 矩形高度
    rect_center_x,        # 矩形中心x坐标
    rect_center_y,        # 矩形中心y坐标
    min_circle_radius=100,
    max_circle_radius=500
) -> Tuple[float, List[Tuple]]  # (circle_radius, circle_centers)
```

---

## 4. OODA各阶段流程

### 4.1 准备阶段（Preparation）

```
输入: XML场景文件
处理:
  1. ScenarioParser解析XML，获取UAV列表和敌方目标
  2. 根据编队要求将UAV分组
  3. 设置领航者-跟随者关系
输出: 分组后的UAV编队
```

### 4.2 侦查阶段（Reconnaissance）

```
输入: 侦查区域列表
处理:
  1. RegionCover对每个侦查区域生成覆盖路径
  2. UAV沿路径执行覆盖式侦查
  3. 识别到目标后更新EnemyTarget状态为DETECTED
输出: 目标列表（带位置和状态）
```

### 4.3 打击阶段（Attack）

```
输入: 识别到的敌方目标列表
处理:
  1. CBPA目标分配算法为每个目标分配UAV
  2. UAV.set_attack_target()设置攻击目标
  3. UAV使用Dubins路径或直线攻击目标
  4. 到达目标后UAV.destroy()
输出: 打击结果（目标摧毁状态）
```

### 4.4 封控阶段（Containment）

```
输入: 封控区域列表
处理:
  1. generate_circles_in_rectangle生成巡逻圆心
  2. 对每个封控区域设置UAV巡逻任务
  3. UAV执行圆形巡逻路径
输出: 封控状态
```

---

## 5. 坐标系说明

### 5.1 坐标系转换

```
WGS84 (经纬度)  <--->  ENU (东北天)  <--->  NED (北东地)
                    MAVROS内部           PX4飞控
```

### 5.2 坐标转换工具（communication/scripts/WGS84toCartesian.py）

```python
class PositionConvert:
    def __init__(self, ref_lon, ref_lat, ref_alt=0)
    def WGS84toENU(lon, lat, alt) -> Tuple[float, float, float]
    def ENUtoWGS84(e, n, u) -> Tuple[float, float, float]
```

---

## 6. 主仿真流程（start_simulation.py）

```python
class SwarmMissionManager:
    def run(self):
        # 1. 初始化ROS节点
        # 2. 解析XML场景
        # 3. 创建UAV对象
        # 4. 进入OODA循环:
        while not all_phases_complete:
            if current_phase == PREPARATION:
                self._execute_preparation()
            elif current_phase == RECONNAISSANCE:
                self._execute_reconnaissance()
            elif current_phase == ATTACK:
                self._execute_attack()
            elif current_phase == CONTAINMENT:
                self._execute_containment()

            # 更新所有UAV位置
            for uav in self.uav_dict.values():
                uav.update_position(SIMULATION_DT)

            # 发送数据到公司
            self._publish_to_company()

        # 评估性能指标
        self._evaluate_metrics()
```

---

## 7. 性能指标

评估模块（evaluate.py）计算以下指标：

| 指标 | 描述 |
|------|------|
| O1 | 识别效率 = 目标识别时间 / 任务总时间 |
| D1 | 规划效率 = 任务规划时间 / 任务总时间 |
| A1 | 打击效率 = 成功打击目标数 / 总目标数 |
| C1 | 封控效率 = 封控区域内停留时间 / 任务总时间 |
