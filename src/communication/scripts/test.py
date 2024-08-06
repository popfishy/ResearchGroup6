import json
from WGS84toCartesian import PositionConvert


# Path_message
class Waypoint:
    def __init__(self, altitude, latitude, longitude, timestep, velocity):
        self.altitude = altitude
        self.latitude = latitude
        self.longitude = longitude
        self.timestep = timestep
        self.velocity = velocity


# Agent_message
class AgentPlan:
    def __init__(self, agentId, beginTime, expectedDuration, taskCode, path):
        self.agentId = agentId
        self.beginTime = beginTime
        self.expectedDuration = expectedDuration
        self.taskCode = taskCode
        self.path = path  # List of Waypoint objects


# 解析JSON文件为AgentPlan实例的方法
def parse_agent_plan(data):
    """Parse JSON data into AgentPlan instances."""
    agent_plans = []
    for plan_entry in data:
        waypoints = [
            Waypoint(**wp) for wp in plan_entry["path"]
        ]  # 将每个航点数据转换为Waypoint对象
        agent_plan = AgentPlan(
            plan_entry["agentId"],
            plan_entry["beginTime"],
            plan_entry["expectedDuration"],
            plan_entry["taskCode"],
            waypoints,
        )
        agent_plans.append(agent_plan)  # 将创建的AgentPlan对象添加到列表中
    return agent_plans


def parse_agent_plan_from_file(file_path):
    """从指定JSON文件解析出AgentPlan实例列表"""
    with open(file_path, "r", encoding="utf-8") as file:  # 打开并读取JSON文件
        data = json.load(file)  # 加载JSON文件内容为Python对象
        planned_results = data.get("Planned_result", [])  # 获取"Planned_result"部分数据
        agent_plans = parse_agent_plan(planned_results)  # 解析数据
    return agent_plans


# Given JSON string
json_data_file = "ResearchGroup5Result.json"
agent_plans = parse_agent_plan_from_file(json_data_file)

# # 假设您已经有了如上代码解析得到的agent_plans列表
# for agent_plan in agent_plans:
#     print(f"无人机ID: {agent_plan.agentId}")
#     print("航点路径:")
#     for waypoint in agent_plan.path:
#         print(f"  - 经度: {waypoint.longitude}, 纬度: {waypoint.latitude}")

PC = PositionConvert()
# 如果只需要第一个无人机的第一个航点的经纬度
if agent_plans:  # 确保列表不为空
    first_drone = agent_plans[0]  # 获取第一个无人机的计划
    if first_drone.path:  # 确保路径不为空
        first_waypoint = first_drone.path[1]  # 获取第一个航点
        print(
            f"第一个无人机的第一个航点经纬度: 经度={first_waypoint.longitude}, 纬度={first_waypoint.latitude}, 高度={first_waypoint.altitude}"
        )
        x_new, y_new, z_new = PC.GPStoXYZ(
            first_waypoint.latitude,
            first_waypoint.longitude,
            first_waypoint.altitude,
            24.4434194,
            119.6982324,
            0,
        )
        print(f"GPS_TO_XYZ: {x_new}, {y_new}, {z_new}")
