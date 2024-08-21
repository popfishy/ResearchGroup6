"""
Author: popfishy
Date: 2024-08-21 9:14:25
LastEditors: Please set LastEditors
LastEditTime: 2024-06-24 23:46:35
Description: 
"""

import rospy
import json
from group6_interfaces.msg import PusimKeyString, Target, Targets, Plan, AgentData, AgentsData, TimeSyn
from group6_interfaces.srv import DronePlans, DronePlansResponse, DronePlansRequest

# from rospy_message_converter import json_message_converter
from WGS84toCartesian import PositionConvert

last_key = -1
agent_id_map = {}
task_start_time_mapping = {}

class Targetpoint:
    def __init__(self, latitude, longitude, altitude, timestep, velocity):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.timestep = timestep
        self.velocity = velocity


class Task:
    def __init__(
        self,
        agentId,
        taskCode,
        beginTime,
        expectedDuration,
        velocity,
        latitude,
        longitude,
        altitude,
        path=None,
    ):
        self.agentId = agentId
        self.beginTime = beginTime
        self.expectedDuration = expectedDuration
        self.taskCode = taskCode
        self.velocity = velocity
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.path = path if path is not None else []

    @classmethod
    def from_dict(cls, plan_entry):
        """Helper to create an AgentPlan instance from a single plan dictionary."""
        targetpoints = [cls._parse_waypoint(wp) for wp in plan_entry["path"]]
        return cls(
            plan_entry["agentId"],
            plan_entry["task_flag"],
            plan_entry["beginTime"],
            plan_entry["expectedDuration"],
            plan_entry["velocity"],
            plan_entry["latitude"],
            plan_entry["longitude"],
            plan_entry["altitude"],
            targetpoints,
        )

    @staticmethod
    def _parse_waypoint(wp_data):
        """Helper static method to create a Waypoint instance from dict data."""
        return Targetpoint(**wp_data)

    @classmethod
    def all_from_json(cls, json_data):
        """Parses JSON data and returns a list of AgentPlan instances."""
        planned_results = json_data.get("Planned_result", [])
        return [cls.from_dict(plan_entry) for plan_entry in planned_results]

    @classmethod
    def to_drone_plan_list(cls, agent_plans):
        """Converts list of AgentPlan instances to DronePlan list."""
        return [{"id": plan.agentId, "longitude": wp.longitude} for plan in agent_plans for wp in plan.path]


class AgentPlan:
    def __init__(self, agentId, plans=None):
        self.agentId = agentId
        self.plans = plans if plans is not None else []


class PathsVisualization:
    def __init__(self):
        self.agent_plans = []
        self.pub_set = []
        self.gvf_ode_set = {}
        self.qgc_param_setter = None
        self.agent_id_map = {}
        self.task_start_time_mapping = {}

    def data_parse(self):
        global task_start_time_mapping, agent_id_map

        agents_plan = self.parse_agent_plan_from_file("../json/test.json")
        # 排序
        agents_plan = self.data_sort(agents_plan)

         # 生成映射表
        self.data_id_map(agents_plan)
        self.data_task_map(agents_plan)


        # 定义参考点的GPS坐标
        lat_ref = agents_plan[0].plans[0].latitude  # 参考点纬度
        lon_ref = agents_plan[0].plans[0].longitude  # 参考点经度
        alt_ref = 0  # 参考点高度（海拔）
        # TODO 暂时无原点，使用第一架无人机的第一个航点的经纬度作为原点
        PC = PositionConvert(lat_ref, lon_ref, alt_ref)
        # 发布消息
        agents_msg = DronePlansRequest()
        for agent_plan in agents_plan:
            agent_msg = AgentData()
            for plan in agent_plan.plans:
                plan_msg = Plan()
                agent_msg.agentId = plan.agentId
                plan_msg.beginTime = plan.beginTime
                plan_msg.expectedDuration = plan.expectedDuration
                # TODO
                plan_msg.expectedQuantity = 0
                plan_msg.order = 0
                plan_msg.status = 0
                plan_msg.taskPhase = ""
                plan_msg.latitude, plan_msg.longitude, plan_msg.altitude = PC.WGS84toENU(
                    plan.latitude, plan.longitude, plan.altitude
                )
                plan_msg.taskCode = plan.taskCode
                for target_point in plan.path:
                    target_msg = Target()
                    # 定义目标点的GPS坐标
                    lat = target_point.latitude  # 目标点纬度
                    lon = target_point.longitude  # 目标点经度
                    alt = target_point.altitude  # 目标点高度（海拔）
                    target_msg.x, target_msg.y, target_msg.z = PC.WGS84toENU(lat, lon, alt)
                    # print(f"GPS_TO_XYZ: {target_msg.x}, {target_msg.y}, {target_msg.z}")
                    target_msg.timestep = target_point.timestep
                    target_msg.velocity = target_point.velocity
                    plan_msg.targets.append(target_msg)
                agent_msg.plans.append(plan_msg)
            agents_msg.agents_data.append(agent_msg)
          
        for key, value in task_start_time_mapping.items():
            trajectory_list = []
            first_id_of_task = value[1][0]
            for plan in agents_plan[first_id_of_task].plans:
                last_timestamp = plan.beginTime
                if plan.taskCode == key:
                    for targetpoint in plan.targets:
                        now_timestamp = targetpoint.timestep
                        # TODO 因为课题五的错误数据
                        time_expectation = (now_timestamp - last_timestamp) if now_timestamp > last_timestamp else 10
                        trajectory_list.append([targetpoint.x, targetpoint.y, targetpoint.z, time_expectation])
                        last_timestamp = now_timestamp

        # 读取id映射表，并根据映射表进行分组
    def data_id_map(self, agents_plan):
        global last_key, agent_id_map
        for i in range(len(agents_plan)):
            agent_id = agents_plan[i].agentId
            if agent_id not in agent_id_map.keys():
                last_key += 1
                new_key = last_key
                agent_id_map[agent_id] = new_key
        rospy.loginfo(f"ID映射表为: {agent_id_map}")

    def data_task_map(self, agents_plan):
        global task_start_time_mapping, agent_id_map
        task_start_time_mapping_ = {}
        for agent_plan in agents_plan:
            for plan in agent_plan.plans:
                # 检查是否该任务类型已经在映射表中
                if plan.taskCode not in task_start_time_mapping_:
                    # 如果不在，添加该任务类型和对应的最早开始时间
                    task_start_time_mapping_[plan.taskCode] = [plan.beginTime, [agent_id_map[agent_plan.agentId]]]
                else:
                    # 如果已存在，比较并更新为更早的开始时间（如果需要）
                    if plan.beginTime < task_start_time_mapping_[plan.taskCode][0]:
                        value = task_start_time_mapping_[plan.taskCode]
                        task_start_time_mapping_[plan.taskCode] = [plan.beginTime, value[1]]
                    value = task_start_time_mapping_[plan.taskCode]
                    value[1].append(agent_id_map[agent_plan.agentId])
                    task_start_time_mapping_[plan.taskCode] = value
        sorted_mapping = sorted(task_start_time_mapping_.items(), key=lambda x: x[1][0])
        task_start_time_mapping = {k: v for k, v in sorted_mapping}
        rospy.loginfo(f"任务映射表为: {task_start_time_mapping}")


    def parse_agent_plan_from_file(self, file_path):
        global first_receive_flag
        """从指定JSON文件解析出AgentPlan实例列表"""
        with open(file_path, "r", encoding="utf-8") as file:  # 打开并读取JSON文件
            data = json.load(file)  # 加载JSON文件内容为Python对象
            if first_receive_flag:
                ros_timestamp = rospy.Time.now().to_sec()
                company_timestamp = float(data["timestamp"])
                self.time_syn_msg.company_timestamp = company_timestamp
                self.time_syn_msg.ros_timestamp = ros_timestamp
                first_receive_flag = False
            agent_plans = self.parse_agent_plan(data)  # 解析数据
            file.close()
        return agent_plans