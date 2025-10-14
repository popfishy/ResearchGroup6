"""
Author: popfishy
Date: 2024-06-24 21:14:25
LastEditors: Please set LastEditors
LastEditTime: 2025-09-25 23:46:35
Description: 
"""

import rospy
import json
from group6_interfaces.msg import PusimKeyString, Target, Targets, Plan, AgentData, AgentsData, TimeSyn
from group6_interfaces.srv import DronePlans, DronePlansResponse, DronePlansRequest

# from rospy_message_converter import json_message_converter
from WGS84toCartesian import PositionConvert
import socket


# global variable
last_key = -1
agent_id_map = {}
task_start_time_mapping = {}
first_receive_flag = True

task_counter = {
    "准备": 0,
    "侦察": 0,
    "跟踪": 0
}

TASK_NAME_MAP = {
    "准备": "Prepare",
    "侦察": "Recon",
    "跟踪": "Track"
}


# Path_message
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
        status,
        order=None,
        expectedQuantity=None,
        path=None,
    ):
        self.agentId = agentId
        self.taskCode = taskCode
        self.beginTime = beginTime
        self.expectedDuration = expectedDuration
        self.velocity = velocity
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.status = status
        self.order = order if order is not None else 0
        self.expectedQuantity = expectedQuantity if expectedQuantity is not None else None
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
            plan_entry["status"],
            order=plan_entry.get("order"),
            expectedQuantity=plan_entry.get("expectedQuantity"),
            path=targetpoints,
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

# 服务请求
class JsonReassembler_srv:
    def __init__(self):
        self.data_segments = {}
        self.total_size = 0
        self.received_data = ""

        self.time_syn_msg = TimeSyn()
        rospy.Subscriber("/ResearchGroup5Result", PusimKeyString, self.callback, queue_size=10)
        self.time_synchronization_pub = rospy.Publisher("/TimeSyn", TimeSyn, queue_size=1)
        self.agents_plan_client = rospy.ServiceProxy("/AgentsData", DronePlans)
        rospy.wait_for_service("/AgentsData")

        print("开始发布时间戳")
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.time_synchronization_pub.publish(self.time_syn_msg)
            rate.sleep()

    def callback(self, data):
        # 处理接收到的消息
        index = data.index
        size = data.size
        if not self.total_size:
            self.total_size = size

        self.data_segments[index] = data.data

        # 检查是否所有数据都已接收
        # print(f"index = {index}, len(self.data_segments) = {len(self.data_segments)},self.total_size = {self.total_size}")
        if len(self.data_segments) == (self.total_size):
            try:
                sorted_segments = [self.data_segments[i] for i in sorted(self.data_segments)]
                self.received_data = "".join(sorted_segments)
                received_json = json.loads(self.received_data)

                planned_result = received_json.get("Planned_result", [])
                if not planned_result:
                    task_name = "Unknown"
                else:
                    task_name = planned_result[0].get("task_name", "Unknown")

                if isinstance(task_name, str):
                    pass
                else:
                    task_name = "Unknown"
                
                if task_name not in TASK_NAME_MAP:
                    rospy.logwarn(f"Unknown task_name: {task_name}, using 'Unknown'")
                    task_key = "Unknown"
                    filename_prefix = "Unknown"
                else:
                    task_key = task_name
                    filename_prefix = TASK_NAME_MAP[task_name]
                
                # 增加计数
                global task_counter
                task_counter[task_key] += 1
                count = task_counter[task_key]

                filename = f"../json/ResearchGroup5Result_{filename_prefix}_{count}.json"
                # with open(filename, "w", encoding="utf-8") as file:
                #     json.dump(received_json, file, indent=4, ensure_ascii=False)
                rospy.loginfo(f"JSON data saved to {filename} (task: {task_name})")

                agents_plan = self.parse_agent_plan_from_data(received_json)
                self.reassemble_data_then_client(agents_plan)

            except ValueError:
                rospy.logerr("Failed to parse JSON data")
            except Exception as e:
                rospy.logerr(f"Unexpected error during JSON processing: {e}")


            self.data_segments.clear()  # 清除缓存
            self.received_data = ""  # 清空重组数据
            self.total_size = 0

    def reassemble_data_then_client(self, agents_plan=None):
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
                plan_msg.expectedQuantity = plan.expectedQuantity
                plan_msg.velocity = plan.velocity
                plan_msg.order = plan.order
                plan_msg.status = plan.status
                plan_msg.taskCode = plan.taskCode
                # TODO
                plan_msg.taskPhase = ''
                plan_msg.latitude, plan_msg.longitude, plan_msg.altitude = PC.WGS84toENU(
                    plan.latitude, plan.longitude, plan.altitude
                )
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
        resp = self.agents_plan_client.call(agents_msg)
        if resp.success:
            rospy.loginfo(f"成功完成数据解析")
        else:
            rospy.logerr(f"数据解析失败")

    # 按照agent的id排序，再对agent下的任务开始时间排序
    def data_sort(self, agents_plan):
        agents_plan = sorted(agents_plan, key=lambda x: x.agentId)
        for i in range(len(agents_plan)):
            agents_plan[i].plans = sorted(agents_plan[i].plans, key=lambda x: x.beginTime)
        return agents_plan

    def data_id_map(self, agents_plan):
        global last_key, agent_id_map
        for i in range(len(agents_plan)):
            agent_id = agents_plan[i].agentId
            if agent_id not in agent_id_map.keys():
                last_key += 1
                new_key = last_key
                agent_id_map[agent_id] = new_key
        # 生成txt文件
        with open("../map/agent_id_map.txt", "w") as file:
            for key, value in agent_id_map.items():
                file.write(f"{key}: {value}\n")
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
        # 将映射表写入txt文件
        with open("../map/task_start_time_map.txt", "w") as file:
            for task_code, start_time in task_start_time_mapping.items():
                file.write(f"TaskCode: {task_code}, StartTime: {start_time}\n")
        rospy.loginfo(f"任务映射表为: {task_start_time_mapping}")

    # 解析JSON文件为AgentPlan实例的方法
    def parse_agent_plan(self, data):
        """Parse JSON data into AgentPlan instances."""
        agents_plan = []
        plans = Task.all_from_json(data)
        # 不存在无人机编号为-1
        last_id = plans[0].agentId
        last_index = 0
        for i in range(len(plans)):
            now_id = plans[i].agentId
            if (now_id != last_id) & (i != (len(plans) - 1)):
                agents_plan.append(AgentPlan(last_id, plans[last_index:i]))
                last_id = now_id
                last_index = i
            if (now_id != last_id) & (i == (len(plans) - 1)):
                agents_plan.append(AgentPlan(last_id, plans[last_index:i]))
                agents_plan.append(AgentPlan(now_id, plans[i:]))
            if (now_id == last_id) & (i == (len(plans) - 1)):
                agents_plan.append(AgentPlan(last_id, plans[i:]))
        return agents_plan

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

    def parse_agent_plan_from_data(self, data):
        global first_receive_flag
        if first_receive_flag:
            ros_timestamp = rospy.Time.now().to_sec()
            company_timestamp = float(data["timestamp"])
            self.time_syn_msg.company_timestamp = company_timestamp
            self.time_syn_msg.ros_timestamp = ros_timestamp
            first_receive_flag = False
        agent_plans = self.parse_agent_plan(data)  # 解析数据
        return agent_plans


if __name__ == "__main__":
    rospy.init_node("json_reassembler_node", anonymous=True)
    reassembler = JsonReassembler_srv()
    rospy.spin()
