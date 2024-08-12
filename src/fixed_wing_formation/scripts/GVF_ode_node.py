import rospy
from group6_interfaces.msg import PusimKeyString, Target, Targets, AgentData, AgentsData
from geometry_msgs.msg import PoseStamped
from group6_interfaces.srv import DronePlans, DronePlansResponse, DronePlansRequest
from GVF_ode import GVF_ode
import numpy as np

# Global variable
agent_id_map = {}  # 格式为{0：103015，1: 103019}
task_start_time_mapping = {}

class Targetpoint:
    def __init__(self, x, y, z, timestep, velocity):
        self.x = x
        self.y = y
        self.z = z
        self.timestep = timestep
        self.velocity = velocity


class Task:
    def __init__(
        self,
        agentId,
        beginTime,
        expectedDuration,
        expectedQuantity,
        order,
        status,
        taskCode,
        taskPhase,
        latitude,
        longitude,
        altitude,
        path=None,
    ):
        self.agentId = agentId
        self.beginTime = beginTime
        self.expectedDuration = expectedDuration
        self.expectedQuantity = expectedQuantity
        self.order = order
        self.status = status
        self.taskCode = taskCode
        self.taskPhase = taskPhase
        self.x = latitude
        self.y = longitude
        self.z = altitude
        self.path = path if path is not None else []

    @classmethod
    def from_dict(cls, plan_entry):
        """Helper to create an AgentPlan instance from a single plan dictionary."""
        targetpoints = [cls._parse_waypoint(wp) for wp in plan_entry["path"]]
        return cls(
            plan_entry["agentId"],
            plan_entry["beginTime"],
            plan_entry["expectedDuration"],
            plan_entry["expectedQuantity"],
            plan_entry["order"],
            plan_entry["status"],
            plan_entry["taskCode"],
            plan_entry["taskPhase"],
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


class GVF_ode_node:
    def __init__(self):
        self.uav_num = None
        self.agent_id = 0
        self.agent_plans = []
        self.trajectory_list = []
        self.pub_set = []
        self.gvf_ode_set = []
        # self.subscriber = rospy.Subscriber("/AgentsData", AgentsData, self.callback)
        server = rospy.Service("/AgentsData", DronePlans, self.callback)
        while self.uav_num == None:
            continue
        x_coords, y_coords = self.generate_drone_positions(self.uav_num, 100, 100)
        self.gvf_ode = GVF_ode(self.agent_plans[0].agentId, self.uav_num, x_coords, y_coords)

        # 根据映射表驱动任务运行
        global agent_id_map, task_start_time_mapping

        
        



        # for agent_plan in self.agent_plans:
        #     pub = rospy.Publisher(
        #         self.gvf_ode.uav_type + "_" + str(agent_plan.agentId) + "/mavros/GVF_ode/pose",
        #         PoseStamped,
        #         queue_size=1,
        #     )
        for i in range(self.uav_num):
            pub = rospy.Publisher(
                self.gvf_ode.uav_type + "_" + str(i) + "/mavros/GVF_ode/pose",
                PoseStamped,
                queue_size=1,
            )
            self.pub_set.append(pub)
        # for targetpoint in self.agent_plans[0].path.targets:
        #     # TODO time不确定
        #     self.trajectory_list.append([targetpoint.x, targetpoint.y/10, -targetpoint.z, 50])
        #     print(self.trajectory_list)
        self.trajectory_list = [[0, 0, 0, 0], [1000, 1000, 50, 50], [1000, 2000, 100, 100], [0, 3000, 50, 50]]
        # print(self.trajectory_list)
        self.gvf_ode.update_waypoint(self.trajectory_list)
        self.gvf_ode.calculate_path()
        global_paths = self.gvf_ode.global_paths
        # 设置发布频率为10Hz
        rate = rospy.Rate(4)

        numpoints = 313
        for i in range(len(self.agent_plans)):
            p = global_paths[i]
            for j in range(numpoints):
                for k in range(self.uav_num):
                    target = PoseStamped()
                    target.pose.position.x = p[j, k * 5]
                    target.pose.position.y = p[j, k * 5 + 1]
                    target.pose.position.z = p[j, k * 5 + 2]
                    self.pub_set[k].publish(target)
                rospy.sleep(0.25)

    def callback(self, agents_msg):
        print("接收到数据")
        # self.uav_num = len(agents_msg.agents_data)
        self.uav_num = 7
        for agent_msg in agents_msg.agents_data:
            
            agent_plan = AgentPlan(agent_msg.agentId, agent_msg.plans)
            self.agent_plans.append(agent_plan)
        resp = DronePlansResponse()
        resp.success.data = True
        return resp

    def generate_pub_set(self,agent_id_map):
        pass

    def generate_gvf_ode_set(self,agent_plans):
        pass
    

    def generate_drone_positions(self, num_drones, x_dis, y_dis):
        x_coords = []
        y_coords = []

        rows = int(np.ceil(num_drones**0.5))  # 确定方阵的行数
        cols = (num_drones + rows - 1) // rows  # 确定方阵的列数

        for i in range(rows):
            for j in range(cols):
                if len(x_coords) < num_drones:
                    x_coord = j * x_dis
                    y_coord = i * y_dis
                    x_coords.append(x_coord)
                    y_coords.append(y_coord)

        return x_coords, y_coords


if __name__ == "__main__":
    rospy.init_node("GVF_ode_node", anonymous=True)
    # 读取id映射表，并根据映射表进行分组
    with open('../../communication/map/agent_id_map.txt', 'r') as file:
        for line in file:
            line = line.strip()
            if line:
                key, value = line.split(': ')
                agent_id_map[int(key)] = int(value)
            
    with open('../../communication/map/task_start_time_map.txt', 'r') as file:
        for line in file:
            line = line.strip()
            if line:
                key_value_pairs = line.split(', ')
                task_code = key_value_pairs[0].split(': ')[1]
                start_time = key_value_pairs[1].split(': ')[1]
                task_start_time_mapping[task_code] = float(start_time)

    print(agent_id_map)
    print(task_start_time_mapping)
    gvf_ode_node = GVF_ode_node()
    rospy.spin()
