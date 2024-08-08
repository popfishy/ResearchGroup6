import rospy
from group6_interfaces.msg import PusimKeyString, Target, Targets, AgentData, AgentsData
from geometry_msgs.msg import PoseStamped
from group6_interfaces.srv import DronePlans, DronePlansResponse, DronePlansRequest
from GVF_ode import GVF_ode


class Targetpoint:
    def __init__(self, x, y, z, timestep, velocity):
        self.x = x
        self.y = y
        self.z = z
        self.timestep = timestep
        self.velocity = velocity


# Agent_message
class AgentPlan:
    def __init__(self, agentId, beginTime, expectedDuration, taskCode, path=None):
        self.agentId = agentId
        self.beginTime = beginTime
        self.expectedDuration = expectedDuration
        self.taskCode = taskCode
        self.path = path if path is not None else []

    @classmethod
    def from_dict(cls, plan_entry):
        """Helper to create an AgentPlan instance from a single plan dictionary."""
        targetpoints = [cls._parse_waypoint(wp) for wp in plan_entry["path"]]
        return cls(
            plan_entry["agentId"],
            plan_entry["beginTime"],
            plan_entry["expectedDuration"],
            plan_entry["taskCode"],
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


class GVF_ode_node:
    def __init__(self):
        self.uav_num = None
        self.agent_id = 0
        self.agent_plans = []
        self.trajectory_list = []
        self.pub_set = []
        # self.subscriber = rospy.Subscriber("/AgentsData", AgentsData, self.callback)
        server = rospy.Service("/AgentsData", DronePlans, self.callback)
        while self.uav_num == None:
            continue

        self.gvf_ode = GVF_ode(self.agent_plans[0].agentId, self.uav_num)
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
        print(self.trajectory_list)
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

    # 发布订阅的写法
    # def callback(self, agents_msg):
    #     print("接收到数据")
    #     # self.uav_num = len(agents_msg.agents_data)
    #     self.uav_num = 10
    #     for agent_msg in agents_msg.agents_data:
    #         agent_plan = AgentPlan(
    #             agent_msg.agentId,
    #             agent_msg.beginTime,
    #             agent_msg.durationTime,
    #             agent_msg.taskCode,
    #             agent_msg.targets,
    #         )
    #         self.agent_plans.append(agent_plan)

    def callback(self, agents_msg):
        print("接收到数据")
        # self.uav_num = len(agents_msg.agents_data)
        self.uav_num = 10
        for agent_msg in agents_msg.agents_data:
            agent_plan = AgentPlan(
                agent_msg.agentId,
                agent_msg.beginTime,
                agent_msg.durationTime,
                agent_msg.taskCode,
                agent_msg.targets,
            )
            self.agent_plans.append(agent_plan)
        rospy.loginfo(agents_msg.agents_data[0].agentId)
        rospy.loginfo(agents_msg.agents_data[0].beginTime)
        resp = DronePlansResponse()
        resp.success.data = True
        rospy.loginfo(type(resp.success))
        return resp


if __name__ == "__main__":
    rospy.init_node("GVF_ode_node", anonymous=True)
    gvf_ode_node = GVF_ode_node()
    rospy.spin()
