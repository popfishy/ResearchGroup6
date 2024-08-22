import rospy
from group6_interfaces.msg import TimeSyn
from geometry_msgs.msg import PoseStamped
from group6_interfaces.srv import DronePlans, DronePlansResponse, DronePlansRequest
from GVF_ode import GVF_ode
from QGC_param_set import QGCParamSetter
import numpy as np
import threading

# Global variable
agent_id_map = {}  # 格式为{0：103015，1: 103019}
task_start_time_mapping = {}
FW_AIRSPD_TRIM = 15  # 空速


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
        taskCode,
        beginTime,
        expectedDuration,
        expectedQuantity,
        velocity,
        latitude,
        longitude,
        altitude,
        order,
        status,
        path=None,
    ):
        self.agentId = agentId
        self.beginTime = beginTime
        self.expectedDuration = expectedDuration
        self.expectedQuantity = expectedQuantity
        self.taskCode = taskCode
        self.velocity = velocity
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.order = order
        self.status = status
        self.path = path if path is not None else []


class AgentPlan:
    def __init__(self, agentId, plans=None):
        self.agentId = agentId
        self.plans = plans if plans is not None else []


class GVF_ode_node:
    def __init__(self):
        self.agent_plans = []
        self.trajectory_list = []
        self.pub_set = []
        self.gvf_ode_set = {}
        self.qgc_param_setter = None

        self.uav_type = "plane"
        self.ros_timestamp = 0
        self.company_timestamp = 0
        self.ros_timenow = rospy.Time.now().to_sec()
        self.time_synchronization_sub = rospy.Subscriber("/TimeSyn", TimeSyn, self.time_synchronization_callback)
        service = rospy.Service("/AgentsData", DronePlans, self.callback)

    def callback(self, agents_msg):
        print("接收到数据")
        for agent_msg in agents_msg.agents_data:
            agent_plan = AgentPlan(agent_msg.agentId, agent_msg.plans)
            self.agent_plans.append(agent_plan)
            print(agent_msg.plans)
        resp = DronePlansResponse()
        resp.success.data = True

        # 初始化任务
        id_map, task_map = self.get_mapping_table()
        # 初始化QGC参数
        # self.init_qgc_params(self.uav_type, len(id_map))

        # 生成无人机位置发布器
        self.generate_pub_set(id_map)

        # 生成GVF_ode序列，计算轨迹
        start_time = rospy.Time.now().to_sec()
        self.generate_gvf_ode_set(task_map, self.agent_plans)
        end_time = rospy.Time.now().to_sec()
        print(f"路径生成时间为：{end_time - start_time}，初始化完毕。")

        while self.company_timestamp == 0:
            continue
        print("开始执行任务")
        self.start_perform_tasks(id_map, task_map, self.agent_plans)

        return resp

    def time_synchronization_callback(self, time_syn_msg):
        self.company_timestamp = time_syn_msg.company_timestamp
        self.ros_timestamp = time_syn_msg.ros_timestamp
        rospy.loginfo(f"company_timestamp: {self.company_timestamp}")
        if self.company_timestamp != 0:
            self.time_synchronization_sub.unregister()

    def init_qgc_params(self, uav_type, vehicle_num):
        self.qgc_param_setter = QGCParamSetter(uav_type, vehicle_num)
        # 空速
        self.qgc_param_setter.set_float_param("FW_AIRSPD_MAX", 80.0)
        self.qgc_param_setter.set_float_param("FW_AIRSPD_TRIM", 60.0)
        # 油门
        self.qgc_param_setter.set_float_param("FW_THR_MAX", 100.0)
        self.qgc_param_setter.set_float_param("FW_THR_CRUISE", 80.0)
        # GPS used
        self.qgc_param_setter.set_int_param("EKF2_AID_MASK", 1)
        # Barometer used for hight measurement
        self.qgc_param_setter.set_int_param("EKF2_HGT_MODE", 0)
        self.qgc_param_setter.set_int_param("NAV_DLL_ACT", 0)
        self.qgc_param_setter.set_int_param("NAV_RCL_ACT", 0)
        self.qgc_param_setter.set_int_param("COM_RCL_EXCEPT", 4)

    def generate_pub_set(self, id_map):
        for pub in self.pub_set:
            pub.unregister()
        self.pub_set = []
        for key, value in id_map.items():
            pub = rospy.Publisher(
                self.uav_type + "_" + str(value) + "/mavros/GVF_ode/pose",
                PoseStamped,
                queue_size=1,
            )
            self.pub_set.append(pub)

    def generate_gvf_ode_set(self, task_map, agent_plans):
        self.gvf_ode_set = {}
        for key, value in task_map.items():
            trajectory_list = []
            uav_num = len(value[1])
            # TODO 速度缩放因子对无人机编队队形有影响，需对输出进行处理
            x_coords, y_coords = self.generate_drone_positions(uav_num, 100, 100)
            gvf_ode = GVF_ode(value[1], uav_num, x_coords, y_coords)
            first_id_of_task = value[1][0]
            for plan in agent_plans[first_id_of_task].plans:
                last_timestamp = plan.beginTime
                # TODO 速度缩放因子，解决速度无法达到预期速度的问题
                velocity_scaling = FW_AIRSPD_TRIM / plan.velocity
                if plan.taskCode == key:
                    for targetpoint in plan.targets:
                        now_timestamp = targetpoint.timestep
                        # TODO 课题五可能存在错误数据
                        time_expectation = (now_timestamp - last_timestamp) if now_timestamp > last_timestamp else 10
                        trajectory_list.append(
                            [
                                targetpoint.x,
                                targetpoint.y,
                                targetpoint.z,
                                time_expectation,
                            ]
                        )
                        last_timestamp = now_timestamp
            gvf_ode.update_waypoint(trajectory_list)
            gvf_ode.calculate_path()
            self.gvf_ode_set[key] = gvf_ode

    # 读取id映射表，并根据映射表进行分组
    def get_mapping_table(self):
        global agent_id_map, task_start_time_mapping
        with open("../../communication/map/agent_id_map.txt", "r") as file:
            for line in file:
                line = line.strip()
                if line:
                    key, value = line.split(": ")
                    agent_id_map[int(key)] = int(value)

        with open("../../communication/map/task_start_time_map.txt", "r") as file:
            lines = file.readlines()
            for line in lines:
                task_code = line.split("TaskCode: ")[1].split(",")[0].strip()
                start_time = eval(line.split("StartTime: ")[1].strip())
                task_start_time_mapping[task_code] = start_time
        return agent_id_map, task_start_time_mapping

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

    # 开始执行任务
    def start_perform_tasks(self, id_map, task_map, agent_plans):
        for key, value in task_map.items():
            begin_time = value[0]
            uav_ids = value[1]
            task_code = key
            for plan in agent_plans[uav_ids[0]].plans:
                if plan.taskCode == task_code:
                    task_begin_pose = PoseStamped()
                    task_begin_pose.pose.position.x = plan.latitude
                    task_begin_pose.pose.position.y = plan.longitude
                    task_begin_pose.pose.position.z = plan.altitude
                    break
            if task_begin_pose is None:
                rospy.logwarn(f"No valid plan found for task: {task_code}")
                continue

            gvf_ode = self.gvf_ode_set[task_code]
            for i, uav_id in enumerate(uav_ids):
                uav_pose = PoseStamped()
                uav_pose.pose.position.x = task_begin_pose.pose.position.x + gvf_ode.x_coords[i]
                uav_pose.pose.position.y = task_begin_pose.pose.position.y + gvf_ode.y_coords[i]
                uav_pose.pose.position.z = task_begin_pose.pose.position.z
                self.pub_set[uav_id].publish(uav_pose)
                rospy.loginfo(
                    f"无人机{uav_id}的当前任务{task_code}的初始点: {uav_pose.pose.position.x},{uav_pose.pose.position.y},{uav_pose.pose.position.z}"
                )
            rospy.loginfo(f"任务: {task_code}的开始时间为: {begin_time}")
            while (self.ros_timenow - self.ros_timestamp) < (begin_time - self.company_timestamp):
                rospy.sleep(0.5)
                self.ros_timenow = rospy.Time.now().to_sec()
            self.ros_timenow = rospy.Time.now().to_sec()
            rospy.loginfo(
                f"开始执行任务: {task_code}，当前仿真公司时间为：{self.company_timestamp + self.ros_timenow - self.ros_timestamp}"
            )

            # 启动线程发布目标
            # TODO：同一时间可以存在多个线程，会存在任务冲突的情形
            publish_thread = threading.Thread(target=self.publish_targets, args=(gvf_ode, uav_ids))
            publish_thread.start()

            # TODO：运行完毕后才会执行下一个任务
            # self.publish_targets(gvf_ode, uav_ids)

    def publish_targets(self, gvf_ode, uav_ids):
        for i in range(len(gvf_ode.trajectory_list) - 1):
            p = gvf_ode.global_paths[i]
            # TODO：发布的目标点的数量
            numpoints = p.shape[0]
            for j in range(numpoints):
                for k in range(len(uav_ids)):
                    target = PoseStamped()
                    target.pose.position.x = p[j, k * 5]
                    target.pose.position.y = p[j, k * 5 + 1]
                    target.pose.position.z = p[j, k * 5 + 2]
                    self.pub_set[uav_ids[k]].publish(target)
                # TODO 加入目标点判断机制
                rospy.sleep(0.4)


if __name__ == "__main__":
    rospy.init_node("GVF_ode_node", anonymous=True)
    gvf_ode_node = GVF_ode_node()
    rospy.spin()
