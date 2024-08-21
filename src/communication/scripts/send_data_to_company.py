import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
import sys
from gazebo_msgs.msg import ModelStates
from pusim_msgs.msg import PusimKeyString, PusimsKeyString
from WGS84toCartesian import PositionConvert
import tf
import json
from group6_interfaces.msg import TimeSyn
import threading

vehicle_type = sys.argv[1]
vehicle_num = int(sys.argv[2])
multi_pose_pub = [None] * vehicle_num
multi_speed_pub = [None] * vehicle_num
multi_camera_pose_pub = [None] * vehicle_num
multi_local_pose = [PoseStamped() for i in range(vehicle_num)]
multi_speed = [TwistStamped() for i in range(vehicle_num)]
multi_path = []
agent_id_map = {}

uav_num = None
flag = True
company_timestamp = 0
ros_timestamp = 0
poses_data = {}
# 创建一个锁对象
poses_data_lock = threading.Lock()


def time_synchronization_callback(time_syn_msg):
    global company_timestamp, ros_timestamp
    company_timestamp = time_syn_msg.company_timestamp
    ros_timestamp = time_syn_msg.ros_timestamp
    rospy.loginfo(f"company_timestamp: {company_timestamp}")
    if company_timestamp != 0:
        time_synchronization_sub.unregister()


def gazebo_data_callback(msg):
    global uav_num, flag, multi_path

    uav_num = len(msg.pose)
    # print(uav_num)
    while uav_num == None:
        continue
    for vehicle_id in range(vehicle_num):
        # 获取gazebo中每架无人机的位姿真值
        id = msg.name.index(vehicle_type + "_" + str(vehicle_id))
        multi_local_pose[vehicle_id].header.stamp = rospy.Time().now()
        multi_local_pose[vehicle_id].header.frame_id = "map"
        multi_local_pose[vehicle_id].pose = msg.pose[id]  # 位姿
        multi_speed[vehicle_id].twist = msg.twist[id]

        # 将四元数转化为欧拉角
        quaternion = (
            multi_local_pose[vehicle_id].pose.orientation.x,
            multi_local_pose[vehicle_id].pose.orientation.y,
            multi_local_pose[vehicle_id].pose.orientation.z,
            multi_local_pose[vehicle_id].pose.orientation.w,
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        multi_local_pose[vehicle_id].pose.orientation.x = euler[0]
        multi_local_pose[vehicle_id].pose.orientation.y = euler[1]
        multi_local_pose[vehicle_id].pose.orientation.z = euler[2]

        # 将xyz转化为经纬高
        PC = PositionConvert(24.4819, 119.616, 0)
        x = multi_local_pose[vehicle_id].pose.position.x
        y = multi_local_pose[vehicle_id].pose.position.y
        z = multi_local_pose[vehicle_id].pose.position.z
        # print(multi_local_pose[vehicle_id].pose)
        lat, lon, alt = PC.ENUtoWGS84(x, y, z)
        # print("-------------------------------------------------")
        multi_local_pose[vehicle_id].pose.position.x = lat
        multi_local_pose[vehicle_id].pose.position.y = lon
        multi_local_pose[vehicle_id].pose.position.z = alt

        if multi_local_pose[vehicle_id].pose.position.x != None:
            msgTojson(multi_local_pose, multi_speed)
            multi_path = []


def msgTojson(multi_local_pose, multi_speed):

    global flag, company_timestamp, ros_timestamp, agent_id_map, poses_data

    key = "SwarmTrajectoryResults"
    name = "ResearchGroup6"
    timestamp = company_timestamp + (rospy.Time.now().to_sec() - ros_timestamp)


    poses_data_lock.acquire()
    try:
        poses_data["key"] = key
        poses_data["name"] = name
        poses_data["timestamp"] = timestamp

        for i in range(vehicle_num):
            dic = {}
            dic["agentId"] = agent_id_map[i]
            # 速度存在坐标系方向问题
            dic["velocity_x"] = multi_speed[i].twist.linear.x
            dic["velocity_y"] = multi_speed[i].twist.linear.y
            dic["velocity_z"] = multi_speed[i].twist.linear.z
            dic["latitude"] = multi_local_pose[i].pose.position.x
            dic["longitude"] = multi_local_pose[i].pose.position.y
            dic["altitude"] = multi_local_pose[i].pose.position.z
            # 飞机机体坐标系
            dic["roll"] = multi_local_pose[i].pose.orientation.x
            dic["pitch"] = multi_local_pose[i].pose.orientation.y
            dic["yaw"] = multi_local_pose[i].pose.orientation.z
            multi_path.append(dic)

        poses_data["agents"] = multi_path
    finally:
        poses_data_lock.release()
    save_data_to_json(poses_data, "../json/ResearchGroup6Result.json")


def save_data_to_json(data, filename):
    # 将数据转换为JSON字符串
    json_str = json.dumps(data, indent=4)
    # print(json_str)
    # 将JSON字符串写入文件
    with open(filename, "w") as file:
        # file.truncate()
        file.write(json_str)


def jsonTosting(filename = None):
    global poses_data
    poses_data_lock.acquire()
    try:
        json_data = json.dumps(poses_data, indent=4)
    finally:
        poses_data_lock.release()

    if filename != None:
        with open(filename, "r", encoding="utf-8") as file:
            json_data = file.read()
    
    # 计算字符串长度
    length_of_string = len(json_data)
    # 字符串每450字节分割
    chunk_size = 450
    chunks = [json_data[i : i + chunk_size] for i in range(0, len(json_data), chunk_size)]

    total_chunks = len(chunks)

    json_string_data = [PusimKeyString() for i in range(total_chunks)]

    for k in range(total_chunks):
        pusim_key_string = PusimKeyString()
        pusim_key_string.key = "SwarmTrajectoryResults"
        pusim_key_string.index = k
        pusim_key_string.size = total_chunks
        pusim_key_string.data = chunks[k]
        json_string_data[k] = pusim_key_string

    return json_string_data


def get_mapping_table():
    global agent_id_map
    agent_id_map_old = {}
    with open("../../communication/map/agent_id_map.txt", "r") as file:
        for line in file:
            line = line.strip()
            if line:
                key, value = line.split(": ")
                agent_id_map_old[int(key)] = int(value)
    for key, value in agent_id_map_old.items():
        agent_id_map[value] = key


if __name__ == "__main__":
    get_mapping_table()
    rospy.init_node(vehicle_type + "_get_pose_truth")
    gazebo_model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_data_callback, queue_size=1)
    time_synchronization_sub = rospy.Subscriber("/TimeSyn", TimeSyn, time_synchronization_callback)
    pub = rospy.Publisher("/ResearchGroup6Result", PusimKeyString, queue_size=20)

    # rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        json_string_data = jsonTosting()
        if company_timestamp != 0:
            for data in json_string_data:
                pub.publish(data)
                rospy.sleep(0.1)
            rospy.loginfo_once("成功给公司发布数据")
        # rate.sleep()  # 等待直到下一次迭代
    rospy.spin()
