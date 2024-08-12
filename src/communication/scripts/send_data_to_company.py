import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
import sys
from gazebo_msgs.msg import ModelStates
from group6_interfaces.msg import PusimKeyString, PusimsKeyString
from WGS84toCartesian import PositionConvert
import tf
import json
import os
from std_msgs.msg import String

vehicle_type = sys.argv[1]
vehicle_num = int(sys.argv[2])
multi_pose_pub = [None]*vehicle_num
multi_speed_pub = [None]*vehicle_num
multi_camera_pose_pub = [None]*vehicle_num
multi_local_pose = [PoseStamped() for i in range(vehicle_num)]
multi_speed = [Vector3Stamped() for i in range(vehicle_num)]
multi_path = []
# rate = rospy.Rate(1) 
uav_num = None

flag = True



def gazebo_data_callback(msg):
    global uav_num, flag, multi_path

    uav_num = len(msg.pose)
    # print(uav_num)
    while uav_num == None:
        continue
    for vehicle_id in range(vehicle_num):
        # 获取gazebo中每架无人机的位姿真值
        id = msg.name.index(vehicle_type+'_'+str(vehicle_id))
        multi_local_pose[vehicle_id].header.stamp = rospy.Time().now()
        multi_local_pose[vehicle_id].header.frame_id = 'map'
        multi_local_pose[vehicle_id].pose = msg.pose[id]  # 位姿

        # 将四元数转化为欧拉角
        quaternion = (
            multi_local_pose[vehicle_id].pose.orientation.x,
            multi_local_pose[vehicle_id].pose.orientation.y,
            multi_local_pose[vehicle_id].pose.orientation.z,
            multi_local_pose[vehicle_id].pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        multi_local_pose[vehicle_id].pose.orientation.x = euler[0]
        multi_local_pose[vehicle_id].pose.orientation.y = euler[1]
        multi_local_pose[vehicle_id].pose.orientation.z = euler[2]

        # 将xyz转化为经纬高
        PC = PositionConvert(0, 0, 0)
        x = multi_local_pose[vehicle_id].pose.position.x
        y = multi_local_pose[vehicle_id].pose.position.y
        z = multi_local_pose[vehicle_id].pose.position.z
        # print(multi_local_pose[vehicle_id].pose)
        lat, lon, alt = PC.XYZtoWGS84(x, y, z)
        # print("-------------------------------------------------")
        multi_local_pose[vehicle_id].pose.position.x = lat
        multi_local_pose[vehicle_id].pose.position.y = lon
        multi_local_pose[vehicle_id].pose.position.z = alt

        if multi_local_pose[vehicle_id].pose.position.x != None:
            msgTojson(multi_local_pose)
            multi_path = []
        # rospy.loginfo(multi_local_pose[vehicle_id].pose)
    # rospy.loginfo("1")

def msgTojson(data):

    global flag

    # 存储所有数据的字典
    poses_data = {}

    key = "SwarmTrajectoryResults"
    name = "ResearchGroup6"
    timestamp = 50

    poses_data["key"] = key
    poses_data["name"] = name
    poses_data["timestamp"] = timestamp

    for i in range(vehicle_num):
        dic = {}
        dic["agentId"] = i
        dic["velocity"] = 60
        dic["altitude"] = data[i].pose.position.z
        dic["latitude"] = data[i].pose.position.x
        dic["longitude"] = data[i].pose.position.y
        dic["yaw"] = data[i].pose.orientation.z
        dic["pitch"] = data[i].pose.orientation.y
        dic["roll"] = data[i].pose.orientation.x
        multi_path.append(dic)

    poses_data["agents"] = multi_path
    # print(poses_data)
    save_data_to_json(poses_data, "ResearchGroup6Result.json")

def save_data_to_json(data, filename):
    # 将数据转换为JSON字符串
    json_str = json.dumps(data, indent=4)
    # print(json_str)
    # 将JSON字符串写入文件
    with open(filename, 'w') as file:
        # file.truncate()
        file.write(json_str)

def jsonTosting(filename):
    with open(filename, 'r', encoding='utf-8') as file:
        json_data = file.read()
    # 计算字符串长度
    print(type(json_data))
    length_of_string = len(json_data)
    print(f"The length of the JSON string is: {length_of_string}")
    # 字符串每450字节分割
    chunk_size = 450
    chunks = [json_data[i:i + chunk_size] for i in range(0, len(json_data), chunk_size)]

    total_chunks = len(chunks)

    json_string_data = [PusimKeyString() for i in range(total_chunks)]
    pusim_key_strings_msg = PusimsKeyString()

    for i in range(total_chunks):
        pusim_key_string = PusimKeyString()
        pusim_key_string.key = "SwarmTrajectoryResults"
        pusim_key_string.index = i
        pusim_key_string.size = total_chunks
        pusim_key_string.data = chunks[i]
        # json_strings_data.PusimKeyStrings.append(json_strings_data[i])
        pusim_key_strings_msg.PusimsKeyString.append(pusim_key_string)

    return pusim_key_strings_msg

    # 计算总共有多少段
    # total_chunks = len(chunks)
    # for i in range(total_chunks):
    #     print(chunks[i])
    # print("-------------------------------------------------------+++-------")

# # # TODO 可以在这个地方获取每次发送的时间
# def doMsg(event):

#     rospy.loginfo("+++++++++++++++")
#     rospy.loginfo(event.current_real.to_sec())
#     gazebo_model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_data_callback, queue_size=1)
#     jsonTosting('ResearchGroup6Result.json')

if __name__ == '__main__':
    rospy.init_node(vehicle_type+'_get_pose_truth')
    # timer = rospy.Timer(rospy.Duration(1), doMsg)
    gazebo_model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_data_callback, queue_size=1)
    pub = rospy.Publisher("/Group6Data", PusimsKeyString, queue_size=1)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print(jsonTosting('ResearchGroup6Result.json'))
        pub.publish(jsonTosting('ResearchGroup6Result.json'))
        rate.sleep()  # 等待直到下一次迭代
    rospy.spin()