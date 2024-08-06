#!/usr/bin/env python

import rospy
from group_msg.msg import EntityData, Multipath, Waypoint  # Replace with the actual ROS message package and type
import json


class Waypoint:
    def __init__(self, altitude, latitude, longitude, timestep, velocity):
        self.altitude = altitude
        self.latitude = latitude
        self.longitude = longitude
        self.timestep = timestep
        self.velocity = velocity


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
        waypoints = [cls._parse_waypoint(wp) for wp in plan_entry["path"]]
        return cls(
            plan_entry["agentId"],
            plan_entry["beginTime"],
            plan_entry["expectedDuration"],
            plan_entry["taskCode"],
            waypoints,
        )

    @staticmethod
    def _parse_waypoint(wp_data):
        """Helper static method to create a Waypoint instance from dict data."""
        return Waypoint(**wp_data)

    @classmethod
    def all_from_json(cls, json_data):
        """Parses JSON data and returns a list of AgentPlan instances."""
        planned_results = json_data.get("Planned_result", [])
        return [cls.from_dict(plan_entry) for plan_entry in planned_results]


def main():
    rospy.init_node("drone_plan_publisher")

    json_data_file = "/home/zyh/Group6Data/src/analyseData/json/ResearchGroup5Result.json"
    try:
        with open(json_data_file, "r", encoding="utf-8") as file:
            # 导入数据包
            json_data = json.load(file)
            all_agent_plans = AgentPlan.all_from_json(json_data)
            vehicle_num = len(all_agent_plans)

            # 定义每架无人机的发布对象
            # agent_plan_pub = [None] * len(all_agent_plans)
            # for i in range(vehicle_num):
            #     agent_plan_pub[i] = rospy.Publisher('/Vehicle_' + str(i) + '/trace_point', EntityData, queue_size=10)
            # rate = rospy.Rate(20)
            agent_plan_pub = rospy.Publisher("/Vehicle/trace_point", EntityData, queue_size=1)
            rate = rospy.Rate(20)
            # 处理每架无人机要发布的信息
            while not rospy.is_shutdown():
                ros_msg = EntityData()

                # Publish each agent plan as a ROS message
                for i in range(vehicle_num):
                    path_msg = Multipath()
                    ros_msg.agentId.append(all_agent_plans[i].agentId)
                    ros_msg.beginTime.append(all_agent_plans[i].beginTime)
                    ros_msg.durationTime.append(all_agent_plans[i].expectedDuration)
                    # 注意路径点处理可能需要调整，取决于你的数据结构
                    path_msg.path = [
                        Waypoint(
                            altitude=wp.altitude,
                            latitude=wp.latitude,
                            longitude=wp.longitude,
                            timestep=wp.timestep,
                            velocity=wp.velocity,
                        )
                        for wp in all_agent_plans[i].path
                    ]
                    ros_msg.paths.append(path_msg)
                    ros_msg.taskCode.append(all_agent_plans[i].taskCode)
                agent_plan_pub.publish(ros_msg)

                try:
                    rospy.loginfo("无人机的路径信息", ros_msg.paths[0].path[0].longitude)
                    rate.sleep()
                except:
                    continue

    except FileNotFoundError:
        rospy.logerr(f"文件 {json_data_file} 未找到。")
    except json.JSONDecodeError as e:
        rospy.logerr(f"解析JSON文件时发生错误: {e}")


if __name__ == "__main__":
    main()
