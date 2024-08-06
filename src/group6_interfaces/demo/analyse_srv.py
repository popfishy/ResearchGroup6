#!/usr/bin/env python

import rospy
from group_srv.srv import DronePlans, DronePlansResponse  # Replace 'your_package' with your actual package name
from group_msg.msg import EntitySrv, Multipath, Waypoint  # Replace with the actual ROS message package and type
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
        waypoints = [cls._parse_waypoint(wp) for wp in plan_entry['path']]
        return cls(
            plan_entry['agentId'],
            plan_entry['beginTime'],
            plan_entry['expectedDuration'],
            plan_entry['taskCode'],
            waypoints
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

    @classmethod
    def to_drone_plan_list(cls, agent_plans):
        """Converts list of AgentPlan instances to DronePlan list."""
        return [{"id": plan.agentId, "longitude": wp.longitude} for plan in agent_plans for wp in plan.path]

# def handle_drone_plans_request(req):
#     """Service callback function."""
#     path_srv = EntitySrv()
#     response = DronePlansResponse()
#     response.way_point_path = []
#     vehicle_num = len(all_agent_plans)
#     for i in range(vehicle_num):
#         path_msg = Multipath()
#         path_srv.agentId = all_agent_plans[i].agentId
#         path_srv.beginTime = all_agent_plans[i].beginTime
#         path_srv.durationTime = all_agent_plans[i].expectedDuration
#         path_msg.path = [Waypoint(altitude=wp.altitude, 
#                                       latitude=wp.latitude, 
#                                       longitude=wp.longitude, 
#                                       timestep=wp.timestep, 
#                                       velocity=wp.velocity) 
#                              for wp in all_agent_plans[i].path]
#         path_srv.paths = path_msg
#         path_srv.taskCode = all_agent_plans[i].taskCode
#         response.way_point_path.append(path_srv)
#     # response.drone_plans = AgentPlan.to_drone_plan_list(all_agent_plans)
#     return response

def handle_drone_plans_request(req):
    """Service callback function."""
    response = DronePlansResponse()
    response.way_point_path = []  # Initialize the response list

    for plan in all_agent_plans:
        path_srv = EntitySrv()  # Create a new EntitySrv instance for each plan
        path_srv.agentId = plan.agentId
        path_srv.beginTime = plan.beginTime
        path_srv.durationTime = plan.expectedDuration
        
        # Convert Waypoint list to Multipath
        multipath = Multipath()
        multipath.path = [Waypoint(altitude=wp.altitude, 
                                   latitude=wp.latitude, 
                                   longitude=wp.longitude, 
                                   timestep=wp.timestep, 
                                   velocity=wp.velocity) 
                          for wp in plan.path]
        path_srv.paths = multipath
        
        path_srv.taskCode = plan.taskCode
        response.way_point_path.append(path_srv)  # Append the instance to the response list

    return response


if __name__ == '__main__':
    rospy.init_node('drone_plan_service')
    json_data_file = '/home/zyh/Group6Data/src/analyseData/json/ResearchGroup5Result.json'
    
    try:
        with open(json_data_file, 'r', encoding='utf-8') as file:
            json_data = json.load(file)
            all_agent_plans = AgentPlan.all_from_json(json_data)
            
        service = rospy.Service('get_drone_plans', DronePlans, handle_drone_plans_request)
        rospy.loginfo("Drone plans service is ready.")
        rospy.spin()
    except FileNotFoundError:
        rospy.logerr(f"File {json_data_file} not found.")
    except json.JSONDecodeError as e:
        rospy.logerr(f"Error decoding JSON file: {e}")