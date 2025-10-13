#!/usr/bin/env python3
import random
import numpy as np
import math
from CBPA.lib.Robot import Robot
from CBPA.lib.Task import Task
from CBPA.lib.WorldInfo import WorldInfo

# 实验二：CBBA
def create_robots_and_tasks_exp2(num_robots: int, num_tasks: int, WorldInfoInput: WorldInfo, config_data):
    """
    Generate robots and tasks based on a json configuration file.

    config_data:
        config_file_name = "config.json"
        json_file = open(config_file_name)
        config_data = json.load(json_file)

    """

    # Create some default robots 默认元素

    # rec_robot
    robot_rec_default = Robot()
    # robot type
    robot_rec_default.robot_type = config_data["ROBOT_TYPES"].index("rec_robot")
    # robot cruise velocity (m/s)
    robot_rec_default.nom_velocity = float(config_data["REC_DEFAULT"]["NOM_VELOCITY"])
    # # robot fuel penalty (per meter)
    # robot_quad_default.fuel = float(config_data["QUAD_DEFAULT"]["FUEL"])

    # str_robot
    robot_str_default = Robot()
    # robot type
    robot_str_default.robot_type = config_data["ROBOT_TYPES"].index("str_robot")
    # robot cruise velocity (m/s)
    robot_str_default.nom_velocity = float(config_data["STR_DEFAULT"]["NOM_VELOCITY"])
    # # robot fuel penalty (per meter)

    #My Task
    task_rec_default = Task()
    # task type
    task_rec_default.task_type = config_data["TASK_TYPES"].index("rec_task")
    # task reward 固定任务收益
    task_rec_default.task_value = float(config_data["REC_TASK_DEFAULT"]["TASK_VALUE"])
    # task start time (sec)
    task_rec_default.start_time = float(config_data["REC_TASK_DEFAULT"]["START_TIME"])
    # task expiry time (sec)
    task_rec_default.end_time = float(config_data["REC_TASK_DEFAULT"]["END_TIME"])
    # task default duration (sec)
    task_rec_default.duration = float(config_data["REC_TASK_DEFAULT"]["DURATION"])

    task_str_default = Task()
    # task type
    task_str_default.task_type = config_data["TASK_TYPES"].index("str_task")
    # task reward 固定任务收益
    task_str_default.task_value = float(config_data["STR_TASK_DEFAULT"]["TASK_VALUE"])
    # task start time (sec)
    task_str_default.start_time = float(config_data["STR_TASK_DEFAULT"]["START_TIME"])
    # task expiry time (sec)
    task_str_default.end_time = float(config_data["STR_TASK_DEFAULT"]["END_TIME"])
    # task default duration (sec)
    task_str_default.duration = float(config_data["STR_TASK_DEFAULT"]["DURATION"])


    # create empty list, each element is a dataclass Robot() or Task()
    global RobotList, TaskList
    RobotList = []
    TaskList = []
    robot_pos_x = []
    robot_pos_y = []
    task_pos_x = []
    task_pos_y = []    

    # create random robots  随机元素
    count = 0
    for idx_robot in range(0, num_robots):
        # create a new instance of dataclass robot_rec_default
        # if idx_robot/num_robots < 0.5:
        #     RobotList.append(Robot(**robot_str_default.__dict__))
        #     RobotList[idx_robot].rec_capability = 0
        #     RobotList[idx_robot].str_capability = 5
        # else:
        #     RobotList.append(Robot(**robot_rec_default.__dict__))
        #     RobotList[idx_robot].str_capability = 0
        #     RobotList[idx_robot].rec_capability = 1

        RobotList.append(Robot(**robot_str_default.__dict__))
        RobotList[idx_robot].rec_capability = 0
        RobotList[idx_robot].str_capability = 100

        #RobotList.append(Robot(**robot_quad_default.__dict__))
        RobotList[idx_robot].robot_id = idx_robot
        RobotList[idx_robot].x = random.uniform(WorldInfoInput.limit_x[0]+5, WorldInfoInput.limit_x[1]-10)
        RobotList[idx_robot].y = random.uniform(WorldInfoInput.limit_y[0]+10, WorldInfoInput.limit_y[1]-10)
        robot_pos_x.append(RobotList[idx_robot].x)
        robot_pos_y.append(RobotList[idx_robot].y)
        # RobotList[idx_robot].x = (idx_robot-(num_robots/2))*20
        # RobotList[idx_robot].y = 0
        RobotList[idx_robot].z = 0
    print("robot pose: x:",robot_pos_x,"  y:",robot_pos_y)

    # create random tasks (track only)
    for idx_task in range(0, num_tasks):
        # create a new instance of dataclass task_track_default
        # if idx_task/num_tasks < 0.5:
        #     TaskList.append(Task(**task_str_default.__dict__))
        #     TaskList[idx_task].rec_need = 0
        #     TaskList[idx_task].str_need = 1
        # else:
        #     TaskList.append(Task(**task_rec_default.__dict__))
        #     TaskList[idx_task].rec_need = 1
        #     TaskList[idx_task].str_need = 0

        TaskList.append(Task(**task_str_default.__dict__))
        TaskList[idx_task].rec_need = 0
        TaskList[idx_task].str_need = 30    

        TaskList[idx_task].task_id = idx_task
        TaskList[idx_task].x = random.uniform(WorldInfoInput.limit_x[0]+20, WorldInfoInput.limit_x[1]-10)
        TaskList[idx_task].y = random.uniform(WorldInfoInput.limit_y[0]+10, WorldInfoInput.limit_y[1]-10)
        TaskList[idx_task].z = 0
        task_pos_x.append(TaskList[idx_task].x)
        task_pos_y.append(TaskList[idx_task].y)
        

        # TaskList[idx_task].start_time = random.uniform(50, max(float(config_data["REC_TASK_DEFAULT"]["END_TIME"]),
        #                                                       float(config_data["STR_TASK_DEFAULT"]["END_TIME"])) -
        #                                                max(float(config_data["REC_TASK_DEFAULT"]["DURATION"]),
        #                                                   float(config_data["STR_TASK_DEFAULT"]["DURATION"])))
        start_time_init = [30,75,120,165,210,25,70,110,150,200]
        # TaskList[idx_task].start_time = start_time_init[idx_task]
        TaskList[idx_task].start_time = 0
        TaskList[idx_task].end_time = TaskList[idx_task].start_time + 350
    print("task pose: x:",task_pos_x,"  y:",task_pos_y)

    for n in range(num_tasks):
        print("Task " + str(n))
        # print(str(TaskList[n].x)+", "+str(TaskList[n].y)+", "+str(TaskList[n].z))
        # print(str(TaskList[n].start_time)+" --- "+str(TaskList[n].end_time))
        print("str_need: "+str(TaskList[n].str_need)+ ", rec_needed: "+str(TaskList[n].rec_need)+", X:"+str(TaskList[n].x)+", Y:"+str(TaskList[n].y)+", start_time:"+str(TaskList[n].start_time))
    for m in range(num_robots):
        print("Robot " + str(m) + " " + str(RobotList[m].robot_type))
        # print(str(RobotList[m].x)+", "+str(RobotList[m].y)+", "+str(RobotList[m].z))
        # print(RobotList[m].robot_type)
        print("str_capability: "+str(RobotList[m].str_capability)+", rec_capability: "+str(RobotList[m].rec_capability)+", X:"+str(RobotList[m].x)+", Y:"+str(RobotList[m].y))

    return RobotList, TaskList

# 固定任务位置
def create_robots_and_tasks_exp2_3(num_robots: int, num_tasks: int, WorldInfoInput: WorldInfo, config_data): 
    """
    Generate robots and tasks based on a json configuration file.

    config_data:
        config_file_name = "config.json"
        json_file = open(config_file_name)
        config_data = json.load(json_file)

    """

    # Create some default robots 默认元素

    # rec_robot
    robot_rec_default = Robot()
    # robot type
    robot_rec_default.robot_type = config_data["ROBOT_TYPES"].index("rec_robot")
    # robot cruise velocity (m/s)
    robot_rec_default.nom_velocity = float(config_data["REC_DEFAULT"]["NOM_VELOCITY"])
    # # robot fuel penalty (per meter)
    # robot_quad_default.fuel = float(config_data["QUAD_DEFAULT"]["FUEL"])

    # str_robot
    robot_str_default = Robot()
    # robot type
    robot_str_default.robot_type = config_data["ROBOT_TYPES"].index("str_robot")
    # robot cruise velocity (m/s)
    robot_str_default.nom_velocity = float(config_data["STR_DEFAULT"]["NOM_VELOCITY"])
    # # robot fuel penalty (per meter)

    #My Task
    task_rec_default = Task()
    # task type
    task_rec_default.task_type = config_data["TASK_TYPES"].index("rec_task")
    # task reward 固定任务收益
    task_rec_default.task_value = float(config_data["REC_TASK_DEFAULT"]["TASK_VALUE"])
    # task start time (sec)
    task_rec_default.start_time = float(config_data["REC_TASK_DEFAULT"]["START_TIME"])
    # task expiry time (sec)
    task_rec_default.end_time = float(config_data["REC_TASK_DEFAULT"]["END_TIME"])
    # task default duration (sec)
    task_rec_default.duration = float(config_data["REC_TASK_DEFAULT"]["DURATION"])

    task_str_default = Task()
    # task type
    task_str_default.task_type = config_data["TASK_TYPES"].index("str_task")
    # task reward 固定任务收益
    task_str_default.task_value = float(config_data["STR_TASK_DEFAULT"]["TASK_VALUE"])
    # task start time (sec)
    task_str_default.start_time = float(config_data["STR_TASK_DEFAULT"]["START_TIME"])
    # task expiry time (sec)
    task_str_default.end_time = float(config_data["STR_TASK_DEFAULT"]["END_TIME"])
    # task default duration (sec)
    task_str_default.duration = float(config_data["STR_TASK_DEFAULT"]["DURATION"])


    # create empty list, each element is a dataclass Robot() or Task()
    global RobotList, TaskList
    RobotList = []
    TaskList = []

    robot_pos_x = []
    robot_pos_y = []
    task_pos_x = []
    task_pos_y = []

    # create random robots  随机元素
    count = 0

    for idx_robot in range(0, num_robots):
        RobotList.append(Robot(**robot_str_default.__dict__))
        RobotList[idx_robot].rec_capability = 0
        RobotList[idx_robot].str_capability = 100

        #RobotList.append(Robot(**robot_quad_default.__dict__))
        RobotList[idx_robot].robot_id = idx_robot
        RobotList[idx_robot].x = random.uniform(WorldInfoInput.limit_x[0]+5, WorldInfoInput.limit_x[1]-10)
        RobotList[idx_robot].y = random.uniform(WorldInfoInput.limit_y[0]+10, WorldInfoInput.limit_y[1]-10)
        # RobotList[idx_robot].x = (idx_robot-(num_robots/2))*20
        # RobotList[idx_robot].y = 0
        RobotList[idx_robot].z = 0
          
    
    # create random tasks (track only)
    for idx_task in range(0, num_tasks):
        # create a new instance of dataclass task_track_default
        # if idx_task/num_tasks < 0.5:
        #     TaskList.append(Task(**task_str_default.__dict__))
        #     TaskList[idx_task].rec_need = 0
        #     TaskList[idx_task].str_need = 1
        # else:
        #     TaskList.append(Task(**task_rec_default.__dict__))
        #     TaskList[idx_task].rec_need = 1
        #     TaskList[idx_task].str_need = 0

        TaskList.append(Task(**task_str_default.__dict__))
        TaskList[idx_task].rec_need = 0
        TaskList[idx_task].str_need = 1    

        TaskList[idx_task].task_id = idx_task
        TaskList[idx_task].x = random.uniform(WorldInfoInput.limit_x[0]+20, WorldInfoInput.limit_x[1]-10)
        TaskList[idx_task].y = random.uniform(WorldInfoInput.limit_y[0]+10, WorldInfoInput.limit_y[1]-10)
        TaskList[idx_task].z = 0
        task_pos_x.append(TaskList[idx_task].x)
        task_pos_y.append(TaskList[idx_task].y)
        
        start_time_init = [30,75,120,165,210,25,70,110,150,200]
        # TaskList[idx_task].start_time = start_time_init[idx_task]
        TaskList[idx_task].start_time = 0
        TaskList[idx_task].end_time = TaskList[idx_task].start_time + 250
    print("task pose: x:",task_pos_x,"  y:",task_pos_y)

    for n in range(num_tasks):
        print("Task " + str(n))
        print("str_need: "+str(TaskList[n].str_need)+ ", rec_needed: "+str(TaskList[n].rec_need)+", X:"+str(TaskList[n].x)+", Y:"+str(TaskList[n].y)+", start_time:"+str(TaskList[n].start_time))
    for m in range(num_robots):
        print("Robot " + str(m) + " " + str(RobotList[m].robot_type))
        print("str_capability: "+str(RobotList[m].str_capability)+", rec_capability: "+str(RobotList[m].rec_capability)+", X:"+str(RobotList[m].x)+", Y:"+str(RobotList[m].y))

    return RobotList, TaskList

# 实验二
def create_robots_and_tasks_exp2_2(num_robots: int, num_tasks: int, WorldInfoInput: WorldInfo, config_data):
    """
    Generate robots and tasks based on a json configuration file.

    config_data:
        config_file_name = "config.json"
        json_file = open(config_file_name)
        config_data = json.load(json_file)

    """

    # Create some default robots 默认元素

    # rec_robot
    robot_rec_default = Robot()
    # robot type
    robot_rec_default.robot_type = config_data["ROBOT_TYPES"].index("rec_robot")
    # robot cruise velocity (m/s)
    robot_rec_default.nom_velocity = float(config_data["REC_DEFAULT"]["NOM_VELOCITY"])
    # # robot fuel penalty (per meter)
    # robot_quad_default.fuel = float(config_data["QUAD_DEFAULT"]["FUEL"])

    # str_robot
    robot_str_default = Robot()
    # robot type
    robot_str_default.robot_type = config_data["ROBOT_TYPES"].index("str_robot")
    # robot cruise velocity (m/s)
    robot_str_default.nom_velocity = float(config_data["STR_DEFAULT"]["NOM_VELOCITY"])
    # # robot fuel penalty (per meter)

    #My Task
    task_rec_default = Task()
    # task type
    task_rec_default.task_type = config_data["TASK_TYPES"].index("rec_task")
    # task reward 固定任务收益
    task_rec_default.task_value = float(config_data["REC_TASK_DEFAULT"]["TASK_VALUE"])
    # task start time (sec)
    task_rec_default.start_time = float(config_data["REC_TASK_DEFAULT"]["START_TIME"])
    # task expiry time (sec)
    task_rec_default.end_time = float(config_data["REC_TASK_DEFAULT"]["END_TIME"])
    # task default duration (sec)
    task_rec_default.duration = float(config_data["REC_TASK_DEFAULT"]["DURATION"])

    task_str_default = Task()
    # task type
    task_str_default.task_type = config_data["TASK_TYPES"].index("str_task")
    # task reward 固定任务收益
    task_str_default.task_value = float(config_data["STR_TASK_DEFAULT"]["TASK_VALUE"])
    # task start time (sec)
    task_str_default.start_time = float(config_data["STR_TASK_DEFAULT"]["START_TIME"])
    # task expiry time (sec)
    task_str_default.end_time = float(config_data["STR_TASK_DEFAULT"]["END_TIME"])
    # task default duration (sec)
    task_str_default.duration = float(config_data["STR_TASK_DEFAULT"]["DURATION"])


    # create empty list, each element is a dataclass Robot() or Task()
    global RobotList, TaskList
    RobotList = []
    TaskList = []

    # create random robots  随机元素
    count = 0
    for idx_robot in range(0, num_robots):
        # create a new instance of dataclass robot_rec_default
        if idx_robot/num_robots < 0.5:
            RobotList.append(Robot(**robot_rec_default.__dict__))
            RobotList[idx_robot].rec_capability = 4
            RobotList[idx_robot].str_capability = 0
        else:
            RobotList.append(Robot(**robot_str_default.__dict__))
            RobotList[idx_robot].str_capability = 4
            RobotList[idx_robot].rec_capability = 0     

        #RobotList.append(Robot(**robot_quad_default.__dict__))
        RobotList[idx_robot].robot_id = idx_robot
        RobotList[idx_robot].x = random.uniform(WorldInfoInput.limit_x[0]+5, WorldInfoInput.limit_x[1]-10)
        RobotList[idx_robot].y = random.uniform(WorldInfoInput.limit_y[0]+10, WorldInfoInput.limit_y[1]-10)
        # RobotList[idx_robot].x = (idx_robot-(num_robots/2))*20
        # RobotList[idx_robot].y = 0
        RobotList[idx_robot].z = 0
          

    # create random tasks (track only)
    # time_list = [55,83,105,238,256,297,57,126,187,245]
    for idx_task in range(0, num_tasks):
        # create a new instance of dataclass task_track_default
        if idx_task/num_tasks <= 0.5:
            TaskList.append(Task(**task_rec_default.__dict__))
            TaskList[idx_task].rec_need = 1
            TaskList[idx_task].str_need = 0
        else:
            TaskList.append(Task(**task_str_default.__dict__))
            TaskList[idx_task].rec_need = 0
            TaskList[idx_task].str_need = 1
        TaskList[idx_task].task_id = idx_task
        TaskList[idx_task].x = random.uniform(WorldInfoInput.limit_x[0]+20, WorldInfoInput.limit_x[1]-10)
        TaskList[idx_task].y = random.uniform(WorldInfoInput.limit_y[0]+10, WorldInfoInput.limit_y[1]-10)
        TaskList[idx_task].z = 0
        
        # TaskList[idx_task].start_time = random.uniform(50, max(float(config_data["REC_TASK_DEFAULT"]["END_TIME"]),
        #                                                       float(config_data["STR_TASK_DEFAULT"]["END_TIME"])) -
        #                                                max(float(config_data["REC_TASK_DEFAULT"]["DURATION"]),
        #                                                   float(config_data["STR_TASK_DEFAULT"]["DURATION"])))

        TaskList[idx_task].start_time = 0
        # if idx_task <= num_tasks/3:
        #     TaskList[idx_task].start_time = random.uniform(0, (float(config_data["TASK_DEFAULT"]["END_TIME"])-float(config_data["TASK_DEFAULT"]["DURATION"]))/3)
        # elif num_tasks/3 < idx_task <= 2*num_tasks/3:
        #     TaskList[idx_task].start_time = random.uniform((float(config_data["TASK_DEFAULT"]["END_TIME"])-float(config_data["TASK_DEFAULT"]["DURATION"]))/3, (float(config_data["TASK_DEFAULT"]["END_TIME"])-float(config_data["TASK_DEFAULT"]["DURATION"]))*2/3)
        # else:
        #     TaskList[idx_task].start_time = random.uniform((float(config_data["TASK_DEFAULT"]["END_TIME"])-float(config_data["TASK_DEFAULT"]["DURATION"]))*2/3, float(config_data["TASK_DEFAULT"]["END_TIME"])-float(config_data["TASK_DEFAULT"]["DURATION"]))
        # TaskList[idx_task].start_time = 350
        TaskList[idx_task].end_time = TaskList[idx_task].start_time + 250

    for n in range(num_tasks):
        print("Task " + str(n))
        # print(str(TaskList[n].x)+", "+str(TaskList[n].y)+", "+str(TaskList[n].z))
        # print(str(TaskList[n].start_time)+" --- "+str(TaskList[n].end_time))
        print("str_need: "+str(TaskList[n].str_need)+ ", rec_needed: "+str(TaskList[n].rec_need)+", X:"+str(TaskList[n].x)+", Y:"+str(TaskList[n].y)+", start_time:"+str(TaskList[n].start_time))
    for m in range(num_robots):
        print("Robot " + str(m) + " " + str(RobotList[m].robot_type))
        # print(str(RobotList[m].x)+", "+str(RobotList[m].y)+", "+str(RobotList[m].z))
        # print(RobotList[m].robot_type)
        print("str_capability: "+str(RobotList[m].str_capability)+", rec_capability: "+str(RobotList[m].rec_capability)+", X:"+str(RobotList[m].x)+", Y:"+str(RobotList[m].y))

    return RobotList, TaskList

# 实验二：CBTA
def create_robots_and_tasks_exp2_4(num_robots: int, num_tasks: int, WorldInfoInput: WorldInfo, config_data): # 
    """
    Generate robots and tasks based on a json configuration file.

    config_data:
        config_file_name = "config.json"
        json_file = open(config_file_name)
        config_data = json.load(json_file)

    """

    # Create some default robots 默认元素

    # rec_robot
    robot_rec_default = Robot()
    robot_rec_default.robot_type = config_data["ROBOT_TYPES"].index("rec_robot")
    robot_rec_default.nom_velocity = float(config_data["REC_DEFAULT"]["NOM_VELOCITY"])
    # str_robot
    robot_str_default = Robot()
    robot_str_default.robot_type = config_data["ROBOT_TYPES"].index("str_robot")
    robot_str_default.nom_velocity = float(config_data["STR_DEFAULT"]["NOM_VELOCITY"])
    # rs_robot
    robot_rs_default = Robot()
    robot_rs_default.robot_type = config_data["ROBOT_TYPES"].index("rs_robot")
    robot_rs_default.nom_velocity = float(config_data["RS_DEFAULT"]["NOM_VELOCITY"])

    #My Task
    task_default = Task()
    task_default.task_type = config_data["TASK_TYPES"].index("coalition_task")
    task_default.task_value = float(config_data["TASK_DEFAULT"]["TASK_VALUE"])
    task_default.start_time = float(config_data["TASK_DEFAULT"]["START_TIME"])
    task_default.end_time = float(config_data["TASK_DEFAULT"]["END_TIME"])
    task_default.duration = float(config_data["TASK_DEFAULT"]["DURATION"])


    # create empty list, each element is a dataclass Robot() or Task()
    global RobotList, TaskList
    RobotList = []
    TaskList = []
    # robot_pos = [[0, 130], [0, 330], [0, 700], [0, 970], [0, 1270]]
    #robot_pos = [[0, 200], [0, 400], [0, 600], [0, 800], [0, 1000], [0, 1200]]
    robot_pos = [[0, 100], [0, 200], [0, 300], [0, 400], [0, 500], [0, 600], [0, 700], [0, 800], [0, 900], [0, 1000], [0, 1100], [0, 1200]]
    for idx_robot in range(0, num_robots):
        if idx_robot % 2 == 0:
            RobotList.append(Robot(**robot_rec_default.__dict__))
            RobotList[idx_robot].rec_capability = 1
            RobotList[idx_robot].str_capability = 0
        else:
            RobotList.append(Robot(**robot_str_default.__dict__))
            RobotList[idx_robot].rec_capability = 0
            RobotList[idx_robot].str_capability = 100
        RobotList[idx_robot].robot_id = idx_robot
        # RobotList[idx_robot].x = random.uniform(WorldInfoInput.limit_x[0]+5, WorldInfoInput.limit_x[1]-10)
        # RobotList[idx_robot].y = random.uniform(WorldInfoInput.limit_y[0]+10, WorldInfoInput.limit_y[1]-10)
        RobotList[idx_robot].x = robot_pos[idx_robot][0]
        RobotList[idx_robot].y = robot_pos[idx_robot][1]
        RobotList[idx_robot].z = 0

    task_pos = [[150,330], [150,970]]
    for idx_task in range(0, num_tasks):
        TaskList.append(Task(**task_default.__dict__))
        TaskList[idx_task].task_id = idx_task
        # if idx_task <= 1:
        #     TaskList[idx_task].x = task_pos[idx_task][0]
        #     TaskList[idx_task].y = task_pos[idx_task][1]
        # else:
        #     TaskList[idx_task].x = random.uniform(WorldInfoInput.limit_x[0]+300, WorldInfoInput.limit_x[1]-10)
        #     TaskList[idx_task].y = random.uniform(WorldInfoInput.limit_y[0]+10, WorldInfoInput.limit_y[1]-10)

        TaskList[idx_task].x = random.uniform(WorldInfoInput.limit_x[0]+300, WorldInfoInput.limit_x[1]-10)
        TaskList[idx_task].y = random.uniform(WorldInfoInput.limit_y[0]+10, WorldInfoInput.limit_y[1]-10)
        TaskList[idx_task].z = 0
        TaskList[idx_task].rec_need = 1
        TaskList[idx_task].str_need = 30
        TaskList[idx_task].start_time = 0
        TaskList[idx_task].end_time =  500

    for n in range(num_tasks):
        print("Task " + str(n))
        print("str_need: "+str(TaskList[n].str_need)+ ", rec_needed: "+str(TaskList[n].rec_need)+", X:"+str(TaskList[n].x)+", Y:"+str(TaskList[n].y)+", start_time:"+str(TaskList[n].start_time))
    for m in range(num_robots):
        print("Robot " + str(m) + " " + str(RobotList[m].robot_type))
        print("str_capability: "+str(RobotList[m].str_capability)+", rec_capability: "+str(RobotList[m].rec_capability)+", X:"+str(RobotList[m].x)+", Y:"+str(RobotList[m].y))


    return RobotList, TaskList

# 在线分配：CBPA
def create_robots_and_tasks_online(num_robots: int, num_tasks: int, robot_pose: list, task_pose: list, robot_str_capability: list, WorldInfoInput: WorldInfo, config_data): # 
    """
    Generate robots and tasks based on a json configuration file.

    config_data:
        config_file_name = "config.json"
        json_file = open(config_file_name)
        config_data = json.load(json_file)

    """
    # rec_robot
    robot_rec_default = Robot()
    robot_rec_default.robot_type = config_data["ROBOT_TYPES"].index("rec_robot")
    robot_rec_default.nom_velocity = float(config_data["REC_DEFAULT"]["NOM_VELOCITY"])
    # str_robot
    robot_str_default = Robot()
    robot_str_default.robot_type = config_data["ROBOT_TYPES"].index("str_robot")
    robot_str_default.nom_velocity = float(config_data["STR_DEFAULT"]["NOM_VELOCITY"])
    # rs_robot
    robot_rs_default = Robot()
    robot_rs_default.robot_type = config_data["ROBOT_TYPES"].index("rs_robot")
    robot_rs_default.nom_velocity = float(config_data["RS_DEFAULT"]["NOM_VELOCITY"])

    #My Task
    task_default = Task()
    task_default.task_type = config_data["TASK_TYPES"].index("coalition_task")
    task_default.task_value = float(config_data["TASK_DEFAULT"]["TASK_VALUE"])
    task_default.start_time = float(config_data["TASK_DEFAULT"]["START_TIME"])
    task_default.end_time = float(config_data["TASK_DEFAULT"]["END_TIME"])
    task_default.duration = float(config_data["TASK_DEFAULT"]["DURATION"])


    # create empty list, each element is a dataclass Robot() or Task()
    global RobotList, TaskList
    RobotList = []
    TaskList = []
    # robot_pos = [[150, 300], [150, 500], [150, 700], [150, 900], [150, 1100]]
    # robot_pos = [[150, 130], [150, 330], [150, 700], [150, 970], [150, 1270]]
    robot_type = [0, 0, 2, 1, 1, 0, 0, 0, 0, 0]  #0:rec 1:str 2:rs
    robot_rec_capability = [3, 3, 3, 0, 0, 0, 0, 0, 0, 0]

    #task_pos = [[850, 350], [1450, 550], [2100, 430], [740, 820], [1120, 740], [1710, 970], [2360, 680], [730, 1260], [1440, 1020], [1950, 1280]]
    # task_pos = [[850, 250], [1450, 450], [2360, 630], [740, 820], [1120, 600], [1810, 870], [2100, 580], [730, 1260], [1440, 1020], [2050, 1180]]

    task_rec_need = [1, 2, 3, 3, 3, 2, 2, 3, 2, 2]
    task_str_need = [8, 6, 8, 7, 8, 6, 9, 10, 8, 9]
    task_start_time = [90, 220, 350, 90, 220, 270, 350, 90, 220, 350]
    task_end_time = [120, 250, 380, 120, 250, 300, 380, 120, 250, 380]

    for idx_robot in range(0, num_robots):
        if robot_type[idx_robot] == 0:
            RobotList.append(Robot(**robot_rec_default.__dict__))
        elif robot_type[idx_robot] == 1:
            RobotList.append(Robot(**robot_str_default.__dict__))
        else:
            RobotList.append(Robot(**robot_rs_default.__dict__))
        RobotList[idx_robot].rec_capability = robot_rec_capability[idx_robot]
        RobotList[idx_robot].str_capability = robot_str_capability[idx_robot]
        RobotList[idx_robot].robot_id = idx_robot
        RobotList[idx_robot].x = robot_pose[idx_robot][0]
        RobotList[idx_robot].y = robot_pose[idx_robot][1]
        RobotList[idx_robot].z = 0

    for idx_task in range(0, num_tasks):
        TaskList.append(Task(**task_default.__dict__))
        TaskList[idx_task].task_id = idx_task
        TaskList[idx_task].x = task_pose[idx_task][0]
        TaskList[idx_task].y = task_pose[idx_task][1]
        TaskList[idx_task].z = 0
        if TaskList[idx_task].x == 0:
            TaskList[idx_task].rec_need = 0
            TaskList[idx_task].str_need = 0
            TaskList[idx_task].start_time = 0
            TaskList[idx_task].end_time = 0
        else:
            TaskList[idx_task].rec_need = task_rec_need[idx_task]
            TaskList[idx_task].str_need = task_str_need[idx_task]
        TaskList[idx_task].start_time = 0
        TaskList[idx_task].end_time = 300
    for n in range(num_tasks):
        print("Task " + str(n))
        print("str_need: "+str(TaskList[n].str_need)+ ", rec_needed: "+str(TaskList[n].rec_need)+", X:"+str(TaskList[n].x)+", Y:"+str(TaskList[n].y)+", start_time:"+str(TaskList[n].start_time))
    for m in range(num_robots):
        print("Robot " + str(m) + " " + str(RobotList[m].robot_type))
        print("str_capability: "+str(RobotList[m].str_capability)+", rec_capability: "+str(RobotList[m].rec_capability)+", X:"+str(RobotList[m].x)+", Y:"+str(RobotList[m].y))

    return RobotList, TaskList

# 实验一：侦察机器人和打击机器人挨在一起
def create_robots_and_tasks1(num_robots: int, num_tasks: int, WorldInfoInput: WorldInfo, config_data): # 
    """
    Generate robots and tasks based on a json configuration file.

    config_data:
        config_file_name = "config.json"
        json_file = open(config_file_name)
        config_data = json.load(json_file)

    """

    # Create some default robots 默认元素

    # rec_robot
    robot_rec_default = Robot()
    # robot type
    robot_rec_default.robot_type = config_data["ROBOT_TYPES"].index("rec_robot")
    # robot cruise velocity (m/s)
    robot_rec_default.nom_velocity = float(config_data["REC_DEFAULT"]["NOM_VELOCITY"])
    # # robot fuel penalty (per meter)
    # robot_quad_default.fuel = float(config_data["QUAD_DEFAULT"]["FUEL"])

    # str_robot
    robot_str_default = Robot()
    # robot type
    robot_str_default.robot_type = config_data["ROBOT_TYPES"].index("str_robot")
    # robot cruise velocity (m/s)
    robot_str_default.nom_velocity = float(config_data["STR_DEFAULT"]["NOM_VELOCITY"])
    # # robot fuel penalty (per meter)
    # robot_car_default.fuel = float(config_data["CAR_DEFAULT"]["FUEL"])

    robot_rs_default = Robot()
    # robot type
    robot_rs_default.robot_type = config_data["ROBOT_TYPES"].index("rs_robot")
    # robot cruise velocity (m/s)
    robot_rs_default.nom_velocity = float(config_data["RS_DEFAULT"]["NOM_VELOCITY"])

    #My Task
    task_default = Task()
    # task type
    task_default.task_type = config_data["TASK_TYPES"].index("coalition_task")
    # task reward 固定任务收益
    task_default.task_value = float(config_data["TASK_DEFAULT"]["TASK_VALUE"])
    # task start time (sec)
    task_default.start_time = float(config_data["TASK_DEFAULT"]["START_TIME"])
    # task expiry time (sec)
    task_default.end_time = float(config_data["TASK_DEFAULT"]["END_TIME"])
    # task default duration (sec)
    task_default.duration = float(config_data["TASK_DEFAULT"]["DURATION"])


    # create empty list, each element is a dataclass Robot() or Task()
    global RobotList, TaskList
    RobotList = []
    TaskList = []

    #robot 0
    RobotList.append(Robot(**robot_rec_default.__dict__))
    RobotList[0].rec_capability = 10
    RobotList[0].str_capability = 0
    RobotList[0].robot_id = 0
    RobotList[0].x = 150
    RobotList[0].y = 300
    RobotList[0].z = 0

    #robot 1
    RobotList.append(Robot(**robot_str_default.__dict__))
    RobotList[1].rec_capability = 0
    RobotList[1].str_capability = 35
    RobotList[1].robot_id = 1
    RobotList[1].x = 150
    RobotList[1].y = 500
    RobotList[1].z = 0

    #robot 2
    RobotList.append(Robot(**robot_rs_default.__dict__))
    RobotList[2].rec_capability = 10
    RobotList[2].str_capability = 35
    RobotList[2].robot_id = 2
    RobotList[2].x = 150
    RobotList[2].y = 700
    RobotList[2].z = 0

    #robot3
    RobotList.append(Robot(**robot_rec_default.__dict__))
    RobotList[3].rec_capability = 13
    RobotList[3].str_capability = 0
    RobotList[3].robot_id = 3
    RobotList[3].x = 150
    RobotList[3].y = 900
    RobotList[3].z = 0

    #robot4
    RobotList.append(Robot(**robot_str_default.__dict__))
    RobotList[4].rec_capability = 0
    RobotList[4].str_capability = 22
    
    RobotList[4].robot_id = 4
    RobotList[4].x = 150
    RobotList[4].y = 1100
    RobotList[4].z = 0

    #task0
    TaskList.append(Task(**task_default.__dict__))
    TaskList[0].task_id = 0
    TaskList[0].x = 850
    TaskList[0].y = 350
    TaskList[0].z = 0
    TaskList[0].rec_need = 1
    TaskList[0].str_need = 8
    TaskList[0].start_time = 90
    TaskList[0].end_time = 120

    #task1
    TaskList.append(Task(**task_default.__dict__))
    TaskList[1].task_id = 1
    TaskList[1].x = 1450
    TaskList[1].y = 550
    TaskList[1].z = 0
    TaskList[1].rec_need = 2
    TaskList[1].str_need = 6
    TaskList[1].start_time = 220 
    TaskList[1].end_time = 250

    #task2
    TaskList.append(Task(**task_default.__dict__))
    TaskList[2].task_id = 2
    TaskList[2].x = 2100
    TaskList[2].y = 430
    TaskList[2].z = 0
    TaskList[2].rec_need = 3
    TaskList[2].str_need = 7
    TaskList[2].start_time = 350
    TaskList[2].end_time = 380

    #task3
    TaskList.append(Task(**task_default.__dict__))
    TaskList[3].task_id = 3
    TaskList[3].x = 740
    TaskList[3].y = 820
    TaskList[3].z = 0
    TaskList[3].rec_need = 3
    TaskList[3].str_need = 7
    TaskList[3].start_time = 90
    TaskList[3].end_time = 120

    #task4
    TaskList.append(Task(**task_default.__dict__))
    TaskList[4].task_id = 4
    TaskList[4].x = 1120
    TaskList[4].y = 740
    TaskList[4].z = 0
    TaskList[4].rec_need = 3
    TaskList[4].str_need = 8
    TaskList[4].start_time = 220
    TaskList[4].end_time = 250

    #task5
    TaskList.append(Task(**task_default.__dict__))
    TaskList[5].task_id = 5
    TaskList[5].x = 1710
    TaskList[5].y = 970
    TaskList[5].z = 0
    TaskList[5].rec_need = 2
    TaskList[5].str_need = 10
    TaskList[5].start_time = 270
    TaskList[5].end_time = 300

    #task6
    TaskList.append(Task(**task_default.__dict__))
    TaskList[6].task_id = 6
    TaskList[6].x = 2160
    TaskList[6].y = 680
    TaskList[6].z = 0
    TaskList[6].rec_need = 2
    TaskList[6].str_need = 9
    TaskList[6].start_time = 350
    TaskList[6].end_time = 380

    #task7
    TaskList.append(Task(**task_default.__dict__))
    TaskList[7].task_id = 7
    TaskList[7].x = 730
    TaskList[7].y = 1260
    TaskList[7].z = 0
    TaskList[7].rec_need = 3
    TaskList[7].str_need = 10
    TaskList[7].start_time = 90
    TaskList[7].end_time = 120

    #task8
    TaskList.append(Task(**task_default.__dict__))
    TaskList[8].task_id = 8
    TaskList[8].x = 1440
    TaskList[8].y = 1020
    TaskList[8].z = 0
    TaskList[8].rec_need = 2
    TaskList[8].str_need = 8
    TaskList[8].start_time = 220
    TaskList[8].end_time = 250

    #task9
    TaskList.append(Task(**task_default.__dict__))
    TaskList[9].task_id = 9
    TaskList[9].x = 1950
    TaskList[9].y = 1280
    TaskList[9].z = 0
    TaskList[9].rec_need = 2
    TaskList[9].str_need = 7
    TaskList[9].start_time = 350
    TaskList[9].end_time = 380

    

    for n in range(num_tasks):
        TaskList[n].end_time = 300
        print("Task " + str(n))
        # print(str(TaskList[n].x)+", "+str(TaskList[n].y)+", "+str(TaskList[n].z))
        # print(str(TaskList[n].start_time)+" --- "+str(TaskList[n].end_time))
        print("str_need: "+str(TaskList[n].str_need)+ ", rec_needed: "+str(TaskList[n].rec_need)+", X:"+str(TaskList[n].x)+", Y:"+str(TaskList[n].y)+", start_time:"+str(TaskList[n].start_time))
    for m in range(num_robots):
        print("Robot " + str(m) + " " + str(RobotList[m].robot_type))
        # print(str(RobotList[m].x)+", "+str(RobotList[m].y)+", "+str(RobotList[m].z))
        # print(RobotList[m].robot_type)
        print("str_capability: "+str(RobotList[m].str_capability)+", rec_capability: "+str(RobotList[m].rec_capability)+", X:"+str(RobotList[m].x)+", Y:"+str(RobotList[m].y))


    return RobotList, TaskList

# 实验一：机器人初始位置顺序排版 终版，不修改！！！
def create_robots_and_tasks2(num_robots: int, num_tasks: int, WorldInfoInput: WorldInfo, config_data): # 
    """
    Generate robots and tasks based on a json configuration file.

    config_data:
        config_file_name = "config.json"
        json_file = open(config_file_name)
        config_data = json.load(json_file)

    """

    # Create some default robots 默认元素

    # rec_robot
    robot_rec_default = Robot()
    robot_rec_default.robot_type = config_data["ROBOT_TYPES"].index("rec_robot")
    robot_rec_default.nom_velocity = float(config_data["REC_DEFAULT"]["NOM_VELOCITY"])
    # str_robot
    robot_str_default = Robot()
    robot_str_default.robot_type = config_data["ROBOT_TYPES"].index("str_robot")
    robot_str_default.nom_velocity = float(config_data["STR_DEFAULT"]["NOM_VELOCITY"])
    # rs_robot
    robot_rs_default = Robot()
    robot_rs_default.robot_type = config_data["ROBOT_TYPES"].index("rs_robot")
    robot_rs_default.nom_velocity = float(config_data["RS_DEFAULT"]["NOM_VELOCITY"])

    #My Task
    task_default = Task()
    task_default.task_type = config_data["TASK_TYPES"].index("coalition_task")
    task_default.task_value = float(config_data["TASK_DEFAULT"]["TASK_VALUE"])
    task_default.start_time = float(config_data["TASK_DEFAULT"]["START_TIME"])
    task_default.end_time = float(config_data["TASK_DEFAULT"]["END_TIME"])
    task_default.duration = float(config_data["TASK_DEFAULT"]["DURATION"])


    # create empty list, each element is a dataclass Robot() or Task()
    global RobotList, TaskList
    RobotList = []
    TaskList = []
    # robot_pos = [[150, 300], [150, 500], [150, 700], [150, 900], [150, 1100]]
    robot_pos = [[150, 130], [150, 330], [150, 700], [150, 970], [150, 1270]]
    robot_type = [0, 0, 2, 1, 1]  #0:rec 1:str 2:rs
    robto_rec_capability = [3, 3, 3, 0, 0]
    robot_str_capability = [0, 0, 30, 30, 25]

    task_pos = [[850, 350], [1450, 550], [2100, 430], [740, 820], [1120, 740], [1710, 970], [2360, 680], [730, 1260], [1440, 1020], [1950, 1280]]
    # task_pos = [[850, 250], [1450, 450], [2360, 630], [740, 820], [1120, 600], [1810, 870], [2100, 580], [730, 1260], [1440, 1020], [2050, 1180]]

    task_rec_need = [1, 2, 3, 3, 3, 2, 2, 3, 2, 2]
    task_str_need = [8, 6, 8, 7, 8, 6, 9, 10, 8, 9]
    task_start_time = [90, 220, 350, 90, 220, 270, 350, 90, 220, 350]
    task_end_time = [120, 250, 380, 120, 250, 300, 380, 120, 250, 380]

    for idx_robot in range(0, num_robots):
        if robot_type[idx_robot] == 0:
            RobotList.append(Robot(**robot_rec_default.__dict__))
        elif robot_type[idx_robot] == 1:
            RobotList.append(Robot(**robot_str_default.__dict__))
        else:
            RobotList.append(Robot(**robot_rs_default.__dict__))
        RobotList[idx_robot].rec_capability = robto_rec_capability[idx_robot]
        RobotList[idx_robot].str_capability = robot_str_capability[idx_robot]
        RobotList[idx_robot].robot_id = idx_robot
        RobotList[idx_robot].x = robot_pos[idx_robot][0]
        RobotList[idx_robot].y = robot_pos[idx_robot][1]
        RobotList[idx_robot].z = 0

    for idx_task in range(0, num_tasks):
        TaskList.append(Task(**task_default.__dict__))
        TaskList[idx_task].task_id = idx_task
        TaskList[idx_task].x = task_pos[idx_task][0]
        TaskList[idx_task].y = task_pos[idx_task][1]
        TaskList[idx_task].z = 0
        TaskList[idx_task].rec_need = task_rec_need[idx_task]
        TaskList[idx_task].str_need = task_str_need[idx_task]
        TaskList[idx_task].start_time = task_start_time[idx_task]
        TaskList[idx_task].end_time = task_end_time[idx_task]


    # #robot 0
    # RobotList.append(Robot(**robot_rec_default.__dict__))
    # RobotList[0].rec_capability = 10
    # RobotList[0].str_capability = 0
    # RobotList[0].robot_id = 0
    # RobotList[0].x = 150
    # RobotList[0].y = 300
    # RobotList[0].z = 0

    # #robot 1
    # RobotList.append(Robot(**robot_rec_default.__dict__))
    # RobotList[1].rec_capability = 13
    # RobotList[1].str_capability = 0
    # RobotList[1].robot_id = 1
    # RobotList[1].x = 150
    # RobotList[1].y = 500
    # RobotList[1].z = 0

    # #robot 2
    # RobotList.append(Robot(**robot_rs_default.__dict__))
    # RobotList[2].rec_capability = 10
    # RobotList[2].str_capability = 35
    # RobotList[2].robot_id = 2
    # RobotList[2].x = 150
    # RobotList[2].y = 700
    # RobotList[2].z = 0

    # #robot3
    # RobotList.append(Robot(**robot_str_default.__dict__))
    # RobotList[3].rec_capability = 0
    # RobotList[3].str_capability = 35
    # RobotList[3].robot_id = 3
    # RobotList[3].x = 150
    # RobotList[3].y = 900
    # RobotList[3].z = 0

    # #robot4
    # RobotList.append(Robot(**robot_str_default.__dict__))
    # RobotList[4].rec_capability = 0
    # RobotList[4].str_capability = 22
    
    # RobotList[4].robot_id = 4
    # RobotList[4].x = 150
    # RobotList[4].y = 1100
    # RobotList[4].z = 0

    # #task0
    # TaskList.append(Task(**task_default.__dict__))
    # TaskList[0].task_id = 0
    # TaskList[0].x = 850
    # TaskList[0].y = 350
    # TaskList[0].z = 0
    # TaskList[0].rec_need = 1
    # TaskList[0].str_need = 8
    # TaskList[0].start_time = 90
    # TaskList[0].end_time = 120

    # #task1
    # TaskList.append(Task(**task_default.__dict__))
    # TaskList[1].task_id = 1
    # TaskList[1].x = 1450
    # TaskList[1].y = 550
    # TaskList[1].z = 0
    # TaskList[1].rec_need = 2
    # TaskList[1].str_need = 6
    # TaskList[1].start_time = 220 
    # TaskList[1].end_time = 250

    # #task2
    # TaskList.append(Task(**task_default.__dict__))
    # TaskList[2].task_id = 2
    # TaskList[2].x = 2100
    # TaskList[2].y = 430
    # TaskList[2].z = 0
    # TaskList[2].rec_need = 3
    # TaskList[2].str_need = 7
    # TaskList[2].start_time = 350
    # TaskList[2].end_time = 380

    # #task3
    # TaskList.append(Task(**task_default.__dict__))
    # TaskList[3].task_id = 3
    # TaskList[3].x = 740
    # TaskList[3].y = 820
    # TaskList[3].z = 0
    # TaskList[3].rec_need = 3
    # TaskList[3].str_need = 7
    # TaskList[3].start_time = 90
    # TaskList[3].end_time = 120

    # #task4
    # TaskList.append(Task(**task_default.__dict__))
    # TaskList[4].task_id = 4
    # TaskList[4].x = 1120
    # TaskList[4].y = 740
    # TaskList[4].z = 0
    # TaskList[4].rec_need = 3
    # TaskList[4].str_need = 8
    # TaskList[4].start_time = 220
    # TaskList[4].end_time = 250

    # #task5
    # TaskList.append(Task(**task_default.__dict__))
    # TaskList[5].task_id = 5
    # TaskList[5].x = 1610
    # TaskList[5].y = 970
    # TaskList[5].z = 0
    # TaskList[5].rec_need = 2
    # TaskList[5].str_need = 10
    # TaskList[5].start_time = 270
    # TaskList[5].end_time = 300

    # #task6
    # TaskList.append(Task(**task_default.__dict__))
    # TaskList[6].task_id = 6
    # TaskList[6].x = 2160
    # TaskList[6].y = 680
    # TaskList[6].z = 0
    # TaskList[6].rec_need = 2
    # TaskList[6].str_need = 9
    # TaskList[6].start_time = 350
    # TaskList[6].end_time = 380

    # #task7
    # TaskList.append(Task(**task_default.__dict__))
    # TaskList[7].task_id = 7
    # TaskList[7].x = 730
    # TaskList[7].y = 1260
    # TaskList[7].z = 0
    # TaskList[7].rec_need = 3
    # TaskList[7].str_need = 10
    # TaskList[7].start_time = 90
    # TaskList[7].end_time = 120

    # #task8
    # TaskList.append(Task(**task_default.__dict__))
    # TaskList[8].task_id = 8
    # TaskList[8].x = 1440
    # TaskList[8].y = 1020
    # TaskList[8].z = 0
    # TaskList[8].rec_need = 2
    # TaskList[8].str_need = 8
    # TaskList[8].start_time = 220
    # TaskList[8].end_time = 250

    # #task9
    # TaskList.append(Task(**task_default.__dict__))
    # TaskList[9].task_id = 9
    # TaskList[9].x = 1950
    # TaskList[9].y = 1280
    # TaskList[9].z = 0
    # TaskList[9].rec_need = 2
    # TaskList[9].str_need = 7
    # TaskList[9].start_time = 350
    # TaskList[9].end_time = 380

    

    for n in range(num_tasks):
        TaskList[n].end_time = 300
        print("Task " + str(n))
        # print(str(TaskList[n].x)+", "+str(TaskList[n].y)+", "+str(TaskList[n].z))
        # print(str(TaskList[n].start_time)+" --- "+str(TaskList[n].end_time))
        print("str_need: "+str(TaskList[n].str_need)+ ", rec_needed: "+str(TaskList[n].rec_need)+", X:"+str(TaskList[n].x)+", Y:"+str(TaskList[n].y)+", start_time:"+str(TaskList[n].start_time))
    for m in range(num_robots):
        print("Robot " + str(m) + " " + str(RobotList[m].robot_type))
        # print(str(RobotList[m].x)+", "+str(RobotList[m].y)+", "+str(RobotList[m].z))
        # print(RobotList[m].robot_type)
        print("str_capability: "+str(RobotList[m].str_capability)+", rec_capability: "+str(RobotList[m].rec_capability)+", X:"+str(RobotList[m].x)+", Y:"+str(RobotList[m].y))


    return RobotList, TaskList


# 区域9等分，任务点随机, world(242*136)
def create_robots_and_tasks6(num_robots: int, num_tasks: int, WorldInfoInput: WorldInfo, config_data):  
    """
    Generate robots and tasks based on a json configuration file.

    config_data:
        config_file_name = "config.json"
        json_file = open(config_file_name)
        config_data = json.load(json_file)

    """

    # Create some default robots 默认元素

    # rec_robot
    robot_rec_default = Robot()
    # robot type
    robot_rec_default.robot_type = config_data["ROBOT_TYPES"].index("rec_robot")
    # robot cruise velocity (m/s)
    robot_rec_default.nom_velocity = float(config_data["REC_DEFAULT"]["NOM_VELOCITY"])

    # str_robot
    robot_str_default = Robot()
    # robot type
    robot_str_default.robot_type = config_data["ROBOT_TYPES"].index("str_robot")
    # robot cruise velocity (m/s)
    robot_str_default.nom_velocity = float(config_data["STR_DEFAULT"]["NOM_VELOCITY"])

    # rs_robot
    robot_rs_default = Robot()
    # robot type
    robot_rs_default.robot_type = config_data["ROBOT_TYPES"].index("rs_robot")
    # robot cruise velocity (m/s)
    robot_rs_default.nom_velocity = float(config_data["RS_DEFAULT"]["NOM_VELOCITY"])

    #My Task
    task_default = Task()
    # task type
    task_default.task_type = config_data["TASK_TYPES"].index("coalition_task")
    # task reward 固定任务收益
    task_default.task_value = float(config_data["TASK_DEFAULT"]["TASK_VALUE"])
    # task start time (sec)
    task_default.start_time = float(config_data["TASK_DEFAULT"]["START_TIME"])
    # task expiry time (sec)
    task_default.end_time = float(config_data["TASK_DEFAULT"]["END_TIME"])
    # task default duration (sec)
    task_default.duration = float(config_data["TASK_DEFAULT"]["DURATION"])


    # create empty list, each element is a dataclass Robot() or Task()
    global RobotList, TaskList
    RobotList = []
    TaskList = []
    # robot_reccapability_list = []
    # robot_strcapability_list = []
    # task_recneed_list = []
    # task_strneed_list = []
    robot_pos = [[15, 30],[15, 50],[15,70],[15, 90],[15, 110]]
    world_pos = [[[60, 120],[0, 60]], [[120,180],[0,60]], [[180, 240],[0, 60]],
                  [[60, 120],[60, 100]], [[120,180],[60, 100]], [[180, 240],[60, 100]], 
                  [[60, 120],[100, 135]], [[120,180],[100, 135]], [[180, 240],[100, 135]], [[80, 220],[0, 136]]]
    # create random robots  随机元素
    for idx_robot in range(0, num_robots):
        # create a new instance of dataclass robot_rec_default
        if idx_robot/num_robots < 0.4:
            RobotList.append(Robot(**robot_rec_default.__dict__))
            RobotList[idx_robot].rec_capability = random.randint(5,15)
            RobotList[idx_robot].str_capability = 0
        elif 0.4 <= idx_robot/num_robots < 0.8:
            RobotList.append(Robot(**robot_str_default.__dict__))
            RobotList[idx_robot].str_capability = random.randint(25,50)
            RobotList[idx_robot].rec_capability = 0
        else: 
            RobotList.append(Robot(**robot_rs_default.__dict__))   
            RobotList[idx_robot].str_capability = random.randint(15,25)
            RobotList[idx_robot].rec_capability = random.randint(5,10)
        
        RobotList[idx_robot].robot_id = idx_robot
        RobotList[idx_robot].x = robot_pos[idx_robot][0]
        RobotList[idx_robot].y = robot_pos[idx_robot][1]
        RobotList[idx_robot].z = 0

    # create random tasks (track only)
    for idx_task in range(0, num_tasks):
        # create a new instance of dataclass task_track_default
        TaskList.append(Task(**task_default.__dict__))
        
        TaskList[idx_task].task_id = idx_task
        TaskList[idx_task].x = random.uniform(world_pos[idx_task][0][0],world_pos[idx_task][0][1])
        TaskList[idx_task].y = random.uniform(world_pos[idx_task][1][0], world_pos[idx_task][1][1])
        TaskList[idx_task].z = 0
        TaskList[idx_task].rec_need = random.randint(1,3)
        TaskList[idx_task].str_need = random.randint(5,10)
        #if idx_task <= num_tasks*0.4:
        # if idx_task == 0 or idx_task == 3 or idx_task == 6:
        #     TaskList[idx_task].start_time = random.uniform(0, (float(config_data["TASK_DEFAULT"]["END_TIME"])-float(config_data["TASK_DEFAULT"]["DURATION"]))/3)
        # #elif num_tasks*0.4 < idx_task <= num_tasks*0.8:
        # elif idx_task == 2 or idx_task == 4 or idx_task == 7:
        #     TaskList[idx_task].start_time = random.uniform((float(config_data["TASK_DEFAULT"]["END_TIME"])-float(config_data["TASK_DEFAULT"]["DURATION"]))/3, (float(config_data["TASK_DEFAULT"]["END_TIME"])-float(config_data["TASK_DEFAULT"]["DURATION"]))*2/3)
        # elif idx_task == 9:
        #     TaskList[idx_task].start_time = random.uniform(0, (float(config_data["TASK_DEFAULT"]["END_TIME"])))
        # else:
        #     TaskList[idx_task].start_time = random.uniform((float(config_data["TASK_DEFAULT"]["END_TIME"])-float(config_data["TASK_DEFAULT"]["DURATION"]))*2/3, float(config_data["TASK_DEFAULT"]["END_TIME"])-float(config_data["TASK_DEFAULT"]["DURATION"]))
        TaskList[idx_task].start_time = 350
        TaskList[idx_task].end_time = TaskList[idx_task].start_time + TaskList[idx_task].duration

    for n in range(num_tasks):
        print("Task " + str(n))
        # print(str(TaskList[n].x)+", "+str(TaskList[n].y)+", "+str(TaskList[n].z))
        # print(str(TaskList[n].start_time)+" --- "+str(TaskList[n].end_time))
        print("str_need: "+str(TaskList[n].str_need)+ ", rec_needed: "+str(TaskList[n].rec_need)+", X:"+str(TaskList[n].x)+", Y:"+str(TaskList[n].y)+", start_time:"+str(TaskList[n].start_time))
    for m in range(num_robots):
        print("Robot " + str(m) + " " + str(RobotList[m].robot_type))
        # print(str(RobotList[m].x)+", "+str(RobotList[m].y)+", "+str(RobotList[m].z))
        # print(RobotList[m].robot_type)
        print("str_capability: "+str(RobotList[m].str_capability)+", rec_capability: "+str(RobotList[m].rec_capability)+", X:"+str(RobotList[m].x)+", Y:"+str(RobotList[m].y))

    return RobotList, TaskList

def create_robots_and_tasks_XFD(num_robots: int, num_tasks: int, WorldInfoInput: WorldInfo, config_data):  
    """
    Generate robots and tasks based on a json configuration file.

    config_data:
        config_file_name = "config.json"
        json_file = open(config_file_name)
        config_data = json.load(json_file)

    """

    # Create some default robots 默认元素

    # rec_robot
    robot_rec_default = Robot()
    # robot type
    robot_rec_default.robot_type = config_data["ROBOT_TYPES"].index("rec_robot")
    # robot cruise velocity (m/s)
    robot_rec_default.nom_velocity = float(config_data["REC_DEFAULT"]["NOM_VELOCITY"])

    # str_robot
    robot_str_default = Robot()
    # robot type
    robot_str_default.robot_type = config_data["ROBOT_TYPES"].index("str_robot")
    # robot cruise velocity (m/s)
    robot_str_default.nom_velocity = float(config_data["STR_DEFAULT"]["NOM_VELOCITY"])

    # rs_robot
    robot_rs_default = Robot()
    # robot type
    robot_rs_default.robot_type = config_data["ROBOT_TYPES"].index("rs_robot")
    # robot cruise velocity (m/s)
    robot_rs_default.nom_velocity = float(config_data["RS_DEFAULT"]["NOM_VELOCITY"])

    #My Task
    task_default = Task()
    # task type
    task_default.task_type = config_data["TASK_TYPES"].index("coalition_task")
    # task reward 固定任务收益
    task_default.task_value = float(config_data["TASK_DEFAULT"]["TASK_VALUE"])
    # task start time (sec)
    task_default.start_time = float(config_data["TASK_DEFAULT"]["START_TIME"])
    # task expiry time (sec)
    task_default.end_time = float(config_data["TASK_DEFAULT"]["END_TIME"])
    # task default duration (sec)
    task_default.duration = float(config_data["TASK_DEFAULT"]["DURATION"])


    # create empty list, each element is a dataclass Robot() or Task()
    global RobotList, TaskList
    RobotList = []
    TaskList = []
    # robot_reccapability_list = []
    # robot_strcapability_list = []
    # task_recneed_list = []
    # task_strneed_list = []
    robot_pos = [[15, 30],[15, 50],[15,70],[15, 90],[15, 110]]
    world_pos = [[[60, 120],[0, 60]], [[120,180],[0,60]], [[180, 240],[0, 60]],
                  [[60, 120],[60, 100]], [[120,180],[60, 100]], [[180, 240],[60, 100]], 
                  [[60, 120],[100, 135]], [[120,180],[100, 135]], [[180, 240],[100, 135]], [[80, 220],[0, 136]]]
    # create random robots  随机元素
    for idx_robot in range(0, num_robots):
        # create a new instance of dataclass robot_rec_default
        
        RobotList.append(Robot(**robot_rs_default.__dict__))   
        RobotList[idx_robot].str_capability = 1
        RobotList[idx_robot].rec_capability = 1

        RobotList[idx_robot].robot_id = idx_robot
        RobotList[idx_robot].x = random.uniform(WorldInfoInput.limit_x[0], WorldInfoInput.limit_x[1])
        RobotList[idx_robot].y = random.uniform(WorldInfoInput.limit_y[0], WorldInfoInput.limit_y[1])
        RobotList[idx_robot].z = 300
        RobotList[idx_robot].status = True

    # create random tasks (track only)
    for idx_task in range(0, num_tasks):
        # create a new instance of dataclass task_track_default
        TaskList.append(Task(**task_default.__dict__))
        
        TaskList[idx_task].task_id = idx_task
        TaskList[idx_task].x = random.uniform(WorldInfoInput.limit_x[0], WorldInfoInput.limit_x[1])
        TaskList[idx_task].y = random.uniform(WorldInfoInput.limit_y[0], WorldInfoInput.limit_y[1])
        TaskList[idx_task].z = 0
        # TaskList[idx_task].rec_need = math.ceil(num_robots/num_tasks)
        TaskList[idx_task].rec_need = 3
        TaskList[idx_task].str_need = 0
        TaskList[idx_task].start_time = 350
        TaskList[idx_task].end_time = TaskList[idx_task].start_time + TaskList[idx_task].duration

    for n in range(num_tasks):
        print("Task " + str(n))
        # print(str(TaskList[n].x)+", "+str(TaskList[n].y)+", "+str(TaskList[n].z))
        # print(str(TaskList[n].start_time)+" --- "+str(TaskList[n].end_time))
        print("str_need: "+str(TaskList[n].str_need)+ ", rec_needed: "+str(TaskList[n].rec_need)+", X:"+str(TaskList[n].x)+", Y:"+str(TaskList[n].y)+", start_time:"+str(TaskList[n].start_time))
    for m in range(num_robots):
        print("Robot " + str(m) + " " + str(RobotList[m].robot_type))
        # print(str(RobotList[m].x)+", "+str(RobotList[m].y)+", "+str(RobotList[m].z))
        # print(RobotList[m].robot_type)
        print("str_capability: "+str(RobotList[m].str_capability)+", rec_capability: "+str(RobotList[m].rec_capability)+", X:"+str(RobotList[m].x)+", Y:"+str(RobotList[m].y))

    return RobotList, TaskList


def create_robots_and_tasks_homogeneous(num_robots: int, num_tasks: int, WorldInfoInput: WorldInfo, config_data):
    """
    Generate robots and tasks (only 1 type) based on a json configuration file.

    config_data:
        config_file_name = "config.json"
        json_file = open(config_file_name)
        config_data = json.load(json_file)

    """

    # Create some default robots
    # quad
    robot_quad_default = Robot()
    # robot type
    robot_quad_default.robot_type = config_data["ROBOT_TYPES"].index("quad")
    # robot cruise velocity (m/s)
    robot_quad_default.nom_velocity = float(config_data["QUAD_DEFAULT"]["NOM_VELOCITY"])
    # # robot fuel penalty (per meter)
    # robot_quad_default.fuel = float(config_data["QUAD_DEFAULT"]["FUEL"])

    # Create some default tasks
    # Track
    task_track_default = Task()
    # task type
    task_track_default.task_type = config_data["TASK_TYPES"].index("track")
    # task reward
    task_track_default.task_value = float(config_data["TRACK_DEFAULT"]["TASK_VALUE"])
    # task start time (sec)
    task_track_default.start_time = float(config_data["TRACK_DEFAULT"]["START_TIME"])
    # task expiry time (sec)
    task_track_default.end_time = float(config_data["TRACK_DEFAULT"]["END_TIME"])
    # task default duration (sec)
    task_track_default.duration = float(config_data["TRACK_DEFAULT"]["DURATION"])

    # create empty list, each element is a dataclass Robot() or Task()
    RobotList = []
    TaskList = []

    # create random robots
    for idx_robot in range(num_robots):
        # create a new instance of dataclass robot_quad_default
        RobotList.append(Robot(**robot_quad_default.__dict__))

        # RobotList.append(Robot(**robot_quad_default.__dict__))
        RobotList[idx_robot].robot_id = idx_robot
        RobotList[idx_robot].x = random.uniform(WorldInfoInput.limit_x[0], WorldInfoInput.limit_x[1])
        RobotList[idx_robot].y = random.uniform(WorldInfoInput.limit_y[0], WorldInfoInput.limit_y[1])
        RobotList[idx_robot].z = 0

    # create random tasks (track only)
    for idx_task in range(num_tasks):
        # create a new instance of dataclass task_track_default
        TaskList.append(Task(**task_track_default.__dict__))

        TaskList[idx_task].task_id = idx_task
        TaskList[idx_task].x = random.uniform(WorldInfoInput.limit_x[0], WorldInfoInput.limit_x[1])
        TaskList[idx_task].y = random.uniform(WorldInfoInput.limit_y[0], WorldInfoInput.limit_y[1])
        TaskList[idx_task].z = 0
        TaskList[idx_task].start_time = 0.0
        TaskList[idx_task].duration = 0.0
        TaskList[idx_task].end_time = 0.0

    for n in range(num_tasks):
        print("Task " + str(n))
        print(str(TaskList[n].x)+", "+str(TaskList[n].y)+", "+str(TaskList[n].z))
        print(str(TaskList[n].start_time)+" - "+str(TaskList[n].end_time))
    for m in range(num_robots):
        print("Robot " + str(m))
        print(str(RobotList[m].x)+", "+str(RobotList[m].y)+", "+str(RobotList[m].z))

    return RobotList, TaskList


def remove_from_list(list_input: list, index: int):
    """
    Remove item from list at location specified by index, then append -1 at the end.
    An exact implementation of remove_from_list in CBCA MATLAB version:
    http://acl.mit.edu/projects/consensus-based-bundle-algorithm
    But it's not used in this repository due to the inefficiency.

    Example:
        list_input = [0, 1, 2, 3, 4]
        index = 2
        list_output = remove_from_list(list_input, index)
        list_output = [0, 1, 3, 4, -1]
    """

    list_output = ((-1) * np.ones((1, len(list_input)))).flatten()
    list_output[0: index] = np.array(list_input[0: index])
    list_output[index: -1] = np.array(list_input[index+1:])
    return list_output.tolist()


def insert_in_list(list_input: list, value: float, index: int):
    """
    Insert value into list at location specified by index, and delete the last one of original list.
    An exact implementation of insert_in_list in CBCA MATLAB version:
    http://acl.mit.edu/projects/consensus-based-bundle-algorithm
    But it's not used in this repository due to the inefficiency.

    Example:
        list_input = [0, 1, 2, 3, 4]
        value = 100
        index = 2
        list_output = insert_in_list(list_input, value, index)
        list_output = [0, 1, 100, 2, 3]
    """

    list_output = ((-1) * np.ones((1, len(list_input)))).flatten()
    list_output[0: index] = np.array(list_input[0: index])
    list_output[index] = value
    list_output[index+1:] = np.array(list_input[index:-1])
    return list_output.tolist()
