from CBPA.lib.Task import Task


# pos1-------------pos4
#  |                |
# pos2-------------pos3
class Region:
    def __init__(self, region_id: int, pos1, pos2, pos3, pos4):
        self.region_id = region_id
        self.pos1 = pos1
        self.pos2 = pos2
        self.pos3 = pos3
        self.pos4 = pos4
        self.xmin = pos2[0]
        self.xmax = pos4[0]
        self.ymin = pos2[2]
        self.ymax = pos4[2]
        self.z = pos2[1]
        self.length = pos4[0] - pos2[0]
        self.width = pos4[2] - pos2[2]
        self.center = ((pos4[0] + pos2[0]) / 2, (pos4[2] + pos2[2]) / 2)
        self.area = self.length * self.width
        self.uav_id_list = []
        self.task_id_list = []
        self.threat_level: float = 0

    def task_is_in_region(self, task: Task):
        # Check if the task's position is within the region's boundaries
        return self.xmin <= task.x <= self.xmax and self.ymin <= task.y <= self.ymax

    def get_task_id_list(self, task_list):
        for task in task_list:
            is_in_region = self.task_is_in_region(task)
            if is_in_region:
                self.threat_level = self.threat_level + task.robot_need
                self.task_id_list.append(task.task_id)
