import rospy
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import ParamValue
import sys

class QGCParamSetter:
    def __init__(self, vehicle_type, vehicle_num):
        self.vehicle_type = vehicle_type
        self.vehicle_num = vehicle_num
        self.param_setter_list = []
        for i in range(self.vehicle_num):
            self.param_setter_list.append(rospy.ServiceProxy(f'{self.vehicle_type}_{i}/mavros/param/set', ParamSet))
    
    # 设置QGC整数类型参数，返回修改是否成功。若vehicle_ids为空时，默认修改所有无人机的参数
    def set_int_param(self, param_name, param_value, vehicle_ids = None):
        if vehicle_ids is None:
            vehicle_ids = range(self.vehicle_num)
        try:
            results = []
            for vehicle_id in vehicle_ids:
                value = ParamValue()
                value.real = 0  
                value.integer = param_value
                response = self.param_setter_list[vehicle_id](param_name, value)
                results.append(response.success)
            if sum(results) == len(results):
                print(f"Parameter: {param_name} set successfully.")
            else:
                print(f"Failed to set parameter: {param_name}")
        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))
        
            

    # 设置QGC浮点类型参数，返回修改是否成功。若vehicle_ids为空时，默认修改所有无人机的参数
    def set_float_param(self, param_name, param_value, vehicle_ids = None):
        if vehicle_ids is None:
            vehicle_ids = range(self.vehicle_num)
        try:
            results = []
            for vehicle_id in vehicle_ids:
                value = ParamValue()
                value.real = param_value
                value.integer = 0
                response = self.param_setter_list[vehicle_id](param_name, value)
                results.append(response.success)
            if sum(results) == len(results):
                print(f"Parameter: {param_name} set successfully.")
            else:
                print(f"Failed to set parameter: {param_name}")
        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))
            


# 示例调用
if __name__ == "__main__":
    vehicle_type = sys.argv[1]
    vehicle_num = int(sys.argv[2])
    rospy.init_node('param_setter')
    param_setter = QGCParamSetter(vehicle_type, vehicle_num)
    param_setter.set_float_param("FW_AIRSPD_MAX",80.0)
    param_setter.set_float_param("FW_AIRSPD_TRIM",60.0)
    param_setter.set_float_param("FW_THR_MAX",100.0)
    param_setter.set_float_param("FW_THR_CRUISE",80.0)
