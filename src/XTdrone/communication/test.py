import rospy
from mavros_msgs.srv import ParamGet, ParamSet

def set_param(param_id, value):
  rospy.wait_for_service('/mavros/param/set')
  try:
    param_set = rospy.ServiceProxy('/mavros/param/set', ParamSet)
    param_set(param_id, value)
    print(value)
  except rospy.ServiceException as e:
    print("Service call failed: %s" % e)

def get_param(param_id):
  rospy.wait_for_service('/mavros/param/get')
  try:
    param_get = rospy.ServiceProxy('/mavros/param/get', ParamGet)
    response = param_get(param_id)
    return response.value
  except rospy.ServiceException as e:
    print("Service call failed: %s" % e)

# 修改参数FW_AIRSPD_TRIM 的值为 10.0
set_param('FW_AIRSPD_TRIM', 20.0)

# 获取参数FW_AIRSPD_TRIM 的值
trim_value = get_param('FW_AIRSPD_TRIM')
print("FW_AIRSPD_TRIM value:", trim_value)