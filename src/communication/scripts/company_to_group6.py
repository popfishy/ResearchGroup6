import rospy
from group6_interfaces.msg import PusimKeyString

def paras_json_data(filename):
    with open(filename, 'r', encoding='utf-8') as file:
        json_data = file.read()
        # 计算字符串长度
        pusim_key_strings_msg = PusimKeyString()
        pusim_key_strings_msg.index = 0
        pusim_key_strings_msg.size = 1
        pusim_key_strings_msg.key = "TaskAllocationPlan"
        pusim_key_strings_msg.data = json_data

        return pusim_key_strings_msg
def main():
    rospy.init_node("send_company_data")
    pub = rospy.Publisher("/ResearchGroup5Result", PusimKeyString, queue_size=1)
    filename = "../json/ResearchGroup5Result_Prepare_1.json"
    plan_data = paras_json_data(filename)

    rospy.sleep(5)
    pub.publish(plan_data)

    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        pub.publish(plan_data)
        rate.sleep()

if __name__ == '__main__':
    main()