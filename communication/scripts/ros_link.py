'''
Author: popfishy
Date: 2024-06-24 21:14:25
LastEditors: Please set LastEditors
LastEditTime: 2024-06-24 23:46:35
Description: 
'''
import rospy
import json
from std_msgs.msg import String
from pusim_msgs.msg import PusimEntityData,PusimKeyString
from rospy_message_converter import json_message_converter

class JsonReassembler:
    def __init__(self):
        self.data_segments = {}
        self.total_size = 0
        self.received_data = ""
        self.subscriber = rospy.Subscriber('/ResearchGroup5Result', PusimKeyString, self.callback)
    
    def callback(self, data):
        # 处理接收到的消息
        index = data.index
        size = data.size
        key = data.key
        segment_data = data.data
        if not self.total_size:
            self.total_size = size
        
        # 存储数据片段
        self.data_segments[index] = segment_data
        # print("%s",segment_data)

        # 检查是否所有数据都已接收
        if len(self.data_segments) == self.total_size:
            self.reassemble_data()
        
    def reassemble_data(self):
        # 按照index排序并重组数据
        sorted_segments = [self.data_segments[i] for i in sorted(self.data_segments)]
        self.received_data = ''.join(sorted_segments)

        try:
            # 尝试解析重组后的JSON数据
            received_json = json.loads(self.received_data)
            rospy.loginfo("Reconstructed JSON data: %s", received_json)
            
            # 将重组后的JSON数据保存到本地文件
            with open('../json/ResearchGroup5Result.json', 'w') as file:
                json.dump(received_json, file, indent=4)
            rospy.loginfo("JSON data saved to ResearchGroup5Result.json")

            self.data_segments.clear()  # 清除缓存
            self.received_data = ""     # 清空重组数据
        except ValueError:
            rospy.logerr("Failed to parse JSON data")

if __name__ == "__main__":
    rospy.init_node('json_reassembler_node', anonymous=True)
    reassembler = JsonReassembler()
    rospy.spin()
