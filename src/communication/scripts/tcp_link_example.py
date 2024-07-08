'''
Author: popfishy
Date: 2024-06-12 21:31:48
LastEditors: Please set LastEditors
LastEditTime: 2024-06-12 21:42:09
Description: json数据字段信息  camp:红蓝方 1红  entityType:飞机实体类型 upEntityID:1024 velocity:m/s
'''
import socket
import json
# from PathPlanning import process

def read_json_from_file(file_path):
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            data = json.load(file)
            return data
    except FileNotFoundError:
        print(f"The file {file_path} was not found.")
    except json.JSONDecodeError:
        print(f"Error decoding JSON from the file {file_path}.")

def main():
    # 创建一个客户端 Socket 对象
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 设置服务端地址和端口号
    server_address = ('10.66.1.90', 13334)  # 这里显式地指定服务器的 IP 地址和端口号

    # 连接到服务端
    # 发送数据到服务端,每次必须发
    # try:
    #     client_socket.connect(server_address)
    #     print("Connected to server")
    #     message = 'ResearchGroup5'
    #     client_socket.sendall(message.encode())
    # except socket.error as e:
    #     print(f"Connection failed: {e}")

    try:
        #
        # data = {
        #     'name': 'ResearchGroup5',
        #     'key': 'RequestMission'
        # }
        # json_data = json.dumps(data).encode('utf-8')
        # client_socket.sendall(json_data)

        # 循环接受和发送算法结果
        while True:
            # 接收服务端响应的数据
            # received_data = b''
            # while True:
            #     chunk = client_socket.recv(4096)  # 每次接收4096字节
            #     received_data += chunk
            #     if len(chunk) < 4096:
            #         break

            print('-----------------------------------------------')
            print('Received New Data')
            print('-----------------------------------------------')

            # # 检查接收到的数据是否是完整的JSON
            # try:
            #     received_str = received_data.decode('utf-8')
            #     json_response = json.loads(received_str)
            # except json.JSONDecodeError as e:
            #     print(f"Error decoding JSON: {e}")
            #     print("Received data:", received_str)  # 打印接收到的数据以进行调试
            #     continue  # 跳过当前循环，继续接收新的请

            # 解码数据为字符串
            # received_str = received_data.decode('utf-8')

            # 存入和读取json object从文件中
            # with open('received_data.json', 'w', encoding='utf-8') as f:
            #     f.write(received_str)
            json_file_path = 'json/received_data.json'
            json_response = read_json_from_file(json_file_path)

            # json_response = json.loads(received_str)  # 替换单引号为双引号

            # 处理接收到的 JSON 数据
            sendScheduleGroup = json_response["sendScheduleGroup"]
            missioncode2dest = {}   # key (str) = mission task code, value(tuple) = (lat, longitude)
            agentid2missioncode = {}
            for mission in sendScheduleGroup:
                mission_code = mission["taskCode"]
                agent_id = mission["agentId"]
                agentid2missioncode[agent_id] = mission_code
                if mission_code not in missioncode2dest:
                    latitude = mission["latitude"]
                    longitude = mission["longitude"]
                    missioncode2dest[mission_code] = [latitude, longitude]

            # 生成路径并保存到结果中
            path_planned = {}
            id2startandend = {}     # key: agentid ; value: { 'start': coordinates, 'goal': coordinates}
            for key, value in json_response.items():
                if key == "key":
                    break
                else:
                    current_agent = value["agentId"]
                    current_missionCode = agentid2missioncode[current_agent]
                    dest_latitude, dest_longitude = missioncode2dest[current_missionCode]
                    start_latitude, start_longitude = value["latitude"], value["longitude"]
                    # 在此处启动路径规划算法
                    coordinates = {}
                    coordinates['start'] = (start_latitude, start_longitude)
                    coordinates['goal'] = (dest_latitude, dest_longitude)
                    id2startandend[current_agent] = coordinates

            # processed_path = process(id2startandend)  # 处理节点

            all_planned_result = []
            for id, path in processed_path.items():
                all_plane_info = {}
                all_plane_info["agentId"] = id
                all_plane_info["taskCode"] = agentid2missioncode[id]
                all_plane_info['beginTime'] = 203
                all_plane_info['expectedDuration'] = 200
                current_point_info = []
                if not path:
                    continue
                for i in range(len(path)):

                    latitude, longitude = path[i][0], path[i][1]
                    coordinates2info = {}
                    coordinates2info['latitude'] = latitude
                    coordinates2info['longitude'] = longitude
                    coordinates2info['altitude'] = 200
                    coordinates2info['velocity'] = 60

                    current_point_info.append(coordinates2info)
                all_plane_info['path'] = current_point_info
                all_planned_result.append(all_plane_info)

            # 发送结果
            path_planned['name'] = 'ResearchGroup5'
            path_planned['key'] = 'PlanningResults'
            path_planned['Planned_result'] = all_planned_result

            result_path_json = json.dumps(path_planned, indent=4).encode('utf-8')
            client_socket.sendall(result_path_json)
            print('----------------- Finished sending for this iteration -----------------------')
            print()

    except socket.error as e:
        print(f"Connection failed: {e}")
    finally:
        # 关client_socket.close()闭 Socket 连接
        client_socket.close()
        print()

if __name__ == "__main__":
    main()
