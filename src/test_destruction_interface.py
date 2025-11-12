#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import random
from start_simulation import SwarmMissionManager
from communication.scripts.tcp_client_communication import TCPClient
import os
import json

def _save_data_to_json(data, filepath: str):
    """将数据保存为JSON文件"""
    try:
        # 确保目录存在
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        json_str = json.dumps(data, indent=4)
        with open(filepath, "w") as file:
            file.write(json_str)
    except Exception as e:
        print(f"Error saving data to JSON file '{filepath}': {e}")

def test_destruction_message():
    """
    测试两种损毁消息场景：
    1. UAV攻击敌方目标（droneIsDeath=1）
    2. 敌方目标攻击UAV（droneIsDeath=0）
    """
    print("=== Starting Destruction Interface Test ===\n")

    # 1. Initialize the Mission Manager and load the scenario
    mission_manager = SwarmMissionManager()
    xml_file_path = '山地丛林.xml'
    
    print(f"Loading scenario from '{xml_file_path}'...")
    if not mission_manager.initialize_uavs_from_xml(xml_file_path):
        print("Failed to initialize scenario from XML. Aborting test.")
        return

    # Check if we have enough entities to run the test
    if len(mission_manager.uav_dict) < 1:
        print("Error: Not enough UAVs loaded from XML to perform the test (need at least 1).")
        return
    
    if not mission_manager.attack_targets:
        print("Error: No enemy targets loaded from XML to perform the test.")
        return

    # 2. Select IDs from the loaded data
    all_uav_ids = list(mission_manager.uav_dict.keys())
    uav_id = random.choice(all_uav_ids)
    enemy_target_id = random.choice(mission_manager.attack_targets).id
    
    print(f"Selected UAV ID: {uav_id}")
    print(f"Selected Enemy Target ID: {enemy_target_id}\n")

    # 3. Initialize TCP Client
    SERVER_HOST = '10.66.1.86'
    SERVER_PORT = 13334
    CLIENT_IP = '10.66.1.102'
    
    tcp_client = TCPClient(host=SERVER_HOST, port=SERVER_PORT, client_ip=CLIENT_IP)

    print("Connecting to the server...")
    if not tcp_client.connect():
        print("Failed to connect to the server. Message was not sent.")
        return
    
    try:
        print("Connection successful.\n")
        
        # ========== 测试场景1: UAV攻击敌方目标 ==========
        print("--- Test Case 1: UAV攻击敌方目标 (droneIsDeath=1) ---")
        destruction_message_1 = {
            "key": "DestoryEntity",
            "name": "ResearchGroup6",
            "timestamp": time.time(),
            "agents": [
                {
                    "drone_id": uav_id,  # UAV作为攻击者
                    "target_id": 2499,  # 敌方目标被摧毁
                    "droneIsDeath": 1  # UAV会死亡
                }
            ]
        }
        
        _save_data_to_json(destruction_message_1, "communication/json/ResearchGroup6ResultTest_UAV_Attack.json")
        print(f"Message 1: UAV-{uav_id} 攻击敌方目标 {enemy_target_id}, droneIsDeath=1")
        print("Sending message 1...")
        tcp_client.send_json(destruction_message_1)
        print("Message 1 sent successfully.\n")
        time.sleep(1)  # 等待消息处理
        
        # ========== 测试场景2: 敌方目标攻击UAV ==========
        print("--- Test Case 2: 敌方目标攻击UAV (droneIsDeath=0) ---")
        # 选择另一个UAV作为目标（避免重复）
        remaining_uav_ids = [uid for uid in all_uav_ids if uid != uav_id]
        if remaining_uav_ids:
            target_uav_id = random.choice(remaining_uav_ids)
        else:
            target_uav_id = uav_id  # 如果没有其他UAV，使用同一个
        
        destruction_message_2 = {
            "key": "DestoryEntity",
            "name": "ResearchGroup6",
            "timestamp": time.time(),
            "agents": [
                {
                    "drone_id": 2204,  # 敌方目标作为攻击者
                    "target_id": target_uav_id,  # UAV被摧毁
                    "droneIsDeath": 0  # 敌方目标不会死亡
                }
            ]
        }
        
        # _save_data_to_json(destruction_message_2, "communication/json/ResearchGroup6ResultTest_Enemy_Attack.json")
        # print(f"Message 2: 敌方目标 {enemy_target_id} 攻击UAV-{target_uav_id}, droneIsDeath=0")
        # print("Sending message 2...")
        # tcp_client.send_json(destruction_message_2)
        # print("Message 2 sent successfully.\n")
        # time.sleep(1)  # 等待消息处理
        
    finally:
        print("Disconnecting from the server.")
        tcp_client.disconnect()

    print("\n=== Destruction Interface Test Finished ===")


if __name__ == '__main__':
    test_destruction_message()
