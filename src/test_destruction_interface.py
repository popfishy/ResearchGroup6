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
    A test case to send a single "DestoryEntity" message to the server.
    It loads the scenario from XML to get valid attacker and target IDs.
    """
    print("--- Starting Destruction Interface Test ---")

    # 1. Initialize the Mission Manager and load the scenario
    mission_manager = SwarmMissionManager()
    xml_file_path = '山地丛林.xml'
    
    print(f"Loading scenario from '{xml_file_path}'...")
    if not mission_manager.initialize_uavs_from_xml(xml_file_path):
        print("Failed to initialize scenario from XML. Aborting test.")
        return

    # Check if we have enough entities to run the test
    if len(mission_manager.uav_dict) < 2:
        print("Error: Not enough UAVs loaded from XML to perform the test (need at least 2).")
        return
    
    if not mission_manager.attack_targets:
        print("Error: No enemy targets loaded from XML to perform the test.")
        return

    # 2. Select attacker and target IDs from the loaded data
    all_uav_ids = list(mission_manager.uav_dict.keys())
    attacker_ids = random.sample(all_uav_ids, 2)
    target_id = random.choice(mission_manager.attack_targets).id
    
    print(f"Selected Attackers (drone_id): {attacker_ids[0]}, {attacker_ids[1]}")
    print(f"Selected Target (target_id): {target_id}")

    # 3. Construct the destruction message
    destruction_message = {
        "key": "DestoryEntity",
        "name": "ResearchGroup6",
        "timestamp": time.time(),
        "agents": [
            {
                "drone_id": 1298, 
                "target_id": 1280
            }
        ]
    }

    _save_data_to_json(destruction_message, "communication/json/ResearchGroup6ResultTest1.json")
    # 4. Initialize TCP Client and send the message
    SERVER_HOST = '10.66.1.93'
    SERVER_PORT = 13334
    CLIENT_IP = '10.66.1.192' # Make sure this is the correct IP for the client machine
    
    tcp_client = TCPClient(host=SERVER_HOST, port=SERVER_PORT, client_ip=CLIENT_IP)

    print("\nConnecting to the server...")
    if tcp_client.connect():
        try:
            print("Connection successful. Sending destruction message...")
            tcp_client.send_json(destruction_message)
            print("Message sent successfully.")
            # Wait a moment before disconnecting to ensure message is processed
            time.sleep(1)
        finally:
            print("Disconnecting from the server.")
            tcp_client.disconnect()
    else:
        print("Failed to connect to the server. Message was not sent.")

    print("\n--- Destruction Interface Test Finished ---")


if __name__ == '__main__':
    test_destruction_message()
