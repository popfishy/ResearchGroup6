#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import json
import time
import numpy as np

class TCPClient:
    """A generic TCP client for sending and receiving JSON messages."""
    def __init__(self, host, port, client_ip):
        """
        Initializes the TCP client.
        :param host: The server's hostname or IP address.
        :param port: The port used by the server.
        """
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        self.data_buffer = ""
        self.client_ip = client_ip

    def connect(self):
        """Establishes a connection to the TCP server."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10.0)  # 10-second timeout for connection attempt
            print(f"\n ******成功连接到 {self.host}:{self.port}******")
            self.socket.connect((self.host, self.port))
            self.connected = True

            # 发送组标识信息
            group_id = "ResearchGroup6*"
            print(f"发送组标识: {group_id}")
            group_id_bytes = group_id.encode('utf-8')
            self.socket.sendall(group_id_bytes)
            
            # 等待一小段时间确保标识发送完成
            time.sleep(0.1)
            
            return True
        except Exception as e:
            print(f"Failed to connect to the server: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """Closes the connection to the server."""
        self.connected = False
        if self.socket:
            try:
                self.socket.close()
                print("Disconnected from the server.")
            except Exception as e:
                print(f"Error while disconnecting: {e}")
        self.socket = None

    def send_json(self, data: dict):
        """
        Serializes a dictionary to JSON and sends it to the server.
        A '*' is appended as a message delimiter.

        :param data: The dictionary to send.
        :return: True if sending was successful, False otherwise.
        """
        if not self.connected or not self.socket:
            print("Cannot send data: not connected to the server.")
            return False
        
        try:
            message = json.dumps(data, ensure_ascii=False)
            if not message.endswith('*'):
                message += '*'
            
            self.socket.sendall(message.encode('utf-8'))
            # print(f"Sent message: {message[:100]}...")
            return True
        except Exception as e:
            print(f"Failed to send message: {e}")
            self.connected = False  # Assume connection is lost on send error
            return False

    def receive_json_messages(self) -> list:
        """
        Receives data, handles message framing using '*' as a delimiter,
        and parses complete JSON messages.

        :return: A list of successfully parsed JSON objects (dictionaries).
        """
        if not self.connected or not self.socket:
            return []

        try:
            self.socket.settimeout(0.1)  # Use a short timeout for non-blocking reads
            raw_data = self.socket.recv(4096)
            if not raw_data:
                # Connection closed by the server
                print("Connection closed by the server.")
                self.disconnect()
                return []

            self.data_buffer += raw_data.decode('utf-8')
        except socket.timeout:
            # Expected when no data is available
            return []
        except Exception as e:
            print(f"Failed to receive data: {e}")
            self.disconnect()
            return []

        # Process the buffer to extract complete messages
        messages = []
        if '*' in self.data_buffer:
            parts = self.data_buffer.split('*')
            self.data_buffer = parts[-1]  # Keep the last, possibly incomplete, part
            complete_parts = parts[:-1]

            for part in complete_parts:
                if not part.strip():
                    continue
                try:
                    message_data = json.loads(part)
                    messages.append(message_data)
                    print(f"Received message: {part[:100]}...")
                except json.JSONDecodeError as e:
                    print(f"JSON decode error for message part: '{part[:100]}...'. Error: {e}")
        
        return messages

if __name__ == '__main__':
    # Example usage
    SERVER_HOST = '10.66.1.93'  # Change to your server's IP  
    SERVER_PORT = 13334         # Change to your server's port
    CLIENT_IP = '127.0.0.1'     # Replace with actual client IP if needed

    client = TCPClient(SERVER_HOST, SERVER_PORT, CLIENT_IP)
    if client.connect():
        try:
            # Example: send a test message
            test_msg = {"cmd": "status_request", "from": CLIENT_IP}
            client.send_json(test_msg)

            # Listen for incoming messages for 10 seconds
            start_time = time.time()
            while time.time() - start_time < 10:
                msgs = client.receive_json_messages()
                for msg in msgs:
                    print("Parsed message:", msg)
                time.sleep(0.01)  # Small delay to avoid busy loop
        finally:
            client.disconnect()
    else:
        print("Could not connect to server.")