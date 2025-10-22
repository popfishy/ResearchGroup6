#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import json
import time
import logging
import numpy as np
from datetime import datetime

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(f'tcp_client_comm_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


class NumpyEncoder(json.JSONEncoder):
    """Custom JSON encoder for numpy data types."""
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        return super(NumpyEncoder, self).default(obj)


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
            logger.info(f"Connecting to server at {self.host}:{self.port}...")
            self.socket.connect((self.host, self.port))
            self.connected = True
            logger.info("Successfully connected to the server.")

            # TODO 发送标识信息
            group_id = "ResearchGroup6*"
            logger.info(f"发送组标识: {group_id}")
            group_id_bytes = group_id.encode('utf-8')
            self.socket.sendall(group_id_bytes)
            
            # 等待一小段时间确保标识发送完成
            time.sleep(0.1)
            
            # 发送客户端信息
            # client_info = {
            #     "type": "client_info",
            #     "client_ip": self.client_ip,
            #     "timestamp": time.time(),
            #     "message": "课题六算法客户端已连接"
            # }
            # self.send_message(client_info)

            return True
        except Exception as e:
            logger.error(f"Failed to connect to the server: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """Closes the connection to the server."""
        self.connected = False
        if self.socket:
            try:
                self.socket.close()
                logger.info("Disconnected from the server.")
            except Exception as e:
                logger.error(f"Error while disconnecting: {e}")
        self.socket = None

    def send_json(self, data: dict):
        """
        Serializes a dictionary to JSON and sends it to the server.
        A '*' is appended as a message delimiter.

        :param data: The dictionary to send.
        :return: True if sending was successful, False otherwise.
        """
        if not self.connected or not self.socket:
            logger.warning("Cannot send data: not connected to the server.")
            return False
        
        try:
            # ensure_ascii=False
            message = json.dumps(data, ensure_ascii=False)
            if not message.endswith('*'):
                message += '*'
            
            self.socket.sendall(message.encode('utf-8'))
            logger.info(f"Sent message: {message[:100]}...")
            return True
        except Exception as e:
            logger.error(f"Failed to send message: {e}")
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
                logger.warning("Connection closed by the server.")
                self.disconnect()
                return []

            self.data_buffer += raw_data.decode('utf-8')
        except socket.timeout:
            # This is expected when no new data is available
            return []
        except Exception as e:
            logger.error(f"Failed to receive data: {e}")
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
                    logger.info(f"Received message: {part[:100]}...")
                except json.JSONDecodeError as e:
                    logger.warning(f"JSON decode error for message part: '{part[:100]}...'. Error: {e}")
        
        return messages

if __name__ == '__main__':
    # Example usage
    # This block will only run when the script is executed directly
    # In a real application, you would import the TCPClient class.
    
    SERVER_HOST = '10.66.1.93'  # Change to your server's IP
    SERVER_PORT = 13334         # Change to your server's port