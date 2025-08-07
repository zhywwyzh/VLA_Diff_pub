import logging
import time
import zmq
from typing import Dict

from openpi_client import base_policy as _base_policy
from openpi_client import msgpack_numpy

class ZMQClientPolicy(_base_policy.BasePolicy):

    def __init__(self, host: str = "127.0.0.1", port: int = 3000, timeout: int = 20000):
        self._host = host
        self._port = port
        self._packer = msgpack_numpy.Packer()
        
        # Create ZMQ context
        self._context = zmq.Context()
        
        # Create PUB socket for sending observations
        self._pub_socket = self._context.socket(zmq.PUB)
        self._pub_socket.setsockopt(zmq.SNDTIMEO, timeout)
        self._pub_socket.setsockopt(zmq.LINGER, 0)
        
        # Create SUB socket for receiving actions
        self._sub_socket = self._context.socket(zmq.SUB)
        self._sub_socket.setsockopt(zmq.RCVTIMEO, timeout)
        self._sub_socket.setsockopt(zmq.LINGER, 0)
        self._sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages
        
        # Connect to server
        self._connect()

    def _connect(self):
        # Connect PUB socket to server's SUB port (for sending observations)
        pub_connection_str = f"tcp://{self._host}:{self._port + 1}"
        logging.info(f"Connecting PUB socket to server: {pub_connection_str}")
        self._pub_socket.connect(pub_connection_str)
        
        # Connect SUB socket to server's PUB port (for receiving actions)
        sub_connection_str = f"tcp://{self._host}:{self._port}"
        logging.info(f"Connecting SUB socket to server: {sub_connection_str}")
        self._sub_socket.connect(sub_connection_str)

    def infer(self, obs: Dict) -> Dict:
        try:
            # Send observation to server
            data = self._packer.pack(obs)
            self._pub_socket.send(data)
            
            # Wait for action from server
            response = self._sub_socket.recv()
            return msgpack_numpy.unpackb(response)
            
        except zmq.Again:
            logging.error("Request timed out. Reconnecting...")
            self._connect() 
            return self.infer(obs)
        except Exception as e:
            logging.error(f"Inference request failed: {str(e)}")
            raise

    def close(self):
        self._pub_socket.close()
        self._sub_socket.close()
        self._context.term()