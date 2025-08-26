#!/usr/bin/env python
import rospy
import socket
import struct
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from std_msgs.msg import Header

class ActionReceiver:
    def __init__(self):
        rospy.init_node('action_receiver', anonymous=True)
        
        # ROS Publisher
        self.pub_actions = rospy.Publisher('/uav_actions', PoseStamped, queue_size=10)
        self.pub_yaw = rospy.Publisher('/aim_yaw', Float32, queue_size=10)
        
        # Socket配置
        self.host = '0.0.0.0'
        self.port = 9998
        self.sock = None
        self.conn = None
        
        # 接收频率控制 (10Hz)
        self.rate = rospy.Rate(10)
        
        # 设置socket超时(秒)
        self.socket_timeout = 1.0

    def initialize_socket(self):
        """初始化socket连接"""
        if self.sock:
            self.sock.close()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(self.socket_timeout)
        self.sock.bind((self.host, self.port))
        self.sock.listen(1)
        rospy.loginfo(f"Waiting for connection on {self.host}:{self.port}...")

    def convert_to_posestamped(self, action_array):
        """将6维动作数据转换为PoseStamped消息"""
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "uav_frame"
        
        msg.pose.position.x = action_array[0]
        msg.pose.position.y = action_array[1]
        msg.pose.position.z = action_array[2]
        
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        
        return msg

    def process_connection(self):
        """处理单个连接会话"""
        try:
            self.conn.settimeout(self.socket_timeout)
            while not rospy.is_shutdown():
                try:
                    # 1. 接收数据长度
                    data_len_bytes = self.conn.recv(4)
                    if not data_len_bytes:
                        rospy.loginfo("Connection closed by client")
                        break
                    
                    data_len = struct.unpack('!I', data_len_bytes)[0]
                    
                    # 2. 接收实际数据
                    data = b''
                    while len(data) < data_len and not rospy.is_shutdown():
                        try:
                            packet = self.conn.recv(data_len - len(data))
                            if not packet:
                                break
                            data += packet
                        except socket.timeout:
                            continue
                    
                    if len(data) != data_len:
                        rospy.logwarn("Incomplete data received")
                        continue
                    
                    # 3. 处理数据
                    try:
                        action_chunk = np.frombuffer(data, dtype=np.float64).reshape(-1, 4)
                        rospy.loginfo(f"Received {len(action_chunk)} actions")
                        print(action_chunk)
                        
                        # 4. 发布数据
                        for action in action_chunk:
                            if rospy.is_shutdown():
                                break
                            pose_msg = self.convert_to_posestamped(action[0:3])
                            aim_yaw = action[3]
                            self.pub_actions.publish(pose_msg)
                            self.pub_yaw.publish(Float32(aim_yaw))
                            self.rate.sleep()
                            
                    except Exception as e:
                        rospy.logerr(f"Data processing error: {e}")
                        continue
                        
                except socket.timeout:
                    continue
                except socket.error as e:
                    rospy.logerr(f"Socket error: {e}")
                    break
                    
        except Exception as e:
            rospy.logerr(f"Connection error: {e}")
        finally:
            if self.conn:
                self.conn.close()
            self.conn = None

    def run(self):
        """主运行循环"""
        self.initialize_socket()
        
        while not rospy.is_shutdown():
            try:
                # 等待新连接
                self.conn, addr = self.sock.accept()
                rospy.loginfo(f"Connected to {addr}")
                
                # 处理连接
                self.process_connection()
                
                # 连接断开后重新初始化socket
                self.initialize_socket()
                
            except socket.timeout:
                continue
            except Exception as e:
                rospy.logerr(f"Main loop error: {e}")
                if not rospy.is_shutdown():
                    rospy.sleep(1)  # 防止错误循环占用CPU
                    self.initialize_socket()

    def shutdown(self):
        """清理资源"""
        if self.conn:
            self.conn.close()
        if self.sock:
            self.sock.close()
        rospy.loginfo("Receiver shutdown complete")

if __name__ == '__main__':
    receiver = ActionReceiver()
    rospy.on_shutdown(receiver.shutdown)
    try:
        receiver.run()
    except rospy.ROSInterruptException:
        pass
