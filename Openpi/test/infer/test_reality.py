import json
import logging
import socket
import struct
import threading
import time
from datetime import datetime
import imageio.v2 as imageio
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
# from openpi_client import websocket_client_policy as _websocket_client_policy
from openpi_client import zmq_client_policy as _zmq_client_policy
import cv2
import os

# 全局变量存储接收到的数据
first_image = None  # 存储第一张图像（恒定）
current_image = None  # 存储当前图像
current_odom = None  # 存储当前odom数据
received_data_lock = threading.Lock()  # 线程锁保证数据安全
# last_state = [0, 0, 0, 0, 0, 0]
last_state = None
first_state = None

# 目标主机配置
# TARGET_IP = "10.1.1.64"
TARGET_IP = "192.168.100.104"
# TARGET_IP = "192.168.101.35"
TARGET_PORT = 9998 

def image_callback(image_data, image_format):
    """接收图像数据的回调函数"""
    global first_image, current_image
    with received_data_lock:
        # raw_image = (image_data.copy(), image_format)
        # resized_image = cv2.resize(raw_image, (256, 256))
        raw_image = image_data.copy()
        resized_image = cv2.resize(raw_image, (256, 256))
        if first_image is None:  # 保存第一张接收到的图像
            first_image = resized_image
        current_image = resized_image # 更新当前图像
        
        cv2.imshow("image", resized_image)
        cv2.waitKey(1)
        

def odom_callback(odom_data):
    """接收odom数据的回调函数"""
    global current_odom, first_state, last_state
    with received_data_lock:
        raw_odom = odom_data.copy()
        x, y, z, w = raw_odom[3:7]
        row, pitch, yaw = quaternion_to_euler(x, y, z, w)
        if first_state is None:
            first_state = np.array([raw_odom[0], raw_odom[1], raw_odom[2], row, pitch, yaw])
            last_state = np.array([raw_odom[0], raw_odom[1], raw_odom[2], row, pitch, yaw])
        current_odom = np.array([raw_odom[0], raw_odom[1], raw_odom[2], row, pitch, yaw])  # 更新当前odom数据
        # print("cur_odom: ", current_odom[0], " ",current_odom[1], " ",current_odom[2])
        # current_odom = odom_data.copy()

def quaternion_to_euler(x, y, z, w):
        """将四元数转换为欧拉角 (roll, pitch, yaw)"""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        
        return roll, pitch, yaw

def send_actions_to_target(action_chunk):
    """将action_chunk发送到目标主机"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((TARGET_IP, TARGET_PORT))
            
            # 将action_chunk转换为字节流
            action_bytes = action_chunk.tobytes()
            
            # 发送数据长度（4字节网络序）
            data_len = len(action_bytes)
            s.send(struct.pack('!I', data_len))
            
            # 发送实际数据
            s.sendall(action_bytes)
            
            logging.info(f"Sent {data_len} bytes of action data to {TARGET_IP}:{TARGET_PORT}")
    except Exception as e:
        logging.error(f"Failed to send actions: {e}")

def get_prompt_from_task_index(tasks_jsonl_path, task_index):
    """从tasks.jsonl获取prompt"""
    try:
        with open(tasks_jsonl_path, 'r') as f:
            for line in f:
                data = json.loads(line)
                if data.get('task_index') == task_index:
                    return data.get('task')
    except FileNotFoundError:
        logging.error(f"tasks.jsonl文件未找到: {tasks_jsonl_path}")
    return None

def calculate_trajectory_deviation(traj1, traj2):
    """计算轨迹偏差"""
    pos1, rot1 = traj1[:, :3], traj1[:, 3:]
    pos2, rot2 = traj2[:, :3], traj2[:, 3:]
    positional_errors = np.linalg.norm(pos1 - pos2, axis=1)
    positional_mae = np.mean(positional_errors)
    rotational_errors = np.abs(rot1 - rot2)
    rotational_mae = np.mean(rotational_errors)
    return positional_mae, rotational_mae, positional_errors

def visualize_trajectories_3d(traj1, traj2):
    """可视化3D轨迹"""
    pos1 = traj1[:, :3]
    pos2 = traj2[:, :3]
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(pos1[:, 1], pos1[:, 0], pos1[:, 2], 'o-', label='Original', color='blue')
    ax.plot(pos2[:, 1], pos2[:, 0], pos2[:, 2], 'o-', label='Inferred', color='red')
    plt.show()

def main():
    global last_state
    global first_state
    # 初始化socket接收数据
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('0.0.0.0', 9999))
        s.listen(1)
        print("Waiting for connection...")
        conn, addr = s.accept()
        
        # 启动数据接收线程
        receiver_thread = threading.Thread(target=receive_data, args=(conn,))
        receiver_thread.daemon = True
        receiver_thread.start()

    # 等待第一张图像到达
    while first_image is None:
        time.sleep(0.1)
    
    # 主处理逻辑
    HOST = "127.0.0.1"
    PORT = 3000
    # client = _websocket_client_policy.WebsocketClientPolicy(HOST, PORT)
    client = _zmq_client_policy.ZMQClientPolicy(HOST, PORT)

    current_time = time.time()
    last_active_time = time.time()
    
    while True:
        with received_data_lock:
            current_time = time.time()
            time_elapsed = current_time - last_active_time

            if last_state is None or first_state is None:
                distance = 100
            else:
                distance = np.linalg.norm(current_odom[0:2] - last_state[0:2], axis=0)
                
            print(f"current_odom:{current_odom[0:3]}")
            print(f"last_state:{last_state[0:3]}")
            print(f"distance:{distance}")

            # if((distance < 0.2) or (time_elapsed > 100.0)):
            if(distance < 0.4):

                last_active_time = current_time

                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
                # 保存first_image
                if first_image is not None:
                    first_img_path = f"./trail/first_image/first_{timestamp}.jpeg"
                    os.makedirs(os.path.dirname(first_img_path), exist_ok=True)
                    imageio.imwrite(first_img_path, first_image)
                
                # 保存current_image
                if current_image is not None:
                    current_img_path = f"./trail/current_image/current_{timestamp}.jpeg"
                    os.makedirs(os.path.dirname(current_img_path), exist_ok=True)
                    imageio.imwrite(current_img_path, current_image)
                
                # 保存current_odom到文件(追加模式)
                if current_odom is not None:
                    odom_path = "./trail/odom.txt"
                    os.makedirs(os.path.dirname(odom_path), exist_ok=True)
                    with open(odom_path, 'a') as f:  # 'a'表示追加模式
                        f.write(f"{timestamp}: {current_odom.tolist()}\n")
                
                prompt = get_prompt_from_task_index("/home/diff/VLA/Openpi/test/infer/test_data/meta/tasks.jsonl", 0)
                
                # 构造element
                element = {
                    "observation/image": first_image,
                    "observation/wrist_image": current_image,
                    "observation/state": (current_odom-first_state),
                    "prompt": prompt
                }
                
                print("\n===========================================================")
                print(prompt)
                print("===========================================================\n")

                # # 发送推理请求
                # try:
                #     action_chunk = client.infer(element)["actions"]
                #     print(action_chunk)
                try:
                    result = client.infer(element)
                    action_chunk = result["actions"]
                    last_state = action_chunk[4]
                    
                    # 将action_chunk发送到目标主机
                    if isinstance(action_chunk, np.ndarray):
                        send_actions_to_target(action_chunk)

                except Exception as e:
                    logging.error(f"Inference error: {e}")

        time.sleep(0.1)  # 控制处理频率

def deserialize_odometry(data):
    # 跳过消息头（示例，实际需根据完整格式解析）
    pos = np.frombuffer(data[84:108], dtype=np.float64)  # position.x/y/z
    quat = np.frombuffer(data[108:140], dtype=np.float64)  # orientation.x/y/z/w
    return np.concatenate([pos, quat])

def receive_data(conn):
    """接收数据的线程函数"""
    while True:
        try:
            # 接收消息类型 (1=image, 2=odom)
            msg_type = conn.recv(1)
            if not msg_type:
                break
                
            msg_type = ord(msg_type)  # Convert to integer
            
            if msg_type == 1:  # 图像数据
                # 接收格式长度和格式字符串
                fmt_len_bytes = conn.recv(4)
                if len(fmt_len_bytes) != 4:
                    break
                fmt_len = struct.unpack('!I', fmt_len_bytes)[0]  # Network byte order
                
                image_format = conn.recv(fmt_len).decode('utf-8')
                if len(image_format) != fmt_len:
                    break
                
                # 接收图像数据长度和数据
                data_len_bytes = conn.recv(4)
                if len(data_len_bytes) != 4:
                    break
                data_len = struct.unpack('!I', data_len_bytes)[0]  # Network byte order
                
                data = b''
                while len(data) < data_len:
                    packet = conn.recv(data_len - len(data))
                    if not packet:
                        break
                    data += packet

                try:
                    # image = imageio.imread(data, format=image_format)
                    image = imageio.imread(data, format="jpeg")
                    # print("Received image")
                    image_callback(image, image_format)
                except Exception as e:
                    logging.error(f"Image decoding error: {e}")
                
                
                
            # elif msg_type == 2:  # odom
            #     data_len_bytes = conn.recv(4)
            #     data_len = struct.unpack('!I', data_len_bytes)[0]
            #     data = conn.recv(data_len)
            #     if len(data) == 705:
            #         odom_data = deserialize_odometry(data)
            #         odom_callback(odom_data)
            #     else:
            #         print("Size Error")
            #         pass
            elif msg_type == 2:  # odom数据
                try:
                    # 接收固定56字节的odom数据 (7个double)
                    data = conn.recv(56)
                    if len(data) != 56:
                        print(f"Size Error: expected 56 bytes, got {len(data)} bytes")
                        continue
                    
                    # 解析7个double值 [x,y,z, qx,qy,qz,qw]
                    odom_data = np.frombuffer(data, dtype=np.float64)
                    
                    # 调试打印
                    # print(f"Received odom: pos=({odom_data[0]:.3f}, {odom_data[1]:.3f}, {odom_data[2]:.3f}) "
                    #       f"orient=({odom_data[3]:.3f}, {odom_data[4]:.3f}, {odom_data[5]:.3f}, {odom_data[6]:.3f})")
                    
                    # 调用回调函数
                    odom_callback(odom_data)
                    
                except Exception as e:
                    print(f"Error processing odometry: {str(e)}")
                    continue

             

        except Exception as e:
            logging.error(f"Error: {e}")
            break

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
