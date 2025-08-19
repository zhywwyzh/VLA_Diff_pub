#!/usr/bin/env python3

import json
import logging
import sys
import os
import re

import imageio.v2 as imageio
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import time
from sensor_msgs.msg import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import String
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import cv2
from cv_bridge import CvBridge
import threading
from utils.vlm.openai_serve import open_serve
from utils.param import COMMAND_TYPE

from openpi_client import websocket_client_policy as _websocket_client_policy

class UAVPolicyNode(Node):
    def __init__(self):
        # ROS2节点初始化
        super().__init__('uav_policy_node')

        # 声明参数
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8000)
        self.declare_parameter('replan_steps', 10)
        # self.declare_parameter('prompt', 'Navigate to the target location')

        # 创建CV桥接器
        self.bridge = CvBridge()

        # 存储最新的状态的图像
        self.current_state = None        
        self.current_image = None
        self.current_wrist_image = None
        self.state_lock = threading.Lock()
        self.image_lock = threading.Lock()
        self.depth_lock = threading.Lock()
        self.first_image = None
        self.first_image_received = False
        self.count = 0
        self.save_count = 0  # 添加保存计数器
        self.image_save_count = 0  # 添加图像保存计数器
        self.last_state = [0, 0, 1, 0, 0, 0]
        self.last_inference_time = None  # 添加上次推理时间记录
        self.inference_timeout = 5.0  # 设置推理超时时间（秒），可以根据需要调整
        self.rgb_image = None
        self.depth_image = None
        self.depth_info = None
        self.save = None
        self.plan = True
        self.command_content = ["door", (0,0,1,0,0,0), "fridge"]
        self.first_command = False
        self.command_type = COMMAND_TYPE.WAIT
        self.task_id_mllm = []
        self.task_id_vlm = []
        self.pub_goal = False

        # 创建图像保存目录
        self.image_save_dir = '/home/zhywwyzh/workspace/VLA_Diff/Openpi/test/infer/trail/saved_images'
        os.makedirs(self.image_save_dir, exist_ok=True)

        # 创建模型输入图像保存目录
        self.model_input_dir = '/home/zhywwyzh/workspace/VLA_Diff/Openpi/test/infer/trail/model_input_images'
        os.makedirs(self.model_input_dir, exist_ok=True)

        # 订阅无人机状态和图像话题
        self.odom_sub = self.create_subscription(Odometry,
                                                '/drone_0_visual_slam/odom',
                                                self.odom_callback,
                                                10)

        self.image_sub = self.create_subscription(CompressedImage,
                                                '/camera1/color/image/compressed',
                                                self.image_callback,
                                                10)

        self.depth_sub = self.create_subscription(CompressedImage,
                                                '/camera1/depth/image/compressed',
                                                self.depth_callback,
                                                10)

        self.depth_info_sub = self.create_subscription(CameraInfo,
                                                       'camera1/depth/info',
                                                       self.depth_info_callback,
                                                       10)

        # 订阅指令需求
        self.command_type_sub = self.create_subscription(Int32,
                                               "command/type",
                                               self.command_type_callback,
                                               10)
        
        # self.command_content_sub = self.create_subscription(String,
        #                                         "command/content",
        #                                         self.command_content_callback,
        #                                         10)

        # 发布无人机动作
        self.action_pub = self.create_publisher(PoseStamped,
                                                '/goal',
                                                10)
        
        # 初始化WebSocker客户端
        self.client = None
        self.prompt = None

        # 轨迹存储
        self.original_trajectory = []
        self.inferred_trajectory = []

        self.tasks_jsonl_path = "/home/zhywwyzh/workspace/VLA_Diff/Openpi/test/infer/test_data/meta/tasks.jsonl"
        self.task_index = 2

        # 从参数服务器获取配置
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.replan_steps = self.get_parameter('replan_steps').get_parameter_value().integer_value
        # self.prompt = self.get_parameter('prompt').get_parameter_value().string_value

        # self.prompt = self.get_prompt_from_task_index()
        # if self.prompt is None:
        #     self.prompt = "无人机实时轨迹控制"
        #     self.get_logger().warn("使用默认提示")
        # print(f"使用提示: {self.prompt}")

        # 连接WebSocket服务器
        # print("尝试连接服务器")
        # self.connect_websocket()
        # print("已连接至服务器")

    def get_prompt_from_task_index(self):
        """
        从 tasks.jsonl 文件中根据 task_index 查找并返回对
        self.task_index = 2应的 prompt。
        """
        try:
            if not os.path.exists(self.tasks_jsonl_path):
                logging.error(f"tasks.jsonl 文件不存在: {self.tasks_jsonl_path}")
                return None
                
            with open(self.tasks_jsonl_path, 'r') as f:
                for line in f:
                    data = json.loads(line)
                    if data.get('task_index') == self.task_index:
                        return data.get('task')
        except FileNotFoundError:
            logging.error(f"tasks.jsonl 文件未找到: {self.tasks_jsonl_path}")
            return None
        except json.JSONDecodeError as e:
            logging.error(f"解析 tasks.jsonl 文件时出错: {self.tasks_jsonl_path}, 错误: {e}")
            return None
        except Exception as e:
            logging.error(f"读取 tasks.jsonl 文件时出错: {e}")
            return None
        
        logging.warning(f"在 {self.tasks_jsonl_path} 中未找到 task_index {self.task_index} 对应的任务。")
        return None
    
    def connect_websocket(self):
        """连接WebSocket服务器"""
        try:
            logging.info(f"正在连接到服务器 ws://{self.host}:{self.port}")
            self.client = _websocket_client_policy.WebsocketClientPolicy(self.host, self.port)
        except Exception as e:
            logging.error(f"连接WebSocket服务器失败: {e}")
            self.get_logger().error("WebSocket连接失败，节点即将关闭")
            rclpy.shutdown()

    def odom_callback(self, msg):
        # print(f"state_lock:{self.state_lock.locked()}")
        """处理无人机状态回调"""
        with self.state_lock:
            # 从Odometry消息中提取位置和姿态
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            
            # 将四元数转换为欧拉角
            roll, pitch, yaw = self.quaternion_to_euler(orientation.x, 
                                                        orientation.y, 
                                                        orientation.z, 
                                                        orientation.w)
            
            # 更新当前状态: [x, y, z, roll, pitch, yaw]
            self.current_state = np.array([
                position.x, position.y, position.z,
                roll, pitch, yaw
            ])
            # print(f"当前状态: {self.current_state}")
    
    def image_callback(self, msg):
        """处理图像回调"""
        try:
            with self.image_lock:
            # 将压缩图像转换为OpenCV格式
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                self.rgb_image = cv_image.copy()
                
        except Exception as e:
            logging.error(f"图像处理错误: {e}")

    def depth_callback(self, msg):
        """处理深度图回调"""
        try:
            with self.depth_lock:
                np_arr = np.frombuffer(msg.data, np.uint8)
                depth_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # 保留原始深度
                # print(f"depth_image:{depth_image}")
                
                depth_m = depth_image[:,:,0]

                depth_32 = depth_m.astype(np.float32) / 255.0 * 5.0
                depth_32[depth_32 == 0] = 16.0

                # 存储到类变量，供其他线程使用
                self.depth_image = depth_32.copy()
                # print(f"depth_image:{self.depth_image}")                  

        except Exception as e:
            logging.error(f"深度图处理错误: {e}")

    def command_type_callback(self, msg):
        """处理指令需求回调"""
        self.command_type = msg.data
        print(f"当前指令类型: {self.command_type}")

    def command_content_callback(self, msg):
        """处理指令内容回调"""
        self.command_content.append(json.loads(msg.data))

    def depth_info_callback(self, msg):
        """处理深度相机参数回调"""
        if self.depth_info is None:
            height = msg.height
            width = msg.width
            fx = msg.k[0]
            fy = msg.k[4]
            cx = msg.k[2]
            cy = msg.k[5]
            self.depth_info = { 'height': height, 'width': width, 'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy}

    def quaternion_to_euler(self, x, y, z, w):
        """将四元数转换为欧拉角 (roll, pitch, yaw)"""
        # 这里实现了四元数到欧拉角的转换
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
    
    def prepare_image_for_inference(self, image):
        """准备推理用的图像，确保尺寸为256x256"""
        if image is None:
            return None
        
        # 确保图像是256x256
        if image.shape[:2] != (256, 256):
            image = cv2.resize(image, (256, 256))
        
        return image
    
    def publish_action(self, action):
        """发布动作到ROS话题"""
        if len(action) < 3:
            logging.error("动作数据不足3个元素")
            return
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        # 设置位置
        pose_msg.pose.position.x = float(action[0])
        pose_msg.pose.position.y = float(action[1])
        pose_msg.pose.position.z = float(action[2])

        # 将欧拉角转换为四元数
        # qx, qy, qz, qw = self.euler_to_quaternion(action[3], action[4], action[5])
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 0.0
        self.action_pub.publish(pose_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """将欧拉角转换为四元数"""
        # 这里实现了欧拉角到四元数的转换
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qx, qy, qz, qw
    
    def bbox_center(self, results):
        if not results:
            raise ValueError("Results list is empty")
        bbox = results[0].get("bbox_2d", None)
        if bbox is None or len(bbox) != 4:
            raise ValueError("Invalid bbox_2d format")
        x1, y1, x2, y2 = bbox
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        return center_x, center_y
    
    def euler_rpy_to_R(self, roll, pitch, yaw):
        """R = Rz(yaw) @ Ry(pitch) @ Rx(roll)"""
        sr, cr = np.sin(roll),  np.cos(roll)
        sp, cp = np.sin(pitch), np.cos(pitch)
        sy, cy = np.sin(yaw),   np.cos(yaw)
        Rx = np.array([[1, 0, 0],
                    [0, cr, -sr],
                    [0, sr,  cr]], dtype=np.float64)
        Ry = np.array([[ cp, 0, sp],
                    [  0, 1,  0],
                    [-sp, 0, cp]], dtype=np.float64)
        Rz = np.array([[cy, -sy, 0],
                    [sy,  cy, 0],
                    [ 0,   0, 1]], dtype=np.float64)
        return Rz @ Ry @ Rx

    def _backproject(self, u, v, depth_value, fx, fy, cx, cy, mode="z"):
        d = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=np.float64)
        if mode == "z":  # projective depth
            Z = depth_value
            return d * Z
        elif mode == "range":  # ray length
            s = d / np.linalg.norm(d)
            return s * depth_value
        else:
            raise ValueError(f"Unknown depth mode: {mode}")

    def pixel_to_world(self, results, depth_scale=1.0, window=0, depth_mode="z"):
        # 取像素 & 深度（略，与你现有一致）...
        u, v = self.bbox_center(results)
        fx, fy, cx, cy = self.depth_info['fx'], self.depth_info['fy'], self.depth_info['cx'], self.depth_info['cy']

        ui, vi = int(round(u)), int(round(v))
        if window > 0:
            u0, u1 = max(0, ui - window), min(self.depth_image.shape[1] - 1, ui + window)
            v0, v1 = max(0, vi - window), min(self.depth_image.shape[0] - 1, vi + window)
            patch = self.depth_image[v0:v1+1, u0:u1+1].astype(np.float64)
            depth_raw = np.median(patch)
        else:
            depth_raw = float(self.depth_image[vi, ui])

        depth_val = depth_raw * depth_scale
        if not np.isfinite(depth_val) or depth_val <= 0:
            raise ValueError(f"无效深度 depth={depth_val}")

        # 相机像素 -> 相机坐标（根据 depth_mode 处理）
        cam_point_cv = self._backproject(u, v, depth_val, fx, fy, cx, cy, depth_mode)

        # 相机(OpenCV x右,y下,z前) -> 机器人(base_link x前,y左,z上)
        R_cam_to_robot = np.array([
            [ 0,  0,  1],   # x_robot =  z_cam
            [-1,  0,  0],   # y_robot = -x_cam
            [ 0, -1,  0]    # z_robot = -y_cam
        ], dtype=np.float64)
        cam_point_robot = R_cam_to_robot @ cam_point_cv

        # 机器人 -> 世界（若 self.current_state 的 RPY 是 世界->机器人，则转置）
        tx, ty, tz, roll, pitch, yaw = map(float, self.current_state.tolist())
        Rrw = self.euler_rpy_to_R(roll, pitch, yaw).T
        t_world = np.array([tx, ty, tz], dtype=np.float64) - [0.6, 0.0, 0.0]

        world_point = Rrw @ cam_point_robot + t_world
        return world_point

    def run_inference(self):
        """执行推理循环"""
        rate = self.create_rate(20)  # 20Hz
        waypoint = None
        print("任务开始")

        while rclpy.ok():
            if self.command_type == COMMAND_TYPE.STOP:
                print("任务终止")
                rclpy.shutdown()
                break

            # 检查是否有有效的状态和图像
            with self.depth_lock, self.image_lock, self.state_lock:
                # print(f"state_lock: {self.state_lock.locked()},image_lock:{self.image_lock.locked()}")
                # if (self.current_state is None or
                #     self.current_wrist_image is None or
                #     not self.first_image_received):
                #     print("等待中... 图像未准备好")
                #     rate.sleep()
                #     continue

                if self.rgb_image is None:
                    # print("等待中... RGB图像未准备好")
                    time.sleep(0.01)
                    # rate.sleep()
                    continue
                
                if self.current_state is None:
                    time.sleep(0.01)
                    continue
                
                if self.depth_image is None:
                    time.sleep(0.01)
                    # rate.sleep
                    continue

            state = self.current_state.copy()

            # if image is None or wrist_image is None:
            #     print("等待中... wrist图像未准备好")
            #     rate.sleep()
            #     continue

            distance = np.linalg.norm(state[0:2] - self.last_state[0:2], axis=0)
            
            # 检查是否满足推理条件：位置移动距离 OR 时间超时
            current_time = self.get_clock().now()
            should_infer = False
            # print("开始推理")
            # print(self.command_content[0])
            # print(isinstance(self.command_content[0], (list, tuple, np.ndarray)))

            # 条件1：位置移动距离小于阈值（原逻辑，表示已到达目标点附近）
            if distance < 0.01 and self.command_type == COMMAND_TYPE.WAIT:
                should_infer = True
                inference_reason = f"位置移动距离小于阈值: {distance:.4f} < 0.01"
                # print(f"first_command: {self.first_command}")
                if self.first_command:
                    self.first_command = False
                    print(f"到达任务点{waypoint}，准备执行下一步任务")
                    self.command_type = COMMAND_TYPE.WAIT
                    self.command_content.pop(0)
                    if not self.command_content:
                        print("所有任务完成，停止推理")
                        rclpy.shutdown()

            # 条件2：时间超时
            elif self.last_inference_time is not None and self.command_type == COMMAND_TYPE.WAIT:
                time_since_last_inference = (current_time - self.last_inference_time).nanoseconds / 1e9
                if time_since_last_inference > self.inference_timeout:
                    should_infer = True
                    inference_reason = f"时间超时: {time_since_last_inference:.2f}s > {self.inference_timeout}s"
                else:
                    print(f"等待中... 距离: {distance:.4f}, 时间间隔: {time_since_last_inference:.2f}s")

            # 进行推理
            else:
                print("准备进行推理")
                if isinstance(self.command_content[0], (list, tuple, np.ndarray)) and self.command_type == COMMAND_TYPE.GO:
                    print("开始规划")
                    self.first_command = True
                    print(f"当前任务内容: {np.array(self.command_content[0])}")
                    waypoint = np.array(self.command_content[0])[0:3]
                    print(f"当前任务点: {waypoint}")
                    print(f"Go to the point: {waypoint}")

                    self.pub_goal = True

                else:
                    # 第一次推理
                    should_infer = True
                    inference_reason = "首次推理"
                    self.first_command = True

            if should_infer and waypoint is None and self.command_type == COMMAND_TYPE.GO:
                try:
                    self.first_command = True
                    print(f"开始推理 - {inference_reason}")

                    # 保存输入到模型的图像
                    timestamp = self.get_clock().now().to_msg()

                    # 调用openai进行处理
                    result = open_serve(self.rgb_image, self.command_content[0])
                    print(f"推理结果: {result}")

                    # 更新推理时间记录
                    self.last_inference_time = current_time

                    # 生成waypoint
                    waypoint = self.pixel_to_world(result)

                    print(f"Go to the {self.command_content[0]}: {waypoint}")

                    self.pub_goal = True

                    # waypoint = [1.4371602535247803, 4.463369369506836, 0.0]

                except Exception as e:
                    logging.error(f"推理错误: {e}")

            # 发布动作
            if waypoint is not None and self.command_type == COMMAND_TYPE.GO and self.pub_goal:
                self.publish_action(waypoint)
                self.last_state = waypoint.copy()
                self.command_type = COMMAND_TYPE.WAIT
                self.pub_goal = False

            rate.sleep()


def main():
    logging.basicConfig(level=logging.INFO)

    rclpy.init()
    try:
        print("Program starting...")
        node = UAVPolicyNode()

        # 在单独的线程中运行推理循环
        inference_thread = threading.Thread(target=node.run_inference)
        inference_thread.start()

        # 主线程运行 ROS2 spin
        rclpy.spin(node)

        # 等待推理线程结束
        inference_thread.join()

    except KeyboardInterrupt:
        pass
    except Exception as e:
        logging.error(f"主程序错误: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
