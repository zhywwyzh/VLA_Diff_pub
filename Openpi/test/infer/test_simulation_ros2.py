#!/usr/bin/env python3

import json
import logging
import sys
import os
import re
import pdb

import imageio.v2 as imageio
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import time
from sensor_msgs.msg import Image

import rclpy
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
from utils.param import COMMAND_TYPE, VLA_STATE

from openpi_client import websocket_client_policy as _websocket_client_policy

from base_policy import BasePolicyNode

class UAVPolicyNode(BasePolicyNode):
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

        # 存储最新的状态和图像
        self.current_image = None
        self.current_wrist_image = None
        self.first_image = None
        self.first_image_received = False
        self.count = 0
        self.save_count = 0  # 添加保存计数器
        self.image_save_count = 0  # 添加图像保存计数器
        self.last_state = [0, 0, 1, 0, 0, 0]
        self.last_plan_time = None  # 添加上次推理时间记录
        self.inference_timeout = 5.0  # 设置推理超时时间（秒），可以根据需要调整
        self.save = None
        self.plan = True
        self.command_content = ["请前往左侧第一个饮水机右侧，给我图中的二维坐标，只给坐标，其他的什么都不要输出",
                                "找到右侧第一个门的位置，给我图中的二维坐标，只给坐标，其他的什么都不要输出",
                                '找到树的位置，给我图中的二维坐标，只给坐标，其他的什么都不要输出', 
                                "前进"]
        self.replan = False
        self.task_id = [1, 2, 2, 1]
        # self.command_content = ["fridge", (-16.146, 3.6, 0.877, 0, 0, 0), "fridge"]
        self.first_command = False
        self.command_type = COMMAND_TYPE.WAIT
        self.task_id_mllm = []
        self.task_id_vlm = []
        self.pub_goal = False
        self.arrival_distance = 0.1
        self.time_out = 10.0
        self.vla_state = None

        # 创建图像保存目录
        self.image_save_dir = '/home/zhywwyzh/workspace/VLA_Diff/Openpi/test/infer/trail/saved_images'
        os.makedirs(self.image_save_dir, exist_ok=True)

        # 创建模型输入图像保存目录
        self.model_input_dir = '/home/zhywwyzh/workspace/VLA_Diff/Openpi/test/infer/trail/model_input_images'
        os.makedirs(self.model_input_dir, exist_ok=True)

        # 订阅指令需求
        self.command_type_sub = self.create_subscription(Int32,
                                               "command/type",
                                               self.command_type_callback,
                                               10)
        
        self.command_content_sub = self.create_subscription(String,
                                                "command/content",
                                                self.command_content_callback,
                                                10)

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

    def command_type_callback(self, msg):
        """处理指令需求回调"""
        self.command_type = msg.data
        print(f"当前指令类型: {self.command_type}")
        if self.command_type == COMMAND_TYPE.NEXT:
            self.command_content.pop(0)
            self.vla_state = VLA_STATE.PLAN

        if self.command_type == COMMAND_TYPE.GO_ORIGIN:
            self.vla_state = VLA_STATE.GO_ORIGIN

    def command_content_callback(self, msg):
        """处理指令内容回调"""
        self.command_content.append(json.loads(msg.data))

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
        if len(action) == 3:
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0
        else:
            pose_msg.pose.orientation.x = float(action[3])
            pose_msg.pose.orientation.y = float(action[4])
            pose_msg.pose.orientation.z = float(action[5])
            pose_msg.pose.orientation.w = float(action[6])
        self.action_pub.publish(pose_msg)

    def run_inference(self):
        """执行推理"""
        rate = self.create_rate(20)
        waypoint = None
        self.vla_state = VLA_STATE.INIT
        self.last_plan_time = None

        # 以下2部分后续可用task id直接替代
        def is_waypoint_cmd(cmd) -> bool:
            if isinstance(cmd, (list, tuple, np.ndarray)):
                try:
                    arr = np.array(cmd, dtype=float).reshape(-1)
                    return arr.size >= 3 and np.all(np.isfinite(arr[:3]))
                except Exception:
                    return False
            return False
        
        def is_label_cmd(cmd) -> bool:
            return isinstance(cmd, str) and len(cmd) > 0

        # input("Press Enter to continue...")
        self.get_logger().info("任务开始")

        while rclpy.ok():
            if self.command_type == COMMAND_TYPE.STOP:
                self.vla_state = VLA_STATE.STOP
            # pdb.set_trace()
            match self.vla_state:
                case VLA_STATE.INIT:
                    frame = self.get_frame_snapshot()
                    if self.depth_info and self.get_frame_snapshot() is not None:
                        print("初始化完成")
                        self.vla_state = VLA_STATE.WAIT
                    else:
                        rate.sleep()
                        continue
                case VLA_STATE.WAIT:
                    if not self.command_content:
                        self.vla_state = VLA_STATE.FINISH
                        continue
                    if self.command_type == COMMAND_TYPE.GO:
                        self.vla_state = VLA_STATE.PLAN
                    else:
                        rate.sleep()

                case VLA_STATE.PLAN:
                    try:
                        frame = self.get_frame_snapshot()
                        if frame is None or self.depth_info is None:
                            self.get_logger().warn("传感器未准备好，重新初始化")
                            self.vla_state = VLA_STATE.INIT
                            continue

                        cmd = self.command_content[0]

                        if is_waypoint_cmd(cmd):
                            waypoint = np.array(cmd, dtype=float).reshape(-1)[:3].tolist()
                            self.get_logger().info(f"收到直接导航导航点：{waypoint}")
                            self.vla_state = VLA_STATE.PUBLISH

                        elif is_label_cmd(cmd):
                            result = open_serve(frame.rgb_image, cmd)
                            self.get_logger().info(f"推理结果：{result}")
                            # --- 关键修改：字符串转成 list[int] ---
                            if isinstance(result, str):
                                # 去掉括号和空格，按逗号分割
                                result = result.strip("()\" ")
                                result = [int(x) for x in result.split(",")]
                            waypoint, self.replan = self.pixel_to_world(result, frame)
                            self.get_logger().info(f"收到标签指令：{cmd}，将前往{waypoint}")
                            self.vla_state = VLA_STATE.PUBLISH

                        else:
                            self.get_logger().warn(f"收到未知指令：{cmd}，请检查指令格式")
                            self.command_content.pop(0)
                            self.vla_state = VLA_STATE.WAIT

                        self.last_plan_time = self.get_clock().now()

                    except Exception as e:
                        logging.error(f"规划失败: {e}")
                        self.vla_state = VLA_STATE.ERROR
                
                case VLA_STATE.PUBLISH:
                    try:
                        if waypoint is None:
                            self.get_logger().warn("没有有效的导航点，将重新规划")
                            self.vla_state = VLA_STATE.PLAN
                            continue

                        self.publish_action(waypoint)
                        self.last_state = np.array([waypoint[0], waypoint[1], waypoint[2], 0,0,0], dtype=np.float64)
                        self.first_command = True
                        self.command_type = COMMAND_TYPE.WAIT
                        self.vla_state = VLA_STATE.WAIT_ACTION_FINISH

                        if self.replan:
                            time.sleep(1)  # 等待动作发布完成
                            self.vla_state = VLA_STATE.PLAN

                    except Exception as e:
                        logging.error(f"发布: {e}")
                        self.vla_state = VLA_STATE.ERROR

                case VLA_STATE.WAIT_ACTION_FINISH:
                    try:
                        frame = self.get_frame_snapshot()
                        if waypoint is None:
                            self.vla_state = VLA_STATE.PLAN
                            continue
                        if frame is None:
                            rate.sleep()
                            continue
                        current_state = np.array(frame.current_state[0:3], dtype=np.float64)
                        last_state = np.array(self.last_state[0:3], dtype=np.float64)

                        distance = np.linalg.norm(current_state - last_state) - 0.4
                        time_elapsed = 0.0 if self.last_plan_time is None \
                            else (self.get_clock().now() - self.last_plan_time).nanoseconds / 1e9
                        
                        print(f"距离目的地：{distance}m，运行时长：{time_elapsed:.2f}秒")

                        if distance < self.arrival_distance:
                            self.get_logger().info(f"到达目标点 {waypoint}，距离: {distance:.2f} m")
                            self.first_command = False
                            self.command_content.pop(0)
                            waypoint = None
                            self.vla_state = VLA_STATE.FINISH if not self.command_content else VLA_STATE.WAIT
                            continue

                        # if time_elapsed > self.time_out:
                        #     cmd = self.command_content[0] if self.command_content else None
                        #     if is_label_cmd(cmd) or is_waypoint_cmd(cmd):
                        #         self.get_logger().warn("超时，重新规划")
                        #         self.vla_state = VLA_STATE.PLAN

                        #     else:
                        #         self.get_logger().warn(f"收到未知指令：{cmd}，请检查指令格式")
                        #         self.command_content.pop(0)
                        #         self.vla_state = VLA_STATE.WAIT
                                
                        rate.sleep()

                    except Exception as e:
                        logging.error(f"动作完成失败: {e}")

                case VLA_STATE.FINISH:
                    self.get_logger().info("所有任务完成")
                    rclpy.shutdown()

                case VLA_STATE.STOP:
                    self.get_logger().info("任务收到停止指令")
                    rclpy.shutdown()
                
                case VLA_STATE.ERROR:
                    self.get_logger().error("发生错误，等待正确指令")
                    time.sleep(1)
                    self.vla_state = VLA_STATE.WAIT

                case VLA_STATE.GO_ORIGIN:
                    self.get_logger().info("收到 GO_ORIGIN 指令")
                    waypoint1 = [13.75556939149453, 1.1951389392754197, 1.6000002883663185,
                                 9.283559481815831e-09, -8.782741600475798e-10, 0.40138711153766954, 0.9159085034496877]
                    self.publish_action(waypoint1)
                    time.sleep(3)
                    waypoint2 = [13.136453639674091, 0.5770826187959396, 1.5999999937387603,
                                 6.791822847841003e-06, 2.862005646358964e-06, -0.9238737612250798, 0.3826973651143999]
                    self.publish_action(waypoint2)
                    self.vla_state = VLA_STATE.WAIT

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