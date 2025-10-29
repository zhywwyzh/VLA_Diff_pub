#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import logging
import sys
import os
import re
import pdb
import time
import threading
import math

import imageio.v2 as imageio
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

import rospy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import String, Int32, Empty, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
from quadrotor_msgs.msg import GoalSet

import cv2
from cv_bridge import CvBridge

from utils.vlm.openai_serve_real import open_serve_nav, open_serve_search
from utils.param import COMMAND_TYPE, VLA_STATE

from base_policy_real import BasePolicyNode  # 假定已有 ROS1 版本或与 ROS 无关的 Base
from utils.server.publish_client_ori import MessageClient
from utils.server.receive_client import GeminiMessageClient

class UAVPolicyNode(BasePolicyNode):
    def __init__(self):
        super().__init__()

        # 创建CV桥接器
        self.bridge = CvBridge()

        # 存储最新的状态和图像
        self.first_image = None
        self.last_state = [0, 0, 1, 0, 0, 0]
        self.last_plan_time = None
        self.inference_timeout = 5.0
        self.last_command = None
        self.first_rgb = None
        self.prepare_content = [
            "前往最左侧的白色柱子",
            "前往最近的椅子",
            "请原地左转60度",
            "前往黄色架子第二层",
            "请原地左转90度"
            "前往黄色大柜子",
            "前往白色柱子",
            "请原地右转90度"
            ]
        self.command_content = []
        self.content = self.prepare_content.copy()
        self.pre_prompt = self.prepare_content
        self.pre_prompt.append("完成任务")
        self.prompt_bf = self.pre_prompt.copy()
        self.replan_time = [0, 0, 0, 0, 0, 0, 0]
        self.replan_bf = self.replan_time.copy()
        self.replan_content = None
        self.task_id = [1, 2, 2, 1]
        self.command_type = COMMAND_TYPE.WAIT
        self.task_id_mllm = []
        self.task_id_vlm = []
        self.vla_state = None           # 当前VLA FSM状态
        self.frame = None               # 当前传感器接收数据
        self.ego_state_trigger = False  # ego_planner结束状态触发
        self.mllm_message = None        # 从mllm接收到的最新消息
        self.if_yaw = False             # 是否在任务最后执行转角操作
        self.result = None              # 当前模型返回结果
        self.waypoint = None            # 当前导航目标点
        self.first_plan = False         # 是否为第一次规划
        # TODO 这里的任务是否完成和指令是否完成需要重新考虑设计
        self.finish_mission = False     # 当前任务是否完成
        self.finish_command = False     # 当前指令是否完成
        self.go_origin = False          # 是否执行返回起点的任务
        self.is_label = False           # 当前指令是否为目标搜索指令
        self.bridge = CvBridge()        # CV桥接器
        self.bbox = [0, 0, 0, 0]        # 当前观察结果的bbox
        self.first_bbox = []            # 给定起始bbox
        self.if_plan = False            # 是否进入规划状态
        self.see_none = 0               # 连续未见目标计数
        self.vis_first = False          # 起始状态是否看到目标
        self.vis_cur = False            # 当前是否看到目标
        self.over_edge = False          # 检测到的物体是否在深度之外
        self.if_safe_dis = True         # 是否保持安全距离
        self.safe_dis = 0.7             # 安全距离阈值

        # 搜索相关参数
        self.replan_count = 0           # 重规划计数
        self.max_yaw_search = 6         # 最大搜索次数
        self.cur_yaw_search = 0         # 当前搜索次数
        self.search_rot_yaw = 60        # 搜索时候每次旋转的角度
        self.max_z_search = 2           # 最大向上搜索次数
        self.cur_z_search = 0           # 当前向上搜索次数
        self.search_rot_z = 0.5         # 搜索时候每次向上移动的距离
        self.back_percent = 0.5         # 向后搜索相关参数
        self.allow_min_depth = 0.5      # 允许的最小距离障碍物深度
        self.back_dis = 0.5             # 障碍物过多搜索时候后退的距离

        current_dir = os.path.dirname(os.path.abspath(__file__))
        parent_dir = os.path.dirname(current_dir)

        # 创建图像保存目录
        self.image_save_dir = os.path.join(parent_dir, 'Openpi/test/infer/trail/saved_images')
        os.makedirs(self.image_save_dir, exist_ok=True)

        # 创建模型输入图像保存目录
        self.model_input_dir = os.path.join(parent_dir, 'Openpi/test/infer/trail/model_input_images')
        os.makedirs(self.model_input_dir, exist_ok=True)

        # 订阅指令需求
        self.command_type_sub = rospy.Subscriber(
            "command/type", Int32, self.command_type_callback, queue_size=10
        )
        self.command_content_sub = rospy.Subscriber(
            "command/content", String, self.command_content_callback, queue_size=10
        )

        self.type_pub = rospy.Publisher("command/type", Int32, queue_size=10)

        # 订阅ego_planner的状态指令
        self.ego_state_trigger_sub = rospy.Subscriber(
            "/planning/ego_state_trigger", Bool, self.ego_state_trigger_callback, queue_size=10
        )

        # 发布无人机动作
        self.action_pub = rospy.Publisher(
            '/goal_with_id_from_station', GoalSet, queue_size=10
        )

        # 发布急停指令
        self.emergency_stop_pub = rospy.Publisher(
            'command/emergency_stop', Empty, queue_size=10
        )

        # 发布任务图片
        self.first_image_pub = rospy.Publisher(
            'first_mission_image', Image, queue_size=10
        )

        self.current_image_pub = rospy.Publisher(
            'current_mission_image', Image, queue_size=10
        )

        # 发布bbox图像
        self.bbox_image_pub = rospy.Publisher(
            'bbox_mission_image', Image, queue_size=10
        )

        # 发布监控
        self.monitor_command_type_pub = rospy.Publisher(
            'monitor/command_type', Int32, queue_size=10
        )
        self.monitor_vla_state_pub = rospy.Publisher(
            'monitor/vla_state', Int32, queue_size=10
        )
        self.monitor_timer = rospy.Timer(rospy.Duration(0.1), self.publish_monitor_status)

        self.command_content_pub = rospy.Publisher(
            "monitor/command_content", String, queue_size=10
        )

        self.finish_mission_pub = rospy.Publisher(
            "monitor/finish_mission", Bool, queue_size=10
        )

        self.finish_command_pub = rospy.Publisher(
            "monitor/finish_command", Bool, queue_size=10
        )

        # 初始化 WebSocket / 服务端客户端
        self.client = None
        self.prompt = None

        # 初始化 mllm 客户端
        self.receive_client = GeminiMessageClient()
        self.publish_client = MessageClient()

        # 轨迹存储
        self.original_trajectory = []
        self.inferred_trajectory = []
        self.tasks_jsonl_path = os.path.join(parent_dir, 'Openpi/test/infer/test_data/meta/tasks.jsonl')
        self.task_index = 2

        # 从参数服务器获取配置（ROS1）
        self.host = rospy.get_param('~host', '127.0.0.1')
        self.port = int(rospy.get_param('~port', 8000))
        self.replan_steps = int(rospy.get_param('~replan_steps', 10))

        # 启动消息监听线程
        self.listener_thread = threading.Thread(target=self.listen_messages, daemon=True)
        self.listener_thread.start()

    def publish_monitor_status(self, event):
        """定期发布监控状态"""
        type_msg = Int32()
        type_msg.data = self.command_type
        self.monitor_command_type_pub.publish(type_msg)

        state_msg = Int32()
        state_msg.data = self.vla_state if self.vla_state is not None else -1
        self.monitor_vla_state_pub.publish(state_msg)

        finish_mission_msg = Bool()
        finish_mission_msg.data = self.finish_mission
        self.finish_mission_pub.publish(finish_mission_msg)

        finish_command_msg = Bool()
        finish_command_msg.data = self.finish_command
        self.finish_command_pub.publish(finish_command_msg)

    def publish_command_content(self, command_content):
        """发布当前指令内容"""
        content_msg = String()
        content_msg.data = json.dumps(command_content, ensure_ascii=False)
        self.command_content_pub.publish(content_msg)

    def command_type_callback(self, msg: Int32):
        """处理指令需求回调"""
        self.command_type = msg.data
        rospy.loginfo(f"当前指令类型: {self.command_type}")

        match self.command_type:
            case COMMAND_TYPE.NEXT:
                self.publish_command_content(self.command_content)
                self.vla_state = VLA_STATE.PLAN

            case COMMAND_TYPE.AGAIN:
                self.command_content.insert(0, self.last_command)
                self.publish_command_content(self.command_content)
                self.vla_state = VLA_STATE.PLAN

            case COMMAND_TYPE.GO_ORIGIN:
                self.vla_state = VLA_STATE.GO_ORIGIN

            case COMMAND_TYPE.EMERGENCY_STOP:
                print("🛑 收到紧急停止指令")
                self.emergency_stop_pub.publish(Empty())

            case COMMAND_TYPE.RESTART:
                print("🔄 收到重启指令，重新初始化")
                self.command_content = []
                self.first_rgb = self.frame.rgb_image.copy()
                self.vla_state = VLA_STATE.WAIT_FOR_MISSION

            case COMMAND_TYPE.GET_PRE:
                rospy.loginfo("收到 GET_PRE 指令，准备执行任务")
                self.command_content = []
                # replan_times = self.replan_time.pop(0)
                if self.prepare_content:
                    # print(f"规划任务为: {self.prepare_content}")
                    prompt = self.pre_prompt.pop(0)
                    rospy.loginfo(f"\033[93m当前任务为: {prompt}\033[0m")
                    self.replan_content = self.prepare_content.pop(0)
                    # rospy.loginfo(f"接受指令：{self.replan_content}")
                    self.command_content.append(self.replan_content)
                    self.publish_command_content(self.command_content)
                    # self.vla_state = VLA_STATE.PLAN
                    self.if_plan = True
                    time.sleep(0.5)
                    # while replan_times > 0:
                    # TODO 整合下finish_mission和finish_command，这两个参数现在应该是没有差异的
                    while not self.finish_mission:
                        if self.finish_command:
                            # self.finish_command = False
                            break
                        if self.is_label:
                            self.command_content.append(self.replan_content)
                            self.publish_command_content(self.command_content)
                            # self.vla_state = VLA_STATE.PLAN
                            self.if_plan = True
                            time.sleep(0.5)
                        else:
                            time.sleep(1)
                            # replan_times -= 1
                    # self.finish_command = False
                    self.finish_mission = False
                else:
                    # if self.finish_mission:
                    rospy.loginfo("完成所有任务")

            case COMMAND_TYPE.REPLAN:
                self.prepare_content = self.content.copy()
                self.pre_prompt = self.prompt_bf.copy()
                self.replan_time = self.replan_bf.copy()

            case _:  # 默认情况
                rospy.logwarn(f"收到未知指令: {self.command_type}")


    def command_content_callback(self, msg: String):
        """处理指令内容回调"""
        # 原封不动保存整段文本
        self.command_content.append(msg.data)
        self.publish_command_content(self.command_content)
        # self.vla_state = VLA_STATE.PLAN
        self.if_plan = True
        # rospy.loginfo(f"✅ 收到完整 content 消息: {msg.data}")

    def calculate_plan_yaw(self, first_waypoint, first_frame, current_frame, over_edge):
        """计算规划的航向角"""
        if first_waypoint is None or first_frame is None or current_frame is None:
            rospy.logwarn("计算航向角时缺少必要参数")
            return
        
        # 提取当前位置和第一次判断时的位置
        current_pos = current_frame.current_state[:3]  # [x, y, z]
        first_pos = first_frame.current_state[:3]      # [x, y, z]
        target_pos = np.array(first_waypoint[:3])     # [x, y, z]
        
        # if not over_edge:
        #     # 情况1：物体未超出极限范围，直接朝向目标点
        #     direction_vector = target_pos - current_pos
        #     target_yaw = math.atan2(direction_vector[1], direction_vector[0])
        #     rospy.loginfo("物体未超出范围，直接朝向目标点")
        if not over_edge:
            # TODO 现在这个逻辑是根据仅进行一次旋转计算的
            # 情况1：物体未超出极限范围，yaw朝向改为“起点->目标”与“起始朝向”的角平分线
            first_yaw = float(first_frame.current_state[5])
            # 射线1：起始位置 -> 目标位置
            v1 = (target_pos[:2] - first_pos[:2]).astype(np.float64)
            # 射线2：起始朝向对应的单位方向向量
            v2 = np.array([math.cos(first_yaw), math.sin(first_yaw)], dtype=np.float64)
            eps = 1e-6
            if np.linalg.norm(v1) < eps:
                # 起点与目标几乎重合：无法定义“起点->目标”方向，直接保持起始朝向
                target_yaw = first_yaw
            else:
                u1 = v1 / (np.linalg.norm(v1) + eps)   # 单位化 “起点->目标”
                b  = u1 + v2
                if np.linalg.norm(b) < 1e-6:
                    # 退化：两射线近乎反向（内角≈π），向量和接近0，选择朝向目标方向以避免不确定性
                    target_yaw = math.atan2(u1[1], u1[0])
                else:
                    target_yaw = math.atan2(b[1], b[0])
            # 将 yaw 归一化到 [-pi, pi]
            rospy.loginfo("物体未超出范围，使用角平分线朝向（起始->目标 与 起始朝向）")

        else:
            # 情况2：物体超出极限范围，朝向两条射线的角平分线
            ray1 = target_pos - first_pos
            ray1_yaw = math.atan2(ray1[1], ray1[0])
            
            ray2 = target_pos - current_pos
            ray2_yaw = math.atan2(ray2[1], ray2[0])
            
            angle_diff = ray2_yaw - ray1_yaw
            
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            target_yaw = ray1_yaw + angle_diff / 2
            
            rospy.loginfo(f"物体超出范围，朝向角平分线方向")
        
        target_yaw = (target_yaw + math.pi) % (2 * math.pi) - math.pi
        
        new_waypoint = np.array([
            current_pos[0],
            current_pos[1], 
            current_pos[2],
            0.0,
            0.0,
            target_yaw
        ], dtype=np.float64)
        
        rospy.loginfo(f"计算得到目标航向角: {math.degrees(target_yaw):.1f}°")
        
        return new_waypoint

    # TODO 调整ego回调逻辑
    def ego_state_trigger_callback(self, msg: Bool):
        """处理ego_state_trigger回调"""
        print("收到 ego_state_trigger 信号")
        self.ego_state_trigger = msg.data
        # self.vla_state = VLA_STATE.EGO_FINISH

    # TODO 修改提取方案
    def get_command_content(self):
        """从mllm消息中提取指令内容"""
        if self.mllm_message is not None:
            return self.mllm_message
        return None

    def backoff_waypoint(self):
        depth = self.frame.depth_image
        valid_depth = np.isfinite(depth) & (depth > 0.1 + 1e-6)
        valid_count = int(valid_depth.sum())
        if valid_count == 0:
            rospy.logwarn("深度图无有效数据，无法判断是否需要后退")
            return None
        close_ratio = float ((depth[valid_depth] < self.allow_min_depth).mean())

        if close_ratio > self.back_percent:
            xyz = self.frame.current_state[:3]
            yaw = self.frame.current_state[5]

            dx = -self.back_dis * math.cos(yaw)
            dy = -self.back_dis * math.sin(yaw)

            new_x = xyz[0] + dx
            new_y = xyz[1] + dy
            new_z = xyz[2]

            waypoint = np.array([new_x, new_y, new_z, 0.0, 0.0, yaw], dtype=np.float64)
            return waypoint
        return None

    # TODO 修改监听端口
    # def listen_messages(self):
    #     """循环监听mllm新消息"""
    #     while not rospy.is_shutdown():
    #         try:
    #             message = self.receive_client.get_new_messages()
    #             if message:
    #                 self.mllm_message = message[0]["text"]
    #                 self.command_content.append(self.mllm_message)
    #                 rospy.loginfo(f"Received message: {self.mllm_message}")
    #         except Exception as e:
    #             rospy.logerr(f"消息监听出错: {e}")
    #         time.sleep(0.5)

    def listen_messages(self):
        """循环监听mllm新消息"""
        while not rospy.is_shutdown():
            message = self.receive_client.get_new_messages()
            
            if message:
                # 检测到"我将执行以下操作"时，提取内容并更新 prepare_content 和 pre_prompt
                if re.search("我将执行以下操作", message[0]["text"]):
                    numbered_actions = re.findall(r'^\s*\d+\s*[\.、\)]\s*(.+?)\s*$', message[0]["text"], flags=re.M)
                    # 清理末尾句号等冗余标点
                    actions = [re.sub(r'[。．.]$', '', a).strip() for a in numbered_actions if a.strip()]
                    if actions:
                        self.prepare_content = actions
                        self.pre_prompt = actions + ["完成任务"]
                        type_msg = Int32()
                        type_msg.data = COMMAND_TYPE.GET_PRE
                        self.type_pub.publish(type_msg)
                        self.finish_command = False
                        rospy.loginfo(f"更新任务列表: {self.prepare_content}")
                
                # 检测到"原地左转|右转n度"时，提取内容并更新 prepare_content 和 pre_prompt
                elif re.search(r"好的，我将原地(左|右)转(\d+)度", message[0]["text"]):
                    action = re.findall(r"原地(左|右)转(\d+)度", message[0]["text"])
                    if action:
                        direction, degree = action[0]
                        self.prepare_content = [f"原地{direction}转{degree}度"]
                        self.pre_prompt = [f"原地{direction}转{degree}度", "完成任务"]
                        type_msg = Int32()
                        type_msg.data = COMMAND_TYPE.GET_PRE
                        self.type_pub.publish(type_msg)
                        self.finish_command = False
                        rospy.loginfo(f"更新任务列表: {self.prepare_content}")

                # 检测到"前往**"时，提取内容并更新 prepare_content 和 pre_prompt
                elif re.search(r"好的，我将前往([\u4e00-\u9fa5]+)", message[0]["text"]):
                    destination = re.findall(r"前往([\u4e00-\u9fa5]+)", message[0]["text"])
                    if destination:
                        self.prepare_content = [f"前往{destination[0]}"]
                        self.pre_prompt = [f"前往{destination[0]}", "完成任务"]
                        type_msg = Int32()
                        type_msg.data = COMMAND_TYPE.GET_PRE
                        self.type_pub.publish(type_msg)
                        self.finish_command = False
                        rospy.loginfo(f"更新任务列表: {self.prepare_content}")

                # 检测到"需要进行什么样的分析"时，发送图像并记录状态
                elif re.search(r"需要进行什么样的分析", message[0]["text"]) or re.search(r"需求进行", message[0]["text"]):
                    if self.frame is not None and hasattr(self.frame, 'rgb_image') and self.frame.rgb_image is not None:
                        rospy.loginfo("收到图像分析请求，发送当前帧图像")
                        response = self.publish_client.send_image(self.frame.rgb_image, "分析当前图像")
                        rospy.loginfo(f"当前帧接收状态: {response['message']}")
            time.sleep(0.5)

    def publish_action(self, action, look_forward=True, goal_to_follower=False):
        """发布动作到ROS话题"""
        # if len(action) < 3:
        #     logging.error("动作数据不足3个元素")
        #     return

        pose_msg = GoalSet()
        pose_msg.to_drone_ids = [0]

        # 设置位置
        pt = Point()
        pt.x = float(action[0])
        pt.y = float(action[1])
        pt.z = float(action[2])
        pose_msg.goal = [pt]

        # 姿态（若只给 xyz，则用单位四元数）
        if len(action) == 3:
            roll, pitch, yaw = self.quaternion_to_euler(0.0, 0.0, 0.0, 1.0)
        else:
            roll, pitch, yaw = float(action[3]), float(action[4]), float(action[5])
        pose_msg.yaw = [yaw]

        # 设置如何飞行
        pose_msg.look_forward = look_forward
        pose_msg.goal_to_follower = goal_to_follower

        self.action_pub.publish(pose_msg)

    def get_judgement(self, message):
        """判断是否包含 True 或 False"""
        text = message[0]['text']
        has_true = "True" in text
        has_false = "False" in text
        if has_true and has_false:
            print("无法判断结果，请重试")
            return False
        elif has_true:
            return True
        elif has_false:
            return False
        else:
            print("未收到相关结果，请重试")
            return False

    def publish_image(self, image, pub):
        """发布图像到ROS话题"""
        # 确定编码方式
        if image is None:
            return
        # 判断编码
        if image.dtype == np.uint8:
            if len(image.shape) == 3 and image.shape[2] == 3:
                encoding = "bgr8"
            elif len(image.shape) == 2:
                encoding = "mono8"
            else:
                rospy.logerr(f"不支持的图像 shape: {image.shape}")
                return
        elif image.dtype == np.uint16:
            encoding = "16UC1"   # 深度图常用
        elif image.dtype == np.float32:
            encoding = "32FC1"   # 浮点深度图常用
        else:
            rospy.logerr(f"不支持的图像 dtype: {image.dtype}")
            return

        # 转换并发布
        ros_img = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
        pub.publish(ros_img)
        # rospy.loginfo(f"图像已发布，编码: {encoding}, shape: {image.shape}, dtype: {image.dtype}")


    # TODO 推理逻辑初始化，初始与mllm的通信
    def run_inference(self):
        """执行推理"""
        rate = rospy.Rate(20)
        waypoint = None
        self.vla_state = VLA_STATE.INIT
        self.last_plan_time = None
        rospy.loginfo("等待传感器准备...")

        while self.get_frame_snapshot() is None:
            time.sleep(1)
        
        self.first_image = self.frame.rgb_image

        def is_waypoint_cmd(c) -> bool:
            if isinstance(c, (list, tuple, np.ndarray)):
                arr = np.array(c, dtype=float).reshape(-1)
                return arr.size >= 3 and np.all(np.isfinite(arr[:3]))
            return False

        def is_label_cmd(c) -> bool:
            return isinstance(c, str) and len(c) > 0
        
        def is_rot_cmd(c) -> bool:
            return bool(re.search(r"原地(左|右)转(\d+(?:\.\d+)?)度", c))

        rospy.loginfo("任务开始")

        while not rospy.is_shutdown():
            if self.command_type == COMMAND_TYPE.STOP:
                self.vla_state = VLA_STATE.STOP

            self.frame = self.get_frame_snapshot()
            
            # if self.ego_state_trigger:
            #     self.vla_state = VLA_STATE.EGO_FINISH
            #     self.ego_state_trigger = False
                # continue

            match self.vla_state:
                case VLA_STATE.INIT:
                    if getattr(self, 'depth_info', None) and self.get_frame_snapshot() is not None:
                        print("初始化完成")
                        response = self.publish_client.send_image(self.frame.rgb_image, "分析当前图像")
                        rospy.loginfo(f"当前帧接收状态: {response['message']}")
                        self.vla_state = VLA_STATE.WAIT_FOR_MISSION
                    else:
                        rate.sleep()
                        continue

                case VLA_STATE.WAIT_FOR_MISSION:
                    if not self.command_content:
                        # self.vla_state = VLA_STATE.FINISH
                        # rospy.loginfo("当前任务为空，请传入下一步任务")
                        if self.if_plan:
                            self.vla_state = VLA_STATE.PLAN
                            self.if_plan = False
                        time.sleep(0.1)
                        continue

                case VLA_STATE.REPLY_MLLM:
                    # rospy.loginfo("到达目的地，基于MLLM回复")
                    self.vla_state = VLA_STATE.WAIT_FOR_MISSION
                    response = self.publish_client.send_image(self.frame.rgb_image)
                    rospy.loginfo(f"当前帧接收状态: {response['message']}")
                    continue

                # TODO 调整plan与replan
                case VLA_STATE.PLAN:
                    try:
                        # if self.frame is None or getattr(self, 'depth_info', None) is None:
                        if self.frame is None:
                            rospy.logwarn("传感器未准备好，重新初始化")
                            self.vla_state = VLA_STATE.INIT
                            continue
                        if self.finish_mission:
                            print("上一任务完成，执行下一任务")
                            self.finish_mission = False
                            self.first_rgb = self.frame.rgb_image.copy()
                            self.first_bbox = []
                            self.first_frame = None

                        cmd = self.command_content[0]

                        if is_waypoint_cmd(cmd):
                            self.is_label = False
                            waypoint = np.array(cmd, dtype=float).reshape(-1)[:3].tolist()
                            rospy.loginfo(f"收到直接导航导航点：{waypoint}")
                            self.vla_state = VLA_STATE.PUBLISH

                        elif is_rot_cmd(cmd):
                            self.is_label = False
                            match = re.search(r"原地(左|右)转(\d+(?:\.\d+)?)度", cmd)
                            direction = match.group(1)
                            angle = float(match.group(2))
                            yaw_angle = angle if direction == "左" else -angle
                            print(f"执行旋转: {yaw_angle} 度")
                            xyz = self.frame.current_state[:3]
                            yaw = self.frame.current_state[5] + yaw_angle / 180 * math.pi
                            waypoint = np.array([
                                xyz[0],
                                xyz[1],
                                xyz[2],
                                0.0,
                                0.0,
                                yaw
                            ], dtype=np.float64)
                            self.waypoint = waypoint
                            self.vla_state = VLA_STATE.WAIT_ACTION_FINISH
                            self.command_content.pop(0)
                            self.finish_mission = True
                            self.finish_command = True
                            self.publish_action(waypoint, look_forward=False)
                            time.sleep(2)
                            # self.vla_state = VLA_STATE.WAIT_ACTION_FINISH
                            # type_msg = Int32()
                            # type_msg.data = COMMAND_TYPE.GET_PRE
                            # self.type_pub.publish(type_msg)
                            # print("准备下一项目")

                        elif is_label_cmd(cmd):
                            # 执行后退观察动作
                            back_waypoint = self.backoff_waypoint()
                            if back_waypoint is not None:
                                rospy.loginfo("检测到距离障碍物过近，执行后退观察动作")
                                self.waypoint = back_waypoint
                                self.vla_state = VLA_STATE.WAIT_ACTION_FINISH
                                self.publish_action(back_waypoint, look_forward=False)
                                continue
                            self.is_label = True
                            if not self.first_plan:
                                self.first_plan = True
                                self.first_rgb = self.frame.rgb_image.copy()
                                self.first_bbox = []
                                self.first_frame = None
                            self.publish_image(self.first_rgb, self.first_image_pub)
                            self.publish_image(self.frame.rgb_image, self.current_image_pub)
                            origin_time = time.time()
                            rgb_image = self.frame.rgb_image
                            current_frame = self.frame
                            self.vis_first = open_serve_search(self.first_rgb, cmd)

                            if not self.vis_first:
                                if self.cur_yaw_search < self.max_yaw_search:
                                    if self.cur_yaw_search < self.max_yaw_search // 2:
                                        rospy.loginfo(f"目标未见，原地左转{self.search_rot_yaw}度继续搜索")
                                        xyz = self.frame.current_state[:3]
                                        yaw = self.frame.current_state[5] + self.search_rot_yaw / 180 * math.pi
                                        waypoint = np.array([
                                            xyz[0],
                                            xyz[1],
                                            xyz[2],
                                            0.0,
                                            0.0,
                                            yaw
                                        ], dtype=np.float64)
                                        self.waypoint = waypoint
                                        self.vla_state = VLA_STATE.WAIT_ACTION_FINISH
                                        self.command_content.pop(0)
                                        self.publish_action(waypoint, look_forward=False)

                                        # TODO 这里是没有replan的逻辑，后续加入replan后删除
                                        self.command_content.append(self.replan_content)
                                        self.publish_command_content(self.command_content)
                                        self.if_plan = True

                                        time.sleep(1)
                                    else:
                                        rospy.loginfo(f"目标未见，原地右转{self.search_rot_yaw}度继续搜索")
                                        if self.cur_yaw_search == self.max_yaw_search // 2 :
                                            k = self.cur_yaw_search + 1
                                        else:
                                            k = 1
                                        xyz = self.frame.current_state[:3]
                                        yaw = self.frame.current_state[5] - k * self.search_rot_yaw / 180 * math.pi
                                        waypoint = np.array([
                                            xyz[0],
                                            xyz[1],
                                            xyz[2],
                                            0.0,
                                            0.0,
                                            yaw
                                        ], dtype=np.float64)
                                        self.waypoint = waypoint
                                        self.vla_state = VLA_STATE.WAIT_ACTION_FINISH
                                        self.command_content.pop(0)
                                        self.publish_action(waypoint, look_forward=False)

                                        # TODO 这里是没有replan的逻辑，后续加入replan后删除
                                        self.command_content.append(self.replan_content)
                                        self.publish_command_content(self.command_content)
                                        self.if_plan = True

                                        time.sleep(1)
                                    self.first_plan = False
                                    self.cur_yaw_search += 1
                                    continue
                                elif self.cur_z_search < self.max_z_search:
                                    rospy.loginfo(f"目标未见，向上移动{self.search_rot_z}米继续搜索")
                                    xyz = self.frame.current_state[:3]
                                    z = xyz[2] + self.search_rot_z
                                    yaw = self.frame.current_state[5] + self.max_yaw_search // 2 * self.search_rot_yaw / 180 * math.pi
                                    waypoint = np.array([
                                        xyz[0],
                                        xyz[1],
                                        z,
                                        0.0,
                                        0.0,
                                        yaw
                                    ], dtype=np.float64)
                                    self.waypoint = waypoint
                                    self.vla_state = VLA_STATE.WAIT_ACTION_FINISH
                                    self.command_content.pop(0)
                                    self.publish_action(waypoint, look_forward=False)
                                    
                                    # TODO 这里是没有replan的逻辑，后续加入replan后删除
                                    self.command_content.append(self.replan_content)
                                    self.publish_command_content(self.command_content)
                                    self.if_plan = True
                                        
                                    time.sleep(1)
                                    self.first_plan = False
                                    self.cur_z_search += 1
                                    self.cur_yaw_search = 0
                                    continue
                                else:
                                    self.cur_yaw_search = 0
                                    self.cur_z_search = 0
                                    rospy.logwarn("连续多次未能识别到目标物体，当前任务结束")
                                    self.finish_mission = True
                                    self.ego_state_trigger = True
                                    self.vla_state = VLA_STATE.WAIT_ACTION_FINISH
                                    continue

                            self.bbox, self.result, self.finish_mission = open_serve_nav(self.first_rgb, current_frame.rgb_image, cmd)
                            if self.bbox is not None:
                                # 发布bbox图像
                                pt1 = (int(self.bbox[0]), int(self.bbox[1]))
                                pt2 = (int(self.bbox[2]), int(self.bbox[3]))
                                bbox_image = rgb_image.copy()
                                color_red_rgb = (0, 0, 255)
                                thickness = 4
                                cv2.rectangle(bbox_image, pt1, pt2, color_red_rgb, thickness)
                                self.publish_image(bbox_image, self.bbox_image_pub)
                                rospy.loginfo(f"推理耗时: {time.time() - origin_time:.2f} 秒")
                                # print(f"像素中心位于：{self.result}")
                                # pdb.set_trace()
                                pattern = re.compile(r'的(左侧|右侧|上方|下方)\s*(?:。|\.)')
                                match = re.search(pattern, cmd)
                                if match:
                                    direction = match.group(1)
                                    rospy.loginfo(f"检测到方位关键词：{direction}，将 if_safe_dis 置为 False")
                                    self.if_safe_dis = False
                                waypoint = self.pixel_to_world(self.result, current_frame, percent_point=0.3, direction=direction)

                                distance = np.linalg.norm(np.array(waypoint[:3]) - np.array(self.frame.current_state[:3]))
                                if distance < 0.3:
                                    rospy.loginfo("距离过近，暂不发布动作")
                                    self.command_content.pop(0)
                                    self.vla_state = VLA_STATE.WAIT_ACTION_FINISH
                                    continue
                            elif self.see_none < 5:
                                rospy.logwarn("未能识别到目标物体，重新规划")
                                self.command_content.pop(0)
                                self.vla_state = VLA_STATE.WAIT_ACTION_FINISH
                                self.see_none += 1
                                time.sleep(0.5)
                                continue
                            else:
                                self.see_none = 0
                                rospy.logwarn("连续多次未能识别到目标物体，当前任务结束")
                                self.finish_mission = True
                                self.ego_state_trigger = True
                            if self.first_frame is None:
                                self.first_frame = current_frame
                                self.first_bbox = self.bbox

                            rospy.loginfo(f"任务完成状态: {self.finish_mission}")
                            if self.finish_mission:
                                self.vla_state = VLA_STATE.WAIT_ACTION_FINISH
                                self.finish_command = True    
                                time.sleep(0.1)                            
                                # self.command_content.pop(0)
                                # self.waypoint = self.calculate_plan_yaw(self.first_waypoint, self.first_frame, current_frame, self.over_edge)
                                self.waypoint = self.calculate_plan_yaw(self.waypoint, self.first_frame, self.frame, False)
                                self.if_yaw = True
                                continue

                            self.waypoint = waypoint
                            self.vla_state = VLA_STATE.PUBLISH

                        else:
                            rospy.logwarn(f"收到未知指令：{cmd}，请检查指令格式")
                            self.vla_state = VLA_STATE.WAIT_FOR_MISSION

                        self.last_plan_time = rospy.Time.now()

                    except Exception as e:
                        logging.error(f"规划失败: {e}")
                        self.vla_state = VLA_STATE.ERROR

                case VLA_STATE.PUBLISH:
                    if waypoint is None:
                        rospy.logwarn("没有有效的导航点，将重新规划")
                        self.publish_command_content(self.command_content)
                        self.vla_state = VLA_STATE.PLAN
                        time.sleep(0.1)
                        continue
                    # self.vla_state = VLA_STATE.WAIT_ACTION_FINISH
                    # if self.finish_mission:
                    #     self.finish_command = True
                    #     continue

                    self.publish_action(waypoint)
                    self.command_content.pop(0)
                    self.vla_state = VLA_STATE.WAIT_ACTION_FINISH
                    print(f"当前位于: {[round(x, 2) for x in self.frame.current_state]}")
                    print(f"发布导航点: {[round(x, 2) for x in waypoint]}\n")
                    self.last_state = np.array([waypoint[0], waypoint[1], waypoint[2], 0, 0, 0], dtype=np.float64)
                    # time.sleep(0.5)  # 等待动作发布完成

                case VLA_STATE.WAIT_ACTION_FINISH:
                    # self.frame = self.get_frame_snapshot()                    
                    if self.ego_state_trigger:
                        # print(f"当前ego_state_trigger状态: {self.ego_state_trigger}")
                        self.vla_state = VLA_STATE.EGO_FINISH
                        self.ego_state_trigger = False
                        continue
                    if self.if_plan:
                        self.vla_state = VLA_STATE.PLAN
                        self.if_plan = False
                    time.sleep(0.1)
                    continue

                case VLA_STATE.FINISH:
                    rospy.loginfo("所有任务完成")
                    rospy.signal_shutdown("All tasks completed")
                    break

                case VLA_STATE.STOP:
                    rospy.loginfo("任务收到停止指令")
                    rospy.signal_shutdown("Stopped by command")
                    break

                case VLA_STATE.ERROR:
                    rospy.logerr("发生错误，等待正确指令")
                    time.sleep(1)
                    self.vla_state = VLA_STATE.WAIT_FOR_MISSION

                case VLA_STATE.GO_ORIGIN:
                    rospy.loginfo("收到 GO_ORIGIN 指令")
                    self.go_origin = True
                    roll, pitch, yaw = self.quaternion_to_euler(0, 0, 0.998, 0.062)
                    yaw = 0
                    self.waypoint = [22.425, -1.338, 1,
                                 0, 0, math.pi]
                    self.publish_action(self.waypoint, look_forward=True)
                    self.if_yaw = True
                    self.vla_state = VLA_STATE.WAIT_ACTION_FINISH

                case VLA_STATE.EGO_FINISH:
                    rospy.loginfo("收到 ego_planner 信号")
                    if self.go_origin:
                        rospy.loginfo("返回原点")
                        # self.if_yaw = False
                        # rospy.loginfo(f"当前ego_state_trigger状态: {self.ego_state_trigger}")
                        if self.if_yaw:
                            rospy.loginfo("执行旋转")
                            self.publish_action(self.waypoint, look_forward=False)
                        self.vla_state = VLA_STATE.WAIT_ACTION_FINISH
                        self.if_yaw = False
                        self.finish_command = False
                        self.finish_mission = False
                        self.ego_state_trigger = False
                        self.go_origin = False
                    if self.finish_command:
                        print("当前任务完成，执行尾部动作")
                        if self.if_yaw:
                            rospy.loginfo("执行旋转")
                            self.publish_action(self.waypoint, look_forward=False)
                            self.vla_state = VLA_STATE.WAIT_ACTION_FINISH
                            self.if_yaw = False
                            continue
                        # else:
                        # time.sleep(1)
                        self.vla_state = VLA_STATE.REPLY_MLLM
                        type_msg = Int32()
                        type_msg.data = COMMAND_TYPE.GET_PRE
                        self.type_pub.publish(type_msg)
                        self.finish_command = False
                        print("准备下一项目")
                    else:
                        rospy.loginfo("等待任务完成信号")
                        self.vla_state = VLA_STATE.WAIT_ACTION_FINISH

            rate.sleep()


def main():
    logging.basicConfig(level=logging.INFO)
    rospy.init_node('uav_policy_node', anonymous=True)

    print("Program starting...")
    node = UAVPolicyNode()
    # 在单独的线程中运行推理循环
    inference_thread = threading.Thread(target=node.run_inference)
    inference_thread.start()

    # 主线程运行 ROS1 spin
    rospy.spin()

    # 等待推理线程结束
    inference_thread.join()


if __name__ == "__main__":
    main()
