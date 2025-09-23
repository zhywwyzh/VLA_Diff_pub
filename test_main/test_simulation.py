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
import ast
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
from cv_bridge import CvBridge

import cv2
from cv_bridge import CvBridge

from utils.vlm.openai_serve import open_serve
from utils.param import COMMAND_TYPE, VLA_STATE

from base_policy import BasePolicyNode  # 假定已有 ROS1 版本或与 ROS 无关的 Base
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
        self.first_mission_frame = None
        self.prepare_content = [
            "<image>\n<image>\n<image>\n你将得到首次观察rgb图像、当前观察的rgb图像、当前观察的depth图像。请进入首次观察图像中左侧通道，并给出在世界空间中任务目标点相对相机中心的变换三维坐标点与yaw角旋转角。给出的三维坐标可以不与深度对应，但不得超出对应深度范围。请按照json格式{\"pos\": (x, y, z), \"yaw\": delta_yaw, \"mission_finish\": *}输出结果。已知深度相机的深度范围为0.2m-5.0m,参数为fx = fy = 442.025，cx = 320, cy = 240，不存在畸变，图像尺寸为height: 480, width: 640",
            "<image>\n<image>\n<image>\n你将得到首次观察rgb图像、当前观察的rgb图像、当前观察的depth图像。请原地左转45度，并给出在世界空间中任务目标点相对相机中心的yaw角旋转角。请按照json格式{\"pos\": (0.0, 0.0, 0.0), \"yaw\": delta_yaw}输出结果。已知深度相机的深度范围为0.2m-5.0m,参数为fx = fy = 442.025，cx = 320, cy = 240，不存在畸变，图像尺寸为height: 480, width: 640",
            "<image>\n<image>\n<image>\n你将得到首次观察rgb图像、当前观察的rgb图像、当前观察的depth图像。请向前进入首次观察图像中右侧门内，并给出在世界空间中任务目标点相对相机中心的变换三维坐标点与yaw角旋转角。给出的三维坐标可以不与深度对应，但不得超出对应深度范围。请按照json格式{\"pos\": (x, y, z), \"yaw\": delta_yaw, \"mission_finish\": *}输出结果。已知深度相机的深度范围为0.2m-5.0m,参数为fx = fy = 442.025，cx = 320, cy = 240，不存在畸变，图像尺寸为height: 480, width: 640",
            "<image>\n<image>\n<image>\n你将得到首次观察rgb图像、当前观察的rgb图像、当前观察的depth图像。请原地右转45度，并给出在世界空间中任务目标点相对相机中心的yaw角旋转角。请按照json格式{\"pos\": (0.0, 0.0, 0.0), \"yaw\": delta_yaw}输出结果。已知深度相机的深度范围为0.2m-5.0m,参数为fx = fy = 442.025，cx = 320, cy = 240，不存在畸变，图像尺寸为height: 480, width: 640",
            "<image>\n<image>\n<image>\n你将得到首次观察rgb图像、当前观察的rgb图像、当前观察的depth图像。请前进到首次观察图像中间办公桌前，并给出在世界空间中任务目标点相对相机中心的变换三维坐标点与yaw角旋转角。给出的三维坐标可以不与深度对应，但不得超出对应深度范围。请按照json格式{\"pos\": (x, y, z), \"yaw\": delta_yaw, \"mission_finish\": *}输出结果。已知深度相机的深度范围为0.2m-5.0m,参数为fx = fy = 442.025，cx = 320, cy = 240，不存在畸变，图像尺寸为height: 480, width: 640"
            ]
        self.command_content = []
        self.content = self.prepare_content.copy()
        self.pre_prompt = [
            "请前进到图像中左侧通道",
            "请原地左转45度",
            "请向前进入首次观察图像中右侧门内",
            "请原地右转45度",
            "请前进到首次观察图像中间办公桌前"
            ]
        self.prompt_bf = self.pre_prompt.copy()
        self.replan_time = [0, 0, 0, 0, 0, 0, 0]
        self.replan_bf = self.replan_time.copy()
        self.replan_content = None
        self.replan = False
        self.task_id = [1, 2, 2, 1]
        self.command_type = COMMAND_TYPE.WAIT
        self.task_id_mllm = []
        self.task_id_vlm = []
        self.vla_state = None
        self.frame = None
        self.ego_state_trigger = False
        self.mllm_message = None
        self.if_yaw = False
        self.result = None
        self.waypoint = None
        self.finish_mission = False
        self.first_plan = False
        self.finish_command = False
        self.bridge = CvBridge()

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

    def command_type_callback(self, msg: Int32):
        """处理指令需求回调"""
        self.command_type = msg.data
        rospy.loginfo(f"当前指令类型: {self.command_type}")
        if self.command_type == COMMAND_TYPE.NEXT:
            self.vla_state = VLA_STATE.PLAN
        if self.command_type == COMMAND_TYPE.AGAIN:
            self.command_content.insert(0, self.last_command)
            self.vla_state = VLA_STATE.PLAN
        if self.command_type == COMMAND_TYPE.GO_ORIGIN:
            self.vla_state = VLA_STATE.GO_ORIGIN
        if self.command_type == COMMAND_TYPE.EMERGENCY_STOP:
            print("🛑 收到紧急停止指令")
            self.emergency_stop_pub.publish(Empty())
        if self.command_type == COMMAND_TYPE.RESTART:
            print("🔄 收到重启指令，重新初始化")
            self.command_content = []
            self.first_mission_frame = self.frame.rgb_image.copy()
            self.vla_state = VLA_STATE.WAIT
        if self.command_type == COMMAND_TYPE.GET_PRE:
            replan_times = self.replan_time.pop(0)
            if self.prepare_content:
                prompt = self.pre_prompt.pop(0)
                rospy.loginfo(f"接受指令：{prompt}")
                self.replan_content = self.prepare_content.pop(0)
                self.command_content.append(self.replan_content)
                self.vla_state = VLA_STATE.PLAN
                while replan_times > 0:
                    time.sleep(4)
                    self.command_content.append(self.replan_content)
                    self.vla_state = VLA_STATE.PLAN
                    replan_times -= 1
                self.finish_command = True
            else:
                rospy.loginfo("完成所有任务")
        if self.command_type == COMMAND_TYPE.REPLAN:
            self.prepare_content = self.content.copy()
            self.pre_prompt = self.prompt_bf.copy()
            self.replan_time = self.replan_bf.copy()

    def command_content_callback(self, msg: String):
        """处理指令内容回调"""
        # 原封不动保存整段文本
        self.command_content.append(msg.data)
        self.vla_state = VLA_STATE.PLAN
        # rospy.loginfo(f"✅ 收到完整 content 消息: {msg.data}")

    def ego_state_trigger_callback(self, msg: Bool):
        """处理ego_state_trigger回调"""
        self.ego_state_trigger = msg.data
        # self.if_yaw = False
        # rospy.loginfo(f"当前ego_state_trigger状态: {self.ego_state_trigger}")
        if self.if_yaw:
            rospy.loginfo("执行旋转")
            # rospy.loginfo(f"waypoint:{self.waypoint}")
            self.publish_action(self.waypoint, look_forward=False)
            self.vla_state = VLA_STATE.WAIT_ACTION_FINISH
            self.if_yaw = False
            self.ego_state_trigger = False
        else:
            time.sleep(0.8)
            self.vla_state = VLA_STATE.REPLY_MLLM
            if self.finish_command:
                type_msg = Int32()
                type_msg.data = 8
                self.type_pub.publish(type_msg)
                self.finish_command = False
                self.finish_mission = True

    def get_command_content(self):
        """从mllm消息中提取指令内容"""
        if self.mllm_message is not None:
            return self.mllm_message
        return None

    def listen_messages(self):
        """循环监听mllm新消息"""
        pass

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
        try:
            ros_img = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
            pub.publish(ros_img)
            # rospy.loginfo(f"图像已发布，编码: {encoding}, shape: {image.shape}, dtype: {image.dtype}")
        except Exception as e:
            rospy.logerr(f"图像发布失败: {e}")


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
        response = self.publish_client.send_image(self.first_image)
        rospy.loginfo(f"发送第一张图片: {response}")

        # while not rospy.is_shutdown():
        #     if self.mllm_message is not None:
        #         rospy.loginfo(f"收到任务信息: {self.mllm_message}")
        #         self.command_content = self.get_command_content()
        #         self.mllm_message.append(self.command_content)
        #         break

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

            match self.vla_state:
                case VLA_STATE.INIT:
                    if getattr(self, 'depth_info', None) and self.get_frame_snapshot() is not None:
                        print("初始化完成")
                        self.vla_state = VLA_STATE.WAIT
                    else:
                        rate.sleep()
                        continue

                case VLA_STATE.WAIT:
                    if not self.command_content:
                        # self.vla_state = VLA_STATE.FINISH
                        # rospy.loginfo("当前任务为空，请传入下一步任务")
                        time.sleep(1)
                        continue

                case VLA_STATE.REPLY_MLLM:
                    # rospy.loginfo("到达目的地，基于MLLM回复")
                    self.vla_state = VLA_STATE.WAIT
                    response = self.publish_client.send_image(self.frame.rgb_image)
                    # rospy.loginfo(f"当前帧接收状态: {response['message']}")
                    continue

                case VLA_STATE.PLAN:
                    try:
                        if self.frame is None or getattr(self, 'depth_info', None) is None:
                            rospy.logwarn("传感器未准备好，重新初始化")
                            self.vla_state = VLA_STATE.INIT
                            continue
                        if self.finish_mission:
                            # self.command_content.pop(0)
                            print("上一任务完成，执行下一任务")
                            self.finish_mission = False
                            self.first_mission_frame = self.frame.rgb_image.copy()

                        cmd = self.command_content[0]

                        if is_waypoint_cmd(cmd):
                            waypoint = np.array(cmd, dtype=float).reshape(-1)[:3].tolist()
                            rospy.loginfo(f"收到直接导航导航点：{waypoint}")
                            self.vla_state = VLA_STATE.PUBLISH

                        elif is_rot_cmd(cmd):
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
                            self.publish_action(waypoint, look_forward=False)
                            self.command_content.pop(0)
                            self.vla_state = VLA_STATE.WAIT_ACTION_FINISH

                        elif is_label_cmd(cmd):
                            if not self.first_plan:
                                self.first_plan = True
                                self.first_mission_frame = self.frame.rgb_image.copy()
                            # TODO 待修改vlm输出信息结构
                            self.publish_image(self.first_mission_frame, self.first_image_pub)
                            self.publish_image(self.frame.rgb_image, self.current_image_pub)
                            origin_time = time.time()
                            self.result, self.finish_mission = open_serve(self.first_mission_frame, self.frame.rgb_image, cmd)
                            rospy.loginfo(f"推理耗时: {time.time() - origin_time:.2f} 秒")
                            # TODO 待修改mllm输入信息结构
                            pos_offset = np.array(self.result["pos"], dtype=np.float64)
                            xyz = self.frame.current_state[:3] + pos_offset
                            yaw = self.frame.current_state[5] + self.result["yaw"] / 180 * math.pi
                            if abs(yaw) > 1e-3:
                                self.if_yaw = True
                            waypoint = np.array([
                                xyz[0],
                                xyz[1],
                                xyz[2],
                                0.0,
                                0.0,
                                yaw
                            ], dtype=np.float64)

                            self.waypoint = waypoint

                            # rospy.loginfo(f"将前往{waypoint}")
                            self.vla_state = VLA_STATE.PUBLISH

                        else:
                            rospy.logwarn(f"收到未知指令：{cmd}，请检查指令格式")
                            self.vla_state = VLA_STATE.WAIT

                        self.last_plan_time = rospy.Time.now()

                    except Exception as e:
                        logging.error(f"规划失败: {e}")
                        self.vla_state = VLA_STATE.ERROR

                case VLA_STATE.PUBLISH:
                    if waypoint is None:
                        rospy.logwarn("没有有效的导航点，将重新规划")
                        self.vla_state = VLA_STATE.PLAN
                        continue

                    self.publish_action(waypoint)
                    self.command_content.pop(0)
                    self.vla_state = VLA_STATE.WAIT_ACTION_FINISH
                    print(f"当前位于: {self.frame.current_state}")
                    print(f"发布导航点: {waypoint}\n\n")
                    self.last_state = np.array([waypoint[0], waypoint[1], waypoint[2], 0, 0, 0], dtype=np.float64)
                    # time.sleep(0.5)  # 等待动作发布完成           
                case VLA_STATE.WAIT_ACTION_FINISH:
                    # self.frame = self.get_frame_snapshot()
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
                    self.vla_state = VLA_STATE.WAIT

                case VLA_STATE.GO_ORIGIN:
                    rospy.loginfo("收到 GO_ORIGIN 指令")
                    roll, pitch, yaw = self.quaternion_to_euler(6.79e-06, 2.86e-06, 1.000000, 0.000796)
                    self.waypoint = [11.810, -0.530, 1.400,
                                 roll, pitch, yaw]
                    self.publish_action(self.waypoint, look_forward=True)
                    # time.sleep(4)
                    # self.publish_action(self.waypoint, look_forward=False)
                    self.if_yaw = True
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
