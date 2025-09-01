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

from utils.vlm.openai_serve import open_serve
from utils.param import COMMAND_TYPE, VLA_STATE

from base_policy import BasePolicyNode  # 假定已有 ROS1 版本或与 ROS 无关的 Base
from utils.server.publish_client import MessageClient
from utils.server.receive_client import GeminiMessageClient


class UAVPolicyNode(BasePolicyNode):
    def __init__(self):
        super().__init__()

        # 创建CV桥接器
        self.bridge = CvBridge()

        # 存储最新的状态和图像
        self.current_image = None
        self.current_wrist_image = None
        self.first_image = None
        self.first_image_received = False
        self.count = 0
        self.save_count = 0
        self.image_save_count = 0
        self.last_state = [0, 0, 1, 0, 0, 0]
        self.last_plan_time = None
        self.inference_timeout = 5.0
        self.save = None
        self.plan = True
        self.last_command = None
        self.first_mission_frame = None
        self.first_plan = False
        self.command_content = ['start_mission']
        self.replan = False
        self.task_id = [1, 2, 2, 1]
        self.command_type = COMMAND_TYPE.WAIT
        self.task_id_mllm = []
        self.task_id_vlm = []
        self.pub_goal = False
        self.arrival_distance = 0.1
        self.time_out = 10.0
        self.vla_state = None
        self.frame = None
        self.ego_state_trigger = False
        self.mllm_message = None
        self.if_yaw = False
        self.waypoint = None
        self.result = None
        self.finish_mission = True

        # 创建图像保存目录
        self.image_save_dir = '/home/zhywwyzh/workspace/VLA_Diff/Openpi/test/infer/trail/saved_images'
        os.makedirs(self.image_save_dir, exist_ok=True)

        # 创建模型输入图像保存目录
        self.model_input_dir = '/home/zhywwyzh/workspace/VLA_Diff/Openpi/test/infer/trail/model_input_images'
        os.makedirs(self.model_input_dir, exist_ok=True)

        # 订阅指令需求
        self.command_type_sub = rospy.Subscriber(
            "command/type", Int32, self.command_type_callback, queue_size=10
        )
        self.command_content_sub = rospy.Subscriber(
            "command/content", String, self.command_content_callback, queue_size=10
        )

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

        # 初始化 WebSocket / 服务端客户端
        self.client = None
        self.prompt = None

        # 初始化 mllm 客户端
        self.receive_client = GeminiMessageClient()
        self.publish_client = MessageClient()

        # 轨迹存储
        self.original_trajectory = []
        self.inferred_trajectory = []

        self.tasks_jsonl_path = "/home/zhywwyzh/workspace/VLA_Diff/Openpi/test/infer/test_data/meta/tasks.jsonl"
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

    def command_content_callback(self, msg: String):
        """处理指令内容回调"""
        try:
            self.command_content.append(json.loads(msg.data))
        except Exception as e:
            rospy.logerr(f"解析 command/content 失败: {e}")

    # TODO 调整ego回调逻辑
    def ego_state_trigger_callback(self, msg: Bool):
        """处理ego_state_trigger回调"""
        self.ego_state_trigger = msg.data
        # rospy.loginfo(f"当前ego_state_trigger状态: {self.ego_state_trigger}")
        # self.val_state = VLA_STATE.PLAN
        if self.finish_mission:
            if self.if_yaw:
                # rospy.loginfo("执行旋转")
                # rospy.loginfo(f"waypoint:{self.waypoint}")
                self.publish_action(self.waypoint, look_forward=False)
                self.vla_state = VLA_STATE.WAIT_ACTION_FINISH
                self.if_yaw = False
                self.ego_state_trigger = False
            else:
                time.sleep(0.8)
                self.vla_state = VLA_STATE.REPLY_MLLM

    # TODO 修改提取方案
    def get_command_content(self):
        """从mllm消息中提取指令内容"""
        if self.mllm_message is not None:
            return self.mllm_message
        return None

    # TODO 修改监听端口
    def listen_messages(self):
        """循环监听mllm新消息"""
        while not rospy.is_shutdown():
            try:
                message = self.receive_client.get_new_messages()
                if message:
                    self.mllm_message = message[0]["text"]
                    self.command_content.append(self.mllm_message)
                    rospy.loginfo(f"Received message: {self.mllm_message}")
            except Exception as e:
                rospy.logerr(f"消息监听出错: {e}")
            time.sleep(0.5)

    def publish_action(self, action, look_forward=True, goal_to_follower=False):
        """发布动作到ROS话题"""
        if len(action) < 3:
            logging.error("动作数据不足3个元素")
            return

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
            roll, pitch, yaw = self.quaternion_to_euler(float(action[3]), float(action[4]), float(action[5]), float(action[6]))
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

    # TODO 推理逻辑初始化，初始与mllm的通信
    def run_inference(self):
        """执行推理"""
        rate = rospy.Rate(20)
        replan_rate = rospy.Rate(2)
        waypoint = None
        self.vla_state = VLA_STATE.INIT
        self.last_plan_time = None

        while not rospy.is_shutdown():
            if self.mllm_message is not None:
                self.get_logger().info(f"收到任务信息: {self.mllm_message}")
                self.command_content = self.get_command_content()
                self.mllm_message.append(self.command_content)
                break
            time.sleep(0.5)

        def is_waypoint_cmd(c) -> bool:
            if isinstance(c, (list, tuple, np.ndarray)):
                arr = np.array(c, dtype=float).reshape(-1)
                return arr.size >= 3 and np.all(np.isfinite(arr[:3]))
            return False

        def is_label_cmd(c) -> bool:
            return isinstance(c, str) and len(c) > 0

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
                        rospy.loginfo("当前任务为空，请传入下一步任务")
                        time.sleep(1)
                        continue
                    # if self.command_type == COMMAND_TYPE.GO:
                    #     self.vla_state = VLA_STATE.PLAN
                    # else:
                    #     rate.sleep()

                case VLA_STATE.REPLY_MLLM:
                    rospy.loginfo("到达目的地，基于MLLM回复")
                    self.vla_state = VLA_STATE.WAIT
                    response = self.publish_client.send_image(self.frame.rgb_image)
                    rospy.loginfo(f"当前帧接收状态: {response['message']}")
                    continue

                # TODO 调整plan与replan
                case VLA_STATE.PLAN:
                    try:
                        if self.frame is None or getattr(self, 'depth_info', None) is None:
                            rospy.logwarn("传感器未准备好，重新初始化")
                            self.vla_state = VLA_STATE.INIT
                            continue

                        if self.finish_mission:
                            self.command_content.pop(0)
                            self.finish_mission = False
                            self.first_mission_frame = self.frame

                        cmd = self.command_content[0]

                        if is_waypoint_cmd(cmd):
                            waypoint = np.array(cmd, dtype=float).reshape(-1)[:3].tolist()
                            rospy.loginfo(f"收到直接导航导航点：{waypoint}")
                            self.vla_state = VLA_STATE.PUBLISH

                        elif is_label_cmd(cmd):
                            if not self.first_plan:
                                self.first_plan = True
                                self.first_mission_frame = self.frame.rgb_image.copy()
                                response = self.publish_client.send_image([self.first_mission_frame, self.frame.rgb_image], cmd, "uav_policy_node")

                            # TODO 待修改vlm输出信息结构
                            result, self.finish_mission = open_serve(self.first_mission_frame, self.frame.rgb_image, cmd)
                            rospy.loginfo(f"推理结果：{result}，是否达到目的地：{self.finish_mission}")

                            # TODO 待修改mllm输入信息结构
                            waypoint, self.replan = self.pixel_to_world(result, self.frame)
                            rospy.loginfo(f"将前往{waypoint}")
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
                    self.vla_state = VLA_STATE.WAIT_ACTION_FINISH
                    print(f"发布导航点: {waypoint}")
                    self.last_state = np.array([waypoint[0], waypoint[1], waypoint[2], 0, 0, 0], dtype=np.float64)
                    # time.sleep(0.5)  # 等待动作发布完成
                    # self.vla_state = VLA_STATE.ERROR
            
                case VLA_STATE.WAIT_ACTION_FINISH:
                    if not self.finish_mission:
                        self.vla_state = VLA_STATE.PLAN
                        replan_rate.sleep()
                        continue
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
                    roll, pitch, yaw = self.quaternion_to_euler(6.791822847841003e-06, 2.862005646358964e-06, 0.9351266681836927, -0.3543135820874861)
                    waypoint = [13.136453639674091, 0.5770826187959396, 1.5999999937387603,
                                 roll, pitch, yaw]
                    self.publish_action(waypoint, look_forward=False)
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
