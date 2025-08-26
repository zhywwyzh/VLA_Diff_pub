#!/usr/bin/env python3
import rospy
import time
import sys
import os
import threading
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class PositionImageRecorder:
    def __init__(self, duration, output_dir, image_dir, data_file):
        self.recording = False
        self.start_time = 0
        self.duration = float(duration)
        self.output_dir = output_dir
        self.image_dir = image_dir
        self.data_file = data_file
        self.target_file = os.path.join(output_dir, "target_points.csv")  # 目标点存储文件
        self.file = None
        self.image_counter = 0
        self.bridge = CvBridge()
        self.latest_odom = None
        self.odom_lock = threading.Lock()
        self.target_position = None  # 存储目标点位置
        self.position_tolerance = 0.001  # 位置容差 (米)
        self.orientation_tolerance = 0.05  # 姿态容差
        self.target_counter = 0  # 目标点计数器
        self.reached_target = False  # 标记是否已到达目标点
        self.extra_frames_count = 0  # 额外记录的帧数计数
        self.max_extra_frames = 5  # 最多额外记录的帧数
        
        # 确保图像目录和目标点文件目录存在
        os.makedirs(self.image_dir, exist_ok=True)
        
        # 初始化目标点文件并写入表头
        with open(self.target_file, 'w') as f:
            f.write("目标点ID,时间戳(秒),位置X,位置Y,位置Z,姿态X,姿态Y,姿态Z,姿态W\n")
        
        # 订阅目标点话题
        rospy.Subscriber("/uav_actions", PoseStamped, self.goal_callback)
        
        # 订阅无人机位姿话题
        rospy.Subscriber("/drone_0_visual_slam/odom", Odometry, self.odom_callback)
        
        # 订阅图像话题
        rospy.Subscriber("/camera/color/image/compressed", CompressedImage, self.image_callback)
        
        rospy.loginfo("位置和图像记录器已初始化，等待目标点...")
        print(f"记录编号: {os.path.basename(output_dir)}")
        print("位置和图像记录器已初始化，等待目标点...")
        print(f"目标点将存储在: {self.target_file}")
    
    def goal_callback(self, msg):
        # 记录目标点到CSV文件
        current_time = time.time() - self.start_time if self.recording else 0.0
        with open(self.target_file, 'a') as f:
            pos = msg.pose.position
            ori = msg.pose.orientation
            f.write(f"{self.target_counter},{current_time:.3f},")
            f.write(f"{pos.x:.3f},{pos.y:.3f},{pos.z:.3f},")
            f.write(f"{ori.x:.6f},{ori.y:.6f},{ori.z:.6f},{ori.w:.6f}\n")
        
        self.target_counter += 1
        print(f"✓ 已保存目标点 #{self.target_counter} 到 {os.path.basename(self.target_file)}")
        print(f"目标点位置: X={pos.x:.2f}, Y={pos.y:.2f}, Z={pos.z:.2f}")
        
        if not self.recording:
            self.recording = True
            self.start_time = time.time()
            self.file = open(self.data_file, 'w')
            self.file.write("时间戳(秒),位置X,位置Y,位置Z,姿态X,姿态Y,姿态Z,姿态W,图像文件名\n")
            
            # 存储目标点位置
            self.target_position = {
                'position': msg.pose.position,
                'orientation': msg.pose.orientation
            }
            
            rospy.loginfo("目标点已接收! 开始记录位置数据和图像...")
            print("✓ 目标点已接收! 开始记录位置数据和图像...")
    
    def check_position_reached(self, position, orientation):
        """检查无人机是否到达目标点位置"""
        if self.target_position is None:
            return False
        
        # 计算位置差异
        pos_diff = np.sqrt(
            (position.x - self.target_position['position'].x) ** 2 +
            (position.y - self.target_position['position'].y) ** 2 +
            (position.z - self.target_position['position'].z) ** 2
        )
        
        # 计算姿态差异（四元数点积）
        target_ori = self.target_position['orientation']
        ori_dot = abs(
            orientation.x * target_ori.x +
            orientation.y * target_ori.y +
            orientation.z * target_ori.z +
            orientation.w * target_ori.w
        )
        # 四元数点积接近1表示方向一致
        ori_diff = 1.0 - ori_dot
        
        # 检查是否在容差范围内
        position_reached = pos_diff < self.position_tolerance
        orientation_reached = ori_diff < self.orientation_tolerance
        
        # 如果需要更严格的检查，可以同时要求位置和姿态都满足条件
        return position_reached # and orientation_reached
    
    def odom_callback(self, msg):
        if self.recording:
            current_time = time.time() - self.start_time
            # if current_time > self.duration:
            #     self.stop_recording("记录时长已到")
            #     return
            
            # 检查是否到达目标点
            if self.target_position is not None and not self.reached_target:
                position_reached = self.check_position_reached(
                    msg.pose.pose.position,
                    msg.pose.pose.orientation
                )
                if position_reached:
                    print("✓ 已到达目标点! 将额外记录2帧后停止...")
                    self.reached_target = True
            
            # 更新最新位姿数据
            with self.odom_lock:
                self.latest_odom = {
                    'time': current_time,
                    'position': msg.pose.pose.position,
                    'orientation': msg.pose.pose.orientation
                }
    
    def image_callback(self, msg):
        if not self.recording:
            return
            
        current_time = time.time() - self.start_time
        # if current_time > self.duration:
        #     self.stop_recording("记录时长已到")
        #     return
        
        try:
            # 转换压缩图像为OpenCV格式
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            
            # 获取当前位姿（如果有）
            with self.odom_lock:
                current_odom = self.latest_odom
            
            # 保存图像
            image_filename = f"image_{self.image_counter:05d}.jpg"
            image_path = os.path.join(self.image_dir, image_filename)
            cv2.imwrite(image_path, cv_image)
            
            # 记录数据
            if current_odom is not None:
                pos = current_odom['position']
                ori = current_odom['orientation']
                self.file.write(f"{current_odom['time']:.3f},{pos.x:.3f},{pos.y:.3f},{pos.z:.3f},")
                self.file.write(f"{ori.x:.6f},{ori.y:.6f},{ori.z:.6f},{ori.w:.6f},{image_filename}\n")
            else:
                self.file.write(f"{current_time:.3f},,,, , , , ,{image_filename}\n")
            
            self.image_counter += 1
            print(f"✓ 已保存图像: {image_filename} (计数: {self.image_counter})")
            
            # 如果已到达目标点，进行额外记录
            if self.reached_target:
                self.extra_frames_count += 1
                print(f"✓ 额外记录第 {self.extra_frames_count} 帧")
                
                # 达到额外记录帧数后停止
                if self.extra_frames_count >= self.max_extra_frames:
                    print(f"✓ 已完成额外记录 {self.max_extra_frames} 帧! 将停止记录...")
                    self.stop_recording(f"已到达目标点并完成额外记录{self.max_extra_frames}帧")
            
        except Exception as e:
            rospy.logerr(f"图像处理错误: {str(e)}")
    
    def stop_recording(self, reason="未知原因"):
        if self.recording:
            self.recording = False
            if self.file:
                self.file.close()
            rospy.loginfo(f"记录完成! ({reason}) 数据保存在 {self.output_dir}")
            print(f"✓ 记录完成! ({reason})")
            print(f"位置数据: {os.path.basename(self.data_file)}")
            print(f"目标点数据: {os.path.basename(self.target_file)}")
            print(f"图像数量: {self.image_counter} (保存在 {os.path.basename(self.image_dir)}/)")
            rospy.signal_shutdown(reason)

if __name__ == '__main__':
    if len(sys.argv) < 5:
        print("参数错误: 需要记录时长、输出目录、图像目录和数据文件")
        print("用法: rosrun your_package position_image_recorder.py <duration> <output_dir> <image_dir> <data_file>")
        sys.exit(1)
        
    rospy.init_node('position_image_recorder', anonymous=True)
    recorder = PositionImageRecorder(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        if recorder.file:
            recorder.file.close()
        print("脚本被中断! 已保存数据.")