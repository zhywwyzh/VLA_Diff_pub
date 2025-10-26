#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import message_filters
from sensor_msgs.msg import Image, CompressedImage # 导入 CompressedImage
from nav_msgs.msg import Odometry
import cv2 # 导入 OpenCV 库
from cv_bridge import CvBridge, CvBridgeError # 导入 CvBridge

# import pyrealsense2 as rs

# def syncColorAndDepth(color_msg, depth_msg):
#     # 对齐版本
#     aligned_frames = align.process(frames)
#     depth_frame_aligned = aligned_frames .get_depth_frame()
#     color_frame_aligned = aligned_frames .get_color_frame()
#     # if not depth_frame_aligned or not color_frame_aligned:
#     #     continue
#     color_image_aligned = np.asanyarray(color_frame_aligned.get_data())
#     if USE_ROS_BAG:
#         color_image_aligned=cv2.cvtColor(color_image_aligned,cv2.COLOR_BGR2RGB)
#     depth_image_aligned = np.asanyarray(depth_frame_aligned.get_data())
 
#     depth_colormap_aligned = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_aligned, alpha=0.05), cv2.COLORMAP_JET)
#     images_aligned = np.hstack((color_image_aligned, depth_colormap_aligned))
#     if show_pic:
#         cv2.imshow('aligned_images', images_aligned)
#     return color_image_aligned,depth_image_aligned,depth_colormap_aligned

class TopicSynchronizer:
    def __init__(self):
        """
        初始化节点、发布者、订阅者和 message_filters 同步器。
        """
        rospy.init_node('topic_synchronizer_node', anonymous=True)

        rospy.loginfo("节点初始化...")

        # --- 初始化 CvBridge 实例 ---
        self.bridge = CvBridge()

        # --- 从参数服务器获取话题名称，如果未设置则使用默认值 ---
        color_topic = rospy.get_param('~color_topic', '/camera/color/image_raw')
        depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_raw')
        odom_topic = rospy.get_param('~odom_topic', '/ekf_quat/ekf_odom')

        # --- 初始化发布者 ---
        # 现在彩色图像和深度图像发布 CompressedImage 消息
        self.color_pub = rospy.Publisher(color_topic + '_fix', CompressedImage, queue_size=10)
        self.depth_pub = rospy.Publisher(depth_topic + '_fix', CompressedImage, queue_size=10)
        self.odom_pub = rospy.Publisher(odom_topic + '_fix', Odometry, queue_size=10) # Odometry 不变

        rospy.loginfo("发布者已创建:")
        rospy.loginfo("  - %s (Type: CompressedImage)", self.color_pub.name)
        rospy.loginfo("  - %s (Type: CompressedImage)", self.depth_pub.name)
        rospy.loginfo("  - %s (Type: Odometry)", self.odom_pub.name)


        # --- 初始化订阅者 (使用 message_filters) ---
        # 订阅的仍然是原始的 Image 消息
        color_sub = message_filters.Subscriber(color_topic, Image)
        depth_sub = message_filters.Subscriber(depth_topic, Image)
        odom_sub = message_filters.Subscriber(odom_topic, Odometry)

        rospy.loginfo("正在订阅话题:")
        rospy.loginfo("  - %s (Type: Image)", color_topic)
        rospy.loginfo("  - %s (Type: Image)", depth_topic)
        rospy.loginfo("  - %s (Type: Odometry)", odom_topic)

        # --- 配置 ApproximateTimeSynchronizer ---
        # 'slop': (单位：秒) 决定了消息时间戳之间可接受的最大差异。
        self.ats = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub, odom_sub],
            queue_size=10,
            slop=0.1,  # 允许0.1秒的时间差，根据实际传感器频率调整
            allow_headerless=False
        )

        # --- 注册回调函数 ---
        self.ats.registerCallback(self.callback)

        rospy.loginfo("时间同步器已启动，等待消息...")

    def callback(self, color_msg, depth_msg, odom_msg):
        """
        当接收到一组时间同步的消息时，此函数被调用。
        :param color_msg: sensor_msgs/Image 类型的彩色图像消息
        :param depth_msg: sensor_msgs/Image 类型的深度图像消息
        :param odom_msg: nav_msgs/Odometry 类型的里程计消息
        """
        try:
            # --- 处理彩色图像 ---
            # 将 ROS Image 消息转换为 OpenCV 图像
            cv_color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8") # "bgr8" 适合彩色图
            # 将 OpenCV 图像编码为 JPEG 格式
            # quality参数 (0-100) 可以控制压缩质量和文件大小，默认为95
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90] # 90% 质量
            result, encoded_color_image = cv2.imencode('.jpg', cv_color_image, encode_param)
            if not result:
                rospy.logerr("无法编码彩色图像为 JPEG!")
                return
            # 创建 CompressedImage 消息
            compressed_color_msg = CompressedImage()
            compressed_color_msg.header = color_msg.header
            compressed_color_msg.format = "jpeg"
            compressed_color_msg.data = encoded_color_image.tobytes()
            self.color_pub.publish(compressed_color_msg)

            # --- 处理深度图像 ---
            # 深度图通常是16位单通道图像。转换为OpenCV时需要注意编码
            # "passthrough" 保留原始编码，如果深度图是16UC1，则cv_depth_image也会是这个类型
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            
            encode_param_png = [int(cv2.IMWRITE_PNG_COMPRESSION), 3] # PNG 压缩级别 (0-9)
            result_depth, encoded_depth_image = cv2.imencode('.png', cv_depth_image, encode_param_png) # PNG 压缩
            if not result_depth:
                rospy.logerr("无法编码深度图像为 PNG!")
                return

            compressed_depth_msg = CompressedImage()
            compressed_depth_msg.header = depth_msg.header
            compressed_depth_msg.format = "png" # 格式指定为 png
            compressed_depth_msg.data = encoded_depth_image.tobytes()
            self.depth_pub.publish(compressed_depth_msg)

            # --- 处理里程计消息 ---
            # 里程计消息直接转发
            self.odom_pub.publish(odom_msg)

        except CvBridgeError as e:
            rospy.logerr("CvBridgeError: %s", e)
        except Exception as e:
            rospy.logerr("处理消息时发生错误: %s", e)

if __name__ == '__main__':
    try:
        synchronizer = TopicSynchronizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点已关闭。")
        pass