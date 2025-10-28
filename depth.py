#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

class ImageInverter:
    def __init__(self):
        rospy.init_node('image_inverter', anonymous=True)
        
        self.bridge = CvBridge()
        
        # 订阅原始图像话题
        self.image_sub = rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", 
            Image, 
            self.image_callback
        )
        
        # 发布处理后的图像话题
        self.image_pub = rospy.Publisher(
            "/camera/aligned_depth/image_raw", 
            Image, 
            queue_size=10
        )
        
        rospy.loginfo("图像反转节点已启动，正在监听 /camera/aligned_depth_to_color/image_raw")
        
    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
            # 反转图像：255 - 像素值
            inverted_image = 255 - cv_image
            
            # 将处理后的图像转换回ROS消息格式
            # 保持原始消息的所有信息不变，只修改图像数据
            inverted_msg = self.bridge.cv2_to_imgmsg(inverted_image, encoding=msg.encoding)
            
            # 复制原始消息的所有头部和信息
            inverted_msg.header = msg.header
            inverted_msg.height = msg.height
            inverted_msg.width = msg.width
            inverted_msg.encoding = msg.encoding
            inverted_msg.is_bigendian = msg.is_bigendian
            inverted_msg.step = msg.step
            
            # 发布处理后的图像
            self.image_pub.publish(inverted_msg)
            
        except Exception as e:
            rospy.logerr("图像处理错误: %s", str(e))
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        inverter = ImageInverter()
        inverter.run()
    except rospy.ROSInterruptException:
        pass