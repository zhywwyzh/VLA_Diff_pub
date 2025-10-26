#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

# 全局变量来存储最大深度值
max_depth_value = 0
bridge = CvBridge()

def image_callback(msg):
    global max_depth_value
    
    try:
        # 将ROS图像消息转换为OpenCV图像
        # 对于深度图像，使用'16UC1'编码
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='8SC1')
        
        # 检查图像中的最大值
        current_max = np.max(cv_image)
        
        # 更新全局最大深度值
        if current_max > max_depth_value:
            max_depth_value = current_max
            rospy.loginfo("新的最大深度值: %d", max_depth_value)
        
        # 打印当前帧的统计信息
        rospy.loginfo_once("深度图像信息: 尺寸=%s, 类型=%s, 范围=[%d, %d]", 
                          str(cv_image.shape), cv_image.dtype, 
                          np.min(cv_image), np.max(cv_image))
        
        # 如果你想查看非零值的统计信息（排除背景）
        non_zero_values = cv_image[cv_image > 0]
        if len(non_zero_values) > 0:
            rospy.loginfo_throttle(5, "非零深度值 - 最小值: %d, 最大值: %d, 平均值: %.2f", 
                                 np.min(non_zero_values), np.max(non_zero_values), 
                                 np.mean(non_zero_values))
        
    except Exception as e:
        rospy.logerr("处理深度图像时出错: %s", str(e))

def main():
    rospy.init_node('image_listener', anonymous=True)
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, image_callback)
    
    rospy.loginfo("开始监听深度图像，检查最大深度值...")
    rospy.spin()

if __name__ == '__main__':
    main()