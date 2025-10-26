#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2指令发布器 - 用于发送指令到test_simulation_ros2.py
"""

import json
import logging
import sys
import os

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ROSCommandPublisher:
    """ROS2指令发布器类"""
    
    def __init__(self):
        self.node = None
        self.publisher = None
        self.initialized = False
        
    def initialize(self):
        """初始化ROS2节点"""
        if not HAS_ROS2:
            logger.error("ROS2未安装或导入失败")
            return False
            
        try:
            # 检查ROS2环境
            if not os.environ.get('ROS_DOMAIN_ID'):
                logger.warning("ROS_DOMAIN_ID未设置，使用默认值")
            
            # 初始化ROS2
            if not rclpy.ok():
                rclpy.init()
            
            # 创建节点
            self.node = Node('command_publisher_node')
            
            # 创建发布器
            self.publisher = self.node.create_publisher(
                String,
                '/command/content',
                10
            )
            
            self.initialized = True
            logger.info("ROS2指令发布器初始化成功")
            return True
            
        except Exception as e:
            logger.error(f"ROS2初始化失败: {e}")
            return False
    
    def publish_command(self, command):
        """发布指令到ROS2话题"""
        if not self.initialized:
            if not self.initialize():
                return False, "ROS2初始化失败"
        
        try:
            # 创建消息
            msg = String()
            msg.data = json.dumps(command)
            
            # 发布消息
            self.publisher.publish(msg)
            
            # 短暂等待确保消息发送
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            logger.info(f"成功发布指令: {command}")
            return True, "指令发送成功"
            
        except Exception as e:
            logger.error(f"发布指令失败: {e}")
            return False, str(e)
    
    def shutdown(self):
        """关闭发布器"""
        try:
            if self.node:
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            logger.info("ROS2发布器已关闭")
        except Exception as e:
            logger.error(f"关闭ROS2发布器时出错: {e}")

# 全局发布器实例
_publisher_instance = None

def get_publisher():
    """获取发布器实例（单例模式）"""
    global _publisher_instance
    if _publisher_instance is None:
        _publisher_instance = ROSCommandPublisher()
    return _publisher_instance

def publish_to_ros(command):
    """发布指令到ROS2（便捷函数）"""
    publisher = get_publisher()
    return publisher.publish_command(command)

def check_ros2_available():
    """检查ROS2是否可用"""
    if not HAS_ROS2:
        return False, "ROS2 Python库未安装"
    
    try:
        # 检查环境变量
        if not os.environ.get('ROS_DISTRO'):
            return False, "ROS_DISTRO环境变量未设置"
        
        # 尝试初始化（不创建节点）
        if not rclpy.ok():
            rclpy.init()
            rclpy.shutdown()
        
        return True, "ROS2环境正常"
        
    except Exception as e:
        return False, f"ROS2环境检查失败: {e}"

if __name__ == "__main__":
    """命令行测试"""
    if len(sys.argv) != 2:
        print("用法: python ros_publisher.py '指令内容'")
        sys.exit(1)
    
    command = sys.argv[1]
    
    # 检查ROS2环境
    available, msg = check_ros2_available()
    if not available:
        print(f"ROS2不可用: {msg}")
        sys.exit(1)
    
    # 发送指令
    success, result = publish_to_ros(command)
    if success:
        print(f"指令发送成功: {command}")
    else:
        print(f"指令发送失败: {result}")
        sys.exit(1)
