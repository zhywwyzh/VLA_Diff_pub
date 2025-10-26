#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2转发功能集成测试脚本
"""

import requests
import json
import time
import sys

def test_main_service():
    """测试主服务连接"""
    print("🔍 测试主服务连接...")
    try:
        response = requests.get('http://localhost:1024/api/test', timeout=5)
        result = response.json()
        
        if result['success']:
            print("✅ 主服务连接正常")
            print(f"   - 模型: {result['model']}")
            print(f"   - API状态: {result['api_status']}")
            print(f"   - ROS2可用: {result.get('ros2_available', False)}")
            print(f"   - ROS2状态: {result.get('ros2_status', '未知')}")
            print(f"   - ROS2发布器: {result.get('ros2_publisher_available', False)}")
            return True
        else:
            print("❌ 主服务测试失败")
            return False
    except Exception as e:
        print(f"❌ 主服务连接失败: {e}")
        return False

def test_ros_command(command):
    """测试ROS2指令发送"""
    print(f"🚀 测试发送ROS2指令: {command}")
    try:
        response = requests.post(
            'http://localhost:1024/api/ros/command',
            json={'command': command},
            timeout=10
        )
        result = response.json()
        
        if result['success']:
            print("✅ ROS2指令发送成功")
            print(f"   - 指令: {result['command']}")
            print(f"   - 方法: {result.get('method', '未知')}")
            if result.get('output'):
                print(f"   - 输出: {result['output']}")
            return True
        else:
            print(f"❌ ROS2指令发送失败: {result.get('error', '未知错误')}")
            return False
    except Exception as e:
        print(f"❌ ROS2指令发送异常: {e}")
        return False

def test_forward_config():
    """测试转发配置"""
    print("🔧 测试转发配置...")
    try:
        response = requests.get('http://localhost:1024/api/config/forward', timeout=5)
        result = response.json()
        
        if result['success']:
            print("✅ 转发配置获取成功")
            print(f"   - 转发URL: {result.get('forward_url', '未设置')}")
            print(f"   - 启用状态: {result.get('enabled', False)}")
            return True
        else:
            print("❌ 转发配置获取失败")
            return False
    except Exception as e:
        print(f"❌ 转发配置测试失败: {e}")
        return False

def main():
    """主测试函数"""
    print("🧪 开始ROS2转发功能集成测试")
    print("=" * 50)
    
    # 测试主服务
    if not test_main_service():
        print("\n❌ 主服务测试失败，请确保app.py正在运行")
        return False
    
    print("\n" + "-" * 30)
    
    # 测试转发配置
    if not test_forward_config():
        print("\n⚠️ 转发配置测试失败，但不影响ROS2功能")
    
    print("\n" + "-" * 30)
    
    # 测试ROS2指令发送
    test_commands = [
        "前往两个饮水机正中间",
        "找到右侧第一个门",
        "前往右侧门的右后方",
        "找到树的位置"
    ]
    
    success_count = 0
    for i, cmd in enumerate(test_commands, 1):
        print(f"\n📝 测试 {i}/{len(test_commands)}")
        if test_ros_command(cmd):
            success_count += 1
        time.sleep(1)  # 避免发送过快
    
    print("\n" + "=" * 50)
    print(f"🎯 测试结果: {success_count}/{len(test_commands)} 个指令发送成功")
    
    if success_count == len(test_commands):
        print("🎉 所有测试通过！ROS2转发功能正常工作")
        return True
    else:
        print("⚠️ 部分测试失败，请检查ROS2环境和test_simulation_ros2.py运行状态")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)