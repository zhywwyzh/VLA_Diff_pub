#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试多图片转发功能

这个脚本测试新的多图片转发功能，确保图片信息能正确传递到外部接收器。
"""

import requests
import base64
import json
from PIL import Image
import io
import os

def create_test_image(width=200, height=200, color='red'):
    """创建测试图片"""
    img = Image.new('RGB', (width, height), color)
    buffer = io.BytesIO()
    img.save(buffer, format='PNG')
    buffer.seek(0)
    img_data = base64.b64encode(buffer.getvalue()).decode('utf-8')
    return f"data:image/png;base64,{img_data}"

def test_multiple_images():
    """测试多图片功能"""
    print("🧪 测试多图片转发功能")
    print("=" * 50)
    
    # 创建多个测试图片
    images = [
        create_test_image(200, 200, 'red'),
        create_test_image(150, 150, 'blue'),
        create_test_image(100, 100, 'green')
    ]
    
    # 准备测试数据
    test_data = {
        'text': '请分析这些图片',
        'images': images,  # 发送多张图片
        'model': 'gemini-2.0-flash-exp'
    }
    
    print(f"📤 发送测试请求...")
    print(f"   文字: {test_data['text']}")
    print(f"   图片数量: {len(images)}")
    
    try:
        # 发送到主服务
        response = requests.post(
            'http://localhost:1024/api/chat',
            json=test_data,
            timeout=30
        )
        
        if response.status_code == 200:
            result = response.json()
            if result.get('success'):
                print("✅ 测试请求发送成功")
                print(f"📝 AI回复: {result['response']['text'][:100]}...")
                
                # 检查外部接收器是否收到消息
                print("\n🔍 检查外部接收器...")
                check_response = requests.get('http://localhost:3000/api/messages')
                if check_response.status_code == 200:
                    messages = check_response.json()
                    if messages.get('messages'):
                        latest_msg = messages['messages'][0]
                        print("✅ 外部接收器收到消息:")
                        print(f"   消息ID: {latest_msg.get('id')}")
                        print(f"   消息类型: {latest_msg.get('message_type')}")
                        print(f"   图片数量: {latest_msg.get('image_count', 0)}")
                        print(f"   用户输入图片: {latest_msg.get('user_input', {}).get('image_count', 0)}")
                        
                        if latest_msg.get('images_info'):
                            print("🖼️ 图片信息:")
                            for img_info in latest_msg['images_info']:
                                print(f"     图片 {img_info.get('index')}: {img_info.get('format')} ({img_info.get('size')})")
                    else:
                        print("⚠️ 外部接收器没有收到消息")
                else:
                    print(f"❌ 无法连接到外部接收器: {check_response.status_code}")
            else:
                print(f"❌ 主服务返回错误: {result.get('error')}")
        else:
            print(f"❌ 请求失败: {response.status_code}")
            print(response.text)
            
    except Exception as e:
        print(f"❌ 测试失败: {e}")

def test_service_status():
    """检查服务状态"""
    print("\n🔍 检查服务状态...")
    
    services = [
        ('主服务', 'http://localhost:1024/api/test'),
        ('外部接收器', 'http://localhost:3000/api/health'),
        ('外部消息接收器', 'http://localhost:3001/api/health')
    ]
    
    for name, url in services:
        try:
            response = requests.get(url, timeout=5)
            if response.status_code == 200:
                print(f"✅ {name}: 运行中")
            else:
                print(f"⚠️ {name}: 响应异常 ({response.status_code})")
        except Exception as e:
            print(f"❌ {name}: 无法连接 ({e})")

if __name__ == '__main__':
    print("🚀 多图片转发功能测试")
    print("=" * 50)
    
    # 检查服务状态
    test_service_status()
    
    # 等待用户确认
    input("\n按Enter键开始测试...")
    
    # 执行测试
    test_multiple_images()
    
    print("\n🎉 测试完成！")
    print("\n💡 提示:")
    print("1. 如果测试成功，说明多图片转发功能正常工作")
    print("2. 检查外部接收器控制台输出，应该能看到图片信息")
    print("3. 检查 received_images/ 目录，应该能看到保存的图片文件")
