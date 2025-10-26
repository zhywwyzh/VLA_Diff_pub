#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
增强版消息发送工具 - 支持发送图片到3001端口

这个工具扩展了原有的消息发送功能，增加了图片上传支持。
可以发送文本消息、图片消息，或者文本+图片的组合消息。
"""

import requests
import json
import time
import sys
import os
import base64
from PIL import Image
import io
from typing import List, Optional

def image_to_base64(image_path: str) -> str:
    """将图片文件转换为base64格式"""
    try:
        with Image.open(image_path) as img:
            # 如果图片太大，进行压缩
            max_size = (1024, 1024)
            if img.size[0] > max_size[0] or img.size[1] > max_size[1]:
                print(f"📏 图片尺寸 {img.size} 过大，压缩至 {max_size}")
                img.thumbnail(max_size, Image.Resampling.LANCZOS)
            
            # 转换为RGB（如果是RGBA）
            if img.mode in ('RGBA', 'P'):
                rgb_img = Image.new('RGB', img.size, (255, 255, 255))
                rgb_img.paste(img, mask=img.split()[-1] if img.mode == 'RGBA' else None)
                img = rgb_img
            
            # 转换为base64
            buffer = io.BytesIO()
            img.save(buffer, format='JPEG', quality=85)
            buffer.seek(0)
            img_data = base64.b64encode(buffer.getvalue()).decode('utf-8')
            
            return f"data:image/jpeg;base64,{img_data}", img.size
            
    except Exception as e:
        print(f"❌ 处理图片 {image_path} 时出错: {e}")
        return None, None

def create_test_image(width=300, height=200, color='blue', text="测试图片") -> str:
    """创建测试图片"""
    try:
        from PIL import ImageDraw, ImageFont
        
        # 创建图片
        img = Image.new('RGB', (width, height), color)
        draw = ImageDraw.Draw(img)
        
        # 尝试加载字体
        try:
            # 在macOS上尝试使用系统字体
            font = ImageFont.truetype("/System/Library/Fonts/PingFang.ttc", 24)
        except:
            try:
                # 备用字体
                font = ImageFont.truetype("/System/Library/Fonts/Arial.ttf", 24)
            except:
                # 默认字体
                font = ImageFont.load_default()
        
        # 添加文字
        text_bbox = draw.textbbox((0, 0), text, font=font)
        text_width = text_bbox[2] - text_bbox[0]
        text_height = text_bbox[3] - text_bbox[1]
        
        x = (width - text_width) // 2
        y = (height - text_height) // 2
        
        draw.text((x, y), text, fill='white', font=font)
        
        # 转换为base64
        buffer = io.BytesIO()
        img.save(buffer, format='JPEG', quality=90)
        buffer.seek(0)
        img_data = base64.b64encode(buffer.getvalue()).decode('utf-8')
        
        return f"data:image/jpeg;base64,{img_data}", (width, height)
        
    except Exception as e:
        print(f"❌ 创建测试图片时出错: {e}")
        return None, None

def send_message_with_images(text: str, 
                           images: Optional[List[str]] = None,
                           sender: str = "测试程序", 
                           message_type: str = "text", 
                           priority: str = "normal"):
    """发送包含图片的消息到3001端口"""
    url = "http://localhost:3001/api/send"
    
    # 准备图片信息
    images_info = []
    if images:
        for i, img_path in enumerate(images):
            if os.path.exists(img_path):
                img_data, img_size = image_to_base64(img_path)
                if img_data:
                    images_info.append({
                        'index': i + 1,
                        'filename': os.path.basename(img_path),
                        'size': f"{img_size[0]}x{img_size[1]}" if img_size else "unknown",
                        'format': 'JPEG',
                        'data': img_data
                    })
                    print(f"✅ 图片 {i+1}: {os.path.basename(img_path)} ({img_size})")
                else:
                    print(f"❌ 图片 {i+1}: {os.path.basename(img_path)} 处理失败")
            else:
                print(f"❌ 图片文件不存在: {img_path}")
    
    data = {
        "text": text,
        "sender": sender,
        "type": message_type,
        "priority": priority,
        "images_info": images_info if images_info else None,
        "image_count": len(images_info)
    }
    
    try:
        response = requests.post(url, json=data, timeout=10)
        
        if response.status_code == 200:
            result = response.json()
            if result['success']:
                print(f"✅ 消息发送成功: ID {result['message_id']}")
                return True
            else:
                print(f"❌ 发送失败: {result.get('error', '未知错误')}")
                return False
        else:
            print(f"❌ HTTP错误: {response.status_code}")
            return False
            
    except Exception as e:
        print(f"❌ 发送异常: {e}")
        return False

def send_test_images():
    """发送测试图片"""
    print("🖼️ 生成测试图片...")
    
    # 创建几个不同的测试图片
    test_images = []
    colors = ['red', 'green', 'blue']
    
    for i, color in enumerate(colors, 1):
        img_data, img_size = create_test_image(
            width=300 + i*50, 
            height=200 + i*30, 
            color=color, 
            text=f"测试图片 {i}"
        )
        if img_data:
            # 保存临时图片文件
            temp_filename = f"temp_test_image_{i}.jpg"
            
            # 从base64保存到文件
            img_bytes = base64.b64decode(img_data.split(',')[1])
            with open(temp_filename, 'wb') as f:
                f.write(img_bytes)
            
            test_images.append(temp_filename)
            print(f"✅ 生成测试图片 {i}: {temp_filename} ({img_size})")
    
    if test_images:
        print(f"\n📤 发送包含 {len(test_images)} 张图片的消息...")
        success = send_message_with_images(
            text=f"这是一条包含 {len(test_images)} 张测试图片的消息，用于测试图片发送功能。",
            images=test_images,
            sender="图片测试程序",
            message_type="text",
            priority="normal"
        )
        
        # 清理临时文件
        for temp_file in test_images:
            try:
                os.remove(temp_file)
                print(f"🗑️ 清理临时文件: {temp_file}")
            except:
                pass
        
        return success
    else:
        print("❌ 无法生成测试图片")
        return False

def send_custom_message_with_images():
    """发送自定义消息（支持图片）"""
    print("📝 发送自定义消息（支持图片）")
    print("=" * 50)
    
    text = input("请输入消息内容: ").strip()
    if not text:
        print("❌ 消息内容不能为空")
        return
    
    sender = input("发送者 (默认: 手动测试): ").strip() or "手动测试"
    
    print("\n消息类型:")
    print("1. text - 普通文本")
    print("2. query - 查询")  
    print("3. command - 命令")
    print("4. notification - 通知")
    
    type_choice = input("选择类型 (1-4, 默认: 1): ").strip() or "1"
    type_map = {"1": "text", "2": "query", "3": "command", "4": "notification"}
    message_type = type_map.get(type_choice, "text")
    
    print("\n优先级:")
    print("1. normal - 普通")
    print("2. high - 高")
    print("3. urgent - 紧急")
    
    priority_choice = input("选择优先级 (1-3, 默认: 1): ").strip() or "1"
    priority_map = {"1": "normal", "2": "high", "3": "urgent"}
    priority = priority_map.get(priority_choice, "normal")
    
    # 处理图片
    images = []
    add_images = input("\n是否添加图片? (y/N): ").strip().lower()
    
    if add_images in ['y', 'yes']:
        while True:
            img_path = input(f"请输入图片路径 (第{len(images)+1}张，直接回车结束): ").strip()
            if not img_path:
                break
            
            if os.path.exists(img_path):
                images.append(img_path)
                print(f"✅ 添加图片: {os.path.basename(img_path)}")
            else:
                print(f"❌ 文件不存在: {img_path}")
            
            if len(images) >= 5:  # 限制最多5张图片
                print("⚠️ 最多支持5张图片")
                break
    
    print(f"\n📤 发送消息...")
    print(f"   内容: {text}")
    print(f"   发送者: {sender}")
    print(f"   类型: {message_type}")
    print(f"   优先级: {priority}")
    print(f"   图片数量: {len(images)}")
    
    success = send_message_with_images(text, images, sender, message_type, priority)
    if success:
        print("✅ 消息发送成功！")
    else:
        print("❌ 消息发送失败！")

def check_3001_service():
    """检查3001端口服务状态"""
    try:
        response = requests.get("http://localhost:3001/api/health", timeout=5)
        if response.status_code == 200:
            data = response.json()
            print("✅ 3001端口服务运行正常")
            print(f"   服务: {data.get('service', 'unknown')}")
            print(f"   总消息数: {data.get('total_messages', 0)}")
            print(f"   未读消息: {data.get('unread_messages', 0)}")
            return True
        else:
            print(f"⚠️ 3001端口服务响应异常: {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ 无法连接到3001端口服务: {e}")
        print("💡 请确保 incoming_receiver.py 正在运行")
        return False

def main():
    """主函数"""
    print("📨 增强版消息发送工具 (支持图片)")
    print("=" * 50)
    print("目标地址: http://localhost:3001/api/send")
    print("=" * 50)
    
    # 检查服务状态
    if not check_3001_service():
        print("\n❌ 服务未运行，请先启动 incoming_receiver.py")
        return
    
    while True:
        print("\n请选择操作:")
        print("1. 发送测试图片消息")
        print("2. 发送自定义消息 (支持图片)")
        print("3. 发送纯文本消息")
        print("4. 检查服务状态")
        print("5. 退出")
        
        choice = input("\n请输入选择 (1-5): ").strip()
        
        if choice == "1":
            send_test_images()
        elif choice == "2":
            send_custom_message_with_images()
        elif choice == "3":
            # 兼容原有功能
            from test_send_message import send_custom_message
            try:
                send_custom_message()
            except:
                print("❌ 无法调用原有发送功能")
        elif choice == "4":
            check_3001_service()
        elif choice == "5":
            print("👋 再见！")
            break
        else:
            print("❌ 无效选择，请重试")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n👋 程序已退出")
