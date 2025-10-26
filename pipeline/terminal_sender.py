#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
终端发送程序 - 向系统发送图片和文字
支持发送到3001端口(消息队列)或1024端口(直接对话)

使用方法:
1. 发送文字: python terminal_sender.py -t "你好"
2. 发送图片: python terminal_sender.py -i "image.jpg" -t "分析这张图片"
3. 发送到消息队列: python terminal_sender.py -t "消息" --queue
"""

import requests
import base64
import json
import argparse
import os
import sys
from PIL import Image
import io

class TerminalSender:
    def __init__(self):
        self.main_url = "http://localhost:1024/api/chat"  # 主程序对话接口
        self.queue_url = "http://localhost:3001/api/send"  # 消息队列接口
    
    def encode_image(self, image_path):
        """编码图片为base64"""
        try:
            # 检查文件是否存在
            if not os.path.exists(image_path):
                print(f"❌ 图片文件不存在: {image_path}")
                return None
            
            # 读取图片
            with open(image_path, 'rb') as f:
                image_data = f.read()
            
            # 编码为base64
            base64_data = base64.b64encode(image_data).decode('utf-8')
            
            # 检测图片格式
            image_format = "jpeg"
            if image_path.lower().endswith(('.png', '.PNG')):
                image_format = "png"
            elif image_path.lower().endswith(('.gif', '.GIF')):
                image_format = "gif"
            
            return f"data:image/{image_format};base64,{base64_data}"
            
        except Exception as e:
            print(f"❌ 图片编码失败: {e}")
            return None
    
    def send_to_main_chat(self, text, image_path=None, images_paths=None):
        """发送到主程序对话接口(1024端口)"""
        print(f"📤 发送到主程序对话接口...")
        
        data = {"message": text}
        
        # 处理单张图片
        if image_path:
            image_data = self.encode_image(image_path)
            if image_data:
                data["image"] = image_data
                print(f"🖼️  包含图片: {image_path}")
        
        # 处理多张图片
        elif images_paths:
            images_data = []
            for img_path in images_paths:
                image_data = self.encode_image(img_path)
                if image_data:
                    images_data.append(image_data)
            
            if images_data:
                data["images"] = images_data
                print(f"🖼️  包含{len(images_data)}张图片")
        
        try:
            response = requests.post(self.main_url, json=data, timeout=30)
            
            if response.status_code == 200:
                result = response.json()
                if result.get('success'):
                    print(f"✅ 发送成功!")
                    print(f"🤖 Gemini回复: {result.get('response', {}).get('text', '')}")
                    
                    # 检查是否生成了语音
                    audio = result.get('response', {}).get('audio')
                    if audio and len(audio) > 100:
                        print(f"🎵 已生成ChatGLM语音 (长度: {len(audio)})")
                    
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
    
    def send_to_queue(self, text, sender="终端程序"):
        """发送到消息队列(3001端口)"""
        print(f"📤 发送到消息队列...")
        
        data = {
            "text": text,
            "sender": sender,
            "type": "text",
            "priority": "normal"
        }
        
        try:
            response = requests.post(self.queue_url, json=data, timeout=5)
            
            if response.status_code == 200:
                result = response.json()
                if result.get('success'):
                    print(f"✅ 消息已加入队列: {result.get('message_id', '')}")
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

def main():
    parser = argparse.ArgumentParser(description='终端发送程序 - 向系统发送图片和文字')
    parser.add_argument('-t', '--text', required=True, help='要发送的文字内容')
    parser.add_argument('-i', '--image', help='单张图片路径')
    parser.add_argument('-m', '--images', nargs='+', help='多张图片路径')
    parser.add_argument('-q', '--queue', action='store_true', help='发送到消息队列(3001端口)，默认发送到主程序(1024端口)')
    parser.add_argument('-s', '--sender', default='终端程序', help='发送者名称(仅队列模式)')
    
    args = parser.parse_args()
    
    # 创建发送器
    sender = TerminalSender()
    
    print("🚀 终端发送程序")
    print("=" * 40)
    print(f"📝 文字: {args.text}")
    
    if args.queue:
        # 发送到消息队列
        if args.image or args.images:
            print("⚠️  消息队列模式不支持图片，将只发送文字")
        
        success = sender.send_to_queue(args.text, args.sender)
        
    else:
        # 发送到主程序对话
        if args.image and args.images:
            print("❌ 不能同时指定单张图片和多张图片")
            sys.exit(1)
        
        success = sender.send_to_main_chat(
            text=args.text,
            image_path=args.image,
            images_paths=args.images
        )
    
    if success:
        print("🎉 任务完成!")
    else:
        print("😞 发送失败")
        sys.exit(1)

if __name__ == "__main__":
    main()
