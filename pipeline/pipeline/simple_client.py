#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
简单的Gemini消息客户端 - 基础使用示例

这是一个最简单的示例，展示如何获取和处理从Gemini聊天系统转发的消息。
适合初学者或需要基础功能的应用。
"""

import requests
import time
import json

def get_messages():
    """获取所有转发的消息"""
    try:
        response = requests.get('http://localhost:3000/api/messages')
        if response.status_code == 200:
            data = response.json()
            return data.get('messages', [])
    except Exception as e:
        print(f"获取消息失败: {e}")
    return []

def get_latest_message():
    """获取最新的一条消息"""
    messages = get_messages()
    return messages[-1] if messages else None

def wait_for_new_message(last_id=0, timeout=30):
    """等待新消息（轮询方式）"""
    start_time = time.time()
    while time.time() - start_time < timeout:
        messages = get_messages()
        for msg in messages:
            if msg['id'] > last_id:
                return msg
        time.sleep(1)
    return None

def process_message(message):
    """处理单条消息 - 在这里添加您的处理逻辑"""
    print(f"\n收到AI回复:")
    print(f"内容: {message['text']}")
    print(f"时间: {message['timestamp']}")
    print(f"ID: {message['id']}")
    
    # 示例处理逻辑
    text = message['text']
    
    # 1. 简单的关键词检测
    if '错误' in text or '问题' in text:
        print("⚠️  检测到问题相关内容")
    
    if '成功' in text or '完成' in text:
        print("✅ 检测到成功相关内容")
    
    # 2. 长度分析
    if len(text) > 100:
        print(f"📄 这是一条长消息 ({len(text)} 字符)")
    
    # 3. 保存到文件 (可选)
    # with open('ai_responses.txt', 'a', encoding='utf-8') as f:
    #     f.write(f"[{message['timestamp']}] {text}\n")
    
    print("-" * 50)

def main():
    """主程序"""
    print("🤖 简单Gemini消息客户端")
    print("=" * 40)
    
    # 检查连接
    try:
        response = requests.get('http://localhost:3000/api/health', timeout=2)
        if response.status_code != 200:
            print("❌ 无法连接到外部接收器")
            print("请确保 external_receiver_example.py 正在运行")
            return
    except:
        print("❌ 无法连接到外部接收器")
        print("请确保 external_receiver_example.py 正在运行")
        return
    
    print("✅ 成功连接到外部接收器")
    
    # 获取现有消息
    existing_messages = get_messages()
    print(f"📋 发现 {len(existing_messages)} 条历史消息")
    
    # 显示最新的几条消息
    if existing_messages:
        print("\n最近的消息:")
        for msg in existing_messages[-3:]:  # 显示最新3条
            print(f"#{msg['id']}: {msg['text'][:50]}{'...' if len(msg['text']) > 50 else ''}")
    
    # 开始监听新消息
    last_id = existing_messages[-1]['id'] if existing_messages else 0
    print(f"\n🔄 开始监听新消息 (从ID {last_id + 1} 开始)...")
    print("💡 在聊天界面与AI对话，这里会显示AI的回复")
    print("⌨️  按 Ctrl+C 退出\n")
    
    try:
        while True:
            # 等待新消息
            new_message = wait_for_new_message(last_id, timeout=5)
            if new_message:
                process_message(new_message)
                last_id = new_message['id']
            
    except KeyboardInterrupt:
        print("\n\n👋 用户中断，程序退出")

if __name__ == '__main__':
    main()
