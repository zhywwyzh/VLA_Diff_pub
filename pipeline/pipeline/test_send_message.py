#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试向3001端口发送消息的脚本
用于演示其他程序如何向系统发送消息
"""

import requests
import json
import time
import sys
import random

def send_message(text, sender="测试程序", message_type="text", priority="normal"):
    """发送消息到3001端口"""
    url = "http://localhost:3001/api/send"
    
    data = {
        "text": text,
        "sender": sender,
        "type": message_type,
        "priority": priority
    }
    
    try:
        response = requests.post(url, json=data, timeout=5)
        
        if response.status_code == 200:
            result = response.json()
            if result['success']:
                print(f"✅ 消息发送成功: {result['message_id']}")
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

def test_various_messages():
    """测试发送各种类型的消息"""
    test_messages = [
        {
            "text": "你好，这是一条来自外部程序的测试消息",
            "sender": "外部测试程序",
            "type": "text",
            "priority": "normal"
        },
        {
            "text": "请帮我分析一下当前系统状态",
            "sender": "监控系统",
            "type": "query", 
            "priority": "normal"
        },
        {
            "text": "检测到异常情况，需要立即处理",
            "sender": "告警系统",
            "type": "notification",
            "priority": "high"
        },
        {
            "text": "执行紧急维护任务",
            "sender": "运维系统",
            "type": "command",
            "priority": "urgent"
        },
        {
            "text": "今天天气不错，适合进行户外活动。这是一条比较长的消息，用来测试界面的显示效果。",
            "sender": "天气系统",
            "type": "text",
            "priority": "normal"
        }
    ]
    
    print("🚀 开始发送测试消息...")
    
    for i, msg in enumerate(test_messages, 1):
        print(f"\n📤 发送消息 {i}/{len(test_messages)}")
        print(f"   发送者: {msg['sender']}")
        print(f"   类型: {msg['type']}")
        print(f"   优先级: {msg['priority']}")
        print(f"   内容: {msg['text'][:50]}...")
        
        success = send_message(**msg)
        if success:
            print(f"   状态: ✅ 成功")
        else:
            print(f"   状态: ❌ 失败")
        
        # 间隔1秒
        time.sleep(1)
    
    print(f"\n🎯 测试完成！已发送 {len(test_messages)} 条消息")

def send_custom_message():
    """发送自定义消息"""
    print("📝 发送自定义消息")
    print("=" * 40)
    
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
    
    print(f"\n📤 发送消息...")
    print(f"   内容: {text}")
    print(f"   发送者: {sender}")
    print(f"   类型: {message_type}")
    print(f"   优先级: {priority}")
    
    success = send_message(text, sender, message_type, priority)
    if success:
        print("✅ 消息发送成功！")
    else:
        print("❌ 消息发送失败！")

def continuous_sender():
    """连续发送消息模式"""
    print("🔄 连续发送模式 (按 Ctrl+C 停止)")
    
    messages = [
        "系统运行正常",
        "CPU使用率: 45%",
        "内存使用率: 60%", 
        "磁盘空间充足",
        "网络连接稳定"
    ]
    
    try:
        counter = 1
        while True:
            msg = random.choice(messages)
            text = f"[{counter}] {msg} - {time.strftime('%H:%M:%S')}"
            
            success = send_message(
                text=text,
                sender="监控系统",
                message_type="notification",
                priority="normal"
            )
            
            if success:
                print(f"✅ 第{counter}条消息发送成功")
            else:
                print(f"❌ 第{counter}条消息发送失败")
            
            counter += 1
            time.sleep(5)  # 每5秒发送一次
            
    except KeyboardInterrupt:
        print(f"\n🛑 停止发送，共发送了 {counter-1} 条消息")

def main():
    """主函数"""
    print("📨 消息发送测试工具")
    print("=" * 40)
    print("目标地址: http://localhost:3001/api/send")
    print("=" * 40)
    
    while True:
        print("\n请选择操作:")
        print("1. 发送测试消息集")
        print("2. 发送自定义消息")
        print("3. 连续发送模式")
        print("4. 退出")
        
        choice = input("\n请输入选择 (1-4): ").strip()
        
        if choice == "1":
            test_various_messages()
        elif choice == "2":
            send_custom_message()
        elif choice == "3":
            continuous_sender()
        elif choice == "4":
            print("👋 再见！")
            break
        else:
            print("❌ 无效选择，请重试")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n👋 程序已退出")
