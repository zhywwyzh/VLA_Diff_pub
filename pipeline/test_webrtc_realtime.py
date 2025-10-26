#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试新的 WebRTC Realtime 架构
"""

import requests
import json
import os
import sys

# 设置代理环境变量（如果需要）
os.environ['HTTPS_PROXY'] = 'http://127.0.0.1:7890'
os.environ['HTTP_PROXY'] = 'http://127.0.0.1:7890'

def test_session_creation():
    """测试 Session 创建"""
    print("🧪 测试 OpenAI Realtime Session 创建...")
    
    try:
        response = requests.post(
            "http://localhost:1024/api/realtime/session",
            json={
                "voice": "alloy",
                "instructions": "你是一个测试助手，请用中文回复。"
            },
            timeout=30
        )
        
        print(f"HTTP 状态码: {response.status_code}")
        
        if response.status_code == 200:
            result = response.json()
            if result.get('success'):
                print("✅ Session 创建成功!")
                session_data = result.get('session', {})
                print(f"Session ID: {session_data.get('id', 'N/A')}")
                print(f"Model: {session_data.get('model', 'N/A')}")
                print(f"Voice: {session_data.get('voice', 'N/A')}")
                
                # 检查 ephemeral key
                client_secret = session_data.get('client_secret', {})
                if client_secret.get('value'):
                    print(f"✅ Ephemeral Key: {client_secret['value'][:20]}...")
                    return True
                else:
                    print("❌ 未获取到 ephemeral key")
                    return False
            else:
                print(f"❌ Session 创建失败: {result.get('error')}")
                return False
        else:
            print(f"❌ HTTP 请求失败: {response.text}")
            return False
            
    except requests.exceptions.Timeout:
        print("❌ 请求超时，可能是网络问题")
        return False
    except requests.exceptions.ProxyError:
        print("❌ 代理连接失败")
        return False
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        return False

def test_realtime_connection():
    """测试实时模型连接"""
    print("\n🧪 测试实时模型连接...")
    
    try:
        response = requests.post(
            "http://localhost:1024/api/realtime/connect",
            json={"model": "gpt-4o-realtime-preview"},
            timeout=30
        )
        
        print(f"HTTP 状态码: {response.status_code}")
        
        if response.status_code == 200:
            result = response.json()
            if result.get('success'):
                print("✅ 实时模型连接成功!")
                print(f"消息: {result.get('message')}")
                return True
            else:
                print(f"❌ 连接失败: {result.get('error')}")
                return False
        else:
            print(f"❌ HTTP 请求失败: {response.text}")
            return False
            
    except Exception as e:
        print(f"❌ 连接测试失败: {e}")
        return False

def main():
    print("🚀 开始测试新的 WebRTC Realtime 架构")
    print("=" * 50)
    
    # 检查服务器是否运行
    try:
        response = requests.get("http://localhost:1024/api/test", timeout=5)
        if response.status_code != 200:
            print("❌ 主服务器未运行，请先启动: python3 app.py")
            sys.exit(1)
    except:
        print("❌ 无法连接到主服务器，请先启动: python3 app.py")
        sys.exit(1)
    
    print("✅ 主服务器运行正常")
    
    # 测试 Session 创建
    session_ok = test_session_creation()
    
    # 测试实时连接
    if session_ok:
        connection_ok = test_realtime_connection()
    else:
        print("⚠️ 跳过连接测试（Session 创建失败）")
        connection_ok = False
    
    print("\n" + "=" * 50)
    print("🎯 测试总结:")
    print(f"  Session 创建: {'✅ 成功' if session_ok else '❌ 失败'}")
    print(f"  实时连接: {'✅ 成功' if connection_ok else '❌ 失败'}")
    
    if session_ok and connection_ok:
        print("\n🎉 所有测试通过！新架构工作正常")
        print("💡 现在可以在前端选择 GPT-4o-Realtime 模型进行对话")
    else:
        print("\n⚠️ 部分测试失败，请检查:")
        print("  1. 代理设置是否正确")
        print("  2. OpenAI API Key 是否有效")
        print("  3. 网络连接是否正常")

if __name__ == "__main__":
    main()
