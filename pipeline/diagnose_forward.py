#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
转发连接问题诊断和修复工具
"""

import requests
import json
import time
import sys
import subprocess

def test_service_status():
    """测试服务状态"""
    print("🔍 检查服务状态...")
    
    # 测试主服务
    try:
        response = requests.get('http://localhost:1024/api/test', timeout=5)
        if response.status_code == 200:
            result = response.json()
            print("✅ 主服务 (1024端口) 正常运行")
            print(f"   - 转发URL: {result.get('forward_url', '未设置')}")
            main_service_ok = True
        else:
            print(f"❌ 主服务响应异常: HTTP {response.status_code}")
            main_service_ok = False
    except Exception as e:
        print(f"❌ 主服务连接失败: {e}")
        main_service_ok = False
    
    # 测试外部接收器
    try:
        response = requests.get('http://localhost:3000/api/health', timeout=5)
        if response.status_code == 200:
            print("✅ 外部接收器 (3000端口) 正常运行")
            external_service_ok = True
        else:
            print(f"❌ 外部接收器响应异常: HTTP {response.status_code}")
            external_service_ok = False
    except Exception as e:
        print(f"❌ 外部接收器连接失败: {e}")
        external_service_ok = False
    
    return main_service_ok, external_service_ok

def test_forward_connection():
    """测试转发连接"""
    print("\n🔗 测试转发连接...")
    
    # 测试设置转发URL
    try:
        response = requests.post(
            'http://localhost:1024/api/config/forward',
            json={'url': 'http://localhost:3000/api/receive'},
            timeout=5
        )
        result = response.json()
        
        if result['success']:
            print("✅ 转发URL设置成功")
            print(f"   - 连接状态: {result.get('connection_status', '未知')}")
            return result.get('connection_status', '').startswith('连接成功')
        else:
            print(f"❌ 转发URL设置失败: {result.get('error', '未知错误')}")
            return False
    except Exception as e:
        print(f"❌ 转发URL设置异常: {e}")
        return False

def test_message_forward():
    """测试消息转发"""
    print("\n📨 测试消息转发...")
    
    # 先清空接收器的消息
    try:
        requests.post('http://localhost:3000/api/clear', timeout=5)
        print("🧹 已清空接收器消息")
    except:
        pass
    
    # 发送测试聊天消息
    try:
        test_message = "这是一条测试消息，用于验证转发功能"
        response = requests.post(
            'http://localhost:1024/api/chat',
            json={'text': test_message},
            timeout=30
        )
        
        if response.status_code == 200:
            result = response.json()
            if result['success']:
                print("✅ 聊天消息发送成功")
                ai_response = result['response']['text']
                print(f"   - AI回复: {ai_response[:100]}...")
                
                # 等待转发完成
                time.sleep(2)
                
                # 检查接收器是否收到消息
                check_response = requests.get('http://localhost:3000/api/messages', timeout=5)
                if check_response.status_code == 200:
                    messages = check_response.json()
                    if messages['total'] > 0:
                        print("✅ 转发功能正常工作")
                        print(f"   - 接收到 {messages['total']} 条消息")
                        return True
                    else:
                        print("❌ 外部接收器未收到转发消息")
                        return False
                else:
                    print("❌ 无法检查接收器消息")
                    return False
            else:
                print(f"❌ 聊天请求失败: {result.get('error', '未知错误')}")
                return False
        else:
            print(f"❌ 聊天请求HTTP错误: {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ 消息转发测试失败: {e}")
        return False

def fix_port_conflicts():
    """修复端口冲突"""
    print("\n🔧 检查端口冲突...")
    
    try:
        # 检查端口占用
        result = subprocess.run(['lsof', '-i', ':1024', '-i', ':3000'], 
                              capture_output=True, text=True)
        
        if result.returncode == 0:
            lines = result.stdout.strip().split('\n')
            processes = {}
            
            for line in lines[1:]:  # 跳过标题行
                parts = line.split()
                if len(parts) >= 9:
                    pid = parts[1]
                    port = parts[8].split(':')[-1]
                    if port in ['1024', '3000', 'hbci']:  # hbci是3000端口的别名
                        if pid not in processes:
                            processes[pid] = []
                        processes[pid].append(port)
            
            print(f"发现 {len(processes)} 个进程占用相关端口:")
            for pid, ports in processes.items():
                port_str = ', '.join(set(ports))
                print(f"   - PID {pid}: 端口 {port_str}")
            
            # 检查是否有重复进程
            if len(processes) > 2:
                print("⚠️ 发现多个进程，可能存在重复启动")
                return False
            else:
                print("✅ 端口占用正常")
                return True
        else:
            print("❌ 无法检查端口占用")
            return False
    except Exception as e:
        print(f"❌ 端口检查失败: {e}")
        return False

def show_browser_info():
    """显示浏览器访问信息"""
    print("\n🌐 浏览器访问信息:")
    print("   - 主界面: http://localhost:1024")
    print("   - 直接打开文件: file:///Users/zzmm4/Desktop/VLA/generative-ai/gemini/multimodal-live-api/pipeline/index.html")
    print("   - 外部接收器: http://localhost:3000")
    print("   - 转发URL应设置为: http://localhost:3000/api/receive")

def main():
    """主诊断函数"""
    print("🩺 转发连接问题诊断工具")
    print("=" * 50)
    
    # 检查服务状态
    main_ok, external_ok = test_service_status()
    
    if not main_ok:
        print("\n❌ 主服务未运行，请先启动: ./start_all.sh")
        return False
    
    if not external_ok:
        print("\n❌ 外部接收器未运行，请检查启动脚本")
        return False
    
    # 检查端口冲突
    fix_port_conflicts()
    
    # 测试转发连接
    forward_ok = test_forward_connection()
    
    if not forward_ok:
        print("\n🔧 尝试修复转发连接...")
        
        # 重新设置转发URL
        try:
            response = requests.post(
                'http://localhost:1024/api/config/forward',
                json={'url': 'http://localhost:3000/api/receive'},
                timeout=10
            )
            result = response.json()
            print(f"转发URL重置结果: {result.get('message', '未知')}")
            print(f"连接状态: {result.get('connection_status', '未知')}")
        except Exception as e:
            print(f"转发URL重置失败: {e}")
    
    # 测试完整的消息转发流程
    print("\n" + "=" * 30)
    message_ok = test_message_forward()
    
    # 显示访问信息
    show_browser_info()
    
    print("\n" + "=" * 50)
    if main_ok and external_ok and forward_ok and message_ok:
        print("🎉 所有功能正常！转发系统工作正常")
        return True
    else:
        print("⚠️ 存在问题，请检查上述错误信息")
        
        # 给出具体的解决建议
        print("\n💡 解决建议:")
        if not main_ok or not external_ok:
            print("   1. 重新启动服务: ./start_all.sh")
        if not forward_ok:
            print("   2. 手动在浏览器中设置转发URL: http://localhost:3000/api/receive")
        if not message_ok:
            print("   3. 检查防火墙设置")
            print("   4. 尝试重启浏览器")
        
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
