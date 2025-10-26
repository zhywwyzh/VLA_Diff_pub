#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
实时语音API连接诊断工具
专门用于诊断GPT-4o-Realtime和GLM-Realtime连接问题
"""

import os
import sys
import json
import time
import requests
import asyncio
import websockets
from datetime import datetime
import subprocess
import socket
import urllib3
from urllib.parse import urlparse
from key import GEMINI_API_KEY, CHATGLM_API_KEY, OPENAI_API_KEY

# 禁用SSL警告
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

# 颜色定义
class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    PURPLE = '\033[0;35m'
    CYAN = '\033[0;36m'
    NC = '\033[0m'  # No Color

def log_info(message):
    print(f"{Colors.BLUE}[INFO]{Colors.NC} {message}")

def log_success(message):
    print(f"{Colors.GREEN}[SUCCESS]{Colors.NC} {message}")

def log_warning(message):
    print(f"{Colors.YELLOW}[WARNING]{Colors.NC} {message}")

def log_error(message):
    print(f"{Colors.RED}[ERROR]{Colors.NC} {message}")

def log_header(message):
    print(f"{Colors.PURPLE}{message}{Colors.NC}")

def print_separator():
    print("=" * 80)

# API配置

def check_network_connectivity():
    """检查网络连接性"""
    log_header("🌐 检查网络连接性")
    
    # 测试域名解析
    test_domains = [
        "www.google.com",
        "api.openai.com", 
        "open.bigmodel.cn",
        "generativelanguage.googleapis.com"
    ]
    
    for domain in test_domains:
        try:
            socket.gethostbyname(domain)
            log_success(f"✓ 域名解析成功: {domain}")
        except socket.gaierror as e:
            log_error(f"✗ 域名解析失败: {domain} - {e}")
    
    print()

def check_proxy_settings():
    """检查代理设置"""
    log_header("🔧 检查代理设置")
    
    proxy_vars = ['HTTP_PROXY', 'HTTPS_PROXY', 'http_proxy', 'https_proxy', 'ALL_PROXY', 'all_proxy']
    
    found_proxy = False
    for var in proxy_vars:
        value = os.environ.get(var)
        if value:
            log_info(f"{var}: {value}")
            found_proxy = True
    
    if not found_proxy:
        log_warning("未检测到代理设置")
    
    print()

def test_http_connectivity():
    """测试HTTP连接"""
    log_header("🌍 测试HTTP连接")
    
    test_urls = [
        ("Google", "https://www.google.com"),
        ("OpenAI API", "https://api.openai.com/v1/models"),
        ("智谱API", "https://open.bigmodel.cn"),
        ("Gemini API", "https://generativelanguage.googleapis.com")
    ]
    
    session = requests.Session()
    session.verify = False  # 忽略SSL验证
    
    for name, url in test_urls:
        try:
            log_info(f"测试连接: {name}")
            response = session.get(url, timeout=10)
            log_success(f"✓ {name} 连接成功 (状态码: {response.status_code})")
        except requests.exceptions.Timeout:
            log_error(f"✗ {name} 连接超时")
        except requests.exceptions.ConnectionError as e:
            log_error(f"✗ {name} 连接错误: {e}")
        except Exception as e:
            log_error(f"✗ {name} 未知错误: {e}")
    
    print()

def test_openai_api_access():
    """测试OpenAI API访问"""
    log_header("🤖 测试OpenAI API访问")
    
    headers = {
        "Authorization": f"Bearer {OPENAI_API_KEY}",
        "Content-Type": "application/json"
    }
    
    try:
        # 测试基础API
        log_info("测试OpenAI基础API...")
        response = requests.get(
            "https://api.openai.com/v1/models",
            headers=headers,
            timeout=15,
            verify=False
        )
        
        if response.status_code == 200:
            log_success("✓ OpenAI基础API访问成功")
            models = response.json()
            # 检查是否有实时模型权限
            realtime_models = [m for m in models['data'] if 'realtime' in m['id']]
            if realtime_models:
                log_success(f"✓ 检测到实时模型权限: {len(realtime_models)}个模型")
            else:
                log_warning("⚠ 未检测到实时模型权限")
        else:
            log_error(f"✗ OpenAI API访问失败: {response.status_code} {response.text}")
            
    except Exception as e:
        log_error(f"✗ OpenAI API测试失败: {e}")
    
    # 测试实时Session API
    try:
        log_info("测试OpenAI Realtime Session API...")
        session_data = {
            "model": "gpt-4o-realtime-preview",
            "voice": "alloy"
        }
        
        response = requests.post(
            "https://api.openai.com/v1/realtime/sessions",
            headers={**headers, "OpenAI-Beta": "realtime=v1"},
            json=session_data,
            timeout=15,
            verify=False
        )
        
        if response.status_code == 200:
            log_success("✓ OpenAI Realtime Session API访问成功")
            session_info = response.json()
            log_info(f"Session ID: {session_info.get('id', 'N/A')}")
        else:
            log_error(f"✗ Realtime Session API失败: {response.status_code} {response.text}")
            
    except Exception as e:
        log_error(f"✗ Realtime Session API测试失败: {e}")
    
    print()

def test_chatglm_api_access():
    """测试ChatGLM API访问"""
    log_header("🧠 测试ChatGLM API访问")
    
    try:
        # 测试智谱基础API
        log_info("测试智谱基础API...")
        headers = {
            "Authorization": f"Bearer {CHATGLM_API_KEY}",
            "Content-Type": "application/json"
        }
        
        # 简单的聊天测试
        test_data = {
            "model": "glm-4",
            "messages": [{"role": "user", "content": "你好"}],
            "max_tokens": 10
        }
        
        response = requests.post(
            "https://open.bigmodel.cn/api/paas/v4/chat/completions",
            headers=headers,
            json=test_data,
            timeout=15,
            verify=False
        )
        
        if response.status_code == 200:
            log_success("✓ ChatGLM基础API访问成功")
        else:
            log_error(f"✗ ChatGLM API访问失败: {response.status_code} {response.text}")
            
    except Exception as e:
        log_error(f"✗ ChatGLM API测试失败: {e}")
    
    print()

async def test_websocket_connectivity():
    """测试WebSocket连接"""
    log_header("🔌 测试WebSocket连接")
    
    # 测试OpenAI WebSocket
    try:
        log_info("测试OpenAI WebSocket连接...")
        uri = f"wss://api.openai.com/v1/realtime?model=gpt-4o-realtime-preview"
        headers = {
            "Authorization": f"Bearer {OPENAI_API_KEY}",
            "OpenAI-Beta": "realtime=v1"
        }
        
        # 短时间连接测试
        websocket = await asyncio.wait_for(
            websockets.connect(uri, additional_headers=headers, open_timeout=10),
            timeout=15
        )
        await websocket.close()
        log_success("✓ OpenAI WebSocket连接成功")
        
    except asyncio.TimeoutError:
        log_error("✗ OpenAI WebSocket连接超时")
    except websockets.exceptions.InvalidStatusCode as e:
        log_error(f"✗ OpenAI WebSocket状态码错误: {e.status_code}")
    except Exception as e:
        log_error(f"✗ OpenAI WebSocket连接失败: {e}")
    
    # 测试ChatGLM WebSocket
    try:
        log_info("测试ChatGLM WebSocket连接...")
        uri = "wss://open.bigmodel.cn/api/paas/v4/realtime"
        headers = {
            "Authorization": f"Bearer {CHATGLM_API_KEY}"
        }
        
        websocket = await asyncio.wait_for(
            websockets.connect(uri, additional_headers=headers, open_timeout=10),
            timeout=15
        )
        await websocket.close()
        log_success("✓ ChatGLM WebSocket连接成功")
        
    except asyncio.TimeoutError:
        log_error("✗ ChatGLM WebSocket连接超时")
    except websockets.exceptions.InvalidStatusCode as e:
        log_error(f"✗ ChatGLM WebSocket状态码错误: {e.status_code}")
    except Exception as e:
        log_error(f"✗ ChatGLM WebSocket连接失败: {e}")
    
    print()

def test_local_service():
    """测试本地服务"""
    log_header("🏠 测试本地服务")
    
    try:
        # 测试主服务
        log_info("测试主服务 (端口1024)...")
        response = requests.get("http://localhost:1024/api/test", timeout=5)
        if response.status_code == 200:
            log_success("✓ 主服务运行正常")
            result = response.json()
            log_info(f"API状态: {result.get('api_status', 'N/A')}")
            log_info(f"模型: {result.get('model', 'N/A')}")
        else:
            log_error(f"✗ 主服务响应错误: {response.status_code}")
            
    except requests.exceptions.ConnectionError:
        log_error("✗ 主服务未运行 (端口1024)")
    except Exception as e:
        log_error(f"✗ 主服务测试失败: {e}")
    
    # 测试实时连接状态
    try:
        log_info("测试实时模型状态...")
        response = requests.get("http://localhost:1024/api/realtime/status", timeout=5)
        if response.status_code == 200:
            result = response.json()
            realtime_models = result.get('realtime_models', {})
            for model, status in realtime_models.items():
                connected = status.get('connected', False)
                status_str = "🟢 已连接" if connected else "🔴 未连接"
                log_info(f"{model}: {status_str}")
        else:
            log_error(f"✗ 实时状态查询失败: {response.status_code}")
            
    except Exception as e:
        log_error(f"✗ 实时状态测试失败: {e}")
    
    print()

def check_system_requirements():
    """检查系统需求"""
    log_header("💻 检查系统需求")
    
    # Python版本
    python_version = sys.version
    log_info(f"Python版本: {python_version}")
    
    # 必要的包
    required_packages = [
        'requests', 'websockets', 'asyncio', 'json', 'urllib3'
    ]
    
    for package in required_packages:
        try:
            __import__(package)
            log_success(f"✓ {package} 已安装")
        except ImportError:
            log_error(f"✗ {package} 未安装")
    
    print()

def provide_solutions():
    """提供解决方案"""
    log_header("💡 解决方案建议")
    
    print(f"{Colors.CYAN}1. 网络连接问题:{Colors.NC}")
    print("   • 检查网络连接是否正常")
    print("   • 尝试使用VPN或代理")
    print("   • 检查防火墙设置")
    print()
    
    print(f"{Colors.CYAN}2. API密钥问题:{Colors.NC}")
    print("   • 确认OpenAI API密钥有效且有实时API权限")
    print("   • 确认ChatGLM API密钥有效")
    print("   • 检查账户余额和使用限制")
    print()
    
    print(f"{Colors.CYAN}3. 代理设置:{Colors.NC}")
    print("   • 如需代理，设置环境变量：")
    print("     export HTTP_PROXY=http://127.0.0.1:7890")
    print("     export HTTPS_PROXY=http://127.0.0.1:7890")
    print("   • 或在代码中配置代理")
    print()
    
    print(f"{Colors.CYAN}4. 本地服务:{Colors.NC}")
    print("   • 确保主服务已启动: python3 app.py")
    print("   • 检查端口占用: lsof -i :1024")
    print("   • 查看服务日志: tail -f logs/main_service.log")
    print()
    
    print(f"{Colors.CYAN}5. 快速修复命令:{Colors.NC}")
    print("   # 重启主服务")
    print("   ./stop_all.sh && ./start_all.sh")
    print()
    print("   # 手动测试连接")
    print("   curl http://localhost:1024/api/test")
    print("   curl -X POST http://localhost:1024/api/realtime/connect \\")
    print("        -H 'Content-Type: application/json' \\")
    print("        -d '{\"model\": \"glm-realtime\"}'")
    print()

async def main():
    """主函数"""
    print()
    log_header("=" * 80)
    log_header("    🔍 实时语音API连接诊断工具")
    log_header("=" * 80)
    print()
    
    log_info(f"诊断时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    
    # 系统检查
    check_system_requirements()
    
    # 网络检查
    check_network_connectivity()
    check_proxy_settings()
    test_http_connectivity()
    
    # API访问测试
    test_openai_api_access()
    test_chatglm_api_access()
    
    # WebSocket测试
    await test_websocket_connectivity()
    
    # 本地服务测试
    test_local_service()
    
    # 提供解决方案
    print_separator()
    provide_solutions()
    
    print_separator()
    log_success("🎯 诊断完成！请根据上述结果排查问题")
    print()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}诊断已取消{Colors.NC}")
    except Exception as e:
        log_error(f"诊断过程中出错: {e}")