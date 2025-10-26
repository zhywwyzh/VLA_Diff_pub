#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
网络连接测试工具
用于诊断Gemini API连接问题
"""

import requests
import socket
import time
import sys
from urllib.parse import urlparse

def test_basic_internet():
    """测试基本网络连接"""
    print("🌐 测试基本网络连接...")
    
    test_urls = [
        "https://www.google.com",
        "https://www.baidu.com",
        "https://httpbin.org/get"
    ]
    
    for url in test_urls:
        try:
            response = requests.get(url, timeout=10)
            if response.status_code == 200:
                print(f"✅ {url} - 连接正常")
                return True
            else:
                print(f"⚠️  {url} - HTTP {response.status_code}")
        except Exception as e:
            print(f"❌ {url} - 连接失败: {e}")
    
    return False

def test_dns_resolution():
    """测试DNS解析"""
    print("\n🔍 测试DNS解析...")
    
    domains = [
        "generativelanguage.googleapis.com",
        "google.com",
        "googleapis.com"
    ]
    
    for domain in domains:
        try:
            ip = socket.gethostbyname(domain)
            print(f"✅ {domain} -> {ip}")
        except Exception as e:
            print(f"❌ {domain} - DNS解析失败: {e}")

def test_gemini_api_endpoints():
    """测试Gemini API端点"""
    print("\n🤖 测试Gemini API端点...")
    
    # 基础连接测试
    api_base = "https://generativelanguage.googleapis.com"
    
    try:
        response = requests.get(f"{api_base}/v1beta/models", timeout=30)
        print(f"✅ Gemini API基础端点可访问 - HTTP {response.status_code}")
        return True
    except requests.exceptions.Timeout:
        print("❌ Gemini API连接超时")
        return False
    except requests.exceptions.ConnectionError as e:
        print(f"❌ Gemini API连接错误: {e}")
        return False
    except Exception as e:
        print(f"❌ Gemini API其他错误: {e}")
        return False

def test_port_connectivity():
    """测试端口连通性"""
    print("\n🔌 测试端口连通性...")
    
    hosts_ports = [
        ("generativelanguage.googleapis.com", 443),
        ("google.com", 443),
        ("8.8.8.8", 53),  # Google DNS
    ]
    
    for host, port in hosts_ports:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10)
            result = sock.connect_ex((host, port))
            sock.close()
            
            if result == 0:
                print(f"✅ {host}:{port} - 端口开放")
            else:
                print(f"❌ {host}:{port} - 端口不可达")
        except Exception as e:
            print(f"❌ {host}:{port} - 测试失败: {e}")

def check_proxy_settings():
    """检查代理设置"""
    print("\n🔧 检查代理设置...")
    
    import os
    proxy_vars = ['http_proxy', 'https_proxy', 'HTTP_PROXY', 'HTTPS_PROXY']
    
    found_proxy = False
    for var in proxy_vars:
        value = os.environ.get(var)
        if value:
            print(f"🔍 发现代理设置: {var} = {value}")
            found_proxy = True
    
    if not found_proxy:
        print("ℹ️  未发现系统代理设置")

def test_alternative_apis():
    """测试替代API"""
    print("\n🔄 测试替代服务...")
    
    # 测试OpenAI API (如果可用)
    try:
        response = requests.get("https://api.openai.com/v1/models", timeout=10)
        print(f"✅ OpenAI API可访问 - HTTP {response.status_code}")
    except Exception as e:
        print(f"❌ OpenAI API不可访问: {e}")
    
    # 测试Anthropic API
    try:
        response = requests.get("https://api.anthropic.com", timeout=10)
        print(f"✅ Anthropic API可访问 - HTTP {response.status_code}")
    except Exception as e:
        print(f"❌ Anthropic API不可访问: {e}")

def suggest_solutions():
    """提供解决方案建议"""
    print("\n💡 解决方案建议:")
    print("1. 检查网络连接是否正常")
    print("2. 如果使用公司网络，联系网络管理员检查防火墙设置")
    print("3. 尝试使用VPN连接")
    print("4. 检查是否需要配置代理")
    print("5. 尝试使用手机热点测试")
    print("6. 检查API密钥是否正确")
    print("7. 考虑使用其他AI服务作为备用方案")

def main():
    print("=" * 60)
    print("🔬 Gemini API 网络连接诊断工具")
    print("=" * 60)
    
    # 执行各项测试
    internet_ok = test_basic_internet()
    test_dns_resolution()
    gemini_ok = test_gemini_api_endpoints()
    test_port_connectivity()
    check_proxy_settings()
    test_alternative_apis()
    
    print("\n" + "=" * 60)
    print("📊 诊断结果总结:")
    print("=" * 60)
    
    if internet_ok:
        print("✅ 基础网络连接: 正常")
    else:
        print("❌ 基础网络连接: 异常")
    
    if gemini_ok:
        print("✅ Gemini API连接: 正常")
        print("\n🎉 网络连接正常！如果仍有问题，可能是API密钥或其他配置问题。")
    else:
        print("❌ Gemini API连接: 异常")
        print("\n⚠️  Gemini API无法访问，这可能是网络限制或地区限制问题。")
    
    suggest_solutions()
    
    print("\n" + "=" * 60)

if __name__ == "__main__":
    main()
