#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
网络代理配置助手
帮助配置代理以访问Google API服务
"""

import os
import sys
import subprocess
import requests
import time

def print_banner():
    print("=" * 60)
    print("🌐 Gemini API 网络配置助手")
    print("=" * 60)
    print()

def test_direct_connection():
    """测试直接连接Google API"""
    print("🔍 测试直接连接Google API...")
    try:
        response = requests.get(
            'https://generativelanguage.googleapis.com',
            timeout=10
        )
        print("✅ 直接连接成功！无需配置代理")
        return True
    except Exception as e:
        print(f"❌ 直接连接失败: {e}")
        return False

def test_proxy_connection(proxy_url):
    """测试代理连接"""
    print(f"🔍 测试代理连接: {proxy_url}")
    try:
        proxies = {
            'http': proxy_url,
            'https': proxy_url
        }
        response = requests.get(
            'https://generativelanguage.googleapis.com',
            proxies=proxies,
            timeout=10
        )
        print("✅ 代理连接成功！")
        return True
    except Exception as e:
        print(f"❌ 代理连接失败: {e}")
        return False

def setup_environment_proxy(proxy_url):
    """设置环境变量代理"""
    os.environ['HTTP_PROXY'] = proxy_url
    os.environ['HTTPS_PROXY'] = proxy_url
    print(f"✅ 环境变量代理已设置: {proxy_url}")

def create_proxy_script(proxy_url):
    """创建代理启动脚本"""
    script_content = f"""#!/bin/bash
# Gemini API 代理配置脚本

export HTTP_PROXY={proxy_url}
export HTTPS_PROXY={proxy_url}

echo "代理配置: {proxy_url}"
echo "启动应用..."

# 启动应用
python3 app.py
"""
    
    with open('start_with_proxy.sh', 'w') as f:
        f.write(script_content)
    
    # 设置执行权限
    os.chmod('start_with_proxy.sh', 0o755)
    print("✅ 创建代理启动脚本: start_with_proxy.sh")

def main():
    print_banner()
    
    # 测试直接连接
    if test_direct_connection():
        print("\n🎉 您的网络可以直接访问Google API，无需配置代理！")
        return
    
    print("\n❗ 直接连接失败，需要配置代理")
    print("\n常见代理配置:")
    print("1. Clash: http://127.0.0.1:7890")
    print("2. V2Ray: http://127.0.0.1:1080")
    print("3. Shadowsocks: http://127.0.0.1:1086")
    print("4. 其他代理")
    print("5. 跳过代理配置")
    
    while True:
        choice = input("\n请选择 (1-5): ").strip()
        
        proxy_urls = {
            '1': 'http://127.0.0.1:7890',
            '2': 'http://127.0.0.1:1080', 
            '3': 'http://127.0.0.1:1086'
        }
        
        if choice in proxy_urls:
            proxy_url = proxy_urls[choice]
            if test_proxy_connection(proxy_url):
                setup_environment_proxy(proxy_url)
                create_proxy_script(proxy_url)
                print(f"\n🎉 代理配置成功！")
                print(f"使用 './start_with_proxy.sh' 启动应用")
                break
            else:
                print("❌ 代理测试失败，请检查代理设置")
                
        elif choice == '4':
            proxy_url = input("请输入代理地址 (例如: http://127.0.0.1:7890): ").strip()
            if test_proxy_connection(proxy_url):
                setup_environment_proxy(proxy_url)
                create_proxy_script(proxy_url)
                print(f"\n🎉 代理配置成功！")
                print(f"使用 './start_with_proxy.sh' 启动应用")
                break
            else:
                print("❌ 代理测试失败，请检查代理设置")
                
        elif choice == '5':
            print("\n⚠️ 跳过代理配置，您可能无法访问Google API")
            print("建议:")
            print("1. 检查防火墙设置")
            print("2. 确认网络连接")
            print("3. 联系网络管理员")
            break
            
        else:
            print("❌ 无效选择，请重新输入")

if __name__ == "__main__":
    main()
