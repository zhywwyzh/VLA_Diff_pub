#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
实时语音API连接修复工具
自动修复常见的连接问题
"""

import os
import sys
import time
import requests
import subprocess
import json
from datetime import datetime

# 颜色定义
class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    PURPLE = '\033[0;35m'
    CYAN = '\033[0;36m'
    NC = '\033[0m'

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

def run_command(command, description):
    """执行命令并返回结果"""
    log_info(f"执行: {description}")
    try:
        result = subprocess.run(command, shell=True, capture_output=True, text=True, timeout=30)
        if result.returncode == 0:
            log_success(f"✓ {description} 成功")
            return True, result.stdout
        else:
            log_error(f"✗ {description} 失败: {result.stderr}")
            return False, result.stderr
    except subprocess.TimeoutExpired:
        log_error(f"✗ {description} 超时")
        return False, "超时"
    except Exception as e:
        log_error(f"✗ {description} 异常: {e}")
        return False, str(e)

def check_and_kill_processes():
    """检查并清理可能冲突的进程"""
    log_header("🔧 清理冲突进程")
    
    ports_to_check = [1024, 3000, 3001]
    
    for port in ports_to_check:
        success, output = run_command(f"lsof -ti:{port}", f"检查端口{port}占用")
        if success and output.strip():
            pids = output.strip().split('\n')
            for pid in pids:
                if pid:
                    run_command(f"kill -9 {pid}", f"杀死进程{pid}")
        time.sleep(1)
    
    print()

def setup_environment():
    """设置环境变量"""
    log_header("🌍 设置环境变量")
    
    # 检查是否需要设置代理
    proxy_vars = ['HTTP_PROXY', 'HTTPS_PROXY', 'http_proxy', 'https_proxy']
    has_proxy = any(os.environ.get(var) for var in proxy_vars)
    
    if not has_proxy:
        log_warning("未检测到代理设置")
        print("如果需要代理访问OpenAI，请运行以下命令：")
        print(f"{Colors.CYAN}export HTTP_PROXY=http://127.0.0.1:7890{Colors.NC}")
        print(f"{Colors.CYAN}export HTTPS_PROXY=http://127.0.0.1:7890{Colors.NC}")
        print()
        
        response = input("是否现在设置代理？(y/N): ").strip().lower()
        if response == 'y':
            proxy_url = input("请输入代理URL (例如: http://127.0.0.1:7890): ").strip()
            if proxy_url:
                os.environ['HTTP_PROXY'] = proxy_url
                os.environ['HTTPS_PROXY'] = proxy_url
                log_success(f"代理已设置: {proxy_url}")
            else:
                log_warning("未设置代理")
    else:
        log_success("代理设置已存在")
    
    print()

def install_dependencies():
    """安装必要依赖"""
    log_header("📦 检查并安装依赖")
    
    # 检查requirements.txt
    if os.path.exists('requirements.txt'):
        success, output = run_command("pip3 install -r requirements.txt", "安装Python依赖")
        if not success:
            log_error("依赖安装失败，尝试强制重新安装")
            run_command("pip3 install -r requirements.txt --force-reinstall", "强制重新安装依赖")
    else:
        log_warning("未找到requirements.txt")
        # 安装关键依赖
        essential_packages = [
            "flask", "flask-cors", "google-generativeai", 
            "openai", "websockets", "requests", "pillow"
        ]
        for package in essential_packages:
            run_command(f"pip3 install {package}", f"安装{package}")
    
    print()

def restart_main_service():
    """重启主服务"""
    log_header("🚀 重启主服务")
    
    # 停止现有服务
    if os.path.exists('stop_all.sh'):
        run_command("chmod +x stop_all.sh && ./stop_all.sh", "停止所有服务")
    else:
        run_command("pkill -f app.py", "停止app.py进程")
    
    time.sleep(3)
    
    # 创建必要目录
    os.makedirs('logs', exist_ok=True)
    os.makedirs('uploads', exist_ok=True)
    
    # 启动主服务
    log_info("启动主服务...")
    
    # 检查是否在虚拟环境中
    venv_path = "../venv/bin/activate"
    if os.path.exists(venv_path):
        command = f"source {venv_path} && nohup python3 app.py > logs/main_service.log 2>&1 &"
    else:
        command = "nohup python3 app.py > logs/main_service.log 2>&1 &"
    
    success, output = run_command(command, "启动主服务")
    
    if success:
        log_info("等待服务启动...")
        time.sleep(5)
        
        # 测试服务是否正常
        try:
            response = requests.get("http://localhost:1024/api/test", timeout=10)
            if response.status_code == 200:
                log_success("✓ 主服务启动成功")
                return True
            else:
                log_error(f"✗ 主服务响应异常: {response.status_code}")
        except Exception as e:
            log_error(f"✗ 主服务连接失败: {e}")
    
    return False

def test_realtime_connections():
    """测试实时连接"""
    log_header("🎤 测试实时模型连接")
    
    models_to_test = ['glm-realtime', 'gpt-4o-realtime-preview']
    
    for model in models_to_test:
        log_info(f"测试{model}连接...")
        try:
            # 尝试连接
            response = requests.post(
                "http://localhost:1024/api/realtime/connect",
                json={"model": model},
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                if result.get('success'):
                    log_success(f"✓ {model} 连接成功")
                    
                    # 测试发送消息
                    time.sleep(2)
                    test_response = requests.post(
                        "http://localhost:1024/api/realtime/send",
                        json={"model": model, "text": "你好"},
                        timeout=15
                    )
                    
                    if test_response.status_code == 200:
                        log_success(f"✓ {model} 消息发送成功")
                    else:
                        log_warning(f"⚠ {model} 消息发送失败")
                        
                else:
                    log_error(f"✗ {model} 连接失败: {result.get('error', '未知错误')}")
            else:
                log_error(f"✗ {model} 连接请求失败: {response.status_code}")
                
        except Exception as e:
            log_error(f"✗ {model} 测试异常: {e}")
        
        print()

def create_startup_script():
    """创建启动脚本"""
    log_header("📝 创建快速启动脚本")
    
    startup_script = """#!/bin/bash

# 快速启动实时语音服务
echo "🚀 启动实时语音服务..."

# 激活虚拟环境（如果存在）
if [ -f "../venv/bin/activate" ]; then
    source ../venv/bin/activate
    echo "✓ 虚拟环境已激活"
fi

# 创建必要目录
mkdir -p logs uploads

# 停止可能存在的服务
pkill -f app.py 2>/dev/null || true

# 启动主服务
echo "启动主服务..."
python3 app.py &

echo "等待服务就绪..."
sleep 5

# 测试服务
curl -s http://localhost:1024/api/test > /dev/null
if [ $? -eq 0 ]; then
    echo "✓ 服务启动成功"
    echo "🌐 访问地址: http://localhost:1024"
    echo "📂 打开文件: $(pwd)/index.html"
else
    echo "✗ 服务启动失败，请查看日志: tail -f logs/main_service.log"
fi
"""
    
    with open('quick_start.sh', 'w') as f:
        f.write(startup_script)
    
    run_command("chmod +x quick_start.sh", "设置启动脚本权限")
    log_success("✓ 快速启动脚本已创建: ./quick_start.sh")
    print()

def show_status_summary():
    """显示状态总结"""
    log_header("📊 系统状态总结")
    
    try:
        # 检查主服务
        response = requests.get("http://localhost:1024/api/test", timeout=5)
        if response.status_code == 200:
            log_success("✓ 主服务: 运行正常")
            result = response.json()
            log_info(f"  模型: {result.get('model', 'N/A')}")
            log_info(f"  API状态: {result.get('api_status', 'N/A')}")
        else:
            log_error("✗ 主服务: 异常")
    except:
        log_error("✗ 主服务: 未运行")
    
    # 检查实时模型状态
    try:
        response = requests.get("http://localhost:1024/api/realtime/status", timeout=5)
        if response.status_code == 200:
            result = response.json()
            models = result.get('realtime_models', {})
            for model, status in models.items():
                connected = status.get('connected', False)
                status_str = "🟢 已连接" if connected else "🔴 未连接"
                log_info(f"  {model}: {status_str}")
        else:
            log_warning("⚠ 实时模型状态查询失败")
    except:
        log_warning("⚠ 无法查询实时模型状态")
    
    print()

def main():
    """主修复流程"""
    print()
    log_header("=" * 80)
    log_header("    🔧 实时语音API连接修复工具")
    log_header("=" * 80)
    print()
    
    log_info(f"修复时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    
    try:
        # 1. 清理冲突进程
        check_and_kill_processes()
        
        # 2. 设置环境
        setup_environment()
        
        # 3. 安装依赖
        install_dependencies()
        
        # 4. 重启服务
        if restart_main_service():
            # 5. 测试连接
            test_realtime_connections()
            
            # 6. 创建启动脚本
            create_startup_script()
            
            # 7. 显示状态
            show_status_summary()
            
            log_header("🎉 修复完成！")
            print()
            log_success("✓ 系统已准备就绪")
            print(f"{Colors.CYAN}下一步:{Colors.NC}")
            print("1. 打开浏览器访问: http://localhost:1024")
            print("2. 或直接打开文件: index.html")
            print("3. 选择实时模型进行测试")
            print()
            print(f"{Colors.CYAN}管理命令:{Colors.NC}")
            print("• 查看日志: tail -f logs/main_service.log")
            print("• 快速启动: ./quick_start.sh")
            print("• 诊断工具: python3 diagnose_realtime_connection.py")
            
        else:
            log_error("✗ 服务启动失败")
            print()
            print(f"{Colors.YELLOW}请尝试手动操作:{Colors.NC}")
            print("1. 查看日志: tail -f logs/main_service.log")
            print("2. 手动启动: python3 app.py")
            print("3. 运行诊断: python3 diagnose_realtime_connection.py")
            
    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}修复已取消{Colors.NC}")
    except Exception as e:
        log_error(f"修复过程中出错: {e}")
        print()
        print(f"{Colors.YELLOW}请运行诊断工具获取详细信息:{Colors.NC}")
        print("python3 diagnose_realtime_connection.py")

if __name__ == "__main__":
    main()
