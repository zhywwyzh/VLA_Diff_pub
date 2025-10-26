#!/bin/bash

# 启动所有文字监听器的脚本

echo "🚀 启动文字监听器合集"
echo "========================================"

# 检查可用端口
check_port() {
    if lsof -Pi :$1 -sTCP:LISTEN -t >/dev/null ; then
        echo "❌ 端口 $1 已被占用"
        return 1
    else
        echo "✅ 端口 $1 可用"
        return 0
    fi
}

echo "\n📍 检查端口状态..."
check_port 6000  # Python simple_listener
check_port 7000  # Node.js
check_port 8000  # Go
check_port 9000  # Java

echo "\n🎯 启动选项:"
echo "1. Python 简单监听器 (端口 6000)"
echo "2. Node.js 监听器 (端口 7000)"
echo "3. Go 监听器 (端口 8000)"
echo "4. Java 监听器 (端口 9000)"
echo "5. 启动所有监听器"

read -p "请选择 (1-5): " choice

case $choice in
    1)
        echo "🐍 启动 Python 简单监听器..."
        source ../venv/bin/activate && python3 simple_listener.py
        ;;
    2)
        echo "🟨 启动 Node.js 监听器..."
        if [ ! -d "node_modules" ]; then
            echo "📦 安装 Node.js 依赖..."
            npm install
        fi
        node nodejs_listener.js
        ;;
    3)
        echo "🔵 启动 Go 监听器..."
        go run go_listener.go
        ;;
    4)
        echo "☕ 启动 Java 监听器..."
        echo "⚠️  需要先下载 Gson 库:"
        echo "   wget https://repo1.maven.org/maven2/com/google/code/gson/gson/2.8.9/gson-2.8.9.jar"
        echo "   javac -cp \".:gson-2.8.9.jar\" JavaTextListener.java"
        echo "   java -cp \".:gson-2.8.9.jar\" JavaTextListener"
        ;;
    5)
        echo "🚀 启动所有监听器..."
        echo "📝 Python 监听器 (后台运行)..."
        source ../venv/bin/activate && python3 simple_listener.py &
        
        if [ -d "node_modules" ]; then
            echo "📝 Node.js 监听器 (后台运行)..."
            node nodejs_listener.js &
        fi
        
        echo "📝 Go 监听器 (后台运行)..."
        go run go_listener.go &
        
        echo "✅ 监听器已启动，按 Ctrl+C 停止所有服务"
        wait
        ;;
    *)
        echo "❌ 无效选择"
        ;;
esac
