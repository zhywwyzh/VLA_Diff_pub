#!/bin/bash

# Gemini 多模态聊天系统启动脚本

echo "========================================"
echo "Gemini 2.5 Flash-Lite 多模态聊天系统"
echo "========================================"

# 检查Python版本
python_version=$(python3 --version 2>&1)
echo "Python版本: $python_version"

# 检查pip是否存在
if ! command -v pip &> /dev/null; then
    echo "错误: pip未安装，请先安装pip"
    exit 1
fi

# 检查是否存在requirements.txt
if [ ! -f "requirements.txt" ]; then
    echo "错误: requirements.txt文件不存在"
    exit 1
fi

# 创建uploads目录
echo "创建上传目录..."
mkdir -p uploads

# 安装依赖（可选）
read -p "是否要安装/更新Python依赖？(y/N): " install_deps
if [[ $install_deps =~ ^[Yy]$ ]]; then
    echo "安装Python依赖..."
    pip install -r requirements.txt
    
    if [ $? -ne 0 ]; then
        echo "错误: 依赖安装失败"
        exit 1
    fi
    echo "依赖安装完成!"
fi

# 检查端口是否被占用
PORT=8080
if lsof -Pi :$PORT -sTCP:LISTEN -t >/dev/null; then
    echo "警告: 端口 $PORT 已被占用"
    read -p "是否继续？服务器可能无法启动 (y/N): " continue_anyway
    if [[ ! $continue_anyway =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo ""
echo "启动配置:"
echo "- 服务端口: $PORT"
echo "- 模型: gemini-2.5-flash-lite"
echo "- 界面地址: http://localhost:$PORT"
echo ""

# 启动服务器
echo "启动服务器..."
python3 app.py

echo ""
echo "服务器已停止运行"
