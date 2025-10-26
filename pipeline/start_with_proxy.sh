#!/bin/bash

# Gemini 多模态聊天系统 - 代理启动脚本
# 参考 realtime-voice-gpt4o 项目的代理配置方案

echo "🌐 配置代理环境变量..."

# 设置代理环境变量 (请根据您的代理配置修改端口)
export https_proxy=http://127.0.0.1:7890
export http_proxy=http://127.0.0.1:7890  
export HTTPS_PROXY=http://127.0.0.1:7890
export HTTP_PROXY=http://127.0.0.1:7890

echo "✅ 代理环境变量已设置:"
echo "  HTTPS_PROXY=$HTTPS_PROXY"
echo "  HTTP_PROXY=$HTTP_PROXY"
echo ""

echo "📝 代理配置说明:"
echo "  - 如果您的代理端口不是7890，请修改此脚本"
echo "  - 常见端口: 7890 (Clash), 1080 (Shadowsocks), 8080 (其他)"
echo "  - 确保代理软件正在运行"
echo ""

echo "🚀 启动带代理的 Gemini 服务..."
echo "💡 提示: OpenAI Realtime API 可能需要代理才能连接"
echo ""

# 激活虚拟环境并启动服务
source ../venv/bin/activate
python3 app.py
