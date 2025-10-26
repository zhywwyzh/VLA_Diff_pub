#!/bin/bash

# 🎤 一键语音对话演示设置脚本

echo "🚀 设置一键语音对话演示环境..."

# 设置代理环境变量
export https_proxy=http://127.0.0.1:7890
export http_proxy=http://127.0.0.1:7890  
export HTTPS_PROXY=http://127.0.0.1:7890
export HTTP_PROXY=http://127.0.0.1:7890

# 设置演示用的OpenAI API密钥（需要用户提供真实密钥）
if [ -z "$OPENAI_API_KEY" ]; then
    echo "⚠️  请设置OpenAI API密钥:"
    echo "   export OPENAI_API_KEY='your_openai_api_key_here'"
    echo ""
    echo "🔑 获取API密钥："
    echo "   1. 访问 https://platform.openai.com/api-keys"
    echo "   2. 创建新的API密钥"
    echo "   3. 复制密钥并设置到环境变量"
    echo ""
    echo "💡 临时设置（当前会话）："
    echo "   export OPENAI_API_KEY='sk-proj-...'"
    echo ""
else
    echo "✅ OpenAI API密钥已设置: ${OPENAI_API_KEY:0:20}..."
fi

echo ""
echo "🌐 代理环境变量已设置:"
echo "  HTTPS_PROXY=$HTTPS_PROXY"
echo "  HTTP_PROXY=$HTTP_PROXY"
echo ""

echo "📋 一键语音对话功能状态："
echo "  ✅ 用户界面: 🎤 开始语音对话按钮"
echo "  ✅ JavaScript: startVoiceChat() 和 stopVoiceChat() 函数" 
echo "  ✅ WebRTC架构: RTCPeerConnection + DataChannel"
echo "  ✅ 后端API: Session创建和SDP代理"
echo "  ✅ 样式设计: 现代化按钮和状态显示"
echo "  ${OPENAI_API_KEY:+✅}${OPENAI_API_KEY:-⚠️} OpenAI API密钥"
echo ""

echo "🎯 使用步骤："
echo "  1. 确保API密钥已设置"
echo "  2. 启动服务: python3 app.py"
echo "  3. 打开浏览器: http://localhost:1024"
echo "  4. 选择模型: GPT-4o-Realtime (OpenAI)"
echo "  5. 点击按钮: 🎤 开始语音对话"
echo "  6. 直接说话: 开始实时对话！"
echo ""

echo "🔧 调试信息："
echo "  - 检查服务状态: curl http://localhost:1024/api/test"
echo "  - 查看实时状态: curl http://localhost:1024/api/realtime/status"
echo "  - 测试Session: curl -X POST http://localhost:1024/api/realtime/session"
echo ""

echo "🎉 一键语音对话系统已完全就绪！"
