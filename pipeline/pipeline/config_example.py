#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Gemini 聊天系统配置文件示例

将此文件复制为 config.py 并根据需要修改配置
"""

# ==================== 端口配置 ====================
# 主服务端口
MAIN_SERVICE_PORT = 1024

# 外部接收器端口  
EXTERNAL_RECEIVER_PORT = 3000

# 转发配置
DEFAULT_FORWARD_HOST = 'localhost'
DEFAULT_FORWARD_PORT = EXTERNAL_RECEIVER_PORT
FORWARD_URL = f"http://{DEFAULT_FORWARD_HOST}:{DEFAULT_FORWARD_PORT}/api/receive"

# ==================== API配置 ====================
# Gemini API Key (请替换为您的实际API Key)
GEMINI_API_KEY = "YOUR_API_KEY_HERE"

# Gemini 模型
GEMINI_MODEL = 'gemini-2.5-flash-lite'

# API 超时设置 (秒)
API_TIMEOUT = 30

# ==================== 文件路径配置 ====================
# 上传文件目录
UPLOAD_FOLDER = './uploads'

# 日志目录
LOG_FOLDER = './logs'

# ==================== 网络配置 ====================
# 代理设置 (如果需要)
HTTP_PROXY = None   # 例如: 'http://127.0.0.1:7890'
HTTPS_PROXY = None  # 例如: 'http://127.0.0.1:7890'

# ==================== 功能开关 ====================
# 是否启用转发功能
ENABLE_FORWARDING = True

# 是否启用语音功能
ENABLE_VOICE = True

# 是否启用图片功能  
ENABLE_IMAGE = True

# 是否启用调试模式
DEBUG_MODE = True

# ==================== 消息处理配置 ====================
# 最大消息长度
MAX_MESSAGE_LENGTH = 1024

# 对话历史保留条数
MAX_CONVERSATION_HISTORY = 50

# ==================== 安全配置 ====================
# 允许的文件类型
ALLOWED_FILE_TYPES = ['png', 'jpg', 'jpeg', 'gif', 'webp']

# 最大文件大小 (字节)
MAX_FILE_SIZE = 5 * 1024 * 1024  # 5MB

# ==================== 示例使用方法 ====================
if __name__ == '__main__':
    print("Gemini 聊天系统配置")
    print("=" * 30)
    print(f"主服务端口: {MAIN_SERVICE_PORT}")
    print(f"外部接收器端口: {EXTERNAL_RECEIVER_PORT}")
    print(f"转发URL: {FORWARD_URL}")
    print(f"Gemini模型: {GEMINI_MODEL}")
    print(f"启用转发: {ENABLE_FORWARDING}")
    print(f"启用语音: {ENABLE_VOICE}")
    print(f"启用图片: {ENABLE_IMAGE}")
    print(f"调试模式: {DEBUG_MODE}")
