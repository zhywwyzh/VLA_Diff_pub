#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
基于成功项目架构的 WebRTC Realtime 客户端
参考: /Users/zzmm4/Desktop/realtime-voice-gpt4o 项目
"""

import asyncio
import json
import logging
import requests
import os
from typing import Optional, Callable

logger = logging.getLogger(__name__)

class OpenAIRealtimeWebRTCClient:
    """
    OpenAI Realtime API WebRTC 客户端
    使用与成功项目相同的架构：Session + WebRTC
    """
    
    def __init__(self, api_key: str, base_url: str = "http://localhost:1024"):
        self.api_key = api_key
        self.base_url = base_url  # 我们的Flask服务器地址
        self.session_data = None
        self.is_connected = False
        
    async def create_session(self, voice: str = "alloy", instructions: str = None):
        """创建 OpenAI Realtime Session"""
        try:
            if instructions is None:
                instructions = "你是一个有用的AI助手。请用友好、自然的语调回答用户的问题。请用中文回复。"
            
            # 调用我们后端的session创建端点
            session_data = {
                "voice": voice,
                "instructions": instructions
            }
            
            logger.info("正在创建 OpenAI Realtime Session...")
            response = requests.post(
                f"{self.base_url}/api/realtime/session",
                json=session_data,
                timeout=30
            )
            
            if response.status_code == 200:
                self.session_data = response.json()
                logger.info("Session 创建成功")
                return True
            else:
                logger.error(f"Session 创建失败: {response.status_code} {response.text}")
                return False
                
        except Exception as e:
            logger.error(f"创建 Session 时出错: {e}")
            return False
    
    async def connect(self):
        """连接到 Realtime API"""
        try:
            # 首先创建 Session
            if not await self.create_session():
                return False
            
            # 在实际项目中，这里应该建立 WebRTC 连接
            # 由于 Python 的 WebRTC 支持有限，我们先标记为已连接
            # 实际的音频处理需要通过前端的 WebRTC 实现
            
            self.is_connected = True
            logger.info("Realtime 连接已建立（通过 Session）")
            return True
            
        except Exception as e:
            logger.error(f"连接 Realtime API 失败: {e}")
            return False
    
    async def disconnect(self):
        """断开连接"""
        self.is_connected = False
        self.session_data = None
        logger.info("Realtime 连接已断开")
    
    async def send_text(self, text: str):
        """发送文本消息"""
        if not self.is_connected:
            logger.error("未连接到 Realtime API")
            return False
        
        try:
            # 发送到我们的后端处理
            response = requests.post(
                f"{self.base_url}/api/realtime/send-text",
                json={"text": text, "session_data": self.session_data},
                timeout=30
            )
            
            return response.status_code == 200
            
        except Exception as e:
            logger.error(f"发送文本失败: {e}")
            return False
