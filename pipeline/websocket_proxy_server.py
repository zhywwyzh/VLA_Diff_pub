#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WebSocket代理服务器
参考 realtime-voice-gpt4o 项目的架构，提供本地代理来解决 OpenAI Realtime API 连接问题
"""

import asyncio
import websockets
import json
import logging
import os
import signal
import sys
from urllib.parse import parse_qs
import aiohttp

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class RealtimeWebSocketProxy:
    def __init__(self, openai_api_key):
        self.openai_api_key = openai_api_key
        self.proxy_url = os.environ.get('HTTPS_PROXY') or os.environ.get('HTTP_PROXY')
        
    async def handle_client(self, websocket, path):
        """处理客户端连接"""
        logger.info(f"客户端连接: {websocket.remote_address}")
        
        # 解析查询参数
        query_params = parse_qs(path.split('?', 1)[1] if '?' in path else '')
        model = query_params.get('model', ['gpt-4o-realtime-preview'])[0]
        
        openai_ws = None
        try:
            # 连接到 OpenAI Realtime API
            openai_url = f"wss://api.openai.com/v1/realtime?model={model}"
            headers = {
                "Authorization": f"Bearer {self.openai_api_key}",
                "OpenAI-Beta": "realtime=v1"
            }
            
            # 配置代理（如果有）
            connector = None
            if self.proxy_url:
                logger.info(f"使用代理: {self.proxy_url}")
                connector = aiohttp.ProxyConnector.from_url(self.proxy_url)
            
            # 创建 aiohttp session
            async with aiohttp.ClientSession(connector=connector) as session:
                async with session.ws_connect(
                    openai_url,
                    headers=headers,
                    timeout=aiohttp.ClientTimeout(total=30)
                ) as openai_ws:
                    
                    logger.info("已连接到 OpenAI Realtime API")
                    
                    # 通知客户端连接成功
                    await websocket.send(json.dumps({
                        "type": "connection.established",
                        "message": "Connected to OpenAI Realtime API"
                    }))
                    
                    # 创建双向转发任务
                    client_to_openai = asyncio.create_task(
                        self.forward_messages(websocket, openai_ws, "Client->OpenAI")
                    )
                    openai_to_client = asyncio.create_task(
                        self.forward_messages(openai_ws, websocket, "OpenAI->Client")
                    )
                    
                    # 等待任一任务完成
                    done, pending = await asyncio.wait(
                        [client_to_openai, openai_to_client],
                        return_when=asyncio.FIRST_COMPLETED
                    )
                    
                    # 取消未完成的任务
                    for task in pending:
                        task.cancel()
                        
        except Exception as e:
            logger.error(f"代理连接错误: {e}")
            try:
                await websocket.send(json.dumps({
                    "type": "error",
                    "error": f"Proxy connection failed: {str(e)}"
                }))
            except:
                pass
        finally:
            logger.info(f"客户端断开: {websocket.remote_address}")
    
    async def forward_messages(self, source, destination, direction):
        """转发消息"""
        try:
            async for message in source:
                if hasattr(message, 'type'):
                    if message.type == aiohttp.WSMsgType.TEXT:
                        data = message.data
                        logger.debug(f"{direction}: {data[:100]}...")
                        await destination.send(data)
                    elif message.type == aiohttp.WSMsgType.BINARY:
                        logger.debug(f"{direction}: Binary data ({len(message.data)} bytes)")
                        await destination.send(message.data)
                    elif message.type == aiohttp.WSMsgType.ERROR:
                        logger.error(f"{direction}: WebSocket error")
                        break
                else:
                    # websockets 库的消息格式
                    logger.debug(f"{direction}: {str(message)[:100]}...")
                    await destination.send(message)
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"{direction}: Connection closed")
        except Exception as e:
            logger.error(f"{direction}: Forward error: {e}")

async def main():
    """主函数"""
    # 检查 API 密钥
    openai_api_key = os.environ.get('OPENAI_API_KEY')
    if not openai_api_key:
        logger.error("请设置 OPENAI_API_KEY 环境变量")
        sys.exit(1)
    
    # 检查代理配置
    proxy_url = os.environ.get('HTTPS_PROXY') or os.environ.get('HTTP_PROXY')
    if proxy_url:
        logger.info(f"检测到代理配置: {proxy_url}")
    else:
        logger.warning("未检测到代理配置，可能无法连接到 OpenAI API")
    
    # 创建代理服务器
    proxy = RealtimeWebSocketProxy(openai_api_key)
    
    # 启动 WebSocket 服务器
    port = 8765
    logger.info(f"启动 WebSocket 代理服务器，端口: {port}")
    logger.info(f"客户端连接地址: ws://localhost:{port}/realtime-proxy")
    
    # 优雅关闭处理
    def signal_handler(signum, frame):
        logger.info("收到关闭信号，正在关闭服务器...")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        async with websockets.serve(proxy.handle_client, "localhost", port, ping_interval=30):
            logger.info("代理服务器已启动，按 Ctrl+C 停止")
            await asyncio.Future()  # 永远等待
    except Exception as e:
        logger.error(f"服务器启动失败: {e}")
        sys.exit(1)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("服务器已停止")
