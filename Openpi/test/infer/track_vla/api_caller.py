#!/usr/bin/env python3
import cv2
import base64
import time
import websockets
import json
import threading
import logging
from typing import Optional
from threading import Thread
from concurrent.futures import Future
import asyncio
import uuid # <--- 导入uuid库

# 配置全局日志格式
logging.basicConfig(
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    level=logging.INFO
)

# WebSocket配置
WS_TIMEOUT = 10  # 超时时间(秒)

class WebSocketClient:
    """WebSocket客户端（无ROS依赖）"""
    def __init__(self, url):
        self.url = url
        self.logger = logging.getLogger("WebSocketClient")
        self.ws = None
        self._connected = False
        self._event_loop = asyncio.new_event_loop()
        self._response_future = None
        
        self._run_thread = Thread(target=self._start_event_loop, daemon=True)
        self._run_thread.start()
        
        connect_task = asyncio.run_coroutine_threadsafe(self._connect(), self._event_loop)
        try:
            connect_task.result(WS_TIMEOUT)
        except TimeoutError:
            raise ConnectionError("连接超时")
        except Exception as e:
            raise ConnectionError(f"连接失败: {str(e)}")

    def _start_event_loop(self):
        asyncio.set_event_loop(self._event_loop)
        self._event_loop.run_forever()

    async def _connect(self):
        try:
            self.ws = await websockets.connect(
                self.url,
                ping_interval=20,
                ping_timeout=30,
                close_timeout=10
            )
            self._connected = True
            self.logger.info(f"已连接到 {self.url}")
            asyncio.create_task(self._listen())
        except Exception as e:
            self.logger.error(f"连接失败: {str(e)}")
            self._connected = False
            raise

    async def _listen(self):
        try:
            while self._connected:
                try:
                    message = await self.ws.recv()
                    await self._handle_message(message)
                except websockets.exceptions.ConnectionClosed as e:
                    self.logger.warning(f"连接已关闭。代码: {e.code}, 原因: '{e.reason}'")
                    break
        finally:
            self._connected = False
            self.logger.warning("消息监听循环退出")

    async def send_message(self, data, timeout=WS_TIMEOUT):
        if not self._connected:
            raise ConnectionError("未连接到服务器")
        if self._response_future and not self._response_future.done():
            raise RuntimeError("存在未完成的请求")
        
        self._response_future = self._event_loop.create_future()
        await self.ws.send(json.dumps(data))
        
        try:
            return await asyncio.wait_for(self._response_future, timeout)
        except asyncio.TimeoutError:
            self.logger.warning("等待响应超时")
            return None
        finally:
            self._response_future = None
    
    async def _handle_message(self, message):
        try:
            msg = json.loads(message)
            action = msg.get("action")
            if action not in ["ai_response", "reset"]:
                self.logger.warning(f"收到未知action类型消息: {action}")
                return
            
            response_data = msg.get("data", {})
            if "msg" not in response_data:
                self.logger.error("响应缺少msg字段")
                return
                
            if self._response_future and not self._response_future.done():
                self._response_future.set_result(str(response_data["msg"]))
            else:
                self.logger.warning("收到未预期的响应")
        except Exception as e:
            self.logger.error(f"消息处理失败: {str(e)}")

def process_image_from_file(image_path: str) -> Optional[str]:
    """从文件路径读取图片并进行Base64编码"""
    try:
        cv_image = cv2.imread(image_path)
        if cv_image is None:
            logging.error(f"无法从路径读取图片: {image_path}")
            return None
        
        # 将OpenCV默认的BGR格式转换为RGB
        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        
        success, buffer = cv2.imencode(".jpg", cv_image_rgb, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not success:
            raise ValueError("图像编码失败")
            
        return base64.b64encode(buffer).decode('utf-8')
    except Exception as e:
        logging.error(f"图像处理失败: {str(e)}")
        return None

def call_remote_api(client: WebSocketClient, instruction: str, image_path: str, action: str, steps: int):
    """调用远程API的示例函数"""
    logging.info(f"准备调用远程API (action: {action})...")
    
    # 2. 构造消息
    data_payload = {
        "task": instruction,
        "seq": "placeholder", # <--- 关键修正：使用固定的字符串 "placeholder"
        "steps": steps,
        "msg_type": 'action_query',
        "image": None,
        # 为了安全起见，使用浮点数
        "start_pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
        "target_pose": {"x": 1.0, "y": 1.0, "theta": 90.0}
    }

    if action == "next":
        image_base64 = process_image_from_file(image_path)
        if not image_base64:
            logging.error("图片处理失败，取消API调用")
            return
        data_payload["image"] = image_base64
    
    msg = {
        "action": action,
        "data": data_payload
    }
    
    # 3. 发送消息并等待结果
    try:
        # 增加日志，打印出将要发送的完整JSON
        logging.info(f"发送JSON: {json.dumps(msg, indent=2)}")
        logging.info("正在发送数据到云端并等待响应...")
        # 从主线程调用运行在客户端线程事件循环中的协程
        future = asyncio.run_coroutine_threadsafe(
            client.send_message(msg),
            client._event_loop
        )
        response = future.result(timeout=WS_TIMEOUT + 2) # 等待结果返回
        
        if response:
            logging.info(f"成功收到API响应: {response}")
        else:
            logging.warning("API调用超时或未返回有效结果。")
            
    except Exception as e:
        logging.error(f"API调用过程中发生错误: {e}")


if __name__ == '__main__':
    # --- 配置 ---
    SERVER_URL = "ws://222.130.22.78:6666"  # 替换为你的服务器地址
    IMAGE_PATH = "/data/vla/VLA_Diff/Openpi/test/infer/infer_output/images_ep800/image_ep800_frame_0.png"               # 替换为你的本地图片路径
    INSTRUCTION = "Proceed past the streetlight from the right side."       # 替换为你的指令

    # --- 主程序 ---
    ws_client = None
    try:
        # 初始化并连接客户端
        logging.info(f"正在连接到服务器: {SERVER_URL}")
        ws_client = WebSocketClient(SERVER_URL)
        
        # 1. 发送 'reset' 来初始化任务 (step=0, 无图像)
        call_remote_api(
            client=ws_client, 
            instruction=INSTRUCTION, 
            image_path=IMAGE_PATH,
            action="reset",
            steps=0
        )
        
        # 等待一下，确保服务器处理完reset
        time.sleep(1)

        # 2. 发送 'next' 来执行下一步 (step=1, 有图像)
        call_remote_api(
            client=ws_client, 
            instruction=INSTRUCTION, 
            image_path=IMAGE_PATH,
            action="next",
            steps=1
        )
        
    except ConnectionError as e:
        logging.fatal(f"无法建立连接: {e}")
    except FileNotFoundError:
        logging.fatal(f"找不到图片文件，请检查路径: {IMAGE_PATH}")
    except Exception as e:
        logging.fatal(f"程序发生未知错误: {e}")
    finally:
        if ws_client and ws_client._connected:
            logging.info("关闭连接...")
            # 安全地关闭事件循环
            ws_client._event_loop.call_soon_threadsafe(ws_client._event_loop.stop)
        logging.info("程序退出。")