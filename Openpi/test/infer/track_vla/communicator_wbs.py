#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter
import cv2
import base64
from cv_bridge import CvBridge
import time
import websockets
import json
import threading
import hashlib
import logging
from typing import Optional
from navi_interface.srv import Upload
from threading import Thread
from concurrent.futures import Future
import asyncio

# 配置全局日志格式
logging.basicConfig(
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    level=logging.INFO
)

# WebSocket配置
WS_RECONNECT_INTERVAL = 5  # 重连间隔(秒)
WS_TIMEOUT = 10  # 超时时间(秒)

class WebSocketClient:
    """WebSocket客户端"""
    def __init__(self, url):
        self.url = url
        self.logger = logging.getLogger("WebSocketClient")
        self.ws = None
        self._connected = False
        self._event_loop = asyncio.new_event_loop()
        self._response_future = None
        
        # 添加事件循环启动方法
        self._run_thread = Thread(
            target=self._start_event_loop, 
            daemon=True
        )
        self._run_thread.start()
        
        # 同步等待连接完成
        self._connect_task = asyncio.run_coroutine_threadsafe(
            self._connect(), 
            self._event_loop
        )
        try:
            self._connect_task.result(WS_TIMEOUT)
        except TimeoutError:
            raise ConnectionError("连接超时")
        except Exception as e:
            raise ConnectionError(f"连接失败: {str(e)}")

    def _start_event_loop(self):
        """事件循环启动方法"""
        asyncio.set_event_loop(self._event_loop)
        self._event_loop.run_forever()

    async def _connect(self):
        """单次连接流程增加Ping配置"""
        try:
            self.ws = await websockets.connect(
                self.url,
                ping_interval=20,  # 每20秒发送一次Ping
                ping_timeout=30,   # 等待Pong响应超时时间
                close_timeout=10   # 关闭超时控制
            )
            self._connected = True
            self.logger.info(f"已连接到 {self.url}")
            asyncio.create_task(self._listen())
        except Exception as e:
            self.logger.error(f"连接失败: {str(e)}")
            self._connected = False
            raise

    async def _listen(self):
        """持续监听消息（增加异常捕获）"""
        try:
            while self._connected:
                try:
                    message = await self.ws.recv()
                    await self._handle_message(message)
                except websockets.exceptions.ConnectionClosedOK:
                    self.logger.info("连接正常关闭")
                    break
                except websockets.exceptions.ConnectionClosedError as e:
                    self.logger.error(f"连接异常关闭: {e.code}")
                    break
                except Exception as e:
                    self.logger.error(f"接收消息异常: {str(e)}")
                    break
        finally:
            self._connected = False
            self.logger.warning("消息监听循环退出")
    async def send_message(self, data, timeout=WS_TIMEOUT):
        """同步化消息发送"""
        if not self._connected:
            raise ConnectionError("未连接到服务器")
        if self._response_future is not None:
            raise RuntimeError("存在未完成的请求")
        self._response_future = self._event_loop.create_future()
        await self.ws.send(json.dumps(data))
        
        try:
            return await asyncio.wait_for(
                self._response_future,
                timeout,
                loop=self._event_loop
            )
        except asyncio.TimeoutError:
            self.logger.warning("等待响应超时")
            return None
        finally:
            self._response_future = None
    
    async def _handle_message(self, message):
        """处理服务器消息"""
        try:
            msg = json.loads(message)
            action = msg.get("action")
            if action != "ai_response" and action != "reset":
                self.logger.warning(f"收到未知action类型消息: {action}")
                return
            
                
            response_data = msg.get("data", {})
            if "msg" not in response_data:
                self.logger.error("响应缺少msg字段")
                return
                
            if self._response_future and not self._response_future.done():
                self._response_future.set_result(str(response_data["msg"]))
            else:
                self.logger.warning("收到未预期的ai_response")
                
        except json.JSONDecodeError:
            self.logger.error("消息JSON解析失败")
        except Exception as e:
            self.logger.error(f"消息处理失败: {str(e)}")

class CloudCommNode(Node):
    """WebSocket通信节点"""
    def __init__(self):
        super().__init__('cloud_communication_node')
        
        # 显式声明参数（包含默认值）
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('response_timeout', 3)
        self.timeout = self.get_parameter('response_timeout').value
        # 初始化调试模式
        self._debug = self.get_parameter('debug_mode').value
        self._logger = self.get_logger()
        if self._debug:
            self._logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
            self._logger.info("调试模式已启用")

        # 初始化服务
        self._init_service()
        
        # 连接管理
        self._init_cloud_connection()
        
        # 性能监控
        self._request_count = 0
        self._success_count = 0
        self._avg_process_time = 0.0

        self.init = True

    def _init_service(self):
        """初始化ROS服务"""
        self.srv = self.create_service(
            Upload, 
            'cloud_communication',
            self._service_handler,
            callback_group=ReentrantCallbackGroup()
        )
        self._logger.info("服务初始化完成")

    def _init_cloud_connection(self):
        """参数化连接配置"""
        self.declare_parameter('server_url', 'ws://222.130.22.78:6666') #222.130.22.78:6666 192.168.61.126 58.214.239.10:8150
        server_url = self.get_parameter('server_url').value
        self._logger.info(f"server_url:{server_url}")
        try:
            # 初始化WebSocket客户端
            self.ws_client = WebSocketClient(server_url)
            self.get_logger().info("云通信节点已初始化")
        except ConnectionError as e:
            self._logger.fatal(str(e))
            raise

    def _process_image(self, ros_image) -> Optional[str]:
        """图像处理，添加调试输出"""
        try:
            # 转换图像
            # start_time = time.time()
            cv_image = CvBridge().imgmsg_to_cv2(ros_image, "rgb8")
            # cv_image = cv2.resize(cv_image,(224, 224), interpolation=cv2.INTER_CUBIC)
            if self._debug:
                self._logger.debug(f"收到图像: {cv_image.shape} 分辨率")
            # self._logger.info(f"图像imgmsg_to_cv2时间: {time.time()-start_time}s")
            # 调试保存
            if self._debug:
                cv2.imwrite(f"debug_.jpg", cv_image)

            # 编码处理
            # start_time = time.time()
            success, buffer = cv2.imencode(".jpg", cv_image, 
                                        [cv2.IMWRITE_JPEG_QUALITY, 85])
            # self._logger.info(f"图像imencode时间: {time.time()-start_time}s")
            if not success:
                raise ValueError("图像编码失败")
                
            return base64.b64encode(buffer).decode('utf-8')
            
        except Exception as e:
            self._logger.error(f"图像处理失败: {str(e)}", throttle_duration_sec=60)
            return None

    def _service_handler(self, request, response):
        """服务处理，添加性能监控"""        
        try:
            # self.get_logger().info(f"start_pose {request.start_pose}")
            start_pose = json.loads(request.start_pose)
            target_pose = json.loads(request.target_pose)
            if request.step == 0:
                cloud_response = self._send_to_cloud(
                    None, 
                    request.instruction,
                    request.step,
                    start_pose,
                    target_pose,
                    action= 'reset'
                )
            # 阶段1: 图像预处理
            image_base64 = self._process_image(request.image)
            if not image_base64:
                response.result = "IMAGE_PROCESS_ERROR"
                return response
            
            


            if request.step != 0:
                self._request_count += 1
                # 阶段2: 云通信
                start_time = time.time()
                cloud_response = self._send_to_cloud(
                                    image_base64, 
                                    request.instruction,
                                    request.step,
                                    start_pose,
                                    target_pose,
                                    action = 'next'
                                )
                # 阶段3: 响应处理
                response.result = cloud_response
                self._success_count += 1
                # 性能统计
                process_time = time.time() - start_time
                self._avg_process_time = (
                    (self._avg_process_time * (self._request_count-1)) + process_time
                ) / self._request_count
                
                self._logger.debug(
                    f"请求处理成功 耗时: {process_time:.2f}s | "
                    f"平均耗时: {self._avg_process_time:.2f}s | "
                    f"成功率: {self._success_count/self._request_count:.1%}"
                )
            else:
                cloud_response = self._send_to_cloud(
                                    image_base64, 
                                    request.instruction,
                                    request.step,
                                    start_pose,
                                    target_pose,
                                    action = 'next'
                                )
                # 阶段3: 响应处理
                response.result = cloud_response   
                
            return response
            
        except Exception as e:
            self._logger.error(f"请求处理异常: {str(e)}")
            response.result = f"SERVER_ERROR: {str(e)}"
            return response

    def _send_to_cloud(self, image_data: str, instruction: str, step: int, start_pose: dict={}, target_pose: dict={}, action: str='next') -> str:
        """发送消息"""
        attempt = 0
        max_attempts = 3
        while attempt < max_attempts:
            try:
                # 构造消息
                msg = {
                    "action": action,
                    "data": {
                        "task": instruction,
                        "seq": "placeholder",
                        "steps": step,
                        "msg_type": 'action_query',
                        "image": image_data,
                        "start_pose": start_pose,
                        "target_pose": target_pose
                    }
                }
                self._logger.debug(f"推理第{step}步...")
                
                # 发送并等待响应
                response = asyncio.run_coroutine_threadsafe(
                    self.ws_client.send_message(msg),
                    self.ws_client._event_loop
                ).result()
                if response:
                    result = response
                else:
                    result = "TIMEOUT"
                
            except Exception as e:
                attempt += 1
                self.get_logger().error(f"处理失败: {str(e)}")
                result = f"ERROR: {str(e)}"
            
            return result
        raise ConnectionError("云通信失败，超过最大重试次数")

def main(args=None):
    rclpy.init(args=args)
    
    try:    
        node = CloudCommNode()
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    except Exception as e:
        print(f"节点运行异常: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()