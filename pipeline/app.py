#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import io
import base64
import json
import tempfile
from datetime import datetime
from flask import Flask, request, jsonify, send_file, send_from_directory
from flask_cors import CORS
import google.generativeai as genai
from openai import OpenAI
from PIL import Image
import speech_recognition as sr
from gtts import gTTS
from pydub import AudioSegment
import logging
import wave
import urllib3
import time
import requests
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry
import subprocess
import shlex
import asyncio
import websockets
import threading
from typing import Dict, Any, Optional
import uuid
from websockets.exceptions import ConnectionClosedError, InvalidStatusCode
import ssl
from pre_prompt import SYSTEM_PROMPT, USER1, ASSISTANT1, USER2, ASSISTANT2, USER3, ASSISTANT3, USER4, ASSISTANT4
from doubao_voice.protocols import MsgType, full_client_request, receive_message
from key import (GEMINI_API_KEY, CHATGLM_API_KEY, OPENAI_API_KEY, DOUBAO_APPID, DOUBAO_ACCESS_TOKEN, DEFAULT_DOUBAO_API_KEY,
                DEFAULT_DOUBAO_BASE_URL, DEFAULT_DOUBAO_EXTRA_HEADERS, DEFAULT_DEFAULT_DOUBAO_ENDPOINT)

# 尝试导入ROS发布器
try:
    from ros_publisher import publish_to_ros, check_ros2_available
    HAS_ROS_PUBLISHER = True
except ImportError:
    HAS_ROS_PUBLISHER = False

# 导入ChatGLM语音SDK
try:
    from zai import ZhipuAiClient
    HAS_CHATGLM_VOICE = True
except ImportError:
    HAS_CHATGLM_VOICE = False
    print("ChatGLM语音SDK未安装，将使用默认TTS")

# 准备豆包语音
USE_DOUBAO_VOICE = True

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)
CORS(app)

# 禁用SSL警告
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

# 配置代理和网络设置
def setup_network_config():
    """配置网络设置，包括代理和超时"""
    # 检查环境变量中的代理设置
    http_proxy = os.environ.get('HTTP_PROXY') or os.environ.get('http_proxy')
    https_proxy = os.environ.get('HTTPS_PROXY') or os.environ.get('https_proxy')
    
    # 如果没有设置代理，尝试常见的代理端口
    if not http_proxy and not https_proxy:
        # 可以根据您的网络环境调整这些代理设置
        # 注释掉或修改为您的实际代理地址
        # os.environ['HTTP_PROXY'] = 'http://127.0.0.1:7890'
        # os.environ['HTTPS_PROXY'] = 'http://127.0.0.1:7890'
        pass
    
    # 配置 requests 会话
    session = requests.Session()
    retry_strategy = Retry(
        total=3,
        backoff_factor=1,
        status_forcelist=[429, 500, 502, 503, 504],
    )
    adapter = HTTPAdapter(max_retries=retry_strategy)
    session.mount("http://", adapter)
    session.mount("https://", adapter)
    
    return session

# 配置网络
network_session = setup_network_config()

# 导入豆包配置
try:
    from doubao_config import (
        DOUBAO_API_KEY, DOUBAO_BASE_URL, DOUBAO_ENDPOINT_MAPPING,
        DEFAULT_DOUBAO_ENDPOINT, DOUBAO_EXTRA_HEADERS,
        get_doubao_endpoint, is_doubao_endpoint_configured
    )
    logger.info("豆包配置文件加载成功")
except ImportError:
    # 回退到直接配置
    DOUBAO_API_KEY = DEFAULT_DOUBAO_API_KEY
    DOUBAO_BASE_URL = DEFAULT_DOUBAO_BASE_URL
    DOUBAO_EXTRA_HEADERS = DEFAULT_DOUBAO_EXTRA_HEADERS
    DEFAULT_DOUBAO_ENDPOINT = DEFAULT_DEFAULT_DOUBAO_ENDPOINT

    def get_doubao_endpoint(model_name):
        return DEFAULT_DOUBAO_ENDPOINT
        
    def is_doubao_endpoint_configured(model_name):
        return model_name == DEFAULT_DOUBAO_ENDPOINT
    
    logger.warning("豆包配置文件未找到，使用默认配置")

# ChatGLM语音配置
CHATGLM_SPEECH_RATE = 0.8  # 默认语速：0.5=慢速，1.0=正常，1.5=快速

# 豆包语音配置
DOUBAO_SPEECH_RATIO = 1

# 豆包深度思考配置
DOUBAO_THINKING_ENABLED = True  # 默认启用深度思考
DOUBAO_SHOW_THINKING_PROCESS = False  # 是否在回答中显示思考过程

# Gemini模型配置
DISABLE_THINKING_MODE = True  # 是否关闭思考模式（针对支持的模型）
PRO_MODEL_TIMEOUT = 120       # Pro模型超时时间（秒）
DEFAULT_MODEL_TIMEOUT = 60    # 其他模型超时时间（秒）

# 配置vllm客户端
try:
    vllm_client = OpenAI(api_key="EMPTY", base_url="http://127.0.0.1:9001/v1")
    logger.info("本地 vLLM(OpenAI兼容) 客户端配置成功: http://127.0.0.1:9001/v1")
except Exception as e:
    logger.error(f"本地 vLLM 客户端配置失败: {e}")
    vllm_client = None

# 配置 API 客户端，增加超时设置
try:
    genai.configure(
        api_key=GEMINI_API_KEY,
        transport='rest',  # 使用REST传输而不是gRPC
        client_options={
            'api_endpoint': 'https://generativelanguage.googleapis.com',
        }
    )
    
    # 当前模型配置（支持动态切换）
    current_model_name = 'gemini-2.5-flash-lite'
    model = genai.GenerativeModel(current_model_name)
    logger.info("Gemini API 配置成功")
    
except Exception as e:
    logger.error(f"Gemini API 配置失败: {e}")
    model = None
    current_model_name = None

# 配置 OpenAI 客户端
try:
    openai_client = OpenAI(api_key=OPENAI_API_KEY)
    logger.info("OpenAI API 配置成功")
except Exception as e:
    logger.error(f"OpenAI API 配置失败: {e}")
    openai_client = None

# 配置豆包客户端
try:
    # 尝试使用官方豆包SDK
    try:
        from volcenginesdkarkruntime import Ark
        doubao_client = Ark(
            base_url=DOUBAO_BASE_URL,
            api_key=DOUBAO_API_KEY,
        )
        DOUBAO_USE_OFFICIAL_SDK = True
        logger.info("豆包 API 配置成功 (使用官方SDK)")
    except ImportError:
        # 回退到OpenAI兼容模式
        doubao_client = OpenAI(
            api_key=DOUBAO_API_KEY,
            base_url=DOUBAO_BASE_URL
        )
        DOUBAO_USE_OFFICIAL_SDK = False
        logger.info("豆包 API 配置成功 (使用OpenAI兼容模式)")
except Exception as e:
    logger.error(f"豆包 API 配置失败: {e}")
    doubao_client = None
    DOUBAO_USE_OFFICIAL_SDK = False

# 初始化ChatGLM客户端
chatglm_client = None
if HAS_CHATGLM_VOICE:
    try:
        chatglm_client = ZhipuAiClient(api_key=CHATGLM_API_KEY)
        logger.info("ChatGLM语音客户端初始化成功")
    except Exception as e:
        logger.error(f"初始化ChatGLM语音客户端失败: {e}")
        chatglm_client = None

# 初始化豆包Voice客户端
doubao_voice_client = None

def get_cluster(voice: str) -> str:
    if voice.startswith("S_"):
        return "volcano_icl"
    return "volcano_tts"

def text_to_speech_doubao(text, appid, access_token, speed_ratio=1.0):
    """同步包装：在无事件循环的脚本中使用"""
    try:
        # 如果当前线程已有事件循环（常见于 FastAPI/Gradio），就禁止同步跑
        asyncio.get_running_loop()
        # 到这里说明已在事件循环中，不能用 asyncio.run
        raise RuntimeError(
            "检测到正在运行的事件循环。请在 async 环境中使用："
            "await text_to_speech_doubao_async(...)"
        )
    except RuntimeError:
        # 没有事件循环，可以安全地跑
        return asyncio.run(
            text_to_speech_doubao_async(text, appid, access_token, speed_ratio=speed_ratio)
        )

# 豆包语音生成函数
async def text_to_speech_doubao_async(text: str, appid: str, access_token: str, 
                           speed_ratio: float = 1, voice_type: str = "zh_female_linjianvhai_moon_bigtts", 
                           encoding: str = "wav", cluster: str = "volcano_tts"):
    """使用豆包语音合成生成语音
    Args:
        text (str): 要转换的文本
        voice_type (str): 语音类型
        appid (str): APP ID
        access_token (str): 访问令牌
        encoding (str): 输出音频编码格式（默认为"wav"）
        cluster (str): 集群名称（如果为空则自动根据语音类型选择）
    Returns:
        str: 生成的语音文件路径
    """
    try:
        # 连接WebSocket服务器
        websocket = await websockets.connect(
            "wss://openspeech.bytedance.com/api/v1/tts/ws_binary",
            additional_headers={"Authorization": f"Bearer;{access_token}"},
            max_size=10 * 1024 * 1024
        )
        logger.info(f"Connected to WebSocket server")
        logger.info("使用豆包生成语音...")

        # 确定使用的集群
        cluster = cluster if cluster else get_cluster(voice_type)

        # 请求的音频数据
        request_payload = {
            "app": {
                "appid": appid,
                "token": access_token,
                "cluster": cluster,
            },
            "user": {
                "uid": str(uuid.uuid4()),
            },
            "audio": {
                "voice_type": voice_type,
                "encoding": encoding,
                "speed_ratio": speed_ratio,
            },
            "request": {
                "reqid": str(uuid.uuid4()),
                "text": text,
                "operation": "submit",
                "with_timestamp": "1",
                "extra_param": json.dumps({"disable_markdown_filter": False}),
            },
        }

        # 发送请求
        await full_client_request(websocket, json.dumps(request_payload).encode())

        # 接收音频数据
        audio_data = bytearray()
        while True:
            msg = await receive_message(websocket)

            if msg.type == MsgType.FrontEndResultServer:
                continue
            elif msg.type == MsgType.AudioOnlyServer:
                audio_data.extend(msg.payload)
                if msg.sequence < 0:  # 最后一条消息
                    break
            else:
                raise RuntimeError(f"TTS转换失败: {msg}")

        # 检查音频数据是否接收成功
        if not audio_data:
            raise RuntimeError("未接收到音频数据")

        # 保存音频文件
        filename = f"{voice_type}_{uuid.uuid4()}.{encoding}"
        with open(filename, "wb") as f:
            f.write(audio_data)
        logger.info(f"生成语音成功，保存到: {filename}")

        return filename

    except Exception as e:
        logger.error(f"豆包语音生成失败: {e}")
        return None

# ChatGLM语音生成函数
def text_to_speech_chatglm(text, speech_rate=0.8):
    """使用ChatGLM GLM-4-Voice生成语音
    
    Args:
        text (str): 要转换的文本
        speech_rate (float): 语速调节系数，0.5=慢速，1.0=正常，1.5=快速
    """
    try:
        if not chatglm_client:
            logger.warning("ChatGLM客户端未初始化")
            return None
        
        logger.info(f"使用ChatGLM生成语音: {text[:50]}... (语速: {speech_rate}x)")
        
        # 根据语速调整提示词
        speed_instruction = ""
        if speech_rate <= 0.7:
            speed_instruction = "5. 请缓慢清晰地朗读，语速要慢一些"
        elif speech_rate >= 1.3:
            speed_instruction = "5. 请快速流畅地朗读，语速要快一些"
        else:
            speed_instruction = "5. 请以正常语速朗读"
        
        # 构造强硬的系统指令，确保只复述文本内容
        strict_prompt = f"""请严格按照以下要求执行：
1. 只能复述下面给出的文本内容
2. 不得添加任何解释、评论或额外的话语
3. 不得改变原文的任何字词
4. 直接朗读原文，语气自然流畅
{speed_instruction}


要朗读的文本：
{text}"""
        
        # 调用GLM-4-Voice API
        response = chatglm_client.chat.completions.create(
            model="glm-4-voice",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": strict_prompt
                        }
                    ]
                }
            ],
            stream=False
        )
        
        # 检查响应中是否包含音频数据
        if response.choices[0].message.audio and response.choices[0].message.audio.get('data'):
            audio_data = response.choices[0].message.audio['data']
            decoded_data = base64.b64decode(audio_data)
            
            # 保存原始WAV文件
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp_file:
                # 根据API文档，GLM-4-Voice返回的是WAV格式
                with wave.open(tmp_file.name, 'wb') as wav_file:
                    wav_file.setnchannels(1)      # 单声道
                    wav_file.setsampwidth(2)      # 16位
                    wav_file.setframerate(44100)  # 44.1kHz
                    wav_file.writeframes(decoded_data)
                
                # 如果需要调整语速，使用pydub处理音频
                if speech_rate != 1.0:
                    try:
                        from pydub import AudioSegment
                        
                        # 加载音频文件
                        audio_segment = AudioSegment.from_wav(tmp_file.name)
                        
                        # 调整播放速度（不改变音调）
                        # speech_rate < 1.0 = 慢速，> 1.0 = 快速
                        if speech_rate != 1.0:
                            # 通过改变帧率来调整速度，然后重新采样回原始帧率
                            new_frame_rate = int(audio_segment.frame_rate * speech_rate)
                            audio_segment = audio_segment._spawn(audio_segment.raw_data, overrides={
                                "frame_rate": new_frame_rate
                            })
                            # 重新采样回44100Hz
                            audio_segment = audio_segment.set_frame_rate(44100)
                        
                        # 保存调整后的音频
                        adjusted_tmp_file = tempfile.NamedTemporaryFile(suffix='.wav', delete=False)
                        audio_segment.export(adjusted_tmp_file.name, format="wav")
                        
                        # 删除原始文件，返回调整后的文件
                        os.unlink(tmp_file.name)
                        logger.info(f"ChatGLM语音生成并调速成功，保存到: {adjusted_tmp_file.name}")
                        return adjusted_tmp_file.name
                        
                    except Exception as audio_error:
                        logger.warning(f"音频调速失败，使用原始音频: {audio_error}")
                        # 如果调速失败，返回原始音频
                        pass
                
                logger.info(f"ChatGLM语音生成成功，保存到: {tmp_file.name}")
                return tmp_file.name
        else:
            logger.warning("ChatGLM响应中没有音频数据")
            return None
            
    except Exception as e:
        logger.error(f"ChatGLM语音生成失败: {e}")
        return None

# 创建上传目录
UPLOAD_FOLDER = './uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

# 对话历史
conversation_history = []

# 转发配置 - 默认设置
DEFAULT_FORWARD_PORT = 3000  # 默认外部接收器端口
FORWARD_URL = f"http://localhost:{DEFAULT_FORWARD_PORT}/api/receive"  # 默认转发地址

class MultimodalChat:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        
    def speech_to_text(self, audio_data):
        """将语音转换为文字"""
        try:
            # 保存音频文件
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp_file:
                tmp_file.write(audio_data)
                tmp_file_path = tmp_file.name
            
            # 尝试使用pydub处理音频格式（如果没有ffmpeg则跳过预处理）
            processed_path = tmp_file_path
            try:
                # 检查音频格式并尝试处理
                audio_segment = AudioSegment.from_file(tmp_file_path)
                # 转换为适合语音识别的格式
                audio_segment = audio_segment.set_frame_rate(16000).set_channels(1)
                
                # 保存为新的wav文件
                with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as processed_file:
                    audio_segment.export(processed_file.name, format="wav")
                    processed_path = processed_file.name
                
                logger.info("音频预处理成功")
                    
            except Exception as process_error:
                logger.warning(f"音频预处理失败，使用原始音频: {process_error}")
                # 如果预处理失败，使用原始音频文件
                processed_path = tmp_file_path
            
            # 使用SpeechRecognition转换语音
            try:
                with sr.AudioFile(processed_path) as source:
                    # 调整识别器设置
                    self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
                    audio = self.recognizer.record(source)
                    
                    # 尝试多种语言识别
                    try:
                        text = self.recognizer.recognize_google(audio, language='zh-CN')
                        logger.info(f"中文语音识别成功: {text}")
                    except sr.UnknownValueError:
                        # 如果中文识别失败，尝试英文
                        text = self.recognizer.recognize_google(audio, language='en-US')
                        logger.info(f"英文语音识别成功: {text}")
                    except sr.RequestError as e:
                        logger.error(f"语音识别服务错误: {e}")
                        return "语音识别服务暂时不可用"
                        
            except Exception as recognition_error:
                logger.error(f"语音识别失败: {recognition_error}")
                return None
            
            # 清理临时文件
            try:
                if processed_path != tmp_file_path:
                    os.unlink(processed_path)
                os.unlink(tmp_file_path)
            except:
                pass
                
            return text
            
        except Exception as e:
            logger.error(f"语音识别错误: {e}")
            return None
    
    def text_to_speech(self, text, lang='zh', speech_rate=0.8):
        """将文字转换为语音 - 优先使用Doubao语音，ChatGLM，回退到gTTS"""
        if doubao_voice_client:
            doubao_audio = text_to_speech_doubao(text, appid=DOUBAO_APPID, access_token=DOUBAO_ACCESS_TOKEN, speed_ratio=DOUBAO_SPEECH_RATIO)
            if doubao_audio:
                return doubao_audio
        # 首先尝试使用ChatGLM语音生成
        if chatglm_client:
            chatglm_audio = text_to_speech_chatglm(text, speech_rate)
            if chatglm_audio:
                return chatglm_audio
        
        # 回退到gTTS
        try:
            tts = gTTS(text=text, lang=lang)
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as tmp_file:
                tts.save(tmp_file.name)
                return tmp_file.name
        except Exception as e:
            logger.error(f"文字转语音错误: {e}")
            return None
    

    
    def process_image(self, image_data):
        """处理图片数据"""
        try:
            # 解码base64图片
            if ',' in image_data:
                image_data = image_data.split(',')[1]
            
            image_bytes = base64.b64decode(image_data)
            image = Image.open(io.BytesIO(image_bytes))
            return image
        except Exception as e:
            logger.error(f"图片处理错误: {e}")
            return None
    
    def generate_response(self, text_input=None, images=None, audio_text=None, model_name=None):
        """生成多模态响应（支持多图片和模型选择，包括 Gemini、OpenAI 和豆包）"""
        # 判断模型类型
        if model_name and ('gpt' in model_name.lower() or 'openai' in model_name.lower()):
            return self._generate_openai_response(text_input, images, audio_text, model_name)
        elif model_name and 'doubao' in model_name.lower():
            return self._generate_doubao_response(text_input, images, audio_text, model_name)
        elif model_name and 'gemini' in model_name.lower():
            return self._generate_gemini_response(text_input, images, audio_text, model_name)
        else:
            return self._generate_vllm_response(text_input, images, audio_text, model_name)

    def _generate_openai_response(self, text_input=None, images=None, audio_text=None, model_name=None):
        """使用 OpenAI API 生成响应"""
        if openai_client is None:
            return "抱歉，OpenAI 服务未配置或不可用，请检查 API 配置。"
        
        try:
            # 构建消息内容
            messages = []
            
            # 系统消息 - 更明确的指导
            system_message = "你是一个智能助手，负责处理文字和图片的多模态信息。"
            messages.append({"role": "system", "content": system_message})
            
            # 添加对话历史 - 对于 OpenAI API，禁用历史上下文以节省 token
            # 注释掉以下代码以禁用对话历史：
            # if conversation_history:
            #     # 如果当前输入很长（包含长文本或图片），减少历史记录
            #     current_input_length = len(text_input) if text_input else 0
            #     if images:
            #         current_input_length += 1000  # 图片大致相当于1000字符
            #     
            #     if current_input_length > 500:
            #         history_limit = 1  # 只保留最近1轮对话
            #     elif current_input_length > 200:
            #         history_limit = 2  # 保留最近2轮对话
            #     else:
            #         history_limit = 3  # 保留最近3轮对话
            #     
            #     for entry in conversation_history[-history_limit:]:
            #         messages.append({"role": "user", "content": entry['user']})
            #         messages.append({"role": "assistant", "content": entry['assistant']})
            #     
            #     logger.info(f"包含 {history_limit} 轮对话历史 (当前输入长度: {current_input_length})")
            
            logger.info("OpenAI API 调用已禁用对话历史上下文")
            
            # 构建当前用户消息 - 按照OpenAI标准格式
            user_content = []
            
            # 添加文字内容（直接使用原始文字，不加前缀）
            combined_text = ""
            if text_input:
                combined_text += text_input
            if audio_text:
                if combined_text:
                    combined_text += "\n\n[语音识别内容]: " + audio_text
                else:
                    combined_text = audio_text
            
            # 如果有文字内容，添加为第一个元素
            if combined_text.strip():
                user_content.append({
                    "type": "text",
                    "text": combined_text
                })
            
            # 处理图片内容（无论是否有文字都添加图片）
            if images and len(images) > 0:
                # 转换图片为 OpenAI 格式
                for i, image in enumerate(images):
                    try:
                        # 固定图片大小为640x480
                        target_width = 640
                        target_height = 480
                        quality = 85
                        
                        # 调整图片大小为固定尺寸
                        if image.width != target_width or image.height != target_height:
                            image = image.resize((target_width, target_height), Image.Resampling.LANCZOS)
                            logger.info(f"图片 {i+1} 已调整为固定大小: {target_width}x{target_height}")
                        
                        # 将 PIL 图片转换为 base64，保持原始格式或转为JPEG以减小大小
                        buffer = io.BytesIO()
                        
                        # 检查图片模式，确保与保存格式兼容
                        if image.mode in ('RGBA', 'LA', 'P'):
                            # 如果有透明通道，转为PNG
                            image.save(buffer, format='PNG', optimize=True)
                            image_format = 'png'
                        else:
                            # 否则转为JPEG以减小文件大小，使用固定质量
                            if image.mode != 'RGB':
                                image = image.convert('RGB')
                            image.save(buffer, format='JPEG', quality=quality, optimize=True)
                            image_format = 'jpeg'
                        
                        buffer.seek(0)
                        image_data = buffer.getvalue()
                        image_base64 = base64.b64encode(image_data).decode('utf-8')
                        
                        # 检查最终大小
                        size_mb = len(image_data) / (1024 * 1024)
                        logger.info(f"图片 {i+1} 转换完成: 格式={image_format}, 尺寸={image.width}x{image.height}, 大小={size_mb:.2f}MB")
                        
                        # 如果图片仍然太大，降低质量
                        if size_mb > 15:  # 如果超过15MB，进一步压缩
                            logger.warning(f"图片 {i+1} 过大，进行额外压缩")
                            buffer = io.BytesIO()
                            if image_format == 'jpeg':
                                image.save(buffer, format='JPEG', quality=60, optimize=True)
                            else:
                                # PNG转JPEG进行压缩
                                rgb_image = image.convert('RGB')
                                rgb_image.save(buffer, format='JPEG', quality=60, optimize=True)
                                image_format = 'jpeg'
                            
                            buffer.seek(0)
                            image_data = buffer.getvalue()
                            image_base64 = base64.b64encode(image_data).decode('utf-8')
                            size_mb = len(image_data) / (1024 * 1024)
                            logger.info(f"图片 {i+1} 压缩后大小: {size_mb:.2f}MB")
                        
                        # 使用固定的high详细级别
                        detail_level = "high"
                        
                        user_content.append({
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/{image_format};base64,{image_base64}",
                                "detail": detail_level
                            }
                        })
                        
                        logger.info(f"图片 {i+1} 使用固定detail级别: {detail_level}")
                    except Exception as e:
                        logger.error(f"处理图片 {i+1} 时出错: {e}")
                        # 尝试备用方案：强制转PNG
                        try:
                            buffer = io.BytesIO()
                            rgb_image = image.convert('RGB')
                            rgb_image.save(buffer, format='PNG')
                            buffer.seek(0)
                            image_base64 = base64.b64encode(buffer.getvalue()).decode('utf-8')
                            
                            user_content.append({
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/png;base64,{image_base64}",
                                    "detail": "high"
                                }
                            })
                            logger.info(f"图片 {i+1} 使用备用方案转换成功")
                        except Exception as backup_error:
                            logger.error(f"图片 {i+1} 备用转换也失败: {backup_error}")
            
            # 如果没有任何内容，提供默认消息
            if not user_content:
                user_content = [{"type": "text", "text": "Hello"}]
            
            messages.append({"role": "user", "content": user_content})
            
            # 确定使用的模型
            if not model_name or model_name == 'gpt-4o':
                api_model = 'gpt-4o'
            elif model_name == 'gpt-4o-mini':
                api_model = 'gpt-4o-mini'
            else:
                api_model = 'gpt-4o'  # 默认使用 gpt-4o
            
            logger.info(f"正在调用 OpenAI API，模型: {api_model}")
            
            # 调试：记录发送给 OpenAI 的消息
            logger.info(f"发送给 OpenAI 的消息数量: {len(messages)}")
            logger.info(f"用户消息内容长度: {len(str(messages[-1]))}")
            
            # 统一设置max_tokens为4096
            max_tokens = 4096
            
            # 调用 OpenAI API
            start_time = time.time()
            response = openai_client.chat.completions.create(
                model=api_model,
                messages=messages,
                max_tokens=max_tokens,
                temperature=0.7,
                timeout=180  # 3分钟超时
            )
            
            logger.info(f"使用 max_tokens: {max_tokens}")
            
            # 记录耗时
            elapsed_time = time.time() - start_time
            logger.info(f"OpenAI 模型 {api_model} 响应耗时: {elapsed_time:.2f}秒")
            
            if response.choices and len(response.choices) > 0:
                content = response.choices[0].message.content
                logger.info(f"OpenAI API 调用成功，返回内容: {content}")
                
                # 检查是否被拒绝
                if content and ("无法" in content or "抱歉" in content or "不能" in content or "cannot" in content.lower() or "sorry" in content.lower()):
                    logger.warning(f"OpenAI 可能拒绝了请求: {content}")
                
                return content
            else:
                logger.warning("OpenAI API 返回空响应")
                return "抱歉，我暂时无法生成回复。请稍后重试。"
                
        except Exception as e:
            logger.error(f"OpenAI API 调用失败: {e}")
            if "timeout" in str(e).lower():
                return "抱歉，网络连接超时。请检查您的网络连接或稍后重试。"
            elif "rate_limit" in str(e).lower():
                return "抱歉，API 调用频率超限。请稍后重试。"
            elif "invalid_request" in str(e).lower():
                return "抱歉，请求格式有误。请检查输入内容。"
            else:
                return f"抱歉，处理请求时出现错误: {str(e)}"
            
    def _generate_vllm_response(self, text_input=None, images=None, audio_text=None, model_name=None):
        """
        使用本地 vLLM(OpenAI兼容) 生成响应（不影响原来的 gpt-4o / openai / gemini / doubao）
        - text_input: 文本输入
        - images: List[PIL.Image] 已处理好的图片
        - audio_text: 语音转写文本（可空）
        - model_name: 透传给 vLLM 的模型名；不传则默认 'qwen3-vl-8b'
        """
        if vllm_client is None:
            return "抱歉，本地 vLLM 服务不可用，请检查 127.0.0.1:9001 是否在运行。"

        try:
            messages = []
            messages.extend([
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": USER1},
                {"role": "assistant", "content": ASSISTANT1},
                {"role": "user", "content": USER2},
                {"role": "assistant", "content": ASSISTANT2},
                {"role": "user", "content": USER3},
                {"role": "assistant", "content": ASSISTANT3},
                {"role": "user", "content": USER4},
                {"role": "assistant", "content": ASSISTANT4}
            ])

            user_content = []
            combined_text = ""
            if text_input:
                combined_text += text_input
            if audio_text:
                combined_text = (combined_text + "\n\n[语音识别内容]: " + audio_text) if combined_text else audio_text
            if combined_text.strip():
                user_content.append({"type": "text", "text": combined_text})

            # 与原 /api/chat 的图片处理保持一致：固定 640x480，按需 PNG/JPEG，生成 data URL
            if images and len(images) > 0:
                for i, image in enumerate(images):
                    try:
                        target_width, target_height, quality = 640, 480, 85
                        if image.width != target_width or image.height != target_height:
                            image = image.resize((target_width, target_height), Image.Resampling.LANCZOS)
                            logger.info(f"[vLLM] 图片 {i+1} 已调整为: {target_width}x{target_height}")

                        buffer = io.BytesIO()
                        if image.mode in ("RGBA", "LA", "P"):
                            image.save(buffer, format="PNG", optimize=True)
                            image_format = "png"
                        else:
                            if image.mode != "RGB":
                                image = image.convert("RGB")
                            image.save(buffer, format="JPEG", quality=quality, optimize=True)
                            image_format = "jpeg"

                        buffer.seek(0)
                        image_data = buffer.getvalue()
                        image_base64 = base64.b64encode(image_data).decode("utf-8")
                        size_mb = len(image_data) / (1024 * 1024)
                        logger.info(f"[vLLM] 图片 {i+1} 转换: {image_format}, {image.width}x{image.height}, {size_mb:.2f}MB")

                        if size_mb > 15:
                            logger.warning(f"[vLLM] 图片 {i+1} 过大，压缩")
                            buffer = io.BytesIO()
                            if image_format == "jpeg":
                                image.save(buffer, format="JPEG", quality=60, optimize=True)
                            else:
                                rgb_image = image.convert("RGB")
                                rgb_image.save(buffer, format="JPEG", quality=60, optimize=True)
                                image_format = "jpeg"
                            buffer.seek(0)
                            image_data = buffer.getvalue()
                            image_base64 = base64.b64encode(image_data).decode("utf-8")
                            size_mb = len(image_data) / (1024 * 1024)
                            logger.info(f"[vLLM] 图片 {i+1} 压缩后: {size_mb:.2f}MB")

                        user_content.append({
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/{image_format};base64,{image_base64}",
                                "detail": "high"
                            }
                        })
                    except Exception as e:
                        logger.error(f"[vLLM] 处理图片 {i+1} 出错: {e}")
                        try:
                            buffer = io.BytesIO()
                            rgb_image = image.convert("RGB")
                            rgb_image.save(buffer, format="PNG")
                            buffer.seek(0)
                            image_base64 = base64.b64encode(buffer.getvalue()).decode("utf-8")
                            user_content.append({
                                "type": "image_url",
                                "image_url": {"url": f"data:image/png;base64,{image_base64}", "detail": "high"}
                            })
                            logger.info(f"[vLLM] 备用 PNG 转换成功（图片 {i+1}）")
                        except Exception as backup_error:
                            logger.error(f"[vLLM] 备用转换失败（图片 {i+1}）: {backup_error}")

            if not user_content:
                user_content = [{"type": "text", "text": "Hello"}]

            messages.append({"role": "user", "content": user_content})

            # api_model = model_name or "qwen3-vl-8b"  # vLLM 默认模型名，可按你的进程 served name 修改
            api_model = vllm_client.models.list().data[0].id
            logger.info(f"[vLLM] 调用 chat.completions.create, model={api_model}")

            max_tokens = 4096
            start_time = time.time()
            response = vllm_client.chat.completions.create(
                model=api_model,
                messages=messages,
                max_tokens=max_tokens,
                temperature=0.8,
                top_p=0.8
                # timeout=180
            )
            elapsed = time.time() - start_time
            logger.info(f"[vLLM] 响应耗时: {elapsed:.2f}s")

            if response.choices and len(response.choices) > 0:
                content = response.choices[0].message.content
                return content or "抱歉，本地 vLLM 返回了空文本。"
            return "抱歉，本地 vLLM 没有返回可用结果。"

        except Exception as e:
            logger.error(f"[vLLM] 调用失败: {e}")
            emsg = str(e).lower()
            if "timeout" in emsg:
                return "抱歉，vLLM 连接超时，请检查本地服务。"
            elif "invalid" in emsg:
                return "抱歉，vLLM 请求格式可能无效，请检查输入内容。"
            else:
                return f"抱歉，vLLM 处理请求时出现错误: {str(e)}"
    
    def _generate_gemini_response(self, text_input=None, images=None, audio_text=None, model_name=None):
        """使用 Gemini API 生成响应"""
        # 选择使用的模型
        selected_model = model
        if model_name and model_name != current_model_name:
            try:
                selected_model = genai.GenerativeModel(model_name)
                logger.info(f"使用指定模型: {model_name}")
            except Exception as e:
                logger.warning(f"指定模型 {model_name} 不可用，使用默认模型: {e}")
                selected_model = model
        
        if selected_model is None:
            return "抱歉，AI服务暂时不可用，请检查网络连接或API配置。"
            
        try:
            # 构建提示信息
            prompt_parts = []
            
            # 系统指令
            # system_instruction = "你是一个智能助手，能够理解和处理文字、图片和语音输入。请用简洁而友好的中文回复用户。"
            # prompt_parts.append(system_instruction)
            # prompt_parts.extend([
            #     {"role": "system", "content": SYSTEM_PROMPT},
            #     {"role": "user", "content": USER1},
            #     {"role": "assistant", "content": ASSISTANT1},
            #     {"role": "user", "content": USER2},
            #     {"role": "assistant", "content": ASSISTANT2},
            #     {"role": "user", "content": USER3},
            #     {"role": "assistant", "content": ASSISTANT3},
            #     {"role": "user", "content": USER4},
            #     {"role": "assistant", "content": ASSISTANT4}
            # ])

            prompt_parts.extend([
                                f"系统：{SYSTEM_PROMPT}",
                                "对话历史：",
                                f"用户1：{USER1}",
                                f"助手1：{ASSISTANT1}",
                                f"用户2：{USER2}",
                                f"助手2：{ASSISTANT2}",
                                f"用户3：{USER3}",
                                f"助手3：{ASSISTANT3}",
                                f"用户4：{USER4}",
                                f"助手4：{ASSISTANT4}"])

            # 添加对话历史 - 已禁用历史上下文以提高性能
            # if conversation_history:
            #     prompt_parts.append("对话历史:")
            #     for entry in conversation_history[-5:]:
            #         prompt_parts.append(f"用户: {entry['user']}")
            #         prompt_parts.append(f"助手: {entry['assistant']}")
            
            # prompt_parts.append("当前对话:")
            
            logger.info("Gemini API 调用已禁用对话历史上下文")
            
            # 处理用户输入
            user_input_parts = []
            if text_input:
                user_input_parts.append(f"文字输入: {text_input}")
            if audio_text:
                user_input_parts.append(f"语音输入: {audio_text}")
            if images and len(images) > 0:
                if len(images) == 1:
                    user_input_parts.append("用户还上传了一张图片，请分析图片内容。")
                else:
                    user_input_parts.append(f"用户还上传了{len(images)}张图片，请分析这些图片的内容。")
            
            # 构建完整提示
            full_prompt = "\n".join(prompt_parts + user_input_parts)
            
            # 准备模型输入（支持多图片）
            content = [full_prompt]
            if images:
                content.extend(images)
            
            logger.info("正在调用 Gemini API...")
            
            # 使用重试机制调用 Gemini API
            max_retries = 3
            for attempt in range(max_retries):
                try:
                    # 配置生成参数，针对不同模型优化
                    generation_config_params = {
                        'temperature': 0.7,
                        'top_p': 0.8,
                        'top_k': 40,
                        'max_output_tokens': 8192,
                    }
                    
                    # 针对Flash模型，尝试关闭思考模式以提高速度
                    if model_name and 'flash' in model_name.lower() and DISABLE_THINKING_MODE:
                        # 注意：thinking_budget参数在当前SDK版本中可能不可用
                        # 我们通过其他方式优化Flash模型性能
                        generation_config_params['temperature'] = 0.1  # 降低随机性提高速度
                        generation_config_params['top_p'] = 0.9
                        logger.info(f"为Flash模型 {model_name} 优化参数以提高速度")
                    
                    # 对于Pro模型，降低temperature以获得更稳定的输出
                    elif model_name and 'pro' in model_name.lower():
                        generation_config_params['temperature'] = 0.3  # 降低随机性
                        generation_config_params['top_p'] = 0.9       # 提高确定性
                        generation_config_params['top_k'] = 20        # 减少候选词汇
                        generation_config_params['max_output_tokens'] = 8192  # 统一设置为8192
                        logger.info(f"为Pro模型 {model_name} 调整参数以提高稳定性和速度")
                    
                    generation_config = genai.GenerationConfig(**generation_config_params)
                    
                    # 配置安全设置，特别是对于Pro模型
                    safety_settings = [
                        {
                            "category": "HARM_CATEGORY_HARASSMENT",
                            "threshold": "BLOCK_ONLY_HIGH"
                        },
                        {
                            "category": "HARM_CATEGORY_HATE_SPEECH", 
                            "threshold": "BLOCK_ONLY_HIGH"
                        },
                        {
                            "category": "HARM_CATEGORY_SEXUALLY_EXPLICIT",
                            "threshold": "BLOCK_ONLY_HIGH"
                        },
                        {
                            "category": "HARM_CATEGORY_DANGEROUS_CONTENT",
                            "threshold": "BLOCK_ONLY_HIGH"
                        }
                    ]
                    
                    # 调用 API，使用配置的超时时间
                    timeout_duration = PRO_MODEL_TIMEOUT if model_name and 'pro' in model_name.lower() else DEFAULT_MODEL_TIMEOUT
                    logger.info(f"调用模型 {model_name}，超时时间: {timeout_duration}秒")
                    
                    start_time = time.time()
                    response = selected_model.generate_content(
                        content,
                        generation_config=generation_config,
                        safety_settings=safety_settings,
                        request_options={'timeout': timeout_duration}  # Pro模型120秒，其他60秒
                    )
                    
                    # 记录实际耗时
                    elapsed_time = time.time() - start_time
                    logger.info(f"模型 {model_name} 响应耗时: {elapsed_time:.2f}秒")
                    
                    if response:
                        # 检查finish_reason
                        if hasattr(response, 'candidates') and response.candidates:
                            candidate = response.candidates[0]
                            if hasattr(candidate, 'finish_reason'):
                                finish_reason = candidate.finish_reason
                                logger.info(f"响应完成原因: {finish_reason}")
                                
                                # 处理不同的完成原因
                                if finish_reason == 2:  # SAFETY
                                    return "抱歉，您的请求可能触发了安全过滤器。请尝试修改您的问题描述，避免使用可能被误解的词汇。"
                                elif finish_reason == 3:  # RECITATION 
                                    return "抱歉，回复内容可能涉及版权问题。请换个方式提问。"
                                elif finish_reason == 4:  # OTHER
                                    return "抱歉，处理您的请求时遇到了其他问题。请稍后重试。"
                        
                        # 尝试获取响应文本
                        if hasattr(response, 'text') and response.text:
                            logger.info("API调用成功")
                            return response.text
                        else:
                            # 检查是否有parts
                            if hasattr(response, 'candidates') and response.candidates:
                                candidate = response.candidates[0]
                                if hasattr(candidate, 'content') and candidate.content:
                                    if hasattr(candidate.content, 'parts') and candidate.content.parts:
                                        parts_text = []
                                        for part in candidate.content.parts:
                                            if hasattr(part, 'text') and part.text:
                                                parts_text.append(part.text)
                                        if parts_text:
                                            logger.info("从parts获取响应文本成功")
                                            return '\n'.join(parts_text)
                            
                            logger.warning("API返回空响应或无法解析响应内容")
                            return "抱歉，AI暂时无法处理您的请求。请尝试简化您的问题或稍后重试。"
                    else:
                        logger.warning("API返回空响应对象")
                        return "抱歉，我暂时无法生成回复。请稍后重试。"
                        
                except Exception as api_error:
                    logger.warning(f"API调用失败 (尝试 {attempt + 1}/{max_retries}): {api_error}")
                    if attempt < max_retries - 1:
                        time.sleep(2 ** attempt)  # 指数退避
                        continue
                    else:
                        # 所有重试都失败了
                        if "timeout" in str(api_error).lower():
                            return "抱歉，网络连接超时。请检查您的网络连接或稍后重试。"
                        elif "unavailable" in str(api_error).lower():
                            return "抱歉，AI服务暂时不可用。请稍后重试。"
                        else:
                            return f"抱歉，处理请求时出现错误。请稍后重试。"
                
        except Exception as e:
            logger.error(f"生成回复错误: {e}")
            if "timeout" in str(e).lower():
                return "抱歉，网络连接超时。请检查您的网络连接。"
            elif "connection" in str(e).lower():
                return "抱歉，无法连接到AI服务。请检查网络连接。"
            else:
                return "抱歉，处理请求时出现未知错误。请稍后重试。"
    
    def _generate_doubao_response(self, text_input=None, images=None, audio_text=None, model_name=None):
        """使用豆包 API 生成响应"""
        if doubao_client is None:
            return "抱歉，豆包服务未配置或不可用，请检查 API 配置。"
        
        try:
            # 构建消息内容
            messages = []

            # 系统消息
            system_message = "你是一个智能助手，负责处理文字和图片的多模态信息。请用简洁而友好的中文回复用户。"
            messages.extend([
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": USER1},
                {"role": "assistant", "content": ASSISTANT1},
                {"role": "user", "content": USER2},
                {"role": "assistant", "content": ASSISTANT2},
                {"role": "user", "content": USER3},
                {"role": "assistant", "content": ASSISTANT3},
                {"role": "user", "content": USER4},
                {"role": "assistant", "content": ASSISTANT4}
            ])
            # messages.append({"role": "system", "content": system_message})

            # 构建当前用户消息 - 按照豆包API标准格式
            user_content = []
            
            # 添加文字内容
            combined_text = ""
            if text_input:
                combined_text += text_input
            if audio_text:
                if combined_text:
                    combined_text += "\n\n[语音识别内容]: " + audio_text
                else:
                    combined_text = audio_text
            
            # 如果有文字内容，添加为第一个元素
            if combined_text.strip():
                user_content.append({
                    "type": "text",
                    "text": combined_text
                })
            
            # 处理图片内容（豆包支持多模态）
            if images and len(images) > 0:
                # 转换图片为豆包格式
                for i, image in enumerate(images):
                    try:
                        # 固定图片大小为640x480
                        target_width = 640
                        target_height = 480
                        quality = 85
                        
                        # 调整图片大小为固定尺寸
                        if image.width != target_width or image.height != target_height:
                            image = image.resize((target_width, target_height), Image.Resampling.LANCZOS)
                            logger.info(f"图片 {i+1} 已调整为固定大小: {target_width}x{target_height}")
                        
                        # 将 PIL 图片转换为 base64
                        buffer = io.BytesIO()
                        
                        # 检查图片模式，确保与保存格式兼容
                        if image.mode in ('RGBA', 'LA', 'P'):
                            # 如果有透明通道，转为PNG
                            image.save(buffer, format='PNG', optimize=True)
                            image_format = 'png'
                        else:
                            # 否则转为JPEG以减小文件大小
                            if image.mode != 'RGB':
                                image = image.convert('RGB')
                            image.save(buffer, format='JPEG', quality=quality, optimize=True)
                            image_format = 'jpeg'
                        
                        buffer.seek(0)
                        image_data = buffer.getvalue()
                        image_base64 = base64.b64encode(image_data).decode('utf-8')
                        
                        # 检查最终大小
                        size_mb = len(image_data) / (1024 * 1024)
                        logger.info(f"图片 {i+1} 转换完成: 格式={image_format}, 尺寸={image.width}x{image.height}, 大小={size_mb:.2f}MB")
                        
                        # 使用豆包的图片格式
                        user_content.append({
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/{image_format};base64,{image_base64}"
                            }
                        })
                        
                        logger.info(f"图片 {i+1} 已添加到豆包请求中")
                    except Exception as e:
                        logger.error(f"处理图片 {i+1} 时出错: {e}")
            
            # 如果没有任何内容，提供默认消息
            if not user_content:
                user_content = [{"type": "text", "text": "Hello"}]
            
            messages.append({"role": "user", "content": user_content})
            
            # 确定使用的模型 - 映射到正确的接入点ID
            if not model_name or 'doubao' not in model_name.lower():
                api_model = DEFAULT_DOUBAO_ENDPOINT  # 默认豆包模型
            else:
                api_model = get_doubao_endpoint(model_name)
                
                # 检查是否已配置
                if not is_doubao_endpoint_configured(model_name):
                    logger.warning(f"豆包模型 {model_name} 接入点未配置，使用默认接入点: {api_model}")
                elif api_model != model_name:
                    logger.info(f"豆包模型 {model_name} 映射到接入点: {api_model}")
            
            logger.info(f"正在调用豆包 API，模型: {api_model}")
            
            # 调试：记录发送给豆包的消息
            logger.info(f"发送给豆包的消息数量: {len(messages)}")
            logger.info(f"用户消息内容长度: {len(str(messages[-1]))}")
            
            # 根据thinking状态动态设置max_tokens
            if DOUBAO_THINKING_ENABLED:
                max_tokens = 32768  # thinking模式使用更大的token限制
                logger.info("豆包深度思考模式: 使用扩展token限制 32768")
            else:
                max_tokens = 8192   # 普通模式使用标准token限制
                logger.info("豆包普通模式: 使用标准token限制 8192")
            
            # 构建请求参数
            request_params = {
                "model": api_model,
                "messages": messages,
                "max_tokens": max_tokens,
                "temperature": 0.7,
                "timeout": 180  # 3分钟超时
            }
            
            # 添加豆包的thinking功能（可配置）
            thinking_type = "enabled" if DOUBAO_THINKING_ENABLED else "disabled"
            request_params["extra_body"] = {
                "thinking": {
                    "type": thinking_type  # 根据配置启用或禁用深度思考能力
                }
            }
            logger.info(f"豆包深度思考设置: {thinking_type}, max_tokens: {max_tokens}")
            
            # 调用豆包 API - 根据SDK类型调整
            start_time = time.time()
            
            if DOUBAO_USE_OFFICIAL_SDK:
                # 使用官方SDK格式
                request_params["extra_headers"] = DOUBAO_EXTRA_HEADERS
                # 官方SDK保持extra_body格式
                logger.info("使用豆包官方SDK调用")
                response = doubao_client.chat.completions.create(**request_params)
            else:
                # 使用OpenAI兼容模式
                logger.info("使用OpenAI兼容模式调用豆包")
                response = doubao_client.chat.completions.create(**request_params)
            
            # 记录耗时
            elapsed_time = time.time() - start_time
            logger.info(f"豆包模型 {api_model} 响应耗时: {elapsed_time:.2f}秒")
            
            if response.choices and len(response.choices) > 0:
                choice = response.choices[0]
                content = choice.message.content
                
                # 检查豆包thinking模式的特殊响应格式
                thinking_content = None
                final_content = content
                
                # 如果启用了thinking，检查是否有reasoning_content字段（豆包的thinking内容）
                if DOUBAO_THINKING_ENABLED:
                    if hasattr(choice.message, 'reasoning_content'):
                        thinking_content = choice.message.reasoning_content
                        logger.info(f"豆包深度思考内容: {thinking_content[:200] if thinking_content else 'None'}...")
                    elif hasattr(choice.message, 'thinking'):
                        thinking_content = choice.message.thinking
                        logger.info(f"豆包思考内容(thinking字段): {thinking_content[:200] if thinking_content else 'None'}...")
                
                # 记录完整响应信息
                logger.info(f"豆包 API 调用成功")
                logger.info(f"  thinking模式: {'启用' if DOUBAO_THINKING_ENABLED else '禁用'}")
                logger.info(f"  thinking内容长度: {len(thinking_content) if thinking_content else 0}")
                logger.info(f"  最终回答长度: {len(final_content) if final_content else 0}")
                logger.info(f"  最终回答预览: {final_content[:100] if final_content else 'None'}...")
                
                # 处理最终回答内容
                if not final_content and thinking_content:
                    logger.warning("豆包返回了thinking内容但没有最终回答，使用thinking内容")
                    final_content = f"[深度思考过程]\n{thinking_content}"
                elif final_content and thinking_content and DOUBAO_SHOW_THINKING_PROCESS:
                    # 如果用户选择显示思考过程，则在回答前添加
                    final_content = f"[深度思考过程]\n{thinking_content}\n\n[最终回答]\n{final_content}"
                
                # 检查是否被拒绝
                if final_content and ("无法" in final_content or "抱歉" in final_content or "不能" in final_content or "cannot" in final_content.lower() or "sorry" in final_content.lower()):
                    logger.warning(f"豆包可能拒绝了请求: {final_content[:200]}...")
                
                return final_content or "抱歉，豆包返回了空响应。"
            else:
                logger.warning("豆包 API 返回空响应")
                return "抱歉，我暂时无法生成回复。请稍后重试。"
                
        except Exception as e:
            logger.error(f"豆包 API 调用失败: {e}")
            if "timeout" in str(e).lower():
                return "抱歉，网络连接超时。请检查您的网络连接或稍后重试。"
            elif "rate_limit" in str(e).lower():
                return "抱歉，API 调用频率超限。请稍后重试。"
            elif "invalid_request" in str(e).lower():
                return "抱歉，请求格式有误。请检查输入内容。"
            else:
                return f"抱歉，处理请求时出现错误: {str(e)}"



def forward_message_to_external(text, images_info=None, user_input=None, audio_text=None):
    """转发完整消息（包括图片信息）到外部程序"""
    try:
        if not FORWARD_URL:
            return
        
        # 准备转发数据
        forward_data = {
            'text': text,
            'timestamp': datetime.now().isoformat(),
            'source': 'gemini-chat',
            'message_type': 'ai_response'
        }
        
        # 添加用户输入信息
        if user_input:
            forward_data['user_input'] = user_input
        
        # 添加语音识别文本
        if audio_text:
            forward_data['audio_text'] = audio_text
        
        # 添加图片信息
        if images_info:
            forward_data['images_info'] = images_info
        
        # 发送到外部程序（本地连接完全绕过代理）
        # 临时保存代理环境变量
        old_proxies = {}
        proxy_vars = ['HTTP_PROXY', 'HTTPS_PROXY', 'http_proxy', 'https_proxy', 'ALL_PROXY', 'all_proxy']
        for var in proxy_vars:
            if var in os.environ:
                old_proxies[var] = os.environ[var]
                del os.environ[var]
        
        try:
            forward_session = requests.Session()
            forward_session.proxies = {}
            response = forward_session.post(
                FORWARD_URL,
                json=forward_data,
                timeout=5,
                headers={'Content-Type': 'application/json'}
            )
        finally:
            # 恢复代理环境变量
            for var, value in old_proxies.items():
                os.environ[var] = value
        
        if response.status_code == 200:
            image_count = len(images_info) if images_info else 0
            logger.info(f"消息转发成功: 文字{len(text)}字符, 图片{image_count}张")
        else:
            logger.warning(f"消息转发失败: {response.status_code}")
            
    except Exception as e:
        logger.error(f"转发消息失败: {e}")

# 保持向后兼容的简单文字转发函数
def forward_text_to_external(text):
    """转发文字到外部程序（保持向后兼容）"""
    forward_message_to_external(text)

# 获取代理配置
def get_proxy_config():
    """获取代理配置"""
    proxy_url = os.environ.get('HTTPS_PROXY') or os.environ.get('HTTP_PROXY') or \
                os.environ.get('https_proxy') or os.environ.get('http_proxy')
    return proxy_url

def create_websocket_kwargs(proxy_url=None):
    """创建WebSocket连接的参数"""
    kwargs = {
        'open_timeout': 20,
        'ping_timeout': 20,
        'close_timeout': 10
    }
    
    # 如果有代理，需要特殊处理
    if proxy_url:
        logger.info(f"使用代理连接: {proxy_url}")
        # 对于Python websockets库，我们需要自己处理代理
        # 这里先记录代理信息，具体实现可能需要额外的代理库
        kwargs['proxy_url'] = proxy_url
    
    return kwargs

# WebSocket实时语音模型客户端
class RealtimeVoiceClient:
    """实时语音模型基类"""
    def __init__(self, api_key: str):
        self.api_key = api_key
        self.websocket = None
        self.session_id = None
        self.is_connected = False
        
    async def connect(self):
        """连接WebSocket"""
        raise NotImplementedError
        
    async def disconnect(self):
        """断开连接"""
        if self.websocket:
            await self.websocket.close()
            self.websocket = None
            self.is_connected = False
            
    async def send_audio(self, audio_data: bytes):
        """发送音频数据"""
        raise NotImplementedError
        
    async def receive_messages(self):
        """接收消息"""
        raise NotImplementedError

class GPTRealtimeClient(RealtimeVoiceClient):
    """GPT-4o-Realtime WebRTC客户端（基于成功项目架构）"""
    
    def __init__(self, api_key: str, use_webrtc=True):
        super().__init__(api_key)
        self.use_webrtc = use_webrtc  # 使用WebRTC方式而不是直接WebSocket
        self.model = "gpt-4o-realtime-preview"
        self.session_data = None
        
    async def connect(self):
        """连接到OpenAI Realtime API（使用WebRTC方式）"""
        try:
            if self.use_webrtc:
                # 使用WebRTC方式（参考成功项目）
                logger.info("使用 WebRTC 方式连接 OpenAI Realtime API...")
                
                # 第一步：创建 Session
                session_result = await self._create_session()
                if not session_result:
                    logger.error("Session 创建失败")
                    return False
                
                self.session_data = session_result
                self.is_connected = True
                logger.info("GPT-4o Realtime 连接成功（WebRTC Session 方式）")
                return True
            else:
                # 回退到直接 WebSocket 方式（可能不稳定）
                logger.warning("使用直接 WebSocket 方式，可能不稳定")
                return await self._connect_websocket()
                
        except Exception as e:
            logger.error(f"GPT-4o Realtime 连接失败: {e}")
            self.is_connected = False
            return False
    
    async def _create_session(self):
        """创建 OpenAI Realtime Session"""
        try:
            # 调用我们的后端创建 Session
            import requests
            response = requests.post(
                "http://localhost:1024/api/realtime/session",
                json={
                    "voice": "alloy",
                    "instructions": "你是一个有用的AI助手。请用友好、自然的语调回答用户的问题。请用中文回复。"
                },
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                if result.get('success'):
                    logger.info("OpenAI Realtime Session 创建成功")
                    return result.get('session')
                else:
                    logger.error(f"Session 创建失败: {result.get('error')}")
                    return None
            else:
                logger.error(f"Session 创建请求失败: {response.status_code}")
                return None
                
        except Exception as e:
            logger.error(f"创建 Session 时出错: {e}")
            return None
    
    async def _connect_websocket(self):
        """直接 WebSocket 连接（回退方案）"""
        try:
            # 原来的 WebSocket 连接逻辑
            uri = f"wss://api.openai.com/v1/realtime?model={self.model}"
            headers = {
                "Authorization": f"Bearer {self.api_key}",
                "OpenAI-Beta": "realtime=v1"
            }
            
            kwargs = create_websocket_kwargs()
            
            # 连接WebSocket
            self.websocket = await websockets.connect(
                uri, 
                additional_headers=headers,
                open_timeout=kwargs.get('open_timeout', 20),
                ping_timeout=kwargs.get('ping_timeout', 20),
                close_timeout=kwargs.get('close_timeout', 10)
            )
            self.is_connected = True
            
            # 发送初始化会话配置
            session_config = {
                "type": "session.update",
                "session": {
                    "modalities": ["text", "audio"],
                    "instructions": "你是一个智能助手，请用中文回复。",
                    "voice": "alloy",
                    "input_audio_format": "pcm16",
                    "output_audio_format": "pcm16",
                    "input_audio_transcription": {
                        "model": "whisper-1"
                    },
                    "turn_detection": {
                        "type": "server_vad",
                        "threshold": 0.5,
                        "prefix_padding_ms": 300,
                        "silence_duration_ms": 200
                    },
                    "tools": [],
                    "tool_choice": "auto",
                    "temperature": 0.8,
                    "max_response_output_tokens": 4096
                }
            }
            
            await self.websocket.send(json.dumps(session_config))
            logger.info("GPT-4o-Realtime 连接成功")
            return True
            
        except asyncio.TimeoutError:
            logger.error("GPT-4o-Realtime 连接超时")
            self.is_connected = False
            return False
        except InvalidStatusCode as e:
            logger.error(f"GPT-4o-Realtime HTTP错误: {e.status_code}")
            if e.status_code == 401:
                logger.error("API密钥无效或权限不足")
            elif e.status_code == 403:
                logger.error("账户没有实时API权限")
            self.is_connected = False
            return False
        except ConnectionRefusedError:
            logger.error("GPT-4o-Realtime 连接被拒绝，可能是网络问题")
            self.is_connected = False
            return False
        except Exception as e:
            logger.error(f"GPT-4o-Realtime 连接失败: {type(e).__name__}: {e}")
            self.is_connected = False
            return False
    
    async def send_audio(self, audio_data: bytes):
        """发送音频数据到GPT-4o-Realtime（WebRTC方式）"""
        if not self.is_connected:
            logger.error("GPT-4o Realtime 未连接")
            return False
            
        try:
            if self.use_webrtc:
                # WebRTC方式：音频处理需要通过前端WebRTC实现
                logger.info("WebRTC模式下音频发送需要通过前端实现")
                return True
            else:
                # 原WebSocket方式（回退）
                return await self._send_audio_websocket(audio_data)
            
        except Exception as e:
            logger.error(f"发送音频到GPT-4o-Realtime失败: {e}")
            return False
    
    async def send_text(self, text: str):
        """发送文字到GPT-4o-Realtime（WebRTC方式）"""
        if not self.is_connected:
            logger.error("GPT-4o Realtime 未连接")
            return False
            
        try:
            if self.use_webrtc:
                # WebRTC方式：文本也需要通过前端WebRTC DataChannel发送
                logger.info(f"WebRTC模式下文本发送: {text}")
                # 在实际应用中，这里应该通过WebRTC DataChannel发送
                # 目前我们只是记录，实际发送由前端处理
                return True
            else:
                # 原WebSocket方式（回退）
                return await self._send_text_websocket(text)
            
        except Exception as e:
            logger.error(f"发送文字到GPT-4o-Realtime失败: {e}")
            return False
    
    async def _send_audio_websocket(self, audio_data: bytes):
        """WebSocket方式发送音频（回退方案）"""
        if not self.websocket:
            return False
            
        try:
            # 转换为base64
            audio_base64 = base64.b64encode(audio_data).decode('utf-8')
            
            # 发送音频数据
            audio_event = {
                "type": "input_audio_buffer.append",
                "audio": audio_base64
            }
            
            await self.websocket.send(json.dumps(audio_event))
            
            # 提交音频缓冲区
            commit_event = {
                "type": "input_audio_buffer.commit"
            }
            await self.websocket.send(json.dumps(commit_event))
            
            # 创建响应
            response_event = {
                "type": "response.create",
                "response": {
                    "modalities": ["text", "audio"],
                    "instructions": "请回复用户的问题"
                }
            }
            await self.websocket.send(json.dumps(response_event))
            
            return True
            
        except Exception as e:
            logger.error(f"WebSocket发送音频失败: {e}")
            return False
    
    async def _send_text_websocket(self, text: str):
        """WebSocket方式发送文本（回退方案）"""
        if not self.websocket:
            return False
            
        try:
            # 创建对话项
            conversation_item = {
                "type": "conversation.item.create",
                "item": {
                    "type": "message",
                    "role": "user",
                    "content": [
                        {
                            "type": "input_text",
                            "text": text
                        }
                    ]
                }
            }
            await self.websocket.send(json.dumps(conversation_item))
            
            # 创建响应
            response_event = {
                "type": "response.create",
                "response": {
                    "modalities": ["text", "audio"]
                }
            }
            await self.websocket.send(json.dumps(response_event))
            
            return True
            
        except Exception as e:
            logger.error(f"WebSocket发送文本失败: {e}")
            return False

    async def listen_for_responses(self, callback=None):
        """监听WebSocket响应"""
        if not self.is_connected or not self.websocket:
            return
            
        try:
            async for message in self.websocket:
                try:
                    data = json.loads(message)
                    logger.info(f"收到GPT-4o-Realtime响应: {data.get('type', 'unknown')}")
                    
                    if callback:
                        callback(data)
                        
                except json.JSONDecodeError as e:
                    logger.error(f"解析GPT响应JSON失败: {e}")
                except Exception as e:
                    logger.error(f"处理GPT响应失败: {e}")
                    
        except Exception as e:
            logger.error(f"GPT-4o-Realtime监听失败: {e}")
            self.is_connected = False

class GLMRealtimeClient(RealtimeVoiceClient):
    """GLM-Realtime WebSocket客户端"""
    
    def __init__(self, api_key: str):
        super().__init__(api_key)
        self.base_url = "wss://open.bigmodel.cn/api/paas/v4/realtime"
        self.model = "GLM-Realtime-Flash"  # 或 GLM-Realtime-Air
        
    async def connect(self):
        """连接到智谱GLM-Realtime API"""
        try:
            headers = {
                "Authorization": f"Bearer {self.api_key}"
            }
            
            # 使用较短的超时时间进行连接测试
            self.websocket = await asyncio.wait_for(
                websockets.connect(
                    self.base_url, 
                    additional_headers=headers,
                    open_timeout=10,
                    ping_timeout=10
                ), 
                timeout=15
            )
            self.is_connected = True
            
            # 发送初始化会话配置
            session_config = {
                "event_id": f"session_{int(time.time() * 1000)}",
                "type": "session.update",
                "client_timestamp": int(time.time() * 1000),
                "session": {
                    "modalities": ["text", "audio"],
                    "instructions": "你是一个智能助手，请用中文回复。",
                    "voice": "male",  # male 或 female
                    "input_audio_format": "pcm16",
                    "output_audio_format": "pcm16",
                    "input_audio_transcription": {
                        "model": "whisper-1"
                    },
                    "turn_detection": {
                        "type": "server_vad",
                        "threshold": 0.5,
                        "prefix_padding_ms": 300,
                        "silence_duration_ms": 500
                    },
                    "tools": [],
                    "tool_choice": "auto",
                    "temperature": 0.8,
                    "max_tokens": 4096
                }
            }
            
            await self.websocket.send(json.dumps(session_config))
            logger.info("GLM-Realtime 连接成功")
            return True
            
        except asyncio.TimeoutError:
            logger.error("GLM-Realtime 连接超时")
            self.is_connected = False
            return False
        except InvalidStatusCode as e:
            logger.error(f"GLM-Realtime HTTP错误: {e.status_code}")
            if e.status_code == 401:
                logger.error("API密钥无效或权限不足")
            elif e.status_code == 403:
                logger.error("账户没有实时API权限")
            self.is_connected = False
            return False
        except ConnectionRefusedError:
            logger.error("GLM-Realtime 连接被拒绝，可能是网络问题")
            self.is_connected = False
            return False
        except Exception as e:
            logger.error(f"GLM-Realtime 连接失败: {type(e).__name__}: {e}")
            self.is_connected = False
            return False
    
    async def send_audio(self, audio_data: bytes):
        """发送音频数据到GLM-Realtime"""
        if not self.is_connected or not self.websocket:
            return False
            
        try:
            # 转换为base64
            audio_base64 = base64.b64encode(audio_data).decode('utf-8')
            
            # 发送音频数据
            audio_event = {
                "event_id": f"audio_{int(time.time() * 1000)}",
                "type": "input_audio_buffer.append",
                "client_timestamp": int(time.time() * 1000),
                "audio": audio_base64
            }
            
            await self.websocket.send(json.dumps(audio_event))
            
            # 提交音频缓冲区
            commit_event = {
                "event_id": f"commit_{int(time.time() * 1000)}",
                "type": "input_audio_buffer.commit",
                "client_timestamp": int(time.time() * 1000)
            }
            await self.websocket.send(json.dumps(commit_event))
            
            # 创建响应
            response_event = {
                "event_id": f"response_{int(time.time() * 1000)}",
                "type": "response.create",
                "client_timestamp": int(time.time() * 1000),
                "response": {
                    "modalities": ["text", "audio"]
                }
            }
            await self.websocket.send(json.dumps(response_event))
            
            return True
            
        except Exception as e:
            logger.error(f"发送音频到GLM-Realtime失败: {e}")
            return False
    
    async def send_text(self, text: str):
        """发送文字到GLM-Realtime"""
        if not self.is_connected or not self.websocket:
            return False
            
        try:
            # 创建对话项
            conversation_item = {
                "event_id": f"text_{int(time.time() * 1000)}",
                "type": "conversation.item.create",
                "client_timestamp": int(time.time() * 1000),
                "item": {
                    "type": "message",
                    "role": "user",
                    "content": [
                        {
                            "type": "input_text",
                            "text": text
                        }
                    ]
                }
            }
            await self.websocket.send(json.dumps(conversation_item))
            
            # 创建响应
            response_event = {
                "event_id": f"response_{int(time.time() * 1000)}",
                "type": "response.create",
                "client_timestamp": int(time.time() * 1000),
                "response": {
                    "modalities": ["text", "audio"]
                }
            }
            await self.websocket.send(json.dumps(response_event))
            
            return True
            
        except Exception as e:
            logger.error(f"发送文字到GLM-Realtime失败: {e}")
            return False

    async def listen_for_responses(self, callback=None):
        """监听WebSocket响应"""
        if not self.is_connected or not self.websocket:
            return
            
        try:
            async for message in self.websocket:
                try:
                    data = json.loads(message)
                    logger.info(f"收到GLM-Realtime响应: {data.get('type', 'unknown')}")
                    
                    if callback:
                        callback(data)
                        
                except json.JSONDecodeError as e:
                    logger.error(f"解析GLM响应JSON失败: {e}")
                except Exception as e:
                    logger.error(f"处理GLM响应失败: {e}")
                    
        except Exception as e:
            logger.error(f"GLM-Realtime监听失败: {e}")
            self.is_connected = False

# 配置选项
USE_LOCAL_PROXY_FOR_GPT = False  # 是否使用本地代理服务器连接GPT-4o

# 全局实时客户端实例
gpt_realtime_client = None
glm_realtime_client = None

# 存储实时响应的全局变量
realtime_responses = {
    'gpt-4o-realtime-preview': [],
    'glm-realtime': []
}

def handle_realtime_response(model_name, response_data):
    """处理实时模型响应"""
    global realtime_responses
    
    try:
        # 解析不同类型的响应
        response_type = response_data.get('type', '')
        processed_response = {
            'type': response_type,
            'timestamp': time.time(),
            'raw_data': response_data
        }
        
        # 处理文本响应
        if response_type == 'response.text.delta':
            # GLM格式的增量文本
            text_content = response_data.get('delta', '')
            processed_response['text'] = text_content
            processed_response['content_type'] = 'text_delta'
            
        elif response_type == 'response.audio.delta':
            # 音频增量数据
            audio_data = response_data.get('delta', '')
            processed_response['audio'] = audio_data
            processed_response['content_type'] = 'audio_delta'
            
        elif response_type == 'response.text.done':
            # 文本完成
            full_text = response_data.get('text', '')
            processed_response['text'] = full_text
            processed_response['content_type'] = 'text_complete'
            
        elif response_type == 'response.audio.done':
            # 音频完成
            audio_data = response_data.get('audio', '')
            processed_response['audio'] = audio_data
            processed_response['content_type'] = 'audio_complete'
            
        # OpenAI格式的响应
        elif response_type == 'response.audio_transcript.delta':
            # OpenAI音频转录增量
            transcript = response_data.get('delta', '')
            processed_response['text'] = transcript
            processed_response['content_type'] = 'transcript_delta'
            
        elif response_type == 'response.audio_transcript.done':
            # OpenAI音频转录完成
            transcript = response_data.get('transcript', '')
            processed_response['text'] = transcript
            processed_response['content_type'] = 'transcript_complete'
            
        else:
            # 其他类型的响应
            processed_response['content_type'] = 'other'
            processed_response['data'] = response_data
        
        # 添加到响应列表
        realtime_responses[model_name].append(processed_response)
        
        # 限制响应历史数量
        if len(realtime_responses[model_name]) > 50:
            realtime_responses[model_name] = realtime_responses[model_name][-30:]
            
        logger.info(f"处理{model_name}响应: {response_type} -> {processed_response.get('content_type')}")
        
    except Exception as e:
        logger.error(f"处理实时响应失败: {e}")

# 响应回调函数
def gpt_response_callback(response_data):
    handle_realtime_response('gpt-4o-realtime-preview', response_data)

def glm_response_callback(response_data):
    handle_realtime_response('glm-realtime', response_data)

# 初始化聊天处理器
chat_processor = MultimodalChat()

# 静态文件路由
@app.route('/')
def index():
    """提供主页面"""
    return send_from_directory('.', 'index.html')

@app.route('/<path:filename>')
def static_files(filename):
    """提供静态文件"""
    return send_from_directory('.', filename)

@app.route('/api/chat', methods=['POST'])
def chat():
    """主要的聊天接口"""
    try:
        data = request.get_json()
        
        text_input = data.get('text', '')
        # 支持单图片（向后兼容）和多图片
        image_data = data.get('image', None)
        images_data = data.get('images', [])
        audio_data = data.get('audio', None)
        model_name = data.get('model', current_model_name)  # 允许指定模型
        
        # 检查是否为实时语音模型，如果是则重定向到实时API
        if model_name in ['gpt-4o-realtime-preview', 'glm-realtime']:
            return jsonify({
                'success': False,
                'error': f'实时语音模型 {model_name} 需要使用WebSocket连接，请使用 /api/realtime 端点',
                'redirect': '/api/realtime',
                'model_type': 'realtime'
            }), 400
        
        # 调试日志：记录收到的请求数据
        logger.info(f"🔍 收到聊天请求 - 模型: {model_name}, 文字长度: {len(text_input) if text_input else 0}, 图片数量: {len(images_data)}, 有音频: {bool(audio_data)}")
        
        # 处理语音输入
        audio_text = None
        if audio_data:
            try:
                # 解码base64音频数据
                audio_bytes = base64.b64decode(audio_data.split(',')[1] if ',' in audio_data else audio_data)
                audio_text = chat_processor.speech_to_text(audio_bytes)
            except Exception as e:
                logger.error(f"音频处理错误: {e}")
        
        # 处理图片输入（支持多图片）
        images = []
        
        # 处理单图片（向后兼容）
        if image_data:
            image = chat_processor.process_image(image_data)
            if image:
                images.append(image)
        
        # 处理多图片
        if images_data:
            for img_data in images_data:
                image = chat_processor.process_image(img_data)
                if image:
                    images.append(image)
        
        # 限制图片数量
        if len(images) > 8:
            images = images[:8]
            logger.warning("图片数量超过8张，已截取前8张")
        
        # 生成回复（支持多图片）
        response_text = chat_processor.generate_response(
            text_input=text_input,
            images=images,  # 传递图片数组
            audio_text=audio_text,
            model_name=model_name  # 传递指定的模型
        )
        
        # 自动生成ChatGLM语音（如果不是错误消息）- 支持所有模型
        audio_base64 = None

        if USE_DOUBAO_VOICE:
            voice_client_name = "豆包"
        else:
            voice_client_name = "ChatGLM"

        if not any(keyword in response_text for keyword in ['抱歉', '错误', '失败', '无法', '不可用']):
            # 判断模型类型来确定日志信息
            if model_name and ('gpt' in model_name.lower() or 'openai' in model_name.lower()):
                logger.info(f"为{model_name}回复生成{voice_client_name}语音...")
            elif model_name and 'doubao' in model_name.lower():
                logger.info(f"为豆包模型{model_name}回复生成{voice_client_name}语音...")
            else:
                logger.info(f"为Gemini回复生成{voice_client_name}语音...")

            # 使用配置的语速生成语音
            if USE_DOUBAO_VOICE:
                audio_file = text_to_speech_doubao(response_text, appid=DOUBAO_APPID, access_token=DOUBAO_ACCESS_TOKEN, speed_ratio=DOUBAO_SPEECH_RATIO)
            else:
                audio_file = text_to_speech_chatglm(response_text, speech_rate=CHATGLM_SPEECH_RATE)
            if audio_file:
                try:
                    with open(audio_file, 'rb') as f:
                        audio_data = f.read()
                    audio_base64 = base64.b64encode(audio_data).decode('utf-8')
                    # 清理临时文件
                    os.unlink(audio_file)
                    logger.info(f"{voice_client_name}语音生成并编码完成")
                except Exception as e:
                    logger.error(f"读取音频文件失败: {e}")
            else:
                logger.warning(f"{voice_client_name}语音生成失败，前端将使用备用TTS")

        # 保存对话历史
        user_message = []
        if text_input:
            user_message.append(f"文字: {text_input}")
        if audio_text:
            user_message.append(f"语音: {audio_text}")
        if images:
            user_message.append(f"图片: [已上传{len(images)}张图片]")
        
        conversation_history.append({
            'user': '; '.join(user_message),
            'assistant': response_text
        })
        
        # 准备图片信息用于转发
        images_info = []
        if images:
            for i, image in enumerate(images):
                try:
                    # 将PIL图片转换为base64以便转发
                    buffer = io.BytesIO()
                    image.save(buffer, format='PNG')
                    buffer.seek(0)
                    image_base64 = base64.b64encode(buffer.getvalue()).decode('utf-8')
                    
                    images_info.append({
                        'index': i + 1,
                        'format': 'PNG',
                        'size': f"{image.width}x{image.height}",
                        'data': f"data:image/png;base64,{image_base64}"
                    })
                except Exception as e:
                    logger.error(f"处理图片{i+1}转发时出错: {e}")
                    images_info.append({
                        'index': i + 1,
                        'format': 'unknown',
                        'size': 'unknown',
                        'error': str(e)
                    })
        
        # 准备用户输入信息
        user_input_info = {
            'text': text_input,
            'audio_text': audio_text,
            'image_count': len(images)
        }
        
        # 转发完整消息内容到外部程序
        if FORWARD_URL:
            forward_message_to_external(
                text=response_text,
                images_info=images_info if images_info else None,
                user_input=user_input_info,
                audio_text=audio_text
            )
        
        return jsonify({
            'success': True,
            'response': {
                'text': response_text,
                'audio': audio_base64,
                'recognized_speech': audio_text
            }
        })
        
    except Exception as e:
        logger.error(f"聊天接口错误: {e}")
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500

@app.route('/api/history', methods=['GET'])
def get_history():
    """获取对话历史"""
    return jsonify({
        'success': True,
        'history': conversation_history
    })

@app.route('/api/clear', methods=['POST'])
def clear_history():
    """清除对话历史"""
    global conversation_history
    conversation_history = []
    return jsonify({
        'success': True,
        'message': '对话历史已清除'
    })

@app.route('/api/config/forward', methods=['POST'])
def set_forward_url():
    """设置转发URL"""
    global FORWARD_URL
    try:
        data = request.get_json()
        url = data.get('url', '').strip()
        
        if url:
            # 验证URL格式
            if not url.startswith(('http://', 'https://')):
                return jsonify({
                    'success': False,
                    'error': 'URL必须以http://或https://开头'
                }), 400
            
            FORWARD_URL = url
            logger.info(f"转发URL已设置: {FORWARD_URL}")
            
            # 测试连接
            try:
                # 从转发URL构造健康检查URL
                if '/api/receive' in url:
                    health_url = url.replace('/api/receive', '/api/health')
                else:
                    # 如果URL不包含/api/receive，则假设是基础URL
                    base_url = url.rstrip('/')
                    health_url = f"{base_url}/api/health"
                
                # 对于本地连接，完全绕过代理
                # 临时保存代理环境变量
                old_proxies = {}
                proxy_vars = ['HTTP_PROXY', 'HTTPS_PROXY', 'http_proxy', 'https_proxy', 'ALL_PROXY', 'all_proxy']
                for var in proxy_vars:
                    if var in os.environ:
                        old_proxies[var] = os.environ[var]
                        del os.environ[var]
                
                try:
                    test_session = requests.Session()
                    test_session.proxies = {}
                    test_response = test_session.get(health_url, timeout=5)
                finally:
                    # 恢复代理环境变量
                    for var, value in old_proxies.items():
                        os.environ[var] = value
                if test_response.status_code == 200:
                    connection_status = "连接成功"
                else:
                    connection_status = f"连接失败: HTTP {test_response.status_code}"
            except Exception as e:
                connection_status = f"无法连接: {str(e)}"
            
            return jsonify({
                'success': True,
                'message': '转发URL设置成功',
                'url': FORWARD_URL,
                'connection_status': connection_status
            })
        else:
            FORWARD_URL = None
            return jsonify({
                'success': True,
                'message': '转发功能已禁用'
            })
            
    except Exception as e:
        logger.error(f"设置转发URL失败: {e}")
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500

@app.route('/api/config/forward', methods=['GET'])
def get_forward_config():
    """获取转发配置"""
    return jsonify({
        'success': True,
        'forward_url': FORWARD_URL,
        'enabled': FORWARD_URL is not None
    })

@app.route('/api/config/model', methods=['POST'])
def set_model():
    """设置AI模型"""
    global model, current_model_name
    try:
        data = request.get_json()
        model_name = data.get('model', '').strip()
        
        if not model_name:
            return jsonify({
                'success': False,
                'error': '模型名称不能为空'
            }), 400
        
        # 支持的模型列表
        supported_models = [
            # Gemini 模型
            'gemini-2.5-flash-lite',
            'gemini-2.5-flash',
            'gemini-2.5-pro',
            'gemini-live-2.5-flash-preview',
            'gemini-2.5-flash-preview-native-audio-dialog',
            # OpenAI GPT 模型
            'gpt-4o',
            'gpt-4o-mini',
            # 豆包模型 (重新添加所有模型供用户测试)
            'doubao-seed-1-6-vision-250815',
            'doubao-lite-4k',
            'doubao-pro-4k', 
            'doubao-pro-32k',
            'doubao-pro-128k',
            # 实时语音模型
            'gpt-4o-realtime-preview',
            'glm-realtime',
            # 云服务器部署
            'vllm-local-model'
        ]
        
        if model_name not in supported_models:
            return jsonify({
                'success': False,
                'error': f'不支持的模型: {model_name}'
            }), 400
        
        # 尝试创建新模型
        try:
            # 临时清除代理设置，避免网络连接问题
            old_proxies = {}
            proxy_vars = ['HTTP_PROXY', 'HTTPS_PROXY', 'http_proxy', 'https_proxy', 'ALL_PROXY', 'all_proxy']
            for var in proxy_vars:
                if var in os.environ:
                    old_proxies[var] = os.environ[var]
                    del os.environ[var]
            
            try:
                # 判断模型类型并进行相应处理
                if 'gpt' in model_name.lower():
                    # OpenAI 模型 - 验证客户端可用性
                    if openai_client is None:
                        raise Exception("OpenAI 客户端未配置")
                    
                    # 简单验证 OpenAI 连接（不进行实际API调用）
                    logger.info(f"OpenAI 模型验证成功: {model_name}")
                    new_model = None  # OpenAI 不需要预创建模型对象
                elif 'doubao' in model_name.lower():
                    # 豆包模型 - 验证客户端可用性
                    if doubao_client is None:
                        raise Exception("豆包客户端未配置")
                    
                    # 简单验证豆包连接（不进行实际API调用）
                    logger.info(f"豆包模型验证成功: {model_name}")
                    new_model = None  # 豆包不需要预创建模型对象
                else:
                    # Gemini 模型
                    new_model = genai.GenerativeModel(model_name)
                    logger.info(f"Gemini 模型创建成功: {model_name}")
                
                # 更新全局模型配置
                model = new_model
                current_model_name = model_name
                
                logger.info(f"模型已切换到: {model_name}")
                
                return jsonify({
                    'success': True,
                    'message': f'模型已切换到: {model_name}',
                    'model': model_name
                })
                
            finally:
                # 恢复代理环境变量
                for var, value in old_proxies.items():
                    os.environ[var] = value
            
        except Exception as model_error:
            logger.error(f"模型切换失败: {model_error}")
            
            # 提供更友好的错误信息
            error_msg = str(model_error)
            if "proxy" in error_msg.lower() or "connection" in error_msg.lower():
                error_msg = "网络连接问题，请检查网络设置或稍后重试"
            elif "not found" in error_msg.lower() or "invalid" in error_msg.lower():
                error_msg = f"模型 {model_name} 不可用或无效"
            else:
                error_msg = f"模型切换失败: {error_msg}"
            
            return jsonify({
                'success': False,
                'error': error_msg
            }), 500
            
    except Exception as e:
        logger.error(f"设置模型失败: {e}")
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500

@app.route('/api/config/model', methods=['GET'])
def get_model():
    """获取当前模型"""
    return jsonify({
        'success': True,
        'model': current_model_name,
        'available_models': [
            # Gemini 模型
            'gemini-2.5-flash-lite',
            'gemini-2.5-flash',
            'gemini-2.5-pro',
            'gemini-live-2.5-flash-preview',
            'gemini-2.5-flash-preview-native-audio-dialog',
            # OpenAI GPT 模型
            'gpt-4o',
            'gpt-4o-mini',
            # 豆包模型 (重新添加所有模型供用户测试)
            'doubao-seed-1-6-vision-250815',
            'doubao-lite-4k',
            'doubao-pro-4k',
            'doubao-pro-32k', 
            'doubao-pro-128k',
            # 实时语音模型
            'gpt-4o-realtime-preview',
            'glm-realtime',
            # 云服务器部署
            'vllm-local-model'
        ]
    })

@app.route('/api/ros/command', methods=['POST'])
def send_ros_command():
    """发送指令到ROS2节点"""
    try:
        data = request.get_json()
        command = data.get('command', '').strip()
        
        if not command:
            return jsonify({
                'success': False,
                'error': '指令内容不能为空'
            }), 400
        
        # 优先使用Python ROS2发布器
        if HAS_ROS_PUBLISHER:
            try:
                success, result_msg = publish_to_ros(command)
                if success:
                    logger.info(f"ROS2指令发送成功: {command}")
                    return jsonify({
                        'success': True,
                        'message': '指令已发送到ROS2节点',
                        'command': command,
                        'method': 'python_publisher'
                    })
                else:
                    logger.warning(f"Python发布器失败，尝试命令行方式: {result_msg}")
            except Exception as e:
                logger.warning(f"Python发布器异常，尝试命令行方式: {e}")
        
        # 回退到命令行方式
        # test_simulation_ros2.py订阅了/command/content话题，需要JSON格式
        command_json = json.dumps(command)
        ros_command = [
            'ros2', 'topic', 'pub', '--once',
            '/command/content',
            'std_msgs/msg/String',
            f'{{data: {command_json}}}'
        ]
        
        logger.info(f"执行ROS2命令: {' '.join(ros_command)}")
        
        # 执行ROS2命令
        result = subprocess.run(
            ros_command,
            capture_output=True,
            text=True,
            timeout=10  # 10秒超时
        )
        
        if result.returncode == 0:
            logger.info(f"ROS2指令发送成功: {command}")
            return jsonify({
                'success': True,
                'message': '指令已发送到ROS2节点',
                'command': command,
                'method': 'command_line',
                'output': result.stdout.strip() if result.stdout else None
            })
        else:
            error_msg = result.stderr.strip() if result.stderr else '未知错误'
            logger.error(f"ROS2指令发送失败: {error_msg}")
            return jsonify({
                'success': False,
                'error': f'ROS2命令执行失败: {error_msg}'
            }), 500
            
    except subprocess.TimeoutExpired:
        return jsonify({
            'success': False,
            'error': 'ROS2命令执行超时'
        }), 500
    except FileNotFoundError:
        return jsonify({
            'success': False,
            'error': 'ROS2未安装或ros2命令不在PATH中'
        }), 500
    except Exception as e:
        logger.error(f"发送ROS2指令时出错: {e}")
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500

@app.route('/api/test', methods=['GET'])
def test():
    """测试接口"""
    # 检查API连接状态
    api_status = "可用" if model is not None else "不可用"
    
    # 尝试简单的API调用来验证连接
    connection_test = False
    try:
        if model is not None:
            # 发送一个简单的测试请求
            test_response = model.generate_content(
                "请回复'连接成功'",
                generation_config=genai.GenerationConfig(max_output_tokens=10),
                request_options={'timeout': 10}
            )
            if test_response and test_response.text:
                connection_test = True
    except Exception as e:
        logger.warning(f"API连接测试失败: {e}")
    
    # 检查ROS2环境
    ros2_available = False
    ros2_status = "不可用"
    try:
        if HAS_ROS_PUBLISHER:
            available, msg = check_ros2_available()
            ros2_available = available
            ros2_status = msg
        else:
            # 回退到命令行检查
            ros2_check = subprocess.run(['ros2', '--version'], capture_output=True, text=True, timeout=5)
            ros2_available = ros2_check.returncode == 0
            ros2_status = "命令行可用" if ros2_available else "未安装"
    except Exception as e:
        ros2_status = f"检查失败: {e}"
    
    return jsonify({
        'success': True,
        'message': 'Gemini多模态聊天服务正在运行',
        'model': current_model_name,
        'api_status': api_status,
        'connection_test': "通过" if connection_test else "失败",
        'forward_url': FORWARD_URL,
        'ros2_available': ros2_available,
        'ros2_status': ros2_status,
        'ros2_publisher_available': HAS_ROS_PUBLISHER,
        'features': {
            'multi_image_support': True,
            'model_switching': True,
            'max_images': 8
        },
        'network_info': {
            'proxy_configured': bool(os.environ.get('HTTP_PROXY') or os.environ.get('HTTPS_PROXY')),
            'endpoint': 'https://generativelanguage.googleapis.com'
        }
    })

# ChatGLM语音生成API端点
@app.route('/api/chatglm-tts', methods=['POST'])
def chatglm_text_to_speech():
    """ChatGLM文字转语音API"""
    try:
        data = request.json
        text = data.get('text', '').strip()
        
        if not text:
            return jsonify({
                'success': False,
                'error': '文本内容不能为空'
            }), 400
        
        if not chatglm_client:
            return jsonify({
                'success': False,
                'error': 'ChatGLM语音服务不可用，请检查配置'
            }), 503
        
        # 直接调用ChatGLM语音生成函数，使用配置的语速
        audio_file = text_to_speech_doubao(text, appid=DOUBAO_APPID, access_token=DOUBAO_ACCESS_TOKEN, speed_ratio=DOUBAO_SPEECH_RATIO)
        # audio_file = text_to_speech_chatglm(text, speech_rate=CHATGLM_SPEECH_RATE)
        
        if audio_file:
            # 读取音频文件并转换为base64
            with open(audio_file, 'rb') as f:
                audio_data = f.read()
            
            audio_base64 = base64.b64encode(audio_data).decode('utf-8')
            
            # 清理临时文件
            try:
                os.unlink(audio_file)
            except:
                pass
            
            return jsonify({
                'success': True,
                'message': 'ChatGLM语音生成成功',
                'audio_data': audio_base64,
                'audio_format': 'wav',
                'provider': 'ChatGLM GLM-4-Voice'
            })
        else:
            return jsonify({
                'success': False,
                'error': 'ChatGLM语音生成失败'
            }), 500
            
    except Exception as e:
        logger.error(f"ChatGLM语音生成API错误: {e}")
        return jsonify({
            'success': False,
            'error': f'服务器错误: {str(e)}'
        }), 500

# Gemini模型配置API端点
@app.route('/api/gemini-config', methods=['GET', 'POST'])
def gemini_model_config():
    """获取或设置Gemini模型配置"""
    global DISABLE_THINKING_MODE, PRO_MODEL_TIMEOUT, DEFAULT_MODEL_TIMEOUT
    
    if request.method == 'GET':
        # 获取当前配置
        return jsonify({
            'success': True,
            'config': {
                'disable_thinking_mode': DISABLE_THINKING_MODE,
                'pro_model_timeout': PRO_MODEL_TIMEOUT,
                'default_model_timeout': DEFAULT_MODEL_TIMEOUT
            },
            'description': {
                'disable_thinking_mode': '是否关闭思考模式（仅Flash模型支持）',
                'pro_model_timeout': 'Pro模型超时时间（秒）',
                'default_model_timeout': '其他模型超时时间（秒）'
            }
        })
    
    elif request.method == 'POST':
        # 设置配置
        try:
            data = request.json
            updated = []
            
            if 'disable_thinking_mode' in data:
                DISABLE_THINKING_MODE = bool(data['disable_thinking_mode'])
                updated.append(f"思考模式关闭: {DISABLE_THINKING_MODE}")
            
            if 'pro_model_timeout' in data:
                new_timeout = int(data['pro_model_timeout'])
                if 30 <= new_timeout <= 300:  # 限制在30-300秒之间
                    PRO_MODEL_TIMEOUT = new_timeout
                    updated.append(f"Pro模型超时: {PRO_MODEL_TIMEOUT}秒")
                else:
                    return jsonify({
                        'success': False,
                        'error': 'Pro模型超时时间必须在30-300秒之间'
                    }), 400
            
            if 'default_model_timeout' in data:
                new_timeout = int(data['default_model_timeout'])
                if 15 <= new_timeout <= 180:  # 限制在15-180秒之间
                    DEFAULT_MODEL_TIMEOUT = new_timeout
                    updated.append(f"默认超时: {DEFAULT_MODEL_TIMEOUT}秒")
                else:
                    return jsonify({
                        'success': False,
                        'error': '默认超时时间必须在15-180秒之间'
                    }), 400
            
            if updated:
                logger.info(f"Gemini配置已更新: {', '.join(updated)}")
                return jsonify({
                    'success': True,
                    'message': f'配置已更新: {", ".join(updated)}',
                    'config': {
                        'disable_thinking_mode': DISABLE_THINKING_MODE,
                        'pro_model_timeout': PRO_MODEL_TIMEOUT,
                        'default_model_timeout': DEFAULT_MODEL_TIMEOUT
                    }
                })
            else:
                return jsonify({
                    'success': False,
                    'error': '未提供有效的配置参数'
                }), 400
                
        except (ValueError, TypeError) as e:
            return jsonify({
                'success': False,
                'error': f'无效的参数值: {e}'
            }), 400
        except Exception as e:
            logger.error(f"设置Gemini配置失败: {e}")
            return jsonify({
                'success': False,
                'error': f'设置配置失败: {e}'
            }), 500

# 豆包深度思考配置API端点
@app.route('/api/doubao-thinking', methods=['GET', 'POST'])
def doubao_thinking_config():
    """获取或设置豆包深度思考配置"""
    global DOUBAO_THINKING_ENABLED
    
    if request.method == 'GET':
        # 获取当前深度思考配置
        return jsonify({
            'success': True,
            'thinking_enabled': DOUBAO_THINKING_ENABLED,
            'description': '豆包深度思考功能可以提供更深入和周全的回答，但可能会稍微增加响应时间'
        })
    
    elif request.method == 'POST':
        # 设置深度思考
        try:
            data = request.json
            new_enabled = bool(data.get('thinking_enabled', DOUBAO_THINKING_ENABLED))
            
            DOUBAO_THINKING_ENABLED = new_enabled
            status = "启用" if new_enabled else "禁用"
            logger.info(f"豆包深度思考已{status}")
            
            return jsonify({
                'success': True,
                'message': f'豆包深度思考已{status}',
                'thinking_enabled': DOUBAO_THINKING_ENABLED
            })
            
        except Exception as e:
            logger.error(f"设置豆包深度思考失败: {e}")
            return jsonify({
                'success': False,
                'error': f'设置失败: {e}'
            }), 500

# ChatGLM语速配置API端点
@app.route('/api/chatglm-speech-rate', methods=['GET', 'POST'])
def chatglm_speech_rate_config():
    """获取或设置ChatGLM语速配置"""
    global CHATGLM_SPEECH_RATE
    
    if request.method == 'GET':
        # 获取当前语速配置
        return jsonify({
            'success': True,
            'speech_rate': CHATGLM_SPEECH_RATE,
            'description': {
                '0.5': '慢速',
                '0.8': '稍慢',
                '1.0': '正常',
                '1.2': '稍快',
                '1.5': '快速'
            }
        })
    
    elif request.method == 'POST':
        # 设置语速
        try:
            data = request.json
            new_rate = float(data.get('speech_rate', CHATGLM_SPEECH_RATE))
            
            # 限制语速范围
            if not (0.3 <= new_rate <= 2.0):
                return jsonify({
                    'success': False,
                    'error': '语速值必须在0.3到2.0之间'
                }), 400
            
            CHATGLM_SPEECH_RATE = new_rate
            logger.info(f"ChatGLM语速已设置为: {new_rate}x")
            
            return jsonify({
                'success': True,
                'message': f'语速已设置为: {new_rate}x',
                'speech_rate': CHATGLM_SPEECH_RATE
            })
            
        except (ValueError, TypeError) as e:
            return jsonify({
                'success': False,
                'error': f'无效的语速值: {e}'
            }), 400
        except Exception as e:
            logger.error(f"设置语速失败: {e}")
            return jsonify({
                'success': False,
                'error': f'设置语速失败: {e}'
            }), 500

# 实时语音模型API端点
@app.route('/api/realtime/connect', methods=['POST'])
def realtime_connect():
    """连接实时语音模型"""
    global gpt_realtime_client, glm_realtime_client
    
    try:
        data = request.get_json()
        model_name = data.get('model', '')
        
        if model_name not in ['gpt-4o-realtime-preview', 'glm-realtime']:
            return jsonify({
                'success': False,
                'error': f'不支持的实时模型: {model_name}'
            }), 400
        
        # 初始化对应的客户端
        if model_name == 'gpt-4o-realtime-preview':
            if not gpt_realtime_client:
                gpt_realtime_client = GPTRealtimeClient(OPENAI_API_KEY, use_webrtc=True)
            
            # 连接(需要在异步环境中运行)
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            try:
                success = loop.run_until_complete(gpt_realtime_client.connect())
                if success:
                    # 启动响应监听线程
                    def start_listener():
                        listener_loop = asyncio.new_event_loop()
                        asyncio.set_event_loop(listener_loop)
                        listener_loop.run_until_complete(
                            gpt_realtime_client.listen_for_responses(gpt_response_callback)
                        )
                        listener_loop.close()
                    
                    listener_thread = threading.Thread(target=start_listener, daemon=True)
                    listener_thread.start()
                    logger.info(f"{model_name} 响应监听器已启动")
                    
                    return jsonify({
                        'success': True,
                        'message': f'{model_name} 连接成功，响应监听器已启动',
                        'model': model_name,
                        'session_id': gpt_realtime_client.session_id
                    })
                else:
                    return jsonify({
                        'success': False,
                        'error': f'{model_name} 连接失败'
                    }), 500
            finally:
                loop.close()
                
        elif model_name == 'glm-realtime':
            if not glm_realtime_client:
                glm_realtime_client = GLMRealtimeClient(CHATGLM_API_KEY)
            
            # 连接(需要在异步环境中运行)
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            try:
                success = loop.run_until_complete(glm_realtime_client.connect())
                if success:
                    # 启动响应监听线程
                    def start_listener():
                        listener_loop = asyncio.new_event_loop()
                        asyncio.set_event_loop(listener_loop)
                        listener_loop.run_until_complete(
                            glm_realtime_client.listen_for_responses(glm_response_callback)
                        )
                        listener_loop.close()
                    
                    listener_thread = threading.Thread(target=start_listener, daemon=True)
                    listener_thread.start()
                    logger.info(f"{model_name} 响应监听器已启动")
                    
                    return jsonify({
                        'success': True,
                        'message': f'{model_name} 连接成功，响应监听器已启动',
                        'model': model_name,
                        'session_id': glm_realtime_client.session_id
                    })
                else:
                    return jsonify({
                        'success': False,
                        'error': f'{model_name} 连接失败'
                    }), 500
            finally:
                loop.close()
                
    except Exception as e:
        logger.error(f"连接实时模型失败: {e}")
        return jsonify({
            'success': False,
            'error': f'连接失败: {str(e)}'
        }), 500

@app.route('/api/realtime/disconnect', methods=['POST'])
def realtime_disconnect():
    """断开实时语音模型连接"""
    global gpt_realtime_client, glm_realtime_client
    
    try:
        data = request.get_json()
        model_name = data.get('model', '')
        
        if model_name == 'gpt-4o-realtime-preview' and gpt_realtime_client:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(gpt_realtime_client.disconnect())
                gpt_realtime_client = None
            finally:
                loop.close()
                
        elif model_name == 'glm-realtime' and glm_realtime_client:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(glm_realtime_client.disconnect())
                glm_realtime_client = None
            finally:
                loop.close()
        
        return jsonify({
            'success': True,
            'message': f'{model_name} 连接已断开'
        })
        
    except Exception as e:
        logger.error(f"断开实时模型连接失败: {e}")
        return jsonify({
            'success': False,
            'error': f'断开连接失败: {str(e)}'
        }), 500

@app.route('/api/realtime/send', methods=['POST'])
def realtime_send():
    """发送消息到实时语音模型"""
    global gpt_realtime_client, glm_realtime_client
    
    try:
        data = request.get_json()
        model_name = data.get('model', '')
        text_input = data.get('text', '')
        audio_data = data.get('audio', None)
        
        if model_name not in ['gpt-4o-realtime-preview', 'glm-realtime']:
            return jsonify({
                'success': False,
                'error': f'不支持的实时模型: {model_name}'
            }), 400
        
        # 准备发送消息
        client = None
        if model_name == 'gpt-4o-realtime-preview':
            client = gpt_realtime_client
        elif model_name == 'glm-realtime':
            client = glm_realtime_client
        
        if not client or not client.is_connected:
            return jsonify({
                'success': False,
                'error': f'{model_name} 未连接，请先连接'
            }), 400
        
        # 发送消息
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            success = False
            
            # 发送音频
            if audio_data:
                # 解码base64音频数据
                try:
                    audio_bytes = base64.b64decode(audio_data.split(',')[1] if ',' in audio_data else audio_data)
                    success = loop.run_until_complete(client.send_audio(audio_bytes))
                except Exception as e:
                    logger.error(f"音频数据处理失败: {e}")
                    return jsonify({
                        'success': False,
                        'error': f'音频数据处理失败: {str(e)}'
                    }), 400
            
            # 发送文字
            elif text_input:
                success = loop.run_until_complete(client.send_text(text_input))
            
            if success:
                return jsonify({
                    'success': True,
                    'message': '消息发送成功',
                    'model': model_name,
                    'note': '请通过WebSocket监听响应'
                })
            else:
                return jsonify({
                    'success': False,
                    'error': '消息发送失败'
                }), 500
                
        finally:
            loop.close()
            
    except Exception as e:
        logger.error(f"发送实时消息失败: {e}")
        return jsonify({
            'success': False,
            'error': f'发送失败: {str(e)}'
        }), 500

@app.route('/api/realtime/status', methods=['GET'])
def realtime_status():
    """获取实时语音模型连接状态"""
    global gpt_realtime_client, glm_realtime_client
    
    gpt_status = {
        'connected': gpt_realtime_client.is_connected if gpt_realtime_client else False,
        'session_id': gpt_realtime_client.session_id if gpt_realtime_client else None
    }
    
    glm_status = {
        'connected': glm_realtime_client.is_connected if glm_realtime_client else False,
        'session_id': glm_realtime_client.session_id if glm_realtime_client else None
    }
    
    return jsonify({
        'success': True,
        'realtime_models': {
            'gpt-4o-realtime-preview': gpt_status,
            'glm-realtime': glm_status
        },
        'supported_models': ['gpt-4o-realtime-preview', 'glm-realtime'],
        'note': '实时模型需要WebSocket连接，支持双向音频流'
    })

@app.route('/api/realtime/responses', methods=['GET'])
def get_realtime_responses():
    """获取实时模型的响应"""
    global realtime_responses
    
    try:
        model_name = request.args.get('model', '')
        since_timestamp = float(request.args.get('since', 0))
        
        if model_name not in realtime_responses:
            return jsonify({
                'success': False,
                'error': f'不支持的模型: {model_name}'
            }), 400
        
        # 获取指定时间戳之后的响应
        responses = [
            resp for resp in realtime_responses[model_name] 
            if resp['timestamp'] > since_timestamp
        ]
        
        return jsonify({
            'success': True,
            'model': model_name,
            'responses': responses,
            'total_count': len(realtime_responses[model_name]),
            'new_count': len(responses),
            'latest_timestamp': responses[-1]['timestamp'] if responses else since_timestamp
        })
        
    except Exception as e:
        logger.error(f"获取实时响应失败: {e}")
        return jsonify({
            'success': False,
            'error': f'获取响应失败: {str(e)}'
        }), 500

@app.route('/api/realtime/responses/clear', methods=['POST'])
def clear_realtime_responses():
    """清除实时响应历史"""
    global realtime_responses
    
    try:
        data = request.get_json()
        model_name = data.get('model', '') if data else ''
        
        if model_name and model_name in realtime_responses:
            # 清除指定模型的响应
            realtime_responses[model_name] = []
            return jsonify({
                'success': True,
                'message': f'{model_name} 响应历史已清除'
            })
        elif not model_name:
            # 清除所有响应
            for key in realtime_responses:
                realtime_responses[key] = []
            return jsonify({
                'success': True,
                'message': '所有实时响应历史已清除'
            })
        else:
            return jsonify({
                'success': False,
                'error': f'不支持的模型: {model_name}'
            }), 400
            
    except Exception as e:
        logger.error(f"清除实时响应失败: {e}")
        return jsonify({
            'success': False,
            'error': f'清除失败: {str(e)}'
        }), 500

@app.route('/api/realtime/proxy-config', methods=['GET', 'POST'])
def realtime_proxy_config():
    """配置实时模型代理设置"""
    global USE_LOCAL_PROXY_FOR_GPT, gpt_realtime_client
    
    if request.method == 'GET':
        # 获取当前代理配置
        proxy_url = get_proxy_config()
        return jsonify({
            'success': True,
            'config': {
                'use_local_proxy_for_gpt': USE_LOCAL_PROXY_FOR_GPT,
                'system_proxy': proxy_url,
                'local_proxy_url': 'ws://localhost:8765/realtime-proxy'
            },
            'status': {
                'has_system_proxy': bool(proxy_url),
                'gpt_client_connected': gpt_realtime_client.is_connected if gpt_realtime_client else False
            },
            'instructions': {
                'start_proxy_server': 'python3 websocket_proxy_server.py',
                'proxy_server_port': 8765
            }
        })
    
    elif request.method == 'POST':
        # 设置代理配置
        try:
            data = request.get_json()
            new_use_proxy = bool(data.get('use_local_proxy_for_gpt', USE_LOCAL_PROXY_FOR_GPT))
            
            if new_use_proxy != USE_LOCAL_PROXY_FOR_GPT:
                USE_LOCAL_PROXY_FOR_GPT = new_use_proxy
                
                # 如果 GPT 客户端正在连接，需要重新创建
                if gpt_realtime_client and gpt_realtime_client.is_connected:
                    logger.warning("GPT代理配置已更改，当前连接将在下次连接时生效")
                
                # 重置客户端以使用新配置
                gpt_realtime_client = None
                
                proxy_mode = "本地代理服务器" if USE_LOCAL_PROXY_FOR_GPT else "直接连接"
                logger.info(f"GPT-4o 连接模式已切换为: {proxy_mode}")
                
                return jsonify({
                    'success': True,
                    'message': f'GPT-4o 连接模式已切换为: {proxy_mode}',
                    'config': {
                        'use_local_proxy_for_gpt': USE_LOCAL_PROXY_FOR_GPT
                    },
                    'note': '下次连接时生效'
                })
            else:
                return jsonify({
                    'success': True,
                    'message': '配置未更改',
                    'config': {
                        'use_local_proxy_for_gpt': USE_LOCAL_PROXY_FOR_GPT
                    }
                })
                
        except Exception as e:
            logger.error(f"设置代理配置失败: {e}")
            return jsonify({
                'success': False,
                'error': f'设置失败: {str(e)}'
            }), 500

# OpenAI Realtime Session API 端点（参考成功项目架构）
@app.route('/api/realtime/session', methods=['POST'])
def create_realtime_session():
    """创建 OpenAI Realtime Session（WebRTC方式）"""
    try:
        data = request.get_json()
        voice = data.get('voice', 'alloy')
        instructions = data.get('instructions', '你是一个有用的AI助手。请用友好、自然的语调回答用户的问题。请用中文回复。')
        
        # 支持的语音列表
        supported_voices = ["nova", "shimmer", "echo", "onyx", "fable", "alloy"]
        valid_voice = voice if voice in supported_voices else "alloy"
        
        # 创建 Session 的请求数据
        session_data = {
            "model": "gpt-4o-realtime-preview",
            "voice": valid_voice,
            "instructions": instructions
        }
        
        # 获取代理配置
        proxy_url = get_proxy_config()
        headers = {
            "Authorization": f"Bearer {OPENAI_API_KEY}",
            "Content-Type": "application/json",
            "OpenAI-Beta": "realtime=v1"
        }
        
        # 配置请求选项
        request_kwargs = {
            'json': session_data,
            'headers': headers,
            'timeout': 30
        }
        
        # 如果有代理，使用代理
        if proxy_url:
            logger.info(f"使用代理创建 Realtime Session: {proxy_url}")
            request_kwargs['proxies'] = {
                'http': proxy_url,
                'https': proxy_url
            }
        
        logger.info("正在创建 OpenAI Realtime Session...")
        
        # 调用 OpenAI Session API
        response = requests.post(
            "https://api.openai.com/v1/realtime/sessions",
            **request_kwargs
        )
        
        if response.status_code == 200:
            session_result = response.json()
            logger.info("OpenAI Realtime Session 创建成功")
            
            return jsonify({
                'success': True,
                'session': session_result,
                'message': 'Session 创建成功'
            })
        else:
            error_text = response.text
            logger.error(f"Session 创建失败: {response.status_code} {error_text}")
            
            return jsonify({
                'success': False,
                'error': f'Session 创建失败: {response.status_code} {error_text}'
            }), response.status_code
            
    except requests.exceptions.Timeout:
        logger.error("Session 创建超时")
        return jsonify({
            'success': False,
            'error': 'Session 创建超时，请检查网络连接'
        }), 408
    except requests.exceptions.ProxyError:
        logger.error("代理连接失败")
        return jsonify({
            'success': False,
            'error': '代理连接失败，请检查代理设置'
        }), 502
    except Exception as e:
        logger.error(f"创建 Session 时出错: {e}")
        return jsonify({
            'success': False,
            'error': f'服务器错误: {str(e)}'
        }), 500

@app.route('/api/realtime/sdp', methods=['POST'])
def proxy_realtime_sdp():
    """代理 WebRTC SDP 交换（参考成功项目）"""
    try:
        data = request.get_json()
        sdp = data.get('sdp')
        ephemeral_key = data.get('ephemeral_key')
        
        if not sdp or not ephemeral_key:
            return jsonify({
                'success': False,
                'error': '缺少 SDP 或 ephemeral key'
            }), 400
        
        # 获取代理配置
        proxy_url = get_proxy_config()
        headers = {
            "Authorization": f"Bearer {ephemeral_key}",
            "Content-Type": "application/sdp",
            "OpenAI-Beta": "realtime=v1"
        }
        
        # 配置请求选项
        request_kwargs = {
            'data': sdp,
            'headers': headers,
            'timeout': 30
        }
        
        # 如果有代理，使用代理
        if proxy_url:
            logger.info(f"使用代理进行 SDP 交换: {proxy_url}")
            request_kwargs['proxies'] = {
                'http': proxy_url,
                'https': proxy_url
            }
        
        logger.info("正在进行 WebRTC SDP 交换...")
        
        # 调用 OpenAI Realtime SDP 端点
        response = requests.post(
            "https://api.openai.com/v1/realtime?model=gpt-4o-realtime-preview",
            **request_kwargs
        )
        
        if response.status_code == 200:
            answer_sdp = response.text
            logger.info("SDP 交换成功")
            
            return answer_sdp, 200, {'Content-Type': 'application/sdp'}
        else:
            error_text = response.text
            logger.error(f"SDP 交换失败: {response.status_code} {error_text}")
            
            return jsonify({
                'success': False,
                'error': f'SDP 交换失败: {response.status_code} {error_text}'
            }), response.status_code
            
    except Exception as e:
        logger.error(f"SDP 代理时出错: {e}")
        return jsonify({
            'success': False,
            'error': f'SDP 代理错误: {str(e)}'
        }), 500

# 端口配置
MAIN_SERVICE_PORT = 1024  # 主服务端口

if __name__ == '__main__':
    logger.info("启动 Gemini 多模态聊天服务...")
    logger.info(f"主服务端口: {MAIN_SERVICE_PORT}")
    logger.info(f"默认转发端口: {DEFAULT_FORWARD_PORT}")
    logger.info(f"转发URL: {FORWARD_URL}")
    app.run(debug=True, host='0.0.0.0', port=MAIN_SERVICE_PORT)
