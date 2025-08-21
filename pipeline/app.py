#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import io
import base64
import json
import tempfile
from datetime import datetime
from flask import Flask, request, jsonify, send_file
from flask_cors import CORS
import google.generativeai as genai
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

# 配置 Gemini API
GEMINI_API_KEY = "AIzaSyBYcfcuuwWFbKCCGCdTYIzw8Oib8kOaLM0"

# 配置 API 客户端，增加超时设置
try:
    genai.configure(
        api_key=GEMINI_API_KEY,
        transport='rest',  # 使用REST传输而不是gRPC
        client_options={
            'api_endpoint': 'https://generativelanguage.googleapis.com',
        }
    )
    
    # 使用 Gemini 2.5 Flash-Lite 模型
    model = genai.GenerativeModel('gemini-2.5-flash-lite')
    logger.info("Gemini API 配置成功")
    
except Exception as e:
    logger.error(f"Gemini API 配置失败: {e}")
    model = None

# 创建上传目录
UPLOAD_FOLDER = './uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

# 对话历史
conversation_history = []

# 转发配置
FORWARD_URL = None  # 外部程序接收地址

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
    
    def text_to_speech(self, text, lang='zh'):
        """将文字转换为语音"""
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
    
    def generate_response(self, text_input=None, image=None, audio_text=None):
        """生成多模态响应"""
        if model is None:
            return "抱歉，AI服务暂时不可用，请检查网络连接或API配置。"
            
        try:
            # 构建提示信息
            prompt_parts = []
            
            # 系统指令
            system_instruction = "你是一个智能助手，能够理解和处理文字、图片和语音输入。请用简洁而友好的中文回复用户。"
            prompt_parts.append(system_instruction)
            
            # 添加对话历史（最近的5轮）
            if conversation_history:
                prompt_parts.append("对话历史:")
                for entry in conversation_history[-5:]:
                    prompt_parts.append(f"用户: {entry['user']}")
                    prompt_parts.append(f"助手: {entry['assistant']}")
            
            prompt_parts.append("当前对话:")
            
            # 处理用户输入
            user_input_parts = []
            if text_input:
                user_input_parts.append(f"文字输入: {text_input}")
            if audio_text:
                user_input_parts.append(f"语音输入: {audio_text}")
            if image:
                user_input_parts.append("用户还上传了一张图片，请分析图片内容。")
            
            # 构建完整提示
            full_prompt = "\n".join(prompt_parts + user_input_parts)
            
            # 准备模型输入
            content = [full_prompt]
            if image:
                content.append(image)
            
            logger.info("正在调用 Gemini API...")
            
            # 使用重试机制调用 Gemini API
            max_retries = 3
            for attempt in range(max_retries):
                try:
                    # 配置生成参数，减少超时时间
                    generation_config = genai.GenerationConfig(
                        temperature=0.7,
                        top_p=0.8,
                        top_k=40,
                        max_output_tokens=1024,
                    )
                    
                    # 调用 API，设置较短的超时时间
                    response = model.generate_content(
                        content,
                        generation_config=generation_config,
                        request_options={'timeout': 30}  # 30秒超时
                    )
                    
                    if response and response.text:
                        logger.info("API调用成功")
                        return response.text
                    else:
                        logger.warning("API返回空响应")
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



def forward_text_to_external(text):
    """转发文字到外部程序"""
    try:
        if not FORWARD_URL:
            return
        
        # 准备转发数据
        forward_data = {
            'text': text,
            'timestamp': datetime.now().isoformat(),
            'source': 'gemini-chat'
        }
        
        # 发送到外部程序
        response = requests.post(
            FORWARD_URL,
            json=forward_data,
            timeout=5,
            headers={'Content-Type': 'application/json'}
        )
        
        if response.status_code == 200:
            logger.info(f"文字转发成功: {text[:50]}...")
        else:
            logger.warning(f"文字转发失败: {response.status_code}")
            
    except Exception as e:
        logger.error(f"转发文字失败: {e}")

# 初始化聊天处理器
chat_processor = MultimodalChat()

@app.route('/api/chat', methods=['POST'])
def chat():
    """主要的聊天接口"""
    try:
        data = request.get_json()
        
        text_input = data.get('text', '')
        image_data = data.get('image', None)
        audio_data = data.get('audio', None)
        
        # 处理语音输入
        audio_text = None
        if audio_data:
            try:
                # 解码base64音频数据
                audio_bytes = base64.b64decode(audio_data.split(',')[1] if ',' in audio_data else audio_data)
                audio_text = chat_processor.speech_to_text(audio_bytes)
            except Exception as e:
                logger.error(f"音频处理错误: {e}")
        
        # 处理图片输入
        image = None
        if image_data:
            image = chat_processor.process_image(image_data)
        
        # 生成回复
        response_text = chat_processor.generate_response(
            text_input=text_input,
            image=image,
            audio_text=audio_text
        )
        
        # 生成语音回复
        audio_file_path = chat_processor.text_to_speech(response_text)
        
        # 读取音频文件并转换为base64
        audio_base64 = None
        if audio_file_path:
            try:
                with open(audio_file_path, 'rb') as f:
                    audio_bytes = f.read()
                    audio_base64 = base64.b64encode(audio_bytes).decode('utf-8')
                os.unlink(audio_file_path)  # 清理临时文件
            except Exception as e:
                logger.error(f"音频文件处理错误: {e}")
        
        # 保存对话历史
        user_message = []
        if text_input:
            user_message.append(f"文字: {text_input}")
        if audio_text:
            user_message.append(f"语音: {audio_text}")
        if image:
            user_message.append("图片: [已上传图片]")
        
        conversation_history.append({
            'user': '; '.join(user_message),
            'assistant': response_text
        })
        
        # 转发文字内容到外部程序
        if FORWARD_URL:
            forward_text_to_external(response_text)
        
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
                test_response = requests.get(f"{url.rsplit('/', 1)[0]}/api/health", timeout=5)
                if test_response.status_code == 200:
                    connection_status = "连接成功"
                else:
                    connection_status = f"连接失败: HTTP {test_response.status_code}"
            except:
                connection_status = "无法连接"
            
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
    
    return jsonify({
        'success': True,
        'message': 'Gemini多模态聊天服务正在运行',
        'model': 'gemini-2.5-flash-lite',
        'api_status': api_status,
        'connection_test': "通过" if connection_test else "失败",
        'forward_url': FORWARD_URL,
        'network_info': {
            'proxy_configured': bool(os.environ.get('HTTP_PROXY') or os.environ.get('HTTPS_PROXY')),
            'endpoint': 'https://generativelanguage.googleapis.com'
        }
    })

if __name__ == '__main__':
    logger.info("启动 Gemini 多模态聊天服务...")
    app.run(debug=True, host='0.0.0.0', port=1024)
