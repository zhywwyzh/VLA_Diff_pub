#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import io
import base64
import json
import tempfile
from flask import Flask, request, jsonify, send_file
from flask_cors import CORS
import google.generativeai as genai
from PIL import Image
import speech_recognition as sr
from gtts import gTTS
from pydub import AudioSegment
import logging
import wave
from key import GEMINI_API_KEY

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)
CORS(app)

# 配置 Gemini API
genai.configure(api_key=GEMINI_API_KEY)

# 使用 Gemini 2.5 Flash-Lite 模型
model = genai.GenerativeModel('gemini-2.5-flash-lite')

# 创建上传目录
UPLOAD_FOLDER = './uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

# 对话历史
conversation_history = []

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
            
            # 尝试使用pydub处理音频格式
            try:
                audio_segment = AudioSegment.from_file(tmp_file_path)
                # 转换为适合语音识别的格式
                audio_segment = audio_segment.set_frame_rate(16000).set_channels(1)
                
                # 保存为新的wav文件
                with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as processed_file:
                    audio_segment.export(processed_file.name, format="wav")
                    processed_path = processed_file.name
                
                # 使用SpeechRecognition转换语音
                with sr.AudioFile(processed_path) as source:
                    audio = self.recognizer.record(source)
                    # 尝试多种语言识别
                    try:
                        text = self.recognizer.recognize_google(audio, language='zh-CN')
                    except:
                        text = self.recognizer.recognize_google(audio, language='en-US')
                
                # 清理临时文件
                os.unlink(processed_path)
                    
            except Exception as process_error:
                logger.warning(f"音频预处理失败，尝试直接识别: {process_error}")
                # 直接尝试识别原始音频
                with sr.AudioFile(tmp_file_path) as source:
                    audio = self.recognizer.record(source)
                    text = self.recognizer.recognize_google(audio, language='zh-CN')
                
            # 清理原始临时文件
            os.unlink(tmp_file_path)
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
            
            # 调用 Gemini API
            response = model.generate_content(content)
            
            if response.text:
                return response.text
            else:
                return "抱歉，我无法生成回复。"
                
        except Exception as e:
            logger.error(f"生成回复错误: {e}")
            return f"抱歉，处理请求时出现错误: {str(e)}"

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

@app.route('/api/test', methods=['GET'])
def test():
    """测试接口"""
    return jsonify({
        'success': True,
        'message': 'Gemini多模态聊天服务正在运行',
        'model': 'gemini-2.5-flash-lite'
    })

if __name__ == '__main__':
    logger.info("启动 Gemini 多模态聊天服务...")
    app.run(debug=True, host='0.0.0.0', port=8080)
