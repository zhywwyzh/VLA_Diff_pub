#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ChatGLM语音功能测试脚本
测试GLM-4-Voice API的集成效果
"""

import sys
import os
import base64
import wave
import tempfile
from key import CHATGLM_API_KEY

# 添加当前目录到Python路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from zai import ZhipuAiClient
    print("✅ ChatGLM SDK导入成功")
except ImportError as e:
    print(f"❌ ChatGLM SDK导入失败: {e}")
    print("请运行: pip install zai-sdk")
    sys.exit(1)

# ChatGLM API密钥

def test_chatglm_voice():
    """测试ChatGLM语音生成"""
    try:
        print("🚀 开始测试ChatGLM语音生成...")
        
        # 初始化客户端
        client = ZhipuAiClient(api_key=CHATGLM_API_KEY)
        print("✅ ChatGLM客户端初始化成功")
        
        # 测试文本
        test_text = "您好，这是微分智飞无人机语音操控系统的测试语音。ChatGLM语音模型集成成功！"
        print(f"📝 测试文本: {test_text}")
        
        # 调用GLM-4-Voice API
        print("🎤 正在生成语音...")
        response = client.chat.completions.create(
            model="glm-4-voice",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": test_text
                        }
                    ]
                }
            ],
            stream=False
        )
        
        # 检查响应
        if response.choices[0].message.audio and response.choices[0].message.audio.get('data'):
            audio_data = response.choices[0].message.audio['data']
            decoded_data = base64.b64decode(audio_data)
            
            # 保存为WAV文件
            output_file = "test_chatglm_voice_output.wav"
            with wave.open(output_file, 'wb') as wav_file:
                wav_file.setnchannels(1)      # 单声道
                wav_file.setsampwidth(2)      # 16位
                wav_file.setframerate(44100)  # 44.1kHz
                wav_file.writeframes(decoded_data)
            
            print(f"✅ 语音生成成功！")
            print(f"📁 音频文件保存到: {output_file}")
            print(f"🔊 文件大小: {len(decoded_data)} 字节")
            
            # 在macOS上尝试播放
            try:
                import subprocess
                subprocess.run(['afplay', output_file], check=True)
                print("🎵 音频播放完成")
            except:
                print("💡 请手动播放音频文件来验证效果")
            
            return True
            
        else:
            print("❌ 响应中没有音频数据")
            print(f"响应内容: {response.choices[0].message}")
            return False
            
    except Exception as e:
        print(f"❌ ChatGLM语音生成失败: {e}")
        return False

def test_api_connection():
    """测试API连接"""
    try:
        print("🔗 测试ChatGLM API连接...")
        client = ZhipuAiClient(api_key=CHATGLM_API_KEY)
        
        # 简单的文本生成测试
        response = client.chat.completions.create(
            model="glm-4",
            messages=[{"role": "user", "content": "你好"}],
            max_tokens=10
        )
        
        print("✅ API连接正常")
        return True
        
    except Exception as e:
        print(f"❌ API连接失败: {e}")
        return False

if __name__ == "__main__":
    print("=" * 60)
    print("   🎤 ChatGLM语音功能测试")
    print("=" * 60)
    
    # 测试API连接
    if not test_api_connection():
        print("请检查网络连接和API密钥")
        sys.exit(1)
    
    print()
    
    # 测试语音生成
    if test_chatglm_voice():
        print("\n🎉 ChatGLM语音集成测试成功！")
        print("✨ 您现在可以在无人机操控系统中使用高质量的AI语音了")
    else:
        print("\n💔 ChatGLM语音测试失败")
        print("🔧 系统将自动回退到浏览器TTS")
    
    print("=" * 60)
