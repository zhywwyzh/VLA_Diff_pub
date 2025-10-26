#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
简单的文字监听器
用于接收和处理从Gemini聊天系统转发的文字内容
"""

from flask import Flask, request, jsonify
from datetime import datetime
import threading
import time

app = Flask(__name__)

def process_text(text, source="gemini-chat"):
    """
    处理接收到的文字内容
    在这里添加您的处理逻辑
    """
    print(f"\n🔥 开始处理文字内容...")
    print(f"📝 内容: {text}")
    print(f"📊 长度: {len(text)} 字符")
    print(f"🕒 时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    # ========================================
    # 在这里添加您的处理逻辑
    # ========================================
    
    # 示例1: 文字分析
    if len(text) > 50:
        print("📏 这是一条长消息")
    else:
        print("📏 这是一条短消息")
    
    # 示例2: 关键词检测
    keywords = ["测试", "帮助", "问题", "API"]
    found_keywords = [kw for kw in keywords if kw in text]
    if found_keywords:
        print(f"🔍 发现关键词: {', '.join(found_keywords)}")
    
    # 示例3: 保存到文件
    try:
        with open('received_texts.txt', 'a', encoding='utf-8') as f:
            f.write(f"[{datetime.now()}] {text}\n")
        print("💾 已保存到文件")
    except Exception as e:
        print(f"❌ 保存失败: {e}")
    
    # 示例4: 发送到其他系统（您可以修改这部分）
    # send_to_other_system(text)
    
    print("✅ 处理完成\n")

def send_to_other_system(text):
    """
    发送到其他系统的示例函数
    您可以根据需要修改这个函数
    """
    # 示例：发送到其他API
    # import requests
    # try:
    #     response = requests.post('http://your-other-system.com/api', 
    #                            json={'text': text})
    #     print(f"发送到其他系统成功: {response.status_code}")
    # except Exception as e:
    #     print(f"发送到其他系统失败: {e}")
    pass

@app.route('/api/receive', methods=['POST'])
def receive_text():
    """接收文字的API端点"""
    try:
        data = request.get_json()
        
        if not data:
            return jsonify({'success': False, 'error': '没有数据'}), 400
        
        text = data.get('text', '')
        source = data.get('source', 'unknown')
        
        if not text:
            return jsonify({'success': False, 'error': '文字内容为空'}), 400
        
        # 在后台线程处理文字，避免阻塞响应
        threading.Thread(target=process_text, args=(text, source)).start()
        
        return jsonify({
            'success': True,
            'message': '文字已接收并开始处理',
            'length': len(text)
        })
        
    except Exception as e:
        print(f"❌ 接收文字时出错: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/health', methods=['GET'])
def health_check():
    """健康检查"""
    return jsonify({
        'success': True,
        'status': 'running',
        'service': 'simple-text-listener',
        'time': datetime.now().isoformat()
    })

@app.route('/', methods=['GET'])
def index():
    """简单的状态页面"""
    return f"""
    <h1>🎧 简单文字监听器</h1>
    <p><strong>状态:</strong> 运行中</p>
    <p><strong>接收端点:</strong> /api/receive</p>
    <p><strong>时间:</strong> {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</p>
    <p><strong>说明:</strong> 这个服务会接收并处理转发的文字内容</p>
    """

if __name__ == '__main__':
    print("🎧 启动简单文字监听器...")
    print("📍 服务地址: http://localhost:6000")
    print("📍 接收端点: http://localhost:6000/api/receive")
    print("💡 在Gemini聊天系统中设置转发URL为: http://localhost:6000/api/receive")
    print("🔧 请在 process_text() 函数中添加您的处理逻辑")
    print("-" * 60)
    
    app.run(
        host='0.0.0.0',
        port=6000,
        debug=False  # 关闭debug模式，减少输出干扰
    )
