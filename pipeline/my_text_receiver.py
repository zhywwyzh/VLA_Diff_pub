#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
简单的文字接收程序示例
用于接收从Gemini聊天系统转发的文字内容
"""

from flask import Flask, request, jsonify
from datetime import datetime
import json
import os

app = Flask(__name__)

# 存储接收到的消息
received_messages = []

@app.route('/api/receive', methods=['POST'])
def receive_message():
    """接收转发的文字消息"""
    try:
        # 获取JSON数据
        data = request.get_json()
        
        if not data:
            return jsonify({'success': False, 'error': '没有接收到数据'}), 400
        
        # 提取文字内容
        text = data.get('text', '')
        source = data.get('source', 'unknown')
        timestamp = data.get('timestamp', datetime.now().isoformat())
        
        if not text:
            return jsonify({'success': False, 'error': '文字内容为空'}), 400
        
        # 创建消息记录
        message = {
            'id': len(received_messages) + 1,
            'text': text,
            'source': source,
            'timestamp': timestamp,
            'received_at': datetime.now().isoformat(),
            'length': len(text)
        }
        
        # 保存消息
        received_messages.append(message)
        
        # 打印到控制台
        print("=" * 60)
        print(f"📨 收到新消息 (#{message['id']})")
        print("=" * 60)
        print(f"来源: {source}")
        print(f"时间: {timestamp}")
        print(f"内容: {text}")
        print(f"长度: {len(text)} 字符")
        print(f"接收时间: {message['received_at']}")
        print("=" * 60)
        
        # 保存到文件（可选）
        save_to_file(message)
        
        return jsonify({
            'success': True,
            'message': f'消息已接收 (ID: {message["id"]})',
            'id': message['id']
        })
        
    except Exception as e:
        print(f"❌ 接收消息时出错: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/messages', methods=['GET'])
def get_messages():
    """获取所有接收到的消息"""
    return jsonify({
        'success': True,
        'total': len(received_messages),
        'messages': received_messages
    })

@app.route('/api/latest', methods=['GET'])
def get_latest_message():
    """获取最新的消息"""
    if received_messages:
        return jsonify({
            'success': True,
            'message': received_messages[-1]
        })
    else:
        return jsonify({
            'success': False,
            'error': '没有消息'
        }), 404

@app.route('/api/clear', methods=['POST'])
def clear_messages():
    """清空所有消息"""
    global received_messages
    count = len(received_messages)
    received_messages = []
    return jsonify({
        'success': True,
        'message': f'已清空 {count} 条消息'
    })

@app.route('/api/health', methods=['GET'])
def health_check():
    """健康检查"""
    return jsonify({
        'success': True,
        'status': 'running',
        'message_count': len(received_messages),
        'service': 'text-receiver'
    })

@app.route('/', methods=['GET'])
def index():
    """主页 - 显示接收到的消息"""
    html = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>文字接收器</title>
        <meta charset="UTF-8">
        <style>
            body {{ font-family: Arial, sans-serif; margin: 20px; background: #f5f5f5; }}
            .container {{ max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }}
            h1 {{ color: #333; text-align: center; }}
            .stats {{ background: #e8f4fd; padding: 15px; border-radius: 5px; margin-bottom: 20px; }}
            .message {{ background: #f9f9f9; padding: 15px; margin: 10px 0; border-left: 4px solid #007bff; }}
            .message-header {{ font-weight: bold; color: #007bff; margin-bottom: 5px; }}
            .message-content {{ margin: 10px 0; }}
            .message-footer {{ font-size: 0.9em; color: #666; }}
            .no-messages {{ text-align: center; color: #999; padding: 40px; }}
            .btn {{ background: #007bff; color: white; padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; margin: 5px; }}
            .btn:hover {{ background: #0056b3; }}
        </style>
        <script>
            function refreshPage() {{ location.reload(); }}
            function clearMessages() {{
                if (confirm('确定要清空所有消息吗？')) {{
                    fetch('/api/clear', {{method: 'POST'}})
                        .then(() => location.reload());
                }}
            }}
            // 每5秒自动刷新
            setInterval(refreshPage, 5000);
        </script>
    </head>
    <body>
        <div class="container">
            <h1>📨 文字接收器</h1>
            <div class="stats">
                <strong>总消息数：</strong> {len(received_messages)} 条<br>
                <strong>服务状态：</strong> 运行中<br>
                <strong>接收端点：</strong> /api/receive
            </div>
            <div>
                <button class="btn" onclick="refreshPage()">🔄 刷新</button>
                <button class="btn" onclick="clearMessages()">🗑️ 清空</button>
            </div>
    """
    
    if received_messages:
        for msg in reversed(received_messages):  # 最新的在前
            html += f"""
            <div class="message">
                <div class="message-header">消息 #{msg['id']} - 来源: {msg['source']}</div>
                <div class="message-content">{msg['text']}</div>
                <div class="message-footer">
                    时间: {msg['timestamp']} | 接收: {msg['received_at']} | 长度: {msg['length']} 字符
                </div>
            </div>
            """
    else:
        html += '<div class="no-messages">暂无接收到的消息</div>'
    
    html += """
        </div>
    </body>
    </html>
    """
    return html

def save_to_file(message):
    """保存消息到文件"""
    try:
        # 创建logs目录
        logs_dir = 'logs'
        if not os.path.exists(logs_dir):
            os.makedirs(logs_dir)
        
        # 保存到JSON文件
        log_file = os.path.join(logs_dir, 'received_messages.json')
        
        # 读取现有数据
        existing_data = []
        if os.path.exists(log_file):
            try:
                with open(log_file, 'r', encoding='utf-8') as f:
                    existing_data = json.load(f)
            except:
                existing_data = []
        
        # 添加新消息
        existing_data.append(message)
        
        # 保存回文件
        with open(log_file, 'w', encoding='utf-8') as f:
            json.dump(existing_data, f, ensure_ascii=False, indent=2)
            
    except Exception as e:
        print(f"⚠️ 保存到文件失败: {e}")

if __name__ == '__main__':
    print("🚀 启动文字接收器...")
    print("📍 服务地址: http://localhost:4000")
    print("📍 接收端点: http://localhost:4000/api/receive")
    print("📍 查看消息: http://localhost:4000")
    print("💡 在Gemini聊天系统中设置转发URL为: http://localhost:4000/api/receive")
    print("-" * 60)
    
    app.run(
        host='0.0.0.0',
        port=4000,
        debug=True
    )
