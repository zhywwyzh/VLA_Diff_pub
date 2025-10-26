#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
外部程序示例 - 接收Gemini聊天系统转发的消息

这是一个简单的示例程序，展示如何接收从Gemini多模态聊天系统转发的消息。
您可以基于这个示例开发自己的消息处理逻辑。

使用方法:
1. 运行此脚本: python external_receiver_example.py
2. 在聊天系统中设置转发URL: http://localhost:3000/api/receive
3. 与AI聊天，消息将被转发到此程序
"""

import json
import logging
from datetime import datetime
from flask import Flask, request, jsonify
from flask_cors import CORS

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# 创建Flask应用
app = Flask(__name__)
CORS(app)

# 存储接收到的消息
received_messages = []

@app.route('/api/receive', methods=['POST'])
def receive_message():
    """接收来自Gemini聊天系统的转发消息"""
    try:
        # 获取消息数据
        data = request.get_json()
        
        if not data:
            return jsonify({
                'success': False,
                'error': '无效的JSON数据'
            }), 400
        
        # 提取消息信息
        text = data.get('text', '')
        timestamp = data.get('timestamp', '')
        source = data.get('source', 'unknown')
        message_type = data.get('message_type', 'unknown')
        images_info = data.get('images_info', [])
        user_input = data.get('user_input', {})
        audio_text = data.get('audio_text', '')
        
        # 记录消息
        message_info = {
            'id': len(received_messages) + 1,
            'text': text,
            'timestamp': timestamp,
            'source': source,
            'message_type': message_type,
            'images_info': images_info,
            'user_input': user_input,
            'audio_text': audio_text,
            'received_at': datetime.now().isoformat(),
            'length': len(text),
            'image_count': len(images_info) if images_info else 0
        }
        
        received_messages.append(message_info)
        
        # 打印到控制台
        print(f"\n{'='*60}")
        print(f"📨 收到新消息 (#{message_info['id']})")
        print(f"{'='*60}")
        print(f"来源: {source}")
        print(f"消息类型: {message_type}")
        print(f"时间: {timestamp}")
        print(f"内容: {text}")
        print(f"长度: {len(text)} 字符")
        
        # 显示图片信息
        if images_info:
            print(f"图片数量: {len(images_info)} 张")
            for img_info in images_info:
                print(f"  图片 {img_info.get('index', '?')}: {img_info.get('format', 'unknown')} ({img_info.get('size', 'unknown')})")
        
        # 显示用户输入信息
        if user_input:
            if user_input.get('text'):
                print(f"用户文字输入: {user_input['text']}")
            if user_input.get('audio_text'):
                print(f"用户语音输入: {user_input['audio_text']}")
            if user_input.get('image_count', 0) > 0:
                print(f"用户上传图片: {user_input['image_count']} 张")
        
        print(f"接收时间: {message_info['received_at']}")
        print(f"{'='*60}\n")
        
        # 日志记录
        logger.info(f"接收到消息: ID={message_info['id']}, 长度={len(text)}")
        
        # 这里可以添加您的自定义处理逻辑
        # 例如：
        # - 保存到数据库
        # - 发送到其他服务
        # - 触发自动化流程
        # - 分析文本内容
        process_message(message_info)
        
        return jsonify({
            'success': True,
            'message': '消息接收成功',
            'message_id': message_info['id'],
            'processed_at': message_info['received_at']
        })
        
    except Exception as e:
        logger.error(f"处理消息时出错: {e}")
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500

def process_message(message_info):
    """处理接收到的消息 - 在这里添加您的自定义逻辑"""
    
    text = message_info['text']
    images_info = message_info.get('images_info', [])
    user_input = message_info.get('user_input', {})
    message_type = message_info.get('message_type', 'unknown')
    
    # 示例1: 关键词检测
    keywords = ['紧急', '重要', '帮助', '错误', '问题']
    if any(keyword in text for keyword in keywords):
        print(f"🚨 检测到重要消息: {text[:50]}...")
        # 这里可以发送通知、邮件等
    
    # 示例2: 长度分析
    if len(text) > 200:
        print(f"📄 检测到长消息: {len(text)} 字符")
        # 可以进行更详细的分析
    
    # 示例3: 图片处理分析
    if images_info:
        print(f"🖼️ 图片处理分析:")
        for img_info in images_info:
            if 'error' in img_info:
                print(f"  ❌ 图片 {img_info.get('index', '?')} 处理出错: {img_info['error']}")
            else:
                print(f"  ✅ 图片 {img_info.get('index', '?')}: {img_info.get('format', 'unknown')} ({img_info.get('size', 'unknown')})")
                # 这里可以添加图片分析逻辑，比如：
                # - 保存图片到文件
                # - 进行图片识别
                # - 分析图片内容
                # - 统计图片尺寸等
        
        # 可以在这里保存图片数据
        save_images_to_file(images_info, message_info['id'])
    
    # 示例4: 用户输入分析
    if user_input:
        print(f"👤 用户输入分析:")
        if user_input.get('text'):
            print(f"  📝 文字输入: {len(user_input['text'])} 字符")
        if user_input.get('audio_text'):
            print(f"  🎤 语音输入: {len(user_input['audio_text'])} 字符")
        if user_input.get('image_count', 0) > 0:
            print(f"  🖼️ 上传图片: {user_input['image_count']} 张")
    
    # 示例5: 简单的情感分析（基于关键词）
    positive_words = ['好', '棒', '优秀', '满意', '喜欢', '成功']
    negative_words = ['不好', '失败', '错误', '问题', '困难', '不满']
    
    positive_score = sum(1 for word in positive_words if word in text)
    negative_score = sum(1 for word in negative_words if word in text)
    
    if positive_score > negative_score:
        sentiment = "积极"
    elif negative_score > positive_score:
        sentiment = "消极"
    else:
        sentiment = "中性"
    
    print(f"😊 情感分析: {sentiment} (积极:{positive_score}, 消极:{negative_score})")
    
    # 示例6: 数据存储（这里只是打印，实际可以保存到文件或数据库）
    if len(received_messages) % 10 == 0:
        print(f"📊 统计: 已接收 {len(received_messages)} 条消息")

def save_images_to_file(images_info, message_id):
    """保存图片到文件的示例函数"""
    try:
        import os
        import base64
        
        # 创建图片保存目录
        save_dir = f"received_images/message_{message_id}"
        os.makedirs(save_dir, exist_ok=True)
        
        for img_info in images_info:
            if 'data' in img_info and not 'error' in img_info:
                # 解析base64图片数据
                data_url = img_info['data']
                if data_url.startswith('data:image/'):
                    # 提取base64部分
                    header, base64_data = data_url.split(',', 1)
                    img_bytes = base64.b64decode(base64_data)
                    
                    # 保存图片
                    filename = f"image_{img_info.get('index', 1)}.{img_info.get('format', 'png').lower()}"
                    filepath = os.path.join(save_dir, filename)
                    
                    with open(filepath, 'wb') as f:
                        f.write(img_bytes)
                    
                    print(f"  💾 图片已保存: {filepath}")
                    
    except Exception as e:
        print(f"  ⚠️ 保存图片失败: {e}")

@app.route('/api/messages', methods=['GET'])
def get_messages():
    """获取所有接收到的消息"""
    return jsonify({
        'success': True,
        'total': len(received_messages),
        'messages': received_messages
    })

@app.route('/api/messages/<int:message_id>', methods=['GET'])
def get_message(message_id):
    """获取特定消息"""
    message = next((msg for msg in received_messages if msg['id'] == message_id), None)
    
    if message:
        return jsonify({
            'success': True,
            'message': message
        })
    else:
        return jsonify({
            'success': False,
            'error': '消息不存在'
        }), 404

@app.route('/api/clear', methods=['POST'])
def clear_messages():
    """清空所有消息"""
    global received_messages
    count = len(received_messages)
    received_messages = []
    
    logger.info(f"清空了 {count} 条消息")
    
    return jsonify({
        'success': True,
        'message': f'已清空 {count} 条消息'
    })

@app.route('/api/stats', methods=['GET'])
def get_stats():
    """获取统计信息"""
    if not received_messages:
        return jsonify({
            'success': True,
            'stats': {
                'total_messages': 0,
                'total_length': 0,
                'average_length': 0,
                'first_message': None,
                'last_message': None
            }
        })
    
    total_length = sum(msg['length'] for msg in received_messages)
    
    return jsonify({
        'success': True,
        'stats': {
            'total_messages': len(received_messages),
            'total_length': total_length,
            'average_length': round(total_length / len(received_messages), 2),
            'first_message': received_messages[0]['received_at'],
            'last_message': received_messages[-1]['received_at']
        }
    })

@app.route('/api/health', methods=['GET'])
def health_check():
    """健康检查"""
    return jsonify({
        'success': True,
        'status': 'running',
        'service': 'External Message Receiver',
        'timestamp': datetime.now().isoformat(),
        'received_messages': len(received_messages)
    })

@app.route('/', methods=['GET'])
def index():
    """简单的状态页面"""
    return f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>外部消息接收器</title>
        <meta charset="utf-8">
        <style>
            body {{ font-family: Arial, sans-serif; margin: 40px; }}
            .stats {{ background: #f5f5f5; padding: 20px; border-radius: 8px; }}
            .message {{ border: 1px solid #ddd; padding: 10px; margin: 10px 0; border-radius: 5px; }}
        </style>
    </head>
    <body>
        <h1>🔄 外部消息接收器</h1>
        <div class="stats">
            <h3>📊 统计信息</h3>
            <p>接收消息数: {len(received_messages)}</p>
            <p>服务状态: 运行中</p>
            <p>当前时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</p>
        </div>
        
        <h3>📨 最近消息</h3>
        {"".join([f'''<div class="message">
            <strong>#{msg["id"]}</strong> ({msg["received_at"][:19]})
            <br>类型: {msg.get("message_type", "unknown")}
            {"<br>🖼️ 包含图片: " + str(msg.get("image_count", 0)) + " 张" if msg.get("image_count", 0) > 0 else ""}
            <br>{msg["text"][:100]}{"..." if len(msg["text"]) > 100 else ""}
        </div>''' for msg in sorted(received_messages[-5:], key=lambda x: x['timestamp'], reverse=True)])}
        
        <h3>🔗 API端点</h3>
        <ul>
            <li><code>POST /api/receive</code> - 接收消息</li>
            <li><code>GET /api/messages</code> - 获取所有消息</li>
            <li><code>GET /api/stats</code> - 获取统计信息</li>
            <li><code>POST /api/clear</code> - 清空消息</li>
        </ul>
    </body>
    </html>
    """

# 端口配置
EXTERNAL_RECEIVER_PORT = 3000  # 外部接收器端口

if __name__ == '__main__':
    print("🚀 启动外部消息接收器...")
    print(f"📍 服务地址: http://localhost:{EXTERNAL_RECEIVER_PORT}")
    print(f"📍 接收端点: http://localhost:{EXTERNAL_RECEIVER_PORT}/api/receive")
    print(f"📍 状态页面: http://localhost:{EXTERNAL_RECEIVER_PORT}")
    print(f"💡 在Gemini聊天系统中设置转发URL为: http://localhost:{EXTERNAL_RECEIVER_PORT}/api/receive")
    print("-" * 60)
    
    app.run(
        host='0.0.0.0',
        port=EXTERNAL_RECEIVER_PORT,
        debug=True
    )
