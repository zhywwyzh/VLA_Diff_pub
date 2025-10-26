#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
接收外部发送消息的服务 - 3001端口
用于接收其他程序发送到本系统的消息

使用方法:
1. 运行此脚本: python incoming_receiver.py
2. 其他程序可以通过 POST http://localhost:3001/api/send 发送消息
3. 消息会显示在主界面右侧对话框中
4. 可以一键转发到Gemini对话输入框
"""

import json
import logging
from datetime import datetime
from flask import Flask, request, jsonify
from flask_cors import CORS
import requests
import threading
import time

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
incoming_messages = []
max_messages = 100  # 最多保存100条消息

@app.route('/api/send', methods=['POST'])
def receive_incoming_message():
    """接收外部程序发送的消息"""
    try:
        # 获取消息数据
        data = request.get_json()
        
        if not data:
            return jsonify({
                'success': False,
                'error': '无效的JSON数据'
            }), 400
        
        # 提取消息信息
        text = data.get('text', data.get('message', ''))
        sender = data.get('sender', data.get('from', 'unknown'))
        message_type = data.get('type', 'text')
        priority = data.get('priority', 'normal')
        images_info = data.get('images_info', [])
        image_count = data.get('image_count', len(images_info) if images_info else 0)
        
        if not text:
            return jsonify({
                'success': False,
                'error': '消息内容不能为空'
            }), 400
        
        # 记录消息
        message_info = {
            'id': len(incoming_messages) + 1,
            'text': text,
            'sender': sender,
            'type': message_type,
            'priority': priority,
            'images_info': images_info,
            'image_count': image_count,
            'timestamp': datetime.now().isoformat(),
            'length': len(text),
            'read': False
        }
        
        # 添加到消息列表（保持最新100条）
        incoming_messages.append(message_info)
        if len(incoming_messages) > max_messages:
            incoming_messages.pop(0)
        
        # 打印到控制台
        print(f"\n{'='*60}")
        print(f"📥 收到外部消息 (#{message_info['id']})")
        print(f"{'='*60}")
        print(f"发送者: {sender}")
        print(f"类型: {message_type}")
        print(f"优先级: {priority}")
        print(f"内容: {text}")
        print(f"长度: {len(text)} 字符")
        
        # 显示图片信息
        if images_info:
            print(f"图片数量: {len(images_info)} 张")
            for img_info in images_info:
                filename = img_info.get('filename', f"图片{img_info.get('index', '?')}")
                format_info = img_info.get('format', 'unknown')
                size_info = img_info.get('size', 'unknown')
                print(f"  📷 {filename}: {format_info} ({size_info})")
        
        print(f"时间: {message_info['timestamp']}")
        print(f"{'='*60}\n")
        
        # 日志记录
        logger.info(f"接收到消息: ID={message_info['id']}, 发送者={sender}, 长度={len(text)}")
        
        # 处理特殊类型的消息
        process_incoming_message(message_info)
        
        return jsonify({
            'success': True,
            'message': '消息接收成功',
            'message_id': message_info['id'],
            'timestamp': message_info['timestamp']
        })
        
    except Exception as e:
        logger.error(f"处理消息时出错: {e}")
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500

def process_incoming_message(message_info):
    """处理接收到的消息"""
    text = message_info['text']
    priority = message_info.get('priority', 'normal')
    images_info = message_info.get('images_info', [])
    
    # 高优先级消息特殊处理
    if priority == 'high' or priority == 'urgent':
        print(f"🚨 高优先级消息: {text[:50]}...")
    
    # 图片处理
    if images_info:
        print(f"🖼️ 处理 {len(images_info)} 张图片...")
        save_incoming_images(images_info, message_info['id'])
    
    # 根据消息类型处理
    message_type = message_info.get('type', 'text')
    if message_type == 'command':
        print(f"⚡ 命令消息: {text}")
    elif message_type == 'query':
        print(f"❓ 查询消息: {text}")
    elif message_type == 'notification':
        print(f"🔔 通知消息: {text}")

def save_incoming_images(images_info, message_id):
    """保存接收到的图片"""
    try:
        import os
        import base64
        
        # 创建图片保存目录
        save_dir = f"incoming_images/message_{message_id}"
        os.makedirs(save_dir, exist_ok=True)
        
        for img_info in images_info:
            if 'data' in img_info:
                try:
                    # 解析base64图片数据
                    data_url = img_info['data']
                    if data_url.startswith('data:image/'):
                        # 提取base64部分
                        header, base64_data = data_url.split(',', 1)
                        img_bytes = base64.b64decode(base64_data)
                        
                        # 确定文件名
                        filename = img_info.get('filename', f"image_{img_info.get('index', 1)}")
                        if not filename.lower().endswith(('.jpg', '.jpeg', '.png', '.gif')):
                            filename += '.jpg'
                        
                        filepath = os.path.join(save_dir, filename)
                        
                        with open(filepath, 'wb') as f:
                            f.write(img_bytes)
                        
                        print(f"  💾 图片已保存: {filepath}")
                        
                except Exception as e:
                    print(f"  ⚠️ 保存图片 {img_info.get('index', '?')} 失败: {e}")
                    
    except Exception as e:
        print(f"⚠️ 图片保存过程出错: {e}")

@app.route('/api/messages', methods=['GET'])
def get_incoming_messages():
    """获取所有接收到的消息"""
    # 支持分页和过滤
    page = request.args.get('page', 1, type=int)
    per_page = request.args.get('per_page', 20, type=int)
    unread_only = request.args.get('unread_only', 'false').lower() == 'true'
    
    # 过滤消息
    messages = incoming_messages
    if unread_only:
        messages = [msg for msg in messages if not msg.get('read', False)]
    
    # 按时间倒序排列（最新的在前）
    messages = sorted(messages, key=lambda x: x['timestamp'], reverse=True)
    
    # 分页
    start = (page - 1) * per_page
    end = start + per_page
    paginated_messages = messages[start:end]
    
    return jsonify({
        'success': True,
        'total': len(incoming_messages),
        'unread_count': len([msg for msg in incoming_messages if not msg.get('read', False)]),
        'messages': paginated_messages,
        'page': page,
        'per_page': per_page,
        'has_more': end < len(messages)
    })

@app.route('/api/messages/<int:message_id>/read', methods=['POST'])
def mark_message_read(message_id):
    """标记消息为已读"""
    message = next((msg for msg in incoming_messages if msg['id'] == message_id), None)
    
    if message:
        message['read'] = True
        return jsonify({
            'success': True,
            'message': '消息已标记为已读'
        })
    else:
        return jsonify({
            'success': False,
            'error': '消息不存在'
        }), 404

@app.route('/api/messages/mark_all_read', methods=['POST'])
def mark_all_read():
    """标记所有消息为已读"""
    for message in incoming_messages:
        message['read'] = True
    
    return jsonify({
        'success': True,
        'message': f'已标记 {len(incoming_messages)} 条消息为已读'
    })

@app.route('/api/clear', methods=['POST'])
def clear_messages():
    """清空所有消息"""
    global incoming_messages
    count = len(incoming_messages)
    incoming_messages = []
    
    logger.info(f"清空了 {count} 条消息")
    
    return jsonify({
        'success': True,
        'message': f'已清空 {count} 条消息'
    })

@app.route('/api/stats', methods=['GET'])
def get_stats():
    """获取统计信息"""
    if not incoming_messages:
        return jsonify({
            'success': True,
            'stats': {
                'total_messages': 0,
                'unread_messages': 0,
                'total_length': 0,
                'average_length': 0,
                'senders': {},
                'message_types': {}
            }
        })
    
    # 统计发送者
    senders = {}
    message_types = {}
    total_length = 0
    unread_count = 0
    
    for msg in incoming_messages:
        sender = msg.get('sender', 'unknown')
        msg_type = msg.get('type', 'text')
        
        senders[sender] = senders.get(sender, 0) + 1
        message_types[msg_type] = message_types.get(msg_type, 0) + 1
        total_length += msg['length']
        
        if not msg.get('read', False):
            unread_count += 1
    
    return jsonify({
        'success': True,
        'stats': {
            'total_messages': len(incoming_messages),
            'unread_messages': unread_count,
            'total_length': total_length,
            'average_length': round(total_length / len(incoming_messages), 2),
            'senders': senders,
            'message_types': message_types
        }
    })

@app.route('/api/health', methods=['GET'])
def health_check():
    """健康检查"""
    return jsonify({
        'success': True,
        'status': 'running',
        'service': 'Incoming Message Receiver',
        'port': 3001,
        'timestamp': datetime.now().isoformat(),
        'total_messages': len(incoming_messages),
        'unread_messages': len([msg for msg in incoming_messages if not msg.get('read', False)])
    })

@app.route('/', methods=['GET'])
def index():
    """简单的状态页面"""
    unread_count = len([msg for msg in incoming_messages if not msg.get('read', False)])
    
    return f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>消息接收服务 - 3001端口</title>
        <meta charset="utf-8">
        <style>
            body {{ font-family: Arial, sans-serif; margin: 40px; }}
            .stats {{ background: #e3f2fd; padding: 20px; border-radius: 8px; margin-bottom: 20px; }}
            .message {{ border: 1px solid #ddd; padding: 15px; margin: 10px 0; border-radius: 5px; }}
            .unread {{ border-left: 4px solid #2196F3; background: #f3f9ff; }}
            .sender {{ font-weight: bold; color: #1976D2; }}
            .timestamp {{ font-size: 12px; color: #666; }}
            .priority-high {{ border-left-color: #f44336; }}
            .priority-urgent {{ border-left-color: #d32f2f; background: #ffebee; }}
        </style>
    </head>
    <body>
        <h1>📨 消息接收服务</h1>
        <div class="stats">
            <h3>📊 服务状态</h3>
            <p>服务端口: <strong>3001</strong></p>
            <p>接收消息数: <strong>{len(incoming_messages)}</strong></p>
            <p>未读消息: <strong>{unread_count}</strong></p>
            <p>服务状态: <span style="color: green;">运行中</span></p>
            <p>当前时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</p>
        </div>
        
        <h3>📥 最近消息</h3>
        {"".join([f'''
        <div class="message {'unread' if not msg.get('read', False) else ''} priority-{msg.get('priority', 'normal')}">
            <div class="sender">{msg.get('sender', 'unknown')} [{msg.get('type', 'text')}]</div>
            <div>{msg["text"][:150]}{"..." if len(msg["text"]) > 150 else ""}</div>
            {f'<div style="color: #2196F3; font-size: 12px;">🖼️ 包含 {msg.get("image_count", 0)} 张图片</div>' if msg.get('image_count', 0) > 0 else ''}
            <div class="timestamp">{msg["timestamp"][:19]}</div>
        </div>
        ''' for msg in sorted(incoming_messages[-10:], key=lambda x: x['timestamp'], reverse=True)])}
        
        <h3>🔗 API端点</h3>
        <ul>
            <li><code>POST /api/send</code> - 发送消息到本系统</li>
            <li><code>GET /api/messages</code> - 获取所有消息</li>
            <li><code>POST /api/messages/mark_all_read</code> - 标记所有消息为已读</li>
            <li><code>GET /api/stats</code> - 获取统计信息</li>
            <li><code>POST /api/clear</code> - 清空消息</li>
        </ul>
        
        <h3>📝 使用示例</h3>
        <pre>
# 发送文本消息
curl -X POST http://localhost:3001/api/send \\
  -H "Content-Type: application/json" \\
  -d '{{"text": "你好，这是一条测试消息", "sender": "测试程序", "type": "text"}}'

# 发送高优先级命令
curl -X POST http://localhost:3001/api/send \\
  -H "Content-Type: application/json" \\
  -d '{{"text": "执行紧急任务", "sender": "控制系统", "type": "command", "priority": "high"}}'
        </pre>
    </body>
    </html>
    """

# 端口配置
INCOMING_RECEIVER_PORT = 3001

if __name__ == '__main__':
    print("🚀 启动消息接收服务...")
    print(f"📍 服务地址: http://localhost:{INCOMING_RECEIVER_PORT}")
    print(f"📍 发送端点: http://localhost:{INCOMING_RECEIVER_PORT}/api/send")
    print(f"📍 状态页面: http://localhost:{INCOMING_RECEIVER_PORT}")
    print(f"💡 其他程序可以通过 POST /api/send 发送消息到本系统")
    print("-" * 60)
    
    app.run(
        host='0.0.0.0',
        port=INCOMING_RECEIVER_PORT,
        debug=True
    )
