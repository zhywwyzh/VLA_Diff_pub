#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Gemini 聊天系统 - 外部客户端示例

这个示例展示了如何在您的程序中调用和处理从Gemini聊天系统转发的消息。
包含实时监听、批量处理、数据分析等功能。

使用方法:
1. 确保外部接收器正在运行: python external_receiver_example.py
2. 确保主服务正在运行: python app.py  
3. 运行此客户端: python client_example.py
4. 在聊天界面与AI对话，观察此程序的输出
"""

import requests
import time
import json
import threading
from datetime import datetime, timedelta
from typing import List, Dict, Optional
import logging

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class GeminiMessageClient:
    """Gemini消息客户端 - 用于接收和处理转发的AI回复"""
    
    def __init__(self, receiver_host='localhost', receiver_port=3000):
        self.base_url = f"http://{receiver_host}:{receiver_port}"
        self.last_message_id = 0
        self.running = False
        self.message_handlers = []
        
    def add_message_handler(self, handler):
        """添加消息处理器"""
        self.message_handlers.append(handler)
        
    def get_all_messages(self) -> List[Dict]:
        """获取所有消息"""
        try:
            response = requests.get(f"{self.base_url}/api/messages", timeout=5)
            if response.status_code == 200:
                data = response.json()
                return data.get('messages', [])
        except Exception as e:
            logger.error(f"获取消息失败: {e}")
        return []
    
    def get_new_messages(self) -> List[Dict]:
        """获取新消息（自上次检查以来）"""
        messages = self.get_all_messages()
        new_messages = [msg for msg in messages if msg['id'] > self.last_message_id]
        if new_messages:
            self.last_message_id = max(msg['id'] for msg in new_messages)
        return new_messages
    
    def get_stats(self) -> Dict:
        """获取统计信息"""
        try:
            response = requests.get(f"{self.base_url}/api/stats", timeout=5)
            if response.status_code == 200:
                return response.json()
        except Exception as e:
            logger.error(f"获取统计信息失败: {e}")
        return {}
    
    def clear_messages(self) -> bool:
        """清空所有消息"""
        try:
            response = requests.post(f"{self.base_url}/api/clear", timeout=5)
            return response.status_code == 200
        except Exception as e:
            logger.error(f"清空消息失败: {e}")
            return False
    
    def is_receiver_running(self) -> bool:
        """检查外部接收器是否运行"""
        try:
            response = requests.get(f"{self.base_url}/api/health", timeout=2)
            return response.status_code == 200
        except:
            return False
    
    def start_monitoring(self, interval=1):
        """开始实时监控新消息"""
        if self.running:
            logger.warning("监控已在运行中")
            return
            
        if not self.is_receiver_running():
            logger.error("外部接收器未运行，请先启动 external_receiver_example.py")
            return
            
        self.running = True
        logger.info("开始监控新消息...")
        
        def monitor_loop():
            while self.running:
                try:
                    new_messages = self.get_new_messages()
                    for message in new_messages:
                        self._process_message(message)
                    time.sleep(interval)
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    logger.error(f"监控过程中出错: {e}")
                    time.sleep(interval)
        
        # 在后台线程中运行监控
        self.monitor_thread = threading.Thread(target=monitor_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
    
    def stop_monitoring(self):
        """停止监控"""
        self.running = False
        logger.info("停止监控")
    
    def _process_message(self, message: Dict):
        """处理单条消息"""
        logger.info(f"收到新消息 #{message['id']}")
        
        # 调用所有注册的处理器
        for handler in self.message_handlers:
            try:
                handler(message)
            except Exception as e:
                logger.error(f"消息处理器出错: {e}")

# 示例消息处理器
class MessageProcessor:
    """消息处理器示例"""
    
    @staticmethod
    def simple_printer(message: Dict):
        """简单打印处理器"""
        print(f"\n{'='*50}")
        print(f"📨 收到AI回复 #{message['id']}")
        print(f"{'='*50}")
        print(f"内容: {message['text']}")
        print(f"时间: {message['timestamp']}")
        print(f"长度: {message['length']} 字符")
        print(f"来源: {message['source']}")
        print(f"{'='*50}\n")
    
    @staticmethod
    def keyword_detector(message: Dict):
        """关键词检测处理器"""
        text = message['text'].lower()
        
        # 定义关键词类别
        keywords = {
            '紧急': ['紧急', '立即', '马上', '赶快', 'urgent'],
            '问题': ['错误', '问题', '故障', '失败', 'error', 'problem'],
            '积极': ['好的', '成功', '完成', '正确', 'excellent', 'success'],
            '技术': ['代码', '程序', '算法', '数据库', 'python', 'javascript']
        }
        
        detected = []
        for category, words in keywords.items():
            if any(word in text for word in words):
                detected.append(category)
        
        if detected:
            print(f"🔍 关键词检测 #{message['id']}: {', '.join(detected)}")
    
    @staticmethod
    def length_analyzer(message: Dict):
        """长度分析处理器"""
        length = message['length']
        if length > 200:
            print(f"📄 长消息警告 #{message['id']}: {length} 字符")
        elif length < 10:
            print(f"📝 短消息 #{message['id']}: {length} 字符")
    
    @staticmethod
    def save_to_file(message: Dict):
        """保存到文件处理器"""
        filename = f"gemini_messages_{datetime.now().strftime('%Y%m%d')}.txt"
        with open(filename, 'a', encoding='utf-8') as f:
            f.write(f"\n[{message['timestamp']}] #{message['id']}\n")
            f.write(f"{message['text']}\n")
            f.write("-" * 50 + "\n")

class MessageAnalyzer:
    """消息分析器"""
    
    def __init__(self, client: GeminiMessageClient):
        self.client = client
    
    def analyze_recent_messages(self, hours=1) -> Dict:
        """分析最近几小时的消息"""
        messages = self.client.get_all_messages()
        cutoff_time = datetime.now() - timedelta(hours=hours)
        
        recent_messages = []
        for msg in messages:
            try:
                msg_time = datetime.fromisoformat(msg['received_at'].replace('Z', '+00:00'))
                if msg_time.replace(tzinfo=None) > cutoff_time:
                    recent_messages.append(msg)
            except:
                continue
        
        analysis = {
            'total_messages': len(recent_messages),
            'total_characters': sum(msg['length'] for msg in recent_messages),
            'average_length': 0,
            'longest_message': None,
            'shortest_message': None,
            'time_range': f"最近 {hours} 小时"
        }
        
        if recent_messages:
            analysis['average_length'] = analysis['total_characters'] / len(recent_messages)
            analysis['longest_message'] = max(recent_messages, key=lambda x: x['length'])
            analysis['shortest_message'] = min(recent_messages, key=lambda x: x['length'])
        
        return analysis
    
    def print_analysis(self, analysis: Dict):
        """打印分析结果"""
        print(f"\n📊 消息分析报告 - {analysis['time_range']}")
        print("=" * 40)
        print(f"消息总数: {analysis['total_messages']}")
        print(f"总字符数: {analysis['total_characters']}")
        print(f"平均长度: {analysis['average_length']:.1f} 字符")
        
        if analysis['longest_message']:
            print(f"最长消息: #{analysis['longest_message']['id']} ({analysis['longest_message']['length']} 字符)")
        
        if analysis['shortest_message']:
            print(f"最短消息: #{analysis['shortest_message']['id']} ({analysis['shortest_message']['length']} 字符)")
        print("=" * 40 + "\n")

def main():
    """主函数 - 演示各种使用方式"""
    print("🚀 Gemini 消息客户端示例")
    print("=" * 50)
    
    # 创建客户端
    client = GeminiMessageClient()
    
    # 检查连接
    if not client.is_receiver_running():
        print("❌ 错误: 外部接收器未运行")
        print("请先运行: python external_receiver_example.py")
        return
    
    print("✅ 成功连接到外部接收器")
    
    # 显示当前统计
    stats = client.get_stats()
    if stats.get('success'):
        stats_data = stats.get('stats', {})
        print(f"📊 当前统计: {stats_data.get('total_messages', 0)} 条消息")
    
    # 添加消息处理器
    client.add_message_handler(MessageProcessor.simple_printer)
    client.add_message_handler(MessageProcessor.keyword_detector)
    client.add_message_handler(MessageProcessor.length_analyzer)
    client.add_message_handler(MessageProcessor.save_to_file)
    
    # 创建分析器
    analyzer = MessageAnalyzer(client)
    
    # 显示现有消息
    existing_messages = client.get_all_messages()
    if existing_messages:
        print(f"\n📋 发现 {len(existing_messages)} 条历史消息")
        client.last_message_id = max(msg['id'] for msg in existing_messages)
        
        # 分析现有消息
        analysis = analyzer.analyze_recent_messages(24)  # 分析最近24小时
        analyzer.print_analysis(analysis)
    
    # 开始实时监控
    print("🔄 开始实时监控新消息...")
    print("💡 现在可以在聊天界面与AI对话，此程序会自动显示AI的回复")
    print("⌨️  按 Ctrl+C 退出程序")
    print("-" * 50)
    
    try:
        # 启动监控
        client.start_monitoring(interval=0.5)  # 每0.5秒检查一次
        
        # 定期显示分析报告
        last_analysis_time = time.time()
        
        while True:
            time.sleep(10)  # 每10秒检查一次
            
            # 每5分钟显示一次分析报告
            if time.time() - last_analysis_time > 300:  # 5分钟
                analysis = analyzer.analyze_recent_messages(1)  # 分析最近1小时
                if analysis['total_messages'] > 0:
                    analyzer.print_analysis(analysis)
                last_analysis_time = time.time()
                
    except KeyboardInterrupt:
        print("\n\n👋 用户中断，正在退出...")
        client.stop_monitoring()
        
        # 显示最终统计
        final_stats = client.get_stats()
        if final_stats.get('success'):
            stats_data = final_stats.get('stats', {})
            print(f"📊 最终统计: {stats_data.get('total_messages', 0)} 条消息")
        
        print("✅ 程序已退出")

if __name__ == '__main__':
    main()
