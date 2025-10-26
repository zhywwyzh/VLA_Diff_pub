# Gemini 多模态聊天系统转发功能

## 📋 项目简介

这是一个基于 Google Gemini 2.5 Flash-Lite 的多模态聊天系统，支持文字、语音和图片输入，并提供消息转发功能，可以将 AI 的回复自动转发到外部程序进行进一步处理。

## 🏗️ 系统架构

```
用户界面 (index.html) 
    ↓
主服务 (app.py:1024)
    ↓ 转发AI回复
外部接收器 (external_receiver_example.py:3000)
    ↓
您的自定义处理程序
```

## 🚀 快速启动

### 一键启动所有服务
```bash
./start_all.sh
```

### 手动启动
1. **启动主服务**:
   ```bash
   python app.py
   ```

2. **启动外部接收器**:
   ```bash
   python external_receiver_example.py
   ```

3. **访问界面**: 打开 `index.html` 文件或访问 `http://localhost:1024`

## ⚙️ 端口配置

### 默认端口设置
- **主服务端口**: `1024` (在 `app.py` 中的 `MAIN_SERVICE_PORT`)
- **外部接收器端口**: `3000` (在 `external_receiver_example.py` 中的 `EXTERNAL_RECEIVER_PORT`)
- **转发URL**: `http://localhost:3000/api/receive` (在 `app.py` 中的 `DEFAULT_FORWARD_PORT`)

### 如何修改端口

#### 1. 修改主服务端口
在 `app.py` 文件中找到：
```python
# 端口配置
MAIN_SERVICE_PORT = 1024  # 主服务端口
```
将 `1024` 修改为您想要的端口号。

#### 2. 修改外部接收器端口
在 `external_receiver_example.py` 文件中找到：
```python
# 端口配置
EXTERNAL_RECEIVER_PORT = 3000  # 外部接收器端口
```
将 `3000` 修改为您想要的端口号。

#### 3. 修改默认转发端口
在 `app.py` 文件中找到：
```python
# 转发配置 - 默认设置
DEFAULT_FORWARD_PORT = 3000  # 默认外部接收器端口
```
将 `3000` 修改为您的外部接收器端口号。

#### 4. 修改启动脚本中的端口
在 `start_all.sh` 文件中找到：
```bash
MAIN_PORT=1024
EXTERNAL_PORT=3000
```
相应修改这些值。

## 🔄 转发功能说明

### 自动转发
系统默认会将所有 AI 回复自动转发到 `http://localhost:3000/api/receive`。

### 手动配置转发URL
您也可以通过 Web 界面或 API 动态修改转发URL：

**通过API设置**:
```bash
curl -X POST http://localhost:1024/api/config/forward \
  -H "Content-Type: application/json" \
  -d '{"url": "http://your-server:port/api/receive"}'
```

**查看当前配置**:
```bash
curl http://localhost:1024/api/config/forward
```

## 📡 外部程序调用指南

### 接收转发消息的API端点

外部程序需要提供一个 POST 端点来接收转发的消息：

```python
@app.route('/api/receive', methods=['POST'])
def receive_message():
    data = request.get_json()
    # data 包含:
    # {
    #   "text": "AI回复的文本内容",
    #   "timestamp": "2025-08-21T15:20:39.977356",
    #   "source": "gemini-chat"
    # }
    return jsonify({"success": True})
```

### 查询已转发的消息

外部接收器提供以下API端点：

#### 1. 获取所有消息
```bash
curl http://localhost:3000/api/messages
```

#### 2. 获取特定消息
```bash
curl http://localhost:3000/api/messages/1
```

#### 3. 获取统计信息
```bash
curl http://localhost:3000/api/stats
```

#### 4. 清空所有消息
```bash
curl -X POST http://localhost:3000/api/clear
```

#### 5. 健康检查
```bash
curl http://localhost:3000/api/health
```

## 💡 示例调用程序

### Python 客户端示例
参考 `client_example.py` 文件，它展示了如何：
- 连接到外部接收器
- 实时监听新消息
- 处理接收到的AI回复

### 基本使用模式
```python
import requests
import time

# 获取最新消息
def get_latest_messages():
    response = requests.get('http://localhost:3000/api/messages')
    if response.status_code == 200:
        data = response.json()
        return data['messages']
    return []

# 处理消息
def process_message(message):
    print(f"收到AI回复: {message['text']}")
    print(f"时间: {message['timestamp']}")
    print(f"长度: {message['length']} 字符")
    
    # 在这里添加您的处理逻辑
    # 例如：保存到数据库、发送邮件、触发其他流程等

# 实时监控新消息
last_count = 0
while True:
    messages = get_latest_messages()
    if len(messages) > last_count:
        # 处理新消息
        for msg in messages[last_count:]:
            process_message(msg)
        last_count = len(messages)
    time.sleep(1)  # 每秒检查一次
```

## 🛠️ 开发调试

### 查看日志
```bash
# 查看主服务日志
tail -f logs/main_service.log

# 查看外部接收器日志
tail -f logs/external_receiver.log
```

### 测试转发功能
```bash
# 直接发送测试消息到主服务
curl -X POST http://localhost:1024/api/chat \
  -H "Content-Type: application/json" \
  -d '{"text": "测试转发功能"}'

# 检查是否收到转发消息
curl http://localhost:3000/api/messages
```

## 🔧 自定义外部处理器

您可以基于 `external_receiver_example.py` 创建自己的消息处理器：

1. **复制示例文件**:
   ```bash
   cp external_receiver_example.py my_custom_processor.py
   ```

2. **修改处理逻辑**:
   在 `process_message()` 函数中添加您的自定义逻辑

3. **修改端口** (如果需要):
   ```python
   EXTERNAL_RECEIVER_PORT = 4000  # 使用不同端口
   ```

4. **更新转发配置**:
   ```bash
   curl -X POST http://localhost:1024/api/config/forward \
     -H "Content-Type: application/json" \
     -d '{"url": "http://localhost:4000/api/receive"}'
   ```

## 📚 API 文档

### 主服务 API (默认端口1024)
- `POST /api/chat` - 发送聊天消息
- `GET /api/history` - 获取聊天历史
- `POST /api/clear` - 清除聊天历史
- `POST /api/config/forward` - 设置转发URL
- `GET /api/config/forward` - 获取转发配置
- `GET /api/test` - 服务健康检查

### 外部接收器 API (默认端口3000)
- `POST /api/receive` - 接收转发消息
- `GET /api/messages` - 获取所有消息
- `GET /api/messages/<id>` - 获取特定消息
- `GET /api/stats` - 获取统计信息
- `POST /api/clear` - 清空消息
- `GET /api/health` - 健康检查

## 🚨 常见问题

### Q: 转发功能不工作
A: 检查：
1. 外部接收器是否正在运行
2. 转发URL是否正确设置
3. 端口是否被占用
4. 防火墙设置

### Q: 如何修改AI模型
A: 在 `app.py` 中修改：
```python
model = genai.GenerativeModel('gemini-2.5-flash-lite')  # 改为其他模型
```

### Q: 如何添加认证
A: 在外部接收器的 `/api/receive` 端点中添加认证逻辑

## 📞 支持

如果您遇到问题，请：
1. 查看日志文件 (`logs/` 目录)
2. 检查端口占用情况
3. 确认所有服务正在运行
4. 验证转发URL配置

## 📝 更新日志

- v1.1: 添加默认转发端口配置
- v1.0: 初始版本，基本转发功能