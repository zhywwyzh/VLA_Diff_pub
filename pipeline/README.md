# Gemini 2.5 Flash-Lite 多模态聊天系统

这是一个基于 Google Gemini 2.5 Flash-Lite 模型的多模态聊天系统，支持语音、图片、文字输入，并提供语音和文字输出。系统还支持将文字消息转发到外部程序。

## ✨ 功能特性

- 🎤 **语音输入**: 支持实时录音和语音识别（中文/英文）
- 🖼️ **图片输入**: 支持图片上传和智能分析
- 💬 **文字输入**: 支持文字聊天和键盘快捷键
- 🔊 **语音输出**: 自动生成语音回复（TTS）
- 📱 **实时通信**: WebSocket 实时消息推送
- 🔗 **外部转发**: 将AI回复转发到其他程序
- 📱 **响应式设计**: 支持桌面和移动端
- 🎨 **现代UI**: Material Design 风格界面

## 🚀 快速开始

### 环境要求

- Python 3.8+
- 现代浏览器（支持WebRTC和MediaRecorder）
- 网络连接（用于访问Gemini API）

### 安装步骤

1. **克隆或下载代码**
   ```bash
   cd /Users/zzmm4/Desktop/VLA/generative-ai/gemini/multimodal-live-api/pipeline
   ```

2. **安装Python依赖**
   ```bash
   pip install -r requirements.txt
   ```

3. **配置API密钥**
   
   代码中已预设API密钥，如需更换请编辑 `app.py` 文件中的：
   ```python
   GEMINI_API_KEY = "您的API密钥"
   ```

4. **启动服务器**
   ```bash
   python app.py
   ```

5. **打开浏览器**
   
   访问 `http://localhost:8080`，打开 `index.html` 文件

## 📖 使用指南

### 基本聊天

1. **文字聊天**: 在输入框中输入消息，按回车或点击发送按钮
2. **语音聊天**: 点击麦克风按钮开始录音，再次点击停止录音
3. **图片聊天**: 点击相机按钮上传图片，可配合文字或语音描述

### 高级功能

#### 外部程序转发

1. 在配置面板中设置外部程序URL
2. 格式示例: `http://localhost:3000/api/receive`
3. AI回复将自动以JSON格式转发到该URL

转发的数据格式：
```json
{
    "text": "AI回复内容",
    "timestamp": "2024-01-01T12:00:00.000Z",
    "source": "gemini-multimodal-chat"
}
```

#### 语音控制

- 录音时点击停止按钮结束录音
- 支持中文和英文语音识别
- 自动播放AI语音回复

#### 消息管理

- 点击清除按钮删除所有聊天记录
- 点击复制按钮复制AI回复
- 点击音量按钮重新播放语音

## 🔧 API 接口

### 主要端点

#### POST /api/chat
发送多模态消息

请求格式：
```json
{
    "text": "文字内容",
    "image": "data:image/jpeg;base64,/9j/4AAQ...",
    "audio": "base64编码的音频数据"
}
```

响应格式：
```json
{
    "success": true,
    "response": {
        "text": "AI回复文字",
        "audio": "base64编码的语音回复",
        "recognized_speech": "识别的语音文字"
    }
}
```

#### POST /api/set_forward_url
设置外部转发URL

#### GET /api/get_forward_url
获取当前转发URL

#### POST /api/clear
清除聊天历史

#### GET /api/test
测试服务状态

### WebSocket 事件

- `connect`: 连接建立
- `disconnect`: 连接断开
- `new_message`: 新消息广播
- `send_message`: 发送实时消息

## 🏗️ 项目结构

```
pipeline/
├── app.py              # Flask后端服务器
├── index.html          # 前端HTML界面
├── styles.css          # CSS样式文件
├── script.js           # JavaScript前端逻辑
├── requirements.txt    # Python依赖
├── README.md          # 项目文档
└── uploads/           # 文件上传目录（自动创建）
```

## ⚙️ 配置选项

### 环境变量

可以通过环境变量覆盖默认配置：

```bash
export GEMINI_API_KEY="your-api-key"
export PORT=8080
export HOST="0.0.0.0"
```

### 模型参数

在 `app.py` 中可以调整Gemini模型配置：

```python
# 切换模型
model = genai.GenerativeModel('gemini-2.5-flash-lite')

# 调整生成参数
generation_config = {
    "temperature": 0.7,
    "top_p": 0.8,
    "max_output_tokens": 2048,
}
```

## 🔍 故障排除

### 常见问题

1. **录音不工作**
   - 检查浏览器权限，允许麦克风访问
   - 确保使用HTTPS或localhost访问

2. **语音识别失败**
   - 检查网络连接
   - 尝试说话更清晰
   - 检查浏览器兼容性

3. **图片上传失败**
   - 确保图片小于5MB
   - 支持格式：JPG, PNG, GIF, WebP

4. **连接失败**
   - 检查后端服务是否启动
   - 检查端口8080是否被占用
   - 查看浏览器控制台错误信息

### 日志查看

后端日志会显示在控制台中，包含：
- API调用详情
- 错误信息
- 连接状态
- 转发状态

## 🌐 部署说明

### 本地部署
直接运行 `python app.py` 即可

### 生产部署

1. **使用Gunicorn**
   ```bash
   pip install gunicorn
   gunicorn -w 4 -b 0.0.0.0:8080 app:app
   ```

2. **使用Docker**
   ```dockerfile
   FROM python:3.9
   WORKDIR /app
   COPY requirements.txt .
   RUN pip install -r requirements.txt
   COPY . .
   EXPOSE 8080
   CMD ["python", "app.py"]
   ```

3. **Nginx反向代理**
   ```nginx
   server {
       listen 80;
       server_name your-domain.com;
       
       location / {
           proxy_pass http://localhost:8080;
           proxy_http_version 1.1;
           proxy_set_header Upgrade $http_upgrade;
           proxy_set_header Connection 'upgrade';
           proxy_set_header Host $host;
       }
   }
   ```

## 🔒 安全注意事项

- 请勿在公网直接暴露API密钥
- 建议使用环境变量管理敏感信息
- 生产环境请配置HTTPS
- 限制文件上传大小和类型
- 考虑添加用户认证

## 🤝 开发指南

### 添加新功能

1. **后端API**: 在 `app.py` 中添加新的路由
2. **前端UI**: 在 `index.html` 中添加界面元素
3. **样式**: 在 `styles.css` 中添加样式
4. **逻辑**: 在 `script.js` 中添加交互逻辑

### 自定义模型

```python
# 在app.py中修改
model = genai.GenerativeModel('your-custom-model')
```

### 集成其他服务

通过修改 `forward_to_external` 方法，可以集成：
- 数据库存储
- 消息队列
- 第三方API
- 日志系统

## 📄 许可证

本项目仅供学习和研究使用。使用Gemini API需要遵守Google的服务条款。

## 🆘 支持

如有问题，请检查：
1. 浏览器控制台错误
2. 服务器日志输出
3. 网络连接状态
4. API配额使用情况

---

**注意**: 请确保在使用前已正确配置API密钥，并检查Gemini API的使用配额和计费情况。
