# 终端发送程序使用指南

## 📋 功能说明

`terminal_sender.py` 是一个命令行工具，可以从终端向系统发送文字和图片消息。

## 🎯 两种发送模式

### 1. **直接对话模式** (推荐) - 发送到1024端口
- 直接与Gemini对话，立即获得回复
- 支持发送图片+文字
- 自动生成ChatGLM语音

### 2. **消息队列模式** - 发送到3001端口  
- 消息进入队列，需要在网页界面手动处理
- 仅支持文字，不支持图片

## 🚀 使用方法

### 基础文字对话
```bash
# 发送简单文字消息
python terminal_sender.py -t "你好，请介绍一下自己"

# 发送到消息队列
python terminal_sender.py -t "这是一条队列消息" --queue
```

### 图片分析
```bash
# 发送单张图片+文字
python terminal_sender.py -i "photo.jpg" -t "请分析这张图片"

# 发送多张图片+文字
python terminal_sender.py -m "img1.jpg" "img2.png" "img3.gif" -t "比较这些图片的差异"
```

### 高级用法
```bash
# 指定发送者名称(队列模式)
python terminal_sender.py -t "来自机器人的消息" --queue -s "ROS机器人"

# 长文本消息
python terminal_sender.py -t "请帮我总结一下今天的会议内容，包括：1. 讨论的主要议题 2. 决定事项 3. 后续行动计划"
```

## 📊 参数说明

| 参数 | 说明 | 必需 | 示例 |
|------|------|------|------|
| `-t, --text` | 要发送的文字内容 | ✅ | `-t "你好"` |
| `-i, --image` | 单张图片路径 | ❌ | `-i "photo.jpg"` |
| `-m, --images` | 多张图片路径 | ❌ | `-m "img1.jpg" "img2.png"` |
| `-q, --queue` | 发送到消息队列模式 | ❌ | `--queue` |
| `-s, --sender` | 发送者名称(仅队列模式) | ❌ | `-s "机器人"` |

## ✅ 支持的图片格式

- JPEG (.jpg, .jpeg)
- PNG (.png)
- GIF (.gif)

## 🔗 服务端口

- **1024端口**: 主程序对话接口 (默认)
- **3001端口**: 消息队列接口 (使用 `--queue` 参数)

## 📝 使用示例

### 场景1: 快速提问
```bash
python terminal_sender.py -t "什么是人工智能？"
```

### 场景2: 图片识别
```bash
python terminal_sender.py -i "cat.jpg" -t "这是什么动物？"
```

### 场景3: 多图比较
```bash
python terminal_sender.py -m "before.jpg" "after.jpg" -t "对比这两张图片的变化"
```

### 场景4: 发送到队列
```bash
python terminal_sender.py -t "请检查系统状态" --queue -s "监控系统"
```

## 🎵 语音功能

使用直接对话模式时，系统会自动：
1. 调用Gemini生成文字回复
2. 使用ChatGLM生成高质量中文语音
3. 在网页界面自动播放语音

## ⚠️ 注意事项

1. **图片路径**: 确保图片文件存在且可读
2. **网络连接**: 需要稳定的网络连接
3. **服务状态**: 确保目标服务(1024/3001端口)正在运行
4. **图片大小**: 建议图片不超过5MB
5. **队列限制**: 消息队列模式不支持图片

## 🛠️ 故障排除

### 常见错误
- `❌ 图片文件不存在`: 检查图片路径是否正确
- `❌ HTTP错误`: 检查服务是否启动
- `❌ 发送异常`: 检查网络连接

### 检查服务状态
```bash
# 检查主程序
curl http://localhost:1024/api/test

# 检查消息队列
curl http://localhost:3001/api/health
```
