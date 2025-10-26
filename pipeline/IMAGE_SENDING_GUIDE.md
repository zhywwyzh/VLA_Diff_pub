# 📷 图片发送功能使用指南

## 🎯 概述

现在您可以向3001端口的消息接收服务发送包含图片的消息了！我们为您提供了一个增强版的发送工具，支持发送文本+图片的组合消息。

## 🚀 快速开始

### 1. 启动服务

首先确保3001端口的接收服务正在运行：

```bash
# 方法1: 使用一键启动脚本
./start_all.sh

# 方法2: 手动启动3001端口服务
python incoming_receiver.py
```

### 2. 使用增强版发送工具

```bash
python send_message_with_images.py
```

## 📋 功能说明

### ✨ 新增功能

1. **📤 发送测试图片消息** - 自动生成测试图片并发送
2. **🖼️ 发送自定义消息+图片** - 支持上传本地图片文件
3. **📝 发送纯文本消息** - 兼容原有功能
4. **🔍 检查服务状态** - 实时查看3001端口服务状态

### 🎨 图片处理特性

- **自动压缩**: 大图片自动压缩到1024x1024以内
- **格式转换**: 自动转换为JPEG格式以节省空间
- **多图片支持**: 最多支持5张图片
- **自动保存**: 接收到的图片自动保存到`incoming_images/`目录

## 📖 使用示例

### 示例1: 发送测试图片

```bash
# 运行发送工具
python send_message_with_images.py

# 选择选项1: 发送测试图片消息
# 工具会自动生成3张不同颜色的测试图片并发送
```

### 示例2: 发送自定义图片

```bash
# 运行发送工具并选择选项2
# 按提示输入消息内容、选择类型和优先级
# 当询问是否添加图片时，输入 'y'
# 然后输入图片文件路径，如: /path/to/your/image.jpg
```

### 示例3: 通过代码调用

```python
from send_message_with_images import send_message_with_images

# 发送包含图片的消息
success = send_message_with_images(
    text="请分析这些图片",
    images=["image1.jpg", "image2.png"],
    sender="我的程序",
    message_type="query",
    priority="normal"
)
```

## 🔧 API 接口

### 发送消息 API

**端点**: `POST http://localhost:3001/api/send`

**请求格式**:
```json
{
    "text": "消息内容",
    "sender": "发送者名称",
    "type": "text|query|command|notification",
    "priority": "normal|high|urgent",
    "images_info": [
        {
            "index": 1,
            "filename": "image.jpg",
            "size": "300x200",
            "format": "JPEG",
            "data": "data:image/jpeg;base64,/9j/4AAQ..."
        }
    ],
    "image_count": 1
}
```

## 📁 文件结构

```
pipeline/
├── send_message_with_images.py    # 增强版发送工具
├── test_send_message.py           # 原版发送工具
├── incoming_receiver.py           # 3001端口接收服务
├── incoming_images/               # 接收到的图片保存目录
│   └── message_1/                 # 按消息ID分组
│       ├── image_1.jpg
│       └── image_2.jpg
└── received_images/               # 从3000端口转发的图片
    └── message_1/
        └── image_1.png
```

## 🎯 使用场景

### 1. 系统监控
```bash
# 发送包含截图的系统状态报告
python send_message_with_images.py
# 选择自定义消息，上传服务器截图
```

### 2. 问题报告
```bash
# 发送包含错误截图的问题报告
# 消息类型选择: notification
# 优先级选择: high
```

### 3. 数据分析
```bash
# 发送包含图表的数据分析请求
# 消息类型选择: query
# 上传数据图表文件
```

## 📊 接收效果

当您发送包含图片的消息后，在3001端口服务的控制台会看到：

```
============================================================
📥 收到外部消息 (#1)
============================================================
发送者: 测试程序
类型: text
优先级: normal
内容: 这是一条包含图片的测试消息
长度: 15 字符
图片数量: 2 张
  📷 image1.jpg: JPEG (300x200)
  📷 image2.jpg: JPEG (400x300)
时间: 2024-01-15T10:30:00.123456
============================================================

🖼️ 处理 2 张图片...
  💾 图片已保存: incoming_images/message_1/image1.jpg
  💾 图片已保存: incoming_images/message_1/image2.jpg
```

## 🔗 与转发功能的区别

| 功能 | 3001端口接收 | 3000端口转发 |
|------|-------------|-------------|
| 作用 | 接收外部程序发送的消息 | 接收AI回复的转发 |
| 图片来源 | 外部程序上传 | 用户在聊天界面上传 |
| 保存位置 | `incoming_images/` | `received_images/` |
| 消息类型 | 外部→系统 | AI→外部 |

## 🆘 常见问题

### Q: 为什么图片发送失败？
A: 请检查：
1. 图片文件是否存在
2. 文件格式是否支持 (JPG/PNG/GIF)
3. 文件大小是否过大 (建议<10MB)
4. 3001端口服务是否正在运行

### Q: 图片在哪里保存？
A: 图片会自动保存到 `incoming_images/message_ID/` 目录中

### Q: 支持哪些图片格式？
A: 支持常见格式 (JPG, PNG, GIF等)，会自动转换为JPEG格式

### Q: 如何查看接收到的消息？
A: 访问 `http://localhost:3001` 查看Web界面，或调用 API `GET /api/messages`

## 🎉 总结

现在您已经拥有了完整的图片发送和接收功能：

- ✅ 向3001端口发送文本+图片消息
- ✅ 自动保存和处理接收到的图片  
- ✅ Web界面显示图片信息
- ✅ 兼容原有的文本消息功能
- ✅ 与AI转发功能的多图片支持互补

享受您的多媒体消息系统吧！🚀
