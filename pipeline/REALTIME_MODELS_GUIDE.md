# 实时语音模型使用指南

本文档介绍如何使用新增的GPT-4o-Realtime和GLM-Realtime实时语音模型。

## 🎯 功能概述

新增了两个实时语音模型：
- **GPT-4o-Realtime-Preview**: OpenAI的实时语音对话模型
- **GLM-Realtime**: 智谱AI的实时语音对话模型

### 特性对比

| 特性 | 传统模型 | 实时模型 |
|------|----------|----------|
| 通信方式 | HTTP REST API | WebSocket |
| 延迟 | 较高（数秒） | 极低（毫秒级） |
| 打断支持 | ❌ | ✅ |
| 流式音频 | ❌ | ✅ |
| 实时对话 | ❌ | ✅ |

## 📋 API 端点

### 1. 获取模型列表
```bash
GET /api/config/model
```

响应包含新增的实时模型：
```json
{
  "available_models": [
    "gemini-2.5-flash-lite",
    "gpt-4o",
    "gpt-4o-mini",
    "gpt-4o-realtime-preview",
    "glm-realtime"
  ]
}
```

### 2. 检查实时模型状态
```bash
GET /api/realtime/status
```

响应示例：
```json
{
  "success": true,
  "realtime_models": {
    "gpt-4o-realtime-preview": {
      "connected": false,
      "session_id": null
    },
    "glm-realtime": {
      "connected": true,
      "session_id": "session_1234567890"
    }
  },
  "supported_models": ["gpt-4o-realtime-preview", "glm-realtime"]
}
```

### 3. 连接实时模型
```bash
POST /api/realtime/connect
Content-Type: application/json

{
  "model": "gpt-4o-realtime-preview"
}
```

成功响应：
```json
{
  "success": true,
  "message": "gpt-4o-realtime-preview 连接成功",
  "model": "gpt-4o-realtime-preview",
  "session_id": "session_xxx"
}
```

### 4. 发送消息
```bash
POST /api/realtime/send
Content-Type: application/json

{
  "model": "gpt-4o-realtime-preview",
  "text": "你好，请回复我"
}
```

或发送音频：
```bash
POST /api/realtime/send
Content-Type: application/json

{
  "model": "gpt-4o-realtime-preview",
  "audio": "data:audio/wav;base64,UklGRnoG..."
}
```

### 5. 断开连接
```bash
POST /api/realtime/disconnect
Content-Type: application/json

{
  "model": "gpt-4o-realtime-preview"
}
```

## 🔄 API 隔离机制

实时模型与传统模型完全隔离：

### 传统模型使用
```bash
POST /api/chat
{
  "model": "gpt-4o",
  "text": "你好"
}
```
✅ 正常处理

### 实时模型误用检测
```bash
POST /api/chat
{
  "model": "gpt-4o-realtime-preview",
  "text": "你好"
}
```
❌ 返回错误：
```json
{
  "success": false,
  "error": "实时语音模型 gpt-4o-realtime-preview 需要使用WebSocket连接，请使用 /api/realtime 端点",
  "redirect": "/api/realtime",
  "model_type": "realtime"
}
```

## 🎮 使用流程

### 基本流程
1. **连接** → 调用 `/api/realtime/connect`
2. **发送** → 调用 `/api/realtime/send`
3. **监听** → 通过WebSocket接收响应
4. **断开** → 调用 `/api/realtime/disconnect`

### 示例代码

#### Python 示例
```python
import requests
import json

BASE_URL = "http://localhost:1024"

# 1. 连接
connect_resp = requests.post(f"{BASE_URL}/api/realtime/connect", 
                           json={"model": "gpt-4o-realtime-preview"})

if connect_resp.status_code == 200:
    print("✅ 连接成功")
    
    # 2. 发送消息
    send_resp = requests.post(f"{BASE_URL}/api/realtime/send",
                            json={
                                "model": "gpt-4o-realtime-preview",
                                "text": "你好，请回复我"
                            })
    
    if send_resp.status_code == 200:
        print("✅ 消息发送成功")
        print("💡 请通过WebSocket监听响应")
    
    # 3. 断开连接
    requests.post(f"{BASE_URL}/api/realtime/disconnect",
                 json={"model": "gpt-4o-realtime-preview"})
```

#### JavaScript 示例
```javascript
const BASE_URL = "http://localhost:1024";

async function useRealtimeModel() {
    // 1. 连接
    const connectResp = await fetch(`${BASE_URL}/api/realtime/connect`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({model: "gpt-4o-realtime-preview"})
    });
    
    if (connectResp.ok) {
        console.log("✅ 连接成功");
        
        // 2. 发送消息
        const sendResp = await fetch(`${BASE_URL}/api/realtime/send`, {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({
                model: "gpt-4o-realtime-preview",
                text: "你好，请回复我"
            })
        });
        
        if (sendResp.ok) {
            console.log("✅ 消息发送成功");
            console.log("💡 请通过WebSocket监听响应");
        }
        
        // 3. 断开连接
        await fetch(`${BASE_URL}/api/realtime/disconnect`, {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({model: "gpt-4o-realtime-preview"})
        });
    }
}
```

## ⚙️ 配置说明

### OpenAI Realtime API
- **端点**: `wss://api.openai.com/v1/realtime`
- **认证**: Bearer Token (OPENAI_API_KEY)
- **模型**: `gpt-4o-realtime-preview-2024-10-01`
- **支持格式**: PCM16 音频，文本

### 智谱GLM Realtime API
- **端点**: `wss://open.bigmodel.cn/api/paas/v4/realtime`
- **认证**: Bearer Token (CHATGLM_API_KEY)
- **模型**: `GLM-Realtime-Flash` / `GLM-Realtime-Air`
- **支持格式**: PCM16 音频，文本

## 🔊 音频支持

### 输入音频格式
- **格式**: PCM16
- **采样率**: 16kHz
- **声道**: 单声道
- **编码**: Base64

### 输出音频格式
- **格式**: PCM16
- **采样率**: 24kHz (OpenAI) / 16kHz (GLM)
- **声道**: 单声道
- **传输**: 流式 WebSocket

## 🚨 注意事项

1. **网络连接**: 实时模型需要稳定的网络连接
2. **API密钥**: 确保配置了正确的API密钥
3. **并发限制**: 
   - OpenAI: 根据账户等级限制
   - 智谱: V0:5并发, V1:10并发, V2:15并发, V3:20并发
4. **费用计算**:
   - OpenAI: 按分钟计费
   - 智谱: 音频0.18-0.3元/分钟，视频1.2-2.1元/分钟

## 🧪 测试

运行测试脚本：
```bash
python3 test_realtime_models.py
```

测试内容：
- ✅ 模型列表包含实时模型
- ✅ 普通API正确拒绝实时模型
- ✅ 实时状态接口可用
- ✅ 连接/断开功能
- ✅ 消息发送功能

## 🔮 未来扩展

计划中的功能：
- [ ] WebSocket 服务端事件监听
- [ ] 前端实时语音界面
- [ ] 音频流处理优化
- [ ] 多会话管理
- [ ] 实时字幕显示

---

## 📞 技术支持

如有问题，请检查：
1. API密钥配置是否正确
2. 网络连接是否稳定
3. 服务日志中的错误信息
4. WebSocket连接是否正常

更多信息请参考：
- [OpenAI Realtime API 文档](https://platform.openai.com/docs/guides/realtime)
- [智谱GLM-Realtime API 文档](https://docs.bigmodel.cn/cn/guide/models/sound-and-video/glm-realtime)
