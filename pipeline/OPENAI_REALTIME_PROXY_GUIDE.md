# OpenAI Realtime API 连接解决方案

## 📋 问题背景

参考 `/Users/zzmm4/Desktop/realtime-voice-gpt4o` 项目的成功经验，我们发现 OpenAI Realtime API 的连接问题主要是由于网络和代理配置导致的。

## 🔧 解决方案

### 方案1: 使用系统代理 + 环境变量（推荐）

1. **启动代理软件**（如Clash、Shadowsocks等）
2. **使用代理启动脚本**：
   ```bash
   ./start_with_proxy.sh
   ```
   
3. **如果代理端口不是7890，请编辑脚本**：
   ```bash
   nano start_with_proxy.sh
   # 修改端口号，常见端口：
   # - 7890 (Clash)
   # - 1080 (Shadowsocks)
   # - 8080 (其他代理)
   ```

### 方案2: 本地WebSocket代理服务器

如果方案1不行，可以使用本地代理服务器（类似成功项目的架构）：

1. **启动本地代理服务器**：
   ```bash
   # 设置代理环境变量
   export HTTPS_PROXY=http://127.0.0.1:7890
   export OPENAI_API_KEY="your_openai_api_key"
   
   # 启动代理服务器
   python3 websocket_proxy_server.py
   ```

2. **在另一个终端启动主服务**：
   ```bash
   # 配置使用本地代理
   curl -X POST http://localhost:1024/api/realtime/proxy-config \
        -H "Content-Type: application/json" \
        -d '{"use_local_proxy_for_gpt": true}'
   
   # 启动主服务
   python3 app.py
   ```

## 🎯 连接流程

1. **选择 GPT-4o-Realtime 模型**
2. **点击"连接实时模型"**
3. **等待连接成功提示**
4. **在下方输入框开始对话**

## 🔍 技术原理

### 成功项目的架构
- **前端** ↔ **本地代理服务器** ↔ **OpenAI API**
- 使用 Node.js + `HttpsProxyAgent` 处理代理连接
- WebSocket 双向转发消息

### 我们的实现
- **Python Flask** + **WebSocket代理服务器**
- 使用 `aiohttp.ProxyConnector` 处理代理
- 支持直连和代理两种模式

## 📁 新增文件

- `start_with_proxy.sh` - 代理启动脚本
- `websocket_proxy_server.py` - 本地WebSocket代理服务器
- `OPENAI_REALTIME_PROXY_GUIDE.md` - 此使用指南

## 🛠️ API端点

- `GET /api/realtime/proxy-config` - 查看代理配置
- `POST /api/realtime/proxy-config` - 切换代理模式

## 💡 故障排除

1. **连接超时**：
   - 检查代理软件是否运行
   - 确认代理端口配置正确
   - 尝试使用本地代理服务器

2. **代理服务器启动失败**：
   - 检查端口8765是否被占用
   - 确认已安装 `aiohttp` 依赖

3. **API密钥问题**：
   - 确认 OpenAI API 密钥有效
   - 检查账户是否有 Realtime API 权限

## 🎉 参考项目

感谢 `realtime-voice-gpt4o` 项目的启发，其成功的代理架构为我们提供了宝贵的经验。
