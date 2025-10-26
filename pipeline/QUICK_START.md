# 🚀 快速开始指南

## 1. 一键启动所有服务
```bash
./start_all.sh
```

## 2. 测试转发功能
1. 打开浏览器访问聊天界面
2. 与AI对话
3. 运行客户端程序查看转发的消息：

### 简单客户端（推荐初学者）
```bash
python simple_client.py
```

### 高级客户端（完整功能）
```bash
python client_example.py
```

## 3. 修改端口配置

### 主服务端口 (默认 1024)
编辑 `app.py`：
```python
MAIN_SERVICE_PORT = 1024  # 改为您想要的端口
```

### 外部接收器端口 (默认 3000)
编辑 `external_receiver_example.py`：
```python
EXTERNAL_RECEIVER_PORT = 3000  # 改为您想要的端口
```

编辑 `app.py` 中的转发配置：
```python
DEFAULT_FORWARD_PORT = 3000  # 改为与外部接收器相同的端口
```

### 启动脚本端口
编辑 `start_all.sh`：
```bash
MAIN_PORT=1024        # 主服务端口
EXTERNAL_PORT=3000    # 外部接收器端口
```

## 4. 自定义消息处理

复制并修改示例程序：
```bash
cp simple_client.py my_client.py
# 编辑 my_client.py 中的 process_message() 函数
```

## 5. 检查服务状态

```bash
# 检查主服务
curl http://localhost:1024/api/test

# 检查外部接收器
curl http://localhost:3000/api/health

# 查看转发的消息
curl http://localhost:3000/api/messages
```

## 6. 常用命令

```bash
# 查看日志
tail -f logs/main_service.log
tail -f logs/external_receiver.log

# 停止所有服务
./stop_all.sh

# 重启所有服务
./restart_all.sh
```

## 🆘 遇到问题？

1. **服务无法启动** → 检查端口是否被占用
2. **转发不工作** → 确认两个服务都在运行，检查转发URL配置
3. **无法连接** → 检查防火墙设置，确认端口配置正确

详细文档请查看 `README.md`
