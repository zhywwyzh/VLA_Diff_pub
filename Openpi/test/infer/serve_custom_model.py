#!/usr/bin/env python3
"""
自定义模型服务器启动脚本
用于启动您自己的模型进行评测

使用方法:
python serve_custom_model.py --config your_config --checkpoint your_checkpoint --port 8000
"""

import argparse
import logging
import socket
import sys
import os

# 添加必要的路径
sys.path.append('/data/vla/VLA_Diff/Openpi/src')

from openpi.policies import policy_config
from openpi.serving import websocket_policy_server
from openpi.training import config as _config


def create_custom_policy(config_name, checkpoint_path, default_prompt=None):
    """
    创建自定义策略模型
    
    Args:
        config_name: 配置名称 (例如: "pi0_uav_low_mem_finetune")
        checkpoint_path: 检查点路径
        default_prompt: 默认提示词
    """
    try:
        # 获取配置
        config = _config.get_config(config_name)
        
        # 创建训练好的策略
        policy = policy_config.create_trained_policy(
            config, 
            checkpoint_path, 
            default_prompt=default_prompt
        )
        
        return policy
    except Exception as e:
        logging.error(f"创建策略时出错: {e}")
        raise


def main():
    parser = argparse.ArgumentParser(description="启动自定义模型服务器")
    parser.add_argument("--config", required=True, help="模型配置名称")
    parser.add_argument("--checkpoint", required=True, help="模型检查点路径")
    parser.add_argument("--port", type=int, default=8000, help="服务器端口")
    parser.add_argument("--host", default="0.0.0.0", help="服务器主机地址")
    parser.add_argument("--prompt", help="默认提示词")
    
    args = parser.parse_args()
    
    # 检查检查点路径是否存在
    if not os.path.exists(args.checkpoint):
        logging.error(f"检查点路径不存在: {args.checkpoint}")
        return
    
    logging.info(f"正在加载模型...")
    logging.info(f"配置: {args.config}")
    logging.info(f"检查点: {args.checkpoint}")
    
    try:
        # 创建策略
        policy = create_custom_policy(
            config_name=args.config,
            checkpoint_path=args.checkpoint,
            default_prompt=args.prompt
        )
        
        policy_metadata = policy.metadata
        
        # 获取本机IP
        hostname = socket.gethostname()
        try:
            local_ip = socket.gethostbyname(hostname)
        except socket.gaierror:
            local_ip = "127.0.0.1"
            logging.warning(f"无法解析主机名 {hostname}, 使用 {local_ip} 代替。")
        
        logging.info(f"正在创建服务器 (host: {hostname}, ip: {local_ip})")
        
        # 启动 WebSocket 服务器
        server = websocket_policy_server.WebsocketPolicyServer(
            policy=policy,
            host=args.host,
            port=args.port,
            metadata=policy_metadata,
        )
        
        logging.info(f"模型服务器已启动在 ws://{local_ip}:{args.port}")
        logging.info("等待客户端连接...")
        server.serve_forever()
        
    except Exception as e:
        logging.error(f"启动服务器时出错: {e}")
        raise


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO, 
        format='%(asctime)s - %(levelname)s - %(message)s',
        force=True
    )
    main()