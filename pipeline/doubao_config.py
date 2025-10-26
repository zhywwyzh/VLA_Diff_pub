#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from key import DOUBAO_API_KEY, DOUBAO_BASE_URL, DOUBAO_ENDPOINT_MAPPING, DEFAULT_DOUBAO_ENDPOINT, DOUBAO_EXTRA_HEADERS

"""
豆包模型配置文件
请在这里配置您的接入点ID
"""

def get_doubao_endpoint(model_name):
    """获取豆包模型的实际接入点ID"""
    endpoint = DOUBAO_ENDPOINT_MAPPING.get(model_name, DEFAULT_DOUBAO_ENDPOINT)
    
    # 检查是否为占位符
    if endpoint.startswith('YOUR_') and endpoint.endswith('_ENDPOINT_ID'):
        print(f"⚠️  模型 {model_name} 的接入点ID未配置，使用默认接入点")
        return DEFAULT_DOUBAO_ENDPOINT
    
    return endpoint

def is_doubao_endpoint_configured(model_name):
    """检查豆包模型接入点是否已配置"""
    endpoint = DOUBAO_ENDPOINT_MAPPING.get(model_name, '')
    return not (endpoint.startswith('YOUR_') and endpoint.endswith('_ENDPOINT_ID'))

# 使用示例
if __name__ == "__main__":
    print("豆包模型配置测试:")
    for model_name in DOUBAO_ENDPOINT_MAPPING:
        endpoint = get_doubao_endpoint(model_name)
        configured = is_doubao_endpoint_configured(model_name)
        status = "✅ 已配置" if configured else "⚠️ 需配置"
        print(f"  {model_name} -> {endpoint} ({status})")
