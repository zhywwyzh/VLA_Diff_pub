#!/usr/bin/env python3
import requests
import json

# 测试API
def test_api():
    url = "http://localhost:8080/api/test"
    try:
        response = requests.get(url, timeout=5)
        print(f"测试接口状态: {response.status_code}")
        print(f"响应: {response.json()}")
    except Exception as e:
        print(f"测试接口失败: {e}")

    # 测试聊天API
    url = "http://localhost:8080/api/chat"
    data = {
        "text": "你好",
        "image": None,
        "audio": None
    }
    
    try:
        response = requests.post(url, json=data, timeout=30)
        print(f"\n聊天接口状态: {response.status_code}")
        if response.status_code == 200:
            result = response.json()
            print(f"成功: {result.get('success')}")
            if result.get('success'):
                print(f"AI回复: {result['response']['text']}")
            else:
                print(f"错误: {result.get('error')}")
        else:
            print(f"HTTP错误: {response.text}")
    except Exception as e:
        print(f"聊天接口失败: {e}")

if __name__ == "__main__":
    test_api()
