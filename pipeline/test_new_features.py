#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
新功能测试脚本 - 模型切换和多图片上传
"""

import requests
import json
import time
import base64
import sys
from PIL import Image
import io

def create_test_image(width=200, height=200, color='red'):
    """创建测试图片"""
    img = Image.new('RGB', (width, height), color)
    buffer = io.BytesIO()
    img.save(buffer, format='PNG')
    buffer.seek(0)
    
    # 转换为base64
    img_data = base64.b64encode(buffer.read()).decode('utf-8')
    return f"data:image/png;base64,{img_data}"

def test_service_status():
    """测试服务状态"""
    print("🔍 测试服务状态...")
    try:
        response = requests.get('http://localhost:1024/api/test', timeout=10)
        if response.status_code == 200:
            result = response.json()
            print("✅ 主服务正常运行")
            print(f"   - 当前模型: {result.get('model', '未知')}")
            print(f"   - 多图片支持: {result.get('features', {}).get('multi_image_support', False)}")
            print(f"   - 模型切换支持: {result.get('features', {}).get('model_switching', False)}")
            print(f"   - 最大图片数: {result.get('features', {}).get('max_images', 0)}")
            return True, result
        else:
            print(f"❌ 服务响应异常: HTTP {response.status_code}")
            return False, None
    except Exception as e:
        print(f"❌ 服务连接失败: {e}")
        return False, None

def test_model_switching():
    """测试模型切换功能"""
    print("\n🔄 测试模型切换功能...")
    
    # 获取可用模型
    try:
        response = requests.get('http://localhost:1024/api/config/model', timeout=5)
        if response.status_code == 200:
            result = response.json()
            current_model = result.get('model')
            available_models = result.get('available_models', [])
            print(f"当前模型: {current_model}")
            print(f"可用模型: {', '.join(available_models)}")
        else:
            print("❌ 无法获取模型信息")
            return False
    except Exception as e:
        print(f"❌ 获取模型信息失败: {e}")
        return False
    
    # 测试切换到不同的模型
    test_models = ['gemini-2.5-flash', 'gemini-2.5-flash-lite']
    
    for model_name in test_models:
        if model_name == current_model:
            continue
            
        print(f"\n尝试切换到: {model_name}")
        try:
            response = requests.post(
                'http://localhost:1024/api/config/model',
                json={'model': model_name},
                timeout=15
            )
            
            if response.status_code == 200:
                result = response.json()
                if result['success']:
                    print(f"✅ 成功切换到: {model_name}")
                    
                    # 测试新模型是否工作
                    test_response = requests.post(
                        'http://localhost:1024/api/chat',
                        json={'text': '你好，这是模型切换测试', 'model': model_name},
                        timeout=30
                    )
                    
                    if test_response.status_code == 200:
                        chat_result = test_response.json()
                        if chat_result['success']:
                            print(f"✅ 新模型响应正常: {chat_result['response']['text'][:50]}...")
                        else:
                            print(f"❌ 新模型响应失败: {chat_result.get('error', '未知错误')}")
                    else:
                        print(f"❌ 新模型聊天测试失败: HTTP {test_response.status_code}")
                        
                    # 切换回原模型
                    requests.post(
                        'http://localhost:1024/api/config/model',
                        json={'model': current_model},
                        timeout=15
                    )
                    break
                else:
                    print(f"❌ 切换失败: {result.get('error', '未知错误')}")
            else:
                print(f"❌ 切换请求失败: HTTP {response.status_code}")
        except Exception as e:
            print(f"❌ 模型切换异常: {e}")
    
    return True

def test_multi_image_upload():
    """测试多图片上传功能"""
    print("\n🖼️ 测试多图片上传功能...")
    
    # 创建多个测试图片
    test_images = [
        create_test_image(150, 150, 'red'),
        create_test_image(200, 200, 'green'),
        create_test_image(180, 180, 'blue')
    ]
    
    print(f"创建了 {len(test_images)} 张测试图片")
    
    # 测试多图片聊天
    try:
        response = requests.post(
            'http://localhost:1024/api/chat',
            json={
                'text': '请描述这些图片的颜色和特征',
                'images': test_images
            },
            timeout=45
        )
        
        if response.status_code == 200:
            result = response.json()
            if result['success']:
                print("✅ 多图片聊天成功")
                print(f"   AI回复: {result['response']['text'][:100]}...")
                return True
            else:
                print(f"❌ 多图片聊天失败: {result.get('error', '未知错误')}")
                return False
        else:
            print(f"❌ 多图片聊天请求失败: HTTP {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ 多图片聊天异常: {e}")
        return False

def test_combined_features():
    """测试组合功能"""
    print("\n🔀 测试组合功能...")
    
    # 创建测试图片
    test_image = create_test_image(200, 200, 'purple')
    
    # 使用特定模型进行多模态对话
    try:
        response = requests.post(
            'http://localhost:1024/api/chat',
            json={
                'text': '这是一张什么颜色的图片？请用中文简短回答。',
                'images': [test_image],
                'model': 'gemini-2.5-flash'
            },
            timeout=30
        )
        
        if response.status_code == 200:
            result = response.json()
            if result['success']:
                print("✅ 组合功能测试成功")
                print(f"   AI回复: {result['response']['text']}")
                return True
            else:
                print(f"❌ 组合功能测试失败: {result.get('error', '未知错误')}")
                return False
        else:
            print(f"❌ 组合功能请求失败: HTTP {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ 组合功能测试异常: {e}")
        return False

def main():
    """主测试函数"""
    print("🧪 新功能测试 - 模型切换 & 多图片上传")
    print("=" * 50)
    
    # 测试服务状态
    service_ok, service_info = test_service_status()
    if not service_ok:
        print("\n❌ 服务不可用，请先启动服务")
        return False
    
    print("\n" + "-" * 30)
    
    # 测试模型切换
    model_ok = test_model_switching()
    
    print("\n" + "-" * 30)
    
    # 测试多图片上传
    image_ok = test_multi_image_upload()
    
    print("\n" + "-" * 30)
    
    # 测试组合功能
    combined_ok = test_combined_features()
    
    print("\n" + "=" * 50)
    
    # 统计结果
    total_tests = 4
    passed_tests = sum([service_ok, model_ok, image_ok, combined_ok])
    
    print(f"🎯 测试结果: {passed_tests}/{total_tests} 项测试通过")
    
    if passed_tests == total_tests:
        print("🎉 所有新功能测试通过！")
        print("\n✨ 新功能说明:")
        print("   1. 模型切换: 在状态栏下拉菜单中选择不同的Gemini模型")
        print("   2. 多图片上传: 一次可以选择并上传最多8张图片")
        print("   3. 智能处理: AI能同时分析多张图片并给出综合回答")
        print("   4. 兼容性: 保持向后兼容，支持单图片上传")
        return True
    else:
        print("⚠️ 部分功能测试失败，请检查服务状态")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
