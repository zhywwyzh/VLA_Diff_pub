#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
直接测试Gemini API连接
用于验证API密钥和连接问题
"""

import google.generativeai as genai
import sys
import time
from key import GEMINI_API_KEY

# 您的API密钥
API_KEY = GEMINI_API_KEY

def test_api_key():
    """测试API密钥"""
    print("🔑 测试API密钥...")
    
    try:
        genai.configure(api_key=API_KEY)
        print("✅ API密钥配置成功")
        return True
    except Exception as e:
        print(f"❌ API密钥配置失败: {e}")
        return False

def test_list_models():
    """测试列出模型"""
    print("\n📋 测试列出可用模型...")
    
    try:
        models = list(genai.list_models())
        print(f"✅ 成功获取模型列表 ({len(models)} 个模型)")
        
        # 检查是否有我们需要的模型
        target_model = 'gemini-2.5-flash-lite'
        found_model = False
        
        for model in models:
            print(f"   • {model.name}")
            if target_model in model.name:
                found_model = True
        
        if found_model:
            print(f"✅ 找到目标模型: {target_model}")
        else:
            print(f"⚠️  未找到目标模型: {target_model}")
            print("可用的类似模型:")
            for model in models:
                if 'flash' in model.name.lower() or 'lite' in model.name.lower():
                    print(f"   • {model.name}")
        
        return True
        
    except Exception as e:
        print(f"❌ 获取模型列表失败: {e}")
        return False

def test_simple_generation():
    """测试简单的文本生成"""
    print("\n💬 测试简单文本生成...")
    
    try:
        # 尝试不同的模型名称
        model_names = [
            'gemini-2.5-flash-lite',
            'gemini-1.5-flash',
            'gemini-pro'
        ]
        
        for model_name in model_names:
            try:
                print(f"   尝试模型: {model_name}")
                model = genai.GenerativeModel(model_name)
                
                # 简单测试
                response = model.generate_content(
                    "请用中文说一句'你好'",
                    generation_config={
                        "temperature": 0.1,
                        "max_output_tokens": 50,
                    }
                )
                
                if response and response.text:
                    print(f"✅ 模型 {model_name} 测试成功")
                    print(f"   响应: {response.text}")
                    return True
                else:
                    print(f"⚠️  模型 {model_name} 响应为空")
                    
            except Exception as model_error:
                print(f"❌ 模型 {model_name} 测试失败: {model_error}")
                continue
        
        print("❌ 所有模型测试都失败了")
        return False
        
    except Exception as e:
        print(f"❌ 文本生成测试失败: {e}")
        return False

def test_with_timeout():
    """使用短超时测试"""
    print("\n⏱️  测试短超时连接...")
    
    try:
        model = genai.GenerativeModel('gemini-1.5-flash')
        
        start_time = time.time()
        response = model.generate_content(
            "Hello",
            request_options={"timeout": 10}  # 10秒超时
        )
        end_time = time.time()
        
        print(f"✅ 短超时测试成功 (耗时: {end_time - start_time:.2f}秒)")
        print(f"   响应: {response.text}")
        return True
        
    except Exception as e:
        print(f"❌ 短超时测试失败: {e}")
        return False

def main():
    print("=" * 60)
    print("🧪 Gemini API 直接连接测试")
    print("=" * 60)
    
    success_count = 0
    total_tests = 4
    
    # 测试1: API密钥
    if test_api_key():
        success_count += 1
    
    # 测试2: 列出模型
    if test_list_models():
        success_count += 1
    
    # 测试3: 简单生成
    if test_simple_generation():
        success_count += 1
    
    # 测试4: 短超时测试
    if test_with_timeout():
        success_count += 1
    
    print("\n" + "=" * 60)
    print("📊 测试结果总结:")
    print("=" * 60)
    print(f"通过测试: {success_count}/{total_tests}")
    
    if success_count == total_tests:
        print("🎉 所有测试通过！Gemini API连接正常。")
        print("问题可能出在:")
        print("• 应用程序的网络配置")
        print("• 超时设置太短")
        print("• 并发请求问题")
    elif success_count >= 2:
        print("⚠️  部分测试通过，连接存在问题但基本可用。")
        print("建议:")
        print("• 增加超时时间")
        print("• 使用重试机制")
        print("• 检查防火墙设置")
    else:
        print("❌ 大部分测试失败，存在严重的连接问题。")
        print("可能原因:")
        print("• API密钥无效或过期")
        print("• 网络防火墙阻止了连接")
        print("• 地区限制")
        print("• 需要VPN")

if __name__ == "__main__":
    main()
