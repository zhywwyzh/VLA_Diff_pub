#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
最简单的Gemini API测试
"""

import google.generativeai as genai
from key import GEMINI_API_KEY

# 配置API密钥
genai.configure(api_key=GEMINI_API_KEY)

# 创建模型
model = genai.GenerativeModel('gemini-2.5-flash-lite')

print("🧪 测试最简单的Gemini API调用...")

try:
    # 最简单的API调用
    response = model.generate_content("你好，请用中文回复我")
    
    if response and response.text:
        print("✅ API调用成功！")
        print(f"回复: {response.text}")
    else:
        print("❌ API返回空响应")
        
except Exception as e:
    print(f"❌ API调用失败: {e}")
