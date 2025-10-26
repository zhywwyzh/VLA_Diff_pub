#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
OpenAI GPT-4o 文字+图片测试程序
用于直接测试OpenAI API的文字和图片处理能力
"""

import os
import base64
import json
from openai import OpenAI
from PIL import Image
import io
from key import OPENAI_API_KEY

# OpenAI API配置

def image_to_base64(image_path, target_size=(640, 480)):
    """将图片转换为base64格式"""
    try:
        # 打开图片
        with Image.open(image_path) as img:
            print(f"原始图片尺寸: {img.size}")
            print(f"原始图片模式: {img.mode}")
            
            # 调整图片大小
            if img.size != target_size:
                img = img.resize(target_size, Image.Resampling.LANCZOS)
                print(f"调整后尺寸: {img.size}")
            
            # 转换为RGB模式（如果不是的话）
            if img.mode != 'RGB':
                img = img.convert('RGB')
                print(f"转换后模式: {img.mode}")
            
            # 转换为base64
            buffer = io.BytesIO()
            img.save(buffer, format='JPEG', quality=85, optimize=True)
            buffer.seek(0)
            image_data = buffer.getvalue()
            image_base64 = base64.b64encode(image_data).decode('utf-8')
            
            # 计算大小
            size_mb = len(image_data) / (1024 * 1024)
            print(f"图片转换完成: 格式=JPEG, 大小={size_mb:.2f}MB")
            
            return f"data:image/jpeg;base64,{image_base64}"
            
    except Exception as e:
        print(f"图片处理错误: {e}")
        return None

def test_openai_api(text_prompt, image_path=None, model="gpt-4o"):
    """测试OpenAI API"""
    try:
        # 初始化客户端
        client = OpenAI(api_key=OPENAI_API_KEY)
        print(f"使用模型: {model}")
        print(f"文字prompt: {text_prompt}")
        print(f"图片路径: {image_path}")
        print("-" * 50)
        
        # 构建消息
        messages = [
            {
                "role": "system",
                "content": "你是一个智能助手，能够分析图片和回答问题。请用中文回复。"
            }
        ]
        
        # 构建用户消息内容
        user_content = []
        
        # 添加文字
        if text_prompt:
            user_content.append({
                "type": "text",
                "text": text_prompt
            })
        
        # 添加图片
        if image_path and os.path.exists(image_path):
            image_url = image_to_base64(image_path)
            if image_url:
                user_content.append({
                    "type": "image_url",
                    "image_url": {
                        "url": image_url,
                        "detail": "high"
                    }
                })
            else:
                print("图片处理失败，只发送文字")
        
        # 添加用户消息
        messages.append({
            "role": "user",
            "content": user_content
        })
        
        print(f"发送消息数量: {len(messages)}")
        print(f"用户内容元素数量: {len(user_content)}")
        
        # 显示实际发送的消息结构（隐藏图片数据）
        print("\n发送的消息结构:")
        for i, msg in enumerate(messages):
            print(f"消息 {i+1}:")
            print(f"  角色: {msg['role']}")
            if msg['role'] == 'user':
                for j, content in enumerate(msg['content']):
                    if content['type'] == 'text':
                        print(f"  内容 {j+1}: 文字 - {content['text'][:100]}...")
                    elif content['type'] == 'image_url':
                        print(f"  内容 {j+1}: 图片 - detail={content['image_url']['detail']}")
            else:
                print(f"  内容: {msg['content']}")
        
        print("-" * 50)
        print("正在调用OpenAI API...")
        
        # 调用API
        response = client.chat.completions.create(
            model=model,
            messages=messages,
            max_tokens=2048,
            temperature=0.7,
            timeout=180
        )
        
        # 获取响应
        if response.choices and len(response.choices) > 0:
            content = response.choices[0].message.content
            finish_reason = response.choices[0].finish_reason
            
            print(f"\n✅ API调用成功!")
            print(f"完成原因: {finish_reason}")
            print(f"回复长度: {len(content)} 字符")
            print(f"回复内容:\n{content}")
            
            # 检查token使用情况
            if hasattr(response, 'usage'):
                usage = response.usage
                print(f"\nToken使用情况:")
                print(f"  输入tokens: {usage.prompt_tokens}")
                print(f"  输出tokens: {usage.completion_tokens}")
                print(f"  总计tokens: {usage.total_tokens}")
            
            return True, content
        else:
            print("❌ API返回空响应")
            return False, "空响应"
            
    except Exception as e:
        print(f"❌ API调用失败: {e}")
        return False, str(e)

def main():
    """主测试函数"""
    print("=" * 60)
    print("OpenAI GPT-4o 文字+图片测试程序")
    print("=" * 60)
    
    # 测试用例 - 渐进式策略
    test_cases = [
        {
            "name": "策略1：图像坐标分析",
            "text": "请分析这张图片的空间布局。假设图片尺寸是640x480像素，请估算左侧饮水机右后方地面上某个明显特征点的像素坐标位置。请用(x,y)格式给出坐标。",
            "image": "/Users/zzmm4/Desktop/20250820-192524.jpg",
            "model": "gpt-4o"
        },
        {
            "name": "策略2：导航角度计算",
            "text": "这是一个室内导航场景。假设一个移动设备当前面向正前方(0度)，如果要转向左侧通道入口，需要向左转多少度？请只给出角度数值。",
            "image": "/Users/zzmm4/Desktop/20250820-192524.jpg",
            "model": "gpt-4o"
        },
        {
            "name": "策略3：组合任务（温和版）",
            "text": "请帮我完成室内定位任务：1）在图片中找到左侧饮水机右后方的地面位置，估算其在640x480像素图像中的坐标；2）计算从当前视角转向该位置需要的角度。请按格式回答：坐标(x,y)，转向角度z度。",
            "image": "/Users/zzmm4/Desktop/20250820-192524.jpg",
            "model": "gpt-4o"
        },
        {
            "name": "策略4：技术术语版",
            "text": "图像分析任务：在640x480分辨率的图像中，标定左侧饮水机右后方约0.5米处通道入口中线与地砖交汇点的像素坐标。同时计算移动平台从当前朝向转向该位置的旋转角度。输出格式：(x,y),角度。",
            "image": "/Users/zzmm4/Desktop/20250820-192524.jpg",
            "model": "gpt-4o"
        },
        {
            "name": "策略5：分步骤版",
            "text": "请逐步分析：第一步，在图片中识别左侧饮水机的位置；第二步，找到其右后方约半米的地面区域；第三步，估算该位置在图片中的像素坐标(以640x480为基准)；第四步，计算从正前方视角转向该位置的角度。最后用格式(x,y),角度回答。",
            "image": "/Users/zzmm4/Desktop/20250820-192524.jpg",
            "model": "gpt-4o"
        }
    ]
    
    # 被注释掉的其他测试用例
    """
    其他测试用例:
        # {
        #     "name": "文字+图片测试（gpt-4o-mini）",
        #     "text": "请描述这张图片中的内容。",
        #     "image": "/Users/zzmm4/Desktop/20250820-192524.jpg",  # 请替换为实际图片路径
        #     "model": "gpt-4o-mini"
        # },
        # {
        #     "name": "复杂prompt+图片测试",
        #     "text": "对于这张图片，我需要你完成如下任务。任务一，给我找到左侧饮水机右后方约半米处，正好在通道入口的几何中线位置，落在第一块浅色地砖的中心上，在图中的二维坐标，只给我坐标，其他什么都不要。任务二，我现在是一架飞机，要进入到左侧通道继续往通道深处走，你帮我计算一下我这样走了之后，飞机旋转的角度大概是多少。另外我的飞机此时并不是正对着前方的，你得仔细注意一下我的飞机视角偏左还是偏右，偏差多少，再计算我再应该向左转多少度。直接给我转动多少度的答案，其他的什么都不要。最终的答案应该是(x,y),z。",
        #     "image": "/Users/zzmm4/Desktop/20250820-192524.jpg",
        #     "model": "gpt-4o"
        # }
    """
    
    for i, test_case in enumerate(test_cases, 1):
        print(f"\n🧪 测试 {i}: {test_case['name']}")
        print("=" * 60)
        
        # 检查图片文件是否存在
        if test_case['image'] and not os.path.exists(test_case['image']):
            print(f"⚠️  图片文件不存在: {test_case['image']}")
            print("跳过此测试...")
            continue
        
        success, result = test_openai_api(
            text_prompt=test_case['text'],
            image_path=test_case['image'],
            model=test_case['model']
        )
        
        if success:
            print(f"✅ 测试 {i} 成功")
        else:
            print(f"❌ 测试 {i} 失败: {result}")
        
        print("\n" + "=" * 60)
        
        # 询问是否继续
        if i < len(test_cases):
            user_input = input("\n按Enter继续下一个测试，或输入'q'退出: ").strip().lower()
            if user_input == 'q':
                break

if __name__ == "__main__":
    main()
