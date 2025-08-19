from openai import OpenAI
import cv2
import base64
import re
import json
from utils.vlm.prompt import SYSTEM_PROMPT, USER1, ASSISTANT1, USER2, ASSISTANT2


# 初始化客户端
client = OpenAI(api_key='EMPTY', base_url='http://0.0.0.0:6006/v1')

def open_serve(img, input):
    """
    传入 OpenCV 读取的图像对象，返回模型识别出的红杯子像素坐标(JSON格式)。
    
    参数:
        img: OpenCV 读取的 BGR 图像 (numpy.ndarray)
    
    返回:
        模型返回的 JSON 字符串或解析后的 Python 对象
    """
    # input = "door"
    # 将图像编码为base64
    _, buffer = cv2.imencode('.jpg', img)
    img_base64 = base64.b64encode(buffer).decode('utf-8')
    data_url = f'data:image/jpeg;base64,{img_base64}'

    # 获取模型名称
    model_name = client.models.list().data[0].id

    # 构造请求
    response = client.chat.completions.create(
        model=model_name,
        messages=[
            {
                'role':'system',
                'content':[{
                    'type': 'text',
                    'text': SYSTEM_PROMPT
                }]
            },
            {
                'role':'user',
                'content':[{
                    'type': 'text',
                    'text': USER1
                }]
            },
            {
                'role':'assistant',
                'content':[{
                    'type': 'text',
                    'text': ASSISTANT1
                }]
            },
            {
                'role':'user',
                'content':[{
                    'type': 'text',
                    'text': USER2
                }]
            },
            {
                'role':'assistant',
                'content':[{
                    'type': 'text',
                    'text': ASSISTANT2
                }]
            },
            {
                'role': 'user',
                'content': [{
                    'type': 'text',
                    'text':f'{input}'
                }, {
                    'type': 'image_url',
                    'image_url': {
                        'url': data_url,
                    },
                }],
        }],
        temperature=0.8,
        top_p=0.8
    )

    # 返回内容部分
    # 获取字符串内容
    content = response.choices[0].message.content

    # 去掉 ```json ... ``` 包裹
    json_str = re.sub(r"^```json\s*|\s*```$", "", content.strip(), flags=re.DOTALL)

    # 转成 Python 对象
    try:
        return json.loads(json_str)
    except json.JSONDecodeError:
        raise ValueError(f"模型返回的JSON解析失败: {json_str}")

# 使用示例
if __name__ == "__main__":
    img_path = '/home/zhywwyzh/workspace/test_vlm/test_img/rgb.jpg'
    img = cv2.imread(img_path)
    result = open_serve(img)
    print(result)

# vllm serve /home/zhywwyzh/Modelscope/qwen2.5-vl-7B-Instruct-AWQ --dtype auto --port 6006 --max-model-len 1000 --gpu-memory-utilization 0.8