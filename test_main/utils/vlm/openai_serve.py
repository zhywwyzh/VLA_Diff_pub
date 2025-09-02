from openai import OpenAI
import cv2
import base64
import re
import json
from utils.vlm.prompt import SYSTEM_PROMPT, USER1, ASSISTANT1, USER2, ASSISTANT2
import os

os.environ.pop("HTTP_PROXY", None)
os.environ.pop("HTTPS_PROXY", None)
os.environ.pop("ALL_PROXY", None)
os.environ.pop("http_proxy", None)
os.environ.pop("https_proxy", None)
os.environ.pop("all_proxy", None)

# 初始化客户端
client = OpenAI(api_key='EMPTY', base_url='http://127.0.0.1:6006/v1')

def open_serve(img_ori, img_cur, input):
    """
    传入 OpenCV 读取的图像对象，返回模型识别出的红杯子像素坐标(JSON格式)。
    
    参数:
        img: OpenCV 读取的 BGR 图像 (numpy.ndarray)
    
    返回:
        模型返回的 JSON 字符串或解析后的 Python 对象
    """
    finish_mission = False
    # input = "door"
    # 将图像编码为base64
    _, buffer_ori = cv2.imencode('.jpg', img_ori)
    img_ori_base64 = base64.b64encode(buffer_ori).decode('utf-8')
    data_url_ori = f'data:image/jpeg;base64,{img_ori_base64}'

    _, buffer_cur = cv2.imencode('.jpg', img_cur)
    img_cur_base64 = base64.b64encode(buffer_cur).decode('utf-8')
    data_url_cur = f'data:image/jpeg;base64,{img_cur_base64}'

    # 获取模型名称
    model_name = client.models.list().data[0].id

    # 构造请求
    response = client.chat.completions.create(
        model=model_name,
        messages=[
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
                        'url': data_url_ori,
                    },
                },{
                    'type': 'image_url',
                    'image_url': {
                        'url': data_url_cur,
                    },
                },
                ],
        }],
        temperature=0.8,
        top_p=0.8
    )

    # 返回内容部分
    # 获取字符串内容
    content = response.choices[0].message.content
    print(f"模型返回内容: {content}")

    coord_match = re.search(r"\(([\d\s,]+)\)", content)
    if coord_match:
        coord_str = coord_match.group(1)
        loc_in_rgb = [int(x.strip()) for x in coord_str.split(',')]
    else:
        loc_in_rgb = None

    bool_match = re.search(r"(True|False)", content)
    if bool_match:
        finish_mission = bool_match.group(1) == "True"
    else:
        finish_mission = None

    # 去掉 ```json ... ``` 包裹
    # json_str = re.sub(r"^```json\s*|\s*```$", "", content.strip(), flags=re.DOTALL)

    # 转成 Python 对象
    try:
        # return json.loads(json_str)
        return loc_in_rgb, finish_mission
    except json.JSONDecodeError:
        raise ValueError(f"模型返回的JSON解析失败: {content}")

# 使用示例
if __name__ == "__main__":
    img_path = '/home/zhywwyzh/workspace/test_vlm/test_img/rgb.jpg'
    img = cv2.imread(img_path)
    result = open_serve(img)
    print(result)

# vllm serve /home/zhywwyzh/Modelscope/qwen2.5-vl-7B-Instruct-AWQ --dtype auto --port 6006 --max-model-len 3000 --gpu-memory-utilization 0.8