from openai import OpenAI
import cv2
import base64
import re
import json
# from utils.vlm.prompt import USER3, ASSISTANT3
import os
import pdb

os.environ.pop("HTTP_PROXY", None)
os.environ.pop("HTTPS_PROXY", None)
os.environ.pop("ALL_PROXY", None)
os.environ.pop("http_proxy", None)
os.environ.pop("https_proxy", None)
os.environ.pop("all_proxy", None)

# 初始化客户端
client = OpenAI(api_key='EMPTY', base_url='http://127.0.0.1:2233/v1')

def open_serve_search(img, input1):
    """
    传入图像对象，返回模型当前是否可以看到目标物体
    """
    # 将图像编码为base64
    _, buffer = cv2.imencode('.jpg', img)
    img_base64 = base64.b64encode(buffer).decode('utf-8')
    data_url = f'data:image/jpeg;base64,{img_base64}'

    # 获取模型名称
    model_name = client.models.list().data[0].id

    input2 = input1.replace("前往", "").strip()

    input = f"""
        <image>
        你将得到当前观察的rgb图像。
        请判断当前观察的rgb图像中，能否看到{input2}。如果可以，则回复true，否则为false。
        除此之外什么都不要给我。
        最终按照json格式{{\"vis_first\": true/false}}输出。
        """

    # 构造请求
    response = client.chat.completions.create(
        model=model_name,
        messages=[
            {
                'role': 'user',
                'content': [{
                    'type': 'text',
                    'text':f"""
                        <image>
                        <image>
                        你将得到当前观察的rgb图像。
                        请判断当前观察的rgb图像中，能否看到红色衣柜。如果可以，则回复true，否则为false。
                        除此之外什么都不要给我。
                        最终按照json格式{{\"vis_first\": true/false}}输出。
                        """
                }],
                'role': 'assistant',
                'content': [{
                    'type': 'text',
                    'text': '{\"vis_first\": true}'
                }],
                'role': 'user',
                'content': [{
                    'type': 'text',
                    'text':f'{input}'
                }, {
                    'type': 'image_url',
                    'image_url': {
                        'url': data_url,
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
    data = json.loads(content)
    vis_first = data.get("vis_first", False)

    return vis_first

def open_serve_nav(img_first, img_cur, input1):
    """
    传入图像对象，返回模型识别出的红杯子像素坐标(JSON格式)。
    
    参数:
        img: OpenCV 读取的 BGR 图像 (numpy.ndarray)
    
    返回:
        模型返回的 JSON 字符串或解析后的 Python 对象
    """
    finish_mission = False
    # input = "door"
    # 将图像编码为base64
    _, buffer_first = cv2.imencode('.jpg', img_first)
    img_first_base64 = base64.b64encode(buffer_first).decode('utf-8')
    data_url_first = f'data:image/jpeg;base64,{img_first_base64}'

    _, buffer_cur = cv2.imencode('.jpg', img_cur)
    img_cur_base64 = base64.b64encode(buffer_cur).decode('utf-8')
    data_url_cur = f'data:image/jpeg;base64,{img_cur_base64}'

    # 获取模型名称
    model_name = client.models.list().data[0].id

    input2 = input1.replace("前往", "").strip()

    input = f"""
        <image>
        <image>
        你将得到首次观察rgb图像、当前观察的rgb图像。
        请判断当前观察的rgb图像中，能否看到首次观察rgb图像中的{input2}。如果可以，则回复vis_cur为true，否则为false。
        给出看到的{input2}的bounding box，否则出none，除上述什么都不要给我。
        最终按照json格式{{\"bbox_2d\": [x1, y1, x2, y2]或none, \"vis_cur\": true/false}}输出。
        """
        # 最终按照json格式{{\"bbox_2d\": [x1, y1, x2, y2]或none, \"mission_finish\": true/false}}输出。
        # 如果在当前观察的rgb图像中没有识别到{input2}，请返回none。
        # 另外根据首次观察的rgb图像，判断当前观察的rgb图像，判断任务是否完成。
        # 输出mission finish的状态。

    # 构造请求
    response = client.chat.completions.create(
        model=model_name,
        messages=[
            {
                'role': 'user',
                'content': [{
                    'type': 'text',
                    'text':f"""
                        <image>
                        <image>
                        你将得到首次观察rgb图像、当前观察的rgb图像。
                        请判断当前观察的rgb图像中，能否看到首次观察rgb图像中的红色柜子。如果可以，则回复vis_cur为true，否则为false。
                        如果可以看到给出红色柜子的bounding box，否则出none，除上述什么都不要给我。
                        最终按照json格式{{\"bbox_2d\": [x1, y1, x2, y2]或none, \"vis_cur\": true/false}}输出。
                        """
                }],
                'role': 'assistant',
                'content': [{
                    'type': 'text',
                    'text': '{\"bbox_2d\": [162, 200, 324, 378], \"vis_cur\": true}'
                }],
                'role': 'user',
                'content': [{
                    'type': 'text',
                    'text':f"""
                        <image>
                        <image>
                        你将得到首次观察rgb图像、当前观察的rgb图像。
                        请判断当前观察的rgb图像中，能否看到首次观察rgb图像中的黄色箱子。如果可以，则回复vis_cur为true，否则为false。
                        如果可以看到给出黄色箱子的bounding box，否则出none，除上述什么都不要给我。
                        最终按照json格式{{\"bbox_2d\": [x1, y1, x2, y2]或none, \"vis_cur\": true/false}}输出。
                        """
                }],
                'role': 'assistant',
                'content': [{
                    'type': 'text',
                    'text': '{\"bbox_2d\": none, \"vis_cur\": false}'
                }],
                'role': 'user',
                'content': [{
                    'type': 'text',
                    'text':f'{input}'
                }, {
                    'type': 'image_url',
                    'image_url': {
                        'url': data_url_first,
                    },
                }, {
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

    data = json.loads(content)
    # result = {
    #     "pos": data.get("pos", [-1, -1]),
    #     "yaw": data.get("yaw", 0.0),
    # }
    # finish_mission = data.get("mission_finish", False)
    finish_mission = True
    vis_cur = data.get("vis_cur", False)

    # json_match = re.search(r'\{.*\}', content)
    # if json_match:
    #     bbox_data = json.loads(json_match.group())
    #     bbox = bbox_data['bbox_2d']
    # else:
    #     # 如果直接是json字符串
    #     bbox_data = json.loads(content)
    #     bbox = bbox_data['bbox_2d']

    json_match = re.search(r'\{[^{}]*\}', content)
    if json_match:
        try:
            bbox_data = json.loads(json_match.group())
            bbox = bbox_data['bbox_2d']
        except (json.JSONDecodeError, KeyError):
            bbox_data = json.loads(content)
            bbox = bbox_data['bbox_2d']
    else:
        bbox_data = json.loads(content)
        bbox = bbox_data['bbox_2d']

    bbox = None if bbox == 'none' else bbox
    pt1, pt2 = bbox[0:2], bbox[2:4]
    scale_x = 640 / 1000
    scale_y = 480 / 1000
    pt1 = (int(pt1[0] * scale_x), int(pt1[1] * scale_y))
    pt2 = (int(pt2[0] * scale_x), int(pt2[1] * scale_y))
    bbox = [pt1[0], pt1[1], pt2[0], pt2[1]]
    if bbox is not None:
        x1, y1, x2, y2 = bbox
        # x1, y1, x2, y2 = pt1[0], pt1[1], pt2[0], pt2[1]
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)
    else:
        center_x, center_y = -1, -1

    result = {
    "pos": [center_x, center_y],
    "bbox": bbox,
    "vis_cur": vis_cur
    }

    # 转成 Python 对象
    try:
        # return json.loads(json_str)
        return bbox, result, finish_mission
    except json.JSONDecodeError:
        raise ValueError(f"模型返回的JSON解析失败: {content}")

# 使用示例
if __name__ == "__main__":
    img_path = '/home/diff/workspace/VLA_Diff/camera0_00005.jpg'
    cmd = "给我前方白色柱子的像素坐标"
    img = cv2.imread(img_path)
    result = open_serve(img,cmd)
    print(result)

# vllm serve /home/zhywwyzh/workspace/LLaMA-Factory/output/3dgs/7B/full_sft_all_unfreeze_gptq --dtype auto --port 9000 --max-model-len 4096 --gpu-memory-utilization 0.8
# CUDA_VISIBLE_DEVICES=6 vllm serve /vla/LLaMA-Factory/vla-sft/output/full/sft_rounded --dtype auto --port 8000 --max-model-len 4096 --gpu-memory-utilization 0.8

# CUDA_VISIBLE_DEVICES=6 vllm serve /home/zhywwyzh/workspace/LLaMA-Factory/output/sft_rounded_awq --dtype auto --port 8000 --max-model-len 4096 --gpu-memory-utilization 0.8
