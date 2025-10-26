import os
from openai import OpenAI
ARK_API_KEY="6240fe7c-977e-4d88-ba53-a3e63f3767cc"
# 请确保您已将 API Key 存储在环境变量 ARK_API_KEY 中
# 初始化Ark客户端，从环境变量中读取您的API Key
client = OpenAI(
    # 此为默认路径，您可根据业务所在地域进行配置
    base_url="https://ark.cn-beijing.volces.com/api/v3",
    # 从环境变量中获取您的 API Key。此为默认方式，您可根据需要进行修改
    api_key=ARK_API_KEY,
)

response = client.chat.completions.create(
    # 指定您创建的方舟推理接入点 ID，此处已帮您修改为您的推理接入点 ID
    model="doubao-seed-1-6-vision-250815",
    messages=[
        {
            "role": "user",
            "content": [
                {
                    "type": "image_url",
                    "image_url": {
                        "url": "https://ark-project.tos-cn-beijing.ivolces.com/images/view.jpeg"
                    },
                },
                {"type": "text", "text": "这是哪里？"},
            ],
        }
    ],
    extra_body={
        "thinking": {
            # "type": "disabled",  # 不使用深度思考能力
            "type": "enabled", # 使用深度思考能力
            # "type": "auto", # 模型自行判断是否使用深度思考能力
        }
    },
)

print(response.choices[0])