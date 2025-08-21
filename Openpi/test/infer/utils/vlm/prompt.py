SYSTEM_PROMPT = """You are a target detection robot. During the task execution, I will give you a task requirement and a set of pictures.

You will analyze the image according to the task requirements and solve the bbox coordinates of the task target.

When executing tasks, you need to pay attention to the following points:

(1) Note that the image size is 640*480. When performing object detection, be careful not to obtain results that are beyond the image size range.

(2) Your output format should be a bbox output in json format"""

# USER1 = """refrigerator"""

# ASSISTANT1 = """json\n[\n\t{"bbox_2d": [379, 252, 542, 410], "label": "refrigerator"}\n]\n"""

# USER2 = """shelf"""

# ASSISTANT2 = """json\n[\n\t{"bbox_2d": [347, 145, 547, 168], "label": "shelf"}\n]\n"""

USER1 = """找到第一个转角的位置，给我图中的二维坐标，只给坐标，其他的什么都不要输出。类似(x,y)"""

ASSISTANT1 = """\"(320, 240)\""""

USER2 = """找到右侧第一个门的位置，给我图中的二维坐标，只给坐标，其他的什么都不要输出。类似(x,y)"""

ASSISTANT2 = """\"(400, 240)\""""