SYSTEM_PROMPT = """You are a target detection robot. During the task execution, I will give you a task requirement and a set of pictures.

You will analyze the image according to the task requirements and solve the bbox coordinates of the task target.

When executing tasks, you need to pay attention to the following points:

(1) Note that the image size is 640*480. When performing object detection, be careful not to obtain results that are beyond the image size range.

(2) Your output format should be a bbox output in json format"""

# USER1 = """refrigerator"""

# ASSISTANT1 = """json\n[\n\t{"bbox_2d": [379, 252, 542, 410], "label": "refrigerator"}\n]\n"""

# USER2 = """shelf"""

# ASSISTANT2 = """json\n[\n\t{"bbox_2d": [347, 145, 547, 168], "label": "shelf"}\n]\n"""




USER1 = """请结合传入的图1初始观测结果和图2当前观测结果，找到第一个转角的位置，给我它在图2中的二维坐标，只给坐标，其他的什么都不要输出。类似(x,y)。

此外，请根据图1的初始观察结果，告诉我当我位于图2的观测位置时，是否到达了图1的预期目标空间位置。

只返回True or False,其他什么都不要返回"""


ASSISTANT1 = """\"(320, 240)\", True"""


USER2 = """请结合传入的图1初始观测结果和图2当前观测结果，找到右侧第一个门的位置，给我它在图2中的二维坐标，只给坐标，其他的什么都不要输出。类似(x,y)。

此外，请根据图1的初始观察结果，告诉我当我位于图2的观测位置时，是否到达了图1的预期目标空间位置。

只返回True or False,其他什么都不要返回"""


ASSISTANT2 = """\"(400, 240)\", False"""




# USER1 = """请结合传入的图1初始观测结果和图2当前观测结果，请分析两张图片中各自有什么内容"""


# ASSISTANT1 = """图1中有一个冰箱和一个架子，冰箱位于图像的左下角，架子位于右侧。图2中有一个架子，位于图像的左上角。"""


# USER2 = """请结合传入的图1初始观测结果和图2当前观测结果，请分析两张图片中各自有什么内容"""


# ASSISTANT2 = """图1有两个饮水机，第一个饮水机在图像的左下角，第二个饮水机位于图像的中央。图2中有一个饮水机，远处有一个办公桌和一盆绿色植物。"""