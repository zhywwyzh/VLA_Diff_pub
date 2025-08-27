import requests
import cv2
import base64
import numpy as np
import time
from datetime import datetime

class MessageClient:
    def __init__(self, base_url="3001"):
        self.base_url = f"http://localhost:{base_url}/api/send"

    def send_message(self, img: np.ndarray, sender: str,
                     text: str = "好的", msg_type: str = "image",
                     priority: str = "normal"):
        # 准备 image data-url
        images_info = []
        if isinstance(img, np.ndarray) and img.size > 0:
            ok, buf = cv2.imencode('.jpg', img)
            if not ok:
                raise ValueError("cv2.imencode 失败")
            data_url = "data:image/jpeg;base64," + base64.b64encode(buf).decode("utf-8")
            h, w = img.shape[:2]
            images_info.append({
                "index": 1,
                "filename": f"image_{int(time.time())}.jpg",
                "format": "jpeg",
                "size": f"{w}x{h}",
                "data": data_url
            })

        payload = {
            "text": text,                   # 不能是空
            "sender": sender,
            "type": msg_type,
            "priority": priority or "normal",
            "images_info": images_info,
            "image_count": len(images_info),
        }

        try:
            resp = requests.post(self.base_url, json=payload)
            resp.raise_for_status()
            return resp.json()
        except requests.RequestException as e:
            print(f"发送消息失败: {e}")
            return None

    def send_image(self, img, sender: str = "from fsm",
                   text: str = "[image]", priority: str = "normal"):
        return self.send_message(img, sender, text=text,
                                 msg_type="image", priority=priority)

if __name__ == "__main__":
    client = MessageClient()
    # 示例：client.send_image(img, sender="fsm")
