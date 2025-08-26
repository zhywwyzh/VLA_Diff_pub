import requests

class MessageClient:
    def __init__(self, base_url="3001"):
        self.base_url = f"http://localhost:{base_url}/api/send"

    def send_message(self, text: str, sender: str, type: str = "text", priority: str = None):
        """
        发送消息
        :param text: 消息内容
        :param sender: 发送者
        :param type: 消息类型
        :param priority: 消息优先级，可选，示例 "urgent"
        :return: 响应对象
        """
        payload = {
            "text": text,
            "sender": sender
        }
        if priority:
            payload["priority"] = priority

        try:
            response = requests.post(self.base_url, json=payload)
            response.raise_for_status()  # 出错抛异常
            return response.json()
        except requests.RequestException as e:
            print(f"发送消息失败: {e}")
            return None

    def send_normal(self, text: str, sender: str, type: str = "text"):
        """发送普通消息"""
        return self.send_message(text, sender, type)

    def send_urgent(self, text: str, sender: str, type: str = "text"):
        """发送高优先级消息"""
        return self.send_message(text, sender, type, priority="urgent")


if __name__ == "__main__":
    client = MessageClient()

    # 普通消息
    resp1 = client.send_normal("你好，这是测试消息", "测试程序")
    print("普通消息响应:", resp1)

    # 高优先级消息
    resp2 = client.send_urgent("紧急任务", "监控系统")
    print("高优先级消息响应:", resp2)
