import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import ast

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.type_pub = self.create_publisher(Int32, 'command/type', 10)
        self.content_pub = self.create_publisher(String, 'command/content', 10)

    def parse_content(self, raw: str):
        """解析输入: 返回合法字符串 或 list[6个float]，否则返回 None"""
        raw = raw.strip()

        # 尝试解析成 Python 对象（比如 [1,2,3,4,5,6]）
        try:
            parsed = ast.literal_eval(raw)
            if isinstance(parsed, list):
                if len(parsed) == 6 and all(isinstance(x, (int, float)) for x in parsed):
                    return str(parsed)
                else:
                    print("❌ 列表输入不对，必须是 6 个 float")
                    return None
        except Exception:
            pass

        # 尝试逗号分割 "0,0,0,0,0,0"
        try:
            parts = [float(x) for x in raw.split(',')]
            if len(parts) == 6:
                return str(parts)
        except Exception:
            pass

        # 如果不是 list，就当成字符串（非空才接受）
        if raw:
            return raw

        return None

    def run(self):
        while rclpy.ok():
            # ===== 输入 command/type =====
            cmd_type = None
            while cmd_type not in [0, 1, 2]:
                try:
                    cmd_type = int(input("请输入 command/type (0,1,2): "))
                except ValueError:
                    print("❌ 输入必须是整数 (0,1,2)")
                    continue
                if cmd_type not in [0, 1, 2]:
                    print("❌ 输入错误，请输入 0, 1 或 2")

            # 发布 type
            type_msg = Int32()
            type_msg.data = cmd_type
            self.type_pub.publish(type_msg)
            self.get_logger().info(f"✅ Published command/type: {cmd_type}")

            if cmd_type == 1:
                print("🛑 收到 type=1，程序结束")
                break

            elif cmd_type == 2:
                # raw = input("请输入 command/content (字符串 或 list[6个float]): ")
                # content = self.parse_content(raw)
                print("执行下一步任务")

                # if content is None:
                #     print("❌ content 输入非法，返回到 type 阶段")
                #     continue  # 直接回到 type 输入

                # content_msg = String()
                # content_msg.data = content
                # self.content_pub.publish(content_msg)
                # self.get_logger().info(f"✅ Published command/content: {content}")

            # type==0 直接下一轮

def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        print("\n🛑 Ctrl+C 退出程序")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
