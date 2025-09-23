#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32, String
import ast

class CommandPublisher:
    def __init__(self):
        rospy.init_node("command_publisher", anonymous=True)

        self.type_pub = rospy.Publisher("command/type", Int32, queue_size=10)
        self.content_pub = rospy.Publisher("command/content", String, queue_size=10)

        self.command_content = [
            "请结合传入的图1初始观测结果和图2当前观测结果，前往左侧通道入口，给我它在图2中的二维坐标，只给坐标，其他的什么都不要输出。此外，请根据图1的初始观察结果，告诉我当我位于图2的观测位置时，是否到达了图1的预期目标空间位置。只返回True or False,其他什么都不要返回",
            "请结合传入的图1初始观测结果和图2当前观测结果，找到右侧第一个门，给我它在图2中的二维坐标，只给坐标，其他什么都不要输出。此外，请根据图1的初始观察结果，告诉我当我位于图2的观测位置时，是否到达了图1的预期目标空间位置。只返回True or False,其他什么都不要返回",
            "请结合传入的图1初始观测结果和图2当前观测结果，前往右侧门的右后方,给我它在图2中的二维坐标，只给坐标，其他什么都不要输出。此外，请根据图1的初始观察结果，告诉我当我位于图2的观测位置时，是否到达了图1的预期目标空间位置。只返回True or False,其他什么都不要返回",
            "请结合传入的图1初始观测结果和图2当前观测结果，找到树的位置，给我它在图2中的二维坐标，只给坐标，其他什么都不要输出。此外，请根据图1的初始观察结果，告诉我当我位于图2的观测位置时，是否到达了图1的预期目标空间位置。只返回True or False,其他什么都不要返回",
            "前进"
        ]

    def parse_content(self, raw):
        """解析输入: 返回合法字符串 或 list[6个float]，否则返回 None"""
        raw = raw.strip()

        # 尝试解析 Python list
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

        # 尝试逗号分割
        try:
            parts = [float(x) for x in raw.split(',')]
            if len(parts) == 6:
                return str(parts)
        except Exception:
            pass

        # 直接作为字符串
        if raw:
            return raw

        return None

    def run(self):
        rate = rospy.Rate(10)  # 10Hz 循环
        while not rospy.is_shutdown():
            # ===== 输入 command/type =====
            cmd_type = None
            while cmd_type not in [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]:
                try:
                    cmd_type = int(input("请输入 command/type (0:wait, 1:stop, 2:go, 3:next, 4:go_origin, 5:again, 6:emergency_stop, 7:restart, 8:get_pre, 9:replan): "))
                    # cmd_type = int(input("请输入 command/type (4:go_origin, 5:again, 6:emergency_stop, 7:restart, 8:get_pre, 9:restart): "))
                except ValueError:
                    print("❌ 输入必须是整数 (0,1,2,3,4,5,6,7,8,9)")
                    continue
                if cmd_type not in [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]:
                    print("❌ 输入错误，请输入 0, 1, 2, 3, 4, 5, 6, 7, 8 或 9")

            if cmd_type == 1:
                print("🛑 收到 type=1，程序结束")
                break

            elif cmd_type == 2:
                raw = input("please press enter to continue: ")
                content = self.parse_content(raw)
                print("执行下一步任务")

                if content is None:
                    print("❌ content 输入非法，返回到 type 阶段")
                    continue

                content_msg = String()
                content_msg.data = content
                self.content_pub.publish(content_msg)
                rospy.loginfo("✅ Published command/content: %s", content)

            if cmd_type == 0:
                # 直接下一轮
                continue

            # 发布 type
            type_msg = Int32()
            type_msg.data = cmd_type
            self.type_pub.publish(type_msg)
            rospy.loginfo("✅ Published command/type: %d", cmd_type)

            rate.sleep()

def main():
    node = CommandPublisher()
    try:
        node.run()
    except rospy.ROSInterruptException:
        print("\n🛑 Ctrl+C 退出程序")

if __name__ == '__main__':
    main()
