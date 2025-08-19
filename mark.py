#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(Marker, '/visualization_marker', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)  # 每 0.5 秒发一次

    def timer_callback(self):
        marker = Marker()
        marker.header.frame_id = "map"  # 参考坐标系
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "basic_shapes"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # 坐标
        marker.pose.position.x = 2.6078434
        marker.pose.position.y = 0.4830592
        marker.pose.position.z = -0.44855497

        # 姿态（不旋转）
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # 尺寸（单位 m）
        marker.scale.x = 2.0
        marker.scale.y = 2.0
        marker.scale.z = 2.0

        # 颜色（绿色，不透明）
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.publisher_.publish(marker)
        self.get_logger().info("Marker published.")

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
