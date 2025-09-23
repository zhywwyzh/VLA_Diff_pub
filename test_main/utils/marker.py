#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

class OdomRepublisher:
    def __init__(self):
        # 订阅原始 odom
        self.sub = rospy.Subscriber(
            "/drone_0_visual_slam/odom",
            Odometry,
            self.odom_callback,
            queue_size=10
        )

        # 发布新的 odom
        self.odom_pub = rospy.Publisher(
            "/drone_0_visual_slam/odom_fixed",
            Odometry,
            queue_size=10
        )

        # 发布 marker
        self.marker_pub = rospy.Publisher(
            "/drone_0_visual_slam/marker",
            Marker,
            queue_size=10
        )

    def odom_callback(self, msg: Odometry):
        # 修改 frame_id 和 child_frame_id
        msg.header.frame_id = "world"   # 改掉 "/world"
        msg.child_frame_id = ""         # 删除 "/quadrotor"

        # 发布新的 odom
        self.odom_pub.publish(msg)

        # 发布 marker（位置跟随 odom 的位置）
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "odom_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = msg.pose.pose
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node("odom_republisher")
    repub = OdomRepublisher()
    rospy.spin()
