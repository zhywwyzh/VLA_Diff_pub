#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import GoalSet  # 自定义消息

class GoalConverter:
    def __init__(self):
        # 发布者
        self.pub = rospy.Publisher('/goal_with_id', GoalSet, queue_size=10)
        # 订阅者
        rospy.Subscriber('/goal', PoseStamped, self.goal_callback)

    def goal_callback(self, msg):
        out_msg = GoalSet()
        out_msg.drone_id = 0
        out_msg.goal[0] = msg.pose.position.x
        out_msg.goal[1] = msg.pose.position.y
        out_msg.goal[2] = msg.pose.position.z
        self.pub.publish(out_msg)
        rospy.loginfo("Published goal_with_id: id=%d, goal=%s" %
                      (out_msg.drone_id, list(out_msg.goal)))

if __name__ == '__main__':
    rospy.init_node('goal_converter', anonymous=True)
    node = GoalConverter()
    rospy.spin()
