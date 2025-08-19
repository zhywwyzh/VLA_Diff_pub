#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/GoalSet.h>

class GoalConverter
{
public:
    GoalConverter()
    {
        pub_ = nh_.advertise<quadrotor_msgs::GoalSet>("/goal_with_id", 10);
        sub_ = nh_.subscribe("/goal", 10, &GoalConverter::goalCallback, this);
    }

private:
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        quadrotor_msgs::GoalSet out_msg;
        out_msg.drone_id = 0;
        out_msg.goal[0] = msg->pose.position.x;
        out_msg.goal[1] = msg->pose.position.y;
        out_msg.goal[2] = msg->pose.position.z;

        pub_.publish(out_msg);

        ROS_INFO("Published /goal_with_id: id=%d, goal=[%.3f, %.3f, %.3f]",
                 out_msg.drone_id, out_msg.goal[0], out_msg.goal[1], out_msg.goal[2]);
    }

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "goal_converter");
    GoalConverter converter;
    ros::spin();
    return 0;
}
