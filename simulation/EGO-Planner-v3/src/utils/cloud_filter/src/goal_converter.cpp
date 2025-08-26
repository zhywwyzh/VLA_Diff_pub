# include <ros/ros.h>
# include <geometry_msgs/PoseStamped.h>
# include <quadrotor_msgs/GoalSet.h>
# include <tf/tf.h>

class GoalConverter
{
public:
    GoalConverter()
    {
        pub_ = nh_.advertise<quadrotor_msgs::GoalSet>("/goal_with_id_from_station", 10);
        sub_ = nh_.subscribe("/goal", 10, &GoalConverter::goalCallback, this);
    }

private:
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        quadrotor_msgs::GoalSet out_msg;
        out_msg.to_drone_ids.clear();
        out_msg.to_drone_ids.push_back(0);

        geometry_msgs::Point pt;
        pt.x = msg->pose.position.x;
        pt.y = msg->pose.position.y;
        pt.z = msg->pose.position.z;
        out_msg.goal.clear();
        out_msg.goal.push_back(pt);

        double yaw = tf::getYaw(msg->pose.orientation);
        out_msg.yaw.clear();
        out_msg.yaw.push_back(yaw);

        out_msg.look_forward = true;
        out_msg.goal_to_follower = false;

        pub_.publish(out_msg);

        // 正确打印
        ROS_INFO("Published /goal_with_id_from_station: "
                 "drone_id=%u, goal=(%.3f, %.3f, %.3f), yaw=%.3f, look_forward=%s, goal_to_follower=%s",
                 out_msg.to_drone_ids[0],
                 pt.x, pt.y, pt.z, yaw,
                 out_msg.look_forward ? "true" : "false",
                 out_msg.goal_to_follower ? "true" : "false");
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
