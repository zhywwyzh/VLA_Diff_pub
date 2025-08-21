#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_msg)
{
    // 创建一个消息的副本
    sensor_msgs::PointCloud2 output_msg = *input_msg;

    // 将消息头的时间戳替换为当前的ROS时间
    output_msg.header.stamp = ros::Time::now();

    // 发布带有新时间戳的消息
    pub.publish(output_msg);
}

int main(int argc, char** argv)
{
    // 初始化ROS
    ros::init(argc, argv, "timestamp_converter_node");
    ros::NodeHandle nh;

    // 创建一个发布器，发布到 /livox/lidar_ros 话题
    pub = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_ros", 10);

    // 创建一个订阅器，订阅原始的 /livox/lidar 话题
    ros::Subscriber sub = nh.subscribe("/livox/lidar", 10, cloudCallback);

    // 循环等待回调
    ros::spin();

    return 0;
}