#ifndef IMAGE_BRIDGE_HPP
#define IMAGE_BRIDGE_HPP

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <nav_msgs/Odometry.h> 
#include <sensor_msgs/Image.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <mutex>
#include <string>

class ImageBridge {
public:
    ImageBridge(ros::NodeHandle &nh);
    void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg);
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    
private:
    bool connectToTarget();
    void reconnect();
    
    ros::Subscriber sub_;
    ros::Subscriber odom_sub_;
    int sockfd_;
    std::string target_host_;
    int target_port_;
    std::mutex mutex_;
};

#endif // IMAGE_BRIDGE_HPP
