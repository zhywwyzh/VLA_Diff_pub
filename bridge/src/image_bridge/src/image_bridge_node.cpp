#include "image_bridge/ImageBridge.hpp"
#include <ros/ros.h>

ImageBridge::ImageBridge(ros::NodeHandle &nh) {
    // 获取参数
    nh.param<std::string>("target_host", target_host_, "192.168.1.51");
    nh.param("target_port", target_port_, 9999);

    // 创建socket连接
    if(connectToTarget()){
        ROS_INFO("Connection established successfully");
    }else{
        ROS_ERROR("Initial connection failed!"); 
    }

    // 订阅ROS topic
    sub_ = nh.subscribe("/camera/color/image/compressed", 10, &ImageBridge::imageCallback, this);
    odom_sub_ = nh.subscribe("/drone_0_visual_slam/odom", 10, &ImageBridge::odomCallback, this); 
    
    ROS_INFO("Image bridge started, connecting to %s:%d", 
            target_host_.c_str(), target_port_);
}

bool ImageBridge::connectToTarget() {
    sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd_ < 0) {
        ROS_ERROR("Error opening socket");
        return false;
    }

    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(target_port_);
    
    if(inet_pton(AF_INET, target_host_.c_str(), &serv_addr.sin_addr)<=0) {
        ROS_ERROR("Invalid address/ Address not supported");
        return false;
    }

    if (connect(sockfd_, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        ROS_ERROR("Connection Failed");
        close(sockfd_);
        return false;
    }
    
    return true;
}

void ImageBridge::imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
    ROS_INFO("Image Received");
    try {
        std::lock_guard<std::mutex> lock(mutex_);

        // 发送消息类型标识 (1表示图像)
        uint8_t msg_type = 1;
        send(sockfd_, &msg_type, sizeof(msg_type), 0);

        uint32_t fmt_len = msg->format.size();
        uint32_t fmt_len_net = htonl(fmt_len); // 转为网络字节序
        send(sockfd_, &fmt_len_net, sizeof(fmt_len_net), 0);
        send(sockfd_, msg->format.c_str(), fmt_len, 0);
        
        uint32_t data_len = msg->data.size();
        uint32_t data_len_net = htonl(data_len); // 转为网络字节序
        send(sockfd_, &data_len_net, sizeof(data_len_net), 0);
        send(sockfd_, &msg->data[0], data_len, 0);

        // ROS_INFO("Sent image: format=%s, size=%lu bytes", msg->format.c_str(), data_len);
    } catch (...) {
        ROS_ERROR("Error sending image");
        reconnect();
    }
}

void ImageBridge::odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    ROS_INFO("Odometry Received");
    try {
        std::lock_guard<std::mutex> lock(mutex_);

        // 发送消息类型标识 (2表示里程计)
        uint8_t msg_type = 2;
        send(sockfd_, &msg_type, sizeof(msg_type), 0);

        // 准备要发送的数组数据
        double odom_data[7] = {
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        };

        // 打印要发送的数据（调试用）
        ROS_INFO("Sending odometry data: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                odom_data[0], odom_data[1], odom_data[2],
                odom_data[3], odom_data[4], odom_data[5], odom_data[6]);

        // 发送数据（不需要发送长度，因为接收端知道固定是7个double）
        send(sockfd_, odom_data, sizeof(odom_data), 0);

        ROS_INFO("Sent odometry: size=%lu bytes", sizeof(odom_data));
    } catch (...) {
        ROS_ERROR("Error sending odometry");
        reconnect();
    }
}

void ImageBridge::reconnect() {
    std::lock_guard<std::mutex> lock(mutex_);
    close(sockfd_);
    connectToTarget();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_bridge"); 
    ros::NodeHandle nh("~");
    
    ImageBridge bridge(nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}
