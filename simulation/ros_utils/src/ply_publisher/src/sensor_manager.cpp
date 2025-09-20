#include "ply_publisher/sensor_manager.h"
#include <ros/ros.h>

void SensorManager::initParams() {
    ros::NodeHandle nh("~");
    nh.param<std::string>("odom_topic", odom_topic_, "/drone_0_visual_slam/odom");

    nh.param<int>("image_width", image_width_, 640);
    nh.param<int>("image_height", image_height_, 480);
    nh.param<double>("fx", fx_, 442.0250167097101);
    nh.param<double>("fy", fy_, 442.0250167097101);
    nh.param<double>("cx", cx_, 320.0);
    nh.param<double>("cy", cy_, 240.0);
    nh.param<double>("depth_max", depth_max_, 5.0);

    frame_id_ = "world";
}

void SensorManager::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_ = *msg;
    got_odom_ = true;
}

bool SensorManager::hasOdom() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return got_odom_;
}

nav_msgs::Odometry SensorManager::getOdomCopy() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_odom_;
}

void SensorManager::setCameraParams(int width, int height, double fx, double fy, double cx, double cy, double depth_max) {
    image_width_ = width;
    image_height_ = height;
    fx_ = fx; fy_ = fy; cx_ = cx; cy_ = cy;
    depth_max_ = depth_max;
}

sensor_msgs::CameraInfo SensorManager::buildCameraInfo() const {
    sensor_msgs::CameraInfo ci;
    ci.header.frame_id = frame_id_;
    ci.height = image_height_;
    ci.width  = image_width_;
    ci.distortion_model = "plumb_bob";

    // D (rad-tan) 5 params, all zero
    ci.D.resize(5, 0.0);

    // K (3x3)
    ci.K[0] = fx_;  ci.K[1] = 0.0;  ci.K[2] = cx_;
    ci.K[3] = 0.0;  ci.K[4] = fy_;  ci.K[5] = cy_;
    ci.K[6] = 0.0;  ci.K[7] = 0.0;  ci.K[8] = 1.0;

    // R (3x3) = Identity
    ci.R[0] = 1.0;  ci.R[1] = 0.0;  ci.R[2] = 0.0;
    ci.R[3] = 0.0;  ci.R[4] = 1.0;  ci.R[5] = 0.0;
    ci.R[6] = 0.0;  ci.R[7] = 0.0;  ci.R[8] = 1.0;

    // P (3x4)
    ci.P[0] = fx_;  ci.P[1] = 0.0;  ci.P[2] = cx_;  ci.P[3] = 0.0;
    ci.P[4] = 0.0;  ci.P[5] = fy_;  ci.P[6] = cy_;  ci.P[7] = 0.0;
    ci.P[8] = 0.0;  ci.P[9] = 0.0;  ci.P[10]= 1.0;  ci.P[11]= 0.0;

    return ci;
}
