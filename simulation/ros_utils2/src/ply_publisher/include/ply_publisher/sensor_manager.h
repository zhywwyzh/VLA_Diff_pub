#ifndef _SENSOR_MANAGER_H
#define _SENSOR_MANAGER_H

#include <string>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>

class SensorManager {
public:
    SensorManager() = default;
    void initParams();

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    bool hasOdom() const;
    nav_msgs::Odometry getOdomCopy() const;

    sensor_msgs::CameraInfo buildCameraInfo() const;
    void setCameraParams(int width, int height, double fx, double fy, double cx, double cy, double depth_max);

private:
    mutable std::mutex mutex_;
    nav_msgs::Odometry latest_odom_;
    bool got_odom_ = false;

    // 相机内参
    int image_width_ = 640;
    int image_height_ = 480;
    double fx_ = 442.0250167097101;
    double fy_ = 442.0250167097101;
    double cx_ = 320.0;
    double cy_ = 240.0;
    double depth_max_ = 5.0;

    // 里程计话题
    std::string odom_topic_ = "/drone_0_visual_slam/odom";

    // 统一使用 world
    std::string frame_id_ = "world";

public:
    std::string getOdomTopic() const { return odom_topic_; }
    std::string getFrameId()   const { return frame_id_; }
    int    getImageWidth()     const { return image_width_; }
    int    getImageHeight()    const { return image_height_; }
    double getFx()             const { return fx_; }
    double getFy()             const { return fy_; }
    double getCx()             const { return cx_; }
    double getCy()             const { return cy_; }
    double getDepthMax()       const { return depth_max_; }
};

#endif // _SENSOR_MANAGER_H
