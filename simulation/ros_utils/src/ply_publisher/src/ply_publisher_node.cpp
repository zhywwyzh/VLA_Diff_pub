#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

#include <Eigen/Geometry>
#include <limits>
#include <array>
#include <vector>
#include <cmath>

#include "ply_publisher/pc_processor.h"
#include "ply_publisher/sensor_manager.h"
#include "ply_publisher/ground_align.h"

// 32FC1(m) -> 8UC1(0~255)
static inline uint8_t meterToByte(float z_m, float depth_max) {
    if (!(z_m > 0.0f)) return 0u;
    float x = z_m / depth_max;
    if (x < 0.f) x = 0.f;
    if (x > 1.f) x = 1.f;
    return static_cast<uint8_t>(std::round(x * 255.f));
}

static inline Eigen::Matrix3f Rz(float yaw_rad) {
    const float c = std::cos(yaw_rad), s = std::sin(yaw_rad);
    Eigen::Matrix3f R;
    R << c,-s, 0,
         s, c, 0,
         0, 0, 1;
    return R;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ply_publisher_node");
    ros::NodeHandle nh("~");

    // I/O topics (点云、相机信息依旧；深度仅压缩版按 /camera{1..4} 固定发布)
    std::string ply_file;            nh.param<std::string>("ply_file", ply_file, "test.ply");
    std::string point_topic;         nh.param<std::string>("topic_name", point_topic, "/ply_points");
    std::string caminfo_topic;       nh.param<std::string>("camera_info_topic", caminfo_topic, "/camera1/camera_info");

    // rates
    double depth_rate_hz;            nh.param<double>("depth_rate_hz", depth_rate_hz, 10.0);
    double point_rate_hz;            nh.param<double>("point_rate_hz", point_rate_hz, 1.0);

    // 下采样&渲染
    bool enable_down;                nh.param<bool>("enable_downsample", enable_down, true);
    double leaf_size;                nh.param<double>("leaf_size", leaf_size, 0.01);
    double depth_min;                nh.param<double>("depth_min", depth_min, 0.05);
    bool frustum_culling;            nh.param<bool>("frustum_culling", frustum_culling, true);
    bool use_cropbox;                nh.param<bool>("use_cropbox", use_cropbox, false);
    int render_stride;               nh.param<int>("render_stride", render_stride, 1);
    int png_level;                   nh.param<int>("png_level", png_level, 1);
    png_level = std::min(9, std::max(0, png_level));

    // 新增参数：像素溅射半径和光学系开关
    int splat_radius_px;             nh.param<int>("splat_radius_px", splat_radius_px, 1);   // 0=仅(u,v)，1=3x3，2=5x5...
    bool use_optical_convention;     nh.param<bool>("use_optical_convention", use_optical_convention, true);

    // 相机与里程计
    SensorManager sensor_mgr;
    sensor_mgr.initParams();
    sensor_mgr.setCameraParams(sensor_mgr.getImageWidth(), sensor_mgr.getImageHeight(),
                               sensor_mgr.getFx(), sensor_mgr.getFy(), sensor_mgr.getCx(), sensor_mgr.getCy(),
                               sensor_mgr.getDepthMax());
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(sensor_mgr.getOdomTopic(), 10,
                                                                &SensorManager::odomCallback, &sensor_mgr);

    // Unity transform
    std::vector<double> unity_pos_v, unity_rot_v, unity_scale_v;
    nh.param<std::vector<double>>("unity_pos",       unity_pos_v,  std::vector<double>{0, 0, 0});
    nh.param<std::vector<double>>("unity_rot_deg",   unity_rot_v,  std::vector<double>{0, 0, 0});
    nh.param<std::vector<double>>("unity_scale",     unity_scale_v,std::vector<double>{1, 1, 1});

    PCProcessor pcproc;
    pcproc.setDownsampleEnabled(enable_down);
    pcproc.setLeafSize((float)leaf_size, (float)leaf_size, (float)leaf_size);
    pcproc.setUnityTransform(Eigen::Vector3f(unity_pos_v[0], unity_pos_v[1], unity_pos_v[2]),
                             Eigen::Vector3f(unity_rot_v[0], unity_rot_v[1], unity_rot_v[2]),
                             Eigen::Vector3f(unity_scale_v[0], unity_scale_v[1], unity_scale_v[2]));
    if (!pcproc.loadPLY(ply_file) || !pcproc.process()) {
        ROS_ERROR("Failed to prepare point cloud.");
        return 1;
    }

    // 地面对齐
    pcl::PointCloud<PointT>::ConstPtr cloud_in = pcproc.getProcessedCloud();
    pcl::PointCloud<PointT>::ConstPtr cloud_aligned_const;
    Eigen::Affine3f T_align = Eigen::Affine3f::Identity();

    ply_publisher::GroundAlign ground_align;
    ground_align.loadParams(nh);
    ground_align.process(cloud_in, cloud_aligned_const, T_align);

    // Publishers
    ros::Publisher pc_pub      = nh.advertise<sensor_msgs::PointCloud2>(point_topic, 1, true);
    ros::Publisher caminfo_pub = nh.advertise<sensor_msgs::CameraInfo>(caminfo_topic, 1, true);

    // 四个方向的压缩深度图发布器（固定话题名）
    std::array<ros::Publisher,4> depthc_pubs = {
        nh.advertise<sensor_msgs::CompressedImage>("/camera1/depth/image/compressed", 1),
        nh.advertise<sensor_msgs::CompressedImage>("/camera2/depth/image/compressed", 1),
        nh.advertise<sensor_msgs::CompressedImage>("/camera3/depth/image/compressed", 1),
        nh.advertise<sensor_msgs::CompressedImage>("/camera4/depth/image/compressed", 1)
    };

    // 点云消息
    sensor_msgs::PointCloud2 pc2_msg;
    pcl::toROSMsg(*cloud_aligned_const, pc2_msg);
    pc2_msg.header.frame_id = "world";

    // 相机信息
    sensor_msgs::CameraInfo cam_info = sensor_mgr.buildCameraInfo();

    // 频率控制
    ros::Time last_point_pub = ros::Time(0);
    ros::Duration point_period(1.0 / point_rate_hz);
    ros::Rate depth_rate(depth_rate_hz);

    // 预分配渲染缓冲
    const int W = sensor_mgr.getImageWidth();
    const int H = sensor_mgr.getImageHeight();
    const double fx = sensor_mgr.getFx(), fy = sensor_mgr.getFy();
    const double cx = sensor_mgr.getCx(), cy = sensor_mgr.getCy();
    const float depth_max = static_cast<float>(sensor_mgr.getDepthMax());
    std::vector<float> depth_buf(W * H);
    cv::Mat depth32(H, W, CV_32FC1);
    cv::Mat depth8c1(H, W, CV_8UC1);
    cv::Mat depth8c3(H, W, CV_8UC3);
    std::vector<uchar> png_buf;
    std::vector<int> png_params = { cv::IMWRITE_PNG_COMPRESSION, png_level };

    // 最大半视角
    const double half_h_left  = std::atan(cx / fx);
    const double half_h_right = std::atan((W - 1 - cx) / fx);
    const double half_h = std::max(half_h_left, half_h_right);
    const double half_v_up    = std::atan(cy / fy);
    const double half_v_down  = std::atan((H - 1 - cy) / fy);
    const double half_v = std::max(half_v_up, half_v_down);

    auto full_cloud = cloud_aligned_const;

    // 机体系(x前,y左,z上) -> 相机光学系(x右,y下,z前)
    // x_opt = -y_body, y_opt = -z_body, z_opt = x_body
    const Eigen::Matrix3f R_bo = (Eigen::Matrix3f() <<
         0.f, -1.f,  0.f,
         0.f,  0.f, -1.f,
         1.f,  0.f,  0.f).finished();

    // 四个方向的附加机体系偏航角：前(1)=0, 后(2)=π, 左(3)=+π/2, 右(4)=−π/2
    const std::array<float,4> yaw_offsets = {0.f, static_cast<float>(M_PI),
                                             static_cast<float>(M_PI_2), static_cast<float>(-M_PI_2)};

    while (ros::ok()) {
        ros::spinOnce();

        if (sensor_mgr.hasOdom()) {
            nav_msgs::Odometry odom = sensor_mgr.getOdomCopy();

            const ros::Time stamp = odom.header.stamp;
            const std::string frame = "world";

            // 点云按频率发布
            if ((ros::Time::now() - last_point_pub) >= point_period) {
                pc2_msg.header.stamp = ros::Time::now();
                pc2_msg.header.frame_id = frame;
                pc_pub.publish(pc2_msg);
                last_point_pub = ros::Time::now();
            }

            // CameraInfo（发一次即可）
            cam_info.header.stamp = stamp;
            cam_info.header.frame_id = frame;
            caminfo_pub.publish(cam_info);

            // 位姿（T^world_body）
            const Eigen::Vector3f t_wb(odom.pose.pose.position.x,
                                       odom.pose.pose.position.y,
                                       odom.pose.pose.position.z);
            const Eigen::Quaternionf q_wb(odom.pose.pose.orientation.w,
                                          odom.pose.pose.orientation.x,
                                          odom.pose.pose.orientation.y,
                                          odom.pose.pose.orientation.z);
            const Eigen::Matrix3f R_wb = q_wb.toRotationMatrix(); // body->world
            const Eigen::Matrix3f R_bw = R_wb.transpose();        // world->body

            // 可选 CropBox（统一用“前向光学系”朝向估算裁剪盒姿态，避免每个方向单独裁剪带来的不一致）
            pcl::PointCloud<PointT>::ConstPtr cloud_to_render = full_cloud;
            pcl::PointCloud<PointT>::Ptr cropped(new pcl::PointCloud<PointT>);
            if (use_cropbox) {
                float x_lim_max = static_cast<float>(std::tan(half_h) * depth_max);
                float y_lim_max = static_cast<float>(std::tan(half_v) * depth_max);

                pcl::CropBox<PointT> crop;
                crop.setMin(Eigen::Vector4f(-x_lim_max, -y_lim_max, static_cast<float>(depth_min), 1.0f));
                crop.setMax(Eigen::Vector4f( x_lim_max,  y_lim_max, depth_max, 1.0f));

                // 用“前向” optical->world 姿态设置裁剪盒方向
                const Eigen::Matrix3f R_ow_front = use_optical_convention ? (R_bo * R_bw) : R_bw;
                const Eigen::Matrix3f R_wo_front = R_ow_front.transpose();
                Eigen::Vector3f rpy_wo = R_wo_front.eulerAngles(0,1,2);
                crop.setRotation(Eigen::Vector3f(rpy_wo[0], rpy_wo[1], rpy_wo[2]));
                crop.setTranslation(t_wb);

                crop.setInputCloud(full_cloud);
                crop.filter(*cropped);
                cloud_to_render = cropped;
            }

            // 渲染四个方向并压缩发布
            const int stride = std::max(1, render_stride);
            const int r = std::max(0, splat_radius_px);

            for (size_t k = 0; k < yaw_offsets.size(); ++k) {
                // world->optical_k = (R_bo * Rz(ψ_k)) * R_bw
                Eigen::Matrix3f R_ow;
                if (use_optical_convention) {
                    R_ow = (R_bo * Rz(yaw_offsets[k]) * R_bw);
                } else {
                    R_ow = (Rz(yaw_offsets[k]) * R_bw);
                }

                // 栅格化渲染
                std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());

                for (size_t i = 0; i < cloud_to_render->points.size(); i += (size_t)stride) {
                    const auto& pt = cloud_to_render->points[i];
                    const Eigen::Vector3f pw(pt.x, pt.y, pt.z);
                    const Eigen::Vector3f p_cam = R_ow * (pw - t_wb); // world->optical_k

                    const float Z = p_cam.z();
                    if (Z <= static_cast<float>(depth_min) || Z > depth_max) continue;

                    if (frustum_culling) {
                        const float X = p_cam.x(), Y = p_cam.y();
                        const float x_lim = static_cast<float>(std::tan(half_h) * Z);
                        if (X < -x_lim || X > x_lim) continue;
                        const float y_lim = static_cast<float>(std::tan(half_v) * Z);
                        if (Y < -y_lim || Y > y_lim) continue;
                    }

                    const float uf = static_cast<float>((fx * p_cam.x() / Z) + cx);
                    const float vf = static_cast<float>((fy * p_cam.y() / Z) + cy);
                    int u0 = static_cast<int>(std::round(uf));
                    int v0 = static_cast<int>(std::round(vf));
                    if ((unsigned)u0 >= (unsigned)W || (unsigned)v0 >= (unsigned)H) continue;

                    // 溅射：更新 (u0,v0) 周围 (2r+1)^2 的最小深度
                    const int umin = std::max(0, u0 - r), umax = std::min(W - 1, u0 + r);
                    const int vmin = std::max(0, v0 - r), vmax = std::min(H - 1, v0 + r);
                    for (int v = vmin; v <= vmax; ++v) {
                        int base = v * W;
                        for (int u = umin; u <= umax; ++u) {
                            int idx = base + u;
                            if (Z < depth_buf[idx]) depth_buf[idx] = Z;
                        }
                    }
                }

                // 填充 32FC1（如需未压缩，可在此发布）
                for (int rr = 0; rr < H; ++rr) {
                    float* row = depth32.ptr<float>(rr);
                    for (int cc = 0; cc < W; ++cc) {
                        float z = depth_buf[rr*W + cc];
                        row[cc] = (std::isfinite(z) ? z : 0.0f);
                    }
                }

                // 压缩 8UC3（便于可视化）
                for (int rr = 0; rr < H; ++rr) {
                    uint8_t* row = depth8c1.ptr<uint8_t>(rr);
                    const float* src = depth32.ptr<float>(rr);
                    for (int cc = 0; cc < W; ++cc) row[cc] = meterToByte(src[cc], depth_max);
                }
                cv::cvtColor(depth8c1, depth8c3, cv::COLOR_GRAY2BGR);
                png_buf.clear();
                cv::imencode(".png", depth8c3, png_buf, png_params);

                sensor_msgs::CompressedImage comp;
                comp.header.stamp = stamp;
                comp.header.frame_id = frame;
                comp.format = "png";
                comp.data = png_buf;
                depthc_pubs[k].publish(comp);
            }
        } else {
            ROS_WARN_THROTTLE(5.0, "No odom yet on %s; skip depth rendering.", sensor_mgr.getOdomTopic().c_str());
        }

        depth_rate.sleep();
    }

    return 0;
}
