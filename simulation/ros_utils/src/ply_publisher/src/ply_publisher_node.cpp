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
#include <algorithm>
#include <string>

#include "ply_publisher/pc_processor.h"
#include "ply_publisher/sensor_manager.h"
#include "ply_publisher/ground_align.h"

// ★ GPU/CPU 深度渲染器接口
#include "ply_publisher/depth_renderer.h"

// 将米制深度映射到 8bit 可视化（0~depth_max -> 0~255）
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

// 机体系(x前,y左,z上) -> 相机光学系(x右,y下,z前)
static inline Eigen::Matrix3f R_body_to_optical() {
    // 与你给的 R_bo 完全一致
    Eigen::Matrix3f R;
    R << 0.f, -1.f,  0.f,
         0.f,  0.f, -1.f,
         1.f,  0.f,  0.f;
    return R;
}

// 将 yaw（度）映射到相机编号：0->1, +90->2, -90->3, 180/-180->4；其他返回 0（忽略）
static inline int yawToCamIdx(double yaw_deg) {
    int y = static_cast<int>(std::round(yaw_deg));
    if (y ==   0) return 1;
    if (y ==  90) return 2;
    if (y == -90) return 3;
    if (y == 180 || y == -180) return 4;
    return 0;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ply_publisher_node");
    ros::NodeHandle nh("~");

    // ---- Launch 参数（从 launch 读取）----
    std::string ply_file;            nh.param<std::string>("ply_file", ply_file, "test.ply");
    std::string point_topic;         nh.param<std::string>("topic_name", point_topic, "/ply_points");

    // 保留但实际我们发布到 /camera{1..4}/camera_info
    std::string caminfo_topic;       nh.param<std::string>("camera_info_topic", caminfo_topic, "/camera1/camera_info");

    double depth_rate_hz;            nh.param<double>("depth_rate_hz", depth_rate_hz, 10.0);  // 上限频率
    double point_rate_hz;            nh.param<double>("point_rate_hz", point_rate_hz, 1.0);

    bool enable_down;                nh.param<bool>("enable_downsample", enable_down, true);
    double leaf_size;                nh.param<double>("leaf_size", leaf_size, 0.01);
    double depth_min;                nh.param<double>("depth_min", depth_min, 0.05);
    bool frustum_culling;            nh.param<bool>("frustum_culling", frustum_culling, true);
    bool use_cropbox;                nh.param<bool>("use_cropbox", use_cropbox, false);
    int render_stride;               nh.param<int>("render_stride", render_stride, 1);
    int png_level;                   nh.param<int>("png_level", png_level, 1);
    png_level = std::min(9, std::max(0, png_level));

    int splat_radius_px;             nh.param<int>("splat_radius_px", splat_radius_px, 1);   // 0=点投影；1=3x3；2=5x5...
    bool use_optical_convention;     nh.param<bool>("use_optical_convention", use_optical_convention, true);

    // 新增发布开关（从 launch 读取）
    bool publish_compressed_depth;   nh.param<bool>("publish_compressed_depth", publish_compressed_depth, true);
    bool publish_raw_depth;          nh.param<bool>("publish_raw_depth", publish_raw_depth, false);

    // 新增：GPU 开关
    bool use_cuda = false;           nh.param<bool>("use_cuda", use_cuda, false);

    // 新增：要渲染的 yaw 列表（控制渲染哪些方向；映射到 camera1..4）
    std::vector<double> render_yaw_deg_list;
    nh.param<std::vector<double>>("render_yaw_deg_list", render_yaw_deg_list,
                                  std::vector<double>{0.0, 90.0, -90.0, 180.0});

    // Unity 类变换（若需要）
    std::vector<double> unity_pos_v, unity_rot_v, unity_scale_v;
    nh.param<std::vector<double>>("unity_pos",       unity_pos_v,  std::vector<double>{0, 0, 0});
    nh.param<std::vector<double>>("unity_rot_deg",   unity_rot_v,  std::vector<double>{0, 0, 0});
    nh.param<std::vector<double>>("unity_scale",     unity_scale_v,std::vector<double>{1, 1, 1});

    // ---- 相机/里程计管理 ----
    SensorManager sensor_mgr;
    sensor_mgr.initParams();
    sensor_mgr.setCameraParams(sensor_mgr.getImageWidth(), sensor_mgr.getImageHeight(),
                               sensor_mgr.getFx(), sensor_mgr.getFy(), sensor_mgr.getCx(), sensor_mgr.getCy(),
                               sensor_mgr.getDepthMax());
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(sensor_mgr.getOdomTopic(), 50,
                                                                &SensorManager::odomCallback, &sensor_mgr);

    // ---- 点云准备 ----
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

    // ---- ROS Publishers ----
    ros::Publisher pc_pub      = nh.advertise<sensor_msgs::PointCloud2>(point_topic, 1, true);

    // 相机信息（四路）
    std::array<ros::Publisher,4> caminfo_pubs = {
        nh.advertise<sensor_msgs::CameraInfo>("/camera1/camera_info", 1, true),
        nh.advertise<sensor_msgs::CameraInfo>("/camera2/camera_info", 1, true),
        nh.advertise<sensor_msgs::CameraInfo>("/camera3/camera_info", 1, true),
        nh.advertise<sensor_msgs::CameraInfo>("/camera4/camera_info", 1, true)
    };

    // 压缩深度图发布器（固定话题名）
    std::array<ros::Publisher,4> depthc_pubs = {
        nh.advertise<sensor_msgs::CompressedImage>("/camera1/depth/image/compressed", 1),
        nh.advertise<sensor_msgs::CompressedImage>("/camera2/depth/image/compressed", 1),
        nh.advertise<sensor_msgs::CompressedImage>("/camera3/depth/image/compressed", 1),
        nh.advertise<sensor_msgs::CompressedImage>("/camera4/depth/image/compressed", 1)
    };

    // 未压缩 32FC1 深度图（单位米）
    std::array<ros::Publisher,4> depth_raw_pubs = {
        nh.advertise<sensor_msgs::Image>("/camera1/depth/image", 1),
        nh.advertise<sensor_msgs::Image>("/camera2/depth/image", 1),
        nh.advertise<sensor_msgs::Image>("/camera3/depth/image", 1),
        nh.advertise<sensor_msgs::Image>("/camera4/depth/image", 1)
    };

    // 点云一次 latched 发布
    sensor_msgs::PointCloud2 pc2_msg;
    pcl::toROSMsg(*cloud_aligned_const, pc2_msg);
    pc2_msg.header.frame_id = "world";

    // 频率控制（作为上限）
    const ros::Duration depth_period(1.0 / std::max(1e-6, depth_rate_hz));
    const ros::Duration point_period(1.0 / std::max(1e-6, point_rate_hz));

    ros::Time last_point_pub     = ros::Time(0);
    ros::Time last_depth_pub     = ros::Time(0);
    ros::Time last_odom_stamp    = ros::Time(0);  // 已处理过的 odom 时间戳

    // 预分配渲染缓冲
    const int W = sensor_mgr.getImageWidth();
    const int H = sensor_mgr.getImageHeight();
    const double fx = sensor_mgr.getFx(), fy = sensor_mgr.getFy();
    const double cx = sensor_mgr.getCx(), cy = sensor_mgr.getCy();
    const float depth_max = static_cast<float>(sensor_mgr.getDepthMax());

    // 视场（用 K 推半视角）
    const double half_h_left  = std::atan(cx / fx);
    const double half_h_right = std::atan((W - 1 - cx) / fx);
    const double half_h = std::max(half_h_left, half_h_right);
    const double half_v_up    = std::atan(cy / fy);
    const double half_v_down  = std::atan((H - 1 - cy) / fy);
    const double half_v = std::max(half_v_up, half_v_down);

    auto full_cloud = cloud_aligned_const;

    // 机体系->光学系
    const Eigen::Matrix3f R_bo = R_body_to_optical();

    // CUDA/CPU 渲染器内参
    depth_renderer::Intrinsics K;
    K.W = W; K.H = H;
    K.fx = static_cast<float>(fx); K.fy = static_cast<float>(fy);
    K.cx = static_cast<float>(cx); K.cy = static_cast<float>(cy);
    K.depth_min = static_cast<float>(depth_min);
    K.depth_max = depth_max;
    K.splat_radius_px = splat_radius_px;

    // 每帧临时缓冲
    std::vector<float> depth_buf(W * H, 0.0f);  // 存 32F 米
    cv::Mat depth32(H, W, CV_32FC1);
    cv::Mat depth8c1(H, W, CV_8UC1);
    cv::Mat depth8c3(H, W, CV_8UC3);
    std::vector<uchar> png_buf;
    std::vector<int> png_params = { cv::IMWRITE_PNG_COMPRESSION, png_level };

    // 点云打平成连续 float* 供渲染器使用（每次 CropBox 后也可重建，这里先全量）
    std::vector<float> cloud_xyz_flat;
    cloud_xyz_flat.reserve(full_cloud->size() * 3);

    // 主循环
    ros::Rate idle(500.0);  // 主循环空转最高 500Hz

    while (ros::ok()) {
        ros::spinOnce();

        const ros::Time now = ros::Time::now();

        // 点云按频率发布（与 depth 解耦）
        if ((now - last_point_pub) >= point_period) {
            pc2_msg.header.stamp = now;
            pc2_msg.header.frame_id = "world";
            pc_pub.publish(pc2_msg);
            last_point_pub = now;
        }

        // 只有在收到“新”的 odom 且未超出发布上限频率时，才渲染并发布一次 depth
        if (sensor_mgr.hasOdom()) {
            nav_msgs::Odometry odom = sensor_mgr.getOdomCopy();

            // 判定是否新里程计：时间戳变化
            const bool new_odom = (odom.header.stamp != last_odom_stamp);
            const bool under_rate = ((now - last_depth_pub) >= depth_period);

            if (new_odom && under_rate) {
                last_odom_stamp = odom.header.stamp;  // 标记已处理
                const ros::Time stamp = odom.header.stamp;
                const std::string frame = "world";

                // 位姿：R_wb（body->world）与 t_wb
                const Eigen::Vector3f t_wb(odom.pose.pose.position.x,
                                           odom.pose.pose.position.y,
                                           odom.pose.pose.position.z);
                const Eigen::Quaternionf q_wb(odom.pose.pose.orientation.w,
                                              odom.pose.pose.orientation.x,
                                              odom.pose.pose.orientation.y,
                                              odom.pose.pose.orientation.z);
                const Eigen::Matrix3f R_wb = q_wb.toRotationMatrix(); // body->world
                const Eigen::Matrix3f R_bw = R_wb.transpose();        // world->body

                // 可选 CropBox（使用“前向”光学系姿态估算裁剪盒朝向）
                pcl::PointCloud<PointT>::ConstPtr cloud_to_render = full_cloud;
                pcl::PointCloud<PointT>::Ptr cropped(new pcl::PointCloud<PointT>);
                if (use_cropbox) {
                    float x_lim_max = static_cast<float>(std::tan(half_h) * depth_max);
                    float y_lim_max = static_cast<float>(std::tan(half_v) * depth_max);

                    pcl::CropBox<PointT> crop;
                    crop.setMin(Eigen::Vector4f(-x_lim_max, -y_lim_max, static_cast<float>(depth_min), 1.0f));
                    crop.setMax(Eigen::Vector4f( x_lim_max,  y_lim_max, depth_max, 1.0f));

                    // “正前方” optical->world 姿态（yaw=0）
                    Eigen::Matrix3f R_ow_front = use_optical_convention ? (R_bo * R_bw) : (R_bw);
                    const Eigen::Matrix3f R_wo_front = R_ow_front.transpose();
                    Eigen::Vector3f rpy_wo = R_wo_front.eulerAngles(0,1,2);
                    crop.setRotation(Eigen::Vector3f(rpy_wo[0], rpy_wo[1], rpy_wo[2]));
                    crop.setTranslation(t_wb);

                    crop.setInputCloud(full_cloud);
                    crop.filter(*cropped);
                    cloud_to_render = cropped;
                }

                // 打平成连续 xyz 数组（如启用 CropBox 则以裁剪结果为准）
                cloud_xyz_flat.clear();
                cloud_xyz_flat.reserve(cloud_to_render->size() * 3);
                for (const auto& pt : *cloud_to_render) {
                    cloud_xyz_flat.push_back(pt.x);
                    cloud_xyz_flat.push_back(pt.y);
                    cloud_xyz_flat.push_back(pt.z);
                }

                // 相机内参（四路相同）提前准备 & 发布（与本次 depth 对齐）
                sensor_msgs::CameraInfo cam_info;
                cam_info.header.stamp = stamp;
                cam_info.header.frame_id = frame;
                cam_info.width  = W; cam_info.height = H;
                cam_info.K = {fx, 0,  cx,
                              0,  fy, cy,
                              0,  0,  1};
                cam_info.P = {fx, 0,  cx, 0,
                              0,  fy, cy, 0,
                              0,  0,  1,  0};
                cam_info.D.clear(); cam_info.distortion_model = "plumb_bob";

                // 遍历用户指定的 yaw 列表（只处理 0/+90/-90/±180）
                (void)render_stride; // 当前未用到，避免未使用告警
                for (double yaw_deg : render_yaw_deg_list) {
                    const int cam_idx = yawToCamIdx(yaw_deg);
                    if (cam_idx == 0) continue; // 未映射到 1..4 的角度忽略

                    // world->optical 的组合：在 body 内 yaw
                    const float yaw_rad = static_cast<float>(yaw_deg * M_PI / 180.0);
                    const Eigen::Matrix3f Rz_mat = Rz(yaw_rad);

                    Eigen::Matrix3f R_ow_eig;
                    if (use_optical_convention) {
                        R_ow_eig = (R_bo * Rz_mat * R_bw).eval();
                    } else {
                        R_ow_eig = (Rz_mat * R_bw).eval();
                    }

                    // 填给渲染器的 Pose
                    depth_renderer::Pose pose;
                    for (int r=0;r<3;++r)
                        for (int c=0;c<3;++c)
                            pose.R_ow[3*r+c] = R_ow_eig(r,c);
                    pose.t_wb[0] = t_wb.x();
                    pose.t_wb[1] = t_wb.y();
                    pose.t_wb[2] = t_wb.z();

                    // 渲染（GPU/CPU 二选一）
                    bool ok = false;
                    if (use_cuda) {
                        ok = depth_renderer::renderDepthCUDA(cloud_xyz_flat.data(),
                                                             static_cast<int>(cloud_xyz_flat.size()/3),
                                                             pose, K, depth_buf.data());
                    } else {
                        ok = depth_renderer::renderDepthCPU (cloud_xyz_flat.data(),
                                                             static_cast<int>(cloud_xyz_flat.size()/3),
                                                             pose, K, depth_buf.data());
                    }
                    if (!ok) {
                        ROS_WARN_THROTTLE(1.0, "renderDepth failed at yaw %.1f (cam%d)", yaw_deg, cam_idx);
                        continue;
                    }

                    // 拷到 32F Mat，未命中像素（+INF）置 0
                    for (int rr = 0; rr < H; ++rr) {
                        float* row = depth32.ptr<float>(rr);
                        const float* src = depth_buf.data() + rr*W;
                        for (int cc = 0; cc < W; ++cc) {
                            float z = src[cc];
                            row[cc] = (std::isfinite(z) && z>0.f && z<=depth_max) ? z : 0.0f;
                        }
                    }

                    // 发布 CameraInfo（四路相同参数，也单独发布）
                    caminfo_pubs[cam_idx-1].publish(cam_info);

                    // （可选）发布未压缩 32FC1
                    if (publish_raw_depth) {
                        cv_bridge::CvImage cv_img;
                        cv_img.header.stamp = stamp;
                        cv_img.header.frame_id = frame;
                        cv_img.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
                        cv_img.image = depth32;
                        sensor_msgs::Image raw_msg;
                        cv_img.toImageMsg(raw_msg);
                        depth_raw_pubs[cam_idx-1].publish(raw_msg);
                    }

                    // （可选）发布压缩 PNG（8UC3 可视化）
                    if (publish_compressed_depth) {
                        cv::Mat depth8c1_local(H, W, CV_8UC1);
                        for (int rr = 0; rr < H; ++rr) {
                            uint8_t* row = depth8c1_local.ptr<uint8_t>(rr);
                            const float* src = depth32.ptr<float>(rr);
                            for (int cc = 0; cc < W; ++cc) row[cc] = meterToByte(src[cc], depth_max);
                        }
                        cv::Mat depth8c3_local;
                        cv::cvtColor(depth8c1_local, depth8c3_local, cv::COLOR_GRAY2BGR);
                        std::vector<uchar> png_local;
                        cv::imencode(".png", depth8c3_local, png_local, png_params);

                        sensor_msgs::CompressedImage comp;
                        comp.header.stamp = stamp;
                        comp.header.frame_id = frame;
                        comp.format = "png";
                        comp.data = std::move(png_local);
                        depthc_pubs[cam_idx-1].publish(comp);
                    }
                }

                last_depth_pub = now;  // 记录节流时间点
            }
        } else {
            ROS_WARN_THROTTLE(5.0, "No odom yet on %s; skip depth rendering.", sensor_mgr.getOdomTopic().c_str());
        }

        idle.sleep();
    }

    return 0;
}
