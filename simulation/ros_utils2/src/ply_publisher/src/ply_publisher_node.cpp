#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

#include <Eigen/Geometry>

#include <vector>
#include <array>
#include <unordered_map>
#include <tuple>
#include <cmath>
#include <limits>
#include <functional>
#include <mutex>

#include "ply_publisher/pc_processor.h"    // 你的点云加载/下采样类
#include "ply_publisher/sensor_manager.h"  // 你的 SensorManager
#include "ply_publisher/ground_align.h"

using PointT = pcl::PointXYZ; // 如需不同类型请修改

//---------------------- 辅助函数 ----------------------
static inline uint8_t meterToByte(float z_m, float depth_max) {
    if (!(z_m > 0.0f)) return 0u;
    float x = z_m / depth_max;
    x = std::min(1.0f, std::max(0.0f, x));
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

// 三维格子键（用于 unordered_map）
struct BlockKey {
    int ix, iy, iz;
    bool operator==(BlockKey const& o) const {
        return ix==o.ix && iy==o.iy && iz==o.iz;
    }
};
struct BlockKeyHash {
    std::size_t operator()(BlockKey const& k) const noexcept {
        uint64_t a = static_cast<uint32_t>(k.ix);
        uint64_t b = static_cast<uint32_t>(k.iy);
        uint64_t c = static_cast<uint32_t>(k.iz);
        uint64_t h = (a * 73856093u) ^ (b * 19349663u) ^ (c * 83492791u);
        return (size_t)h;
    }
};

// block 结构
struct Block {
    Eigen::Vector3f center;
    float half_diag;             // block 对角半径（用于 margin）
    std::vector<int> indices;    // 点的索引（指向原始 cloud->points）
};

//---------------------- main ----------------------
int main(int argc, char** argv) {
    ros::init(argc, argv, "ply_publisher_node");
    ros::NodeHandle nh("~");

    // 参数（可在 launch 中覆盖）
    std::string ply_file;            nh.param<std::string>("ply_file", ply_file, "test.ply");
    std::string point_topic;         nh.param<std::string>("topic_name", point_topic, "/ply_points");
    std::string caminfo_topic;       nh.param<std::string>("camera_info_topic", caminfo_topic, "/camera0/camera_info");

    double depth_rate_hz;            nh.param<double>("depth_rate_hz", depth_rate_hz, 10.0);  // 上限
    double point_rate_hz;            nh.param<double>("point_rate_hz", point_rate_hz, 1.0);

    bool enable_down;                nh.param<bool>("enable_downsample", enable_down, true);
    double leaf_size;                nh.param<double>("leaf_size", leaf_size, 0.01);

    double depth_min;                nh.param<double>("depth_min", depth_min, 0.05);
    bool frustum_culling;            nh.param<bool>("frustum_culling", frustum_culling, true);
    bool use_cropbox;                nh.param<bool>("use_cropbox", use_cropbox, false);
    int render_stride;               nh.param<int>("render_stride", render_stride, 1);
    int png_level;                   nh.param<int>("png_level", png_level, 1);
    png_level = std::min(9, std::max(0, png_level));

    int splat_radius_px;             nh.param<int>("splat_radius_px", splat_radius_px, 1);
    bool use_optical_convention;     nh.param<bool>("use_optical_convention", use_optical_convention, true);

    bool publish_compressed_depth;   nh.param<bool>("publish_compressed_depth", publish_compressed_depth, true);
    bool publish_raw_depth;          nh.param<bool>("publish_raw_depth", publish_raw_depth, false);

    // 分块参数
    double block_size;               nh.param<double>("block_size", block_size, 0.5); // 默认 0.5m 的块

    // unity transform params（如果有）
    std::vector<double> unity_pos_v, unity_rot_v, unity_scale_v;
    nh.param<std::vector<double>>("unity_pos",       unity_pos_v,  std::vector<double>{0, 0, 0});
    nh.param<std::vector<double>>("unity_rot_deg",   unity_rot_v,  std::vector<double>{0, 0, 0});
    nh.param<std::vector<double>>("unity_scale",     unity_scale_v,std::vector<double>{1, 1, 1});

    // ---------------- Prepare sensors & point cloud ----------------
    SensorManager sensor_mgr;
    sensor_mgr.initParams();
    sensor_mgr.setCameraParams(sensor_mgr.getImageWidth(), sensor_mgr.getImageHeight(),
                               sensor_mgr.getFx(), sensor_mgr.getFy(), sensor_mgr.getCx(), sensor_mgr.getCy(),
                               sensor_mgr.getDepthMax());

    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(sensor_mgr.getOdomTopic(), 50,
                                                                &SensorManager::odomCallback, &sensor_mgr);

    PCProcessor pcproc;
    pcproc.setDownsampleEnabled(enable_down);
    pcproc.setLeafSize((float)leaf_size, (float)leaf_size, (float)leaf_size);
    pcproc.setUnityTransform(Eigen::Vector3f(unity_pos_v[0], unity_pos_v[1], unity_pos_v[2]),
                             Eigen::Vector3f(unity_rot_v[0], unity_rot_v[1], unity_rot_v[2]),
                             Eigen::Vector3f(unity_scale_v[0], unity_scale_v[1], unity_scale_v[2]));

    if (!pcproc.loadPLY(ply_file) || !pcproc.process()) {
        ROS_ERROR("Failed to load/process PLY.");
        return 1;
    }

    pcl::PointCloud<PointT>::ConstPtr cloud = pcproc.getProcessedCloud();
    if (!cloud || cloud->points.empty()) {
        ROS_ERROR("Empty point cloud.");
        return 1;
    }

    // ground align（如果需要）
    pcl::PointCloud<PointT>::ConstPtr cloud_aligned_const;
    Eigen::Affine3f T_align = Eigen::Affine3f::Identity();
    ply_publisher::GroundAlign ground_align;
    ground_align.loadParams(nh);
    ground_align.process(cloud, cloud_aligned_const, T_align);
    auto full_cloud = cloud_aligned_const;

    // ---------------- Build spatial blocks ----------------
    ROS_INFO("Building spatial blocks (block_size = %.3f m)...", block_size);
    std::unordered_map<BlockKey, Block, BlockKeyHash> block_map;
    block_map.reserve(1024);

    const float bs = static_cast<float>(block_size);
    auto idx_of = [&](float v)->int { return static_cast<int>(std::floor(v / bs)); };

    for (size_t i = 0; i < full_cloud->points.size(); ++i) {
        const auto &p = full_cloud->points[i];
        BlockKey k{ idx_of(p.x), idx_of(p.y), idx_of(p.z) };
        auto it = block_map.find(k);
        if (it == block_map.end()) {
            Block blk;
            blk.indices.clear();
            blk.center = Eigen::Vector3f(0,0,0);
            blk.half_diag = (std::sqrt(3.0f) * bs) / 2.0f;
            blk.indices.push_back(static_cast<int>(i));
            block_map.emplace(k, std::move(blk));
        } else {
            it->second.indices.push_back(static_cast<int>(i));
        }
    }

    // compute each block center
    for (auto &kv : block_map) {
        Block &b = kv.second;
        Eigen::Vector3f sum(0,0,0);
        for (int idx : b.indices) {
            const auto &pt = full_cloud->points[idx];
            sum += Eigen::Vector3f(pt.x, pt.y, pt.z);
        }
        b.center = sum / static_cast<float>(b.indices.size());
    }

    ROS_INFO("Built %zu spatial blocks (points=%zu).", block_map.size(), full_cloud->points.size());

    // ---------------- Publishers ----------------
    ros::Publisher pc_pub      = nh.advertise<sensor_msgs::PointCloud2>(point_topic, 1, true);
    ros::Publisher caminfo_pub = nh.advertise<sensor_msgs::CameraInfo>(caminfo_topic, 1, true);

    // 四个深度话题（原来用于四个方向）现在都保留，渲染一次并复制发布到这四个 topic
    std::array<ros::Publisher,4> depthc_pubs = {
        nh.advertise<sensor_msgs::CompressedImage>("/camera0/depth/image/compressed", 1),
        nh.advertise<sensor_msgs::CompressedImage>("/camera1/depth/image/compressed", 1),
        nh.advertise<sensor_msgs::CompressedImage>("/camera2/depth/image/compressed", 1),
        nh.advertise<sensor_msgs::CompressedImage>("/camera3/depth/image/compressed", 1)
    };
    std::array<ros::Publisher,4> depth_raw_pubs = {
        nh.advertise<sensor_msgs::Image>("/camera0/depth/image", 1),
        nh.advertise<sensor_msgs::Image>("/camera1/depth/image", 1),
        nh.advertise<sensor_msgs::Image>("/camera2/depth/image", 1),
        nh.advertise<sensor_msgs::Image>("/camera3/depth/image", 1)
    };

    // prepare pointcloud message
    sensor_msgs::PointCloud2 pc2_msg;
    pcl::toROSMsg(*full_cloud, pc2_msg);
    pc2_msg.header.frame_id = "world";

    sensor_msgs::CameraInfo cam_info = sensor_mgr.buildCameraInfo();

    // camera intrinsics
    const int W = sensor_mgr.getImageWidth();
    const int H = sensor_mgr.getImageHeight();
    const double fx = sensor_mgr.getFx(), fy = sensor_mgr.getFy();
    const double cx = sensor_mgr.getCx(), cy = sensor_mgr.getCy();
    const float depth_max = static_cast<float>(sensor_mgr.getDepthMax());

    const double half_h = std::atan(std::max(cx, (W-1-cx)) / fx);
    const double half_v = std::atan(std::max(cy, (H-1-cy)) / fy);

    // only forward yaw = 0
    const float yaw_forward = 0.f;

    std::vector<int> png_params = { cv::IMWRITE_PNG_COMPRESSION, png_level };

    // world->optical conversion matrix (body->optical)
    const Eigen::Matrix3f R_bo = (Eigen::Matrix3f() <<
         0.f, -1.f,  0.f,
         0.f,  0.f, -1.f,
         1.f,  0.f,  0.f).finished();

    // frequency control
    const ros::Duration depth_period(1.0 / std::max(1e-6, depth_rate_hz));
    const ros::Duration point_period(1.0 / std::max(1e-6, point_rate_hz));
    ros::Time last_point_pub = ros::Time(0), last_depth_pub = ros::Time(0), last_odom_stamp = ros::Time(0);

    ros::Rate idle(500.0);

    // MAIN LOOP
    while (ros::ok()) {
        ros::spinOnce();
        const ros::Time now = ros::Time::now();

        // publish point cloud at lower freq
        if ((now - last_point_pub) >= point_period) {
            pc2_msg.header.stamp = now;
            pc_pub.publish(pc2_msg);
            last_point_pub = now;
        }

        if (!sensor_mgr.hasOdom()) {
            ROS_WARN_THROTTLE(5.0, "No odom received yet.");
            idle.sleep();
            continue;
        }

        nav_msgs::Odometry odom = sensor_mgr.getOdomCopy();
        const bool new_odom = (odom.header.stamp != last_odom_stamp);
        const bool under_rate = ((now - last_depth_pub) >= depth_period);

        if (new_odom && under_rate) {
            last_odom_stamp = odom.header.stamp;
            const ros::Time stamp = odom.header.stamp;
            const std::string frame = "world";

            // publish CameraInfo once per depth publish
            cam_info.header.stamp = stamp;
            cam_info.header.frame_id = frame;
            caminfo_pub.publish(cam_info);

            // compute transforms
            const Eigen::Vector3f t_wb(odom.pose.pose.position.x,
                                       odom.pose.pose.position.y,
                                       odom.pose.pose.position.z);
            const Eigen::Quaternionf q_wb(odom.pose.pose.orientation.w,
                                          odom.pose.pose.orientation.x,
                                          odom.pose.pose.orientation.y,
                                          odom.pose.pose.orientation.z);
            const Eigen::Matrix3f R_wb = q_wb.toRotationMatrix();
            const Eigen::Matrix3f R_bw = R_wb.transpose();

            // 1) coarse select candidate blocks for forward yaw
            std::vector<const Block*> candidate_blocks;
            candidate_blocks.reserve(256);

            Eigen::Matrix3f R_ow;
            if (use_optical_convention) {
                R_ow = (R_bo * Rz(yaw_forward) * R_bw);
            } else {
                R_ow = (Rz(yaw_forward) * R_bw);
            }

            for (auto &kv : block_map) {
                const Block &blk = kv.second;
                Eigen::Vector3f p_cam = R_ow * (blk.center - t_wb);
                const float Zc = p_cam.z();
                const float diag = blk.half_diag;

                if (Zc + diag < static_cast<float>(depth_min)) continue;
                if (Zc - diag > depth_max) continue;
                if (Zc <= 1e-6f) continue;

                const float uf = static_cast<float>(fx * p_cam.x() / Zc + cx);
                const float vf = static_cast<float>(fy * p_cam.y() / Zc + cy);
                float pixel_margin_u = (fx * diag) / std::max(1e-6f, std::abs(Zc));
                float pixel_margin_v = (fy * diag) / std::max(1e-6f, std::abs(Zc));

                if (uf + pixel_margin_u < 0 || uf - pixel_margin_u >= W) continue;
                if (vf + pixel_margin_v < 0 || vf - pixel_margin_v >= H) continue;

                candidate_blocks.push_back(&blk);
            }

            // 2) render once (iterate candidate blocks -> points)
            const int stride = std::max(1, render_stride);
            const int r = std::max(0, splat_radius_px);

            std::vector<float> depth_buf(W * H, std::numeric_limits<float>::infinity());

            for (const Block* bp : candidate_blocks) {
                for (int idx : bp->indices) {
                    if ((idx % stride) != 0) continue;
                    const auto &pt = full_cloud->points[idx];
                    const Eigen::Vector3f pw(pt.x, pt.y, pt.z);
                    const Eigen::Vector3f p_cam = R_ow * (pw - t_wb);
                    const float Z = p_cam.z();
                    if (Z <= static_cast<float>(depth_min) || Z > depth_max) continue;

                    if (frustum_culling) {
                        const float X = p_cam.x(), Y = p_cam.y();
                        const float x_lim = static_cast<float>(std::tan(half_h) * Z);
                        if (X < -x_lim || X > x_lim) continue;
                        const float y_lim = static_cast<float>(std::tan(half_v) * Z);
                        if (Y < -y_lim || Y > y_lim) continue;
                    }

                    const float uf = static_cast<float>(fx * p_cam.x() / Z + cx);
                    const float vf = static_cast<float>(fy * p_cam.y() / Z + cy);
                    int u0 = static_cast<int>(std::round(uf));
                    int v0 = static_cast<int>(std::round(vf));
                    if ((unsigned)u0 >= (unsigned)W || (unsigned)v0 >= (unsigned)H) continue;

                    const int umin = std::max(0, u0 - r), umax = std::min(W - 1, u0 + r);
                    const int vmin = std::max(0, v0 - r), vmax = std::min(H - 1, v0 + r);
                    for (int v = vmin; v <= vmax; ++v) {
                        int base = v * W;
                        for (int u = umin; u <= umax; ++u) {
                            int id = base + u;
                            if (Z < depth_buf[id]) depth_buf[id] = Z;
                        }
                    }
                }
            }

            // 3) convert depth_buf -> mats
            cv::Mat depth32(H, W, CV_32FC1);
            cv::Mat depth8c1(H, W, CV_8UC1);
            for (int y = 0; y < H; ++y) {
                float* dst = depth32.ptr<float>(y);
                uint8_t* dst8 = depth8c1.ptr<uint8_t>(y);
                int base = y * W;
                for (int x = 0; x < W; ++x) {
                    float z = depth_buf[base + x];
                    dst[x] = (std::isfinite(z) ? z : 0.0f);
                    dst8[x] = meterToByte(dst[x], depth_max);
                }
            }

            // 4) publish to ALL 4 topics (same image)
            if (publish_raw_depth) {
                cv_bridge::CvImage cv_img;
                cv_img.header.stamp = stamp;
                cv_img.header.frame_id = frame;
                cv_img.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
                cv_img.image = depth32;
                sensor_msgs::Image raw_msg;
                cv_img.toImageMsg(raw_msg);
                for (int i = 0; i < 4; ++i) {
                    depth_raw_pubs[i].publish(raw_msg);
                }
            }

            if (publish_compressed_depth) {
                cv::Mat depth3;
                cv::cvtColor(depth8c1, depth3, cv::COLOR_GRAY2BGR);
                std::vector<uchar> buf;
                cv::imencode(".png", depth3, buf, png_params);
                sensor_msgs::CompressedImage comp;
                comp.header.stamp = stamp;
                comp.header.frame_id = frame;
                comp.format = "png";
                comp.data = buf;
                for (int i = 0; i < 4; ++i) {
                    depthc_pubs[i].publish(comp);
                }
            }

            last_depth_pub = now;
        } // end if new_odom & under_rate

        idle.sleep();
    } // end while

    return 0;
}
