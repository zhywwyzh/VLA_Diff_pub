#ifndef _PC_PROCESSOR_H
#define _PC_PROCESSOR_H

#include <string>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

using PointT = pcl::PointXYZ;
using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;

class PCProcessor {
public:
    PCProcessor() = default;

    // 读取 PLY
    bool loadPLY(const std::string& ply_path);

    // 设置体素下采样尺寸（等边体素）
    void setLeafSize(float lx, float ly, float lz);

    // 启用/禁用下采样
    void setDownsampleEnabled(bool enabled) { enable_downsample_ = enabled; }

    // 设置 Unity->ROS 的 pos/rot_deg/scale（均从 launch 传入）
    void setUnityTransform(const Eigen::Vector3f& unity_pos,
                           const Eigen::Vector3f& unity_rot_deg,
                           const Eigen::Vector3f& unity_scale);

    // 处理流程：若开启则先下采样，再应用变换；若关闭直接应用变换
    bool process();

    // 结果点云（world 坐标系）
    PointCloudPtr getProcessedCloud() const;

    // 实际使用的齐次变换矩阵
    Eigen::Matrix4f getTransformMatrix() const;

private:
    PointCloudPtr cloud_raw_{new pcl::PointCloud<PointT>};
    PointCloudPtr cloud_down_{new pcl::PointCloud<PointT>};
    PointCloudPtr cloud_transformed_{new pcl::PointCloud<PointT>};

    float leaf_x_ = 0.01f, leaf_y_ = 0.01f, leaf_z_ = 0.01f;
    bool  enable_downsample_ = true;

    // Unity 参数
    Eigen::Vector3f unity_pos_{0,0,0};
    Eigen::Vector3f unity_rot_deg_{0,0,0};
    Eigen::Vector3f unity_scale_{1,1,1};

    Eigen::Matrix4f transform_matrix_ = Eigen::Matrix4f::Identity();

    // 根据 Unity->ROS 规则构建 4x4 变换
    void buildTransformMatrix();
};

#endif // PLY_PUBLISHER_PC_PROCESSOR_H
