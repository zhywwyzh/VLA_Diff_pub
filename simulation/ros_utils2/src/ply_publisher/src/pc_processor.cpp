#include "ply_publisher/pc_processor.h"
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <ros/ros.h>

bool PCProcessor::loadPLY(const std::string& ply_path)
{
    if (pcl::io::loadPLYFile<PointT>(ply_path, *cloud_raw_) == -1) {
        ROS_ERROR_STREAM("PCProcessor: cannot read PLY: " << ply_path);
        return false;
    }
    ROS_INFO_STREAM("PCProcessor: loaded PLY " << ply_path << " points: " << cloud_raw_->size());
    return true;
}

void PCProcessor::setLeafSize(float lx, float ly, float lz)
{
    leaf_x_ = lx; leaf_y_ = ly; leaf_z_ = lz;
}

// Unity(x,y,z) -> ROS(x,z,y): 交换 y<->z
void PCProcessor::buildTransformMatrix()
{
    // 平移：ROS = (ux, uz, uy)
    const float ux = unity_pos_.x(), uy = unity_pos_.y(), uz = unity_pos_.z();
    Eigen::Vector3f translation_ros(ux, uz, uy);

    // 缩放：ROS = (sx, sz, sy)
    const float sx = unity_scale_.x(), sy = unity_scale_.y(), sz = unity_scale_.z();
    Eigen::Vector3f scale_ros(sx, sz, sy);

    // 旋转：角度转弧度
    const float rx = unity_rot_deg_.x() * M_PI / 180.0f;
    const float ry = unity_rot_deg_.y() * M_PI / 180.0f;
    const float rz = unity_rot_deg_.z() * M_PI / 180.0f;

    // ros::ROS_INFO("PCProcessor: Unity->ROS transform:");
    ROS_INFO("  Unity pos (x,y,z):      [%.3f, %.3f, %.3f]", ux, uy, uz);
    ROS_INFO("  Unity rot (deg, x,y,z): [%.3f, %.3f, %.3f]", unity_rot_deg_.x(), unity_rot_deg_.y(), unity_rot_deg_.z());
    ROS_INFO("  Unity scale (x,y,z):    [%.3f, %.3f, %.3f]", sx, sy, sz);


    Eigen::AngleAxisf rot_x(rx, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y(ry, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z(rz, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rotation = (rot_z * rot_y * rot_x).toRotationMatrix();

    transform_matrix_ = Eigen::Matrix4f::Identity();
    transform_matrix_.block<3,3>(0,0) = rotation * scale_ros.asDiagonal();
    transform_matrix_.block<3,1>(0,3) = translation_ros;
}

void PCProcessor::setUnityTransform(const Eigen::Vector3f& unity_pos,
                                    const Eigen::Vector3f& unity_rot_deg,
                                    const Eigen::Vector3f& unity_scale)
{
    unity_pos_ = unity_pos;
    unity_rot_deg_ = unity_rot_deg;
    unity_scale_ = unity_scale;
    buildTransformMatrix();
}

bool PCProcessor::process()
{
    if (!cloud_raw_ || cloud_raw_->empty()) {
        ROS_ERROR("PCProcessor: no raw cloud to process");
        return false;
    }

    if (enable_downsample_) {
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud(cloud_raw_);
        sor.setLeafSize(leaf_x_, leaf_y_, leaf_z_);
        sor.filter(*cloud_down_);
        ROS_INFO_STREAM("PCProcessor: downsampled points: " << cloud_down_->size());
    } else {
        *cloud_down_ = *cloud_raw_;
        ROS_INFO_STREAM("PCProcessor: downsample DISABLED, using raw points: " << cloud_down_->size());
    }

    // 应用 Unity->ROS 变换
    pcl::transformPointCloud(*cloud_down_, *cloud_transformed_, transform_matrix_);
    ROS_INFO_STREAM("PCProcessor: transformed cloud size: " << cloud_transformed_->size());

    return true;
}

PointCloudPtr PCProcessor::getProcessedCloud() const
{
    return cloud_transformed_;
}

Eigen::Matrix4f PCProcessor::getTransformMatrix() const
{
    return transform_matrix_;
}
