#pragma once

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>

// 项目里统一的点类型
using PointT = pcl::PointXYZ;

namespace ply_publisher {

// 参数配置
struct GroundAlignParams {
  bool  enable      = true;   // 是否启用地面对齐
  float ransac_dist = 0.02f;  // RANSAC 内点阈值 (m)
  float eps_deg     = 45.0f;  // 与Z轴夹角容差 (deg)
  int   max_iter    = 2000;   // RANSAC 最大迭代
};

// 单模块完成：参数加载 + RANSAC 拟合 + 对齐
class GroundAlign {
public:
  GroundAlign() = default;

  // 从 ~namesapce 读取参数：enable_ground_align、ground_ransac_dist、ground_eps_deg、ground_max_iter
  void loadParams(ros::NodeHandle& nh_priv) {
    nh_priv.param<bool>("enable_ground_align", params_.enable, params_.enable);
    double dtmp = params_.ransac_dist;
    nh_priv.param<double>("ground_ransac_dist", dtmp, dtmp);
    params_.ransac_dist = static_cast<float>(dtmp);

    dtmp = params_.eps_deg;
    nh_priv.param<double>("ground_eps_deg", dtmp, dtmp);
    params_.eps_deg = static_cast<float>(dtmp);

    nh_priv.param<int>("ground_max_iter", params_.max_iter, params_.max_iter);
  }

  const GroundAlignParams& params() const { return params_; }

  // 处理：若启用则拟合并对齐；失败回退输入。返回 true 表示执行了“对齐流程”（成功/失败都会返回 true），false 表示未启用直接透传。
  bool process(const pcl::PointCloud<PointT>::ConstPtr& in_cloud,
               pcl::PointCloud<PointT>::ConstPtr& out_cloud_const,
               Eigen::Affine3f& T_out);

private:
  // 实际对齐实现
  static bool alignToZ0(const pcl::PointCloud<PointT>::ConstPtr& in_cloud,
                        pcl::PointCloud<PointT>::Ptr& out_cloud,
                        Eigen::Affine3f& T_out,
                        const GroundAlignParams& p);

  GroundAlignParams params_;
};

} // namespace ply_publisher
