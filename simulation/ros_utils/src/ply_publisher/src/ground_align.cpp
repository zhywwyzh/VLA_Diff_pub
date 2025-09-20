#include "ply_publisher/ground_align.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

namespace ply_publisher {

bool GroundAlign::process(const pcl::PointCloud<PointT>::ConstPtr& in_cloud,
                          pcl::PointCloud<PointT>::ConstPtr& out_cloud_const,
                          Eigen::Affine3f& T_out) {
  T_out = Eigen::Affine3f::Identity();

  if (!params_.enable) {
    out_cloud_const = in_cloud;
    ROS_INFO_THROTTLE(5.0, "GroundAlign: disabled, passthrough.");
    return false;
  }

  pcl::PointCloud<PointT>::Ptr aligned;
  bool ok = GroundAlign::alignToZ0(in_cloud, aligned, T_out, params_);
  if (ok) {
    out_cloud_const = aligned; // Ptr -> ConstPtr
  } else {
    out_cloud_const = in_cloud;
  }
  return true;
}

bool GroundAlign::alignToZ0(const pcl::PointCloud<PointT>::ConstPtr& in_cloud,
                            pcl::PointCloud<PointT>::Ptr& out_cloud,
                            Eigen::Affine3f& T_out,
                            const GroundAlignParams& p) {
  if (!in_cloud || in_cloud->empty()) {
    ROS_WARN("GroundAlign: empty input cloud.");
    return false;
  }

  // 1) RANSAC 近水平平面
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(p.max_iter);
  seg.setDistanceThreshold(p.ransac_dist);
  seg.setAxis(Eigen::Vector3f::UnitZ());
  seg.setEpsAngle(p.eps_deg * static_cast<float>(M_PI/180.0));

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
  seg.setInputCloud(in_cloud);
  seg.segment(*inliers, *coeff);

  if (!coeff || coeff->values.size() < 4 || inliers->indices.empty()) {
    ROS_WARN("GroundAlign: RANSAC failed.");
    return false;
  }

  // 平面 ax+by+cz+d=0
  Eigen::Vector3f n(coeff->values[0], coeff->values[1], coeff->values[2]);
  if (n.norm() < 1e-6f) {
    ROS_WARN("GroundAlign: invalid normal.");
    return false;
  }
  n.normalize();

  // 地面内点质心
  pcl::PointCloud<PointT>::Ptr ground(new pcl::PointCloud<PointT>);
  pcl::ExtractIndices<PointT> ex;
  ex.setInputCloud(in_cloud);
  ex.setIndices(inliers);
  ex.filter(*ground);

  Eigen::Vector4f c4;
  pcl::compute3DCentroid(*ground, c4);
  Eigen::Vector3f p0 = c4.head<3>();

  // 法向朝上
  if (n.dot(Eigen::Vector3f::UnitZ()) < 0.f) n = -n;

  // 2) 旋转：n -> +Z
  Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(n, Eigen::Vector3f::UnitZ());
  Eigen::Matrix3f R = q.toRotationMatrix();

  // 旋转后地面质心高度
  Eigen::Vector3f p0_rot = R * p0;

  // 3) 平移：落到 z=0
  float tz = -p0_rot.z();
  Eigen::Translation3f trans(0.f, 0.f, tz);

  T_out = trans * Eigen::Affine3f(R);

  // 4) 应用到整云
  out_cloud.reset(new pcl::PointCloud<PointT>);
  pcl::transformPointCloud(*in_cloud, *out_cloud, T_out.matrix());

  ROS_INFO("GroundAlign: aligned dz=%.4f m, inliers=%zu / %zu",
           tz, ground->size(), in_cloud->size());
  return true;
}

} // namespace ply_publisher
