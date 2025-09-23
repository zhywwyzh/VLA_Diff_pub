#pragma once
#include <vector>

namespace depth_renderer {

struct Intrinsics {
  int   W{640}, H{480};
  float fx{442.025f}, fy{442.025f}, cx{320.f}, cy{240.f};
  float depth_min{0.05f}, depth_max{5.0f};
  int   splat_radius_px{1};
};

struct Pose {
  // R_ow: world->optical (row-major 3x3)
  float R_ow[9];
  // t_wb: camera position in world
  float t_wb[3];
};

bool renderDepthCPU(const float* pts_xyz, int n,
                    const Pose& pose, const Intrinsics& K,
                    float* depth_out);

bool renderDepthCUDA(const float* pts_xyz, int n,
                     const Pose& pose, const Intrinsics& K,
                     float* depth_out);

} // namespace depth_renderer
