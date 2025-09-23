#include "ply_publisher/depth_renderer.h"
#include <algorithm>
#include <limits>
#include <cmath>

using namespace depth_renderer;

bool depth_renderer::renderDepthCPU(const float* pts_xyz, int n,
                                    const Pose& pose, const Intrinsics& K,
                                    float* depth_out)
{
  if (!pts_xyz || !depth_out || n <= 0) return false;

  const int W = K.W, H = K.H;
  const float INFV = std::numeric_limits<float>::infinity();
  std::fill(depth_out, depth_out + W*H, INFV);

  auto zmin = [&](int idx, float z){
    if (z > 0.f && z < depth_out[idx]) depth_out[idx] = z;
  };

  for (int i = 0; i < n; ++i) {
    float wx = pts_xyz[3*i+0];
    float wy = pts_xyz[3*i+1];
    float wz = pts_xyz[3*i+2];

    float dx = wx - pose.t_wb[0];
    float dy = wy - pose.t_wb[1];
    float dz = wz - pose.t_wb[2];

    float x = pose.R_ow[0]*dx + pose.R_ow[1]*dy + pose.R_ow[2]*dz;
    float y = pose.R_ow[3]*dx + pose.R_ow[4]*dy + pose.R_ow[5]*dz;
    float z = pose.R_ow[6]*dx + pose.R_ow[7]*dy + pose.R_ow[8]*dz;

    if (z <= K.depth_min || z > K.depth_max) continue;

    int u0 = (int)std::lround(K.fx * x / z + K.cx);
    int v0 = (int)std::lround(K.fy * y / z + K.cy);
    if (u0 < 0 || u0 >= W || v0 < 0 || v0 >= H) continue;

    const int r = K.splat_radius_px;
    int umin = std::max(0, u0 - r);
    int umax = std::min(W - 1, u0 + r);
    int vmin = std::max(0, v0 - r);
    int vmax = std::min(H - 1, v0 + r);
    for (int v = vmin; v <= vmax; ++v) {
      int base = v * W;
      for (int u = umin; u <= umax; ++u) {
        zmin(base + u, z);
      }
    }
  }
  return true;
}
