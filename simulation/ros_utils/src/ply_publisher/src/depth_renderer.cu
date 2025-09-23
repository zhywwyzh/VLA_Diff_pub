#include "ply_publisher/depth_renderer.h"
#include <cuda_runtime.h>
#include <stdexcept>
#include <cmath>
#include <limits>

using namespace depth_renderer;

static __device__ __forceinline__ float3 mul3x3v(const float R[9], const float3& v) {
  return make_float3(
    R[0]*v.x + R[1]*v.y + R[2]*v.z,
    R[3]*v.x + R[4]*v.y + R[5]*v.z,
    R[6]*v.x + R[7]*v.y + R[8]*v.z
  );
}

static __device__ __forceinline__ void atomicMinFloat(float* addr, float val) {
  unsigned int* uaddr = reinterpret_cast<unsigned int*>(addr);
  unsigned int  old   = atomicAdd(uaddr, 0u);
  while (__uint_as_float(old) > val) {
    unsigned int assumed = old;
    old = atomicCAS(uaddr, assumed, __float_as_uint(val));
    if (old == assumed) break;
  }
}

__global__ void projectAndZMinKernel(
  const float* __restrict__ pts_xyz, int n,
  float* __restrict__ depth, int W, int H,
  float fx, float fy, float cx, float cy,
  float depth_min, float depth_max,
  float t_wb_x, float t_wb_y, float t_wb_z,
  const float* __restrict__ R_ow,
  int r)
{
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;

  float wx = pts_xyz[3*i + 0];
  float wy = pts_xyz[3*i + 1];
  float wz = pts_xyz[3*i + 2];

  float dx = wx - t_wb_x;
  float dy = wy - t_wb_y;
  float dz = wz - t_wb_z;

  float3 pc = mul3x3v(R_ow, make_float3(dx,dy,dz));
  float Z = pc.z;
  if (Z <= depth_min || Z > depth_max) return;

  float uf = fx * pc.x / Z + cx;
  float vf = fy * pc.y / Z + cy;
  int u0 = __float2int_rn(uf);
  int v0 = __float2int_rn(vf);
  if (u0 < 0 || u0 >= W || v0 < 0 || v0 >= H) return;

  int umin = max(0, u0 - r);
  int umax = min(W-1, u0 + r);
  int vmin = max(0, v0 - r);
  int vmax = min(H-1, v0 + r);

  for (int v = vmin; v <= vmax; ++v) {
    int base = v * W;
    for (int u = umin; u <= umax; ++u) {
      atomicMinFloat(&depth[base + u], Z);
    }
  }
}

static inline void gpuCheck(cudaError_t e, const char* msg) {
  if (e != cudaSuccess) {
    throw std::runtime_error(std::string("CUDA Error: ") + msg + " - " + cudaGetErrorString(e));
  }
}

bool depth_renderer::renderDepthCUDA(const float* pts_xyz, int n,
                                     const Pose& pose, const Intrinsics& K,
                                     float* depth_out)
{
  if (!pts_xyz || !depth_out || n <= 0) return false;

  const int W = K.W, H = K.H;
  const size_t depth_bytes = sizeof(float)*W*H;

  float *d_pts=nullptr, *d_depth=nullptr, *d_R=nullptr;
  try {
    gpuCheck(cudaMalloc(&d_pts, sizeof(float)*3*n), "malloc d_pts");
    gpuCheck(cudaMemcpy(d_pts, pts_xyz, sizeof(float)*3*n, cudaMemcpyHostToDevice), "cpy pts");

    gpuCheck(cudaMalloc(&d_depth, depth_bytes), "malloc d_depth");
    // 用 0x7f 填充通常得到 NaN/Inf，用于“极大值”；后面会把非命中转成 +INF
    gpuCheck(cudaMemset(d_depth, 0x7f, depth_bytes), "memset depth");

    gpuCheck(cudaMalloc(&d_R, sizeof(float)*9), "malloc d_R");
    gpuCheck(cudaMemcpy(d_R, pose.R_ow, sizeof(float)*9, cudaMemcpyHostToDevice), "cpy R");

    dim3 block(256);
    dim3 grid((n + block.x - 1)/block.x);
    projectAndZMinKernel<<<grid, block>>>(
      d_pts, n, d_depth, W, H,
      K.fx, K.fy, K.cx, K.cy,
      K.depth_min, K.depth_max,
      pose.t_wb[0], pose.t_wb[1], pose.t_wb[2],
      d_R, K.splat_radius_px
    );
    gpuCheck(cudaPeekAtLastError(), "kernel launch");
    gpuCheck(cudaDeviceSynchronize(), "kernel sync");

    gpuCheck(cudaMemcpy(depth_out, d_depth, depth_bytes, cudaMemcpyDeviceToHost), "cpy back");

    // 归一未命中像素
    for (int i=0;i<W*H;++i) {
      float& z = depth_out[i];
      if (!(z > 0.f) || !std::isfinite(z)) z = std::numeric_limits<float>::infinity();
    }

    cudaFree(d_pts); cudaFree(d_depth); cudaFree(d_R);
    return true;
  } catch (...) {
    if (d_pts)   cudaFree(d_pts);
    if (d_depth) cudaFree(d_depth);
    if (d_R)     cudaFree(d_R);
    return false;
  }
}
