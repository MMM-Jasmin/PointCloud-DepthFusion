#pragma once

// CUDA
#include <cuda.h>
#include <cuda_runtime.h>
#define CUDA_THREADS_PER_BLOCK 1024
#define CUDA_THREADS_PER_BLOCK_2D 32
// CUDA NPP
#include <npp.h>
#include <nppi.h>

namespace Kernels {
extern bool cuda_initialized;

// Cuda error handling
void checkError(const char* msg);
void checkError(cudaError_t error, const char* msg);
void checkNppError(NppStatus error, const char* msg);
// Helper functions
unsigned calc_block_size(unsigned pixel_count, unsigned thread_count);

// Cuda device kernels
__device__ void deproject_pixel_to_point(float points[3], const struct Intrinsics* intrin, const float pixel[2],
                                         float depth);
__device__ void deproject_pixel_to_point(float4* point, const float2 pixel, const float depth, const float color,
                                         const struct Intrinsics* intrin);
__device__ void deproject_pixel_to_point(float4* points, const float2 pixel, const float depth,
                                         const struct Intrinsics* intrin);
__device__ static void project_point_to_pixel(float pixel[2], const struct Intrinsics* intrin, const float point[3]);
__device__ static float atomicMinFloat(float* address, float val);
__device__ static void transform_point_to_point(float to_point[3], const struct Extrinsics* extrin,
                                                const float from_point[3]);
__device__ void kernel_transfer_pixels(int2* mapped_pixels, const Intrinsics* depth_intrin,
                                       const Intrinsics* color_intrin, const Extrinsics* depth_to_color,
                                       float depth_val, int depth_x, int depth_y, int block_index);
// Cuda kernels
__global__ void kernel_transform(float4* points, const float* transform, const int count);
__global__ void kernel_deproject_depth(float4* points, const uint16_t* depth, const uint8_t* color,
                                       const Intrinsics* intrinsics, const uint* image_indices,
                                       const uint indices_count, const float depth_scale);
__global__ void kernel_deproject_depth(float4* points, const uint16_t* depth, const Intrinsics* intrinsics,
                                       const uint* image_indices, const uint indices_count, const float depth_scale);
__global__ void kernel_filter_depth_minmax(bool* mask, int* mask_count, const uint16_t* depth, const int count,
                                           const float depth_scale, const float min_depth, const float max_depth);
__global__ void kernel_project_pointcloud(uint8_t* color, const float4* points, const int count,
                                          const Intrinsics* intrinsics, float* z_buffer, bool mirror_image);
__global__ void kernel_align_depth_to_color(uint16_t* aligned_out, const uint16_t* depth_in, const int2* mapped_pixels,
                                            const Intrinsics* depth_intrin, const Intrinsics* other_intrin);
__global__ void kernel_map_depth_to_color(int2* mapped_pixels, const uint16_t* depth_in, const Intrinsics* depth_intrin,
                                          const Intrinsics* color_intrin, const Extrinsics* depth_to_other,
                                          float depth_scale);
__global__ void kernel_replace_to_zero(uint16_t* aligned_out, const Intrinsics* color_intrin);

// Cuda functions
void init_cuda(bool verbose = false);
void filter_depth_minmax_npp(uint16_t* dev_depth, const uint width, const uint height, const float min_depth,
                             const float max_depth, const float depth_scale, NppStreamContext& npp_context);
void filter_create_mask_npp(uint16_t* dev_depth, uint8_t* dev_mask, const uint width, const uint height,
                            const int roi[4], NppStreamContext& npp_context);
void filter_mask_open_npp(uint8_t* dev_mask, uint8_t* dev_mask_buffer, const uint width, const uint height,
                          cudaStream_t& stream, NppStreamContext& npp_context);
void filter_mask_close_npp(uint8_t* dev_mask, uint8_t* dev_mask_buffer, const uint width, const uint height,
                           cudaStream_t& stream, NppStreamContext& npp_context);
void filter_mask_count_npp(uint& mask_count, uint8_t* dev_mask, const uint width, const uint height,
                           cudaStream_t& stream, NppStreamContext& npp_context);
void create_img_indices(uint* dev_indices, uint8_t* dev_mask, uint pixel_count, cudaStream_t& stream);
void deproject_depth_color(float* dev_points, const uint16_t* dev_depth, const uint8_t* dev_color,
                           const Intrinsics* dev_intrinsics, const uint* dev_indices, const uint indices_count,
                           const uint pixel_count, const float depth_scale, cudaStream_t& stream);
void deproject_depth(float* dev_points, const uint16_t* dev_depth, const Intrinsics* dev_intrinsics,
                     const uint* dev_indices, const uint indices_count, const uint pixel_count, const float depth_scale,
                     cudaStream_t& stream);
void transform(float* dev_points, const uint point_count, const float* dev_transform, cudaStream_t& stream);
void project_pointcloud(uint8_t* dev_color, float* dev_z_buffer, const uint pixel_count, const float* dev_points,
                        const uint point_count, const Intrinsics* dev_intrinsics, bool mirror_image,
                        cudaStream_t& stream);
void filter_color_median_npp(uint8_t* dev_color, uint8_t* dev_color_buffer, const uint width, const uint height,
                             cudaStream_t& stream, NppStreamContext& npp_context);
void filter_color_gauss_npp(uint8_t* dev_color, uint8_t* dev_color_buffer, const uint width, const uint height,
                            cudaStream_t& stream, NppStreamContext& npp_context);
void filter_depth_median_npp(uint16_t* dev_depth, const uint width, const uint height, cudaStream_t& stream,
                             NppStreamContext& npp_context);
void filter_depth_gauss_npp(uint16_t* dev_depth, const uint width, const uint height, cudaStream_t& stream,
                            NppStreamContext& npp_context);
void align_depth_to_color(unsigned char* dev_depth_aligned_out, const uint16_t* dev_depth_in, float depth_scale,
                          const Intrinsics& depth_intrinsics, const Intrinsics* dev_depth_intrinsics,
                          const Extrinsics* dev_depth_to_color_extrinsics, const Intrinsics& color_intrinsics,
                          const Intrinsics* dev_color_intrinsics, const int depth_pixel_count,
                          const int color_pixel_count, int2* dev_pixel_map, cudaStream_t& stream);
void filter_depth_bilateral_gauss_npp(uint16_t* dev_depth, const uint width, const uint height, cudaStream_t& stream,
                                      NppStreamContext& npp_context);
}  // namespace Kernels
