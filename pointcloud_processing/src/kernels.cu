// SYSTEM
#include <cmath>
// PROJECT
#include "intrinsics.h"
#include "compactor.cuh"
#include "kernels.cuh"
#include <cuda.h>

bool Kernels::cuda_initialized = false;

struct bool_predicate {
	__host__ __device__ bool operator()(const bool x) { return x; }
};

struct int_predicate {
	__host__ __device__ bool operator()(const int x) { return x > 0; }
};

// Cuda error handling
void Kernels::checkError(const char* msg) {
	cudaError_t error = cudaGetLastError();
	if (error != cudaSuccess) {
		std::cerr << "Cuda error: " << msg << " failed: " << cudaGetErrorString(error) << std::endl;
		exit(EXIT_FAILURE);
	}
}

void Kernels::checkError(cudaError_t error, const char* msg) {
	if (error != cudaSuccess) {
		std::cerr << "Cuda error: " << msg << " failed: " << cudaGetErrorString(error) << std::endl;
		exit(EXIT_FAILURE);
	}
}

void Kernels::checkNppError(NppStatus error, const char* msg) {
	if (error != NPP_NO_ERROR) {
		std::cerr << "Cuda error: " << msg << " failed: " << error << std::endl;
		exit(EXIT_FAILURE);
	}
}

// Helper functions
unsigned Kernels::calc_block_size(unsigned pixel_count, unsigned thread_count) {
	return ((pixel_count % thread_count) == 0) ? (pixel_count / thread_count) : (pixel_count / thread_count + 1);
}

// Cuda device kernels
__device__ void Kernels::deproject_pixel_to_point(float points[3], const struct Intrinsics* intrin,
                                                  const float pixel[2], float depth) {
	assert(intrin->model != RS2_DISTORTION_MODIFIED_BROWN_CONRADY);
	assert(intrin->model != RS2_DISTORTION_FTHETA);

	float x = (pixel[0] - intrin->ppx) / intrin->fx;
	float y = (pixel[1] - intrin->ppy) / intrin->fy;

	if (intrin->model == DISTORTION_INVERSE_BROWN_CONRADY) {
		float r2 = x * x + y * y;
		float f = 1 + intrin->coeffs[0] * r2 + intrin->coeffs[1] * r2 * r2 + intrin->coeffs[4] * r2 * r2 * r2;
		float ux = x * f + 2 * intrin->coeffs[2] * x * y + intrin->coeffs[3] * (r2 + 2 * x * x);
		float uy = y * f + 2 * intrin->coeffs[3] * x * y + intrin->coeffs[2] * (r2 + 2 * y * y);
		x = ux;
		y = uy;
	}

	points[0] = depth * x;
	points[1] = depth * y;
	points[2] = depth;
}

__device__ void Kernels::deproject_pixel_to_point(float4* point, const float2 pixel, const float depth,
                                                  const float color, const struct Intrinsics* intrin) {
	float x = (pixel.x - intrin->ppx) / intrin->fx;
	float y = (pixel.y - intrin->ppy) / intrin->fy;

	point->x = depth * x;
	point->y = depth * y;
	point->z = depth;
	point->w = color;
}

__device__ void Kernels::deproject_pixel_to_point(float4* points, const float2 pixel, const float depth,
                                                  const struct Intrinsics* intrin) {
	float x = (pixel.x - intrin->ppx) / intrin->fx;
	float y = (pixel.y - intrin->ppy) / intrin->fy;

	points->x = depth * x;
	points->y = depth * y;
	points->z = depth;
	points->w = 1.0f;
}

__device__ static void Kernels::project_point_to_pixel(float pixel[2], const struct Intrinsics* intrin,
                                                       const float point[3]) {
	float x = point[0] / point[2], y = point[1] / point[2];

	if (intrin->model == DISTORTION_MODIFIED_BROWN_CONRADY) {
		float r2 = x * x + y * y;
		float f = 1 + intrin->coeffs[0] * r2 + intrin->coeffs[1] * r2 * r2 + intrin->coeffs[4] * r2 * r2 * r2;
		x *= f;
		y *= f;
		float dx = x + 2 * intrin->coeffs[2] * x * y + intrin->coeffs[3] * (r2 + 2 * x * x);
		float dy = y + 2 * intrin->coeffs[3] * x * y + intrin->coeffs[2] * (r2 + 2 * y * y);
		x = dx;
		y = dy;
	}

	if (intrin->model == DISTORTION_FTHETA) {
		float r = sqrtf(x * x + y * y);
		float rd = 1.0f / intrin->coeffs[0] * atanf(2 * r * tanf(intrin->coeffs[0] / 2.0f));
		x *= rd / r;
		y *= rd / r;
	}

	pixel[0] = x * intrin->fx + intrin->ppx;
	pixel[1] = y * intrin->fy + intrin->ppy;
}

__device__ static float Kernels::atomicMinFloat(float* address, float val) {
	int* address_as_i = (int*)address;
	int old = *address_as_i, assumed;
	do {
		assumed = old;
		old = ::atomicCAS(address_as_i, assumed, __float_as_int(::fminf(val, __int_as_float(assumed))));
	} while (assumed != old);
	return __int_as_float(old);
}

__device__ static void Kernels::transform_point_to_point(float to_point[3], const struct Extrinsics* extrin,
                                                         const float from_point[3]) {
	to_point[0] = extrin->rotation[0] * from_point[0] + extrin->rotation[3] * from_point[1] +
	              extrin->rotation[6] * from_point[2] + extrin->translation[0];
	to_point[1] = extrin->rotation[1] * from_point[0] + extrin->rotation[4] * from_point[1] +
	              extrin->rotation[7] * from_point[2] + extrin->translation[1];
	to_point[2] = extrin->rotation[2] * from_point[0] + extrin->rotation[5] * from_point[1] +
	              extrin->rotation[8] * from_point[2] + extrin->translation[2];
}

__device__ void Kernels::kernel_transfer_pixels(int2* mapped_pixels, const Intrinsics* depth_intrin,
                                                const Intrinsics* color_intrin, const Extrinsics* depth_to_color,
                                                float depth_val, int depth_x, int depth_y, int block_index) {
	float shift = block_index ? 0.5 : -0.5;
	auto depth_size = depth_intrin->width * depth_intrin->height;
	auto mapped_index = block_index * depth_size + (depth_y * depth_intrin->width + depth_x);

	if (mapped_index >= depth_size * 2) return;

	if (depth_val == 0) {
		mapped_pixels[mapped_index] = {-1, -1};
		return;
	}

	float depth_pixel[2] = {depth_x + shift, depth_y + shift}, depth_point[3], other_point[3], other_pixel[2];
	deproject_pixel_to_point(depth_point, depth_intrin, depth_pixel, depth_val);
	transform_point_to_point(other_point, depth_to_color, depth_point);
	project_point_to_pixel(other_pixel, color_intrin, other_point);
	mapped_pixels[mapped_index].x = static_cast<int>(other_pixel[0] + 0.5f);
	mapped_pixels[mapped_index].y = static_cast<int>(other_pixel[1] + 0.5f);
}

// Cuda kernels
__global__ void Kernels::kernel_transform(float4* points, const float* transform, const int count) {
	int idx = blockDim.x * blockIdx.x + threadIdx.x;

	if (idx >= count) {
		return;
	}

	// Rotation
	float rotation[9] = {transform[0], transform[1], transform[2], transform[4], transform[5],
	                     transform[6], transform[8], transform[9], transform[10]};

	float x = rotation[0] * points[idx].x + rotation[1] * points[idx].y + rotation[2] * points[idx].z;
	float y = rotation[3] * points[idx].x + rotation[4] * points[idx].y + rotation[5] * points[idx].z;
	float z = rotation[6] * points[idx].x + rotation[7] * points[idx].y + rotation[8] * points[idx].z;

	// Translation
	float3 translation = {transform[3], transform[7], transform[11]};
	points[idx].x = x + translation.x;
	points[idx].y = y + translation.y;
	points[idx].z = z + translation.z;
}

__global__ void Kernels::kernel_deproject_depth(float4* points, const uint16_t* depth, const uint8_t* color,
                                                const Intrinsics* intrinsics, const uint* image_indices,
                                                const uint indices_count, const float depth_scale) {
	int idx = blockDim.x * blockIdx.x + threadIdx.x;

	if (idx >= static_cast<int>(indices_count)) {
		return;
	}

	int img_idx = static_cast<int>(image_indices[idx]);

	int v = img_idx / intrinsics->width;
	int u = img_idx - v * intrinsics->width;
	const float2 pixel = {static_cast<float>(u), static_cast<float>(v)};
	// Store 3 byte rgb color in 4 byte float as bgr* for ros2 pointcloud2 memory structure
	const uint8_t color_uint8[4] = {color[img_idx * 3 + 2], color[img_idx * 3 + 1], color[img_idx * 3], 0};  // bgr*
	// const uint8_t color_uint8[4] = {color[img_idx * 3], color[img_idx * 3 + 1], color[img_idx * 3 + 2], 0};  // rgb*
	// const uint8_t color_uint8[4] = {0, color[img_idx * 3 + 2], color[img_idx * 3 + 1], color[img_idx * 3]}; // *bgr
	// const uint8_t color_uint8[4] = {0, color[img_idx * 3], color[img_idx * 3 + 1], color[img_idx * 3 + 2]}; // *rgb
	deproject_pixel_to_point(points + idx, pixel, depth_scale * static_cast<float>(depth[img_idx]),
	                         *reinterpret_cast<const float*>(color_uint8), intrinsics);
}

__global__ void Kernels::kernel_deproject_depth(float4* points, const uint16_t* depth, const Intrinsics* intrinsics,
                                                const uint* image_indices, const uint indices_count,
                                                const float depth_scale) {
	int idx = blockDim.x * blockIdx.x + threadIdx.x;

	if (idx >= static_cast<int>(indices_count)) {
		return;
	}

	int img_idx = static_cast<int>(image_indices[idx]);

	int v = img_idx / intrinsics->width;
	int u = img_idx - v * intrinsics->width;
	const float2 pixel = {static_cast<float>(u), static_cast<float>(v)};
	deproject_pixel_to_point(points + idx, pixel, depth_scale * static_cast<float>(depth[img_idx]), intrinsics);
}

__global__ void Kernels::kernel_filter_depth_minmax(bool* mask, int* mask_count, const uint16_t* depth, const int count,
                                                    const float depth_scale, const float min_depth,
                                                    const float max_depth) {
	int idx = blockDim.x * blockIdx.x + threadIdx.x;

	if (idx >= count) {
		return;
	}

	float scaled_depth = depth_scale * static_cast<float>(depth[idx]);
	if ((min_depth <= scaled_depth) && (scaled_depth <= max_depth)) {
		mask[idx] = true;
		atomicAdd(mask_count, 1);
	}
}

__global__ void Kernels::kernel_project_pointcloud(uint8_t* color, const float4* points, const int count,
                                                   const Intrinsics* intrinsics, float* z_buffer, bool mirror_image) {
	int idx = blockDim.x * blockIdx.x + threadIdx.x;

	if (idx >= count) {
		return;
	}

	float image_x = intrinsics->ppx + intrinsics->fx * points[idx].x / points[idx].z;
	float image_y = intrinsics->ppy + intrinsics->fy * points[idx].y / points[idx].z;
	int pixel_x = static_cast<int>(image_x + 0.5f);
	int pixel_y = static_cast<int>(image_y + 0.5f);

	// Check image boundaries
	if (pixel_x < 0 || pixel_y < 0 || pixel_x > intrinsics->width - 1 || pixel_y > intrinsics->height - 1) {
		return;
	}
	int image_idx;
	// Mirror image
	if (mirror_image)
		image_idx = pixel_y * intrinsics->width + (intrinsics->width - 1 - pixel_x);
	else
		image_idx = pixel_y * intrinsics->width + pixel_x;

	// Check z-buffer and skip if pixel z-value is smaller than current
	float old_z_val = atomicMinFloat(&z_buffer[image_idx], points[idx].z);
	if (old_z_val > 0 && old_z_val <= points[idx].z) {
		return;
	}

	// 8bit color packed as float on GPU (Pointcloud2 color order): B,G,R,*
	const uint8_t* rgb = reinterpret_cast<const uint8_t*>(&points[idx].w);
	color[image_idx * 3] = rgb[2];
	color[image_idx * 3 + 1] = rgb[1];
	color[image_idx * 3 + 2] = rgb[0];
}

__global__ void Kernels::kernel_align_depth_to_color(uint16_t* aligned_out, const uint16_t* depth_in,
                                                     const int2* mapped_pixels, const Intrinsics* depth_intrin,
                                                     const Intrinsics* color_intrin) {
	int depth_x = blockIdx.x * blockDim.x + threadIdx.x;
	int depth_y = blockIdx.y * blockDim.y + threadIdx.y;

	auto depth_size = depth_intrin->width * depth_intrin->height;
	int depth_pixel_index = depth_y * depth_intrin->width + depth_x;

	if (depth_pixel_index >= depth_intrin->width * depth_intrin->height) return;

	int2 p0 = mapped_pixels[depth_pixel_index];
	int2 p1 = mapped_pixels[depth_size + depth_pixel_index];

	if (p0.x < 0 || p0.y < 0 || p1.x >= color_intrin->width || p1.y >= color_intrin->height) return;

	unsigned int new_val = depth_in[depth_pixel_index];
	unsigned int* arr = (unsigned int*)aligned_out;
	for (int y = p0.y; y <= p1.y; ++y) {
		for (int x = p0.x; x <= p1.x; ++x) {
			auto other_pixel_index = y * color_intrin->width + x;
			new_val = new_val << 16 | new_val;
			atomicMin(&arr[other_pixel_index / 2], new_val);
		}
	}
}

__global__ void Kernels::kernel_map_depth_to_color(int2* mapped_pixels, const uint16_t* depth_in,
                                                   const Intrinsics* depth_intrin, const Intrinsics* color_intrin,
                                                   const Extrinsics* depth_to_other, float depth_scale) {
	int depth_x = blockIdx.x * blockDim.x + threadIdx.x;
	int depth_y = blockIdx.y * blockDim.y + threadIdx.y;

	int depth_pixel_index = depth_y * depth_intrin->width + depth_x;
	if (depth_pixel_index >= depth_intrin->width * depth_intrin->height) return;
	float depth_val = depth_in[depth_pixel_index] * depth_scale;
	kernel_transfer_pixels(mapped_pixels, depth_intrin, color_intrin, depth_to_other, depth_val, depth_x, depth_y,
	                       blockIdx.z);
}

__global__ void Kernels::kernel_replace_to_zero(uint16_t* aligned_out, const Intrinsics* color_intrin) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;

	auto other_pixel_index = y * color_intrin->width + x;
	if (aligned_out[other_pixel_index] == 0xffff) aligned_out[other_pixel_index] = 0;
}

// Cuda functions
void Kernels::init_cuda(bool verbose) {
	cudaDeviceProp device_prop;
	int gpu_device = 0;
	int device_count = 0;

	cudaGetDeviceCount(&device_count);

	if (gpu_device >= device_count) {
		std::cout << "Error initializing CUDA device #" << gpu_device << std::endl;
		exit(1);
	}
	cudaGetDeviceProperties(&device_prop, gpu_device);
	int major = device_prop.major;
	int minor = device_prop.minor;

	cudaSetDevice(0);
	cuda_initialized = true;

	if (verbose) {
		std::cout << "+-- Cuda" << std::endl;
		std::cout << "| Device:             #" << gpu_device << " " << device_prop.name << " SM " << major << "." << minor
		          << std::endl;
		std::cout << "| Multiprocessors:    " << device_prop.multiProcessorCount << std::endl;
		std::cout << "| Max threads per MP: " << device_prop.maxThreadsPerMultiProcessor << std::endl;
		std::cout << "| Threads per block:  " << CUDA_THREADS_PER_BLOCK << std::endl;
	}
}

void Kernels::filter_depth_minmax_npp(uint16_t* dev_depth, const uint width, const uint height, const float min_depth,
                                      const float max_depth, const float depth_scale, NppStreamContext& npp_context) {
	Npp16u* dev_depth_npp = reinterpret_cast<Npp16u*>(dev_depth);
	// Threshold minimum
	Npp16u thresh_lt = static_cast<Npp16u>(min_depth / depth_scale);
	// Threshold maximum
	Npp16u thresh_gt = static_cast<Npp16u>(max_depth / depth_scale);
	// Region of interest size
	NppiSize roi_size = {static_cast<int>(width), static_cast<int>(height)};
	// Number of bytes between successive rows in the image
	Npp32s img_step = static_cast<int>(width * sizeof(Npp16u));

	// Apply minmax filter
	checkNppError(
	    nppiThreshold_LTValGTVal_16u_C1IR_Ctx(dev_depth_npp, img_step, roi_size, thresh_lt, 0, thresh_gt, 0, npp_context),
	    "filter_depth_minmax_npp");
}

void Kernels::filter_create_mask_npp(uint16_t* dev_depth, uint8_t* dev_mask, const uint width, const uint height,
                                     const int roi[4], NppStreamContext& npp_context) {
	Npp16u* dev_depth_npp = reinterpret_cast<Npp16u*>(dev_depth);
	Npp8u* dev_mask_npp = reinterpret_cast<Npp8u*>(dev_mask);
	int w = static_cast<int>(width);
	int h = static_cast<int>(height);

	// Region of interest
	int roi_offset_x = (roi[0] < 0) ? 0 : roi[0];
	int roi_offset_y = (roi[1] < 0) ? 0 : roi[1];
	int roi_size_x = (roi[2] < 0 || (roi_offset_x + roi[2]) > w) ? w : roi[2];
	int roi_size_y = (roi[3] < 0 || (roi_offset_y + roi[3]) > h) ? h : roi[3];
	NppiSize npp_roi_size = {roi_size_x, roi_size_y};
	int offset = roi_offset_y * w + roi_offset_x;

	// Number of bytes between successive rows in the image
	int img_step = static_cast<int>(width * sizeof(Npp16u));
	// Number of bytes between successive rows in the mask
	int mask_step = static_cast<int>(width * sizeof(Npp8u));

	// Create 8bit mask functioning as binary mask
	checkNppError(nppiCompareC_16u_C1R_Ctx(dev_depth_npp + offset, img_step, Npp16u(0), dev_mask_npp + offset, mask_step,
	                                       npp_roi_size, NPP_CMP_GREATER, npp_context),
	              "filter_create_mask_npp");
}

void Kernels::filter_mask_open_npp(uint8_t* dev_mask, uint8_t* dev_mask_buffer, const uint width, const uint height,
                                   cudaStream_t& stream, NppStreamContext& npp_context) {
	Npp8u* dev_mask_npp = reinterpret_cast<Npp8u*>(dev_mask);
	// Region of interest size
	NppiSize roi_size = {static_cast<int>(width), static_cast<int>(height)};
	// Source offset
	NppiPoint src_offset = {0, 0};
	// Filter mask size
	NppiSize filter_mask_size = {5, 5};
	// Filter mask origin
	// Set anchor point inside the mask; it should round down when odd
	NppiPoint filter_mask_anchor = {filter_mask_size.width / 2, filter_mask_size.height / 2};
	// Number of bytes between successive rows in the mask image
	int mask_step = static_cast<int>(width * sizeof(Npp8u));

	// Create structuring element mask (corners 0 rest 255)
	uint mask_count = static_cast<uint>(filter_mask_size.width * filter_mask_size.height);
	std::vector<Npp8u> filter_mask(mask_count, 255);
	filter_mask[0] = 0;                                                       // top left
	filter_mask[static_cast<uint>(filter_mask_size.width) - 1] = 0;           // top right
	filter_mask[mask_count - static_cast<uint>(filter_mask_size.width)] = 0;  // bottom left
	filter_mask[mask_count - 1] = 0;                                          // bottom right
	// Allocate and upload filter mask to device
	Npp8u* dev_filter_mask = nullptr;
	checkError(cudaMalloc(&dev_filter_mask, mask_count), "cudaMalloc dev_filter_mask");
	checkError(
	    cudaMemcpyAsync(dev_filter_mask, filter_mask.data(), mask_count * sizeof(Npp8u), cudaMemcpyHostToDevice, stream),
	    "cudaMemcpyAsync filter_mask.data() to dev_filter_mask");

	// Create temporary filter buffer
	int buffer_size = 0;
	Npp8u* dev_filter_buffer = nullptr;
	checkNppError(nppiMorphGetBufferSize_8u_C1R(roi_size, &buffer_size), "filter_mask_open_npp");
	checkError(cudaMalloc(&dev_filter_buffer, static_cast<uint>(buffer_size)), "cudaMalloc dev_filter_buffer");

	// Copy mask image to temporary input buffer
	Npp8u* dev_mask_buffer_npp = reinterpret_cast<Npp8u*>(dev_mask_buffer);
	checkError(
	    cudaMemcpyAsync(dev_mask_buffer_npp, dev_mask, width * height * sizeof(Npp8u), cudaMemcpyDeviceToDevice, stream),
	    "cudaMemcpyAsync cam.dev_mask to dev_mask_buffer");

	// Apply morphological open
	checkNppError(
	    nppiMorphOpenBorder_8u_C1R_Ctx(dev_mask_buffer_npp, mask_step, roi_size, src_offset, dev_mask_npp, mask_step,
	                                   roi_size, dev_filter_mask, filter_mask_size, filter_mask_anchor, dev_filter_buffer,
	                                   NPP_BORDER_REPLICATE, npp_context),
	    "filter_mask_open_npp");

	cudaFree(dev_filter_mask);
	cudaFree(dev_filter_buffer);
}

void Kernels::filter_mask_close_npp(uint8_t* dev_mask, uint8_t* dev_mask_buffer, const uint width, const uint height,
                                    cudaStream_t& stream, NppStreamContext& npp_context) {
	Npp8u* dev_mask_npp = reinterpret_cast<Npp8u*>(dev_mask);
	// Region of interest size
	NppiSize roi_size = {static_cast<int>(width), static_cast<int>(height)};
	// Source offset
	NppiPoint src_offset = {0, 0};
	// Filter mask size
	NppiSize filter_mask_size = {5, 5};
	// Filter mask origin
	// Set anchor point inside the mask; it should round down when odd
	NppiPoint filter_mask_anchor = {filter_mask_size.width / 2, filter_mask_size.height / 2};
	// Number of bytes between successive rows in the mask image
	int mask_step = static_cast<int>(width * sizeof(Npp8u));

	// Create structuring element mask (corners 0 rest 255)
	uint filter_mask_count = static_cast<uint>(filter_mask_size.width * filter_mask_size.height);
	std::vector<Npp8u> filter_mask(filter_mask_count, 255);
	filter_mask[0] = 0;                                                              // top left
	filter_mask[static_cast<uint>(filter_mask_size.width) - 1] = 0;                  // top right
	filter_mask[filter_mask_count - static_cast<uint>(filter_mask_size.width)] = 0;  // bottom left
	filter_mask[filter_mask_count - 1] = 0;                                          // bottom right
	// Allocate and upload filter mask to device
	Npp8u* dev_filter_mask = nullptr;
	checkError(cudaMalloc(&dev_filter_mask, filter_mask_count), "cudaMalloc dev_filter_mask");
	checkError(cudaMemcpyAsync(dev_filter_mask, filter_mask.data(), filter_mask_count * sizeof(Npp8u),
	                           cudaMemcpyHostToDevice, stream),
	           "cudaMemcpyAsync cam.dev_depth to dev_depth_buffer");

	// Create temporary filter buffer
	int buffer_size = 0;
	Npp8u* dev_filter_buffer = nullptr;
	checkNppError(nppiMorphGetBufferSize_8u_C1R(roi_size, &buffer_size), "filter_mask_close_npp");
	checkError(cudaMalloc(&dev_filter_buffer, static_cast<uint>(buffer_size)), "cudaMalloc dev_filter_buffer");

	// Copy image to temporary input buffer
	Npp8u* dev_mask_buffer_npp = reinterpret_cast<Npp8u*>(dev_mask_buffer);
	checkError(cudaMemcpyAsync(dev_mask_buffer_npp, dev_mask, static_cast<uint>(width * height) * sizeof(Npp8u),
	                           cudaMemcpyDeviceToDevice, stream),
	           "cudaMemcpyAsync cam.dev_mask to dev_mask_buffer");

	// Apply morphological close
	checkNppError(
	    nppiMorphCloseBorder_8u_C1R_Ctx(dev_mask_buffer_npp, mask_step, roi_size, src_offset, dev_mask_npp, mask_step,
	                                    roi_size, dev_filter_mask, filter_mask_size, filter_mask_anchor,
	                                    dev_filter_buffer, NPP_BORDER_REPLICATE, npp_context),
	    "filter_mask_close_npp");

	cudaFree(dev_filter_mask);
	cudaFree(dev_filter_buffer);
}

void Kernels::filter_mask_count_npp(uint& mask_count, uint8_t* dev_mask, const uint width, const uint height,
                                    cudaStream_t& stream, NppStreamContext& npp_context) {
	Npp8u* dev_mask_npp = reinterpret_cast<Npp8u*>(dev_mask);
	// Region of interest size
	NppiSize roi_size = {static_cast<int>(width), static_cast<int>(height)};
	// Number of bytes between successive rows in the mask
	int mask_step = static_cast<int>(width * sizeof(Npp8u));

	// Mask sum
	Npp64f* dev_mask_sum;
	checkError(cudaMalloc(&dev_mask_sum, sizeof(Npp64f)), "cudaMalloc dev_mask_sum");

	// Create temporary filter buffer
	int buffer_size = 0;
	Npp8u* dev_filter_buffer = nullptr;
	checkNppError(nppiSumGetBufferHostSize_8u_C1R(roi_size, &buffer_size), "filter_mask_count_npp");

	checkError(cudaMalloc(&dev_filter_buffer, static_cast<uint>(buffer_size)), "cudaMalloc dev_filter_buffer");

	// Sum binary mask
	checkNppError(nppiSum_8u_C1R_Ctx(dev_mask_npp, mask_step, roi_size, dev_filter_buffer, dev_mask_sum, npp_context),
	              "filter_mask_count_npp");

	// Copy result to host
	Npp64f* mask_sum = static_cast<Npp64f*>(malloc(sizeof(Npp64f)));

	checkError(cudaMemcpyAsync(mask_sum, dev_mask_sum, sizeof(Npp64f), cudaMemcpyDeviceToHost, stream),
	           "cudaMemcpy dev_mask_sum to mask_sum");
	cudaStreamSynchronize(stream);

	// Calculate valid mask entries
	mask_count = static_cast<uint>(*mask_sum) / NPP_MAX_8U;
	if (mask_count == 0) {
		std::cout << "Warning: empty mask!" << std::endl;
	}

	// Free buffers
	cudaFree(dev_mask_sum);
	cudaFree(dev_filter_buffer);
}

void Kernels::create_img_indices(uint* dev_indices, uint8_t* dev_mask, uint pixel_count, cudaStream_t& stream) {
	int block_size = CUDA_THREADS_PER_BLOCK;
	int compact_count = compactor::compact<uint, uint8_t, int_predicate>(
	    dev_indices, dev_mask, static_cast<int>(pixel_count), block_size, int_predicate(), stream);
	checkError("Kernels create_img_indices compact");
}

void Kernels::deproject_depth_color(float* dev_points, const uint16_t* dev_depth, const uint8_t* dev_color,
                                    const Intrinsics* dev_intrinsics, const uint* dev_indices, const uint indices_count,
                                    const uint pixel_count, const float depth_scale, cudaStream_t& stream) {
	int num_blocks = static_cast<int>(std::ceil(pixel_count / CUDA_THREADS_PER_BLOCK));

	// Deproject depth image to pointcloud
	kernel_deproject_depth<<<num_blocks, CUDA_THREADS_PER_BLOCK, 0, stream>>>(reinterpret_cast<float4*>(dev_points),
	                                                                          dev_depth, dev_color, dev_intrinsics,
	                                                                          dev_indices, indices_count, depth_scale);
}

void Kernels::deproject_depth(float* dev_points, const uint16_t* dev_depth, const Intrinsics* dev_intrinsics,
                              const uint* dev_indices, const uint indices_count, const uint pixel_count,
                              const float depth_scale, cudaStream_t& stream) {
	int num_blocks = static_cast<int>(std::ceil(static_cast<double>(pixel_count) / CUDA_THREADS_PER_BLOCK));

	// Deproject depth image to pointcloud
	kernel_deproject_depth<<<num_blocks, CUDA_THREADS_PER_BLOCK, 0, stream>>>(
	    reinterpret_cast<float4*>(dev_points), dev_depth, dev_intrinsics, dev_indices, indices_count, depth_scale);
}

void Kernels::transform(float* dev_points, const uint point_count, const float* dev_transform, cudaStream_t& stream) {
	int num_blocks = static_cast<int>(std::ceil(static_cast<double>(point_count) / CUDA_THREADS_PER_BLOCK));

	kernel_transform<<<num_blocks, CUDA_THREADS_PER_BLOCK, 0, stream>>>(reinterpret_cast<float4*>(dev_points),
	                                                                    dev_transform, static_cast<int>(point_count));
}

void Kernels::project_pointcloud(uint8_t* dev_color, float* dev_z_buffer, const uint pixel_count,
                                 const float* dev_points, const uint point_count, const Intrinsics* dev_intrinsics,
                                 bool mirror_image, cudaStream_t& stream) {
	int num_blocks = static_cast<int>(std::ceil(static_cast<double>(point_count) / CUDA_THREADS_PER_BLOCK));

	checkError(cudaMemsetAsync(dev_color, 0, pixel_count * sizeof(uint8_t) * 3, stream), "cudaMemsetAsync dev_color");

	// Set zbuffer to max float values
	float float_max = std::numeric_limits<float>::max();
	cuMemsetD32Async(reinterpret_cast<CUdeviceptr>(dev_z_buffer), *reinterpret_cast<unsigned*>(&float_max), pixel_count,
	                 stream);

	kernel_project_pointcloud<<<num_blocks, CUDA_THREADS_PER_BLOCK, 0, stream>>>(
	    dev_color, reinterpret_cast<const float4*>(dev_points), static_cast<int>(point_count), dev_intrinsics,
	    dev_z_buffer, mirror_image);
}

void Kernels::filter_color_median_npp(uint8_t* dev_color, uint8_t* dev_color_buffer, const uint width,
                                      const uint height, cudaStream_t& stream, NppStreamContext& npp_context) {
	Npp8u* dev_color_npp = reinterpret_cast<Npp8u*>(dev_color);
	// Mask size
	NppiSize mask_size = {3, 3};
	// Mask border size
	int mask_border_size = (mask_size.width - 1) / 2;
	// Region of interest size
	NppiSize roi_size = {static_cast<int>(width) - (mask_border_size * 2),
	                     static_cast<int>(height) - (mask_border_size * 2)};
	// Mask origin
	NppiPoint mask_anchor = {mask_border_size, mask_border_size};
	// Number of bytes between successive rows in the image
	Npp32s img_step = static_cast<int>(width * sizeof(Npp8u) * 3);
	// Start of ROI pixel: address of image pointer with offset in bytes applied on temp input buffer
	int roi_offset = mask_border_size * img_step + mask_border_size * static_cast<int>(sizeof(Npp8u)) * 3;

	// Create temporary filter buffer
	Npp32u buffer_size = 0;
	Npp8u* dev_filter_buffer = nullptr;
	checkNppError(nppiFilterMedianGetBufferSize_8u_C3R_Ctx(roi_size, mask_size, &buffer_size, npp_context),
	              "Kernels filter_color_median_npp get buffer size");
	cudaMalloc(&dev_filter_buffer, buffer_size);

	// Copy image to temporary input buffer
	Npp8u* dev_color_buffer_npp = reinterpret_cast<Npp8u*>(dev_color_buffer);
	checkError(cudaMemcpyAsync(dev_color_buffer_npp, dev_color, sizeof(Npp8u) * width * height * 3,
	                           cudaMemcpyDeviceToDevice, stream),
	           "Kernels cudaMemcpy dev_color to dev_color_buffer");

	// Apply filter
	checkNppError(nppiFilterMedian_8u_C3R_Ctx(dev_color_buffer_npp + roi_offset, img_step, dev_color_npp, img_step,
	                                          roi_size, mask_size, mask_anchor, dev_filter_buffer, npp_context),
	              "Kernels filter_color_median_npp");

	cudaFree(dev_filter_buffer);
}

void Kernels::filter_color_gauss_npp(uint8_t* dev_color, uint8_t* dev_color_buffer, const uint width, const uint height,
                                     cudaStream_t& stream, NppStreamContext& npp_context) {
	Npp8u* dev_color_npp = reinterpret_cast<Npp8u*>(dev_color);
	// Mask size
	NppiMaskSize mask_size = NPP_MASK_SIZE_3_X_3;
	// Mask border size
	int mask_border_size = 1;
	// Region of interest size
	// NppiSize roi_size = {static_cast<int>(width - 2), static_cast<int>(height - 2)};
	NppiSize roi_size = {static_cast<int>(width) - (mask_border_size * 2),
	                     static_cast<int>(height) - (mask_border_size * 2)};
	// Number of bytes between successive rows in the image
	Npp32s img_step = static_cast<int>(width * sizeof(Npp8u) * 3);
	// Start of ROI pixel: address of image pointer with offset in bytes applied on temp input buffer
	// int roi_offset = mask_border_size * img_step + mask_border_size * static_cast<int>(sizeof(Npp8u)) * 3;
	int roi_offset = mask_border_size * img_step + mask_border_size * 3;

	// Copy image to temporary input buffer
	Npp8u* dev_color_buffer_npp = reinterpret_cast<Npp8u*>(dev_color_buffer);
	checkError(cudaMemcpyAsync(dev_color_buffer_npp, dev_color, sizeof(Npp8u) * width * height * 3,
	                           cudaMemcpyDeviceToDevice, stream),
	           "Kernels cudaMemcpy dev_color to dev_color_buffer");
return;
	// Apply filter
	// Filter kernels for this function are calculated using a sigma value of 0.4F + (mask width / 2) * 0.6F.
	checkNppError(nppiFilterGauss_8u_C3R_Ctx(dev_color_buffer_npp + img_step + 3, img_step, dev_color_npp + img_step + 3,
	                                         img_step, roi_size, mask_size, npp_context),
	              "Kernels filter_color_gauss_npp");
	// checkNppError(nppiFilterGauss_8u_C3R_Ctx(dev_color_buffer_npp, img_step, dev_color_npp,
	//                                          img_step, roi_size, mask_size, npp_context),
	//               "Kernels filter_color_gauss_npp");
}

void Kernels::filter_depth_median_npp(uint16_t* dev_depth, const uint width, const uint height, cudaStream_t& stream,
                                      NppStreamContext& npp_context) {
	Npp16u* dev_depth_npp = reinterpret_cast<Npp16u*>(dev_depth);
	// Region of interest size
	NppiSize roi_size = {static_cast<int>(width), static_cast<int>(height)};
	// Mask size
	NppiSize mask_size = {3, 3};
	// Mask origin
	// NppiPoint mask_anchor = {0, 0};
	NppiPoint mask_anchor = {mask_size.width / 2, mask_size.height / 2};
	// Number of bytes between successive rows in the image
	Npp32s img_step = static_cast<int>(width * sizeof(Npp16u));

	// Create temporary filter buffer
	Npp32u buffer_size = 0;
	Npp8u* dev_filter_buffer = nullptr;
	checkNppError(nppiFilterMedianGetBufferSize_16u_C1R(roi_size, mask_size, &buffer_size), "filter_depth_median_npp");
	cudaMalloc(&dev_filter_buffer, buffer_size);

	// Copy image to temporary input buffer
	Npp16u* dev_depth_buffer = nullptr;
	cudaMalloc(&dev_depth_buffer, sizeof(Npp16u) * width * height);
	checkError(
	    cudaMemcpyAsync(dev_depth_buffer, dev_depth, sizeof(Npp16u) * width * height, cudaMemcpyDeviceToDevice, stream),
	    "Kernels cudaMemcpy dev_depth to dev_depth_buffer");

	// Apply filter
	checkNppError(nppiFilterMedian_16u_C1R_Ctx(dev_depth_buffer, img_step, dev_depth_npp, img_step, roi_size, mask_size,
	                                           mask_anchor, dev_filter_buffer, npp_context),
	              "Kernels filter_depth_median_npp");

	cudaFree(dev_filter_buffer);
	cudaFree(dev_depth_buffer);
}

void Kernels::filter_depth_gauss_npp(uint16_t* dev_depth, const uint width, const uint height, cudaStream_t& stream,
                                     NppStreamContext& npp_context) {
	Npp16u* dev_depth_npp = reinterpret_cast<Npp16u*>(dev_depth);
	// Region of interest size
	NppiSize roi_size = {static_cast<int>(width), static_cast<int>(height)};
	// Kernel size
	NppiMaskSize mask_size = NPP_MASK_SIZE_5_X_5;
	// Number of bytes between successive rows in the image
	Npp32s img_step = static_cast<int>(width * sizeof(Npp16u));

	// Copy image to temporary input buffer
	Npp16u* dev_depth_buffer = nullptr;
	cudaMalloc(&dev_depth_buffer, sizeof(Npp16u) * width * height);
	checkError(
	    cudaMemcpyAsync(dev_depth_buffer, dev_depth, sizeof(Npp16u) * width * height, cudaMemcpyDeviceToDevice, stream),
	    "Kernels cudaMemcpyAsync dev_depth to dev_depth_buffer");

	// Apply filter
	// Filter kernels for this function are calculated using a sigma value of 0.4F + (mask width / 2) * 0.6F.
	checkNppError(nppiFilterGauss_16u_C1R_Ctx(dev_depth_buffer, img_step, dev_depth_npp, img_step, roi_size, mask_size,
	                                          npp_context),
	              "Kernels filter_depth_gauss_npp");
}

void Kernels::align_depth_to_color(unsigned char* dev_depth_aligned_out, const uint16_t* dev_depth_in,
                                   float depth_scale, const Intrinsics& depth_intrinsics,
                                   const Intrinsics* dev_depth_intrinsics,
                                   const Extrinsics* dev_depth_to_color_extrinsics, const Intrinsics& color_intrinsics,
                                   const Intrinsics* dev_color_intrinsics, const int depth_pixel_count,
                                   const int color_pixel_count, int2* dev_pixel_map, cudaStream_t& stream) {
	int aligned_pixel_count = color_pixel_count;
	size_t aligned_byte_size = static_cast<size_t>(aligned_pixel_count) * 2;

	if (dev_depth_aligned_out == nullptr) cudaMalloc(&dev_depth_aligned_out, aligned_byte_size);

	checkError(cudaMemset(dev_depth_aligned_out, 0xff, aligned_byte_size),
	           "Kernels align_depth_to_color cudaMemset dev_depth_aligned_out");

	if (dev_pixel_map == nullptr)
		cudaMalloc(&dev_pixel_map, static_cast<unsigned int>(depth_pixel_count) * 2 * sizeof(int2));

	// config threads
	dim3 threads(CUDA_THREADS_PER_BLOCK_2D, CUDA_THREADS_PER_BLOCK_2D);
	dim3 depth_blocks(calc_block_size(static_cast<unsigned>(depth_intrinsics.width), threads.x),
	                  calc_block_size(static_cast<unsigned>(depth_intrinsics.height), threads.y));
	dim3 color_blocks(calc_block_size(static_cast<unsigned>(color_intrinsics.width), threads.x),
	                  calc_block_size(static_cast<unsigned>(color_intrinsics.height), threads.y));
	dim3 mapping_blocks(depth_blocks.x, depth_blocks.y, 2);

	kernel_map_depth_to_color<<<mapping_blocks, threads, 0, stream>>>(dev_pixel_map, dev_depth_in, dev_depth_intrinsics,
	                                                                  dev_color_intrinsics, dev_depth_to_color_extrinsics,
	                                                                  depth_scale);

	kernel_align_depth_to_color<<<depth_blocks, threads, 0, stream>>>(
	    (uint16_t*)dev_depth_aligned_out, dev_depth_in, dev_pixel_map, dev_depth_intrinsics, dev_color_intrinsics);

	kernel_replace_to_zero<<<color_blocks, threads, 0, stream>>>((uint16_t*)dev_depth_aligned_out, dev_color_intrinsics);
}

void Kernels::filter_depth_bilateral_gauss_npp(uint16_t* dev_depth, const uint width, const uint height,
                                               cudaStream_t& stream, NppStreamContext& npp_context) {
	Npp16u* dev_depth_npp = reinterpret_cast<Npp16u*>(dev_depth);
	// Region of interest size
	NppiSize roi_size = {static_cast<int>(width), static_cast<int>(height)};
	// Number of bytes between successive rows in the image
	Npp32s img_step = static_cast<int>(width * sizeof(Npp16u));

	// Copy image to temporary input buffer
	Npp16u* dev_depth_buffer = nullptr;
	cudaMalloc(&dev_depth_buffer, sizeof(Npp16u) * width * height);
	checkError(
	    cudaMemcpyAsync(dev_depth_buffer, dev_depth, sizeof(Npp16u) * width * height, cudaMemcpyDeviceToDevice, stream),
	    "Kernels cudaMemcpyAsync dev_depth to dev_depth_buffer");

	NppiSize src_size = {static_cast<int>(width), static_cast<int>(height)};
	NppiPoint src_offset;
	src_offset.x = 0;
	src_offset.y = 0;
	const int radius = 10;                       // 1 indicates a filter kernel size of 3 by 3, 2 indicates 5 by 5, etc
	const int step_src_pixels = 1;               // step between src pixels
	const Npp32f val_square_sigma = 9000000.0f;  // square of the sigma for the relative intensity distance
	const Npp32f pos_square_sigma = 10000.0f;    // square of the sigma for the relative geometric distance
	NppiBorderType border_type = NPP_BORDER_REPLICATE;

	// Apply filter
	checkNppError(nppiFilterBilateralGaussBorder_16u_C1R_Ctx(
	                  dev_depth_buffer, img_step, src_size, src_offset, dev_depth_npp, img_step, roi_size, radius,
	                  step_src_pixels, val_square_sigma, pos_square_sigma, border_type, npp_context),
	              "Kernels filter_depth_bilateral_gauss_npp");
}
