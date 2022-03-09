#pragma once

#define CUDA_THREADS_PER_BLOCK 1024

// SYSTEM
#include <stdint.h>
#include <vector>
#include <chrono>
// CUDA
#include <cuda_runtime.h>
// PCL
#include <pcl/io/png_io.h>
// PROJECT
#include "error_check.h"
#include "intrinsics.h"
#include "kernels.cuh"

/**
 * @brief Depth frame class
 */
class DepthFrame {
 public:
	DepthFrame();
	~DepthFrame();
	void allocate(const Intrinsics intrinsics, cudaStream_t& stream);
	void allocateUnaligned(const Intrinsics intrinsics_unaligned, const Extrinsics extrinsics, cudaStream_t& stream);
	void free();
	/**
	 * @brief Check for frame allocation.
	 * @return True if frame is allocated
	 */
	bool exists() { return is_allocated; }
	/**
	 * @brief Get frame gpu device pointer.
	 * @return Frame device pointer
	 */
	uint16_t* get() { return dev_depth; }
	/**
	 * @brief Get unaligned frame gpu device pointer.
	 * @return Unaligned frame device pointer
	 */
	uint16_t* getUnaligned() { return dev_depth_unaligned; }
	/**
	 * @brief Get frame mask gpu device pointer.
	 * @return Frame mask device pointer
	 */
	uint8_t* getMask() { return dev_mask; }
	/**
	 * @brief Get frame intrinsics.
	 * @return Frame instrinsics
	 */
	Intrinsics getIntrinsics() { return intrinsics; }
	/**
	 * @brief Get frame intrinsics on gpu device.
	 * @return Frame intrinsics device pointer
	 */
	Intrinsics* getDevIntrinsics() { return dev_intrinsics; }
	/**
	 * @brief Get unaligned frame intrinsics.
	 * @return Unaligned frame instrinsics
	 */
	Intrinsics getIntrinsicsUnaligned() { return intrinsics_unaligned; }
	/**
	 * @brief Get unaligned frame intrinsics on gpu device.
	 * @return Unaligned frame intrinsics device pointer
	 */
	Intrinsics* getDevIntrinsicsUnaligned() { return dev_intrinsics_unaligned; }
	/**
	 * @brief Get depth to color frame camera extrinsics.
	 * @return Depth to color frame camera extrinsics
	 */
	Extrinsics getExtrinsics() { return extrinsics; }
	/**
	 * @brief Get depth to color frame camera extrinsics on gpu device.
	 * @return Depth to color frame camera extrinsics device pointer
	 */
	Extrinsics* getDevExtrinsics() { return dev_extrinsics; }
	/**
	 * @brief Get device indices of valid pixels after filtering.
	 * @return Device pixel indices
	 */
	uint* getIndices() { return dev_indices; }
	/**
	 * @brief Get number of pixels.
	 * @return Number of pixels
	 */
	unsigned getPixelCount() { return pixel_count; }
	/**
	 * @brief Get number of pixels of unaligned frame.
	 * @return Number of pixels of unaligned frame
	 */
	unsigned getPixelCountUnaligned() { return pixel_count_unaligned; }
	/**
	 * @brief Get number of valid pixels after filtering.
	 * @return Number of valid pixels
	 */
	unsigned getMaskCount() { return mask_count; }
	/**
	 * @brief Get pixel map for alignment.
	 * @return Pixel map device pointer
	 */
	int2* getDevPixelMap() { return dev_pixel_map; }
	void copyToDevice(const uint16_t* frame);
	void copyToHost(uint16_t* host_frame);
	void copyUnalignedToDevice(const uint16_t* frame_unaligned);
	void filter(const float min_depth, const float max_depth, const float depth_scale, const int roi[4]);
	void setSaveImages(bool save_images, std::string filepath = "", std::string fileprefix = "");

 private:
	cudaStream_t stream;
	NppStreamContext npp_context;
	bool is_allocated = false;
	bool save_images = false;
	std::string filepath;
	std::string fileprefix;

	unsigned pixel_count = 0;
	unsigned pixel_count_unaligned = 0;
	// Intrinsics
	Intrinsics intrinsics;
	Intrinsics intrinsics_unaligned;
	Intrinsics* dev_intrinsics = nullptr;
	Intrinsics* dev_intrinsics_unaligned = nullptr;
	// Extrinsics
	Extrinsics extrinsics;
	Extrinsics* dev_extrinsics = nullptr;
	// Device pointer
	uint16_t* dev_depth = nullptr;
	uint16_t* dev_depth_unaligned = nullptr;
	// Pixel map for alignment
	int2* dev_pixel_map = nullptr;
	// Filter mask
	unsigned mask_count = 0;
	uint8_t* dev_mask = nullptr;
	uint8_t* dev_mask_buffer = nullptr;
	uint* dev_indices = nullptr;

	void saveDepthImage(const std::string filename);
	void saveMaskImage(const std::string filename);
};
