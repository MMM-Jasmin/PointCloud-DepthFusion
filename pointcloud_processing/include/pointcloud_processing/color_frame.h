#pragma once

// SYSTEM
#include <stdint.h>
// CUDA
#include <cuda_runtime.h>
// CUDA NPP
#include <npp.h>
// PCL
#include <pcl/io/png_io.h>
// PROJECT
#include "error_check.h"
#include "intrinsics.h"

/**
 * @brief Rgb color frame class.
 */
class ColorFrame
{
public:
	ColorFrame();
	~ColorFrame();
	void allocate(const Intrinsics intrinsics, cudaStream_t& stream);
	void free();
	/**
	 * @brief Check for frame allocation.
	 * @return True if frame is allocated
	 */
	bool exists()
	{
		return is_allocated;
	}
	/**
	 * @brief Get frame gpu device pointer.
	 * @return Frame device pointer
	 */
	uint8_t* get()
	{
		return dev_color;
	}
	/**
	 * @brief Get frame buffer gpu device pointer.
	 * @return Frame buffer device pointer
	 */
	uint8_t* getBuffer()
	{
		return dev_color_buffer;
	}
	/**
	 * @brief Get frame intrinsics.
	 * @return Frame instrinsics
	 */
	Intrinsics getIntrinsics()
	{
		return intrinsics;
	}
	/**
	 * @brief Get frame intrinsics from gpu device.
	 * @return Frame intrinsics device pointer
	 */
	Intrinsics* getDevIntrinsics()
	{
		return dev_intrinsics;
	}
	/**
	 * @brief Get number of pixels.
	 * @return Number of pixels
	 */
	unsigned getPixelCount()
	{
		return pixel_count;
	}
	void copyToDevice(const uint8_t* frame);
	void copyToHost(uint8_t* host_frame);
	void setSaveImages(bool save_images, std::string filepath = "", std::string fileprefix = "");

private:
	cudaStream_t stream;
	NppStreamContext npp_context;
	bool is_allocated = false;
	bool save_images  = false;
	std::string filepath;
	std::string fileprefix;

	unsigned pixel_count = 0;
	// Intrinsics
	Intrinsics intrinsics;
	Intrinsics* dev_intrinsics = nullptr;
	// Device pointer
	uint8_t* dev_color = nullptr;
	// Filter buffer
	uint8_t* dev_color_buffer = nullptr;

	void saveColorImage(const std::string filename);
};
