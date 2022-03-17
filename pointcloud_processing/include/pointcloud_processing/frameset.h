#pragma once

// SYSTEM
#include <iomanip>
#include <iostream>
#include <stdint.h>
// CUDA
#include <cuda_runtime.h>
// CUDA NPP
#include <npp.h>
// PROJECT
#include "color_frame.h"
#include "depth_frame.h"
#include "error_check.h"
#include "intrinsics.h"

/**
 * @brief Frameset class containing depth and color frames.
 */
class Frameset
{
public:
	Frameset();
	~Frameset();
	/**
	 * @brief Set profiling flag.
	 * @param profiling True to activate profiling
	 */
	void setProfiling(bool profiling)
	{
		this->profiling = profiling;
	}
	/**
	 * @brief Get cuda stream.
	 * @return Cuda stream
	 */
	cudaStream_t* getStream()
	{
		return &stream;
	}
	/**
	 * @brief Set cuda stream.
	 * @param stream
	 */
	void setStream(cudaStream_t* stream = nullptr);
	/**
	 * @brief Get cuda npp context.
	 * @return Cuda npp context
	 */
	NppStreamContext* getNppContext()
	{
		return &npp_context;
	}
	/**
	 * @brief Synchronize cuda stream with host
	 */
	void sync()
	{
		cudaStreamSynchronize(stream);
	}
	void allocateDepthFrame(const Intrinsics& intrinsics);
	void allocateDepthFrameUnaligned(const Intrinsics& intrinsics_unaligned, const Extrinsics& extrinsics);
	void allocateColorFrame(const Intrinsics& intrinsics);
	void setDepthFrame(const uint16_t* depth_frame);
	void setDepthFrameUnaligned(const uint16_t* depth_frame_unaligned);
	void setColorFrame(const uint8_t* color_frame);
	uint16_t* getDepth();
	uint16_t* getDepthUnaligned();
	uint8_t* getColor();
	uint8_t* getMask();
	/**
	 * @brief Get color frame intrinsics.
	 * @return Color frame intrinsics
	 */
	Intrinsics getColorIntrinsics()
	{
		return color_frame.getIntrinsics();
	}
	/**
	 * @brief Get depth frame intrinsics.
	 * @return Depth frame intrinsics
	 */
	Intrinsics getDepthIntrinsics()
	{
		return depth_frame.getIntrinsics();
	}
	/**
	 * @brief Get color frame intrinsics from gpu device.
	 * @return Color frame intrinsics
	 */
	Intrinsics* getColorDevIntrinsics()
	{
		return color_frame.getDevIntrinsics();
	}
	/**
	 * @brief Get depth frame intrinsics from gpu device.
	 * @return Depth frame intrinsics
	 */
	Intrinsics* getDepthDevIntrinsics()
	{
		return depth_frame.getDevIntrinsics();
	}
	/**
	 * @brief Get number of valid pixels.
	 * @return Number of valid pixels
	 */
	unsigned getMaskCount()
	{
		return depth_frame.getMaskCount();
	}
	/**
	 * @brief Set flag for saving images to disk.
	 * @param save_images True to activate image saving
	 * @param filepath Base path for image files
	 * @param fileprefix Prefix for image filenames
	 */
	void setSaveImages(bool save_images, std::string filepath, std::string fileprefix = "")
	{
		color_frame.setSaveImages(save_images, filepath, fileprefix);
		depth_frame.setSaveImages(save_images, filepath, fileprefix);
	}

	void filterDepth(const float min_depth, const float max_depth, const float depth_scale, const std::array<int, 4> = { -1, -1, -1, -1 });
	void deprojectToPointcloud(float* dev_points, const float depth_scale);
	void copyColorToHost(uint8_t* host_color_frame);
	void copyDepthToHost(uint16_t* host_depth_frame);
	void alignDepthToColor(const float depth_scale);
	void filterColor(bool median = false);

private:
	bool sync_check = false;
	bool profiling  = false;
	// Cuda stream
	cudaStream_t stream;
	// Cuda profiling events
	cudaEvent_t start, stop;
	// NPP stream context
	NppStreamContext npp_context;
	// Frames
	DepthFrame depth_frame;
	ColorFrame color_frame;

	void timerStart();
	void timerEnd(std::string msg);
};
