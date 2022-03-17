// PROJECT
#include "frameset.h"
#include "kernels.cuh"

/**
 * @brief Constructor.
 */
Frameset::Frameset()
{
	cudaEventCreate(&start);
	cudaEventCreate(&stop);
}

/**
 * @brief Destructor.
 */
Frameset::~Frameset()
{
	if (start != nullptr) cudaEventDestroy(start);
	if (stop != nullptr) cudaEventDestroy(stop);
}

/**
 * @brief Set cuda stream.
 * @param stream Cuda stream
 */
void Frameset::setStream(cudaStream_t* stream)
{
	this->stream = *stream;
	checkNppError(nppGetStreamContext(&npp_context), "Frameset nppGetStreamContext");
	npp_context.hStream = this->stream;
}

/**
 * @brief Allocate gpu memory for depth frame.
 * @param intrinsics Depth frame intrinsics
 */
void Frameset::allocateDepthFrame(const Intrinsics& intrinsics)
{
	depth_frame.allocate(intrinsics, stream);
}

void Frameset::allocateDepthFrameUnaligned(const Intrinsics& intrinsics_unaligned, const Extrinsics& extrinsics)
{
	depth_frame.allocateUnaligned(intrinsics_unaligned, extrinsics, stream);
}

/**
 * @brief Allocate gpu memory for color frame.
 * @param intrinsics Color frame intrinsics
 */
void Frameset::allocateColorFrame(const Intrinsics& intrinsics)
{
	color_frame.allocate(intrinsics, stream);
}

/**
 * @brief Copy depth frame from host to gpu.
 * @param depth_frame Depth frame host source
 */
void Frameset::setDepthFrame(const uint16_t* depth_frame)
{
	this->depth_frame.copyToDevice(depth_frame);
}

/**
 * @brief Copy unaligned depth frame from host to gpu.
 * @param depth_frame_unaligned Unaligned depth frame host source
 */
void Frameset::setDepthFrameUnaligned(const uint16_t* depth_frame_unaligned)
{
	this->depth_frame.copyUnalignedToDevice(depth_frame_unaligned);
}

/**
 * @brief Copy color frame from host to gpu.
 * @param color_frame Color frame host source
 */
void Frameset::setColorFrame(const uint8_t* color_frame)
{
	this->color_frame.copyToDevice(color_frame);
}

/**
 * @brief Get depth frame gpu device pointer.
 * @return Depth frame device pointer
 */
uint16_t* Frameset::getDepth()
{
	return depth_frame.get();
}

/**
 * @brief Get unaligned depth frame gpu device pointer.
 * @return Unaligned depth frame device pointer
 */
uint16_t* Frameset::getDepthUnaligned()
{
	return depth_frame.getUnaligned();
}

/**
 * @brief Get color frame gpu device pointer.
 * @return Color frame device pointer
 */
uint8_t* Frameset::getColor()
{
	return color_frame.get();
}

/**
 * @brief Get frame mask gpu device pointer.
 * @return Frame mask device pointer
 */
uint8_t* Frameset::getMask()
{
	return depth_frame.getMask();
}

/**
 * @brief Filter depth frame.
 * @param min_depth Minimun depth value in meters
 * @param max_depth Maximum depth value in meters
 * @param depth_scale Factor to convert camera depth unit to meters
 * @param roi Region of interest in pixels: [left, top, width, height]
 */
void Frameset::filterDepth(const float min_depth, const float max_depth, const float depth_scale,
						   const std::array<int, 4> roi)
{
	timerStart();
	depth_frame.filter(min_depth, max_depth, depth_scale, roi.data());
	timerEnd("filter");
}

/**
 * @brief Deproject frameset to pointcloud.
 * @param dev_points Pointcloud gpu device pointer
 * @param depth_scale Factor to convert camera depth unit to meters
 */
void Frameset::deprojectToPointcloud(float* dev_points, const float depth_scale)
{
	timerStart();
	if (color_frame.exists())
	{
		Kernels::deproject_depth_color(dev_points, depth_frame.get(), color_frame.get(), depth_frame.getDevIntrinsics(),
									   depth_frame.getIndices(), static_cast<unsigned>(depth_frame.getMaskCount()),
									   depth_frame.getPixelCount(), depth_scale, stream);
	}
	else
	{
		Kernels::deproject_depth(dev_points, depth_frame.get(), depth_frame.getDevIntrinsics(), depth_frame.getIndices(),
								 static_cast<unsigned>(depth_frame.getMaskCount()), depth_frame.getPixelCount(),
								 depth_scale, stream);
	}
	timerEnd("deproject");
}

/**
 * @brief Copy color frame from gpu to host.
 * @param host_color_frame Color frame host destination
 */
void Frameset::copyColorToHost(uint8_t* host_color_frame)
{
	color_frame.copyToHost(host_color_frame);
}

/**
 * @brief Copy depth frame from gpu to host.
 * @param host_depth_frame Depth frame host destination
 */
void Frameset::copyDepthToHost(uint16_t* host_depth_frame)
{
	depth_frame.copyToHost(host_depth_frame);
}

/**
 * @brief Align unaligned depth frame to color frame with previously set intrinsics and extrinsics.
 * @param depth_scale Factor to convert camera depth unit to meters
 */
void Frameset::alignDepthToColor(const float depth_scale)
{
	Kernels::align_depth_to_color(reinterpret_cast<unsigned char*>(depth_frame.get()), depth_frame.getUnaligned(),
								  depth_scale, depth_frame.getIntrinsicsUnaligned(),
								  depth_frame.getDevIntrinsicsUnaligned(), depth_frame.getDevExtrinsics(),
								  color_frame.getIntrinsics(), color_frame.getDevIntrinsics(),
								  static_cast<int>(depth_frame.getPixelCountUnaligned()),
								  static_cast<int>(color_frame.getPixelCount()), depth_frame.getDevPixelMap(), stream);
}

/**
 * @brief Filter color frame.
 * @param median If true use median filter, otherwise use gauss filter
 */
void Frameset::filterColor(bool median)
{
	if (median)
	{
		Kernels::filter_color_median_npp(
			getColor(), color_frame.getBuffer(), static_cast<unsigned>(getColorIntrinsics().width),
			static_cast<unsigned>(getColorIntrinsics().height), *getStream(), *getNppContext());
	}
	else
	{
		Kernels::filter_color_gauss_npp(getColor(), color_frame.getBuffer(),
										static_cast<unsigned>(getColorIntrinsics().width),
										static_cast<unsigned>(getColorIntrinsics().height), *getStream(), *getNppContext());
	}
}

/**
 * @brief Start gpu timer.
 */
void Frameset::timerStart()
{
	if (profiling) cudaEventRecord(start);
}

/**
 * @brief End gpu timer and display duration and message.
 * @param msg Message to display
 */
void Frameset::timerEnd(std::string msg)
{
	if (profiling)
	{
		cudaEventRecord(stop);
		cudaEventSynchronize(stop);
		float duration = 0;
		cudaEventElapsedTime(&duration, start, stop);
		int max_msg_length = 32;
		msg                = "| " + msg + ":";
		int num_spaces     = (max_msg_length - static_cast<int>(msg.length()));
		if (num_spaces < 1) num_spaces = 1;
		msg.append(std::string(static_cast<uint>(num_spaces), ' '));
		std::cout << msg << std::setprecision(4) << duration << " ms" << std::endl;
	}
}
