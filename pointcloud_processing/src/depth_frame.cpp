// PROJECT
#include "kernels.cuh"
#include "depth_frame.h"

/**
 * @brief Constuctor
 */
DepthFrame::DepthFrame() {}

/**
 * @brief Destructor
 */
DepthFrame::~DepthFrame() { free(); }

/**
 * @brief Allocate memory for frame, mask and intrinsics.
 * @param intrinsics Depth frame intrinsics
 * @param stream Cuda stream
 */
void DepthFrame::allocate(const Intrinsics intrinsics, cudaStream_t& stream) {
	this->stream = stream;
	this->intrinsics = intrinsics;

	free();

	pixel_count = static_cast<unsigned>(this->intrinsics.width * this->intrinsics.height);

	checkError(cudaMalloc(&dev_depth, static_cast<unsigned>(pixel_count) * sizeof(uint16_t)),
	           "DepthFrame cudaMalloc dev_depth");
	checkError(cudaMalloc(&dev_intrinsics, sizeof(Intrinsics)), "DepthFrame cudaMalloc dev_intrinsics");

	checkError(cudaMemcpyAsync(dev_intrinsics, &this->intrinsics, sizeof(Intrinsics), cudaMemcpyHostToDevice, stream),
	           "DepthFrame cudaMemcpy intrinsics to dev_intrinsics");

	checkError(cudaMalloc(&dev_indices, static_cast<uint>(pixel_count) * sizeof(uint)),
	           "DepthFrame cudaMalloc dev_indices");

	checkError(cudaMalloc(&dev_mask, static_cast<uint>(pixel_count) * sizeof(uint8_t)), "DepthFrame cudaMalloc dev_mask");
	checkError(cudaMalloc(&dev_mask_buffer, static_cast<uint>(pixel_count) * sizeof(uint8_t)),
	           "DepthFrame cudaMalloc dev_mask_buffer");

	checkError(cudaMemset(dev_mask, 0, static_cast<uint>(pixel_count) * sizeof(bool)),
	           "DepthFrame cudaMemset cam.dev_mask");

	checkNppError(nppGetStreamContext(&npp_context), "DepthFrame nppGetStreamContext");
	npp_context.hStream = this->stream;

	this->is_allocated = true;
}

/**
 * @brief Allocate unaligned depth frame with intrinsics and depth to color extrinsics.
 * @param intrinsics_unaligned Intrinsics of the unaligned depth frame
 * @param extrinsics Depth to color sensor camera extrinsics
 * @param stream Cuda stream
 */
void DepthFrame::allocateUnaligned(const Intrinsics intrinsics_unaligned, const Extrinsics extrinsics,
                                   cudaStream_t& stream) {
	this->stream = stream;
	this->intrinsics_unaligned = intrinsics_unaligned;
	this->extrinsics = extrinsics;

	if (dev_depth_unaligned != nullptr) cudaFree(dev_depth_unaligned);
	if (dev_intrinsics_unaligned != nullptr) cudaFree(dev_intrinsics_unaligned);
	if (dev_extrinsics != nullptr) cudaFree(dev_extrinsics);
	if (dev_pixel_map != nullptr) cudaFree(dev_pixel_map);

	pixel_count_unaligned = static_cast<unsigned>(this->intrinsics_unaligned.width * this->intrinsics_unaligned.height);

	checkError(cudaMalloc(&dev_depth_unaligned, static_cast<unsigned>(pixel_count_unaligned) * sizeof(uint16_t)),
	           "DepthFrame cudaMalloc dev_depth_unaligned");

	checkError(cudaMalloc(&dev_intrinsics_unaligned, sizeof(Intrinsics)),
	           "DepthFrame cudaMalloc dev_intrinsics_unaligned");
	checkError(cudaMemcpyAsync(dev_intrinsics_unaligned, &this->intrinsics_unaligned, sizeof(Intrinsics),
	                           cudaMemcpyHostToDevice, stream),
	           "DepthFrame cudaMemcpy intrinsics_unaligned to dev_intrinsics_unaligned");

	checkError(cudaMalloc(&dev_extrinsics, sizeof(Extrinsics)), "DepthFrame cudaMalloc dev_extrinsics");
	checkError(cudaMemcpyAsync(dev_extrinsics, &this->extrinsics, sizeof(Extrinsics), cudaMemcpyHostToDevice, stream),
	           "DepthFrame cudaMemcpy extrinsics to dev_extrinsics");

	checkError(cudaMalloc(&dev_pixel_map, static_cast<unsigned int>(pixel_count_unaligned) * 2 * sizeof(int2)),
	           "DepthFrame cudaMalloc dev_pixel_map");
}

/**
 * @brief Free allocated memory.
 */
void DepthFrame::free() {
	if (is_allocated) {
		if (dev_depth != nullptr) cudaFree(dev_depth);
		if (dev_intrinsics != nullptr) cudaFree(dev_intrinsics);
	}
}

/**
 * @brief Copy depth frame from host to gpu.
 * @param frame Depth frame source
 */
void DepthFrame::copyToDevice(const uint16_t* frame) {
	if (save_images) {
		std::string filename = filepath + "/" + fileprefix + "_depth_host_input.png";
		pcl::io::saveShortPNGFile(filename, frame, intrinsics.width, intrinsics.height, 1);
		// std::cout << "saved depth image to: " << filename << std::endl;
	}

	checkError(cudaMemcpyAsync(dev_depth, frame, static_cast<uint>(pixel_count) * sizeof(uint16_t),
	                           cudaMemcpyHostToDevice, stream),
	           "DepthFrame cudaMemcpy frame to dev_depth");
}

/**
 * @brief Copy depth frame from gpu to host
 * @param host_frame Host frame destination
 */
void DepthFrame::copyToHost(uint16_t* host_frame) {
	checkError(cudaMemcpyAsync(host_frame, dev_depth, static_cast<uint>(pixel_count) * sizeof(uint16_t),
	                           cudaMemcpyDeviceToHost, stream),
	           "DepthFrame cudaMemcpy dev_depth to host_frame");
	cudaStreamSynchronize(stream);
}

/**
 * @brief Copy unaligned depth frame from host to gpu.
 * @param frame_unaligned Unaligned depth frame source
 */
void DepthFrame::copyUnalignedToDevice(const uint16_t* frame_unaligned) {
	checkError(
	    cudaMemcpyAsync(dev_depth_unaligned, frame_unaligned, static_cast<uint>(pixel_count_unaligned) * sizeof(uint16_t),
	                    cudaMemcpyHostToDevice, stream),
	    "DepthFrame cudaMemcpy frame_unaligned to dev_depth_unaligned");
}

/**
 * @brief Filter depth frame.
 * @param min_depth Minimum depth value in meters
 * @param max_depth Maximum depth value in meters
 * @param depth_scale Factor to convert camera depth unit to meters
 * @param roi Region of interest in pixels: [left, top, width, height]
 */
void DepthFrame::filter(const float min_depth, const float max_depth, const float depth_scale, const int roi[4]) {
	const unsigned width = static_cast<unsigned>(intrinsics.width);
	const unsigned height = static_cast<unsigned>(intrinsics.height);
	if (save_images) saveDepthImage(filepath + "/" + fileprefix + "_depth_input.png");

	/*
	// ## test bilateral filter
	saveDepthImage(filepath + "/" + fileprefix + "_depth_input.png");
	cudaDeviceSynchronize();
	auto timer = std::chrono::high_resolution_clock::now();
	Kernels::filter_depth_bilateral_gauss_npp(dev_depth, width, height, stream, npp_context);
	cudaDeviceSynchronize();
	double duration = (std::chrono::high_resolution_clock::now() - timer).count() / 1e6;
	std::cout << "## bilateral filter time: " << duration << std::endl;
	saveDepthImage(filepath + "/" + fileprefix + "_depth_bilateral.png");
	*/

	Kernels::filter_depth_minmax_npp(dev_depth, width, height, min_depth, max_depth, depth_scale, npp_context);
	if (save_images) saveDepthImage(filepath + "/" + fileprefix + "_depth_minmax.png");
	Kernels::filter_create_mask_npp(dev_depth, dev_mask, width, height, roi, npp_context);
	if (save_images) saveMaskImage(filepath + "/" + fileprefix + "_mask_1_input.png");
	// Kernels::filter_mask_open_npp(dev_mask, dev_mask_buffer, width, height, stream, npp_context);
	// if (save_images) saveMaskImage(filepath + "/" + fileprefix + "_mask_2_opened.png");
	// Kernels::filter_mask_close_npp(dev_mask, dev_mask_buffer, width, height, stream, npp_context);
	// if (save_images) saveMaskImage(filepath + "/" + fileprefix + "_mask_3_closed.png");
	Kernels::filter_mask_count_npp(mask_count, dev_mask, width, height, stream, npp_context);
	Kernels::create_img_indices(dev_indices, dev_mask, pixel_count, stream);
	if (save_images) saveMaskImage(filepath + "/" + fileprefix + "_mask_4_final.png");
}

/**
 * @brief Set flag for saving images.
 * @param save_images Save images flag
 * @param filepath Base file path
 * @param fileprefix Image file prefix
 */
void DepthFrame::setSaveImages(bool save_images, std::string filepath, std::string fileprefix) {
	this->save_images = save_images;
	if (filepath != "") this->filepath = filepath;
	this->fileprefix = fileprefix;
}

/**
 * @brief Save depth frame to file.
 * @param filename Image filename
 */
void DepthFrame::saveDepthImage(const std::string filename) {
	uint image_bytes = static_cast<unsigned>(intrinsics.width * intrinsics.height) * sizeof(uint16_t);
	uint16_t* host_image = nullptr;
	checkError(cudaMallocHost(&host_image, image_bytes), "cudaMallocHost host_image");

	checkError(cudaMemcpy(host_image, dev_depth, image_bytes, cudaMemcpyDeviceToHost),
	           "DepthFrame saveDepthImage cudaMemcpy dev_depth to host_image");
	cudaStreamSynchronize(stream);
	pcl::io::saveShortPNGFile(filename, host_image, intrinsics.width, intrinsics.height, 1);
}

/**
 * @brief Save frame mask to file.
 * @param filename Image filename
 */
void DepthFrame::saveMaskImage(const std::string filename) {
	cudaStreamSynchronize(stream);
	uint image_bytes = static_cast<unsigned>(intrinsics.width * intrinsics.height) * sizeof(uint8_t);
	uint8_t* host_image = nullptr;
	checkError(cudaMallocHost(&host_image, image_bytes), "cudaMallocHost host_image");

	checkError(cudaMemcpy(host_image, dev_mask, image_bytes, cudaMemcpyDeviceToHost),
	           "DepthFrame saveMaskImage cudaMemcpy dev_mask to host_image");
	cudaStreamSynchronize(stream);
	pcl::io::saveCharPNGFile(filename, host_image, intrinsics.width, intrinsics.height, 1);
}
