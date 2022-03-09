// PROJECT
#include "color_frame.h"

/**
 * @brief Constructor
 */
ColorFrame::ColorFrame() {}

/**
 * @brief Destructor
 */
ColorFrame::~ColorFrame() { free(); }

void ColorFrame::allocate(const Intrinsics intrinsics, cudaStream_t& stream) {
	this->stream = stream;
	this->intrinsics = intrinsics;

	free();

	pixel_count = static_cast<unsigned>(this->intrinsics.width * this->intrinsics.height);

	checkError(cudaMalloc(&dev_color, static_cast<unsigned>(pixel_count) * 3 * sizeof(uint8_t)),
	           "ColorFrame cudaMalloc dev_ptr");
	checkError(cudaMalloc(&dev_color_buffer, static_cast<unsigned>(pixel_count) * 3 * sizeof(uint8_t)),
	           "ColorFrame cudaMalloc dev_color_buffer");
	checkError(cudaMalloc(&dev_intrinsics, sizeof(Intrinsics)), "ColorFrame cudaMalloc dev_intrinsics");

	checkError(cudaMemcpyAsync(dev_intrinsics, &this->intrinsics, sizeof(Intrinsics), cudaMemcpyHostToDevice, stream),
	           "ColorFrame cudaMemcpy intrinsics to dev_intrinsics");

	checkNppError(nppGetStreamContext(&npp_context), "ColorFrame nppGetStreamContext");
	npp_context.hStream = this->stream;

	this->is_allocated = true;
}

/**
 * @brief Free allocated memory.
 */
void ColorFrame::free() {
	if (is_allocated) {
		if (dev_color != nullptr) cudaFree(dev_color);
		if (dev_color_buffer != nullptr) cudaFree(dev_color_buffer);
		if (dev_intrinsics != nullptr) cudaFree(dev_intrinsics);
	}
}

/**
 * @brief Copy color frame to gpu.
 * @param frame Color frame source
 */
void ColorFrame::copyToDevice(const uint8_t* frame) {
	if (save_images) {
		std::string filename = filepath + "/" + fileprefix + "_color_input.png";
		pcl::io::saveRgbPNGFile(filename, frame, intrinsics.width, intrinsics.height);
	}

	checkError(cudaMemcpyAsync(dev_color, frame, static_cast<uint>(pixel_count) * 3 * sizeof(uint8_t),
	                           cudaMemcpyHostToDevice, stream),
	           "ColorFrame cudaMemcpy frame to dev_color");
}

/**
 * @brief Copy color frame from gpu to host
 * @param host_frame Host frame destination
 */
void ColorFrame::copyToHost(uint8_t* host_frame) {
	checkError(cudaMemcpyAsync(host_frame, dev_color, static_cast<uint>(pixel_count) * sizeof(uint8_t) * 3,
	                           cudaMemcpyDeviceToHost, stream),
	           "ColorFrame cudaMemcpy dev_color to host_frame");
	cudaStreamSynchronize(stream);

	if (save_images) {
		std::string filename = filepath + "/" + fileprefix + "_color_output.png";
		pcl::io::saveRgbPNGFile(filename, host_frame, intrinsics.width, intrinsics.height);
	}
}

/**
 * @brief Set flag for saving images.
 * @param save_images Save images flag
 * @param filepath Base file path
 * @param fileprefix Image file prefix
 */
void ColorFrame::setSaveImages(bool save_images, std::string filepath, std::string fileprefix) {
	this->save_images = save_images;
	if (filepath != "") this->filepath = filepath;
	if (fileprefix != "") this->fileprefix = fileprefix;
}

/**
 * @brief Save color frame to file.
 * @param filename Image filename
 */
void ColorFrame::saveColorImage(const std::string filename) {
	uint image_bytes = static_cast<unsigned>(intrinsics.width * intrinsics.height) * sizeof(uint8_t) * 3;
	uint8_t* host_image = nullptr;
	checkError(cudaMallocHost(&host_image, image_bytes), "ColorFrame cudaMallocHost host_image");

	checkError(cudaMemcpy(host_image, dev_color, image_bytes, cudaMemcpyDeviceToHost),
	           "ColorFrame saveDepthImage cudaMemcpy dev_color to host_image");
	cudaStreamSynchronize(stream);
	pcl::io::saveRgbPNGFile(filename, host_image, intrinsics.width, intrinsics.height);
}
