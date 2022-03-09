#pragma once

// SYSTEM
#include <stdint.h>
// EIGEN
#include <eigen3/Eigen/Geometry>
// CUDA
#include <cuda_runtime.h>
// CUDA NPP
#include <npp.h>
// PROJECT
#include "error_check.h"
#include "frameset.h"
#include "kernels.cuh"

/**
 * @brief Pointcloud class.
 */
class Pointcloud {
 public:
	Pointcloud();
	~Pointcloud();
	void setStream(cudaStream_t* stream = nullptr) { this->stream = *stream; }
	void allocate(unsigned point_count);
	void free();
	/**
	 * @brief Get pointcloud gpu device pointer.
	 * @return Pointcloud device pointer
	 */
	float* get() { return dev_points; }
	/**
	 * @brief Get number of points.
	 * @return Number of points
	 */
	unsigned getPointCount() { return point_count; }
	void set(const float* host_points);
	void setTransform(const Eigen::Affine3d& transform);
	void transform();
	void transform(const Eigen::Affine3d& transform);
	void deproject(Frameset& frameset, float depth_scale);
	void append(Pointcloud& cloud);
	void project(Frameset& frameset, bool mirror_image = true);
	void copyToHost(float* host_points);

 private:
	cudaStream_t stream;
	unsigned point_count = 0;
	float* dev_transform = nullptr;
	float* dev_points = nullptr;
	unsigned pointer_offset = 0;
	float* dev_z_buffer = nullptr;
};
