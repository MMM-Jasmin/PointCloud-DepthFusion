// PROJECT
#include "pointcloud.h"

/**
 * @brief Constructor.
 */
Pointcloud::Pointcloud() {}

/**
 * @brief Destructor.
 */
Pointcloud::~Pointcloud()
{
	free();
}

/**
 * @brief Allocate gpu memory for poincloud.
 * @param point_count Number of points
 */
void Pointcloud::allocate(unsigned point_count)
{
	this->point_count = point_count;
	if (dev_points != nullptr)
	{
		cudaFree(dev_points);
		dev_points = nullptr;
	}
	checkError(cudaMalloc(reinterpret_cast<void**>(&dev_points), static_cast<uint>(point_count) * sizeof(float4)),
			   "Pointcloud cudaMalloc dev_points");
	if (dev_transform == nullptr)
	{
		checkError(cudaMalloc(&dev_transform, 16 * sizeof(float)), "Pointcloud cudaMalloc cam.dev_transform");
	}
}

/**
 * @brief Free allocated gpu memory.
 */
void Pointcloud::free()
{
	if (dev_points != nullptr) cudaFree(dev_points);
	if (dev_transform != nullptr) cudaFree(dev_transform);
	if (dev_z_buffer != nullptr) cudaFree(dev_z_buffer);
}

/**
 * @brief Set pointcloud by copying from host to gpu memory.
 * @param host_points Host pointcloud source
 */
void Pointcloud::set(const float* host_points)
{
	uint cloud_bytes = static_cast<uint>(point_count) * sizeof(float4);

	checkError(cudaMemcpyAsync(dev_points, host_points, cloud_bytes, cudaMemcpyHostToDevice, stream),
			   "Pointcloud cudaMemcpy host_points to dev_points");
}

/**
 * @brief Set pointcloud transformation.
 * @param transform Affine transformation
 */
void Pointcloud::setTransform(const Eigen::Affine3d& transform)
{
	Eigen::Matrix<float, 4, 4, Eigen::RowMajor> transform_matrix = transform.cast<float>().matrix();
	;
	const float* transform_flat = transform_matrix.data();

	if (dev_transform == nullptr)
	{
		checkError(cudaMalloc(&dev_transform, 16 * sizeof(float)), "Pointcloud cudaMalloc dev_transform");
	}
	checkError(cudaMemcpyAsync(dev_transform, transform_flat, 16 * sizeof(float), cudaMemcpyHostToDevice, stream),
			   "Pointcloud cudaMemcpy transform_flat to dev_transform");
}

/**
 * @brief Transform pointcloud with set transformation.
 */
void Pointcloud::transform()
{
	Kernels::transform(dev_points, point_count, dev_transform, stream);
}

/**
 * @brief Transform pointcloud with given transformation.
 * @param transform Affine transformation
 */
void Pointcloud::transform(const Eigen::Affine3d& transform)
{
	setTransform(transform);
	this->transform();
}

/**
 * @brief Deproject frameset to pointcloud.
 * @param frameset Frameset source
 * @param depth_scale Factor to convert camera depth unit to meters
 */
void Pointcloud::deproject(Frameset& frameset, float depth_scale)
{
	bool not_allocated = (point_count == 0);
	if (not_allocated)
	{
		point_count = frameset.getMaskCount();
		allocate(point_count);
	}
	pointer_offset += frameset.getMaskCount() * static_cast<int>(sizeof(float));
	frameset.sync();
	frameset.deprojectToPointcloud(dev_points, depth_scale);
}

/**
 * @brief Append to pointcloud.
 * @param cloud Pointcloud to append
 */
void Pointcloud::append(Pointcloud& cloud)
{
	if (pointer_offset >= point_count * static_cast<int>(sizeof(float)))
	{
		std::cout << "error: appending more points to pointcloud than allocated" << std::endl;
		return;
	}
	checkError(
		cudaMemcpyAsync(dev_points + pointer_offset, cloud.get(),
						static_cast<uint>(cloud.getPointCount()) * sizeof(float4), cudaMemcpyDeviceToDevice, stream),
		"Pointcloud cudaMemcpy cloud.get() to dev_points + pointer_offset");
	pointer_offset += cloud.getPointCount() * static_cast<int>(sizeof(float));
}

/**
 * @brief Project pointcloud to color frame of frameset.
 * @param frameset Projection destination frameset
 * @param mirror_image True if image should be mirrored horizontally
 */
void Pointcloud::project(Frameset& frameset, bool mirror_image)
{
	unsigned pixel_count =
		static_cast<unsigned>(frameset.getColorIntrinsics().width * frameset.getColorIntrinsics().height);
	if (dev_z_buffer == nullptr)
	{
		checkError(cudaMalloc(&dev_z_buffer, static_cast<uint>(pixel_count) * sizeof(float)),
				   "Pointcloud cudaMalloc dev_z_buffer");
	}
	Kernels::project_pointcloud(frameset.getColor(), dev_z_buffer, pixel_count, dev_points, point_count,
								frameset.getColorDevIntrinsics(), mirror_image, *frameset.getStream());

	/*
	// Filter color image (now performed by frameset)
	Kernels::filter_color_median_npp(frameset.getColor(), static_cast<unsigned>(frameset.getColorIntrinsics().width),
																	 static_cast<unsigned>(frameset.getColorIntrinsics().height), *frameset.getStream(),
																	 *frameset.getNppContext());
	Kernels::filter_color_gauss_npp(frameset.getColor(), static_cast<unsigned>(frameset.getColorIntrinsics().width),
																	static_cast<unsigned>(frameset.getColorIntrinsics().height), *frameset.getStream(),
																	*frameset.getNppContext());
	*/
}

/**
 * @brief Copy pointcloud from gpu to host.
 * @param host_points Host pointcloud destination
 */
void Pointcloud::copyToHost(float* host_points)
{
	unsigned cloud_bytes = static_cast<unsigned>(point_count) * sizeof(float4);
	checkError(cudaMemcpyAsync(host_points, dev_points, cloud_bytes, cudaMemcpyDeviceToHost, stream),
			   "cudaMemcpy cam.dev_points to cloud_ptr");
	cudaStreamSynchronize(stream);
}
