#pragma once

// PROJECT
#include <fast_gicp/cuda/fast_vgicp_cuda.cuh>
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#include <fast_gicp/gicp/impl/fast_vgicp_cuda_impl.hpp>
// PCL
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief Class for pointcloud registration with vgicp library.
 */
class Registration
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Registration(bool verbose = false);
	~Registration();
	/**
	 * @brief Set verbosity flag.
	 * @param verbose True to activate verbose messages
	 */
	void setVerbosity(bool verbose)
	{
		this->verbose = verbose;
	}
	/**
	 * @brief Set maximum icp iterations.
	 * @param max_iterations Maximum icp iterations
	 */
	void setMaximumIterations(int max_iterations)
	{
		this->max_iterations = max_iterations;
		vgicp_cuda.setMaximumIterations(this->max_iterations);
	}
	/**
	 * @brief Set algorithm termination epsilon for translation.
	 * @param translation_epsilon Translation epsilon
	 */
	void setTranslationEpsilon(double translation_epsilon)
	{
		this->translation_epsilon = translation_epsilon;
		vgicp_cuda.setTransformationEpsilon(this->translation_epsilon);
	}
	/**
	 * @brief Set algorithm termination epsilon for rotation.
	 * @param rotation_epsilon Rotation epsilon
	 */
	void setRotationEpsilon(double rotation_epsilon)
	{
		this->rotation_epsilon = rotation_epsilon;
		vgicp_cuda.setTransformationRotationEpsilon(this->rotation_epsilon);
	}
	/**
	 * @brief Set algorithm termination epsilon for mean squared euclidean distance.
	 * @param euclidean_fitness_epsilon Euclidean fitness epsilon
	 */
	void setEuclideanFitnessEpsilon(double euclidean_fitness_epsilon)
	{
		this->euclidean_fitness_epsilon = euclidean_fitness_epsilon;
		vgicp_cuda.setEuclideanFitnessEpsilon(this->euclidean_fitness_epsilon);
	}
	/**
	 * @brief Set resolution for icp algorithm.
	 * @param resolution Resolution in meters
	 */
	void setResolution(double resolution)
	{
		this->resolution = resolution;
		vgicp_cuda.setResolution(this->resolution);
	}
	/**
	 * @brief Set kernel parameters for icp alogrithm.
	 * @param kernel_width Kernel with
	 * @param kernel_max_dist Kernel maximum distance
	 */
	void setKernelParameters(double kernel_width, double kernel_max_dist)
	{
		vgicp_cuda.setKernelWidth(kernel_width, kernel_max_dist);
	}
	/**
	 * @brief Set grid size for voxel filtering.
	 * @param voxelgrid_size Voxel grid size
	 */
	void setVoxelgridSize(double voxelgrid_size)
	{
		this->voxelgrid_size = static_cast<float>(voxelgrid_size);
	}
	/**
	 * @brief Set target pointcloud.
	 * @param target_cloud Target pointcloud
	 */
	void setTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud)
	{
		this->target_cloud = target_cloud;
	}
	/**
	 * @brief Set source pointcloud.
	 * @param source_cloud Source pointcloud
	 */
	void setSource(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud)
	{
		this->source_cloud = source_cloud;
	}
	void init();
	void icp(Eigen::Affine3d& final_transform, double& euclidean_fitness, const Eigen::Affine3d& initial_transform);
	void voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
	void outlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
	void getAlignedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
	/**
	 * @brief Get target pointcloud size.
	 * @return Target pointcloud size
	 */
	unsigned getTargetSize()
	{
		return target_size;
	}
	/**
	 * @brief Get source pointcloud size.
	 * @return Source pointcloud size
	 */
	unsigned getSourceSize()
	{
		return source_size;
	}

private:
	bool verbose;
	std::string package_share_directory;
	std::string data_path;
	int max_iterations = 1;

	double resolution;
	double translation_epsilon;
	double rotation_epsilon;
	double euclidean_fitness_epsilon;
	float voxelgrid_size;

	unsigned target_size;
	unsigned source_size;

	fast_gicp::FastVGICPCuda<pcl::PointXYZ, pcl::PointXYZ> vgicp_cuda;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud;

	double deg2rad(double degrees)
	{
		return (degrees * M_PI) / 180;
	}
	double rad2deg(double radians)
	{
		return (radians * 180) / M_PI;
	}
	Eigen::Vector3d extractEulerAngles(Eigen::Matrix3d rotation);
};
