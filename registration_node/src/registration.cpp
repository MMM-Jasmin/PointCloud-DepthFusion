// PROJECT
#include "registration.h"
// SYSTEM
#include <stdio.h>
#include <chrono>
// ROS
#include <ament_index_cpp/get_package_share_directory.hpp>

/**
 * @brief Constructor.
 * @param verbose True for verbose messages
 */
Registration::Registration(bool verbose) : verbose(verbose) {
	package_share_directory = ament_index_cpp::get_package_share_directory("registration_node");
	data_path = package_share_directory + "/data";
	init();
}

/**
 * @brief Destructor.
 */
Registration::~Registration() {}

/**
 * @brief Initialize vgicp registration module.
 */
void Registration::init() {
	vgicp_cuda.setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::GPU_RBF_KERNEL);

	// Set default parameters
	vgicp_cuda.setMaximumIterations(64);  // default: 64
	vgicp_cuda.setResolution(0.25);       // default: 1.0
	double kernel_width = 0.2;            // default: 0.25
	double kernel_max_dist = 1.0;         // default: 3.0
	vgicp_cuda.setKernelWidth(kernel_width, kernel_max_dist);
	// max_dist ca. 5 * kernel_width
	// setKernelWidth internal: if (max_dist <= 0.0) max_dist = kernel_width * 5.0;
	euclidean_fitness_epsilon = 1e-5;
	vgicp_cuda.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
	translation_epsilon = 5e-4;
	vgicp_cuda.setTransformationEpsilon(translation_epsilon);
	rotation_epsilon = 2e-3;
	vgicp_cuda.setTransformationRotationEpsilon(rotation_epsilon);
	voxelgrid_size = 0.005f;

	target_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	source_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	aligned_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}

/**
 * @brief Register source to target pointcloud with icp alogrithm.
 * @param final_transform Computed transformation matrix
 * @param euclidean_fitness Mean squared euclidean distance between registered pointclouds
 * @param initial_transform Initial guess of source pointcloud transformation
 */
void Registration::icp(Eigen::Affine3d& final_transform, double& euclidean_fitness,
                       const Eigen::Affine3d& initial_transform) {
	auto timer_start = std::chrono::steady_clock::now();
	unsigned long target_input_size = target_cloud->size();
	unsigned long source_input_size = source_cloud->size();

	// Filter pointclouds
	voxelFilter(source_cloud);
	voxelFilter(target_cloud);
	source_size = source_cloud->size();
	target_size = target_cloud->size();

	if (verbose) {
		std::cout << "+-- ICP started" << std::endl;
		std::cout << "| target size:       " << target_input_size << std::endl;
		std::cout << "| source size:       " << source_input_size << std::endl;
		std::cout << "| voxelsize:         " << voxelgrid_size << std::endl;
		std::cout << "| target filt. size: " << target_cloud->size() << std::endl;
		std::cout << "| source filt. size: " << source_cloud->size() << std::endl;
		std::cout << "| max iterations:    " << vgicp_cuda.getMaximumIterations() << std::endl;
		std::cout << "| resolution:        " << resolution << std::endl;
		std::cout << "| transl. epsilon:   " << vgicp_cuda.getTransformationEpsilon() << std::endl;
		std::cout << "| rotation epsilon:  " << vgicp_cuda.getTransformationRotationEpsilon() << std::endl;
		std::cout << "| euclidean epsilon: " << vgicp_cuda.getEuclideanFitnessEpsilon() << std::endl;
		std::cout << "+--" << std::endl;
	}

	// Iterative closest point
	vgicp_cuda.clearTarget();
	vgicp_cuda.clearSource();
	vgicp_cuda.setInputTarget(target_cloud);
	vgicp_cuda.setInputSource(source_cloud);
	vgicp_cuda.align(*aligned_cloud, initial_transform.matrix().cast<float>());  // Initial guess

	// Final transformation
	Eigen::Matrix4d final_transform_mat4 = vgicp_cuda.getFinalTransformation().cast<double>();
	final_transform.matrix() = final_transform_mat4;

	// Euclidean fitness (mean squared distance between points)
	euclidean_fitness = vgicp_cuda.getFitnessScore();

	bool has_converged = vgicp_cuda.hasConverged();
	if (!has_converged && verbose) {
		auto timer_end = std::chrono::steady_clock::now();
		auto timer_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(timer_end - timer_start);
		std::cout << "+-- ICP has not converged" << std::endl;
		std::cout << "| time:        " << timer_elapsed.count() << " ms" << std::endl;
		std::cout << "| fitness:     " << vgicp_cuda.getFitnessScore() << std::endl;
	}

	if (verbose && has_converged) {
		auto timer_end = std::chrono::steady_clock::now();
		auto timer_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(timer_end - timer_start);
		Eigen::Vector3d translation = final_transform.translation();
		double distance = translation.norm();
		Eigen::Vector3d euler_angles = extractEulerAngles(final_transform.rotation());
		std::cout << "+-- ICP finished" << std::endl;
		std::cout << "| time:          " << timer_elapsed.count() << " ms" << std::endl;
		std::cout << "| fitness:       " << euclidean_fitness << std::endl;
		std::cout << "| angle max deg: " << rad2deg(euler_angles.cwiseAbs().maxCoeff()) << std::endl;
		std::cout << "| angle x deg:   " << rad2deg(euler_angles(0)) << std::endl;
		std::cout << "| angle y deg:   " << rad2deg(euler_angles(1)) << std::endl;
		std::cout << "| angle z deg:   " << rad2deg(euler_angles(2)) << std::endl;
		std::cout << "| distance:      " << distance << std::endl;
		std::cout << "| translation:   " << final_transform_mat4(0, 3) << " " << final_transform_mat4(1, 3) << " "
		          << final_transform_mat4(2, 3) << std::endl;
		std::cout << "| rotation:      " << final_transform_mat4(0, 0) << " " << final_transform_mat4(0, 1) << " "
		          << final_transform_mat4(0, 2) << std::endl;
		std::cout << "|                " << final_transform_mat4(1, 0) << " " << final_transform_mat4(1, 1) << " "
		          << final_transform_mat4(1, 2) << std::endl;
		std::cout << "|                " << final_transform_mat4(2, 0) << " " << final_transform_mat4(2, 1) << " "
		          << final_transform_mat4(2, 2) << std::endl;
		std::cout << "+--" << std::endl;
	}
}

/**
 * @brief Filter pointcloud with voxel grid.
 * @param cloud Pointcloud to filter
 */
void Registration::voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxelgrid;
	voxelgrid.setLeafSize(voxelgrid_size, voxelgrid_size, voxelgrid_size);
	voxelgrid.setInputCloud(cloud);
	voxelgrid.filter(*cloud);
}

/**
 * @brief Filter pointcloud with statistical outlier removal.
 * @param cloud Pointcloud to filter
 */
void Registration::outlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(100);
	sor.setStddevMulThresh(0.25);
	sor.filter(*cloud);
}

/**
 * @brief Get pointcloud aligned by registration.
 * @param cloud Aligned source pointcloud
 */
void Registration::getAlignedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) { cloud = aligned_cloud; }

/**
 * @brief Extract euler angles from rotation matrix.
 * @param rotation Rotation matrix
 * @return Vector of euler angles around x,y and z-axis
 */
Eigen::Vector3d Registration::extractEulerAngles(Eigen::Matrix3d rotation) {
	double ea_x = std::atan2(rotation(2, 1), rotation(2, 2));
	double ea_y = -std::asin(rotation(2, 0));
	double ea_z = std::atan2(rotation(1, 0), rotation(0, 0));
	return Eigen::Vector3d(ea_x, ea_y, ea_z);
}
