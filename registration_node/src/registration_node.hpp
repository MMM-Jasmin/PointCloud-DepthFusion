#pragma once

// SYSTEM
#include <fstream>
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.h>
// ICP
#include <fast_gicp/cuda/fast_vgicp_cuda.cuh>
// PROJECT
#include "pointcloud_processing/frameset.h"
#include "pointcloud_processing/pointcloud.h"
#include "pointcloud_processing/intrinsics.h"
#include "registration.h"

/**
 * @brief Ros2 pointcloud registration node class.
 */
class RegistrationNode : public rclcpp::Node {
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>
		  ImageSyncPolicy;
	typedef message_filters::Synchronizer<ImageSyncPolicy> ImageSync;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo>
		  CameraInfoSyncPolicy;
	typedef message_filters::Synchronizer<CameraInfoSyncPolicy> CameraInfoSync;
	typedef std::chrono::high_resolution_clock::time_point time_point;
	typedef std::chrono::high_resolution_clock hires_clock;

 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	RegistrationNode();
	~RegistrationNode();
	void setExitSignal(std::atomic<bool>* exit_request) { this->exit_request = exit_request; }
	void init();
	void initTimer();

 private:
	std::string topic_depth_left = "/camera_left/depth/image";
	std::string topic_depth_right = "/camera_right/depth/image";
	std::string topic_camerainfo_left = "/camera_left/depth/camera_info";
	std::string topic_camerainfo_right = "/camera_right/depth/camera_info";

	std::atomic<bool>* exit_request;
	std::string node_name = "registration_node";
	std::string package_share_directory;
	bool verbose = false;
	bool debug = false;
	float min_depth = 0.5f;
	float max_depth = 2.0f;
	float depth_scale = 0.00025f;
	float depth_scale_left = -1.f;
	float depth_scale_right = -1.f;
	bool publish_clouds = false;
	double resolution = 0.05;
	double spin_rate = 0.5;
	double voxelsize = 0.001;
	std::array<int, 4> roi_left = {-1, -1, -1, -1};
	std::array<int, 4> roi_right = {-1, -1, -1, -1};
	bool qos_sensor_data = false;
	int qos_history_depth = 2;
	double kernel_width = 0.005;
	double kernel_max_dist = 0.025;
	double rotation_epsilon = 2e-3;
	double translation_epsilon = 1e-4;
	double fitness_epsilon = 1e-12;
	int max_iterations = 1000;

	bool initial_phase = true;
	double initial_resolution = 0.1;
	double voxel_resolution_factor = 0.1;
	double resolution_step = 0.05;
	double current_resolution = initial_resolution;
	double current_voxelsize = initial_resolution * voxel_resolution_factor;
	double best_fitness = std::numeric_limits<double>::max();
	bool cam_upside_down = true;
	bool save_transform = false;
	bool load_transform = false;
	bool adjust_resolution = false;
	bool discard_transform = false;
	bool reset_initial_guess = true;
	bool set_initial_transform = false;

	Registration* registration = nullptr;
	rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle = nullptr;
	message_filters::Subscriber<sensor_msgs::msg::Image> subscriber_depth_left;
	message_filters::Subscriber<sensor_msgs::msg::Image> subscriber_depth_right;
	message_filters::Subscriber<sensor_msgs::msg::CameraInfo> subscriber_camerainfo_left;
	message_filters::Subscriber<sensor_msgs::msg::CameraInfo> subscriber_camerainfo_right;
	rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr publisher_transform = nullptr;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_target_cloud = nullptr;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_aligned_cloud = nullptr;
	ImageSync* depth_sync;
	CameraInfoSync* camera_info_sync;
	int64_t last_callback_timestamp = hires_clock::now().time_since_epoch().count();
	std::string transform_filename;
	std::fstream transform_file;
	sensor_msgs::msg::CameraInfo camera_info_left;
	sensor_msgs::msg::CameraInfo camera_info_right;
	Eigen::Affine3d start_transform;
	Eigen::Affine3d initial_transform;
	Eigen::Affine3d last_transform;
	cudaStream_t target_cuda_stream;
	cudaStream_t source_cuda_stream;
	Frameset target_frameset;
	Frameset source_frameset;
	rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
	rclcpp::TimerBase::SharedPtr timer;

	bool enable_profiling = false;
	std::vector<std::string> profiling_fields = {"euclidean_fitness", "icp_duration", "points_target", "points_source"};
	std::vector<std::vector<double>> profiling_vec;
	int profiling_size = 40;
	std::string profiling_filename = "";

	void icp(const sensor_msgs::msg::Image::ConstSharedPtr& target_depth_msg,
	         const sensor_msgs::msg::Image::ConstSharedPtr& source_depth_msg,
	         sensor_msgs::msg::CameraInfo& target_camerainfo, sensor_msgs::msg::CameraInfo& source_camerainfo,
	         float depth_scale, Eigen::Affine3d& final_transform, const Eigen::Affine3d& initial_transform);
	void depthSyncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg_left,
	                       const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg_right);
	void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_left,
	                        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_right);
	void deprojectDepthCpu(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	                       const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
	                       sensor_msgs::msg::CameraInfo& camerainfo, float depth_scale);
	void declareParameters();
	rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);
	bool saveTransform(Eigen::Affine3d& transform, const std::string filename);
	bool loadTransform(Eigen::Affine3d& transform, const std::string filename);
	void cameraInfo2Intrinsics(const sensor_msgs::msg::CameraInfo& camerainfo, Intrinsics& intrinsics);
	void setResolution(double resolution);
	void timerCallback();

	/**
	 * @brief Extract euler angles from rotation matrix.
	 * @param rotation Rotation matrix
	 * @return Vector of euler angles around x,y and z-axis
	 */
	Eigen::Vector3d extractEulerAngles(Eigen::Matrix3d rotation) {
		double ea_x = std::atan2(rotation(2, 1), rotation(2, 2));
		double ea_y = -std::asin(rotation(2, 0));
		double ea_z = std::atan2(rotation(1, 0), rotation(0, 0));
		return Eigen::Vector3d(ea_x, ea_y, ea_z);
	}
	/**
	 * @brief Convert degrees to radians.
	 * @param degrees Angle in degrees
	 * @return Angle in radians
	 */
	double deg2rad(double degrees) { return (degrees * M_PI) / 180; }
	/**
	 * @brief Convert radians to degrees.
	 * @param radians Angle in radians
	 * @return Angle in degrees
	 */
	double rad2deg(double radians) { return (radians * 180) / M_PI; }
};
