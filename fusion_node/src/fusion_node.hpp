#pragma once

// SYSTEM
#include <iostream>
#include <chrono>
#include <atomic>
#include <queue>
#include <fstream>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
// REALSENSE
#include <librealsense2/rs.hpp>
// PCL
#include <pcl/io/ply_io.h>
// PROJECT
#include "camera_interfaces/msg/depth_frameset.hpp"
#include "pointcloud_processing/pointcloud.h"
#include "camera_interfaces/srv/get_camera_parameters.hpp"

/**
 * @brief ROS node for fusing point clouds generated from input framesets and project merged point cloud to output
 * image.
 */
class FusionNode : public rclcpp::Node {
	typedef std::chrono::steady_clock::time_point time_point;
	typedef std::chrono::steady_clock system_clock;

#if defined(ROS_ELOQUENT)
	typedef rclcpp::callback_group::CallbackGroup CallbackGroup;
	typedef rclcpp::callback_group::CallbackGroupType CallbackGroupType;
	typedef rclcpp::executor::FutureReturnCode FutureReturnCode;
#else
	typedef rclcpp::CallbackGroup CallbackGroup;
	typedef rclcpp::CallbackGroupType CallbackGroupType;
	typedef rclcpp::FutureReturnCode FutureReturnCode;
#endif

 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	FusionNode(const std::string& name = "fusion_node");
	~FusionNode();
	void setExitSignal(std::atomic<bool>* exit_request) { this->exit_request = exit_request; }
	void init();
	// Left frameset callback accessable for direct callback injection
	void framesetLeftCallback(camera_interfaces::msg::DepthFrameset::UniquePtr msg);

 private:
	std::atomic<bool>* exit_request;
	std::string node_name = "fusion_node";
	std::string package_share_directory;
	bool save_data = false;

	// Left frameset from callback
	bool direct_callback = false;

	// Parameters
	bool verbose = false;
	bool debug = false;
	bool qos_sensor_data = false;
	int qos_history_depth = 5;
	float min_depth = 0.5f;
	float max_depth = 2.0f;
	bool align_frames = false;
	bool vertical_image = true;
	bool mirror_image = true;
	bool use_median_filter = false;

	rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
	CallbackGroup::SharedPtr callback_group_left;
	CallbackGroup::SharedPtr callback_group_right;
	CallbackGroup::SharedPtr callback_group_timer;

	// Camera parameter service
	rclcpp::Client<camera_interfaces::srv::GetCameraParameters>::SharedPtr cam_left_param_client;
	rclcpp::Client<camera_interfaces::srv::GetCameraParameters>::SharedPtr cam_right_param_client;

	// Sync framesets
	std::string topic_frameset_left = "/camera_left/frameset";
	std::string topic_frameset_right = "/camera_right/frameset";
	rclcpp::Subscription<camera_interfaces::msg::DepthFrameset>::SharedPtr frameset_left_subscription;
	rclcpp::Subscription<camera_interfaces::msg::DepthFrameset>::SharedPtr frameset_right_subscription;
	rclcpp::TimerBase::SharedPtr sync_timer;
	std::queue<camera_interfaces::msg::DepthFrameset::UniquePtr> frameset_left_msg_queue;
	std::queue<camera_interfaces::msg::DepthFrameset::UniquePtr> frameset_right_msg_queue;
	mutable std::mutex mutex_frameset_left;
	mutable std::mutex mutex_frameset_right;
	std::atomic_int frameset_left_counter;
	std::atomic_int frameset_right_counter;
	void initSyncFramesets();
	void framesetRightCallback(camera_interfaces::msg::DepthFrameset::UniquePtr msg);
	void syncFramesetsTimerCallback();

	// Sync counter
	unsigned sync_callback_counter = 0;
	unsigned sync_loop_counter = 0;
	unsigned sync_drop_left_counter = 0;
	unsigned sync_drop_right_counter = 0;
	unsigned sync_hit_counter = 0;
	unsigned sync_missed_counter = 0;
	unsigned sync_hit_counter_acc = 0;
	double sync_callback_duration_acc = 0;
	double sync_latency_acc = 0;
	time_point sync_callback_time = system_clock::now();

	// Transform
	bool set_camera_pose = false;
	Eigen::Affine3d camera_transform;
	std::vector<double> camera_translation;
	std::vector<double> camera_rotation;
	Eigen::Affine3d left_transform = Eigen::Affine3d::Identity();
	Eigen::Affine3d right_transform = Eigen::Affine3d::Identity();
	rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr transform_subscription;
	void interpolateTransform(Eigen::Affine3d& interpolated_transform, const Eigen::Affine3d& left_transform,
	                          const Eigen::Affine3d& right_transform);
	void transformCallback(const geometry_msgs::msg::TransformStamped::SharedPtr transform_msg);

	// Frames
	uint8_t* color_frame_left;
	uint16_t* depth_frame_left;
	uint8_t* color_frame_right;
	uint16_t* depth_frame_right;
	uint8_t* fused_frame;
	Frameset frameset_left;
	Frameset frameset_right;
	Frameset fused_frameset;
	cudaStream_t cuda_stream_left;
	cudaStream_t cuda_stream_right;
	bool caminfo_left_received = false;
	bool caminfo_right_received = false;
	void allocateFrames();
	float depth_scale_left = -1.f;
	float depth_scale_right = -1.f;

	// Fused image
	sensor_msgs::msg::Image fused_msg;
	image_transport::Publisher fused_publisher;

	// Camera parameters
	sensor_msgs::msg::CameraInfo camerainfo_color_left;
	sensor_msgs::msg::CameraInfo camerainfo_depth_left;
	sensor_msgs::msg::CameraInfo camerainfo_color_right;
	sensor_msgs::msg::CameraInfo camerainfo_depth_right;
	Intrinsics intrinsics_depth_left;
	Intrinsics intrinsics_color_left;
	Intrinsics intrinsics_depth_right;
	Intrinsics intrinsics_color_right;
	Extrinsics extrinsics_left;
	Extrinsics extrinsics_right;
	Intrinsics intrinsics_fused_image;
	void cameraInfo2Intrinsics(const sensor_msgs::msg::CameraInfo& camerainfo, Intrinsics& intrinsics);

	// Process frames
	void processFrames(camera_interfaces::msg::DepthFrameset::UniquePtr frameset_msg_left,
	                   camera_interfaces::msg::DepthFrameset::UniquePtr frameset_msg_right);

	// Profiling
	bool enable_profiling = false;
	std::vector<std::string> profiling_fields = {"loop", "callback",    "filter",        "deproject",   "transform_right",
	                                             "fuse", "transform",   "project",       "publish",     "latency",
	                                             "diff", "copy_to_gpu", "copy_from_gpu", "filter_image"};
	std::vector<std::vector<double>> profiling_vec;
	int profiling_size = 400;
	std::string profiling_filename = "";
	time_point global_timer = system_clock::now();
	double getTiming(time_point& start_time);
	bool save_pointclouds = false;
	unsigned ply_counter = 0;

	void save_pointclouds_ply(Pointcloud& cloud_left, Pointcloud& cloud_right, Eigen::Affine3d& fused_transform);

	/**
	 * @brief Convert degrees to radians.
	 * @param degrees Angle in degrees
	 * @return Angle in radians
	 */
	double deg2rad(double degrees) { return (degrees * M_PI) / 180; }
};
