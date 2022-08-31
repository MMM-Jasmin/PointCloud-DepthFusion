#pragma once

// SYSTEM
#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <queue>
// ROS
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#if defined(ROS_HUMBLE)
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

// Message filter sync
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>

// REALSENSE
#include <librealsense2/rs.hpp>
// PCL
#include <pcl/io/ply_io.h>
// PROJECT
#include "camera_interfaces/msg/depth_frameset.hpp"
#include "camera_interfaces/srv/get_camera_parameters.hpp"
#include "pointcloud_processing/pointcloud.h"

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

	// Message filter sync
	typedef message_filters::sync_policies::ApproximateTime<camera_interfaces::msg::DepthFrameset,
	                                                        camera_interfaces::msg::DepthFrameset>
	    FramesetSyncPolicy;
	typedef message_filters::Synchronizer<FramesetSyncPolicy> FramesetSync;
	using Clock = std::chrono::high_resolution_clock;

 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	FusionNode(const std::string& name = "fusion_node");
	~FusionNode();
	void setExitSignal(std::atomic<bool>* m_p_exit_request) { this->m_p_exit_request = m_p_exit_request; }
	void init();

 private:
	void interpolateTransform(Eigen::Affine3d& interpolated_transform, const Eigen::Affine3d& left_transform,
	                          const Eigen::Affine3d& right_transform);
	void transformCallback(const geometry_msgs::msg::TransformStamped::SharedPtr transform_msg);
	void cameraInfo2Intrinsics(const sensor_msgs::msg::CameraInfo& camerainfo, Intrinsics& intrinsics);
	void allocateFrames();

	// Process frames
	// void processFrames(camera_interfaces::msg::DepthFrameset::UniquePtr frameset_msg_left,
	// camera_interfaces::msg::DepthFrameset::UniquePtr frameset_msg_right);
	double getTiming(time_point& start_time);
	void save_pointclouds_ply(Pointcloud& cloud_left, Pointcloud& cloud_right, Eigen::Affine3d& fused_transform);
	/**
	 * @brief Convert degrees to radians.
	 * @param degrees Angle in degrees
	 * @return Angle in radians
	 */
	double deg2rad(double degrees) { return (degrees * M_PI) / 180; }
	void framesetSyncCallback(const camera_interfaces::msg::DepthFrameset::ConstSharedPtr& frameset_msg_left,
	                          const camera_interfaces::msg::DepthFrameset::ConstSharedPtr& frameset_msg_right);

 private:
	std::atomic<bool>* m_p_exit_request;
	std::string m_node_name = "fusion_node";
	std::string m_package_share_directory;
	bool m_save_data = false;

	// Parameters
	bool m_verbose = false;
	bool m_debug = false;
	bool m_qos_sensor_data = false;
	int m_qos_history_depth = 5;
	float m_min_depth = 0.5f;
	float m_max_depth = 2.0f;
	bool m_align_frames = false;
	bool m_vertical_image = true;
	bool m_mirror_image = true;
	bool m_use_median_filter = false;

	// Quality of service
	rclcpp::QoS m_qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
	CallbackGroup::SharedPtr m_callback_group_left;
	CallbackGroup::SharedPtr m_callback_group_right;
	CallbackGroup::SharedPtr m_callback_group_timer;

	// Camera parameter service
	rclcpp::Client<camera_interfaces::srv::GetCameraParameters>::SharedPtr m_cam_left_param_client;
	rclcpp::Client<camera_interfaces::srv::GetCameraParameters>::SharedPtr m_cam_right_param_client;
	void initCameraParameters();

	// Sync framesets
	std::string m_topic_frameset_left = "/camera_left/frameset";
	std::string m_topic_frameset_right = "/camera_right/frameset";

	// Transform
	bool m_set_camera_pose = false;
	Eigen::Affine3d m_camera_transform;
	std::vector<double> m_camera_translation;
	std::vector<double> m_camera_rotation;
	Eigen::Affine3d m_left_transform = Eigen::Affine3d::Identity();
	Eigen::Affine3d m_right_transform = Eigen::Affine3d::Identity();
	rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr m_transform_subscription;

	// Frames
	uint8_t* m_p_color_frame_left;
	uint16_t* m_p_depth_frame_left;
	uint8_t* m_p_color_frame_right;
	uint16_t* m_p_depth_frame_right;
	uint8_t* m_p_fused_frame;
	Frameset m_frameset_left;
	Frameset m_frameset_right;
	Frameset m_fused_frameset;
	cudaStream_t m_cuda_stream_left;
	cudaStream_t m_cuda_stream_right;
	cudaStream_t m_cuda_stream_fused;
	// bool m_caminfo_left_received = false;
	// bool m_caminfo_right_received = false;
	float m_depth_scale_left = -1.f;
	float m_depth_scale_right = -1.f;

	// Fused image
	sensor_msgs::msg::Image m_fused_msg;
	image_transport::Publisher m_fused_publisher;

	// Camera parameters
	sensor_msgs::msg::CameraInfo m_camerainfo_color_left;
	sensor_msgs::msg::CameraInfo m_camerainfo_depth_left;
	sensor_msgs::msg::CameraInfo m_camerainfo_color_right;
	sensor_msgs::msg::CameraInfo m_camerainfo_depth_right;
	Intrinsics m_intrinsics_depth_left;
	Intrinsics m_intrinsics_color_left;
	Intrinsics m_intrinsics_depth_right;
	Intrinsics m_intrinsics_color_right;
	Extrinsics m_extrinsics_left;
	Extrinsics m_extrinsics_right;
	Intrinsics m_intrinsics_fused_image;

	// Profiling
	bool m_enable_profiling = false;
	// std::vector<std::string> m_profiling_fields = {
	//     "loop",    "callback", "filter",  "deproject", "transform_right", "fuse",          "transform",
	//     "project", "publish",  "latency", "diff",      "copy_to_gpu",     "copy_from_gpu", "filter_image"};
	// std::vector<std::vector<double>> m_profiling_vec;
	// int m_profiling_size = 400;
	// std::string m_profiling_filename = "";
	// time_point m_global_timer = system_clock::now();

	bool m_save_pointclouds = false;
	unsigned m_ply_counter = 0;

	// Message filter sync
	message_filters::Subscriber<camera_interfaces::msg::DepthFrameset> subscriber_frameset_left;
	message_filters::Subscriber<camera_interfaces::msg::DepthFrameset> subscriber_frameset_right;
	FramesetSync* frameset_sync;
	void processSyncedFramesets(const camera_interfaces::msg::DepthFrameset::ConstSharedPtr& frameset_msg_left,
	                            const camera_interfaces::msg::DepthFrameset::ConstSharedPtr& frameset_msg_right);

	// Buffering
	bool m_use_framebuffer = true;
	std::vector<uint8_t*> m_color_buffers_left;
	std::vector<uint8_t*> m_depth_buffers_left;
	std::vector<uint8_t*> m_color_buffers_right;
	std::vector<uint8_t*> m_depth_buffers_right;
	unsigned m_framebuffer_count = 5;
	unsigned m_framebuffer_idx = 0;
	std::atomic_uint m_framebuffer_write_idx;
	std::atomic_uint m_framebuffer_read_idx;
	std::mutex m_mutex_framebuffer;
	// Fused image buffering
	unsigned m_fusedbuffer_count = 5;
	unsigned m_fusedbuffer_idx = 0;
	std::mutex m_mutex_fusedbuffer;
	std::vector<uint8_t*> m_fusedbuffers;

	// Sync timer
	bool m_use_sync_timer = false;
	unsigned m_sync_timer_hz = 120;
	void processSyncedFrames(uint8_t* color_frame_left, uint16_t* depth_frame_left, uint8_t* color_frame_right,
	                         uint16_t* depth_frame_right);
	rclcpp::TimerBase::SharedPtr m_process_timer;
	void processTimerCallback();
	std::atomic_bool m_has_new_framesets;
	// std::queue<unsigned> m_framebuffer_indices;

	// Debug
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_sync_debug_publisher = nullptr;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_process_debug_publisher = nullptr;
	Clock::time_point m_sync_start_time;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_debug_publisher = nullptr;
	Clock::time_point m_process_start_time;
	void publishFrame(const camera_interfaces::msg::DepthFrameset::ConstSharedPtr& frameset_msg);
};
