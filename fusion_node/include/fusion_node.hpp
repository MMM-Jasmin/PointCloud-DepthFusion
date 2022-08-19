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
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/msg/string.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

// PCL
#include <pcl/io/ply_io.h>
// PROJECT
#include "camera_interfaces/msg/depth_frameset.hpp"
#include "camera_interfaces/srv/get_camera_parameters.hpp"
#include "pointcloud_processing/pointcloud.h"

#include "Timer.h"

/**
 * @brief ROS node for fusing point clouds generated from input framesets and project merged point cloud to output
 * image.
 */
class FusionNode : public rclcpp::Node
{
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
	typedef message_filters::sync_policies::ApproximateTime<camera_interfaces::msg::DepthFrameset, camera_interfaces::msg::DepthFrameset>
		FramesetSyncPolicy;
	typedef message_filters::Synchronizer<FramesetSyncPolicy> FramesetSync;
	using Clock = std::chrono::high_resolution_clock;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	FusionNode(const std::string& name = "fusion_node");
	~FusionNode();

	void setExitSignal(std::atomic<bool>* m_pExit_request)
	{
		this->m_pExit_request = m_pExit_request;
	}

	void init();
	// Left frameset callback accessable for direct callback injection
	void framesetLeftCallback(camera_interfaces::msg::DepthFrameset::UniquePtr msg);

private:
	void initSyncFramesets();
	void framesetRightCallback(camera_interfaces::msg::DepthFrameset::UniquePtr msg);
	void syncFramesetsTimerCallback();
	void processFramesCallback();

	void interpolateTransform(Eigen::Affine3d& interpolated_transform, const Eigen::Affine3d& left_transform, const Eigen::Affine3d& right_transform);
	void transformCallback(const geometry_msgs::msg::TransformStamped::SharedPtr transform_msg);

	void cameraInfo2Intrinsics(const sensor_msgs::msg::CameraInfo& camerainfo, Intrinsics& intrinsics);

	void allocateFrames();

	// Process frames
	void processFrames(camera_interfaces::msg::DepthFrameset::UniquePtr frameset_msg_left, camera_interfaces::msg::DepthFrameset::UniquePtr frameset_msg_right);

	double getTiming(time_point& start_time);

	/**
	 * @brief Convert degrees to radians.
	 * @param degrees Angle in degrees
	 * @return Angle in radians
	 */
	double deg2rad(double degrees)
	{
		return (degrees * M_PI) / 180;
	}

	void PrintFPS(const float fps, const float itrTime);
	void CheckFPS(uint64_t* pFrameCnt);
	void framesetSyncCallback(const camera_interfaces::msg::DepthFrameset::ConstSharedPtr& frameset_msg_left, const camera_interfaces::msg::DepthFrameset::ConstSharedPtr& frameset_msg_right);

private:
	std::atomic<bool>* m_pExit_request;
	std::string m_node_name = "fusion_node";
	std::string m_package_share_directory;
	bool m_save_data = false;

	// Left frameset from callback
	bool m_direct_callback = false;

	// Parameters
	bool m_verbose           = false;
	bool m_debug             = false;
	bool m_qos_sensor_data   = false;
	int m_qos_history_depth  = 5;
	float m_min_depth        = 0.5f;
	float m_max_depth        = 2.0f;
	bool m_align_frames      = false;
	bool m_vertical_image    = true;
	bool m_mirror_image      = true;
	bool m_use_median_filter = false;

	rclcpp::QoS m_qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
	CallbackGroup::SharedPtr m_callback_group_left;
	CallbackGroup::SharedPtr m_callback_group_right;
	CallbackGroup::SharedPtr m_callback_group_timer;

	// Camera parameter service
	rclcpp::Client<camera_interfaces::srv::GetCameraParameters>::SharedPtr m_cam_left_param_client;
	rclcpp::Client<camera_interfaces::srv::GetCameraParameters>::SharedPtr m_cam_right_param_client;

	// Sync framesets
	std::string m_topic_frameset_left  = "/camera_left/frameset";
	std::string m_topic_frameset_right = "/camera_right/frameset";
	rclcpp::Subscription<camera_interfaces::msg::DepthFrameset>::SharedPtr m_frameset_left_subscription;
	rclcpp::Subscription<camera_interfaces::msg::DepthFrameset>::SharedPtr m_frameset_right_subscription;
	rclcpp::TimerBase::SharedPtr m_sync_timer;
	rclcpp::TimerBase::SharedPtr m_process_timer;
	std::queue<camera_interfaces::msg::DepthFrameset::UniquePtr> m_frameset_left_msg_queue;
	std::queue<camera_interfaces::msg::DepthFrameset::UniquePtr> m_frameset_right_msg_queue;
	std::mutex m_mutex_frameset_left;
	std::mutex m_mutex_frameset_right;
	std::atomic_int m_frameset_left_counter;
	std::atomic_int m_frameset_right_counter;

	// Sync counter
	unsigned m_sync_callback_counter    = 0;
	unsigned m_sync_loop_counter        = 0;
	unsigned m_sync_drop_left_counter   = 0;
	unsigned m_sync_drop_right_counter  = 0;
	unsigned m_sync_hit_counter         = 0;
	unsigned m_sync_missed_counter      = 0;
	unsigned m_sync_hit_counter_acc     = 0;
	double m_sync_callback_duration_acc = 0;
	double m_sync_latency_acc           = 0;
	time_point m_sync_callback_time     = system_clock::now();

	// Transform
	bool m_set_camera_pose = false;
	Eigen::Affine3d m_camera_transform;
	std::vector<double> m_camera_translation;
	std::vector<double> m_camera_rotation;
	Eigen::Affine3d m_left_transform  = Eigen::Affine3d::Identity();
	Eigen::Affine3d m_right_transform = Eigen::Affine3d::Identity();
	rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr m_transform_subscription;

	// Frames
	uint8_t* m_pColor_frame_left;
	uint16_t* m_pDepth_frame_left;
	uint8_t* m_pColor_frame_right;
	uint16_t* m_pDepth_frame_right;
	uint8_t* m_pFused_frame;
	Frameset m_frameset_left;
	Frameset m_frameset_right;
	Frameset m_fused_frameset;
	cudaStream_t m_cuda_stream_left;
	cudaStream_t m_cuda_stream_right;
	bool m_caminfo_left_received  = false;
	bool m_caminfo_right_received = false;
	float m_depth_scale_left      = -1.f;
	float m_depth_scale_right     = -1.f;

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
	bool m_enable_profiling                     = false;
	std::vector<std::string> m_profiling_fields = { "loop", "callback", "filter", "deproject", "transform_right",
													"fuse", "transform", "project", "publish", "latency",
													"diff", "copy_to_gpu", "copy_from_gpu", "filter_image" };
	std::vector<std::vector<double>> m_profiling_vec;
	int m_profiling_size             = 400;
	std::string m_profiling_filename = "";
	time_point m_global_timer        = system_clock::now();

	bool m_save_pointclouds = false;
	unsigned m_ply_counter  = 0;


	bool m_print_fps = true;
	uint64_t m_frameCnt = 0;
	std::string m_FPS_STR = "test";
	Timer m_timer;        // Timer used to measure the time required for one iteration
	double m_elapsedTime; // Sum of the elapsed time, used to check if one second has passed

	message_filters::Subscriber<camera_interfaces::msg::DepthFrameset> subscriber_frameset_left;
	message_filters::Subscriber<camera_interfaces::msg::DepthFrameset> subscriber_frameset_right;
	FramesetSync* frameset_sync;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_sync_debug_publisher = nullptr;
	Clock::time_point m_sync_start_time;
	void processSyncedFrames(const camera_interfaces::msg::DepthFrameset::ConstSharedPtr& frameset_msg_left, const camera_interfaces::msg::DepthFrameset::ConstSharedPtr& frameset_msg_right);


};
