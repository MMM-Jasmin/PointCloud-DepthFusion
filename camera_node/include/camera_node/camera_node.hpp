#pragma once
#define CAMERA_NODE_USE_MULTITHREADED_EXECUTOR 0

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <image_transport/image_transport.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.h>
// LIBREALSENSE
#include <librealsense2/rs.hpp>
// PROJECT
#include "config.hpp"
#include "realsense.hpp"
#include "pointcloud_processing/frameset.h"
#include "pointcloud_processing/pointcloud.h"
#include "camera_interfaces/msg/depth_frameset.hpp"
#include "camera_interfaces/srv/get_camera_parameters.hpp"

/**
 * @brief Camera node for pointcloud publishing.
 */
class CameraNode : public rclcpp::Node {
	typedef std::chrono::high_resolution_clock::time_point time_point;
	typedef std::chrono::high_resolution_clock hires_clock;
#if defined(ROS_ELOQUENT)
	typedef rclcpp::callback_group::CallbackGroup CallbackGroup;
	typedef rclcpp::callback_group::CallbackGroupType CallbackGroupType;
#else
	typedef rclcpp::CallbackGroup CallbackGroup;
	typedef rclcpp::CallbackGroupType CallbackGroupType;
#endif
#if CAMERA_NODE_USE_MULTITHREADED_EXECUTOR
	typedef rclcpp::executors::MultiThreadedExecutor Executor;
#else
	typedef rclcpp::executors::SingleThreadedExecutor Executor;
#endif

 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	CameraNode(const std::string& name = "camera_node");
	~CameraNode();
	void init();

	/**
	 * @brief Set signal for program exit request.
	 * @param exit_request Exit request signal
	 */
	void setExitSignal(std::atomic<bool>* exit_request) { this->exit_request = exit_request; }
	/**
	 * @brief Set verbosity of displayed messages.
	 * @param verbose True for displaying verbose messages
	 */
	void setVerbosity(bool verbose) { config->verbose = verbose; }
	/**
	 * @brief Set flag for profiling.
	 * @param profiling True for activating profiling
	 */
	void setProfiling(bool profiling) { config->profiling = profiling; }
	void stop();

 private:
	std::atomic<bool>* exit_request;
	std::string package_share_directory;
	bool verbose = false;
	bool debug = false;
	bool use_rs_align = true;
	bool use_rs_queue = false;
	bool use_rs_callback = false;
	bool use_rs_timestamp = true;

	std::string node_name = "camera_node";
	std::string topic_depth = "";
	std::string topic_frameset = "";
	rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));

	rclcpp::TimerBase::SharedPtr publish_timer;
	rclcpp::Publisher<camera_interfaces::msg::DepthFrameset>::SharedPtr frameset_publisher;

	Config* config = nullptr;
	Realsense* realsense = nullptr;
	Executor* executor = nullptr;
	image_transport::CameraPublisher color_publisher;
	image_transport::CameraPublisher depth_publisher;

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher;
	std::chrono::high_resolution_clock::time_point global_timer =
	    std::chrono::high_resolution_clock::time_point(std::chrono::nanoseconds(0));
	rclcpp::Time last_frame_timestamp;
	Intrinsics color_intrinsics;
	Intrinsics depth_intrinsics;
	Extrinsics extrinsics;

	uint8_t* color_frame;
	uint16_t* depth_frame;
	sensor_msgs::msg::CameraInfo color_camerainfo;
	sensor_msgs::msg::CameraInfo depth_camerainfo;
	sensor_msgs::msg::Image::SharedPtr color_msg;
	sensor_msgs::msg::Image::SharedPtr depth_msg;
	sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg;
	Frameset frameset;
	time_point timer = hires_clock::now();
	cudaStream_t cuda_stream;

	rclcpp::TimerBase::SharedPtr publish_cloud_timer;
	CallbackGroup::SharedPtr callback_group_timer;
	rclcpp::Service<camera_interfaces::srv::GetCameraParameters>::SharedPtr service;
	double fps_avg = 0.;

	bool enable_profiling = false;
	std::vector<std::string> profiling_fields = {"loop",    "callback",   "getframes",  "message_creation",
	                                             "publish", "latency_in", "latency_out"};
	std::vector<std::vector<double>> profiling_vec;
	int profiling_size = 400;
	std::string profiling_filename = "";

	void startTimer();
	void stopTimer(std::string msg);
	void convertRs2Intrinsics(const rs2_intrinsics& rs_intrinsics, Intrinsics& intrinsics);
	void publishPointcloud(Pointcloud& cloud, double& timestamp);

	void processPointcloud();
	void initPointcloud();
	void publishDepth();

	void publishFrameset();
	void getCameraParameters(const std::shared_ptr<camera_interfaces::srv::GetCameraParameters::Request> request,
	                         std::shared_ptr<camera_interfaces::srv::GetCameraParameters::Response> response);
	rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);
};
