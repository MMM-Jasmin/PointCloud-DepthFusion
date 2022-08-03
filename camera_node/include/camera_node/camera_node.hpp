#pragma once
#define CAMERA_NODE_USE_MULTITHREADED_EXECUTOR 0

#include <thread>
#include <future>
// ROS2
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/msg/string.hpp"
// OPENCV
#include <opencv2/opencv.hpp>
// LIBREALSENSE
#include <librealsense2/rs.hpp>
// PROJECT
#include "camera_interfaces/msg/depth_frameset.hpp"
#include "camera_interfaces/srv/get_camera_parameters.hpp"
#include "config.hpp"
#include "pointcloud_processing/frameset.h"
#include "realsense.hpp"

#include "Timer.h"

/**
 * @brief Camera node for pointcloud publishing.
 */
class CameraNode : public rclcpp::Node
{
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
	void setExitSignal(std::atomic<bool>* pExit_request)
	{
		m_pExit_request = pExit_request;
	}
	/**
	 * @brief Set verbosity of displayed messages.
	 * @param verbose True for displaying verbose messages
	 */
	void setVerbosity(const bool& verbose)
	{
		m_pConfig->setVerbosity(verbose);
	}
	void stop();

private:
	
	void getNextImages (uint8_t * next_color_frame_bytes, uint16_t * next_depth_frame_bytes, double &m_timestamp, int timeout);
	void publishImage(uint8_t * color_image, int width, int height, std::string frame_id, rclcpp::Time ros_timestamp, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr message_publisher);
	void publishDepthImage(uint16_t * depth_image, int width, int height, std::string frame_id, rclcpp::Time ros_timestamp, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr message_publisher);
	void publishFrameset(uint8_t * color_image, int color_width, int color_height, uint16_t * depth_image, int depth_width, int depth_height, rclcpp::Time ros_timestamp);
	void publishEverything();

	void getCameraParameters(const std::shared_ptr<camera_interfaces::srv::GetCameraParameters::Request> request, std::shared_ptr<camera_interfaces::srv::GetCameraParameters::Response> response) const;

private:
	std::atomic<bool>* m_pExit_request = nullptr;
	std::string m_package_share_directory;
	bool m_verbose          = false;
	bool m_debug            = false;
	bool m_use_rs_align     = true;

	bool m_print_fps = true;
	uint64_t m_frameCnt = 0;
	std::string m_FPS_STR = "";
	Timer m_timer;        // Timer used to measure the time required for one iteration
	double m_elapsedTime; // Sum of the elapsed time, used to check if one second has passed

	std::string m_node_name   = "camera_node";
	rclcpp::QoS m_qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
	rclcpp::QoS m_qos_profile_sysdef = rclcpp::SystemDefaultsQoS();

	rclcpp::TimerBase::SharedPtr m_publish_timer                                             	= nullptr;
	rclcpp::Publisher<camera_interfaces::msg::DepthFrameset>::SharedPtr m_frameset_publisher 	= nullptr;
	//rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_publisher 					= nullptr;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_small_publisher 				= nullptr;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_depth_image_publisher 				= nullptr;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_fps_publisher 	=	 nullptr;

	
	Config* m_pConfig       = nullptr;
	Realsense* m_pRealsense = nullptr;

	std::chrono::high_resolution_clock::time_point m_global_timer = std::chrono::high_resolution_clock::time_point(std::chrono::nanoseconds(0));
	rclcpp::Time m_last_frame_timestamp;
	Intrinsics m_color_intrinsics;
	Intrinsics m_depth_intrinsics;
	Extrinsics m_extrinsics;

	bool m_buffer = true;
	double m_timestamp = 0.0;
	uint8_t* m_pColor_frame_0  = nullptr;
	uint8_t* m_pColor_frame_1  = nullptr;
	uint8_t* m_pColor_frame_small_0  = nullptr;
	uint8_t* m_pColor_frame_small_1  = nullptr;
	uint16_t* m_pDepth_frame_0 = nullptr;
	uint16_t* m_pDepth_frame_1 = nullptr;

	rclcpp::Time m_ros_timestamp;

	sensor_msgs::msg::CameraInfo m_color_camerainfo;
	sensor_msgs::msg::CameraInfo m_depth_camerainfo;
	sensor_msgs::msg::Image::SharedPtr m_color_msg = nullptr;
	sensor_msgs::msg::Image::SharedPtr m_depth_msg = nullptr;
	Frameset m_frameset;
	//time_point m_timer = hires_clock::now();

	rclcpp::Service<camera_interfaces::srv::GetCameraParameters>::SharedPtr m_service = nullptr;
	double m_fps_avg                                                                  = 0.0;
	void PrintFPS(const float fps, const float itrTime);
	void CheckFPS(uint64_t* pFrameCnt);
};
