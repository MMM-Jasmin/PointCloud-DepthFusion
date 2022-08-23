#pragma once
// SYSTEM
// #include <atomic>
#include <chrono>
#include <iostream>
#include <string>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
// OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
// PROJECT
#include "camera_interfaces/msg/depth_frameset.hpp"

/**
 * @brief Image viewer node class for receiving and visualizing image and frameset messages.
 */
class ImageNode : public rclcpp::Node {
	typedef std::chrono::high_resolution_clock::time_point time_point;
	typedef std::chrono::high_resolution_clock Clock;

 public:
	ImageNode();
	~ImageNode();
	void init();
	// void setExitSignal(std::atomic<bool>* m_p_exit_request) { this->m_p_exit_request = m_p_exit_request; }

 private:
	// std::atomic<bool>* m_p_exit_request;
	bool m_verbose = false;
	void waitForKey();
	double m_depth_image_scale = 0.1;

	// Topics
	std::string m_topic_fused = "/fused_image";
	std::string m_topic_depth_left = "/camera_left/depth/image";
	std::string m_topic_depth_right = "/camera_right/depth/image";
	std::string m_topic_frameset_left = "/camera_left/frameset";
	std::string m_topic_frameset_right = "/camera_right/frameset";
	// std::string m_topic_depth = "/camera_left/depth/image";
	// std::string m_topic_depth_thr = "/camera_left/depth/image_threshold";
	// std::string m_topic_image_small = "/camera_left/color/image_small";

	// Quality of service
	rclcpp::QoS m_qos_sensor_profile = rclcpp::SensorDataQoS();
	rclcpp::QoS m_qos_profile = rclcpp::SystemDefaultsQoS();

	// Subscriptions
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_fused_subscription;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_depth_left_subscription = nullptr;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_depth_right_subscription = nullptr;
	rclcpp::Subscription<camera_interfaces::msg::DepthFrameset>::SharedPtr m_frameset_left_subscription = nullptr;
	rclcpp::Subscription<camera_interfaces::msg::DepthFrameset>::SharedPtr m_frameset_right_subscription = nullptr;
	// rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_depth_subscription;
	// rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_depth_thr_subscription;
	// rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_small_subscription;
	// rclcpp::Subscription<camera_interfaces::msg::DepthFrameset>::SharedPtr m_frameset_subscription;

	// Image callbacks
	void fusedCallback(sensor_msgs::msg::Image::UniquePtr img_msg);
	void depthLeftCallback(sensor_msgs::msg::Image::SharedPtr img_msg);
	void depthRightCallback(sensor_msgs::msg::Image::SharedPtr img_msg);
	void framesetLeftCallback(camera_interfaces::msg::DepthFrameset::UniquePtr fset_msg);
	void framesetRightCallback(camera_interfaces::msg::DepthFrameset::UniquePtr fset_msg);
	// void depthCallback(sensor_msgs::msg::Image::SharedPtr img_msg);
	// void imageSmallCallback(sensor_msgs::msg::Image::SharedPtr img_msg);
	// void framesetCallback(camera_interfaces::msg::DepthFrameset::UniquePtr fset_msg);

	// Windows
	bool m_show_fused = true;
	bool m_show_depth_left = false;
	bool m_show_depth_right = false;
	bool m_show_frameset_left = false;
	bool m_show_frameset_right = false;
	std::string m_window_name_fused = "Fused Image";
	std::string m_window_name_depth_left = "Left Depth Image";
	std::string m_window_name_depth_right = "Right Depth Image";
	std::string m_window_name_frameset_left = "Left Frameset";
	std::string m_window_name_frameset_right = "Right Frameset";
	// std::string m_window_name_depth = "Depth_Frame";
	// std::string m_window_name_color = "Color_Frame";
	// std::string m_window_name_frameset = "Frameset_Frame";
	// std::string m_window_name_image_small = "Image_small_Frame";

	// Button callbacks
	static void buttonFusedCallback(int state, void* user_data);
	static void buttonDepthLeftCallback(int state, void* user_data);
	static void buttonDepthRightCallback(int state, void* user_data);
	static void buttonFramesetLeftCallback(int state, void* user_data);
	static void buttonFramesetRightCallback(int state, void* user_data);

	// Timings
	time_point m_callback_time_fused = Clock::now();
	time_point m_callback_time_depth_left = Clock::now();
	time_point m_callback_time_depth_right = Clock::now();
	time_point m_callback_time_frameset_left = Clock::now();
	time_point m_callback_time_frameset_right = Clock::now();
};
