#pragma once
// SYSTEM
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
 * @brief Image viewer node class for receiving and visualizing fused image.
 */
class ImageNode : public rclcpp::Node
{
	typedef std::chrono::high_resolution_clock::time_point time_point;
	typedef std::chrono::high_resolution_clock hires_clock;

public:
	ImageNode();
	void init();

private:
	std::string m_topic_depth  			= "/camera_left/depth/image";
	std::string m_topic_depth_thr		= "/camera_left/depth/image_threshold";
	std::string m_topic_frameset    = "/camera_left/frameset";
	std::string m_topic_image_small	= "/camera_left/color/image_small";
	std::string m_topic_fused_image = "/fused_image";

	std::string m_window_name_depth	 			= "Depth_Frame";
	std::string m_window_name_color	 			= "Color_Frame";
	std::string m_window_name_frameset		= "Frameset_Frame";
	std::string m_window_name_image_small	= "Image_small_Frame";
	std::string m_window_name_fused				= "Fused_Image";

	time_point m_callback_time = hires_clock::now();
	time_point m_callback_time_image_small = hires_clock::now();
	time_point m_callback_time_depth = hires_clock::now();

	double m_loop_duration = 0.0;
	double m_loop_duration_image_small = 0.0;
	double m_loop_duration_depth = 0.0;

	rclcpp::QoS m_qos_sensor_profile = rclcpp::SensorDataQoS();
	rclcpp::QoS m_qos_profile = rclcpp::SystemDefaultsQoS();

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_depth_subscription;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_depth_thr_subscription;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_small_subscription;
	rclcpp::Subscription<camera_interfaces::msg::DepthFrameset>::SharedPtr m_frameset_subscription;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_fused_image_subscription;

	void depthCallback(sensor_msgs::msg::Image::SharedPtr img_msg);
	void imageSmallCallback(sensor_msgs::msg::Image::SharedPtr img_msg);
	void framesetCallback(camera_interfaces::msg::DepthFrameset::UniquePtr fset_msg);
	void fusedCallback(sensor_msgs::msg::Image::UniquePtr img_msg);
	static void button1Callback(int state, void* userData);
};
