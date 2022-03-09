#pragma once
// SYSTEM
#include <iostream>
#include <chrono>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
// OPENCV
#include <opencv2/opencv.hpp>

/**
 * @brief Image viewer node class for receiving and visualizing fused image.
 */
class ImageNode : public rclcpp::Node {
	typedef std::chrono::high_resolution_clock::time_point time_point;
	typedef std::chrono::high_resolution_clock hires_clock;

 public:
	ImageNode();
	void init();

 private:
	std::string topic = "";
	std::string window_name = "Frame";
	
	time_point callback_time = hires_clock::now();

	//rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();
	rclcpp::QoS qos_profile = rclcpp::SystemDefaultsQoS();
	
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription;

	void imageCallback(sensor_msgs::msg::Image::SharedPtr img_msg);
};
