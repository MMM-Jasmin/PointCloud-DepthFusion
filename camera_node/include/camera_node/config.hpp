#pragma once

// SYSTEM
#include <iostream>
// ROS2
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Program mode to run.
 */
enum Mode {
	PRIMARY,     /*!< Node for primary camera */
	SECONDARY,   /*!< Node for secondary camera */
	REGISTRATION /*!< Registration node */
};

/**
 * @brief Node configuration class.
 */
class Config {
 public:
	bool debug = false;
	bool verbose = false;
	bool publish_fps = false;
	bool publish_duration = false;
	bool publish_latency = false;
	bool profiling = false;
	bool log_to_file = false;
	bool print_statistics = false;
	bool qos = false;
	int log_count_max = 400;
	bool simulation = false;
	bool save_data = false;
	Mode mode = PRIMARY;
	float min_depth = 0.5f;
	float max_depth = 2.0f;
	float depth_scale = 0.0001f;
	std::string camera_serial_no = "";
	std::array<int, 4> roi = {-1, -1, -1, -1};
	bool enable_rs_debug = false;
	bool qos_sensor_data = false;
	int qos_history_depth = 2;

	std::string node_namespace = "";
	std::string topic_fps = "profiling/fps";
	std::string topic_duration = "profiling/duration";
	std::string topic_latency = "profiling/latency";
	std::string topic_color = "color/image";
	std::string topic_depth = "depth/image";
	std::string topic_points = "points";
	std::string topic_fused = "fused_image";

	std::string color_image_filename = "";
	std::string depth_image_filename = "";
	std::string color_intrinsics_filename = "";
	std::string depth_intrinsics_filename = "";
	double sim_depth_scale = 0.00025;
	unsigned simulation_framerate = 30;

	Config(rclcpp::Node* node);
	~Config();
	void registerRealsenseParameterCallback(std::function<void(std::string, float)> realsense_parameter_callback);
	std::function<void(std::string, float)> realsenseParameterCallback;
	void declareNodeParameters();

 private:
	rclcpp::Node* node;
	rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle = nullptr;

	rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);
};
