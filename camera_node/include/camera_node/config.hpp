#pragma once

// SYSTEM
#include <iostream>
// ROS2
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Program mode to run.
 */
enum Mode
{
	PRIMARY,     /*!< Node for primary camera */
	SECONDARY,   /*!< Node for secondary camera */
	REGISTRATION /*!< Registration node */
};

/**
 * @brief Node configuration class.
 */
class Config
{
public:
	Config(rclcpp::Node *node);
	~Config();
	void registerRealsenseParameterCallback(std::function<void(std::string, float)> realsense_parameter_callback);
	void declareNodeParameters();

	const bool &debug() 			const { return m_debug; }
	const bool &verbose() 			const { return m_verbose; }
	const bool &publish_fps() 		const { return m_publish_fps; }
	const bool &publish_duration() 	const { return m_publish_duration; }
	const bool &publish_latency() 	const { return m_publish_latency;	}
	const bool &profiling() 		const { return m_profiling; }
	const bool &log_to_file() 		const { return m_log_to_file; }
	const bool &print_statistics() 	const { return m_print_statistics; }
	const bool &qos() 				const { return m_qos; }
	const bool &enable_rs_debug()	const { return m_enable_rs_debug; }
	const bool &qos_sensor_data() 	const { return m_qos_sensor_data; }
	
	const float &min_depth() 	const { return m_min_depth; }
	const float &max_depth() 	const { return m_max_depth; }
	const float &depth_scale()	const { return m_depth_scale; }

	const std::array<int, 4> &roi() const { return m_roi; }

	const Mode &mode() const { return m_mode; }

	const int &qos_history_depth() 	const { return m_qos_history_depth; }
	const int &log_count_max() 		const { return m_log_count_max; }

	const std::string &camera_serial_no() 		   const { return m_camera_serial_no; }
 	const std::string &node_namespace() 		   const { return m_node_namespace; }
	const std::string &topic_fps() 				   const { return m_topic_fps; }
	const std::string &topic_duration() 		   const { return m_topic_duration; }
	const std::string &topic_latency() 			   const { return m_topic_latency; }
	const std::string &topic_color() 		       const { return m_topic_color; }
	const std::string &topic_color_small() 		   const { return m_topic_color_small; }
	const std::string &topic_depth() 			   const { return m_topic_depth; }
	const std::string &topic_frameset() 		   const { return m_topic_frameset; }
	const std::string &color_image_filename() 	   const { return m_color_image_filename; }
	const std::string &depth_image_filename()      const { return m_depth_image_filename; }
	const std::string &color_intrinsics_filename() const { return m_color_intrinsics_filename; }
	const std::string &depth_intrinsics_filename() const { return m_depth_intrinsics_filename; }
	const int &smallImage_width() 				   const { return m_smallImage_width; }
	const int &smallImage_height() 				   const { return m_smallImage_height; }

	void setProfiling(const bool &profiling) { m_profiling = profiling; }
	void setVerbosity(const bool &verbose) { m_verbose = verbose; }
	void setDepthScale(const float &depth_scale) { m_depth_scale = depth_scale; }
	void smallImage_height(const int &smallImage_height) { m_smallImage_height = smallImage_height;	}
	void smallImage_width(const int &smallImage_width) { m_smallImage_width = smallImage_width; }

private:
	rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

private:
	rclcpp::Node *m_node;
	rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_parameters_callback_handle = nullptr;
	std::function<void(std::string, float)> m_realsenseParameterCallback;

	bool m_debug					= false;
	bool m_verbose					= false;
	bool m_publish_fps				= false;
	bool m_publish_duration			= false;
	bool m_publish_latency			= false;
	bool m_profiling				= false;
	bool m_log_to_file				= false;
	bool m_print_statistics			= false;
	bool m_qos						= false;
	int m_log_count_max				= 400;
	Mode m_mode						= PRIMARY;
	float m_min_depth				= 0.5f;
	float m_max_depth				= 2.0f;
	float m_depth_scale				= 0.001f;
	std::string m_camera_serial_no 	= "";
	std::array<int, 4> m_roi		= { -1, -1, -1, -1 };
	bool m_enable_rs_debug			= false;
	bool m_qos_sensor_data			= false;
	int m_qos_history_depth			= 2;
	int m_smallImage_width			= 608;
	int m_smallImage_height			= 608;
	int m_camera_rotation 			= 0;

	std::string m_node_namespace            = "";
	std::string m_topic_fps                 = "profiling/fps";
	std::string m_topic_duration            = "profiling/duration";
	std::string m_topic_latency             = "profiling/latency";
	std::string m_topic_color               = "color/image";
	std::string m_topic_color_small         = "color/image_small";
	std::string m_topic_depth               = "depth/image";
	std::string m_topic_frameset            = "frameset";
	std::string m_color_image_filename      = "";
	std::string m_depth_image_filename      = "";
	std::string m_color_intrinsics_filename = "";
	std::string m_depth_intrinsics_filename = "";
};
