// SYSTEM
#include <experimental/filesystem>
// ROS2
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
// PROJECT
#include "config.hpp"

namespace fs = std::experimental::filesystem;

/**
 * @brief Constructor.
 * @param node Running ros node
 */
Config::Config(rclcpp::Node* node) :
	m_node(node)
{
}

/**
 * @brief Destructor.
 */
Config::~Config() {}

/**
 * @brief Declare DepthFusion ros node parameters.
 */
void Config::declareNodeParameters()
{
	m_parameters_callback_handle = m_node->add_on_set_parameters_callback(std::bind(&Config::parametersCallback, this, std::placeholders::_1));

	// Parameters
	m_node->declare_parameter("mode", "primary");
	m_node->declare_parameter("verbose", false);
	m_node->declare_parameter("qos_sensor_data", false);
	m_node->declare_parameter("qos_history_depth", 2);
	m_node->declare_parameter("min_depth", 0.0);
	m_node->declare_parameter("max_depth", 50.0);
	m_node->declare_parameter("depth_scale", 0.001);
	m_node->declare_parameter("roi", std::vector<long>({ -1, -1, -1, -1 }));
	m_node->declare_parameter("debug.enable_debug", false);
	m_node->declare_parameter("debug.enable_rs_debug", false);
	m_node->declare_parameter("camera_serial_no", "");
	m_node->declare_parameter("inter_cam_sync_mode", 0);

}

/**
 * @brief Called when ros parameter is changed during runtime.
 * @param parameters Changed ros parameters
 * @return Success of changing parameters
 */
rcl_interfaces::msg::SetParametersResult Config::parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
{
	for (const auto& param : parameters)
	{
		std::string parameter_string = param.get_name();

		//std::cout << param << std::endl;

		// Tokenize parameter string with '.' as delimeter
		std::vector<std::string> parameter_string_tokens;
		size_t pos = 0;
		while (pos != std::string::npos)
		{
			size_t next_pos = parameter_string.find('.', pos);
			if (next_pos != std::string::npos)
			{
				parameter_string_tokens.push_back(parameter_string.substr(pos, next_pos - pos));
				pos = next_pos + 1;
			}
			else
			{
				parameter_string_tokens.push_back(parameter_string.substr(pos, std::string::npos));
				pos = std::string::npos;
			}
		}
		std::string parameter_name = parameter_string_tokens.back();

		// Set node parameters from yaml config
		if (parameter_string_tokens.size() == 1)
		{
			if (parameter_name == "mode")
			{
				std::string param_string = param.as_string();
				if (param_string == "primary")
					m_mode = PRIMARY;
				else if (param_string == "secondary")
					m_mode = SECONDARY;
				else if (param_string == "registration")
					m_mode = REGISTRATION;
			}
			else if (parameter_name == "verbose")
				m_verbose = param.as_bool();
			else if (parameter_name == "qos_sensor_data")
				m_qos_sensor_data = param.as_bool();
			else if (parameter_name == "qos_history_depth")
				m_qos_history_depth = static_cast<int>(param.as_int());
			else if (parameter_name == "min_depth")
				m_min_depth = static_cast<float>(param.as_double());
			else if (parameter_name == "max_depth")
				m_max_depth = static_cast<float>(param.as_double());
			else if (parameter_name == "depth_scale")
				m_depth_scale = static_cast<float>(param.as_double());
			else if (parameter_name == "camera_serial_no")
				m_camera_serial_no = param.as_string();
			else if (parameter_name == "roi")
			{
				std::vector<long> roi_vec = param.as_integer_array();
				if (roi_vec.size() == 4)
					m_roi = { static_cast<int>(roi_vec[0]), static_cast<int>(roi_vec[1]), static_cast<int>(roi_vec[2]), static_cast<int>(roi_vec[3]) };
				else
					m_roi = { -1, -1, -1, -1 };
			}
		}
		else if (parameter_string_tokens.size() == 2)
		{
			if (parameter_string_tokens[0] == "debug")
			{
				if (parameter_name == "enable_debug")
					m_debug = param.as_bool();
				else if (parameter_name == "enable_rs_debug")
					m_enable_rs_debug = param.as_bool();
			}
			else if (parameter_string_tokens[0] == "profiling")
			{
				if (parameter_name == "enable_profiling")
					m_profiling = param.as_bool();
				else if (parameter_name == "log_to_file")
					m_log_to_file = param.as_bool();
				else if (parameter_name == "publish_fps")
					m_publish_fps = param.as_bool();
				else if (parameter_name == "publish_duration")
					m_publish_duration = param.as_bool();
				else if (parameter_name == "publish_latency")
					m_publish_latency = param.as_bool();
			}
		}

		// Send parameters to realsense if in namespace "sensor"
		else if (parameter_string_tokens[0] == "sensor")
		{
			float param_float_value = 0.;
			switch (param.get_type())
			{
				case rclcpp::ParameterType::PARAMETER_BOOL:
					param_float_value = static_cast<float>(param.as_bool());
					break;
				case rclcpp::ParameterType::PARAMETER_INTEGER:
					param_float_value = static_cast<float>(param.as_int());
					break;
				case rclcpp::ParameterType::PARAMETER_DOUBLE:
					param_float_value = static_cast<float>(param.as_double());
					break;
				default:
					std::cout << "Config: Unknown realsense parameter type" << std::endl;
					break;
			}

			//if (m_realsenseParameterCallback != nullptr)
				//m_realsenseParameterCallback(parameter_string, param_float_value);
		}
	}

	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	result.reason     = "success";
	return result;
}

/**
 * @brief Registers realsense parameter callback at ros node.
 * @param realsense_parameter_callback Callback function to set realsense option from ros parameter
 */
void Config::registerRealsenseParameterCallback(std::function<void(std::string, float)> realsense_parameter_callback)
{
	//m_realsenseParameterCallback = realsense_parameter_callback;
}
