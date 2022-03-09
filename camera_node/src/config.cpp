// SYSTEM
#include <experimental/filesystem>
// ROS2
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
// PROJECT
#include "config.hpp"

namespace fs = std::experimental::filesystem;

/**
 * @brief Constructor.
 * @param node Running ros node
 */
Config::Config(rclcpp::Node* node) {
	this->node = node;
}

/**
 * @brief Destructor.
 */
Config::~Config() {}

/**
 * @brief Declare DepthFusion ros node parameters.
 */
void Config::declareNodeParameters() {
	parameters_callback_handle =
	    node->add_on_set_parameters_callback(std::bind(&Config::parametersCallback, this, std::placeholders::_1));

	// Parameters
	node->declare_parameter("mode", "primary");
	node->declare_parameter("verbose", false);
	node->declare_parameter("qos_sensor_data", false);
	node->declare_parameter("qos_history_depth", 2);
	node->declare_parameter("min_depth", 0.5);
	node->declare_parameter("max_depth", 2.0);
	node->declare_parameter("depth_scale", 0.0001);
	node->declare_parameter("camera_serial_no", "");
	node->declare_parameter("roi", std::vector<long>({-1, -1, -1, -1}));
	node->declare_parameter("debug.enable_debug", false);
	node->declare_parameter("debug.enable_rs_debug", false);
	node->declare_parameter("debug.save_data", false);
	node->declare_parameter("simulation.simulate_images", false);
	node->declare_parameter("simulation.color_image_filename", "not_set");
	node->declare_parameter("simulation.depth_image_filename", "not_set");
	node->declare_parameter("simulation.color_intrinsics_filename", "not_set");
	node->declare_parameter("simulation.depth_intrinsics_filename", "not_set");
	node->declare_parameter("simulation.depth_scale", 0.00025);
	node->declare_parameter("simulation.framerate", 30);

	node->get_parameter("camera_serial_no", camera_serial_no);
}

/**
 * @brief Called when ros parameter is changed during runtime.
 * @param parameters Changed ros parameters
 * @return Success of changing parameters
 */
rcl_interfaces::msg::SetParametersResult Config::parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {
	for (const auto& param : parameters) {
		std::string parameter_string = param.get_name();

		// Tokenize parameter string with '.' as delimeter
		std::vector<std::string> parameter_string_tokens;
		size_t pos = 0;
		while (pos != std::string::npos) {
			size_t next_pos = parameter_string.find('.', pos);
			if (next_pos != std::string::npos) {
				parameter_string_tokens.push_back(parameter_string.substr(pos, next_pos - pos));
				pos = next_pos + 1;
			} else {
				parameter_string_tokens.push_back(parameter_string.substr(pos, std::string::npos));
				pos = std::string::npos;
			}
		}
		std::string parameter_name = parameter_string_tokens.back();

		// Set node parameters from yaml config
		if (parameter_string_tokens.size() == 1) {
			if (parameter_name == "mode") {
				std::string param_string = param.as_string();
				if (param_string == "primary") {
					this->mode = PRIMARY;
				} else if (param_string == "secondary") {
					this->mode = SECONDARY;
				} else if (param_string == "registration") {
					this->mode = REGISTRATION;
				}
			} else if (parameter_name == "verbose") {
				this->verbose = param.as_bool();
			} else if (parameter_name == "qos_sensor_data") {
				this->qos_sensor_data = param.as_bool();
			} else if (parameter_name == "qos_history_depth") {
				this->qos_history_depth = static_cast<int>(param.as_int());
			} else if (parameter_name == "min_depth") {
				this->min_depth = static_cast<float>(param.as_double());
			} else if (parameter_name == "max_depth") {
				this->max_depth = static_cast<float>(param.as_double());
			} else if (parameter_name == "depth_scale") {
				this->depth_scale = static_cast<float>(param.as_double());
			} else if (parameter_name == "camera_serial_no") {
				this->camera_serial_no = param.as_string();
			} else if (parameter_name == "roi") {
				std::vector<long> roi_vec = param.as_integer_array();
				if (roi_vec.size() == 4) {
					this->roi = {static_cast<int>(roi_vec[0]), static_cast<int>(roi_vec[1]), static_cast<int>(roi_vec[2]),
					             static_cast<int>(roi_vec[3])};
				} else
					this->roi = {-1, -1, -1, -1};
			}
		} else if (parameter_string_tokens.size() == 2) {
			if (parameter_string_tokens[0] == "debug") {
				if (parameter_name == "enable_debug") {
					this->debug = param.as_bool();
				} else if (parameter_name == "enable_rs_debug") {
					this->enable_rs_debug = param.as_bool();
				} else if (parameter_name == "save_data") {
					this->save_data = param.as_bool();
				}
			} else if (parameter_string_tokens[0] == "profiling") {
				if (parameter_name == "enable_profiling") {
					this->profiling = param.as_bool();
				} else if (parameter_name == "log_to_file") {
					this->log_to_file = param.as_bool();
				} else if (parameter_name == "publish_fps") {
					this->publish_fps = param.as_bool();
				} else if (parameter_name == "publish_duration") {
					this->publish_duration = param.as_bool();
				} else if (parameter_name == "publish_latency") {
					this->publish_latency = param.as_bool();
				}
			} else if (parameter_string_tokens[0] == "simulation") {
				if (parameter_name == "simulate_images") {
					this->simulation = param.as_bool();
				} else if (parameter_name == "color_image_filename") {
					this->color_image_filename = fs::absolute(param.as_string());
				} else if (parameter_name == "depth_image_filename") {
					this->depth_image_filename = fs::absolute(param.as_string());
				} else if (parameter_name == "color_intrinsics_filename") {
					this->color_intrinsics_filename = fs::absolute(param.as_string());
				} else if (parameter_name == "depth_intrinsics_filename") {
					this->depth_intrinsics_filename = fs::absolute(param.as_string());
				} else if (parameter_name == "depth_scale") {
					this->sim_depth_scale = param.as_double();
				} else if (parameter_name == "framerate") {
					this->simulation_framerate = static_cast<unsigned>(param.as_int());
				}
			}
		}

		// Send parameters to realsense if in namespace "sensor"
		else if (parameter_string_tokens[0] == "sensor") {
			float param_float_value = 0.;
			switch (param.get_type()) {
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

			if (realsenseParameterCallback != nullptr) {
				realsenseParameterCallback(parameter_string, param_float_value);
			}
		}
	}

	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	result.reason = "success";
	return result;
}

/**
 * @brief Registers realsense parameter callback at ros node.
 * @param realsense_parameter_callback Callback function to set realsense option from ros parameter
 */
void Config::registerRealsenseParameterCallback(std::function<void(std::string, float)> realsense_parameter_callback) {
	realsenseParameterCallback = realsense_parameter_callback;
}
