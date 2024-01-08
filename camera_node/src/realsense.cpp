// SYSTEM
#include <algorithm>
#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <thread>
#include <typeinfo>
// PROJECT
#include "lodepng/lodepng.h"

#include "realsense.hpp"

using namespace std::chrono_literals;
namespace fs = std::experimental::filesystem;
using json   = nlohmann::json;

/**
 * @brief Constructor.
 */
Realsense::Realsense() :
	m_depth_stream(),
	m_color_stream(),
	m_device(),
	m_sensors(),
	m_rs_config(),
	m_pipe(),
	m_pipe_profile(),
	m_color_option_names(),
	m_depth_option_names(),
	m_frameset(),
	m_color_intrinsics(),
	m_depth_intrinsics(),
	m_last_frame_timestamp(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count()),
	m_frameset_queue(),
	m_capture_thread(),
	m_frame_queue(),
	m_dec_filter(),
	m_thr_filter(),
	m_spat_filter(),
	m_temp_filter(),
	m_depth_to_disparity(),
	m_disparity_to_depth(),
	m_framesCallback()
{
}

/**
 * @brief Destructor.
 */
Realsense::~Realsense() {}
/**
 * @brief Get realsense device by number.
 * @param serial_no Realsense serial number
 * @return Realsense device
 */
rs2::device Realsense::getDevice(std::string serial_no)
{
	rs2::context ctx;
	rs2::device_list devices = ctx.query_devices();

	rs2::device selected_device;
	if (devices.size() == 0)
	{
		std::cout << "+-- No RealSense camera found" << std::endl;
		m_pExit_request->store(true);
		return selected_device;
	}
	else if (serial_no == "")
	{
		selected_device = devices[0];
		if (m_verbose)
			std::cout << "+-- Found RealSense camera \"" << selected_device.get_info(RS2_CAMERA_INFO_NAME) << "\"" << std::endl;
	}
	else
	{
		if (m_verbose) std::cout << "+-- Found RealSense cameras" << std::endl;
		bool serial_no_found = false;
		for (unsigned i = 0; i < devices.size(); i++)
		{
			try{
				std::string device_serial_no = devices[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
				if (m_verbose)
					std::cout << "| \"" << devices[i].get_info(RS2_CAMERA_INFO_NAME) << "\" serial no: " << device_serial_no << std::endl;

				if (device_serial_no == serial_no)
				{
					selected_device = devices[i];
					serial_no_found = true;
				}
			}catch(rs2::error e) {
					std::cout << e.what()  << std::endl;

			}
		}

		if (!serial_no_found)
		{
			std::cout << "No Realsense camera with serial no: " << serial_no << " found" << std::endl;
			m_pExit_request->store(true);
		}
		else
		{
			if (m_verbose)
				std::cout << "| Using camera \"" << selected_device.get_info(RS2_CAMERA_INFO_NAME) << "\" serial no: " << serial_no << std::endl;
		}
	}

	return selected_device;
}

/**
 * @brief Configure realsense camera sensors.
 * @param sensors Realsense camera sensors
 */
void Realsense::configureSensors(const std::vector<rs2::sensor>& sensors)
{
	// RealSense L515 sensors:
	// 0: depth sensor (name: L500 Depth Sensor)
	// 1: color sensor (name: RGB Camera)
	// 2: acceleration sensor (name: Motion Module)
	rs2::depth_sensor depth_sensor = sensors[0].as<rs2::depth_sensor>();
	rs2::color_sensor color_sensor = sensors[1].as<rs2::color_sensor>();

	// Search for color and depth sensor
	bool color_sensor_found = false;
	bool depth_sensor_found = false;

	for (auto& sensor : sensors)
	{
		std::string sensor_rs_name      = sensor.get_info(RS2_CAMERA_INFO_NAME);
		std::size_t color_string_found  = sensor_rs_name.find("RGB");
		std::size_t depth_string_found  = sensor_rs_name.find("Depth");
		std::size_t stereo_string_found = sensor_rs_name.find("Stereo Module");

		if (color_string_found != std::string::npos)
		{
			color_sensor       = sensor.as<rs2::color_sensor>();
			color_sensor_found = true;
		}
		else if (depth_string_found != std::string::npos || stereo_string_found != std::string::npos)
		{
			depth_sensor       = sensor.as<rs2::depth_sensor>();
			depth_sensor_found = true;
		}
	}

	if (!color_sensor_found || !depth_sensor_found)
	{
		std::cout << "No realsense camera with color and depth sensor found!" << std::endl;
		m_pExit_request->store(true);
	}

	// Convert all realsense options to ros parameter strings
	std::vector<rs2_option> supported_color_options = color_sensor.get_supported_options();

	for (auto& option : supported_color_options)
	{
		if (color_sensor.is_option_read_only(option)) continue;
		std::string option_string           = rsOptionToString(option);
		m_color_option_names[option_string] = option;
		// std::cout << "color option: " << option << std::endl;
	}

	std::vector<rs2_option> supported_depth_options = depth_sensor.get_supported_options();

	for (auto& option : supported_depth_options)
	{
		if (depth_sensor.is_option_read_only(option)) continue;
		std::string option_string           = rsOptionToString(option);
		m_depth_option_names[option_string] = option;
		// std::cout << "depth option: " << option << std::endl;
	}
}

/**
 * @brief Initialize realsense camera.
 * @param camera_serial_no Choose camera to initialize by serial number if set
 */
void Realsense::init(std::string camera_serial_no)
{
	initPipeline(camera_serial_no);
}

/**
 * @brief Start realsense camera.
 */
void Realsense::start()
{
	startPipeline();
}

/**
 * @brief Stop realsense camera.
 */
void Realsense::stop()
{
	stopPipeline();
}

/**
 * @brief Initialize realsense camera pipeline.
 * @param camera_serial_no Choose camera to initialize by serial number if set
 */
void Realsense::initPipeline(std::string camera_serial_no)
{
	rs2_error* e = nullptr;
	if (m_verbose)
	{
		std::string rsVersion(std::to_string(rs2_get_api_version(&e)).insert(3, ".").insert(1, "."));
		std::cout << "+-- LibRealSense version" << std::endl;
		std::cout << "| Built with v" << RS2_API_VERSION_STR << std::endl;
		std::cout << "| Running with v" << rsVersion << std::endl;
	}

	// Setup realsense device
	rs2::device device = getDevice(camera_serial_no);

	if (m_pExit_request->load()) return;
	m_rs_config.enable_device(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

	// Setup streams
	m_rs_config.disable_all_streams();
	std::string rs_camera_name = device.get_info(RS2_CAMERA_INFO_NAME);

	if (rs_camera_name == "Intel RealSense L515")
	{
		m_rs_config.enable_stream(RS2_STREAM_DEPTH, 1024, 768, RS2_FORMAT_Z16, 30);
		m_rs_config.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
	}
	else if (rs_camera_name == "Intel RealSense D435I" || rs_camera_name == "Intel RealSense D435" || rs_camera_name == "Intel RealSense D415" || rs_camera_name == "Intel RealSense D455")
	{
		m_rs_config.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
		m_rs_config.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
		//m_rs_config.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RGB8, 30);
	}

	// Setup filters
	m_filter_align_to_color = std::make_shared<rs2::align>(RS2_STREAM_COLOR);
	m_depth_to_disparity    = rs2::disparity_transform(true);
	m_disparity_to_depth    = rs2::disparity_transform(false);
	m_thr_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.0);
	m_thr_filter.set_option(RS2_OPTION_MAX_DISTANCE, 2.0);
	m_dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
	//m_spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
	//m_spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5);
	//m_spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
	m_spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.55f);
	m_temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4f);
	m_hole_filter.set_option(RS2_OPTION_HOLES_FILL,1);

	// Build pipeline with configuration
	if (m_rs_config.can_resolve(m_pipe))
	{
		m_pipe_profile = m_rs_config.resolve(m_pipe);

		// Configure sensors
		configureSensors(m_pipe_profile.get_device().query_sensors());
	}
	else
	{
		std::cout << "Wrong realsense configuration" << std::endl;
		m_pExit_request->store(true);
	}

	m_device  = m_pipe_profile.get_device();
	m_sensors = m_pipe_profile.get_device().query_sensors();
}

/**
 * @brief Start realsense camera pipeline.
 */
void Realsense::startPipeline()
{
	std::cout << "Initializing camera" << std::endl;

	// Set advanced parameters
	if (std::string(m_pipe_profile.get_device().get_info(RS2_CAMERA_INFO_PRODUCT_LINE)) == "D400")
	{
		//if (m_pipe_profile.get_device().is<rs400::advanced_mode>())
		//{
			//rs2::depth_sensor depth_sensor = m_pipe_profile.get_device().query_sensors()[0].as<rs2::depth_sensor>();
			//depth_sensor.set_option(RS2_OPTION_VISUAL_PRESET, 2);

			//rs400::advanced_mode advanced_device(m_pipe_profile.get_device());

			// Set depth units
			//auto depth_table       = advanced_device.get_depth_table();
			//uint32_t depth_units   = static_cast<uint32_t>(m_depth_scale * 1e6f);
			//depth_table.depthUnits = depth_units;

			// Set maximal depth
			//depth_table.depthClampMax = static_cast<int32_t>(m_depth_max / m_depth_scale);
			//advanced_device.set_depth_table(depth_table);
		//}
		//else
		//{
		//	std::cout << "Advanced mode not supported" << std::endl;
		//	m_pExit_request->store(true);
		//}
	}

	// Start realsense pipeline
	m_pipe_profile = m_pipe.start(m_rs_config);

	// Wait for streams to settle
	if (m_verbose) std::cout << "Waiting for streams " << std::flush;

	while (!m_pipe.try_wait_for_frames(&m_frameset, 40))
	{
		std::this_thread::sleep_for(40ms);
		if (m_verbose) std::cout << "." << std::flush;
	}

	if (m_verbose) std::cout << " done" << std::endl;

	// Set base time points for system independent camera clock
	if (m_frameset.get_frame_timestamp_domain() == RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK)
	{
		m_system_time_base = static_cast<long>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count());
		m_camera_time_base = static_cast<long>(m_frameset.get_timestamp() * 1e6);
	}

	if (m_verbose)
	{
		std::cout << "+-- Timestamp domains:" << std::endl;
		std::cout << "| Depth sensor: " << m_frameset.get_depth_frame().get_frame_timestamp_domain() << std::endl;
		std::cout << "| Color sensor: " << m_frameset.get_color_frame().get_frame_timestamp_domain() << std::endl;

		if (m_frameset.get_frame_timestamp_domain() == RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK)
		{
			std::cout << "| System base time: " << m_system_time_base << std::endl;
			std::cout << "| Camera base time: " << m_camera_time_base << std::endl;
		}
	}
	std::cout << "+==========[ Camera Started ]==========+" << std::endl;
}

/**
 * @brief Stop realsense camera pipeline.
 */
void Realsense::stopPipeline()
{
	if (m_capture_thread.joinable())
		m_capture_thread.join();

	m_pipe.stop();
	std::cout << "+==========[ Camera Stopped ]==========+" << std::endl;
	m_exit_clear = true;
}

/**
 * @brief Fetch color and depth frame from realsense camera.
 * @param color_frame 8bit RGB Color frame
 * @param depth_frame 16bit grayscale Depth frame
 * @param timestamp Timestamp of the first captured frame 
 * @param timeout Time to wait for frames in milliseconds
 * @return True if frames arrived before timeout, false otherwise
 */
bool Realsense::getFrames(uint8_t* color_frame, uint16_t* depth_frame, double& timestamp, unsigned timeout)
{
	rs2::frameset frameset;
	// Get camera frames
	bool frames_available = false;

	try {

		frames_available = m_pipe.try_wait_for_frames(&frameset, timeout);

		if (frames_available) {
		
			// Align depth frame to color frame
			if (m_align)
			{
				frameset = m_filter_align_to_color->process(frameset);
			}

			// Filter depth frame
			if (m_filter)
			{
				//rs2::depth_frame filtered;
				rs2::depth_frame filtered = frameset.get_depth_frame();
				/*
				The implemented flow of the filters pipeline is in the following order:
				1. apply decimation filter
				2. transform the scene into disparity domain
				3. apply spatial filter
				4. apply temporal filter
				5. revert the results back (if step Disparity filter was applied
				6. apply threshold filter
				to depth domain (each post processing block is optional and can be applied independantly).
				*/
			
				//filtered = m_dec_filter.process(filtered);

				//auto filtered_dispa = m_depth_to_disparity.process(filtered); 	// Depth to disparity
				//filtered = m_spat_filter.process(filtered);			// Spatial filter (has long processing time on big images..)
				rs2::depth_frame filtered_temp = m_temp_filter.process(filtered);			// Temporal filter
				//filtered = m_disparity_to_depth.process(filtered_temp); 	// Disparity to depth
				//filtered = m_hole_filter.process(filtered);

				// copy the full range image to depth_frame image
				//std::memcpy(reinterpret_cast<void*>(depth_frame), filtered_color_image.data, frameset.get_data_size());
				std::memcpy(reinterpret_cast<void*>(depth_frame), filtered_temp.get_data(), filtered_temp.get_data_size());

			} else {
				std::memcpy(reinterpret_cast<void*>(depth_frame), frameset.get_depth_frame().get_data(), frameset.get_depth_frame().get_data_size());
			}

			// Get frames from frameset
			std::memcpy(reinterpret_cast<void*>(color_frame), frameset.get_color_frame().get_data(), frameset.get_color_frame().get_data_size());
	
			// RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK 	Frame timestamp was measured in relation to the camera clock
			// RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME     Frame timestamp was measured in relation to the OS system clock
			// RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME     Frame timestamp was measured in relation to the camera clock and converted
			//   to OS system clock by constantly measure the difference
			// RS2_TIMESTAMP_DOMAIN_COUNT           Number of enumeration values. Not a valid input: intended to be used in
			//   for-loops.

			// - timestamp mesasured in milliseconds
			// - color frame is taken before depth frame (approx 7 ms)
			// - frameset has timestamp of color frame

			if (frameset.get_frame_timestamp_domain() == RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK)
			{
				double elapsed_camera_time = (frameset.get_timestamp() * 1e6) - m_camera_time_base;
				timestamp                  = (m_system_time_base + elapsed_camera_time) / 1e6;
			}
			else
				timestamp = frameset.get_timestamp();

			return true;
		}
		else
		{
			//depth_frame = nullptr;
			//color_frame = nullptr;
			return false;
		}
	} catch(...) {
		std::cout << "realsense loop failed!!" << std::endl;
		return false;
	}
}

/**
 * @brief Set configuration option for realsense sensor.
 * @param sensor Realsense sensor
 * @param option Configuration option
 * @param value New value for configuration option
 */
template<class T>
void Realsense::setSensorOption(rs2::sensor& sensor, const rs2_option& option, const T& value)
{
	/*try
	{
		if (sensor.supports(option) && !sensor.is_option_read_only(option))
			sensor.set_option(option, value);
		else
			std::cout << "Option " << option << " not supported by " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
	}
	catch (const rs2::invalid_value_error& e)
	{
		std::cout << "RealSense option " << option << " received invalid value " << value << std::endl;
	}
	catch (const rs2::error& e)
	{
		std::cout << "RealSense error" << std::endl;
		m_pExit_request->store(true);
	} */
}

/**
 * @brief Set realsense option from ros parameter.
 * @param parameter_string Full parameter string with namespaces
 * @param value New option value
 */
void Realsense::setOptionFromParameter(std::string parameter_string, float value)
{
	// Tokenize parameter string with '.' as delimeter
	std::vector<size_t> delimeter_positions;
	std::vector<std::string> parameter_string_tokens;
	size_t pos = 0;
	while (pos != std::string::npos)
	{
		size_t next_pos = parameter_string.find('.', pos);
		if (next_pos != std::string::npos)
		{
			delimeter_positions.push_back(next_pos);
			parameter_string_tokens.push_back(parameter_string.substr(pos, next_pos - pos));
			pos = next_pos + 1;
		}
		else
		{
			parameter_string_tokens.push_back(parameter_string.substr(pos, std::string::npos));
			pos = std::string::npos;
		}
	}

	// Check realsense parameter namespace sanity
	bool namespace_valid = true;
	if (parameter_string_tokens.size() != 3)
		namespace_valid = false;
	else
	{
		if (parameter_string_tokens[0] != "sensor")
			namespace_valid = false;
		else if (parameter_string_tokens[1] != "color" && parameter_string_tokens[1] != "depth")
			namespace_valid = false;
	}
	if (!namespace_valid)
	{
		std::string parameter_namespace = parameter_string.substr(0, delimeter_positions.back());
		std::cout << "Wrong parameter namespace for realsense. Should be \"sensor.color\" or \"sensor.depth\" but is \"" << parameter_namespace << "\"" << std::endl;
		return;
	}
 
	// Set realsense parameter
	std::vector<rs2::sensor> sensors = m_pipe_profile.get_device().query_sensors();
	std::string option_name          = parameter_string_tokens.back();
	bool option_valid                = true;
	if (parameter_string_tokens[1] == "color")
	{
		rs2::color_sensor color_sensor = sensors[1].as<rs2::color_sensor>();
		auto option                    = m_color_option_names.find(option_name);
		if (option != m_color_option_names.end())
		{
			bool set_option_override = false;

			// Option enable_auto_exposure gets disabled if option exposure is set
			if (option->second == RS2_OPTION_EXPOSURE)
			{
				if (static_cast<bool>(color_sensor.get_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE)))
				{
					set_option_override = true;
					// std::cout << "Disable auto exposure to set exposure value!" << std::endl;
				}
			}
			// Option enable_auto_white_balance gets disabled if option white_balance is set
			else if (option->second == RS2_OPTION_WHITE_BALANCE)
			{
				if (static_cast<bool>(color_sensor.get_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE)))
				{
					set_option_override = true;
					// std::cout << "Disable auto white balance to set white balance value!" << std::endl;
				}
			}
			// Errors for gain ranges
			else if (option->second == RS2_OPTION_GAIN)
				set_option_override = true;

			if (!set_option_override)
			{
				//if (m_verbose) std::cout << "| set " << parameter_string << " to: " << value << std::endl;
				//setSensorOption(color_sensor, option->second, value);
			}
		}
		else
			option_valid = false;
	}
	else if (parameter_string_tokens[1] == "depth")
	{
		rs2::depth_sensor depth_sensor = sensors[0].as<rs2::depth_sensor>();
		auto option                    = m_depth_option_names.find(option_name);
		if (option != m_depth_option_names.end())
		{
			//if (m_verbose) std::cout << "| set " << parameter_string << " to: " << value << std::endl;
			//setSensorOption(depth_sensor, option->second, value);
		}
		else
			option_valid = false;
	}

	if (!option_valid)
		std::cout << "Realsense option " << option_name << " not a declared parameter!" << std::endl;
	
}

/**
 * @brief Convert string to lowercase replacing whitespaces with underscores.
 * @param string_in String to convert
 * @return Lowercase string with underscores instead of whitespaces
 */
std::string Realsense::convertToSnakeCase(const std::string& string_in)
{
	std::string string_out = string_in;
	std::replace(string_out.begin(), string_out.end(), ' ', '_');
	std::locale loc;
	for (char& c : string_out)
		c = std::tolower(c, loc);

	return string_out;
}

/**
 * @brief Convert realsense option to lowercase string replacing whitespaces with underscores.
 * @param option Realsense option
 * @return Lowercase string with underscores instead of whitespaces
 */
std::string Realsense::rsOptionToString(rs2_option option)
{
	return convertToSnakeCase(rs2_option_to_string(option));
}

/**
 * @brief Declare ros parameters for sensor options
 */
void Realsense::declareRosParameters(rclcpp::Node* node)
{
	if (m_debug) std::cout << "+-- Set realsense parameters" << std::endl;
	std::vector<rs2::sensor> sensors = m_pipe_profile.get_device().query_sensors();
	// Declare color sensor parameters
	rs2::color_sensor color_sensor = sensors[1].as<rs2::color_sensor>();
	for (auto& option : m_color_option_names)
	{
		std::string ros_parameter = "sensor.color." + option.first;
		char type_char            = getOptionType(color_sensor, option.second);
		// std::cout << "declare parameter " << ros_parameter << " from option \"" << option.second << "\" with type "
		//           << type_char << std::endl;
		if (type_char == 'b')
			node->declare_parameter(ros_parameter, static_cast<bool>(color_sensor.get_option(option.second)));
		else if (type_char == 'i')
			node->declare_parameter(ros_parameter, static_cast<int>(color_sensor.get_option(option.second)));
		else if (type_char == 'f')
			node->declare_parameter(ros_parameter, color_sensor.get_option(option.second));
	}
	// Declare depth sensor parameters
	rs2::depth_sensor depth_sensor = sensors[0].as<rs2::depth_sensor>();
	for (auto& option : m_depth_option_names)
	{
		std::string ros_parameter = "sensor.depth." + option.first;
		char type_char            = getOptionType(depth_sensor, option.second);
		// std::cout << "declare parameter " << ros_parameter << " from option \"" << option.second << "\" with type "
		//           << type_char << std::endl;
		if (type_char == 'b')
			node->declare_parameter(ros_parameter, static_cast<bool>(depth_sensor.get_option(option.second)));
		else if (type_char == 'i')
			node->declare_parameter(ros_parameter, static_cast<int>(depth_sensor.get_option(option.second)));
		else if (type_char == 'f')
			node->declare_parameter(ros_parameter, depth_sensor.get_option(option.second));
	}
}

/**
 * @brief Get value type from realsense option
 * @param sensor Realsense sensor for option
 * @param option Realsense option
 * @return Character for type: bool='b', int='i', float='f'
 */
char Realsense::getOptionType(rs2::sensor& sensor, rs2_option& option) const
{
	rs2::option_range option_range = sensor.get_option_range(option);
	char type_char                 = 'n';
	if (option_range.step == 1.f)
	{
		if (option_range.min == 0.f && option_range.max == 1.f)
			type_char = 'b';
		else
			type_char = 'i';
	}
	else
		type_char = 'f';
	return type_char;
}

/**
 * @brief Get intrinsics of depth sensor.
 * @return Depth sensor intrinsics
 */
rs2_intrinsics Realsense::getDepthIntrinsics() const
{
	if (m_align)
	{
		return m_pipe_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
	}
	else
	{
		return m_pipe_profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
	}
}

/**
 * @brief Get intrinsics of color sensor.
 * @return Color sensor intrinsics
 */
rs2_intrinsics Realsense::getColorIntrinsics() const
{
	return m_pipe_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
}

/**
 * @brief Get depth sensor to color sensor extrinsics.
 * @return Depth sensor to color sensor extrinsics
 */
rs2_extrinsics Realsense::getDepthToColorExtrinsics() const
{
	return m_pipe_profile.get_stream(RS2_STREAM_DEPTH).get_extrinsics_to(m_pipe_profile.get_stream(RS2_STREAM_COLOR));
}

/**
 * @brief Get ros camera info message for color stream.
 * @param camera_info Ros camera info message
 */
void Realsense::getColorCameraInfo(sensor_msgs::msg::CameraInfo& camera_info) const
{
	rs2_intrinsics intrinsics;
	rs2::video_stream_profile stream_profile;

	stream_profile = m_pipe_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

	intrinsics = stream_profile.get_intrinsics();
	intrinsics2CameraInfo(camera_info, intrinsics);
}

/**
 * @brief Get ros camera info message for depth stream.
 * @param camera_info Ros camera info message
 */
void Realsense::getDepthCameraInfo(sensor_msgs::msg::CameraInfo& camera_info) const
{
	rs2_intrinsics intrinsics;

	rs2::video_stream_profile stream_profile;

	stream_profile = m_pipe_profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
	
	//if (m_align) {
	//	stream_profile = m_pipe_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
	//}
	
	intrinsics = stream_profile.get_intrinsics();

	intrinsics2CameraInfo(camera_info, intrinsics);
}

/**
 * @brief Convert realsense intrinsics to ros camera info message.
 * @param camera_info Ros camera info message.
 * @param intrinsics Realsense instrinsics
 */
void Realsense::intrinsics2CameraInfo(sensor_msgs::msg::CameraInfo& camera_info, const rs2_intrinsics& intrinsics) const
{
	camera_info.width           = static_cast<unsigned>(intrinsics.width);
	camera_info.height          = static_cast<unsigned>(intrinsics.height);
	camera_info.header.frame_id = "optical_frame_id";

	// Intrinsic camera matrix for the raw (distorted) images
	//     [fx  0 cx]
	// K = [ 0 fy cy]
	//     [ 0  0  1]
	camera_info.k.at(0) = static_cast<double>(intrinsics.fx); // Fx
	camera_info.k.at(1) = 0;
	camera_info.k.at(2) = static_cast<double>(intrinsics.ppx); // Cx
	camera_info.k.at(3) = 0;
	camera_info.k.at(4) = static_cast<double>(intrinsics.fy);  // Fy
	camera_info.k.at(5) = static_cast<double>(intrinsics.ppy); // Cy
	camera_info.k.at(6) = 0;
	camera_info.k.at(7) = 0;
	camera_info.k.at(8) = 1;

	// Projection/camera matrix
	//     [fx'  0  cx' Tx]
	// P = [ 0  fy' cy' Ty]
	//     [ 0   0   1   0]
	camera_info.p.at(0)  = camera_info.k.at(0);
	camera_info.p.at(1)  = 0;
	camera_info.p.at(2)  = camera_info.k.at(2);
	camera_info.p.at(3)  = 0; // Tx for stereo camera
	camera_info.p.at(4)  = 0;
	camera_info.p.at(5)  = camera_info.k.at(4);
	camera_info.p.at(6)  = camera_info.k.at(5);
	camera_info.p.at(7)  = 0; // Ty for stereo camera
	camera_info.p.at(8)  = 0;
	camera_info.p.at(9)  = 0;
	camera_info.p.at(10) = 1;
	camera_info.p.at(11) = 0;

	// Rectification matrix (stereo cameras only)
	// A rotation matrix aligning the camera coordinate system to the ideal
	// stereo image plane so that epipolar lines in both stereo images are
	// parallel.
	camera_info.r.at(0) = 1.0;
	camera_info.r.at(1) = 0.0;
	camera_info.r.at(2) = 0.0;
	camera_info.r.at(3) = 0.0;
	camera_info.r.at(4) = 1.0;
	camera_info.r.at(5) = 0.0;
	camera_info.r.at(6) = 0.0;
	camera_info.r.at(7) = 0.0;
	camera_info.r.at(8) = 1.0;

	// Distortion model
	if (intrinsics.model == RS2_DISTORTION_KANNALA_BRANDT4)
		camera_info.distortion_model = "equidistant";
	else
		camera_info.distortion_model = "plumb_bob";

	// The distortion parameters, size depending on the distortion model.
	// For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
	camera_info.d.resize(5);
	for (unsigned long i = 0; i < 5; i++)
		camera_info.d.at(i) = static_cast<double>(intrinsics.coeffs[i]);
}

/**
 * @brief Get stream profile from realsense sensor.
 * @param sensor Realsense sensor
 * @param width Width of the stream
 * @param height Height of the stream
 * @param format Stream format
 * @param fps Stream frames per second
 * @return Stream profile matching given parameters
 */
rs2::stream_profile Realsense::getStreamProfile(rs2::sensor sensor, int width, int height, rs2_format format, int fps) const
{
	rs2::stream_profile stream_profile;
	for (auto profile : sensor.get_stream_profiles())
	{
		if (profile.as<rs2::video_stream_profile>().width() == width && profile.as<rs2::video_stream_profile>().height() == height && profile.fps() == fps && profile.format() == format)
			stream_profile = profile;
	}
	return stream_profile;
}