#pragma once

// SYSTEM
#include <atomic>
#include <chrono>
#include <map>
#include <queue>
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
// OPENCV
#include <opencv2/opencv.hpp>
// LIBREALSENSE
#include <librealsense2/hpp/rs_types.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

/**
 * @brief Realsense camera module based on the librealsense2 sdk.
 */
class Realsense
{
	typedef std::chrono::high_resolution_clock::time_point time_point;
	typedef std::chrono::high_resolution_clock hires_clock;

public:
	Realsense();
	~Realsense();
	/**
	 * @brief Set exit request flat for exiting the program gracefully.
	 * @param pExit_request Exit request flag
	 */
	void setExitSignal(std::atomic<bool>* pExit_request)
	{
		m_pExit_request = pExit_request;
	}
	/**
	 * @brief Set console logging verbosity.
	 * @param verbose Flag for more detailed console logging
	 */
	void setVerbosity(const bool& verbose)
	{
		m_verbose = verbose;
	}
	/**
	 * @brief Set debug mode.
	 * @param debug Flag for debug profiling
	 */
	void setDebug(const bool& debug)
	{
		m_debug = debug;
	}
	/**
	 * @brief Checks if camera is stopped and program can exit.
	 * @return Is module clear to exit
	 */
	bool isexitClear() const
	{
		return m_exit_clear.load();
	}
	/**
	 * @brief Set scale factor of camera depth sensor.
	 * @param depth_scale Depth scale factor
	 */
	void setDepthScale(const float& depth_scale)
	{
		m_depth_scale = depth_scale;
	}
	/**
	 * @brief Set maximum depth value for camera depth sensor.
	 * @param depth_max Maximum depth value in meters
	 */
	void setDepthMax(const float& depth_max)
	{
		m_depth_max = depth_max;
	}
	
	void init(std::string camera_serial_no = "");
	void start();
	void stop();
	bool getFrames(uint8_t* color_frame, uint16_t* depth_frame, double& timestamp, unsigned timeout);
	rs2_intrinsics getDepthIntrinsics() const;
	rs2_intrinsics getColorIntrinsics() const;
	rs2_extrinsics getDepthToColorExtrinsics() const;
	void declareRosParameters(rclcpp::Node* node);
	void setOptionFromParameter(std::string parameter_string, float value);
	void getColorCameraInfo(sensor_msgs::msg::CameraInfo& camera_info) const;
	void getDepthCameraInfo(sensor_msgs::msg::CameraInfo& camera_info) const;

private:
	rs2::device getDevice(std::string serial_no = "");
	std::string convertToSnakeCase(const std::string& string_in);
	std::string rsOptionToString(rs2_option option);
	template<class T>
	void setSensorOption(rs2::sensor& sensor, const rs2_option& option, const T& value);
	void configureSensors(const std::vector<rs2::sensor>& sensors);
	char getOptionType(rs2::sensor& sensor, rs2_option& option) const;
	void intrinsics2CameraInfo(sensor_msgs::msg::CameraInfo& camera_info, const rs2_intrinsics& intrinsics) const;
	rs2::stream_profile getStreamProfile(rs2::sensor sensor, int w, int h, rs2_format format, int fps) const;
	void initPipeline(std::string camera_serial_no = "");
	void startPipeline();
	void stopPipeline();


private:
	std::atomic<bool>* m_pExit_request = nullptr;
	std::atomic<bool> m_exit_clear     = { false };
	bool m_debug                       = false;
	bool m_verbose                     = false;
	bool m_align                       = true;
	bool m_filter                      = true;
	long m_camera_time_base            = 0;
	long m_system_time_base            = 0;
	float m_depth_scale                = 0.0001f;
	float m_depth_max                  = 2.0f;

	rs2::stream_profile m_depth_stream;
	rs2::stream_profile m_color_stream;
	rs2::device m_device;
	std::vector<rs2::sensor> m_sensors;
	rs2::config m_rs_config;
	rs2::pipeline m_pipe;
	rs2::pipeline_profile m_pipe_profile;

	std::map<std::string, rs2_option> m_color_option_names;
	std::map<std::string, rs2_option> m_depth_option_names;
	rs2::frameset m_frameset;
	std::shared_ptr<rs2::filter> m_filter_align_to_color = nullptr;
	rs2_intrinsics m_color_intrinsics;
	rs2_intrinsics m_depth_intrinsics;
	double m_last_frame_timestamp;
	time_point m_timer              = hires_clock::now();
	std::queue<rs2::frameset> m_frameset_queue;
	std::thread m_capture_thread;
	rs2::frame_queue m_frame_queue;

	rs2::decimation_filter m_dec_filter;           // Decimation - reduces depth frame density
	rs2::threshold_filter m_thr_filter;            // Threshold  - removes values outside recommended range
	rs2::spatial_filter m_spat_filter;             // Spatial    - edge-preserving spatial smoothing
	rs2::temporal_filter m_temp_filter;            // Temporal   - reduces temporal noise
	rs2::disparity_transform m_depth_to_disparity; // Transform from depth to disparity
	rs2::disparity_transform m_disparity_to_depth; // Transform from disparity to depth
	rs2::hole_filling_filter m_hole_filter;

	std::function<void(const uint8_t*, const uint16_t*, const double)> m_framesCallback;
};
