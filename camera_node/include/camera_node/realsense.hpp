#pragma once

// SYSTEM
#include <atomic>
#include <chrono>
#include <map>
#include <queue>
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
// LIBREALSENSE
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_types.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

/**
 * @brief Realsense camera module based on the librealsense2 sdk.
 */
class Realsense {
	typedef std::chrono::high_resolution_clock::time_point time_point;
	typedef std::chrono::high_resolution_clock hires_clock;

 public:
	Realsense();
	~Realsense();
	/**
	 * @brief Set exit request flat for exiting the program gracefully.
	 * @param exit_request Exit request flag
	 */
	void setExitSignal(std::atomic<bool>* exit_request) { this->exit_request = exit_request; }
	/**
	 * @brief Set console logging verbosity.
	 * @param verbose Flag for more detailed console logging
	 */
	void setVerbosity(bool verbose) { this->verbose = verbose; }
	/**
	 * @brief Set debug mode.
	 * @param debug Flag for debug profiling
	 */
	void setDebug(bool debug) { this->debug = debug; }
	/**
	 * @brief Checks if camera is stopped and program can exit.
	 * @return Is module clear to exit
	 */
	bool isexitClear() { return this->exit_clear.load(); }
	/**
	 * @brief Set simulation mode for using static images instead of camera input.
	 * @param simulation Flag for simulation mode
	 */
	void setSimulation(bool simulation) { this->simulation = simulation; }
	/**
	 * @brief Set scale factor of camera depth sensor.
	 * @param depth_scale Depth scale factor
	 */
	void setDepthScale(float depth_scale) { this->depth_scale = depth_scale; }
	/**
	 * @brief Set maximum depth value for camera depth sensor.
	 * @param depth_max Maximum depth value in meters
	 */
	void setDepthMax(float depth_max) { this->depth_max = depth_max; }
	/**
	 * @brief Set framerate for camera simulation.
	 * @param simulation_framerate Framerate for simulation in frames per second
	 */
	void setSimulationFramerate(unsigned simulation_framerate) { this->simulation_framerate = simulation_framerate; }
	/**
	 * @brief Set align depth to color frame.
	 * @param align True if depth frame should be aligned to color frame
	 */
	void setAlign(bool align) { this->align = align; }
	/**
	 * @brief Get size of frameset queue.
	 * @return Frameset queue size
	 */
	unsigned getQueueSize() { return static_cast<unsigned>(frameset_queue.size()); }
	/**
	 * @brief Check if frameset is available.
	 * @return True if frameset queue is not empty
	 */
	bool hasFrames() { return (!frameset_queue.empty() || !use_custom_queue); }
	/**
	 * @brief Set queued capture mode.
	 * @param use_custom_queue True if framesets are to be queued
	 */
	void setUseQueue(bool use_custom_queue) { this->use_custom_queue = use_custom_queue; }
	void setUseCallback(bool use_callback) { this->use_callback = use_callback; }
	void init(std::string camera_serial_no = "");
	void start();
	void stop();
	bool getFrames(uint8_t*& color_frame, uint16_t*& depth_frame, double& timestamp, unsigned timeout);
	rs2_intrinsics getDepthIntrinsics();
	rs2_intrinsics getColorIntrinsics();
	rs2_extrinsics getDepthToColorExtrinsics();
	void declareRosParameters(rclcpp::Node* node);
	void setOptionFromParameter(std::string parameter_string, float value);
	void loadImageFiles(std::string color_image_filename, std::string depth_image_filename);
	void loadIntrinsicsFiles(std::string color_intrinsics_filename, std::string depth_intrinsics_filename);
	void getColorCameraInfo(sensor_msgs::msg::CameraInfo& camera_info);
	void getDepthCameraInfo(sensor_msgs::msg::CameraInfo& camera_info);
	bool getFramesQueued(uint8_t*& color_frame, uint16_t*& depth_frame, double& timestamp);
	void getQueue(rs2::frame_queue& frame_queue) { frame_queue = this->frame_queue; }
	std::function<void(const uint8_t*, const uint16_t*, const double)> framesCallback;

 private:
	std::atomic<bool>* exit_request;
	std::atomic<bool> exit_clear;
	bool debug = false;
	bool verbose = false;
	bool simulation = false;
	bool align = false;
	bool filter = false;
	long camera_time_base;
	long system_time_base;
	float depth_scale = 0.0001f;
	float depth_max = 2.0f;

	bool use_custom_queue = false;
	bool use_rs_queue = false;
	bool use_syncer = false;
	bool use_callback = false;
	rs2::syncer syncer;
	rs2::stream_profile depth_stream;
	rs2::stream_profile color_stream;
	rs2::device device;
	std::vector<rs2::sensor> sensors;
	rs2::config rs_config;
	rs2::pipeline pipe;
	rs2::pipeline_profile pipe_profile;

	std::map<std::string, rs2_option> color_option_names;
	std::map<std::string, rs2_option> depth_option_names;
	rs2::frameset frameset;
	std::shared_ptr<rs2::filter> filter_align_to_color;
	std::vector<uint8_t>* sim_color_image;
	std::vector<uint16_t>* sim_depth_image;
	rs2_intrinsics color_intrinsics;
	rs2_intrinsics depth_intrinsics;
	double last_frame_timestamp;
	unsigned simulation_framerate = 30;
	time_point timer = hires_clock::now();
	std::queue<rs2::frameset> frameset_queue;
	std::thread capture_thread;
	rs2::frame_queue frame_queue;

	rs2::decimation_filter dec_filter;            // Decimation - reduces depth frame density
	rs2::threshold_filter thr_filter;             // Threshold  - removes values outside recommended range
	rs2::spatial_filter spat_filter;              // Spatial    - edge-preserving spatial smoothing
	rs2::temporal_filter temp_filter;             // Temporal   - reduces temporal noise
	rs2::disparity_transform depth_to_disparity;  // Transform from depth to disparity
	rs2::disparity_transform disparity_to_depth;  // Transform from disparity to depth

	rs2::device getDevice(std::string serial_no = "");
	std::string convertToSnakeCase(const std::string& string_in);
	std::string rsOptionToString(rs2_option option);
	template <class T>
	void setSensorOption(rs2::sensor& sensor, const rs2_option& option, const T& value);
	void configureSensors(const std::vector<rs2::sensor>& sensors);
	char getOptionType(rs2::sensor& sensor, rs2_option& option);
	void loadIntrinsics(rs2_intrinsics& intrinsics, std::string intrinsics_filename);
	void saveIntrinsics(rs2_intrinsics intrinsics, std::string intrinsics_filename);
	void intrinsics2CameraInfo(sensor_msgs::msg::CameraInfo& camera_info, const rs2_intrinsics& intrinsics);
	void captureLoop();
	void captureFrameset();
	rs2::stream_profile getStreamProfile(rs2::sensor sensor, int w, int h, rs2_format format, int fps);
	void initPipeline(std::string camera_serial_no = "");
	void startPipeline();
	void stopPipeline();
	void initSyncer(std::string camera_serial_no = "");
	void startSyncer();
	void stopSyncer();
	void rsCallback(const rs2::frame& frame);
};
