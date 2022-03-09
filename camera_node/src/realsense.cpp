// SYSTEM
#include <iostream>
#include <fstream>
#include <thread>
#include <algorithm>
#include <typeinfo>
#include <experimental/filesystem>
// PROJECT
#include "lodepng/lodepng.h"
#include "nlohmann/json.hpp"
#include "realsense.hpp"

using namespace std::chrono_literals;
namespace fs = std::experimental::filesystem;
using json = nlohmann::json;

/**
 * @brief Constructor.
 */
Realsense::Realsense() : exit_clear{false} {
	last_frame_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
	                           std::chrono::high_resolution_clock::now().time_since_epoch())
	                           .count();
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
rs2::device Realsense::getDevice(std::string serial_no) {
	rs2::context ctx;
	rs2::device_list devices = ctx.query_devices();

	rs2::device selected_device;
	if (devices.size() == 0) {
		std::cout << "+-- No RealSense camera found" << std::endl;
		exit_request->store(true);
		return selected_device;
	} else if (serial_no == "") {
		selected_device = devices[0];
		if (verbose) {
			std::cout << "+-- Found RealSense camera \"" << selected_device.get_info(RS2_CAMERA_INFO_NAME) << "\""
			          << std::endl;
		}
	} else {
		if (verbose) std::cout << "+-- Found RealSense cameras" << std::endl;
		bool serial_no_found = false;
		for (unsigned i = 0; i < devices.size(); i++) {
			std::string device_serial_no = devices[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
			if (verbose) {
				std::cout << "| \"" << devices[i].get_info(RS2_CAMERA_INFO_NAME) << "\" serial no: " << device_serial_no
				          << std::endl;
			}
			if (device_serial_no == serial_no) {
				selected_device = devices[i];
				serial_no_found = true;
			}
		}
		if (!serial_no_found) {
			std::cout << "No Realsense camera with serial no: " << serial_no << " found" << std::endl;
			exit_request->store(true);
		} else {
			if (verbose) {
				std::cout << "| Using camera \"" << selected_device.get_info(RS2_CAMERA_INFO_NAME)
				          << "\" serial no: " << serial_no << std::endl;
			}
		}
	}

	return selected_device;
}

/**
 * @brief Configure realsense camera sensors.
 * @param sensors Realsense camera sensors
 */
void Realsense::configureSensors(const std::vector<rs2::sensor>& sensors) {
	// RealSense L515 sensors:
	// 0: depth sensor (name: L500 Depth Sensor)
	// 1: color sensor (name: RGB Camera)
	// 2: acceleration sensor (name: Motion Module)
	rs2::depth_sensor depth_sensor = sensors[0].as<rs2::depth_sensor>();
	rs2::color_sensor color_sensor = sensors[1].as<rs2::color_sensor>();

	// Search for color and depth sensor
	bool color_sensor_found = false;
	bool depth_sensor_found = false;
	for (auto& sensor : sensors) {
		std::string sensor_rs_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
		std::size_t color_string_found = sensor_rs_name.find("RGB");
		std::size_t depth_string_found = sensor_rs_name.find("Depth");
		std::size_t stereo_string_found = sensor_rs_name.find("Stereo Module");
		if (color_string_found != std::string::npos) {
			color_sensor = sensor.as<rs2::color_sensor>();
			color_sensor_found = true;
		} else if (depth_string_found != std::string::npos || stereo_string_found != std::string::npos) {
			depth_sensor = sensor.as<rs2::depth_sensor>();
			depth_sensor_found = true;
		}
	}
	if (!color_sensor_found || !depth_sensor_found) {
		std::cout << "No realsense camera with color and depth sensor found!" << std::endl;
		exit_request->store(true);
	}

	// Convert all realsense options to ros parameter strings
	std::vector<rs2_option> supported_color_options = color_sensor.get_supported_options();
	for (auto& option : supported_color_options) {
		if (color_sensor.is_option_read_only(option)) continue;
		std::string option_string = rsOptionToString(option);
		color_option_names[option_string] = option;
		// std::cout << "color option: " << option << std::endl;
	}
	std::vector<rs2_option> supported_depth_options = depth_sensor.get_supported_options();
	for (auto& option : supported_depth_options) {
		if (depth_sensor.is_option_read_only(option)) continue;
		std::string option_string = rsOptionToString(option);
		depth_option_names[option_string] = option;
		// std::cout << "depth option: " << option << std::endl;
	}
}

/**
 * @brief Initialize realsense camera.
 * @param camera_serial_no Choose camera to initialize by serial number if set
 */
void Realsense::init(std::string camera_serial_no) {
	if (use_syncer) {
		initSyncer(camera_serial_no);
	} else {
		initPipeline(camera_serial_no);
	}
}

/**
 * @brief Start realsense camera.
 */
void Realsense::start() {
	if (use_callback) {
		std::function<void(const rs2::frame&)> callback = std::bind(&Realsense::rsCallback, this, std::placeholders::_1);
		pipe_profile = pipe.start(rs_config, callback);
		return;
	}

	if (use_syncer) {
		startSyncer();
	} else {
		startPipeline();
	}
}

/**
 * @brief Stop realsense camera.
 */
void Realsense::stop() {
	if (use_syncer) {
		stopSyncer();
	} else {
		stopPipeline();
	}
}

/**
 * @brief Initialize realsense camera pipeline.
 * @param camera_serial_no Choose camera to initialize by serial number if set
 */
void Realsense::initPipeline(std::string camera_serial_no) {
	rs2_error* e = nullptr;
	if (verbose) {
		std::string rsVersion(std::to_string(rs2_get_api_version(&e)).insert(3, ".").insert(1, "."));
		std::cout << "+-- LibRealSense version" << std::endl;
		std::cout << "| Built with v" << RS2_API_VERSION_STR << std::endl;
		std::cout << "| Running with v" << rsVersion << std::endl;
	}

	// Setup realsense device
	rs2::device device = getDevice(camera_serial_no);

	if (exit_request->load()) return;
	rs_config.enable_device(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

	// Setup streams
	rs_config.disable_all_streams();
	std::string rs_camera_name = device.get_info(RS2_CAMERA_INFO_NAME);
	if (rs_camera_name == "Intel RealSense L515") {
		rs_config.enable_stream(RS2_STREAM_DEPTH, 1024, 768, RS2_FORMAT_Z16, 30);
		rs_config.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
	} else if (rs_camera_name == "Intel RealSense D435" || rs_camera_name == "Intel RealSense D415" ||
	           rs_camera_name == "Intel RealSense D455") {
		rs_config.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
		rs_config.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
	}

	// Setup filters
	filter_align_to_color = std::make_shared<rs2::align>(RS2_STREAM_COLOR);
	depth_to_disparity = rs2::disparity_transform(true);
	disparity_to_depth = rs2::disparity_transform(false);
	thr_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.0);
	thr_filter.set_option(RS2_OPTION_MAX_DISTANCE, 2.0);
	spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
	spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5);
	spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
	temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4f);

	// Build pipeline with configuration
	if (rs_config.can_resolve(pipe)) {
		pipe_profile = rs_config.resolve(pipe);

		// Configure sensors
		configureSensors(pipe_profile.get_device().query_sensors());
	} else {
		std::cout << "Wrong realsense configuration" << std::endl;
		exit_request->store(true);
	}

	this->device = pipe_profile.get_device();
	this->sensors = pipe_profile.get_device().query_sensors();
}

/**
 * @brief Start realsense camera pipeline.
 */
void Realsense::startPipeline() {
	std::cout << "Initializing camera" << std::endl;

	// Set advanced parameters
	if (std::string(pipe_profile.get_device().get_info(RS2_CAMERA_INFO_PRODUCT_LINE)) == "D400") {
		if (pipe_profile.get_device().is<rs400::advanced_mode>()) {
			rs2::depth_sensor depth_sensor = pipe_profile.get_device().query_sensors()[0].as<rs2::depth_sensor>();
			depth_sensor.set_option(RS2_OPTION_VISUAL_PRESET, 1);

			rs400::advanced_mode advanced_device(pipe_profile.get_device());

			// Set depth units
			auto depth_table = advanced_device.get_depth_table();
			uint32_t depth_units = static_cast<uint32_t>(depth_scale * 1e6f);
			depth_table.depthUnits = depth_units;

			// Set maximal depth
			depth_table.depthClampMax = static_cast<int32_t>(depth_max / depth_scale);
			advanced_device.set_depth_table(depth_table);
		} else {
			std::cout << "Advanced mode not supported" << std::endl;
			exit_request->store(true);
		}
	}

	// Start realsense pipeline
	if (use_rs_queue) {
		frame_queue = rs2::frame_queue(10);
		pipe_profile = pipe.start(rs_config, frame_queue);
	} else {
		pipe_profile = pipe.start(rs_config);

		if (debug) {
			// Debug parameters
			std::vector<rs2::sensor> sensors = pipe_profile.get_device().query_sensors();
			rs2::color_sensor depth_sensor = sensors[0].as<rs2::color_sensor>();
			rs2::color_sensor color_sensor = sensors[1].as<rs2::color_sensor>();
			std::cout << "## color:" << std::endl;
			std::cout << "## RS2_OPTION_AUTO_EXPOSURE_PRIORITY = "
			          << color_sensor.get_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY) << std::endl;
			std::cout << "## depth:" << std::endl;
			std::cout << "## RS2_OPTION_DEPTH_UNITS = " << depth_sensor.get_option(RS2_OPTION_DEPTH_UNITS) << std::endl;
			std::cout << "## RS2_OPTION_FRAMES_QUEUE_SIZE = " << depth_sensor.get_option(RS2_OPTION_FRAMES_QUEUE_SIZE)
			          << std::endl;
			std::cout << "## RS2_OPTION_VISUAL_PRESET = " << depth_sensor.get_option(RS2_OPTION_VISUAL_PRESET) << std::endl;

			if (pipe_profile.get_device().is<rs400::advanced_mode>()) {
				rs400::advanced_mode advanced_device(pipe_profile.get_device());
				auto depth_table = advanced_device.get_depth_table();
				std::cout << "## advanced mode depthUnits = " << depth_table.depthUnits << std::endl;
				std::cout << "## advanced mode depthClampMin = " << depth_table.depthClampMin << std::endl;
				std::cout << "## advanced mode depthClampMax = " << depth_table.depthClampMax << std::endl;
			}
		}
	}

	// Wait for streams to settle
	if (verbose) std::cout << "Waiting for streams ";
	if (use_rs_queue) {
		while (!frame_queue.try_wait_for_frame(&frameset, 40)) {
			std::this_thread::sleep_for(40ms);
			std::cout << "." << std::flush;
		}
	} else {
		while (!pipe.try_wait_for_frames(&frameset, 40)) {
			std::this_thread::sleep_for(40ms);
			if (verbose) std::cout << "." << std::flush;
		}
	}
	if (verbose) std::cout << " done" << std::endl;

	// Set base time points for system independent camera clock
	if (frameset.get_frame_timestamp_domain() == RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK) {
		system_time_base = static_cast<long>(std::chrono::duration_cast<std::chrono::nanoseconds>(
		                                         std::chrono::high_resolution_clock::now().time_since_epoch())
		                                         .count());
		camera_time_base = static_cast<long>(frameset.get_timestamp() * 1e6);
	}

	if (verbose) {
		std::cout << "+-- Timestamp domains:" << std::endl;
		std::cout << "| Depth sensor: " << frameset.get_depth_frame().get_frame_timestamp_domain() << std::endl;
		std::cout << "| Color sensor: " << frameset.get_color_frame().get_frame_timestamp_domain() << std::endl;
		if (frameset.get_frame_timestamp_domain() == RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK) {
			std::cout << "| System base time: " << system_time_base << std::endl;
			std::cout << "| Camera base time: " << camera_time_base << std::endl;
		}
	}

	// Start capture thread
	if (use_custom_queue && !simulation) {
		capture_thread = std::thread(std::bind(&Realsense::captureLoop, this));
	}

	std::cout << "+==========[ Camera Started ]==========+" << std::endl;
}

/**
 * @brief Stop realsense camera pipeline.
 */
void Realsense::stopPipeline() {
	if (capture_thread.joinable()) {
		capture_thread.join();
	}
	pipe.stop();
	std::cout << "+==========[ Camera Stopped ]==========+" << std::endl;
	exit_clear = true;
}

/**
 * @brief Initialize realsense camera synchronizer.
 * @param camera_serial_no Choose camera to initialize by serial number if set
 */
void Realsense::initSyncer(std::string camera_serial_no) {
	rs2_error* e = nullptr;
	if (verbose) {
		std::string rsVersion(std::to_string(rs2_get_api_version(&e)).insert(3, ".").insert(1, "."));
		std::cout << "+-- LibRealSense version" << std::endl;
		std::cout << "| Built with v" << RS2_API_VERSION_STR << std::endl;
		std::cout << "| Running with v" << rsVersion << std::endl;
	}

	// Setup realsense device
	device = getDevice(camera_serial_no);
	if (exit_request->load()) return;

	// Configure sensors
	sensors = device.query_sensors();
	rs2::depth_sensor depth_sensor = sensors[0].as<rs2::depth_sensor>();
	rs2::color_sensor color_sensor = sensors[1].as<rs2::color_sensor>();
	configureSensors(sensors);

	// Setup streams
	std::string rs_camera_name = device.get_info(RS2_CAMERA_INFO_NAME);
	if (rs_camera_name == "Intel RealSense L515") {
		depth_stream = getStreamProfile(depth_sensor, 1024, 768, RS2_FORMAT_Z16, 30);
		color_stream = getStreamProfile(color_sensor, 1280, 720, RS2_FORMAT_RGB8, 30);
	} else if (rs_camera_name == "Intel RealSense D435" || rs_camera_name == "Intel RealSense D415" ||
	           rs_camera_name == "Intel RealSense D455") {
		depth_stream = getStreamProfile(depth_sensor, 1280, 720, RS2_FORMAT_Z16, 30);
		color_stream = getStreamProfile(color_sensor, 1280, 720, RS2_FORMAT_RGB8, 30);
	}

	// Setup filters
	filter_align_to_color = std::make_shared<rs2::align>(RS2_STREAM_COLOR);
	depth_to_disparity = rs2::disparity_transform(true);
	disparity_to_depth = rs2::disparity_transform(false);
	// dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
	thr_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.0);
	thr_filter.set_option(RS2_OPTION_MAX_DISTANCE, 2.0);
	spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
	spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5);
	spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
	temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4f);
}

/**
 * @brief Start realsense camera synchronizer.
 */
void Realsense::startSyncer() {
	// Open sensor streams
	rs2::depth_sensor depth_sensor = sensors[0].as<rs2::depth_sensor>();
	rs2::color_sensor color_sensor = sensors[1].as<rs2::color_sensor>();
	depth_sensor.open(depth_stream);
	color_sensor.open(color_stream);

	// Create syncer
	syncer = rs2::syncer(2);

	// Start streams
	depth_sensor.start(syncer);
	color_sensor.start(syncer);

	// Wait for streams to settle
	std::cout << "+-- Initializing camera" << std::endl;
	std::cout << "| Waiting for streams ";
	unsigned frameset_size = 0;
	while (frameset_size < 2 && !exit_request->load()) {
		frameset = syncer.wait_for_frames();
		frameset_size = static_cast<unsigned>(frameset.size());
		std::this_thread::sleep_for(30ms);
		std::cout << "." << std::flush;
	}
	std::cout << " done" << std::endl;

	// Start capture thread
	if (use_custom_queue && !simulation) {
		capture_thread = std::thread(std::bind(&Realsense::captureLoop, this));
	}

	std::cout << "+-- RealSense camera started" << std::endl;
}

/**
 * @brief Stop realsense camera synchronizer.
 */
void Realsense::stopSyncer() {
	if (capture_thread.joinable()) {
		capture_thread.join();
	}
	rs2::depth_sensor depth_sensor = sensors[0].as<rs2::depth_sensor>();
	rs2::color_sensor color_sensor = sensors[1].as<rs2::color_sensor>();
	depth_sensor.stop();
	color_sensor.stop();
	depth_sensor.close();
	color_sensor.close();
	std::cout << "+-- RealSense camera stopped" << std::endl;
	exit_clear = true;
}

/**
 * @brief Frameset capture loop.
 */
void Realsense::captureLoop() {
	while (!exit_request->load() && rclcpp::ok()) {
		captureFrameset();
	}
}

/**
 * @brief Capture frameset and push to queue.
 */
void Realsense::captureFrameset() {
	rs2::frameset frameset;
	bool frame_arrvied = false;
	if (use_syncer) {
		frame_arrvied = (syncer.try_wait_for_frames(&frameset, 33));
	} else {
		frame_arrvied = (pipe.try_wait_for_frames(&frameset, 33));
	}
	if (frame_arrvied) {
		if (frameset_queue.size() > 4) {
			frameset_queue.pop();
			std::cout << "rs frameset queue full" << std::endl;
		}
		frameset_queue.push(frameset);
		if (debug) {
			double frameset_age = (hires_clock::now().time_since_epoch().count() / 1e6 - frameset.get_timestamp());
			std::cout << "| rs capture frameset age:          " << frameset_age << std::endl;
		}
	}
}

/**
 * @brief Fetch color and depth frame from frameset queue.
 * @param color_frame 8bit RGB Color frame
 * @param depth_frame 16bit grayscale Depth frame
 * @param timestamp Timestamp of the first captured frame
 * @return True if frames arrived before timeout, false otherwise
 */
bool Realsense::getFramesQueued(uint8_t*& color_frame, uint16_t*& depth_frame, double& timestamp) {
	if (!frameset_queue.empty()) {
		rs2::frameset frameset = frameset_queue.front();
		timestamp = frameset.get_timestamp();
		if (debug) {
			double frameset_age = (hires_clock::now().time_since_epoch().count() / 1e6 - frameset.get_timestamp());
			std::cout << "| rs time domain:                   " << frameset.get_frame_timestamp_domain() << std::endl;
			std::cout << "| rs frameset age:                  " << frameset_age << std::endl;
			std::cout << "| rs frameset queue size:           " << frameset_queue.size() << std::endl;
		}

		frameset_queue.pop();

		if (align) {
			if (debug) timer = hires_clock::now();
			frameset = filter_align_to_color->process(frameset);
			if (debug) {
				double duration = (hires_clock::now() - timer).count() / 1e6;
				std::cout << "| rs align duration:                " << duration << " ms" << std::endl;
			}
		}

		// Filter depth frame
		rs2::frame filtered;
		if (filter) {
			/*
			The implemented flow of the filters pipeline is in the following order:
			1. apply decimation filter
			2. apply threshold filter
			3. transform the scene into disparity domain
			4. apply spatial filter
			5. apply temporal filter
			6. revert the results back (if step Disparity filter was applied
			to depth domain (each post processing block is optional and can be applied independantly).
			*/
			filtered = frameset.get_depth_frame();
			time_point filter_timer = hires_clock::now();
			timer = hires_clock::now();
			double filter_duration = 0.0;
			// Theshold filter
			filtered = thr_filter.process(filtered);
			if (debug) {
				filter_duration = (hires_clock::now() - timer).count() / 1e6;
				std::cout << "| rs threshold filter duration:     " << filter_duration << " ms" << std::endl;
			}
			// Depth to disparity
			timer = hires_clock::now();
			filtered = depth_to_disparity.process(filtered);
			if (debug) {
				filter_duration = (hires_clock::now() - timer).count() / 1e6;
				std::cout << "| rs depth to disparity duration:   " << filter_duration << " ms" << std::endl;
			}
			/*
			// Spatial filter (has long processing time)
			timer = hires_clock::now();
			filtered = spat_filter.process(filtered);
			if (debug) {
				filter_duration = (hires_clock::now() - timer).count() / 1e6;
				std::cout << "| rs spatial filter duration:       " << filter_duration << " ms" << std::endl;
			}
			*/
			// Temporal filter
			timer = hires_clock::now();
			filtered = temp_filter.process(filtered);
			if (debug) {
				filter_duration = (hires_clock::now() - timer).count() / 1e6;
				std::cout << "| rs temporal filter duration:      " << filter_duration << " ms" << std::endl;
			}
			// Disparity to depth
			timer = hires_clock::now();
			filtered = disparity_to_depth.process(filtered);
			if (debug) {
				filter_duration = (hires_clock::now() - timer).count() / 1e6;
				std::cout << "| rs disparity to depth duration:   " << filter_duration << " ms" << std::endl;
				double all_filter_duration = (hires_clock::now() - filter_timer).count() / 1e6;
				std::cout << "| rs all filter duration:           " << all_filter_duration << " ms" << std::endl << std::endl;
			}
		}

		// Get frames from frameset
		color_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(frameset.get_color_frame().get_data()));
		if (filter) {
			depth_frame = reinterpret_cast<uint16_t*>(const_cast<void*>(filtered.get_data()));
		} else {
			depth_frame = reinterpret_cast<uint16_t*>(const_cast<void*>(frameset.get_depth_frame().get_data()));
		}

		return true;
	} else {
		return false;
	}
}

/**
 * @brief Fetch color and depth frame from realsense camera.
 * @param color_frame 8bit RGB Color frame
 * @param depth_frame 16bit grayscale Depth frame
 * @param timestamp Timestamp of the first captured frame
 * @param timeout Time to wait for frames in milliseconds
 * @return True if frames arrived before timeout, false otherwise
 */
bool Realsense::getFrames(uint8_t*& color_frame, uint16_t*& depth_frame, double& timestamp, unsigned timeout) {
	if (!simulation) {
		// Get camera frames
		bool frames_available = false;
		if (use_rs_queue) {
			frames_available = frame_queue.try_wait_for_frame(&frameset, timeout);
		} else {
			frames_available = pipe.try_wait_for_frames(&frameset, timeout);
		}
		if (frames_available) {
			// Align depth frame to color frame
			if (align) {
				if (debug) timer = hires_clock::now();
				frameset = filter_align_to_color->process(frameset);
				if (debug) {
					double duration = (hires_clock::now() - timer).count() / 1e6;
					std::cout << "| rs align duration:            " << duration << " ms" << std::endl;
				}
			}

			// Filter depth frame
			rs2::frame filtered;
			if (filter) {
				/*
				The implemented flow of the filters pipeline is in the following order:
				1. apply decimation filter
				2. apply threshold filter
				3. transform the scene into disparity domain
				4. apply spatial filter
				5. apply temporal filter
				6. revert the results back (if step Disparity filter was applied
				to depth domain (each post processing block is optional and can be applied independantly).
				*/
				filtered = frameset.get_depth_frame();
				time_point filter_timer = hires_clock::now();
				timer = hires_clock::now();
				double filter_duration = 0.0;
				// Theshold filter
				filtered = thr_filter.process(filtered);
				if (debug) {
					filter_duration = (hires_clock::now() - timer).count() / 1e6;
					std::cout << "| rs threshold filter duration:     " << filter_duration << " ms" << std::endl;
				}
				// Depth to disparity
				timer = hires_clock::now();
				filtered = depth_to_disparity.process(filtered);
				if (debug) {
					filter_duration = (hires_clock::now() - timer).count() / 1e6;
					std::cout << "| rs depth to disparity duration:   " << filter_duration << " ms" << std::endl;
				}
				/*
				// Spatial filter (has long processing time)
				timer = hires_clock::now();
				filtered = spat_filter.process(filtered);
				if (debug) {
					filter_duration = (hires_clock::now() - timer).count() / 1e6;
					std::cout << "| rs spatial filter duration:       " << filter_duration << " ms" << std::endl;
				}
				*/
				// Temporal filter
				timer = hires_clock::now();
				filtered = temp_filter.process(filtered);
				if (debug) {
					filter_duration = (hires_clock::now() - timer).count() / 1e6;
					std::cout << "| rs temporal filter duration:      " << filter_duration << " ms" << std::endl;
				}
				// Disparity to depth
				timer = hires_clock::now();
				filtered = disparity_to_depth.process(filtered);
				if (debug) {
					filter_duration = (hires_clock::now() - timer).count() / 1e6;
					std::cout << "| rs disparity to depth duration:   " << filter_duration << " ms" << std::endl;
					double all_filter_duration = (hires_clock::now() - filter_timer).count() / 1e6;
					std::cout << "| rs all filter duration:           " << all_filter_duration << " ms" << std::endl << std::endl;
				}
			}

			// Get frames from frameset
			color_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(frameset.get_color_frame().get_data()));
			if (filter) {
				depth_frame = reinterpret_cast<uint16_t*>(const_cast<void*>(filtered.get_data()));
			} else {
				depth_frame = reinterpret_cast<uint16_t*>(const_cast<void*>(frameset.get_depth_frame().get_data()));
			}

			// RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK 	Frame timestamp was measured in relation to the camera clock
			// RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME     Frame timestamp was measured in relation to the OS system clock
			// RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME     Frame timestamp was measured in relation to the camera clock and converted
			//   to OS system clock by constantly measure the difference
			// RS2_TIMESTAMP_DOMAIN_COUNT           Number of enumeration values. Not a valid input: intended to be used in
			//   for-loops.

			// - timestamp mesasured in milliseconds
			// - color frame is taken before depth frame (approx 7 ms)
			// - frameset has timestamp of color frame

			if (frameset.get_frame_timestamp_domain() == RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK) {
				double elapsed_camera_time = (frameset.get_timestamp() * 1e6) - camera_time_base;
				timestamp = (system_time_base + elapsed_camera_time) / 1e6;
			} else {
				timestamp = frameset.get_timestamp();
			}

			return true;
		} else {
			depth_frame = nullptr;
			color_frame = nullptr;
			return false;
		}
	} else {
		// Simulation
		double time_since_last_frame = std::chrono::duration_cast<std::chrono::milliseconds>(
		                                   std::chrono::high_resolution_clock::now().time_since_epoch())
		                                   .count() -
		                               last_frame_timestamp;
		double simulation_frame_duration = 1000. / static_cast<double>(simulation_framerate);
		// Simulate camera framerate
		if (time_since_last_frame < simulation_frame_duration) {
			double wait_time = simulation_frame_duration - time_since_last_frame;
			std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(wait_time));
		}
		// Set simulated timestamp to now
		timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
		                std::chrono::high_resolution_clock::now().time_since_epoch())
		                .count();
		// Set frames to simulated images
		color_frame = sim_color_image->data();
		depth_frame = sim_depth_image->data();
		last_frame_timestamp = timestamp;
		return true;
	}
}

/**
 * @brief Set configuration option for realsense sensor.
 * @param sensor Realsense sensor
 * @param option Configuration option
 * @param value New value for configuration option
 */
template <class T>
void Realsense::setSensorOption(rs2::sensor& sensor, const rs2_option& option, const T& value) {
	try {
		if (sensor.supports(option) && !sensor.is_option_read_only(option)) {
			sensor.set_option(option, value);
		} else {
			std::cout << "Option " << option << " not supported by " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
		}
	} catch (const rs2::invalid_value_error& e) {
		std::cout << "RealSense option " << option << " received invalid value " << value << std::endl;
	} catch (const rs2::error& e) {
		std::cout << "RealSense error" << std::endl;
		exit_request->store(true);
	}
}

/**
 * @brief Set realsense option from ros parameter.
 * @param parameter_string Full parameter string with namespaces
 * @param value New option value
 */
void Realsense::setOptionFromParameter(std::string parameter_string, float value) {
	// Tokenize parameter string with '.' as delimeter
	std::vector<size_t> delimeter_positions;
	std::vector<std::string> parameter_string_tokens;
	size_t pos = 0;
	while (pos != std::string::npos) {
		size_t next_pos = parameter_string.find('.', pos);
		if (next_pos != std::string::npos) {
			delimeter_positions.push_back(next_pos);
			parameter_string_tokens.push_back(parameter_string.substr(pos, next_pos - pos));
			pos = next_pos + 1;
		} else {
			parameter_string_tokens.push_back(parameter_string.substr(pos, std::string::npos));
			pos = std::string::npos;
		}
	}

	// Check realsense parameter namespace sanity
	bool namespace_valid = true;
	if (parameter_string_tokens.size() != 3) {
		namespace_valid = false;
	} else {
		if (parameter_string_tokens[0] != "sensor") {
			namespace_valid = false;
		} else if (parameter_string_tokens[1] != "color" && parameter_string_tokens[1] != "depth") {
			namespace_valid = false;
		}
	}
	if (!namespace_valid) {
		std::string parameter_namespace = parameter_string.substr(0, delimeter_positions.back());
		std::cout << "Wrong parameter namespace for realsense. Should be \"sensor.color\" or \"sensor.depth\" but is \""
		          << parameter_namespace << "\"" << std::endl;
		return;
	}

	// Set realsense parameter
	std::vector<rs2::sensor> sensors = pipe_profile.get_device().query_sensors();
	std::string option_name = parameter_string_tokens.back();
	bool option_valid = true;
	if (parameter_string_tokens[1] == "color") {
		rs2::color_sensor color_sensor = sensors[1].as<rs2::color_sensor>();
		auto option = color_option_names.find(option_name);
		if (option != color_option_names.end()) {
			bool set_option_override = false;

			// Option enable_auto_exposure gets disabled if option exposure is set
			if (option->second == RS2_OPTION_EXPOSURE) {
				if (static_cast<bool>(color_sensor.get_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE))) {
					set_option_override = true;
					// std::cout << "Disable auto exposure to set exposure value!" << std::endl;
				}
			}
			// Option enable_auto_white_balance gets disabled if option white_balance is set
			else if (option->second == RS2_OPTION_WHITE_BALANCE) {
				if (static_cast<bool>(color_sensor.get_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE))) {
					set_option_override = true;
					// std::cout << "Disable auto white balance to set white balance value!" << std::endl;
				}
			}
			// Errors for gain ranges
			else if (option->second == RS2_OPTION_GAIN) {
				set_option_override = true;
			}

			if (!set_option_override) {
				if (verbose) std::cout << "| set " << parameter_string << " to: " << value << std::endl;
				setSensorOption(color_sensor, option->second, value);
			}

		} else {
			option_valid = false;
		}
	} else if (parameter_string_tokens[1] == "depth") {
		rs2::depth_sensor depth_sensor = sensors[0].as<rs2::depth_sensor>();
		auto option = depth_option_names.find(option_name);
		if (option != depth_option_names.end()) {
			if (verbose) std::cout << "| set " << parameter_string << " to: " << value << std::endl;
			setSensorOption(depth_sensor, option->second, value);
		} else {
			option_valid = false;
		}
	}
	if (!option_valid) {
		std::cout << "Realsense option " << option_name << " not a declared parameter!" << std::endl;
	}
}

/**
 * @brief Convert string to lowercase replacing whitespaces with underscores.
 * @param string_in String to convert
 * @return Lowercase string with underscores instead of whitespaces
 */
std::string Realsense::convertToSnakeCase(const std::string& string_in) {
	std::string string_out = string_in;
	std::replace(string_out.begin(), string_out.end(), ' ', '_');
	std::locale loc;
	for (char& c : string_out) {
		c = std::tolower(c, loc);
	}
	return string_out;
}

/**
 * @brief Convert realsense option to lowercase string replacing whitespaces with underscores.
 * @param option Realsense option
 * @return Lowercase string with underscores instead of whitespaces
 */
std::string Realsense::rsOptionToString(rs2_option option) { return convertToSnakeCase(rs2_option_to_string(option)); }

/**
 * @brief Declare ros parameters for sensor options
 */
void Realsense::declareRosParameters(rclcpp::Node* node) {
	if (debug) std::cout << "+-- Set realsense parameters" << std::endl;
	std::vector<rs2::sensor> sensors = pipe_profile.get_device().query_sensors();
	// Declare color sensor parameters
	rs2::color_sensor color_sensor = sensors[1].as<rs2::color_sensor>();
	for (auto& option : color_option_names) {
		std::string ros_parameter = "sensor.color." + option.first;
		char type_char = getOptionType(color_sensor, option.second);
		// std::cout << "declare parameter " << ros_parameter << " from option \"" << option.second << "\" with type "
		//           << type_char << std::endl;
		if (type_char == 'b') {
			node->declare_parameter(ros_parameter, static_cast<bool>(color_sensor.get_option(option.second)));
		} else if (type_char == 'i') {
			node->declare_parameter(ros_parameter, static_cast<int>(color_sensor.get_option(option.second)));
		} else if (type_char == 'f') {
			node->declare_parameter(ros_parameter, color_sensor.get_option(option.second));
		}
	}
	// Declare depth sensor parameters
	rs2::depth_sensor depth_sensor = sensors[0].as<rs2::depth_sensor>();
	for (auto& option : depth_option_names) {
		std::string ros_parameter = "sensor.depth." + option.first;
		char type_char = getOptionType(depth_sensor, option.second);
		// std::cout << "declare parameter " << ros_parameter << " from option \"" << option.second << "\" with type "
		//           << type_char << std::endl;
		if (type_char == 'b') {
			node->declare_parameter(ros_parameter, static_cast<bool>(depth_sensor.get_option(option.second)));
		} else if (type_char == 'i') {
			node->declare_parameter(ros_parameter, static_cast<int>(depth_sensor.get_option(option.second)));
		} else if (type_char == 'f') {
			node->declare_parameter(ros_parameter, depth_sensor.get_option(option.second));
		}
	}
}

/**
 * @brief Get value type from realsense option
 * @param sensor Realsense sensor for option
 * @param option Realsense option
 * @return Character for type: bool='b', int='i', float='f'
 */
char Realsense::getOptionType(rs2::sensor& sensor, rs2_option& option) {
	rs2::option_range option_range = sensor.get_option_range(option);
	char type_char = 'n';
	if (option_range.step == 1.f) {
		if (option_range.min == 0.f && option_range.max == 1.f) {
			type_char = 'b';
		} else {
			type_char = 'i';
		}
	} else {
		type_char = 'f';
	}
	return type_char;
}

/**
 * @brief Get intrinsics of depth sensor.
 * @return Depth sensor intrinsics
 */
rs2_intrinsics Realsense::getDepthIntrinsics() {
	if (!simulation) {
		if (align) {
			if (use_syncer) {
				return color_stream.as<rs2::video_stream_profile>().get_intrinsics();
			} else {
				return pipe_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
			}
		} else {
			if (use_syncer) {
				return depth_stream.as<rs2::video_stream_profile>().get_intrinsics();
			} else {
				return pipe_profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
			}
		}
	} else {
		if (align) {
			return color_intrinsics;
		} else {
			return depth_intrinsics;
		}
	}
}

/**
 * @brief Get intrinsics of color sensor.
 * @return Color sensor intrinsics
 */
rs2_intrinsics Realsense::getColorIntrinsics() {
	if (!simulation) {
		if (use_syncer) {
			return color_stream.as<rs2::video_stream_profile>().get_intrinsics();
		} else {
			return pipe_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
		}
	} else {
		return color_intrinsics;
	}
}

/**
 * @brief Get depth sensor to color sensor extrinsics.
 * @return Depth sensor to color sensor extrinsics
 */
rs2_extrinsics Realsense::getDepthToColorExtrinsics() {
	if (!simulation) {
		if (use_syncer) {
			return depth_stream.get_extrinsics_to(color_stream);
		} else {
			return pipe_profile.get_stream(RS2_STREAM_DEPTH).get_extrinsics_to(pipe_profile.get_stream(RS2_STREAM_COLOR));
		}
	} else {
		rs2_extrinsics sim_extrinsics;
		for (int i = 0; i < 9; i++) sim_extrinsics.rotation[i] = 0.f;
		sim_extrinsics.rotation[0] = 1.f;
		sim_extrinsics.rotation[4] = 1.f;
		sim_extrinsics.rotation[8] = 1.f;
		for (int i = 0; i < 3; i++) sim_extrinsics.translation[i] = 0.f;
		return sim_extrinsics;
	}
}

/**
 * @brief Load color and depth png image files for simulation.
 * @param color_image_filename 8bit rgb png image filename with path
 * @param depth_image_filename 16bit grayscale png image filename with path
 */
void Realsense::loadImageFiles(std::string color_image_filename, std::string depth_image_filename) {
	if (fs::exists(fs::path(color_image_filename))) {
		unsigned width, height;
		sim_color_image = new std::vector<uint8_t>();
		unsigned error =
		    lodepng::decode(*sim_color_image, width, height, color_image_filename, LodePNGColorType::LCT_RGB, 8);
		if (error) {
			std::cout << "decoder error " << error << ": " << lodepng_error_text(error) << std::endl;
		}
	} else {
		std::cout << "Simulation enabled but color png image file not found: " << color_image_filename << std::endl;
		exit_request->store(true);
		return;
	}
	if (fs::exists(fs::path(depth_image_filename))) {
		unsigned width, height;
		sim_depth_image = new std::vector<uint16_t>();
		std::vector<uint8_t>* sim_depth_image_byte_ptr = reinterpret_cast<std::vector<uint8_t>*>(sim_depth_image);
		unsigned error =
		    lodepng::decode(*sim_depth_image_byte_ptr, width, height, depth_image_filename, LodePNGColorType::LCT_GREY, 16);
		// Convert big endian lodepng image to little endian image
		for (unsigned i = 0; i < sim_depth_image->size(); i++) {
			sim_depth_image->at(i) = __builtin_bswap16(sim_depth_image->at(i));
			// Scale from 1m=1000 in sim image to 1m=4000 for depth scale 0.00025
			sim_depth_image->at(i) = static_cast<uint16_t>(sim_depth_image->at(i) * 1 / (depth_scale * 1000));
		}
		if (error) {
			std::cout << "decoder error " << error << ": " << lodepng_error_text(error) << std::endl;
		}
	} else {
		std::cout << "Simulation enabled but depth png image file not found: " << depth_image_filename << std::endl;
		exit_request->store(true);
		return;
	}
}

/**
 * @brief Load color and depth json intrinsics files for simulation.
 * @param color_intrinsics_filename Color intrinsics json filename
 * @param depth_intrinsics_filename Depth intrinsics json filename
 */
void Realsense::loadIntrinsicsFiles(std::string color_intrinsics_filename, std::string depth_intrinsics_filename) {
	loadIntrinsics(color_intrinsics, color_intrinsics_filename);
	loadIntrinsics(depth_intrinsics, depth_intrinsics_filename);
}

/**
 * @brief Realsense::loadIntrinsics
 * @param intrinsics Camera intrinsics to set
 * @param intrinsics_filename Intrinsics json filename with path
 */
void Realsense::loadIntrinsics(rs2_intrinsics& intrinsics, std::string intrinsics_filename) {
	// Load intrinsics json file
	json intrinsics_json;
	if (fs::exists(fs::path(intrinsics_filename))) {
		std::ifstream ifs;
		ifs.open(intrinsics_filename);
		ifs >> intrinsics_json;
		ifs.close();
	} else {
		std::cout << "Simulation enabled but camera intrinsics file not found: " << intrinsics_filename << std::endl;
		exit_request->store(true);
		return;
	}

	// Create rs2_intrinsics from json
	// Width of the image in pixels
	intrinsics.width = intrinsics_json["width"];
	// Height of the image in pixels
	intrinsics.height = intrinsics_json["height"];
	// Focal length of the image plane, as a multiple of pixel width
	intrinsics.fx = intrinsics_json["fx"];
	// Focal length of the image plane, as a multiple of pixel height
	intrinsics.fy = intrinsics_json["fy"];
	// Horizontal coordinate of the principal point of the image, as a pixel offset from the left edge
	intrinsics.ppx = intrinsics_json["ppx"];
	// Vertical coordinate of the principal point of the image, as a pixel offset from the top edge
	intrinsics.ppy = intrinsics_json["ppy"];
	// Distortion model of the image
	intrinsics.model = intrinsics_json["model"];
	// Distortion coefficients. Order for Brown-Conrady: [k1, k2, p1, p2, k3]
	// Order for F-Theta Fish-eye: [k1, k2, k3, k4, 0]
	// Other models are subject to their own interpretations
	std::vector<float> coeffs = intrinsics_json["coeffs"].get<std::vector<float>>();
	for (unsigned i = 0; i < 5; i++) {
		intrinsics.coeffs[i] = coeffs[i];
	}

	// rs2_distortion enumerator: Distortion model defines how pixel coordinates should be mapped to sensor coordinates.
	// 0 RS2_DISTORTION_NONE Rectilinear images. No distortion compensation required.
	// 1 RS2_DISTORTION_MODIFIED_BROWN_CONRADY Equivalent to Brown-Conrady distortion, except that tangential distortion
	//     is applied to radially distorted points
	// 2 RS2_DISTORTION_INVERSE_BROWN_CONRADY Equivalent to Brown-Conrady distortion, except undistorts image instead
	//     of distorting it
	// 3 RS2_DISTORTION_FTHETA F-Theta fish-eye distortion model
	// 4 RS2_DISTORTION_BROWN_CONRADY	Unmodified Brown-Conrady distortion model
	// 5 RS2_DISTORTION_KANNALA_BRANDT4 Four parameter Kannala Brandt distortion model
	// 6 RS2_DISTORTION_COUNT	Number of enumeration values. Not a valid input: intended to be used in for-loops.
}

/**
 * @brief Save realsense sensor intrinsics to json file.
 * @param intrinsics Realsense sensor intrinsics
 * @param intrinsics_filename Intrinsics json filename with path
 */
void Realsense::saveIntrinsics(rs2_intrinsics intrinsics, std::string intrinsics_filename) {
	json intrinsics_json;
	// Width of the image in pixels
	intrinsics_json["width"] = intrinsics.width;
	// Height of the image in pixels
	intrinsics_json["height"] = intrinsics.height;
	// Focal length of the image plane, as a multiple of pixel width
	intrinsics_json["fx"] = intrinsics.fx;
	// Focal length of the image plane, as a multiple of pixel height
	intrinsics_json["fy"] = intrinsics.fy;
	// Horizontal coordinate of the principal point of the image, as a pixel offset from the left edge
	intrinsics_json["ppx"] = intrinsics.ppx;
	// Vertical coordinate of the principal point of the image, as a pixel offset from the top edge
	intrinsics_json["ppy"] = intrinsics.ppy;
	// Distortion model of the image
	intrinsics_json["model"] = intrinsics.model;
	// Distortion coefficients. Order for Brown-Conrady: [k1, k2, p1, p2, k3]
	// Order for F-Theta Fish-eye: [k1, k2, k3, k4, 0]
	// Other models are subject to their own interpretations
	intrinsics_json["coeffs"] = intrinsics.coeffs;
	std::cout << "coeffs:" << std::endl;
	for (unsigned i = 0; i < 5; i++) {
		std::cout << intrinsics.coeffs[i];
	}
	std::cout << std::endl;

	std::ofstream ofs;
	ofs.open(intrinsics_filename);
	ofs << intrinsics_json.dump(4) << std::endl;
	ofs.close();
}

/**
 * @brief Get ros camera info message for color stream.
 * @param camera_info Ros camera info message
 */
void Realsense::getColorCameraInfo(sensor_msgs::msg::CameraInfo& camera_info) {
	rs2_intrinsics intrinsics;
	if (!simulation) {
		rs2::video_stream_profile stream_profile;
		if (use_syncer) {
			stream_profile = color_stream.as<rs2::video_stream_profile>();
		} else {
			stream_profile = pipe_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
		}
		intrinsics = stream_profile.get_intrinsics();
		intrinsics2CameraInfo(camera_info, intrinsics);
	} else {
		intrinsics = color_intrinsics;
		intrinsics2CameraInfo(camera_info, intrinsics);
	}
}

/**
 * @brief Get ros camera info message for depth stream.
 * @param camera_info Ros camera info message
 */
void Realsense::getDepthCameraInfo(sensor_msgs::msg::CameraInfo& camera_info) {
	rs2_intrinsics intrinsics;
	if (!simulation) {
		rs2::video_stream_profile stream_profile;
		if (use_syncer) {
			stream_profile = depth_stream.as<rs2::video_stream_profile>();
		} else {
			stream_profile = pipe_profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
		}
		if (align) {
			if (use_syncer) {
				stream_profile = color_stream.as<rs2::video_stream_profile>();
			} else {
				stream_profile = pipe_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
			}
		}
		intrinsics = stream_profile.get_intrinsics();
		intrinsics2CameraInfo(camera_info, intrinsics);
	} else {
		if (align) {
			intrinsics = color_intrinsics;
		} else {
			intrinsics = depth_intrinsics;
		}
		intrinsics2CameraInfo(camera_info, intrinsics);
	}
}

/**
 * @brief Convert realsense intrinsics to ros camera info message.
 * @param camera_info Ros camera info message.
 * @param intrinsics Realsense instrinsics
 */
void Realsense::intrinsics2CameraInfo(sensor_msgs::msg::CameraInfo& camera_info, const rs2_intrinsics& intrinsics) {
	camera_info.width = static_cast<unsigned>(intrinsics.width);
	camera_info.height = static_cast<unsigned>(intrinsics.height);
	camera_info.header.frame_id = "optical_frame_id";

	// Intrinsic camera matrix for the raw (distorted) images
	//     [fx  0 cx]
	// K = [ 0 fy cy]
	//     [ 0  0  1]
	camera_info.k.at(0) = static_cast<double>(intrinsics.fx);  // Fx
	camera_info.k.at(1) = 0;
	camera_info.k.at(2) = static_cast<double>(intrinsics.ppx);  // Cx
	camera_info.k.at(3) = 0;
	camera_info.k.at(4) = static_cast<double>(intrinsics.fy);   // Fy
	camera_info.k.at(5) = static_cast<double>(intrinsics.ppy);  // Cy
	camera_info.k.at(6) = 0;
	camera_info.k.at(7) = 0;
	camera_info.k.at(8) = 1;

	// Projection/camera matrix
	//     [fx'  0  cx' Tx]
	// P = [ 0  fy' cy' Ty]
	//     [ 0   0   1   0]
	camera_info.p.at(0) = camera_info.k.at(0);
	camera_info.p.at(1) = 0;
	camera_info.p.at(2) = camera_info.k.at(2);
	camera_info.p.at(3) = 0;  // Tx for stereo camera
	camera_info.p.at(4) = 0;
	camera_info.p.at(5) = camera_info.k.at(4);
	camera_info.p.at(6) = camera_info.k.at(5);
	camera_info.p.at(7) = 0;  // Ty for stereo camera
	camera_info.p.at(8) = 0;
	camera_info.p.at(9) = 0;
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
	if (intrinsics.model == RS2_DISTORTION_KANNALA_BRANDT4) {
		camera_info.distortion_model = "equidistant";
	} else {
		camera_info.distortion_model = "plumb_bob";
	}

	// The distortion parameters, size depending on the distortion model.
	// For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
	camera_info.d.resize(5);
	for (unsigned long i = 0; i < 5; i++) {
		camera_info.d.at(i) = static_cast<double>(intrinsics.coeffs[i]);
	}
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
rs2::stream_profile Realsense::getStreamProfile(rs2::sensor sensor, int width, int height, rs2_format format, int fps) {
	rs2::stream_profile stream_profile;
	for (auto profile : sensor.get_stream_profiles()) {
		if (profile.as<rs2::video_stream_profile>().width() == width &&
		    profile.as<rs2::video_stream_profile>().height() == height && profile.fps() == fps &&
		    profile.format() == format)
			stream_profile = profile;
	}
	return stream_profile;
}

/**
 * @brief Callback for RealSense library.
 * @param frame Camera images as frameset
 */
void Realsense::rsCallback(const rs2::frame& frame) {
	if (rs2::frameset frameset = frame.as<rs2::frameset>()) {
		const rs2::depth_frame& depth_frame = frameset.get_depth_frame();
		const rs2::video_frame& color_frame = frameset.get_color_frame();

		if (align) {
			if (debug) timer = hires_clock::now();
			frameset = filter_align_to_color->process(frameset);
			if (debug) {
				double duration = (hires_clock::now() - timer).count() / 1e6;
				std::cout << "| rs align duration:            " << duration << " ms" << std::endl;
			}
		}

		// Filter depth frame
		rs2::frame filtered;
		if (filter) {
			/*
			The implemented flow of the filters pipeline is in the following order:
			1. apply decimation filter
			2. apply threshold filter
			3. transform the scene into disparity domain
			4. apply spatial filter
			5. apply temporal filter
			6. revert the results back (if step Disparity filter was applied
			to depth domain (each post processing block is optional and can be applied independantly).
			*/
			filtered = frameset.get_depth_frame();
			time_point filter_timer = hires_clock::now();
			timer = hires_clock::now();
			double filter_duration = 0.0;
			// Theshold filter
			filtered = thr_filter.process(filtered);
			if (debug) {
				filter_duration = (hires_clock::now() - timer).count() / 1e6;
				std::cout << "| rs threshold filter duration:     " << filter_duration << " ms" << std::endl;
			}
			// Depth to disparity
			timer = hires_clock::now();
			filtered = depth_to_disparity.process(filtered);
			if (debug) {
				filter_duration = (hires_clock::now() - timer).count() / 1e6;
				std::cout << "| rs depth to disparity duration:   " << filter_duration << " ms" << std::endl;
			}
			/*
			// Spatial filter (has long processing time)
			timer = hires_clock::now();
			filtered = spat_filter.process(filtered);
			if (debug) {
				filter_duration = (hires_clock::now() - timer).count() / 1e6;
				std::cout << "| rs spatial filter duration:       " << filter_duration << " ms" << std::endl;
			}
			*/
			// Temporal filter
			timer = hires_clock::now();
			filtered = temp_filter.process(filtered);
			if (debug) {
				filter_duration = (hires_clock::now() - timer).count() / 1e6;
				std::cout << "| rs temporal filter duration:      " << filter_duration << " ms" << std::endl;
			}
			// Disparity to depth
			timer = hires_clock::now();
			filtered = disparity_to_depth.process(filtered);
			if (debug) {
				filter_duration = (hires_clock::now() - timer).count() / 1e6;
				std::cout << "| rs disparity to depth duration:   " << filter_duration << " ms" << std::endl;
				double all_filter_duration = (hires_clock::now() - filter_timer).count() / 1e6;
				std::cout << "| rs all filter duration:           " << all_filter_duration << " ms" << std::endl << std::endl;
			}
		}

		double timestamp;
		if (frameset.get_frame_timestamp_domain() == RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK) {
			double elapsed_camera_time = (frameset.get_timestamp() * 1e6) - camera_time_base;
			timestamp = (system_time_base + elapsed_camera_time) / 1e6;
		} else {
			timestamp = frameset.get_timestamp();
		}

		if (filter) {
			framesCallback(reinterpret_cast<const uint8_t*>(color_frame.get_data()),
			               reinterpret_cast<const uint16_t*>(filtered.get_data()), timestamp);
		} else {
			framesCallback(reinterpret_cast<const uint8_t*>(color_frame.get_data()),
			               reinterpret_cast<const uint16_t*>(depth_frame.get_data()), timestamp);
		}
	}
}
