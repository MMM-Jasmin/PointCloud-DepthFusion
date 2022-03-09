// SYSTEM
#include <atomic>
#include <iostream>
#include <thread>
#include <fstream>
// ROS2
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/image.hpp>
// PROJECT
#include "camera_node.hpp"

using namespace std::chrono_literals;

/**
 * @brief Constructor.
 */
CameraNode::CameraNode(const std::string &name) : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true)) {
	this->node_name = name;
	package_share_directory = ament_index_cpp::get_package_share_directory("camera_node");
	config = new Config(this);
	realsense = new Realsense();

	this->declare_parameter<std::string>("topic_depth", "left_depth");
	this->get_parameter("topic_depth", topic_depth);
	this->declare_parameter<bool>("use_rs_align", true);
	this->get_parameter("use_rs_align", use_rs_align);
	this->declare_parameter<std::string>("topic_frameset", node_name + "/frameset");
	this->get_parameter("topic_frameset", topic_frameset);
	this->declare_parameter<bool>("profiling.enable_profiling", false);
	this->get_parameter("profiling.enable_profiling", enable_profiling);
	this->declare_parameter<int>("profiling.log_size", 400);
	this->get_parameter("profiling.log_size", profiling_size);
	this->declare_parameter<std::string>("profiling.filename", node_name + "_profiling.txt");
	this->get_parameter("profiling.filename", profiling_filename);
	this->declare_parameter<bool>("use_rs_timestamp", true);
	this->get_parameter("use_rs_timestamp", use_rs_timestamp);
}

/**
 * @brief Destructor.
 */
CameraNode::~CameraNode() {
	realsense->stop();

	if (realsense != nullptr) delete realsense;
	if (config != nullptr) delete config;
}

/**
 * @brief Initialize camera node.
 */
void CameraNode::init() {
	// Register realsense parameter callback for setting ros parameters
	config->registerRealsenseParameterCallback(
	    std::bind(&Realsense::setOptionFromParameter, realsense, std::placeholders::_1, std::placeholders::_2));
	config->declareNodeParameters();

	this->verbose = config->verbose;
	this->debug = config->debug;

	// Configure realsense module
	realsense->setExitSignal(exit_request);
	realsense->setDebug(config->enable_rs_debug);
	realsense->setVerbosity(config->verbose);
	realsense->setSimulation(config->simulation);
	realsense->setAlign(use_rs_align);
	realsense->setDepthScale(config->depth_scale);
	realsense->setDepthMax(config->max_depth);
	realsense->setUseQueue(use_rs_queue);
	if (!config->simulation) {
		// Initialize realsense camera
		realsense->init(config->camera_serial_no);
		if (exit_request->load()) return;
		// Declare realsense parameters
		realsense->declareRosParameters(this);
	} else {
		if (debug) std::cout << "simulate images" << std::endl;
		// Setup image simulation
		// Unregister parameter callback
		config->registerRealsenseParameterCallback(nullptr);
		// Set framerate
		realsense->setSimulationFramerate(config->simulation_framerate);
		// Set depth scale factor
		realsense->setDepthScale(static_cast<float>(config->sim_depth_scale));
		config->depth_scale = static_cast<float>(config->sim_depth_scale);
		// Load images for simulation
		realsense->loadImageFiles(config->color_image_filename, config->depth_image_filename);
		// Load intrinsics files for simulation
		realsense->loadIntrinsicsFiles(config->color_intrinsics_filename, config->depth_intrinsics_filename);
		if (exit_request->load()) return;
	}

	// Get camera sensor intrinsics
	rs2_intrinsics rs_color_intrinsics = realsense->getColorIntrinsics();
	rs2_intrinsics rs_depth_intrinsics = realsense->getDepthIntrinsics();
	std::copy(reinterpret_cast<uint8_t *>(&rs_color_intrinsics),
	          reinterpret_cast<uint8_t *>(&rs_color_intrinsics) + sizeof(rs2_intrinsics),
	          reinterpret_cast<uint8_t *>(&color_intrinsics));
	std::copy(reinterpret_cast<uint8_t *>(&rs_depth_intrinsics),
	          reinterpret_cast<uint8_t *>(&rs_depth_intrinsics) + sizeof(rs2_intrinsics),
	          reinterpret_cast<uint8_t *>(&depth_intrinsics));

	// Get camera depth to color sensor extrinsics
	rs2_extrinsics rs_extrinsics = realsense->getDepthToColorExtrinsics();
	std::copy(reinterpret_cast<uint8_t *>(&rs_extrinsics),
	          reinterpret_cast<uint8_t *>(&rs_extrinsics) + sizeof(rs2_extrinsics),
	          reinterpret_cast<uint8_t *>(&extrinsics));

	// Get camera info messages
	realsense->getColorCameraInfo(color_camerainfo);
	realsense->getDepthCameraInfo(depth_camerainfo);

	// Quality of service
	if (config->qos_sensor_data) qos_profile = rclcpp::SensorDataQoS();
	qos_profile = qos_profile.keep_last(static_cast<size_t>(config->qos_history_depth));

	frameset_publisher = this->create_publisher<camera_interfaces::msg::DepthFrameset>(topic_frameset, qos_profile);
	publish_timer = this->create_wall_timer(std::chrono::nanoseconds(static_cast<int>(1e9 / 30)),
	                                        std::bind(&CameraNode::publishFrameset, this));

	std::string topic_color = std::string(this->get_name()) + "/" + config->topic_color;
	std::string topic_depth = std::string(this->get_name()) + "/" + config->topic_depth;
	std::string topic_points = std::string(this->get_name()) + "/" + config->topic_points;
	std::string topic_fused = std::string(this->get_name()) + "/" + config->topic_fused;

	// Create camera parameter service
	service = this->create_service<camera_interfaces::srv::GetCameraParameters>(
	    node_name + "/get_camera_parameters",
	    std::bind(&CameraNode::getCameraParameters, this, std::placeholders::_1, std::placeholders::_2));

	// Depth image publisher
	depth_publisher = image_transport::create_camera_publisher(this, topic_depth, qos_profile.get_rmw_qos_profile());

	// Allocate frames
	color_frame = reinterpret_cast<uint8_t *>(
	    malloc(static_cast<unsigned>(color_intrinsics.width * color_intrinsics.height) * 3 * sizeof(uint8_t)));
	depth_frame = reinterpret_cast<uint16_t *>(
	    malloc(static_cast<unsigned>(depth_intrinsics.width * depth_intrinsics.height) * sizeof(uint16_t)));

	// Start realsense camera
	if (!config->simulation) {
		realsense->start();
	}
}

/**
 * @brief Stop RealSense camera.
 */
void CameraNode::stop() {
	// Stop realsense camera
	if (!config->simulation) {
		realsense->stop();
	}
}

/**
 * @brief Convert librealsense intrinsics to own intrinsics structure.
 * @param rs_intrinsics Librealsense intrinsics
 * @param intrinsics Own intrinsics structure
 */
void CameraNode::convertRs2Intrinsics(const rs2_intrinsics &rs_intrinsics, Intrinsics &intrinsics) {
	intrinsics.width = rs_intrinsics.width;
	intrinsics.height = rs_intrinsics.height;
	intrinsics.ppx = rs_intrinsics.ppx;
	intrinsics.ppy = rs_intrinsics.ppy;
	intrinsics.fx = rs_intrinsics.fx;
	intrinsics.fy = rs_intrinsics.fy;
	intrinsics.model = static_cast<Distortion>(static_cast<int>(rs_intrinsics.model));
	for (uint i = 0; i < 5; i++) {
		intrinsics.coeffs[i] = rs_intrinsics.coeffs[i];
	}
}

/**
 * @brief Start profiling timer.
 */
void CameraNode::startTimer() { timer = hires_clock::now(); }

/**
 * @brief Stop profiling timer.
 * @param msg Message to display
 */
void CameraNode::stopTimer(std::string msg) {
	if (debug && config->profiling) {
		cudaDeviceSynchronize();
		double duration = (hires_clock::now() - timer).count() / 1e06;
		int max_msg_length = 32;
		msg = "| " + msg + ":";
		int num_spaces = (max_msg_length - static_cast<int>(msg.length()));
		if (num_spaces < 1) num_spaces = 1;
		msg.append(std::string(static_cast<uint>(num_spaces), ' '));
		std::cout << msg << std::setprecision(4) << duration << " ms" << std::endl;
	}
}

/**
 * @brief Initialize node for point cloud publishing.
 */
void CameraNode::initPointcloud() {
	// Register realsense parameter callback for setting ros parameters
	config->registerRealsenseParameterCallback(
	    std::bind(&Realsense::setOptionFromParameter, realsense, std::placeholders::_1, std::placeholders::_2));
	config->declareNodeParameters();

	this->verbose = config->verbose;
	this->debug = config->debug;

	// Configure realsense module
	realsense->setExitSignal(exit_request);
	realsense->setDebug(config->enable_rs_debug);
	realsense->setVerbosity(config->verbose);
	realsense->setSimulation(config->simulation);
	realsense->setAlign(use_rs_align);
	realsense->setDepthScale(config->depth_scale);
	realsense->setDepthMax(config->max_depth);
	realsense->setUseQueue(use_rs_queue);
	if (!config->simulation) {
		// Initialize realsense camera
		realsense->init(config->camera_serial_no);
		if (exit_request->load()) return;
		// Declare realsense parameters
		realsense->declareRosParameters(this);
	} else {
		// Setup image simulation
		// Unregister parameter callback
		config->registerRealsenseParameterCallback(nullptr);
		// Set framerate
		realsense->setSimulationFramerate(config->simulation_framerate);
		// Set depth scale factor
		realsense->setDepthScale(static_cast<float>(config->sim_depth_scale));
		config->depth_scale = static_cast<float>(config->sim_depth_scale);
		// Load images for simulation
		realsense->loadImageFiles(config->color_image_filename, config->depth_image_filename);
		// Load intrinsics files for simulation
		realsense->loadIntrinsicsFiles(config->color_intrinsics_filename, config->depth_intrinsics_filename);
		if (exit_request->load()) return;
	}

	// Get camera sensor intrinsics
	rs2_intrinsics rs_color_intrinsics = realsense->getColorIntrinsics();
	rs2_intrinsics rs_depth_intrinsics = realsense->getDepthIntrinsics();
	std::copy(reinterpret_cast<uint8_t *>(&rs_color_intrinsics),
	          reinterpret_cast<uint8_t *>(&rs_color_intrinsics) + sizeof(rs2_intrinsics),
	          reinterpret_cast<uint8_t *>(&color_intrinsics));
	std::copy(reinterpret_cast<uint8_t *>(&rs_depth_intrinsics),
	          reinterpret_cast<uint8_t *>(&rs_depth_intrinsics) + sizeof(rs2_intrinsics),
	          reinterpret_cast<uint8_t *>(&depth_intrinsics));

	// Get camera depth to color sensor extrinsics
	rs2_extrinsics rs_extrinsics = realsense->getDepthToColorExtrinsics();
	std::copy(reinterpret_cast<uint8_t *>(&rs_extrinsics),
	          reinterpret_cast<uint8_t *>(&rs_extrinsics) + sizeof(rs2_extrinsics),
	          reinterpret_cast<uint8_t *>(&extrinsics));

	// Get camera info messages
	realsense->getColorCameraInfo(color_camerainfo);
	realsense->getDepthCameraInfo(depth_camerainfo);

	// Quality of service
	size_t qos_history_depth = static_cast<size_t>(config->qos_history_depth);
	rmw_qos_profile_t image_rmw_qos_profile = rmw_qos_profile_default;
	rmw_qos_profile_t cloud_rmw_qos_profile = rmw_qos_profile_default;
	if (config->qos_sensor_data) {
		image_rmw_qos_profile = rmw_qos_profile_sensor_data;
		cloud_rmw_qos_profile = rmw_qos_profile_sensor_data;
	}
	image_rmw_qos_profile.depth = qos_history_depth;
	cloud_rmw_qos_profile.depth = qos_history_depth;
	rclcpp::QoS cloud_qos_profile =
	    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(cloud_rmw_qos_profile), cloud_rmw_qos_profile);

	// Create publish timer
	callback_group_timer = this->create_callback_group(CallbackGroupType::MutuallyExclusive);
	publish_cloud_timer = this->create_wall_timer(std::chrono::nanoseconds(static_cast<int>(1e9 / 30)),
	                                              std::bind(&CameraNode::processPointcloud, this), callback_group_timer);

	// Pointcloud publisher
	cloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(config->topic_points, cloud_qos_profile);

	// Depth image publisher
	depth_publisher = image_transport::create_camera_publisher(this, config->topic_depth, image_rmw_qos_profile);

	// Allocate depth message
	depth_msg = sensor_msgs::msg::Image::SharedPtr(new sensor_msgs::msg::Image);

	// Init cuda
	Kernels::init_cuda(config->verbose);
	cudaStreamCreateWithFlags(&cuda_stream, cudaStreamNonBlocking);

	// Allocate frames
	color_frame = reinterpret_cast<uint8_t *>(
	    malloc(static_cast<unsigned>(color_intrinsics.width * color_intrinsics.height) * 3 * sizeof(uint8_t)));
	depth_frame = reinterpret_cast<uint16_t *>(
	    malloc(static_cast<unsigned>(depth_intrinsics.width * depth_intrinsics.height) * sizeof(uint16_t)));
	frameset.setStream(&cuda_stream);
	frameset.allocateColorFrame(color_intrinsics);
	frameset.allocateDepthFrameUnaligned(depth_intrinsics, extrinsics);
	frameset.allocateDepthFrame(color_intrinsics);

	// Set image saving prefix
	std::string file_prefix = config->node_namespace;
	file_prefix.erase(std::remove(file_prefix.begin(), file_prefix.end(), '/'), file_prefix.end());
	frameset.setSaveImages(config->save_data, package_share_directory + "/data", file_prefix);
}

/**
 * @brief Generate and publish point cloud from fetched frames.
 */
void CameraNode::processPointcloud() {
	if (exit_request->load()) return;
	unsigned timeout = 30;  // milliseconds
	double frame_timestamp = 0.;
	if (realsense->getFrames(color_frame, depth_frame, frame_timestamp, timeout)) {
		// Align depth to color frame
		startTimer();
		frameset.setColorFrame(color_frame);
		stopTimer("set color frame");
		if (use_rs_align) {
			startTimer();
			frameset.setDepthFrame(depth_frame);
			stopTimer("set depth frame");
		} else {
			startTimer();
			frameset.setDepthFrameUnaligned(depth_frame);
			stopTimer("set depth frame");
			startTimer();
			frameset.alignDepthToColor(config->depth_scale);
			stopTimer("align depth frame");
		}

		// Filter depth
		startTimer();
		frameset.filterDepth(config->min_depth, config->max_depth, config->depth_scale, config->roi);
		stopTimer("filter depth");

		// Create pointcloud
		startTimer();
		Pointcloud cloud;
		cloud.setStream(frameset.getStream());
		cloud.deproject(frameset, config->depth_scale);
		stopTimer("create pointcloud");

		double timestamp = frame_timestamp * 1e6;
		publishPointcloud(cloud, timestamp);

		publishDepth();
	}
}

/**
 * @brief Publish point cloud.
 */
void CameraNode::publishPointcloud(Pointcloud &cloud, double &timestamp) {
	rclcpp::Time ros_timestamp(static_cast<int64_t>(timestamp));
	unsigned point_count = static_cast<unsigned>(cloud.getPointCount());

	sensor_msgs::msg::PointCloud2::UniquePtr cloud_msg(new sensor_msgs::msg::PointCloud2());

	sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
	modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
	                              sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
	                              "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
	modifier.resize(point_count);
	cloud_msg->header.frame_id = "camera_left_color_optical_frame";
	cloud_msg->header.stamp = ros_timestamp;
	cloud_msg->width = point_count;
	cloud_msg->height = 1;
	cloud_msg->is_bigendian = false;
	cloud_msg->is_dense = false;
	cloud_msg->point_step = 16;
	cloud_msg->row_step = cloud_msg->width * cloud_msg->point_step;
	cloud.copyToHost(reinterpret_cast<float *>(cloud_msg->data.data()));
	stopTimer("create pointcloud msg");

	// Publish pointcloud
	if (debug) std::cout << "| cloud size:                   " << cloud_msg->width * cloud_msg->height << std::endl;

	cloud_publisher->publish(std::move(cloud_msg));
}

/**
 * @brief Publish depth image.
 */
void CameraNode::publishDepth() {
	rclcpp::Time ros_timestamp = this->now();

	// Publish depth image
	const uint8_t *depth_frame_bytes = reinterpret_cast<const uint8_t *>(depth_frame);
	depth_msg->header.frame_id = "camera_left_color_optical_frame";
	depth_msg->header.stamp = ros_timestamp;
	depth_msg->width = static_cast<uint>(depth_intrinsics.width);
	depth_msg->height = static_cast<uint>(depth_intrinsics.height);
	depth_msg->is_bigendian = false;
	depth_msg->step = depth_msg->width * sizeof(uint16_t);
	depth_msg->encoding = "mono16";
	depth_msg->data.assign(depth_frame_bytes, depth_frame_bytes + (depth_msg->step * depth_msg->height));
	depth_publisher.publish(*depth_msg, depth_camerainfo);
}

/**
 * @brief Callback for ROS parameter change.
 * @param parameters Vector of ROS parameters.
 * @return Success on parameter change
 */
rcl_interfaces::msg::SetParametersResult CameraNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
	for (const auto &param : parameters) {
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
	}

	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	result.reason = "success";
	return result;
}

/**
 * @brief Publish frameset consisting of depth and color frame.
 */
void CameraNode::publishFrameset() {
	if (exit_request->load() || !rclcpp::ok()) {
		publish_timer.get()->cancel();
		realsense->stop();
		return;
	}
	time_point callback_start = hires_clock::now();

	// Image sizes
	int depth_width = depth_intrinsics.width;
	int depth_height = depth_intrinsics.height;
	int color_width = color_intrinsics.width;
	int color_height = color_intrinsics.height;

	// Get frames
	unsigned timeout = 60;
	double timestamp;
	time_point getframes_start = hires_clock::now();
	realsense->getFrames(color_frame, depth_frame, timestamp, timeout);
	double getframes_duration = (hires_clock::now() - getframes_start).count() / 1e6;

	if (depth_frame == nullptr || color_frame == nullptr) {
		if (debug) std::cout << "no frame received in timelimit of " << timeout << " ms" << std::endl;
		return;
	}

	// Set timestamp
	rclcpp::Time ros_timestamp = this->now();  // Node timestamp
	if (use_rs_timestamp) {
		ros_timestamp = rclcpp::Time(int64_t(timestamp * 1e6), RCL_ROS_TIME);  // Camera timestamp
	}
	double latency_in = (this->now() - ros_timestamp).to_chrono<std::chrono::nanoseconds>().count() / 1e6;

	// Cast frames
	uint8_t *depth_frame_bytes = reinterpret_cast<uint8_t *>(depth_frame);
	uint8_t *color_frame_bytes = reinterpret_cast<uint8_t *>(color_frame);

	time_point messages_start = hires_clock::now();

	sensor_msgs::msg::Image depth_msg;
	depth_msg.header.frame_id = "camera_left_color_optical_frame";
	depth_msg.header.stamp = ros_timestamp;
	depth_msg.width = static_cast<uint>(depth_width);
	depth_msg.height = static_cast<uint>(depth_height);
	depth_msg.is_bigendian = false;
	depth_msg.step = depth_msg.width * sizeof(uint16_t);
	depth_msg.encoding = "mono16";
	depth_msg.data.assign(depth_frame_bytes, depth_frame_bytes + (depth_msg.step * depth_msg.height));

	sensor_msgs::msg::Image color_msg;
	color_msg.header.frame_id = "camera_left_color_optical_frame";
	color_msg.header.stamp = ros_timestamp;
	color_msg.width = static_cast<uint>(color_width);
	color_msg.height = static_cast<uint>(color_height);
	color_msg.is_bigendian = false;
	color_msg.step = color_msg.width * 3 * sizeof(uint8_t);
	color_msg.encoding = "rgb8";
	color_msg.data.assign(color_frame_bytes, color_frame_bytes + (color_msg.step * color_msg.height));

	camera_interfaces::msg::DepthFrameset::UniquePtr frameset_msg(new camera_interfaces::msg::DepthFrameset());
	frameset_msg->header.frame_id = "camera_left_color_optical_frame";
	frameset_msg->header.stamp = ros_timestamp;
	frameset_msg->depth_image = depth_msg;
	frameset_msg->color_image = color_msg;

	double message_duration = (hires_clock::now() - messages_start).count() / 1e6;

	time_point publish_start = hires_clock::now();

	// Publish frameset
	frameset_publisher->publish(std::move(frameset_msg));

	// Publish depth frame
	depth_publisher.publish(depth_msg, depth_camerainfo);

	double publish_duration = (hires_clock::now() - publish_start).count() / 1e6;

	if (debug) {
		double callback_duration = (hires_clock::now() - callback_start).count() / 1e6;
		double timer_duration = (hires_clock::now() - timer).count() / 1e6;
		timer = hires_clock::now();
		fps_avg = (fps_avg + (1000 / timer_duration)) / 2;
		std::cout << "callback: " << callback_duration << " ms" << std::endl;
		std::cout << "frames:   " << getframes_duration << " ms" << std::endl;
		std::cout << "message:  " << message_duration << " ms" << std::endl;
		std::cout << "publish:  " << publish_duration << " ms" << std::endl;
		std::cout << "duration: " << timer_duration << " ms" << std::endl;
		std::cout << "fps:      " << 1000 / timer_duration << std::endl;
		std::cout << "avg fps:  " << fps_avg << std::endl;
		std::cout << std::endl;
	}

	if (enable_profiling) {
		double callback_duration = (hires_clock::now() - callback_start).count() / 1e6;
		double timer_duration = (hires_clock::now() - timer).count() / 1e6;
		timer = hires_clock::now();
		double latency_out = (this->now() - ros_timestamp).to_chrono<std::chrono::nanoseconds>().count() / 1e6;
		std::vector<double> vec;
		vec.push_back(timer_duration);
		vec.push_back(callback_duration);
		vec.push_back(getframes_duration);
		vec.push_back(message_duration);
		vec.push_back(publish_duration);
		vec.push_back(latency_in);
		vec.push_back(latency_out);
		profiling_vec.push_back(vec);

		if (profiling_vec.size() == profiling_size) {
			std::cout << "Writing profiline file " << profiling_filename << std::endl;
			std::ofstream profiling_file(profiling_filename);
			for (int i = 0; i < profiling_fields.size(); i++) {
				profiling_file << profiling_fields[i];
				if (i != profiling_fields.size() - 1) profiling_file << ",";
			}
			profiling_file << "\n";
			for (const auto &l : profiling_vec) {
				for (int i = 0; i < l.size(); i++) {
					profiling_file << l[i];
					if (i != l.size() - 1) profiling_file << ",";
				}
				profiling_file << "\n";
			}
			profiling_vec.clear();
		}
	}
}

/**
 * @brief ROS service for camera parameter retrieval.
 * @param request Service request
 * @param response Service response
 */
void CameraNode::getCameraParameters(
    const std::shared_ptr<camera_interfaces::srv::GetCameraParameters::Request> request,
    std::shared_ptr<camera_interfaces::srv::GetCameraParameters::Response> response) {
	response->depth_intrinsics = depth_camerainfo;
	response->color_intrinsics = color_camerainfo;
	for (unsigned long i = 0; i < 3; i++) {
		response->extrinsic_translation[i] = extrinsics.translation[i];
	}
	for (unsigned long i = 0; i < 9; i++) {
		response->extrinsic_rotation[i] = extrinsics.rotation[i];
	}
}
