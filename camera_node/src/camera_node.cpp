// SYSTEM
#include <atomic>
#include <fstream>
#include <iostream>
#include <thread>
// ROS2
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/image.hpp>
// PROJECT
#include "camera_node.hpp"

using namespace std::chrono_literals;

/**
 * @brief Constructor.
 */
CameraNode::CameraNode(const std::string &name) :
	Node(name, rclcpp::NodeOptions().use_intra_process_comms(true)),
	m_package_share_directory(),
	m_color_publisher(),
	m_depth_publisher(),
	m_last_frame_timestamp(),
	m_color_intrinsics(),
	m_depth_intrinsics(),
	m_extrinsics(),
	m_color_camerainfo(),
	m_depth_camerainfo(),
	m_frameset(),
	m_cuda_stream(),
	m_publish_cloud_timer(),
	m_callback_group_timer(),
	m_profiling_vec()
{
	m_node_name               = name;
	m_package_share_directory = ament_index_cpp::get_package_share_directory("camera_node");
	m_pConfig                 = new Config(this);
	m_pRealsense              = new Realsense();

	this->declare_parameter<bool>("use_rs_align", true);
	this->get_parameter("use_rs_align", m_use_rs_align);
	this->declare_parameter<bool>("profiling.enable_profiling", false);
	this->get_parameter("profiling.enable_profiling", m_enable_profiling);
	this->declare_parameter<int>("profiling.log_size", 400);
	this->get_parameter("profiling.log_size", m_profiling_size);
	this->declare_parameter<std::string>("profiling.filename", m_node_name + "_profiling.txt");
	this->get_parameter("profiling.filename", m_profiling_filename);
	this->declare_parameter<bool>("use_rs_timestamp", true);
	this->get_parameter("use_rs_timestamp", m_use_rs_timestamp);
}

/**
 * @brief Destructor.
 */
CameraNode::~CameraNode()
{
	m_pRealsense->stop();

	if (m_pRealsense != nullptr) delete m_pRealsense;
	if (m_pConfig != nullptr) delete m_pConfig;
}

/**
 * @brief Initialize camera node.
 */
void CameraNode::init()
{
	// Register Realsense parameter callback for setting ros parameters
	m_pConfig->registerRealsenseParameterCallback(std::bind(&Realsense::setOptionFromParameter, m_pRealsense, std::placeholders::_1, std::placeholders::_2));
	m_pConfig->declareNodeParameters();

	m_verbose = m_pConfig->verbose();
	m_debug   = m_pConfig->debug();

	// Configure Realsense module
	m_pRealsense->setExitSignal(m_pExit_request);
	m_pRealsense->setDebug(m_pConfig->enable_rs_debug());
	m_pRealsense->setVerbosity(m_pConfig->verbose());
	m_pRealsense->setSimulation(m_pConfig->simulation());
	m_pRealsense->setAlign(m_use_rs_align);
	m_pRealsense->setDepthScale(m_pConfig->depth_scale());
	m_pRealsense->setDepthMax(m_pConfig->max_depth());
	m_pRealsense->setUseQueue(m_use_rs_queue);
	if (!m_pConfig->simulation())
	{
		// Initialize Realsense camera
		m_pRealsense->init(m_pConfig->camera_serial_no());
		if (m_pExit_request->load()) return;
		// Declare Realsense parameters
		m_pRealsense->declareRosParameters(this);
	}
	else
	{
		if (m_debug) std::cout << "simulate images" << std::endl;
		// Setup image simulation
		// Unregister parameter callback
		m_pConfig->registerRealsenseParameterCallback(nullptr);
		// Set framerate
		m_pRealsense->setSimulationFramerate(m_pConfig->simulation_framerate());
		// Set depth scale factor
		m_pRealsense->setDepthScale(static_cast<float>(m_pConfig->sim_depth_scale()));
		m_pConfig->setDepthScale(static_cast<float>(m_pConfig->sim_depth_scale()));
		// Load images for simulation
		m_pRealsense->loadImageFiles(m_pConfig->color_image_filename(), m_pConfig->depth_image_filename());
		// Load intrinsics files for simulation
		m_pRealsense->loadIntrinsicsFiles(m_pConfig->color_intrinsics_filename(), m_pConfig->depth_intrinsics_filename());
		if (m_pExit_request->load()) return;
	}

	// Get camera sensor intrinsics
	rs2_intrinsics rs_color_intrinsics = m_pRealsense->getColorIntrinsics();
	rs2_intrinsics rs_depth_intrinsics = m_pRealsense->getDepthIntrinsics();
	std::copy(reinterpret_cast<uint8_t *>(&rs_color_intrinsics),
			  reinterpret_cast<uint8_t *>(&rs_color_intrinsics) + sizeof(rs2_intrinsics),
			  reinterpret_cast<uint8_t *>(&m_color_intrinsics));
	std::copy(reinterpret_cast<uint8_t *>(&rs_depth_intrinsics),
			  reinterpret_cast<uint8_t *>(&rs_depth_intrinsics) + sizeof(rs2_intrinsics),
			  reinterpret_cast<uint8_t *>(&m_depth_intrinsics));

	// Get camera depth to color sensor extrinsics
	rs2_extrinsics rs_extrinsics = m_pRealsense->getDepthToColorExtrinsics();
	std::copy(reinterpret_cast<uint8_t *>(&rs_extrinsics),
			  reinterpret_cast<uint8_t *>(&rs_extrinsics) + sizeof(rs2_extrinsics),
			  reinterpret_cast<uint8_t *>(&m_extrinsics));

	// Get camera info messages
	m_pRealsense->getColorCameraInfo(m_color_camerainfo);
	m_pRealsense->getDepthCameraInfo(m_depth_camerainfo);

	// Quality of service
	if (m_pConfig->qos_sensor_data()) m_qos_profile = rclcpp::SensorDataQoS();
	m_qos_profile = m_qos_profile.keep_last(static_cast<size_t>(m_pConfig->qos_history_depth()));

	std::string topic_color    = std::string(this->get_name()) + "/" + m_pConfig->topic_color();
	std::string topic_depth    = std::string(this->get_name()) + "/" + m_pConfig->topic_depth();
	std::string topic_frameset = std::string(this->get_name()) + "/" + m_pConfig->topic_frameset();

	m_frameset_publisher = this->create_publisher<camera_interfaces::msg::DepthFrameset>(topic_frameset, m_qos_profile);
	m_publish_timer      = this->create_wall_timer(std::chrono::nanoseconds(static_cast<int>(1e9 / 30)), std::bind(&CameraNode::publishFrameset, this));

	// Create camera parameter service
	m_service = this->create_service<camera_interfaces::srv::GetCameraParameters>(m_node_name + "/get_camera_parameters",std::bind(&CameraNode::getCameraParameters, this, std::placeholders::_1, std::placeholders::_2));

	// Depth image publisher
	m_depth_publisher = image_transport::create_camera_publisher(this, topic_depth, m_qos_profile.get_rmw_qos_profile());

	// Allocate frames
	m_pColor_frame = reinterpret_cast<uint8_t *>(malloc(static_cast<unsigned>(m_color_intrinsics.width * m_color_intrinsics.height) * 3 * sizeof(uint8_t)));
	m_pDepth_frame = reinterpret_cast<uint16_t *>(malloc(static_cast<unsigned>(m_depth_intrinsics.width * m_depth_intrinsics.height) * sizeof(uint16_t)));

	// Start m_pRealsense camera
	if (!m_pConfig->simulation())
		m_pRealsense->start();
}

/**
 * @brief Stop RealSense camera.
 */
void CameraNode::stop()
{
	// Stop m_pRealsense camera
	if (!m_pConfig->simulation())
		m_pRealsense->stop();
}

/**
 * @brief Convert libm_pRealsense intrinsics to own intrinsics structure.
 * @param rs_intrinsics Libm_pRealsense intrinsics
 * @param intrinsics Own intrinsics structure
 */
void CameraNode::convertRs2Intrinsics(const rs2_intrinsics &rs_intrinsics, Intrinsics &intrinsics) const
{
	intrinsics.width  = rs_intrinsics.width;
	intrinsics.height = rs_intrinsics.height;
	intrinsics.ppx    = rs_intrinsics.ppx;
	intrinsics.ppy    = rs_intrinsics.ppy;
	intrinsics.fx     = rs_intrinsics.fx;
	intrinsics.fy     = rs_intrinsics.fy;
	intrinsics.model  = static_cast<Distortion>(static_cast<int>(rs_intrinsics.model));
	for (uint i = 0; i < 5; i++)
		intrinsics.coeffs[i] = rs_intrinsics.coeffs[i];
}

/**
 * @brief Start profiling timer.
 */
void CameraNode::startTimer()
{
	m_timer = hires_clock::now();
}

/**
 * @brief Stop profiling timer.
 * @param msg Message to display
 */
void CameraNode::stopTimer(std::string msg)
{
	if (m_debug && m_pConfig->profiling())
	{
		cudaDeviceSynchronize();
		double duration    = (hires_clock::now() - m_timer).count() / 1e06;
		int max_msg_length = 32;
		msg                = "| " + msg + ":";
		int num_spaces     = (max_msg_length - static_cast<int>(msg.length()));
		if (num_spaces < 1) num_spaces = 1;
		msg.append(std::string(static_cast<uint>(num_spaces), ' '));
		std::cout << msg << std::setprecision(4) << duration << " ms" << std::endl;
	}
}

/**
 * @brief Publish depth image.
 */
void CameraNode::publishDepth()
{
	rclcpp::Time ros_timestamp = this->now();

	// Publish depth image
	const uint8_t *depth_frame_bytes = reinterpret_cast<const uint8_t *>(m_pDepth_frame);
	m_depth_msg->header.frame_id     = "camera_left_color_optical_frame";
	m_depth_msg->header.stamp        = ros_timestamp;
	m_depth_msg->width               = static_cast<uint>(m_depth_intrinsics.width);
	m_depth_msg->height              = static_cast<uint>(m_depth_intrinsics.height);
	m_depth_msg->is_bigendian        = false;
	m_depth_msg->step                = m_depth_msg->width * sizeof(uint16_t);
	m_depth_msg->encoding            = "mono16";
	m_depth_msg->data.assign(depth_frame_bytes, depth_frame_bytes + (m_depth_msg->step * m_depth_msg->height));
	m_depth_publisher.publish(*m_depth_msg, m_depth_camerainfo);
}

/**
 * @brief Callback for ROS parameter change.
 * @param parameters Vector of ROS parameters.
 * @return Success on parameter change
 */
rcl_interfaces::msg::SetParametersResult CameraNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
	for (const auto &param : parameters)
	{
		std::string parameter_string = param.get_name();

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
	}

	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	result.reason     = "success";
	return result;
}

/**
 * @brief Publish frameset consisting of depth and color frame.
 */
void CameraNode::publishFrameset()
{
	if (m_pExit_request->load() || !rclcpp::ok())
	{
		m_publish_timer.get()->cancel();
		m_pRealsense->stop();
		return;
	}
	time_point callback_start = hires_clock::now();

	// Image sizes
	int depth_width  = m_depth_intrinsics.width;
	int depth_height = m_depth_intrinsics.height;
	int color_width  = m_color_intrinsics.width;
	int color_height = m_color_intrinsics.height;

	// Get frames
	unsigned timeout = 60;
	double timestamp;
	time_point getframes_start = hires_clock::now();
	m_pRealsense->getFrames(m_pColor_frame, m_pDepth_frame, timestamp, timeout);
	double getframes_duration = (hires_clock::now() - getframes_start).count() / 1e6;

	if (m_pDepth_frame == nullptr || m_pColor_frame == nullptr)
	{
		if (m_debug) std::cout << "no frame received in timelimit of " << timeout << " ms" << std::endl;
		return;
	}

	// Set timestamp
	rclcpp::Time ros_timestamp = this->now(); // Node timestamp
	if (m_use_rs_timestamp)
		ros_timestamp = rclcpp::Time(int64_t(timestamp * 1e6), RCL_ROS_TIME); // Camera timestamp

	double latency_in = (this->now() - ros_timestamp).to_chrono<std::chrono::nanoseconds>().count() / 1e6;

	// Cast frames
	uint8_t *depth_frame_bytes = reinterpret_cast<uint8_t *>(m_pDepth_frame);
	uint8_t *color_frame_bytes = reinterpret_cast<uint8_t *>(m_pColor_frame);

	time_point messages_start = hires_clock::now();

	sensor_msgs::msg::Image depth_msg;
	depth_msg.header.frame_id = "camera_left_color_optical_frame";
	depth_msg.header.stamp    = ros_timestamp;
	depth_msg.width           = static_cast<uint>(depth_width);
	depth_msg.height          = static_cast<uint>(depth_height);
	depth_msg.is_bigendian    = false;
	depth_msg.step            = depth_msg.width * sizeof(uint16_t);
	depth_msg.encoding        = "mono16";
	depth_msg.data.assign(depth_frame_bytes, depth_frame_bytes + (depth_msg.step * depth_msg.height));

	sensor_msgs::msg::Image color_msg;
	color_msg.header.frame_id = "camera_left_color_optical_frame";
	color_msg.header.stamp    = ros_timestamp;
	color_msg.width           = static_cast<uint>(color_width);
	color_msg.height          = static_cast<uint>(color_height);
	color_msg.is_bigendian    = false;
	color_msg.step            = color_msg.width * 3 * sizeof(uint8_t);
	color_msg.encoding        = "rgb8";
	color_msg.data.assign(color_frame_bytes, color_frame_bytes + (color_msg.step * color_msg.height));

	camera_interfaces::msg::DepthFrameset::UniquePtr frameset_msg(new camera_interfaces::msg::DepthFrameset());
	frameset_msg->header.frame_id = "camera_left_color_optical_frame";
	frameset_msg->header.stamp    = ros_timestamp;
	frameset_msg->depth_image     = depth_msg;
	frameset_msg->color_image     = color_msg;

	double message_duration = (hires_clock::now() - messages_start).count() / 1e6;

	time_point publish_start = hires_clock::now();

	// Publish frameset
	m_frameset_publisher->publish(std::move(frameset_msg));

	// Publish depth frame
	m_depth_publisher.publish(depth_msg, m_depth_camerainfo);

	double publish_duration = (hires_clock::now() - publish_start).count() / 1e6;

	if (m_debug)
	{
		double callback_duration = (hires_clock::now() - callback_start).count() / 1e6;
		double timer_duration    = (hires_clock::now() - m_timer).count() / 1e6;
		m_timer                  = hires_clock::now();
		m_fps_avg                = (m_fps_avg + (1000 / timer_duration)) / 2;
		std::cout << "callback: " << callback_duration << " ms" << std::endl;
		std::cout << "frames:   " << getframes_duration << " ms" << std::endl;
		std::cout << "message:  " << message_duration << " ms" << std::endl;
		std::cout << "publish:  " << publish_duration << " ms" << std::endl;
		std::cout << "duration: " << timer_duration << " ms" << std::endl;
		std::cout << "fps:      " << 1000 / timer_duration << std::endl;
		std::cout << "avg fps:  " << m_fps_avg << std::endl;
		std::cout << std::endl;
	}

	if (m_enable_profiling)
	{
		double callback_duration = (hires_clock::now() - callback_start).count() / 1e6;
		double timer_duration    = (hires_clock::now() - m_timer).count() / 1e6;
		m_timer                  = hires_clock::now();
		double latency_out       = (this->now() - ros_timestamp).to_chrono<std::chrono::nanoseconds>().count() / 1e6;
		std::vector<double> vec;
		vec.push_back(timer_duration);
		vec.push_back(callback_duration);
		vec.push_back(getframes_duration);
		vec.push_back(message_duration);
		vec.push_back(publish_duration);
		vec.push_back(latency_in);
		vec.push_back(latency_out);
		m_profiling_vec.push_back(vec);

		if (m_profiling_vec.size() == m_profiling_size)
		{
			std::cout << "Writing profiline file " << m_profiling_filename << std::endl;
			std::ofstream profiling_file(m_profiling_filename);
			for (int i = 0; i < m_profiling_fields.size(); i++)
			{
				profiling_file << m_profiling_fields[i];
				if (i != m_profiling_fields.size() - 1) profiling_file << ",";
			}
			profiling_file << "\n";
			for (const auto &l : m_profiling_vec)
			{
				for (int i = 0; i < l.size(); i++)
				{
					profiling_file << l[i];
					if (i != l.size() - 1) profiling_file << ",";
				}
				profiling_file << "\n";
			}
			m_profiling_vec.clear();
		}
	}
}

/**
 * @brief ROS service for camera parameter retrieval.
 * @param request Service request
 * @param response Service response
 */
void CameraNode::getCameraParameters(const std::shared_ptr<camera_interfaces::srv::GetCameraParameters::Request> request, std::shared_ptr<camera_interfaces::srv::GetCameraParameters::Response> response) const
{
	response->depth_intrinsics = m_depth_camerainfo;
	response->color_intrinsics = m_color_camerainfo;
	for (unsigned long i = 0; i < 3; i++)
		response->extrinsic_translation[i] = m_extrinsics.translation[i];

	for (unsigned long i = 0; i < 9; i++)
		response->extrinsic_rotation[i] = m_extrinsics.rotation[i];
}
