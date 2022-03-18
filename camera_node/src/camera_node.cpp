// SYSTEM
#include <atomic>
#include <fstream>
#include <iostream>
#include <thread>
// ROS2
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/image.hpp>
//#include <sensor_msgs/msg/CompressedImage.hpp>
// PROJECT
#include "camera_node.hpp"

using namespace std::chrono_literals;

/**
 * @brief Constructor.
 */
CameraNode::CameraNode(const std::string &name) :
	Node(name, rclcpp::NodeOptions().use_intra_process_comms(true)),
	m_package_share_directory(),
	m_last_frame_timestamp(),
	m_color_intrinsics(),
	m_depth_intrinsics(),
	m_extrinsics(),
	m_color_camerainfo(),
	m_depth_camerainfo(),
	m_frameset(),
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

/**f
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
	m_pRealsense->setAlign(m_use_rs_align);
	m_pRealsense->setDepthScale(m_pConfig->depth_scale());
	m_pRealsense->setDepthMax(m_pConfig->max_depth());
	m_pRealsense->setUseQueue(m_use_rs_queue);
	// Initialize Realsense camera
	m_pRealsense->init(m_pConfig->camera_serial_no());
	if (m_pExit_request->load()) return;
	// Declare Realsense parameters
	m_pRealsense->declareRosParameters(this);
	
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

	std::string topic_color_small    = std::string(this->get_name()) + "/" + m_pConfig->topic_color_small();
	std::string topic_depth    = std::string(this->get_name()) + "/" + m_pConfig->topic_depth();
	std::string topic_frameset = std::string(this->get_name()) + "/" + m_pConfig->topic_frameset();

	m_frameset_publisher    = this->create_publisher<camera_interfaces::msg::DepthFrameset>(topic_frameset, m_qos_profile);
	m_image_small_publisher = this->create_publisher<sensor_msgs::msg::Image>(topic_color_small, m_qos_profile);
	m_publish_timer         = this->create_wall_timer(std::chrono::nanoseconds(static_cast<int>(1e9 / 30)), std::bind(&CameraNode::publishEverything, this));

	// Create camera parameter service
	m_service = this->create_service<camera_interfaces::srv::GetCameraParameters>(m_node_name + "/get_camera_parameters",std::bind(&CameraNode::getCameraParameters, this, std::placeholders::_1, std::placeholders::_2));

	// Depth image publisher
	//m_depth_publisher = image_transport::create_camera_publisher(this, topic_depth, m_qos_profile.get_rmw_qos_profile());

	// Allocate frames
	m_pColor_frame_0 = reinterpret_cast<uint8_t *>(malloc(static_cast<unsigned>(m_color_intrinsics.width * m_color_intrinsics.height) * 3 * sizeof(uint8_t)));
	m_pColor_frame_1 = reinterpret_cast<uint8_t *>(malloc(static_cast<unsigned>(m_color_intrinsics.width * m_color_intrinsics.height) * 3 * sizeof(uint8_t)));
	m_pDepth_frame_0 = reinterpret_cast<uint16_t *>(malloc(static_cast<unsigned>(m_depth_intrinsics.width * m_depth_intrinsics.height) * sizeof(uint16_t)));
	m_pDepth_frame_1 = reinterpret_cast<uint16_t *>(malloc(static_cast<unsigned>(m_depth_intrinsics.width * m_depth_intrinsics.height) * sizeof(uint16_t)));

	m_pRealsense->start();

	m_pRealsense->getFrames(m_pColor_frame_0, m_pDepth_frame_0, m_timestamp, 200);
	m_pRealsense->getFrames(m_pColor_frame_1, m_pDepth_frame_1, m_timestamp, 200);
	m_pRealsense->getFrames(m_pColor_frame_0, m_pDepth_frame_0, m_timestamp, 200);
	m_pRealsense->getFrames(m_pColor_frame_1, m_pDepth_frame_1, m_timestamp, 200);
	m_pRealsense->getFrames(m_pColor_frame_0, m_pDepth_frame_0, m_timestamp, 200);
	m_pRealsense->getFrames(m_pColor_frame_1, m_pDepth_frame_1, m_timestamp, 200);

	m_buffer = true;

}

/**
 * @brief Stop RealSense camera.
 */
void CameraNode::stop()
{

	m_pRealsense->stop();
}


void CameraNode::publishFrameset(uint8_t * color_image, int color_width, int color_height, uint8_t * depth_image, int depth_width, int depth_height, rclcpp::Time ros_timestamp, rclcpp::Publisher<camera_interfaces::msg::DepthFrameset>::SharedPtr message_publisher)
{
	sensor_msgs::msg::Image depth_msg;
	depth_msg.header.frame_id = "camera_left_color_optical_frame";
	depth_msg.header.stamp    = ros_timestamp;
	depth_msg.width           = static_cast<uint>(depth_width);
	depth_msg.height          = static_cast<uint>(depth_height);
	depth_msg.is_bigendian    = false;
	depth_msg.step            = depth_msg.width * sizeof(uint16_t);
	depth_msg.encoding        = "mono16";
	depth_msg.data.assign(depth_image, depth_image + (depth_msg.step * depth_msg.height));

	sensor_msgs::msg::Image color_msg;
	color_msg.header.frame_id = "camera_left_color_optical_frame";
	color_msg.header.stamp    = ros_timestamp;
	color_msg.width           = static_cast<uint>(color_width);
	color_msg.height          = static_cast<uint>(color_height);
	color_msg.is_bigendian    = false;
	color_msg.step            = color_msg.width * 3 * sizeof(uint8_t);
	color_msg.encoding        = "rgb8";
	color_msg.data.assign(color_image, color_image + (color_msg.step * color_msg.height));

	camera_interfaces::msg::DepthFrameset::UniquePtr frameset_msg(new camera_interfaces::msg::DepthFrameset());
	frameset_msg->header.frame_id = "camera_left_color_optical_frame";
	frameset_msg->header.stamp    = ros_timestamp;
	frameset_msg->depth_image     = depth_msg;
	frameset_msg->color_image     = color_msg;

	
	// Publish frameset
	message_publisher->publish(std::move(frameset_msg));

}


void CameraNode::publishImageSmall(uint8_t * color_image, int color_width, int color_height, rclcpp::Time ros_timestamp, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr message_publisher)
{

	uint8_t *color_frame_bytes = reinterpret_cast<uint8_t *>(color_image);

	cv::Size full_image_size(static_cast<uint>(color_width),static_cast<uint>(color_height));
	cv::Size small_image_size(static_cast<uint>(m_pConfig->smallImage_width()),static_cast<uint>(m_pConfig->smallImage_height()));
	cv::Mat small_color_image(full_image_size, CV_8UC3, (void *)color_frame_bytes, cv::Mat::AUTO_STEP);
	cv::resize(small_color_image, small_color_image, small_image_size, 0, 0, cv::INTER_AREA);

	sensor_msgs::msg::Image::SharedPtr color_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", small_color_image).toImageMsg();
	color_msg->header.frame_id = "camera_left_color_optical_frame";
	color_msg->header.stamp    = ros_timestamp;


	message_publisher->publish(std::move(*color_msg.get()));

}

/**
 * @brief Publish Everything. Each publisher in it´s own thread and the main thread gets the new images.
 */
void CameraNode::publishEverything()
{
	if (m_pExit_request->load() || !rclcpp::ok())
	{
		m_publish_timer.get()->cancel();
		m_pRealsense->stop();
		return;
	}
	// start the time
	time_point callback_start = hires_clock::now();

	// Pointer and variables
	rclcpp::Time ros_timestamp;
	uint8_t *current_color_frame_bytes;
	uint8_t *current_depth_frame_bytes;
	uint8_t *next_color_frame_bytes;
	uint16_t *next_depth_frame_bytes;

	// ----------------------------------------
	// Set timestamp for the last frame to be published now
	if (m_use_rs_timestamp)
		ros_timestamp = rclcpp::Time(int64_t(m_timestamp * 1e6), RCL_ROS_TIME); // Camera timestamp
	else
		ros_timestamp = this->now(); // Node timestamp

	// ----------------------------------------
	// Set all pointer to the correct buffer
	if(m_buffer){
		// last frames to be published now
		current_depth_frame_bytes = reinterpret_cast<uint8_t *>(m_pDepth_frame_0);
		current_color_frame_bytes = reinterpret_cast<uint8_t *>(m_pColor_frame_0);
		// images polled in this iteration
		next_color_frame_bytes = m_pColor_frame_1;
		next_depth_frame_bytes = m_pDepth_frame_1;
	} else {
		// last frames to be published now
		current_depth_frame_bytes = reinterpret_cast<uint8_t *>(m_pDepth_frame_1);
		current_color_frame_bytes = reinterpret_cast<uint8_t *>(m_pColor_frame_1);
		// images polled in this iteration
		next_color_frame_bytes = m_pColor_frame_0;
		next_depth_frame_bytes = m_pDepth_frame_0;
	}

	// Start the time
	time_point publish_start = hires_clock::now();
	
	// ----------------------------------------
	// Start all publisher threads
	//publishFrameset(color_frame_bytes, color_width, color_height, depth_frame_bytes, depth_width, depth_height, ros_timestamp, m_frameset_publisher);
	auto future_publishFrameset = std::async(&CameraNode::publishFrameset, this, current_color_frame_bytes, m_color_intrinsics.width, m_color_intrinsics.height, current_depth_frame_bytes, m_depth_intrinsics.width, m_depth_intrinsics.height, ros_timestamp, m_frameset_publisher);

	//publishImageSmall(color_frame_bytes, color_width, color_height, ros_timestamp, m_image_small_publisher);
	auto future_publishImageSmall = std::async(&CameraNode::publishImageSmall, this, current_color_frame_bytes, m_color_intrinsics.width, m_color_intrinsics.height, ros_timestamp, m_image_small_publisher);

	// ----------------------------------------
	// Get new images while waiting for the threads
	time_point getframes_start = hires_clock::now();
	m_pRealsense->getFrames(next_color_frame_bytes, next_depth_frame_bytes, m_timestamp, 60);
	double getframes_duration = (hires_clock::now() - getframes_start).count() / 1e6;

	// ----------------------------------------
	// clean up
	future_publishImageSmall.wait();
	future_publishFrameset.wait();
	m_buffer = !m_buffer;

	double publish_duration = (hires_clock::now() - publish_start).count() / 1e6;

	if (m_debug)
	{
		double callback_duration = (hires_clock::now() - callback_start).count() / 1e6;
		double timer_duration    = (hires_clock::now() - m_timer).count() / 1e6;
		m_timer                  = hires_clock::now();
		m_fps_avg                = (m_fps_avg + (1000 / timer_duration)) / 2;
		std::cout << "callback: " << callback_duration << " ms" << std::endl;
		std::cout << "frames:   " << getframes_duration << " ms" << std::endl;
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
		vec.push_back(publish_duration);
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
