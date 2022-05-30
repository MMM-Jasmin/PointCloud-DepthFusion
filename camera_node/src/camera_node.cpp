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

#include "Utils.h"

using namespace std::chrono_literals;

const double ONE_SECOND            = 1000.0; // One second in milliseconds

/**
 * @brief Constructor.
 */
CameraNode::CameraNode(const std::string &name) : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true)),
	m_package_share_directory(),
	m_last_frame_timestamp(),
	m_color_intrinsics(),
	m_depth_intrinsics(),
	m_extrinsics(),
	m_color_camerainfo(),
	m_depth_camerainfo(),
	m_frameset()
{
	m_node_name               = name;
	m_package_share_directory = ament_index_cpp::get_package_share_directory("camera_node");
	m_pConfig                 = new Config(this);
	m_pRealsense              = new Realsense();

	this->declare_parameter<bool>("use_rs_align", true);
	this->declare_parameter("print_fps", true);
	this->declare_parameter("FPS_STR", "FPS" );

	
}

/**f
 * @brief Destructor.
 */
CameraNode::~CameraNode()
{
	m_pRealsense->stop();

	if (m_pRealsense != nullptr) delete m_pRealsense;
	if (m_pConfig != nullptr) delete m_pConfig;
	
	if (m_pColor_frame_0 != nullptr) free (m_pColor_frame_0);
	if (m_pColor_frame_1 != nullptr) free (m_pColor_frame_1);
	if (m_pDepth_frame_0 != nullptr) free (m_pDepth_frame_0);
	if (m_pDepth_frame_1 != nullptr) free (m_pDepth_frame_1);
}

/**
 * @brief Initialize camera node.
 */
void CameraNode::init()
{

	this->get_parameter("use_rs_align", m_use_rs_align);
	this->get_parameter("FPS_STR", m_FPS_STR );
	this->get_parameter("print_fps", m_print_fps);

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
	m_qos_profile = m_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
	m_qos_profile = m_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	m_qos_profile = m_qos_profile.deadline(std::chrono::nanoseconds(static_cast<int>(1e9 / 30)));



	std::string topic_color    		= std::string(this->get_name()) + "/" + m_pConfig->topic_color();
	std::string topic_color_small   = std::string(this->get_name()) + "/" + m_pConfig->topic_color_small();
	std::string topic_depth    		= std::string(this->get_name()) + "/" + m_pConfig->topic_depth();
	std::string topic_depth_thresh  = std::string(this->get_name()) + "/" + m_pConfig->topic_depth() + "_threshold";
	std::string topic_frameset 		= std::string(this->get_name()) + "/" + m_pConfig->topic_frameset();

	m_frameset_publisher    			= this->create_publisher<camera_interfaces::msg::DepthFrameset>(topic_frameset, m_qos_profile);
	//m_image_publisher 					= this->create_publisher<sensor_msgs::msg::Image>(topic_color, m_qos_profile);
	m_image_small_publisher 			= this->create_publisher<sensor_msgs::msg::Image>(topic_color_small, m_qos_profile);
	//m_depth_image_publisher 			= this->create_publisher<sensor_msgs::msg::Image>(topic_depth, m_qos_profile);


	m_publish_timer        				= this->create_wall_timer(std::chrono::nanoseconds(static_cast<int>(1e9 / 30)), std::bind(&CameraNode::publishEverything, this));

	// Create camera parameter service
	m_service = this->create_service<camera_interfaces::srv::GetCameraParameters>(m_node_name + "/get_camera_parameters",std::bind(&CameraNode::getCameraParameters, this, std::placeholders::_1, std::placeholders::_2));

	// Depth image publisher
	//m_depth_publisher = image_transport::create_camera_publisher(this, topic_depth, m_qos_profile.get_rmw_qos_profile());

	// Allocate frames
	m_pColor_frame_0 			= reinterpret_cast<uint8_t 	*>(malloc(static_cast<unsigned>(m_color_intrinsics.width * m_color_intrinsics.height) * 3 * sizeof(uint8_t)));
	m_pColor_frame_1 			= reinterpret_cast<uint8_t 	*>(malloc(static_cast<unsigned>(m_color_intrinsics.width * m_color_intrinsics.height) * 3 * sizeof(uint8_t)));
	m_pColor_frame_small_0  	= reinterpret_cast<uint8_t 	*>(malloc(static_cast<unsigned>(m_pConfig->smallImage_width() * m_pConfig->smallImage_height()) * 3 * sizeof(uint8_t)));
	m_pColor_frame_small_1  	= reinterpret_cast<uint8_t 	*>(malloc(static_cast<unsigned>(m_pConfig->smallImage_width() * m_pConfig->smallImage_height()) * 3 * sizeof(uint8_t)));
	m_pDepth_frame_0 			= reinterpret_cast<uint16_t *>(malloc(static_cast<unsigned>(m_depth_intrinsics.width * m_depth_intrinsics.height) * sizeof(uint16_t)));
	m_pDepth_frame_1 			= reinterpret_cast<uint16_t *>(malloc(static_cast<unsigned>(m_depth_intrinsics.width * m_depth_intrinsics.height) * sizeof(uint16_t)));

	m_pRealsense->start();

	getNextImages(m_pColor_frame_0, m_pDepth_frame_0, std::ref(m_timestamp), 60);
	getNextImages(m_pColor_frame_1, m_pDepth_frame_1, std::ref(m_timestamp), 60);
	getNextImages(m_pColor_frame_0, m_pDepth_frame_0, std::ref(m_timestamp), 60);
	getNextImages(m_pColor_frame_1, m_pDepth_frame_1, std::ref(m_timestamp), 60);
	getNextImages(m_pColor_frame_0, m_pDepth_frame_0, std::ref(m_timestamp), 60);
	getNextImages(m_pColor_frame_1, m_pDepth_frame_1, std::ref(m_timestamp), 60);

	m_buffer = true;

	m_elapsedTime = 0;
	m_timer.Start();
}

/**
 * @brief Stop RealSense camera.
 */
void CameraNode::stop()
{
	m_pRealsense->stop();
}


void CameraNode::publishFrameset(uint8_t * color_image, int color_width, int color_height, uint16_t * depth_image, int depth_width, int depth_height, rclcpp::Time ros_timestamp)
{
	uint8_t * ui8_depth_image = reinterpret_cast<uint8_t *>(depth_image);

	sensor_msgs::msg::Image depth_msg;
	depth_msg.header.frame_id = "camera_depth_optical_frame";
	depth_msg.header.stamp    = ros_timestamp;
	depth_msg.width           = static_cast<uint>(depth_width);
	depth_msg.height          = static_cast<uint>(depth_height);
	depth_msg.is_bigendian    = false;
	depth_msg.step            = depth_msg.width * sizeof(uint16_t);
	depth_msg.encoding        = "mono16";
	depth_msg.data.assign(ui8_depth_image, ui8_depth_image + (depth_msg.step * depth_msg.height));

	sensor_msgs::msg::Image color_msg;
	color_msg.header.frame_id = "camera_color_optical_frame";
	color_msg.header.stamp    = ros_timestamp;
	color_msg.width           = static_cast<uint>(color_width);
	color_msg.height          = static_cast<uint>(color_height);
	color_msg.is_bigendian    = false;
	color_msg.step            = color_msg.width * 3 * sizeof(uint8_t);
	color_msg.encoding        = "rgb8";
	color_msg.data.assign(color_image, color_image + (color_msg.step * color_msg.height));

	camera_interfaces::msg::DepthFrameset::UniquePtr frameset_msg(new camera_interfaces::msg::DepthFrameset());
	frameset_msg->header.frame_id = "camera_optical_frame";
	frameset_msg->header.stamp    = ros_timestamp;
	frameset_msg->depth_image     = depth_msg;
	frameset_msg->color_image     = color_msg;
	
	// Publish frameset
	m_frameset_publisher->publish(std::move(frameset_msg));
}


void CameraNode::publishImage(uint8_t * color_image, int width, int height, std::string frame_id, rclcpp::Time ros_timestamp, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr message_publisher)
{
	sensor_msgs::msg::Image color_msg;
	color_msg.header.frame_id = frame_id;
	color_msg.header.stamp    = ros_timestamp;
	color_msg.width           = width;
	color_msg.height          = height;
	color_msg.is_bigendian    = false;
	color_msg.step            = color_msg.width * 3 * sizeof(uint8_t);
	color_msg.encoding        = "rgb8";
	color_msg.data.assign(color_image, color_image + (color_msg.step * color_msg.height));

	sensor_msgs::msg::Image::UniquePtr color_msg_ptr = std::make_unique<sensor_msgs::msg::Image>(color_msg);

	message_publisher->publish(std::move(color_msg));
}

void CameraNode::publishDepthImage(uint16_t * depth_image, int width, int height, std::string frame_id, rclcpp::Time ros_timestamp, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr message_publisher)
{

	uint8_t * ui8_depth_image = reinterpret_cast<uint8_t *>(depth_image);

	sensor_msgs::msg::Image depth_msg;
	depth_msg.header.frame_id = frame_id;
	depth_msg.header.stamp    = ros_timestamp;
	depth_msg.width           = width;
	depth_msg.height          = height;
	depth_msg.is_bigendian    = false;
	depth_msg.step            = depth_msg.width * sizeof(uint16_t);
	depth_msg.encoding        = "mono16";
	depth_msg.data.assign(ui8_depth_image, ui8_depth_image + (depth_msg.step * depth_msg.height));

	sensor_msgs::msg::Image::UniquePtr depth_msg_ptr = std::make_unique<sensor_msgs::msg::Image>(depth_msg);

	message_publisher->publish(std::move(depth_msg_ptr));

}

void CameraNode::getNextImages (uint8_t *next_color_frame_bytes, uint16_t *next_depth_frame_bytes, double &m_timestamp, int timeout)
{
	m_pRealsense->getFrames(next_color_frame_bytes, next_depth_frame_bytes, m_timestamp, timeout);
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
	uint8_t *current_color_frame_bytes;
	uint8_t *current_color_frame_small_bytes;
	uint16_t *current_depth_frame_bytes;

	uint8_t *next_color_frame_bytes;
	uint8_t *next_color_frame_small_bytes;
	uint16_t *next_depth_frame_bytes;

	// ----------------------------------------
	// Set timestamp for the last frame to be published now
	rclcpp::Time ros_timestamp = rclcpp::Time(int64_t(m_timestamp * 1e6), RCL_ROS_TIME); // Camera timestamp

	// ----------------------------------------
	// Set all pointer to the correct buffer
	if(m_buffer){
		// last frames to be published now
		current_depth_frame_bytes			= m_pDepth_frame_0;
		current_color_frame_bytes 			= m_pColor_frame_0;
		current_color_frame_small_bytes 	= m_pColor_frame_small_0;
		// images polled in this iteration
		next_color_frame_bytes 				= m_pColor_frame_1;
		next_depth_frame_bytes 				= m_pDepth_frame_1;
		next_color_frame_small_bytes 		= m_pColor_frame_small_1;
	} else {
		// last frames to be published now
		current_depth_frame_bytes 			= m_pDepth_frame_1;
		current_color_frame_bytes 			= m_pColor_frame_1;
		current_color_frame_small_bytes 	= m_pColor_frame_small_1;
		// images polled in this iteration
		next_color_frame_bytes 				= m_pColor_frame_0;
		next_depth_frame_bytes 				= m_pDepth_frame_0;
		next_color_frame_small_bytes 		= m_pColor_frame_small_0;
	}

	// ----------------------------------------
	// Get new images
	//getNextImages(next_color_frame_bytes, next_depth_frame_bytes, next_depth_threshold_frame_bytes, std::ref(m_timestamp), 60);
	auto future_getNextImages = std::async(&CameraNode::getNextImages, this, next_color_frame_bytes, next_depth_frame_bytes, std::ref(m_timestamp), 60);

	// ----------------------------------------
	// Start all publisher threads
	auto future_publishFrameset 	= std::async(&CameraNode::publishFrameset, this, current_color_frame_bytes, m_color_intrinsics.width, m_color_intrinsics.height, current_depth_frame_bytes, m_depth_intrinsics.width, m_depth_intrinsics.height, ros_timestamp);
	//auto future_publishdepth 		= std::async(&CameraNode::publishDepthImage, this, current_depth_frame_bytes, m_depth_intrinsics.width, m_depth_intrinsics.height , "camera_depth_optical_frame", ros_timestamp, m_depth_image_publisher);
	//auto future_publishImage 		= std::async(&CameraNode::publishImage, this, current_color_frame_bytes, m_color_intrinsics.width, m_color_intrinsics.height , "camera_color_optical_frame", ros_timestamp, m_image_publisher);

	// ----------------------------------------
	// resize in thread has lead to a memory leak... so do it in the main thread and start thread with that.
	cv::Mat fullimage(cv::Size(m_color_intrinsics.width, m_color_intrinsics.height), CV_8UC3 ,current_color_frame_bytes);
	cv::Mat smallimage;
	cv::resize(fullimage, smallimage, cv::Size(m_pConfig->smallImage_width(), m_pConfig->smallImage_height()), cv::INTER_LINEAR);
	std::memcpy(reinterpret_cast<void*>(current_color_frame_small_bytes), smallimage.data, smallimage.size().width * smallimage.size().height * smallimage.channels() * sizeof(uint8_t) );
	auto future_publishImageSmall = std::async(&CameraNode::publishImage, this, current_color_frame_small_bytes, m_pConfig->smallImage_width(), m_pConfig->smallImage_height() , "camera_smaller_color_optical_frame", ros_timestamp, m_image_small_publisher);

	// ----------------------------------------
	// clean up
	
	// No need to wait for the treads in general.. 
	//future_getNextImages.wait();
	//future_publishImageSmall.wait();
	//future_publishFrameset.wait();

	m_buffer = !m_buffer;


	m_frameCnt++;
	CheckFPS(&m_frameCnt);
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

void CameraNode::CheckFPS(uint64_t* pFrameCnt)
	{
		m_timer.Stop();

		double itrTime      = m_timer.GetElapsedTimeInMilliSec();
		double fps;

		m_elapsedTime += itrTime;

		fps = 1000 / (m_elapsedTime / (*pFrameCnt));

		if (m_elapsedTime >= ONE_SECOND)
		{
			PrintFPS(fps, itrTime);

			*pFrameCnt    = 0;
			m_elapsedTime = 0;
		}

		m_timer.Start();
	}

void CameraNode::PrintFPS(const float fps, const float itrTime)
{
	std::stringstream str("");

	if (fps == 0.0f)
			str << string_format("{\"%s\": 0.0}", m_FPS_STR.c_str());
	else
		str << string_format("{\"%s\": %.2f, \"lastCurrMSec\": %.2f }", m_FPS_STR.c_str(), fps, itrTime);

	auto message = std_msgs::msg::String();
	message.data = str.str();
	//m_fps_publisher->publish(message);

		
	if (m_print_fps)
		RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
}