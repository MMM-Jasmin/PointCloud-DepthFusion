#include "image_node.hpp"

/**
 * @brief Contructor.
 */
ImageNode::ImageNode() :
	Node("image_node", rclcpp::NodeOptions().use_intra_process_comms(false))
{
	
}

/**
 * @brief Initialize image node.
 */
void ImageNode::init()
{
	//m_depth_subscription = this->create_subscription<sensor_msgs::msg::Image>(m_topic_depth, m_qos_profile, std::bind(&ImageNode::depthCallback, this, std::placeholders::_1));
	m_frameset_subscription = this->create_subscription<camera_interfaces::msg::DepthFrameset>(m_topic_frameset, m_qos_profile, std::bind(&ImageNode::framesetCallback, this, std::placeholders::_1));
	cv::namedWindow(m_window_name_frameset, cv::WINDOW_AUTOSIZE);


	//m_image_small_subscription = this->create_subscription<sensor_msgs::msg::Image>( m_topic_image_small, m_qos_profile, std::bind(&ImageNode::imageSmallCallback, this, std::placeholders::_1));
	//cv::namedWindow(m_window_name_image_small, cv::WINDOW_AUTOSIZE);
}

/**
 * @brief Callback function for reveived image message.
 * @param img_msg Received image message
 */
void ImageNode::depthCallback(sensor_msgs::msg::Image::SharedPtr img_msg)
{
	cv::Size image_size(static_cast<int>(img_msg->width), static_cast<int>(img_msg->height));
	cv::Mat color_image(image_size, CV_16UC1, (void *)img_msg->data.data(), cv::Mat::AUTO_STEP);
	m_loop_duration = (hires_clock::now() - m_callback_time).count() / 1e6;
	cv::setWindowTitle(m_window_name_depth, std::to_string(m_loop_duration));
	//cv::setWindowTitle(m_window_name, std::to_string(0.0));
	cv::cvtColor(color_image, color_image, cv::COLOR_RGB2BGR);
	imshow(m_window_name_depth, color_image);

	if (!(cv::waitKey(1) < 0 && cv::getWindowProperty(m_window_name_depth, cv::WND_PROP_AUTOSIZE) >= 0))
		rclcpp::shutdown();

	m_callback_time = hires_clock::now();
}

void ImageNode::imageSmallCallback(sensor_msgs::msg::Image::SharedPtr img_msg)
{
	cv::Size image_size(static_cast<int>(img_msg->width), static_cast<int>(img_msg->height));
	cv::Mat color_image(image_size, CV_8UC3, (void *)img_msg->data.data(), cv::Mat::AUTO_STEP);
	m_loop_duration_image_small = (hires_clock::now() - m_callback_time).count() / 1e6;
	cv::setWindowTitle(m_window_name_image_small, std::to_string(m_loop_duration_image_small));
	//cv::setWindowTitle(m_window_name, std::to_string(0.0));
	cv::cvtColor(color_image, color_image, cv::COLOR_RGB2BGR);
	imshow(m_window_name_image_small, color_image);

	if (!(cv::waitKey(1) < 0 && cv::getWindowProperty(m_window_name_image_small, cv::WND_PROP_AUTOSIZE) >= 0))
		rclcpp::shutdown();

	m_callback_time = hires_clock::now();
}


/**
 * @brief Callback function for reveived image message.
 * @param img_msg Received image message
 */
void ImageNode::framesetCallback(camera_interfaces::msg::DepthFrameset::UniquePtr fset_msg)
{
	m_callback_time = hires_clock::now();
	cv::Size image_size(static_cast<int>(fset_msg.get()->color_image.width), static_cast<int>(fset_msg.get()->color_image.height));
	cv::Mat color_image(image_size, CV_8UC3, (void *)fset_msg.get()->color_image.data.data(), cv::Mat::AUTO_STEP);
	//cv::Mat color_image = cv::imdecode(cv::Mat(fset_msg->color_image.data), cv::IMREAD_UNCHANGED);

	
	cv::setWindowTitle(m_window_name_frameset, std::to_string(m_loop_duration));
	//cv::setWindowTitle(m_window_name, std::to_string(0.0));
	cv::cvtColor(color_image, color_image, cv::COLOR_RGB2BGR);
	imshow(m_window_name_frameset, color_image);

	if (!(cv::waitKey(1) < 0 && cv::getWindowProperty(m_window_name_frameset, cv::WND_PROP_AUTOSIZE) >= 0))
		rclcpp::shutdown();

	m_loop_duration = (hires_clock::now() - m_callback_time).count() / 1e6;
	

}