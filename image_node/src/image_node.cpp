#include "image_node.hpp"

/**
 * @brief Contructor.
 */
ImageNode::ImageNode() :
	Node("image_node", rclcpp::NodeOptions().use_intra_process_comms(false))
{
	// topic = "/fused_image";
	// topic = "/color/image_raw";
}

/**
 * @brief Initialize image node.
 */
void ImageNode::init()
{
	m_depth_subscription = this->create_subscription<sensor_msgs::msg::Image>(m_topic_depth, m_qos_profile, std::bind(&ImageNode::depthCallback, this, std::placeholders::_1));

	cv::namedWindow(m_window_name, cv::WINDOW_AUTOSIZE);
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
	cv::setWindowTitle(m_window_name, std::to_string(m_loop_duration));
	//cv::setWindowTitle(m_window_name, std::to_string(0.0));
	cv::cvtColor(color_image, color_image, cv::COLOR_RGB2BGR);
	imshow(m_window_name, color_image);

	if (!(cv::waitKey(1) < 0 && cv::getWindowProperty(m_window_name, cv::WND_PROP_AUTOSIZE) >= 0))
		rclcpp::shutdown();

	m_callback_time = hires_clock::now();
}
