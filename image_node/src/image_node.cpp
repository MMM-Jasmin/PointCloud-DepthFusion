#include "image_node.hpp"

/**
 * @brief Contructor.
 */
ImageNode::ImageNode() : Node("image_node", rclcpp::NodeOptions().use_intra_process_comms(false)) {
	topic = "/fused_image";
	// topic = "/color/image_raw";
}

/**
 * @brief Initialize image node.
 */
void ImageNode::init() {
	image_subscription = this->create_subscription<sensor_msgs::msg::Image>(
		topic, qos_profile, std::bind(&ImageNode::imageCallback, this, std::placeholders::_1));
		
	cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
}

/**
 * @brief Callback function for reveived image message.
 * @param img_msg Received image message
 */
void ImageNode::imageCallback(sensor_msgs::msg::Image::SharedPtr img_msg) {
	cv::Size image_size(static_cast<int>(img_msg->width), static_cast<int>(img_msg->height));
	cv::Mat color_image(image_size, CV_8UC3, (void*)img_msg->data.data(), cv::Mat::AUTO_STEP);
	double loop_duration = (hires_clock::now() - callback_time).count() / 1e6;
	cv::setWindowTitle(window_name, std::to_string(loop_duration));
	cv::cvtColor(color_image, color_image, cv::COLOR_RGB2BGR);
	imshow(window_name, color_image);
	if (!(cv::waitKey(1) < 0 && cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)) {
		rclcpp::shutdown();
	}
	callback_time = hires_clock::now();
}
