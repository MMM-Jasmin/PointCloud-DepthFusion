#include "image_node.hpp"

/**
 * @brief Contructor.
 */
ImageNode::ImageNode() : Node("image_node", rclcpp::NodeOptions().use_intra_process_comms(false)) {}

/**
 * @brief Destructor.
 *
 */
ImageNode::~ImageNode() {}

/**
 * @brief Initialize image node.
 */
void ImageNode::init() {
	if (m_verbose) std::cout << "OpenCV build information:" << std::endl << cv::getBuildInformation() << std::endl;

	m_qos_profile = m_qos_profile.keep_last(2);
	m_qos_profile = m_qos_profile.lifespan(std::chrono::milliseconds(500));
	m_qos_profile = m_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	m_qos_profile = m_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
	// m_qos_profile = m_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

	m_fused_subscription = this->create_subscription<sensor_msgs::msg::Image>(
	    m_topic_fused, m_qos_profile, std::bind(&ImageNode::fusedCallback, this, std::placeholders::_1));
	cv::namedWindow(m_window_name_fused, cv::WINDOW_NORMAL | cv::WINDOW_GUI_EXPANDED);
	// cv::setWindowProperty(m_window_name_fused, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

	// Create buttons (press CTRL+P to show OpenCV button toolbar)
	std::string button_fused_name = "Show Fused Image";
	std::string button_depth_left_name = "Show Left Depth Image";
	std::string button_depth_right_name = "Show Right Depth Image";
	std::string button_frameset_left_name = "Show Left Frameset";
	std::string button_frameset_right_name = "Show Right Frameset";
	cv::createButton(button_fused_name, ImageNode::buttonFusedCallback, this, cv::QT_PUSH_BUTTON, 1);
	cv::createButton(button_depth_left_name, ImageNode::buttonDepthLeftCallback, this, cv::QT_PUSH_BUTTON, 1);
	cv::createButton(button_depth_right_name, ImageNode::buttonDepthRightCallback, this, cv::QT_PUSH_BUTTON, 1);
	cv::createButton(button_frameset_left_name, ImageNode::buttonFramesetLeftCallback, this, cv::QT_PUSH_BUTTON, 1);
	cv::createButton(button_frameset_right_name, ImageNode::buttonFramesetRightCallback, this, cv::QT_PUSH_BUTTON, 1);
	cv::resizeWindow(m_window_name_fused, 608, 1080);

	// m_depth_subscription = this->create_subscription<sensor_msgs::msg::Image>(
	//     m_topic_depth, m_qos_profile, std::bind(&ImageNode::depthCallback, this, std::placeholders::_1));
	// m_frameset_subscription = this->create_subscription<camera_interfaces::msg::DepthFrameset>(
	//     m_topic_frameset, m_qos_profile, std::bind(&ImageNode::framesetCallback, this, std::placeholders::_1));
	// cv::namedWindow(m_window_name_color, cv::WINDOW_AUTOSIZE);
	// cv::namedWindow(m_window_name_depth, cv::WINDOW_AUTOSIZE);

	// m_image_small_subscription = this->create_subscription<sensor_msgs::msg::Image>(
	//     m_topic_image_small, m_qos_profile, std::bind(&ImageNode::imageSmallCallback, this, std::placeholders::_1));
	// cv::namedWindow(m_window_name_image_small, cv::WINDOW_AUTOSIZE);

	// m_depth_thr_subscription = this->create_subscription<sensor_msgs::msg::Image>(
	//     m_topic_depth, m_qos_profile, std::bind(&ImageNode::depthCallback, this, std::placeholders::_1));
	// cv::namedWindow(m_window_name_depth, cv::WINDOW_AUTOSIZE);
}

/**
 * @brief Wait for pressed key and exit.
 *
 */
void ImageNode::waitForKey() {
	char pressed_key = cv::waitKey(1);
	if (pressed_key == 'q') {
		rclcpp::shutdown();
	}
}

/**
 * @brief Fused image callback function.
 *
 * @param img_msg Fused image message
 */
void ImageNode::fusedCallback(sensor_msgs::msg::Image::UniquePtr img_msg) {
	double callback_duration_ms = (Clock::now() - m_callback_time_fused).count() / 1e6;
	unsigned callback_fps = static_cast<unsigned>(1000 / callback_duration_ms);
	m_callback_time_fused = Clock::now();
	double timestamp_ms = static_cast<rclcpp::Time>(img_msg->header.stamp).nanoseconds() / 1e6;
	unsigned latency_ms = static_cast<unsigned>(
	    (std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count() / 1e6) -
	    timestamp_ms);
	std::string window_title =
	    m_window_name_fused + " [FPS: " + std::to_string(callback_fps) + " LAT: " + std::to_string(latency_ms) + "]";

	cv::Size image_size(static_cast<int>(img_msg->width), static_cast<int>(img_msg->height));
	cv::Mat color_image(image_size, CV_8UC3, (void *)img_msg->data.data(), cv::Mat::AUTO_STEP);
	cv::cvtColor(color_image, color_image, cv::COLOR_RGB2BGR);
	cv::setWindowTitle(m_window_name_fused, window_title);
	imshow(m_window_name_fused, color_image);
	waitForKey();
}

/**
 * @brief Left camera depth image callback function.
 *
 * @param img_msg Left depth image message
 */
void ImageNode::depthLeftCallback(sensor_msgs::msg::Image::SharedPtr img_msg) {
	double callback_duration_ms = (Clock::now() - m_callback_time_depth_left).count() / 1e6;
	unsigned callback_fps = static_cast<unsigned>(1000 / callback_duration_ms);
	m_callback_time_depth_left = Clock::now();
	double timestamp_ms = static_cast<rclcpp::Time>(img_msg->header.stamp).nanoseconds() / 1e6;
	unsigned latency_ms = static_cast<unsigned>(
	    (std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count() / 1e6) -
	    timestamp_ms);
	std::string window_title =
	    m_window_name_depth_left + " [FPS: " + std::to_string(callback_fps) + " LAT: " + std::to_string(latency_ms) + "]";

	cv::Size image_size(static_cast<int>(img_msg->width), static_cast<int>(img_msg->height));
	cv::Mat depth_image(image_size, CV_16UC1, (void *)img_msg->data.data(), cv::Mat::AUTO_STEP);
	cv::convertScaleAbs(depth_image, depth_image, m_depth_image_scale);
	cv::cvtColor(depth_image, depth_image, cv::COLOR_RGB2BGR);
	cv::setWindowTitle(m_window_name_depth_left, window_title);
	imshow(m_window_name_depth_left, depth_image);
	waitForKey();
}

/**
 * @brief Right camera depth image callback function.
 *
 * @param img_msg Right depth image message
 */
void ImageNode::depthRightCallback(sensor_msgs::msg::Image::SharedPtr img_msg) {
	double callback_duration_ms = (Clock::now() - m_callback_time_depth_right).count() / 1e6;
	unsigned callback_fps = static_cast<unsigned>(1000 / callback_duration_ms);
	m_callback_time_depth_right = Clock::now();
	double timestamp_ms = static_cast<rclcpp::Time>(img_msg->header.stamp).nanoseconds() / 1e6;
	unsigned latency_ms = static_cast<unsigned>(
	    (std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count() / 1e6) -
	    timestamp_ms);
	std::string window_title = m_window_name_depth_right + " [FPS: " + std::to_string(callback_fps) +
	                           " LAT: " + std::to_string(latency_ms) + "]";

	cv::Size image_size(static_cast<int>(img_msg->width), static_cast<int>(img_msg->height));
	cv::Mat depth_image(image_size, CV_16UC1, (void *)img_msg->data.data(), cv::Mat::AUTO_STEP);
	cv::convertScaleAbs(depth_image, depth_image, m_depth_image_scale);
	cv::cvtColor(depth_image, depth_image, cv::COLOR_RGB2BGR);
	cv::setWindowTitle(m_window_name_depth_right, window_title);
	imshow(m_window_name_depth_right, depth_image);
	waitForKey();
}

/**
 * @brief Left camera frameset callback function.
 *
 * @param fset_msg Left frameset message
 */
void ImageNode::framesetLeftCallback(camera_interfaces::msg::DepthFrameset::UniquePtr fset_msg) {
	double callback_duration_ms = (Clock::now() - m_callback_time_frameset_left).count() / 1e6;
	unsigned callback_fps = static_cast<unsigned>(1000 / callback_duration_ms);
	m_callback_time_frameset_left = Clock::now();
	double timestamp_ms = static_cast<rclcpp::Time>(fset_msg->header.stamp).nanoseconds() / 1e6;
	unsigned latency_ms = static_cast<unsigned>(
	    (std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count() / 1e6) -
	    timestamp_ms);
	std::string window_title = m_window_name_frameset_left + " [FPS: " + std::to_string(callback_fps) +
	                           " LAT: " + std::to_string(latency_ms) + "]";

	cv::Size color_size(static_cast<int>(fset_msg.get()->color_image.width),
	                    static_cast<int>(fset_msg.get()->color_image.height));
	cv::Size depth_size(static_cast<int>(fset_msg.get()->depth_image.width),
	                    static_cast<int>(fset_msg.get()->depth_image.height));
	cv::Size frameset_size(color_size.width, color_size.height + depth_size.height);
	cv::Mat frameset(frameset_size, CV_8UC3, cv::Scalar::all(0));
	cv::Mat color_image(color_size, CV_8UC3, (void *)fset_msg.get()->color_image.data.data(), cv::Mat::AUTO_STEP);
	cv::Mat depth_image(depth_size, CV_16UC1, (void *)fset_msg.get()->depth_image.data.data(), cv::Mat::AUTO_STEP);
	cv::convertScaleAbs(depth_image, depth_image, m_depth_image_scale);
	cv::cvtColor(color_image, color_image, cv::COLOR_RGB2BGR);
	cv::cvtColor(depth_image, depth_image, cv::COLOR_RGB2BGR);
	cv::Mat color_roi = frameset(cv::Rect(0, 0, color_image.cols, color_image.rows));
	cv::Mat depth_roi = frameset(cv::Rect(0, color_image.rows, depth_image.cols, depth_image.rows));
	color_image.copyTo(color_roi);
	depth_image.copyTo(depth_roi);
	cv::setWindowTitle(m_window_name_frameset_left, window_title);
	imshow(m_window_name_frameset_left, frameset);
	waitForKey();
}

/**
 * @brief Right camera frameset callback function.
 *
 * @param fset_msg Right frameset message
 */
void ImageNode::framesetRightCallback(camera_interfaces::msg::DepthFrameset::UniquePtr fset_msg) {
	double callback_duration_ms = (Clock::now() - m_callback_time_frameset_right).count() / 1e6;
	unsigned callback_fps = static_cast<unsigned>(1000 / callback_duration_ms);
	m_callback_time_frameset_right = Clock::now();
	double timestamp_ms = static_cast<rclcpp::Time>(fset_msg->header.stamp).nanoseconds() / 1e6;
	unsigned latency_ms = static_cast<unsigned>(
	    (std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count() / 1e6) -
	    timestamp_ms);
	std::string window_title = m_window_name_frameset_right + " [FPS: " + std::to_string(callback_fps) +
	                           " LAT: " + std::to_string(latency_ms) + "]";

	cv::Size color_size(static_cast<int>(fset_msg.get()->color_image.width),
	                    static_cast<int>(fset_msg.get()->color_image.height));
	cv::Size depth_size(static_cast<int>(fset_msg.get()->depth_image.width),
	                    static_cast<int>(fset_msg.get()->depth_image.height));
	cv::Size frameset_size(color_size.width, color_size.height + depth_size.height);
	cv::Mat frameset(frameset_size, CV_8UC3, cv::Scalar::all(0));
	cv::Mat color_image(color_size, CV_8UC3, (void *)fset_msg.get()->color_image.data.data(), cv::Mat::AUTO_STEP);
	cv::Mat depth_image(depth_size, CV_16UC1, (void *)fset_msg.get()->depth_image.data.data(), cv::Mat::AUTO_STEP);
	cv::convertScaleAbs(depth_image, depth_image, m_depth_image_scale);
	cv::cvtColor(color_image, color_image, cv::COLOR_RGB2BGR);
	cv::cvtColor(depth_image, depth_image, cv::COLOR_RGB2BGR);
	cv::Mat color_roi = frameset(cv::Rect(0, 0, color_image.cols, color_image.rows));
	cv::Mat depth_roi = frameset(cv::Rect(0, color_image.rows, depth_image.cols, depth_image.rows));
	color_image.copyTo(color_roi);
	depth_image.copyTo(depth_roi);
	cv::setWindowTitle(m_window_name_frameset_right, window_title);
	imshow(m_window_name_frameset_right, frameset);
	waitForKey();
}

/**
 * @brief Fused image button callback function.
 *
 * @param state Button state
 * @param user_data Pointer to node instance
 */
void ImageNode::buttonFusedCallback(int state, void *user_data) {
	ImageNode *p_this = reinterpret_cast<ImageNode *>(user_data);
	p_this->m_show_fused = !p_this->m_show_fused;
	if (p_this->m_fused_subscription == nullptr) {
		if (p_this->m_verbose) std::cout << "subscribe fused" << std::endl;
		p_this->m_fused_subscription = p_this->create_subscription<sensor_msgs::msg::Image>(
		    p_this->m_topic_fused, p_this->m_qos_profile,
		    std::bind(&ImageNode::fusedCallback, p_this, std::placeholders::_1));
	} else {
		if (p_this->m_verbose) std::cout << "unsubscribe fused" << std::endl;
		p_this->m_fused_subscription.reset();
		cv::destroyWindow(p_this->m_window_name_fused);
		if (!(p_this->m_show_fused || p_this->m_show_depth_left || p_this->m_show_depth_right ||
		      p_this->m_show_frameset_left || p_this->m_show_frameset_right)) {
			rclcpp::shutdown();
		}
	}
}

/**
 * @brief Left depth image button callback function.
 *
 * @param state Button state
 * @param user_data Pointer to node instance
 */
void ImageNode::buttonDepthLeftCallback(int state, void *user_data) {
	ImageNode *p_this = reinterpret_cast<ImageNode *>(user_data);
	p_this->m_show_depth_left = !p_this->m_show_depth_left;
	if (p_this->m_depth_left_subscription == nullptr) {
		if (p_this->m_verbose) std::cout << "subscribe depth left" << std::endl;
		p_this->m_depth_left_subscription = p_this->create_subscription<sensor_msgs::msg::Image>(
		    p_this->m_topic_depth_left, p_this->m_qos_profile,
		    std::bind(&ImageNode::depthLeftCallback, p_this, std::placeholders::_1));
		cv::namedWindow(p_this->m_window_name_depth_left, cv::WINDOW_NORMAL);
	} else {
		if (p_this->m_verbose) std::cout << "unsubscribe depth left" << std::endl;
		p_this->m_depth_left_subscription.reset();
		cv::destroyWindow(p_this->m_window_name_depth_left);
		if (!(p_this->m_show_fused || p_this->m_show_depth_left || p_this->m_show_depth_right ||
		      p_this->m_show_frameset_left || p_this->m_show_frameset_right)) {
			rclcpp::shutdown();
		}
	}
}

/**
 * @brief Right depth image button callback function.
 *
 * @param state Button state
 * @param user_data Pointer to node instance
 */
void ImageNode::buttonDepthRightCallback(int state, void *user_data) {
	ImageNode *p_this = reinterpret_cast<ImageNode *>(user_data);
	p_this->m_show_depth_right = !p_this->m_show_depth_right;
	if (p_this->m_depth_right_subscription == nullptr) {
		if (p_this->m_verbose) std::cout << "subscribe depth right" << std::endl;
		p_this->m_depth_right_subscription = p_this->create_subscription<sensor_msgs::msg::Image>(
		    p_this->m_topic_depth_right, p_this->m_qos_profile,
		    std::bind(&ImageNode::depthRightCallback, p_this, std::placeholders::_1));
		cv::namedWindow(p_this->m_window_name_depth_right, cv::WINDOW_NORMAL);
	} else {
		if (p_this->m_verbose) std::cout << "unsubscribe depth right" << std::endl;
		p_this->m_depth_right_subscription.reset();
		cv::destroyWindow(p_this->m_window_name_depth_right);
		if (!(p_this->m_show_fused || p_this->m_show_depth_left || p_this->m_show_depth_right ||
		      p_this->m_show_frameset_left || p_this->m_show_frameset_right)) {
			rclcpp::shutdown();
		}
	}
}

/**
 * @brief Left frameset button callback function.
 *
 * @param state Button state
 * @param user_data Pointer to node instance
 */
void ImageNode::buttonFramesetLeftCallback(int state, void *user_data) {
	ImageNode *p_this = reinterpret_cast<ImageNode *>(user_data);
	p_this->m_show_frameset_left = !p_this->m_show_frameset_left;
	if (p_this->m_frameset_left_subscription == nullptr) {
		if (p_this->m_verbose) std::cout << "subscribe frameset left" << std::endl;
		p_this->m_frameset_left_subscription = p_this->create_subscription<camera_interfaces::msg::DepthFrameset>(
		    p_this->m_topic_frameset_left, p_this->m_qos_profile,
		    std::bind(&ImageNode::framesetLeftCallback, p_this, std::placeholders::_1));
		cv::namedWindow(p_this->m_window_name_frameset_left, cv::WINDOW_NORMAL);
	} else {
		if (p_this->m_verbose) std::cout << "unsubscribe frameset left" << std::endl;
		p_this->m_frameset_left_subscription.reset();
		cv::destroyWindow(p_this->m_window_name_frameset_left);
		if (!(p_this->m_show_fused || p_this->m_show_depth_left || p_this->m_show_depth_right ||
		      p_this->m_show_frameset_left || p_this->m_show_frameset_right)) {
			rclcpp::shutdown();
		}
	}
}

/**
 * @brief Right frameset button callback function.
 *
 * @param state Button state
 * @param user_data Pointer to node instance
 */
void ImageNode::buttonFramesetRightCallback(int state, void *user_data) {
	ImageNode *p_this = reinterpret_cast<ImageNode *>(user_data);
	p_this->m_show_frameset_right = !p_this->m_show_frameset_right;
	if (p_this->m_frameset_right_subscription == nullptr) {
		if (p_this->m_verbose) std::cout << "subscribe frameset right" << std::endl;
		p_this->m_frameset_right_subscription = p_this->create_subscription<camera_interfaces::msg::DepthFrameset>(
		    p_this->m_topic_frameset_right, p_this->m_qos_profile,
		    std::bind(&ImageNode::framesetRightCallback, p_this, std::placeholders::_1));
		cv::namedWindow(p_this->m_window_name_frameset_right, cv::WINDOW_NORMAL);
	} else {
		if (p_this->m_verbose) std::cout << "unsubscribe frameset right" << std::endl;
		p_this->m_frameset_right_subscription.reset();
		cv::destroyWindow(p_this->m_window_name_frameset_right);
		if (!(p_this->m_show_fused || p_this->m_show_depth_left || p_this->m_show_depth_right ||
		      p_this->m_show_frameset_left || p_this->m_show_frameset_right)) {
			rclcpp::shutdown();
		}
	}
}

/**
 * @brief Callback function for reveived image message.
 * @param img_msg Received image message
 */
/*
void ImageNode::depthCallback(sensor_msgs::msg::Image::SharedPtr img_msg) {
  cv::Size image_size(static_cast<int>(img_msg->width), static_cast<int>(img_msg->height));
  cv::Mat color_image(image_size, CV_16UC1, (void *)img_msg->data.data(), cv::Mat::AUTO_STEP);
  // cv::setWindowTitle(m_window_name_depth, std::to_string(m_loop_duration_depth));
  cv::cvtColor(color_image, color_image, cv::COLOR_RGB2BGR);
  imshow(m_window_name_depth, color_image);

  if (!(cv::waitKey(1) < 0 && cv::getWindowProperty(m_window_name_depth, cv::WND_PROP_AUTOSIZE) >= 0))
    rclcpp::shutdown();

  m_loop_duration_depth = (Clock::now() - m_callback_time_depth).count() / 1e6;
  m_callback_time_depth = Clock::now();
}
*/

/*
void ImageNode::imageSmallCallback(sensor_msgs::msg::Image::SharedPtr img_msg) {
  cv::Size image_size(static_cast<int>(img_msg->width), static_cast<int>(img_msg->height));
  cv::Mat color_image(image_size, CV_8UC3, (void *)img_msg->data.data(), cv::Mat::AUTO_STEP);
  // cv::setWindowTitle(m_window_name_image_small, std::to_string(m_loop_duration_image_small));
  cv::cvtColor(color_image, color_image, cv::COLOR_RGB2BGR);
  imshow(m_window_name_image_small, color_image);

  if (!(cv::waitKey(1) < 0 && cv::getWindowProperty(m_window_name_image_small, cv::WND_PROP_AUTOSIZE) >= 0))
    rclcpp::shutdown();

  // m_loop_duration_image_small = (Clock::now() - m_callback_time_image_small).count() / 1e6;
  // m_callback_time_image_small = Clock::now();
}
*/

/**
 * @brief Callback function for reveived image message.
 * @param img_msg Received image message
 */
/*
void ImageNode::framesetCallback(camera_interfaces::msg::DepthFrameset::UniquePtr fset_msg) {
  cv::Size image_size(static_cast<int>(fset_msg.get()->color_image.width),
                      static_cast<int>(fset_msg.get()->color_image.height));
  cv::Mat color_image(image_size, CV_8UC3, (void *)fset_msg.get()->color_image.data.data(), cv::Mat::AUTO_STEP);
  cv::Mat depth_image(image_size, CV_16UC1, (void *)fset_msg.get()->depth_image.data.data(), cv::Mat::AUTO_STEP);
  // cv::Mat color_image = cv::imdecode(cv::Mat(fset_msg->color_image.data), cv::IMREAD_UNCHANGED);
  // cv::setWindowTitle(m_window_name_frameset, std::to_string(m_loop_duration));
  // cv::setWindowTitle(m_window_name, std::to_string(0.0));
  cv::cvtColor(color_image, color_image, cv::COLOR_RGB2BGR);
  cv::convertScaleAbs(depth_image, depth_image, 0.1);
  imshow(m_window_name_color, color_image);
  imshow(m_window_name_depth, depth_image);

  if (!(cv::waitKey(1) < 0 && ((cv::getWindowProperty(m_window_name_color, cv::WND_PROP_AUTOSIZE) >= 0) ||
                               (cv::getWindowProperty(m_window_name_depth, cv::WND_PROP_AUTOSIZE) >= 0))))
    rclcpp::shutdown();

  // m_loop_duration = (Clock::now()- m_callback_time).count() / 1e6;
  // m_callback_time = Clock::now();
}
*/
