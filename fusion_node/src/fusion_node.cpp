#include "fusion_node.hpp"

using namespace std::chrono_literals;

/**
 * @brief Constructor.
 * @param name Node name
 */
FusionNode::FusionNode(const std::string &name) : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true)) {
	this->node_name = name;
	package_share_directory = ament_index_cpp::get_package_share_directory(name);

	this->declare_parameter<bool>("verbose", false);
	this->declare_parameter<bool>("debug", false);
	this->get_parameter("verbose", verbose);
	this->get_parameter("debug", debug);

	this->declare_parameter<bool>("qos_sensor_data", false);
	this->declare_parameter<int>("qos_history_depth", 5);
	this->get_parameter("qos_sensor_data", qos_sensor_data);
	this->get_parameter("qos_history_depth", qos_history_depth);

	this->declare_parameter<bool>("save_data", false);
	this->get_parameter("save_data", save_data);
	this->declare_parameter("depth_scale_left", -1.0);
	this->declare_parameter("depth_scale_right", -1.0);
	this->get_parameter("depth_scale_left", depth_scale_left);
	this->get_parameter("depth_scale_right", depth_scale_right);

	this->declare_parameter("min_depth", 0.5);
	this->declare_parameter("max_depth", 2.0);
	this->get_parameter("min_depth", min_depth);
	this->get_parameter("max_depth", max_depth);

	this->declare_parameter<bool>("align_frames", false);
	this->get_parameter("align_frames", align_frames);

	this->declare_parameter<std::string>("topic_frameset_left", "/camera_left/frameset");
	this->get_parameter("topic_frameset_left", topic_frameset_left);
	this->declare_parameter<std::string>("topic_frameset_right", "/camera_right/frameset");
	this->get_parameter("topic_frameset_right", topic_frameset_right);

	this->declare_parameter<bool>("profiling.enable_profiling", false);
	this->get_parameter("profiling.enable_profiling", enable_profiling);
	this->declare_parameter<int>("profiling.log_size", 400);
	this->get_parameter("profiling.log_size", profiling_size);
	this->declare_parameter<std::string>("profiling.filename", node_name + "_profiling.txt");
	this->get_parameter("profiling.filename", profiling_filename);

	this->declare_parameter<bool>("vertical_image", true);
	this->get_parameter("vertical_image", vertical_image);
	this->declare_parameter<bool>("mirror_image", true);
	this->get_parameter("mirror_image", mirror_image);

	this->declare_parameter<bool>("save_pointclouds", false);
	this->get_parameter("save_pointclouds", save_pointclouds);

	this->declare_parameter<bool>("use_median_filter", false);
	this->get_parameter("use_median_filter", use_median_filter);

	this->declare_parameter<bool>("set_camera_pose", false);
	this->get_parameter("set_camera_pose", set_camera_pose);
	this->declare_parameter("camera_translation", std::vector<double>({0., 0., 0.}));
	this->get_parameter("camera_translation", camera_translation);
	this->declare_parameter("camera_rotation", std::vector<double>({0., 0., 0.}));
	this->get_parameter("camera_rotation", camera_rotation);

	if (verbose) {
		std::cout << "+==========[ Fusion Node ]==========+" << std::endl;
		std::cout << "Quality of service:" << std::endl;
		std::cout << " sensor data: " << qos_sensor_data << std::endl;
		std::cout << " history depth: " << qos_history_depth << std::endl;
	}
}

/**
 * @brief Destructor.
 */
FusionNode::~FusionNode() {}

/**
 * @brief Initialize ROS node.
 */
void FusionNode::init() {
	// Camera parameters service
	cam_left_param_client =
	    this->create_client<camera_interfaces::srv::GetCameraParameters>("/camera_left/get_camera_parameters");
	cam_right_param_client =
	    this->create_client<camera_interfaces::srv::GetCameraParameters>("/camera_right/get_camera_parameters");

	// Wait for left camera parameter service
	while (!cam_left_param_client->wait_for_service(1s)) {
		if (!rclcpp::ok()) {
			std::cout << "Interrupted while waiting for the left camera. Exiting." << std::endl;
			exit_request->store(true);
			return;
		}
		std::cout << "Waiting for left camera..." << std::endl;
	}
	// Fetch camera parameters from service
	auto request = std::make_shared<camera_interfaces::srv::GetCameraParameters::Request>();
	auto result = cam_left_param_client->async_send_request(request);
	if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == FutureReturnCode::SUCCESS) {
		camerainfo_depth_left = result.get()->depth_intrinsics;
		camerainfo_color_left = result.get()->color_intrinsics;
		for (unsigned long i = 0; i < 3; i++) extrinsics_left.translation[i] = result.get()->extrinsic_translation[i];
		for (unsigned long i = 0; i < 9; i++) extrinsics_left.rotation[i] = result.get()->extrinsic_rotation[i];
	} else {
		std::cout << "Failed to call service" << std::endl;
		exit_request->store(true);
		return;
	}

	// Wait for right camera parameter service
	while (!cam_right_param_client->wait_for_service(1s)) {
		if (!rclcpp::ok()) {
			std::cout << "Interrupted while waiting for the right camera. Exiting." << std::endl;
			exit_request->store(true);
			return;
		}
		std::cout << "Waiting for right camera..." << std::endl;
	}
	// Fetch camera parameters from service
	result = cam_right_param_client->async_send_request(request);
	if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == FutureReturnCode::SUCCESS) {
		camerainfo_depth_right = result.get()->depth_intrinsics;
		camerainfo_color_right = result.get()->color_intrinsics;
		for (unsigned long i = 0; i < 3; i++) extrinsics_right.translation[i] = result.get()->extrinsic_translation[i];
		for (unsigned long i = 0; i < 9; i++) extrinsics_right.rotation[i] = result.get()->extrinsic_rotation[i];
	} else {
		std::cout << "Failed to call service" << std::endl;
		exit_request->store(true);
		return;
	}

	// Camera intrinsics
	cameraInfo2Intrinsics(camerainfo_depth_left, intrinsics_depth_left);
	cameraInfo2Intrinsics(camerainfo_color_left, intrinsics_color_left);
	cameraInfo2Intrinsics(camerainfo_depth_right, intrinsics_depth_right);
	cameraInfo2Intrinsics(camerainfo_color_right, intrinsics_color_right);
	intrinsics_fused_image = intrinsics_color_left;
	if (vertical_image) {
		intrinsics_fused_image.width = intrinsics_color_left.height;
		intrinsics_fused_image.height = intrinsics_color_left.width;
		intrinsics_fused_image.fx = intrinsics_color_left.fy;
		intrinsics_fused_image.fy = intrinsics_color_left.fx;
	}
	// D435: focal length 1.93 mm 920 pixel, D455: focal length 1.88 mm 631 pixel
	intrinsics_fused_image.ppx = intrinsics_fused_image.width / 2;
	intrinsics_fused_image.ppy = intrinsics_fused_image.height / 2;

	// Virtual camera pose
	if (set_camera_pose) {
		camera_transform = Eigen::Affine3d::Identity();
		camera_transform.prerotate(Eigen::AngleAxisd(deg2rad(90.), Eigen::Vector3d::UnitZ()));
		// Axis: X: right, Y: down, Z: forward
		Eigen::Vector3d translation(camera_translation[0], camera_translation[1], camera_translation[2]);
		Eigen::Matrix3d rotation;
		rotation = Eigen::AngleAxisd(deg2rad(camera_rotation[0]), Eigen::Vector3d::UnitX()) *
		           Eigen::AngleAxisd(deg2rad(camera_rotation[1]), Eigen::Vector3d::UnitY()) *
		           Eigen::AngleAxisd(deg2rad(camera_rotation[2]), Eigen::Vector3d::UnitZ());
		camera_transform.pretranslate(-translation);
		camera_transform.prerotate(rotation.inverse());
	}

	// Quality of service
	if (qos_sensor_data) qos_profile = rclcpp::SensorDataQoS();
	qos_profile = qos_profile.keep_last(static_cast<size_t>(qos_history_depth));

	// Fused color image publisher
	fused_publisher = image_transport::create_publisher(this, "/fused_image", qos_profile.get_rmw_qos_profile());

	// Initialize framesets
	initSyncFramesets();

	// Transformation subscriber
	CallbackGroup::SharedPtr callback_group_transform = this->create_callback_group(CallbackGroupType::MutuallyExclusive);
	rclcpp::SubscriptionOptions transform_subscription_options;
	transform_subscription_options.callback_group = callback_group_transform;
	transform_subscription = this->create_subscription<geometry_msgs::msg::TransformStamped>(
	    "/registration/transform", rclcpp::SystemDefaultsQoS(),
	    std::bind(&FusionNode::transformCallback, this, std::placeholders::_1), transform_subscription_options);

	// Init cuda
	Kernels::init_cuda(false);
	cudaStreamCreateWithFlags(&cuda_stream_left, cudaStreamNonBlocking);
	cudaStreamCreateWithFlags(&cuda_stream_right, cudaStreamNonBlocking);

	// Allocate frames
	allocateFrames();

	if (verbose) {
		std::cout << "qos:" << std::endl;
		std::cout << " sensor data: "
		          << (qos_profile.get_rmw_qos_profile().reliability == rmw_qos_profile_sensor_data.reliability)
		          << std::endl;
		std::cout << " history depth: " << qos_profile.get_rmw_qos_profile().depth << std::endl;
	}
}

/**
 * @brief Initialize frameset synchronization.
 */
void FusionNode::initSyncFramesets() {
	callback_group_left = this->create_callback_group(CallbackGroupType::MutuallyExclusive);
	rclcpp::SubscriptionOptions subscription_options_left;
	subscription_options_left.callback_group = callback_group_left;
	callback_group_right = this->create_callback_group(CallbackGroupType::Reentrant);
	rclcpp::SubscriptionOptions subscription_options_right;
	subscription_options_right.callback_group = callback_group_right;

	if (!direct_callback) {
		frameset_left_subscription = this->create_subscription<camera_interfaces::msg::DepthFrameset>(
		    topic_frameset_left, qos_profile, std::bind(&FusionNode::framesetLeftCallback, this, std::placeholders::_1),
		    subscription_options_left);
	}
	frameset_right_subscription = this->create_subscription<camera_interfaces::msg::DepthFrameset>(
	    topic_frameset_right, qos_profile, std::bind(&FusionNode::framesetRightCallback, this, std::placeholders::_1),
	    subscription_options_right);

	callback_group_timer = this->create_callback_group(CallbackGroupType::Reentrant);
	this->sync_timer =
	    this->create_wall_timer(std::chrono::nanoseconds(static_cast<int>(1e9 / 30)),
	                            std::bind(&FusionNode::syncFramesetsTimerCallback, this), callback_group_timer);

	frameset_left_counter.store(0);
	frameset_right_counter.store(0);

	if (verbose) {
		std::cout << "sync framesets from topics: " << topic_frameset_left << ", " << topic_frameset_right << std::endl;
	}
	std::cout << "+==========[ Fusion Node Started ]==========+" << std::endl;
}

/**
 * @brief Callback for left frameset to synchronize.
 * @param msg Left frameset message
 */
void FusionNode::framesetLeftCallback(camera_interfaces::msg::DepthFrameset::UniquePtr msg) {
	if (!caminfo_left_received) {
		camerainfo_depth_left = msg->depth_info;
		camerainfo_color_left = msg->color_info;
		caminfo_left_received = true;
	}

	std::lock_guard<std::mutex> lock(mutex_frameset_left);
	if (frameset_left_msg_queue.size() > 100) {
		frameset_left_msg_queue.pop();
	}
	frameset_left_msg_queue.push(std::move(msg));

	frameset_left_counter++;
}

/**
 * @brief Callback for right frameset to synchronize.
 * @param msg Right frameset message
 */
void FusionNode::framesetRightCallback(camera_interfaces::msg::DepthFrameset::UniquePtr msg) {
	if (!caminfo_right_received) {
		camerainfo_depth_right = msg->depth_info;
		camerainfo_color_right = msg->color_info;
		caminfo_right_received = true;
	}

	std::lock_guard<std::mutex> lock(mutex_frameset_right);
	if (frameset_right_msg_queue.size() > 100) {
		frameset_right_msg_queue.pop();
	}
	frameset_right_msg_queue.push(std::move(msg));

	frameset_right_counter++;
}

/**
 * @brief ROS timer callback for frameset synchonization.
 */
void FusionNode::syncFramesetsTimerCallback() {
	time_point processing_time = system_clock::now();

	bool has_framesets = false;
	sync_callback_counter++;

	double frame_rate = 30.;
	long frame_duration_ns = static_cast<long>(1e9 * 1 / frame_rate);
	long max_stamp_diff = frame_duration_ns / 2;

	camera_interfaces::msg::DepthFrameset::UniquePtr frameset_left_msg;
	camera_interfaces::msg::DepthFrameset::UniquePtr frameset_right_msg;
	rclcpp::Time ros_timestamp_left = this->now();
	rclcpp::Time ros_timestamp_right = ros_timestamp_left;
	long stamp_diff = std::numeric_limits<long>::max();

	if (!frameset_right_msg_queue.empty() && !frameset_left_msg_queue.empty()) {
		if (frameset_right_msg_queue.size() <= frameset_left_msg_queue.size()) {
			{
				std::unique_lock<std::mutex> lock_right(mutex_frameset_right);
				frameset_right_msg = std::move(frameset_right_msg_queue.front());
				frameset_right_msg_queue.pop();
				ros_timestamp_right = frameset_right_msg->header.stamp;
			}
			{
				std::unique_lock<std::mutex> lock_left(mutex_frameset_left);
				ros_timestamp_left = frameset_left_msg_queue.front()->header.stamp;
				stamp_diff = (ros_timestamp_left - ros_timestamp_right).nanoseconds();
				while (std::abs(stamp_diff) > max_stamp_diff && frameset_left_msg_queue.size() > 1) {
					sync_drop_left_counter++;
					frameset_left_msg_queue.pop();
					ros_timestamp_left = frameset_left_msg_queue.front()->header.stamp;
					stamp_diff = (ros_timestamp_left - ros_timestamp_right).nanoseconds();
				}
				if (std::abs(stamp_diff) <= max_stamp_diff) {
					frameset_left_msg = std::move(frameset_left_msg_queue.front());
					has_framesets = true;
				}
				frameset_left_msg_queue.pop();
			}
		} else {
			{
				std::unique_lock<std::mutex> lock_left(mutex_frameset_left);
				frameset_left_msg = std::move(frameset_left_msg_queue.front());
				frameset_left_msg_queue.pop();
				ros_timestamp_left = frameset_left_msg->header.stamp;
			}
			{
				std::unique_lock<std::mutex> lock_right(mutex_frameset_right);
				ros_timestamp_right = frameset_right_msg_queue.front()->header.stamp;
				stamp_diff = (ros_timestamp_left - ros_timestamp_right).nanoseconds();
				while (std::abs(stamp_diff) > max_stamp_diff && frameset_right_msg_queue.size() > 1) {
					sync_drop_right_counter++;
					frameset_right_msg_queue.pop();
					ros_timestamp_right = frameset_right_msg_queue.front()->header.stamp;
					stamp_diff = (ros_timestamp_left - ros_timestamp_right).nanoseconds();
				}
				if (std::abs(stamp_diff) <= max_stamp_diff) {
					frameset_right_msg = std::move(frameset_right_msg_queue.front());
					has_framesets = true;
				}
				frameset_right_msg_queue.pop();
			}
		}

		if (has_framesets) {
			rclcpp::Duration ros_latency = this->now() - frameset_right_msg->header.stamp;
			double latency = ros_latency.to_chrono<std::chrono::nanoseconds>().count() / 1e6;

			sync_latency_acc += latency;
			sync_hit_counter++;
			sync_hit_counter_acc++;
		} else {
			sync_missed_counter++;
		}
	} else {
		sync_missed_counter++;
	}

	sync_loop_counter++;
	if (debug && sync_loop_counter % 100 == 0) {
		double drop_left_rate = static_cast<double>(sync_drop_left_counter) / frameset_left_counter.load();
		double drop_right_rate = static_cast<double>(sync_drop_right_counter) / frameset_right_counter.load();
		std::cout << "+--------------------" << std::endl;
		std::cout << "sync_missed_counter: " << sync_missed_counter << std::endl;
		std::cout << "frameset_left_counter: " << frameset_left_counter.load() << std::endl;
		std::cout << "frameset_right_counter: " << frameset_right_counter.load() << std::endl;
		std::cout << "sync_drop_left_counter: " << sync_drop_left_counter << std::endl;
		std::cout << "sync_drop_right_counter: " << sync_drop_right_counter << std::endl;
		std::cout << "left drop rate: " << drop_left_rate << std::endl;
		std::cout << "right drop rate: " << drop_right_rate << std::endl;
		std::cout << "queue_size_left = " << frameset_left_msg_queue.size() << std::endl;
		std::cout << "queue_size_right = " << frameset_right_msg_queue.size() << std::endl;

		std::cout << "sync_loop_counter: " << sync_loop_counter << std::endl;
		std::cout << "sync_hit_counter: " << sync_hit_counter << std::endl;
		std::cout << "sync_missed_counter: " << sync_missed_counter << std::endl;
		double hit_rate = double(sync_hit_counter) / sync_loop_counter;
		std::cout << "hit rate: " << hit_rate << std::endl;
		double callback_fps = sync_callback_counter / sync_callback_duration_acc;
		std::cout << "callback FPS: " << callback_fps << std::endl;
		double sync_fps = sync_hit_counter_acc / sync_callback_duration_acc;
		std::cout << "sync FPS: " << sync_fps << std::endl << std::endl;
		double sync_latency = sync_latency_acc / sync_hit_counter;
		std::cout << "sync latency: " << sync_latency << std::endl << std::endl;

		sync_loop_counter = 0;
		sync_hit_counter = 0;
		sync_missed_counter = 0;
		sync_drop_left_counter = 0;
		sync_drop_right_counter = 0;
		frameset_left_counter.store(0);
		frameset_right_counter.store(0);
		sync_latency_acc = 0;
	}

	if (has_framesets) {
		processFrames(std::move(frameset_left_msg), std::move(frameset_right_msg));
	}

	double callback_duration = (system_clock::now() - sync_callback_time).count() / 1e6;
	sync_callback_duration_acc += callback_duration / 1e3;

	if (debug && sync_loop_counter == 0) {
		double processing_duration = (system_clock::now() - processing_time).count() / 1e6;
		std::cout << "processing_duration: " << processing_duration << " ms" << std::endl;
	}

	sync_callback_time = system_clock::now();
}

/**
 * @brief Process frames from synchonized framesets.
 * @param frameset_msg_left Left frameset message
 * @param frameset_msg_right Right frameset message
 */
void FusionNode::processFrames(camera_interfaces::msg::DepthFrameset::UniquePtr frameset_msg_left,
                               camera_interfaces::msg::DepthFrameset::UniquePtr frameset_msg_right) {
	if (exit_request->load()) {
		sync_timer->cancel();
		rclcpp::shutdown();
	}
	time_point callback_start = system_clock::now();
	time_point time_start = system_clock::now();

	depth_frame_left = reinterpret_cast<uint16_t *>(frameset_msg_left.get()->depth_image.data.data());
	color_frame_left = reinterpret_cast<uint8_t *>(frameset_msg_left.get()->color_image.data.data());
	depth_frame_right = reinterpret_cast<uint16_t *>(frameset_msg_right.get()->depth_image.data.data());
	color_frame_right = reinterpret_cast<uint8_t *>(frameset_msg_right.get()->color_image.data.data());

	// Left frame
	frameset_left.setColorFrame(color_frame_left);
	if (align_frames) {
		frameset_left.setDepthFrameUnaligned(depth_frame_left);
		frameset_left.alignDepthToColor(depth_scale_left);
	} else {
		frameset_left.setDepthFrame(depth_frame_left);
	}

	// Right frame
	frameset_right.setColorFrame(color_frame_right);
	if (align_frames) {
		frameset_left.setDepthFrameUnaligned(depth_frame_left);
		frameset_left.alignDepthToColor(depth_scale_left);
	} else {
		frameset_right.setDepthFrame(depth_frame_right);
	}

	double copy_to_gpu_duration = getTiming(time_start);

	const std::array<int, 4> roi_left = {0, 0, intrinsics_depth_left.width, intrinsics_depth_left.height};
	frameset_left.filterDepth(min_depth, max_depth, depth_scale_left, roi_left);
	frameset_right.filterDepth(min_depth, max_depth, depth_scale_right);

	double filter_frames_duration = getTiming(time_start);

	// Deproject to point clouds
	Pointcloud fused_cloud;
	fused_cloud.setStream(frameset_left.getStream());
	fused_cloud.allocate(frameset_left.getMaskCount() + frameset_right.getMaskCount());
	fused_cloud.deproject(frameset_left, depth_scale_left);
	Pointcloud cloud_right;
	cloud_right.setStream(frameset_right.getStream());
	cloud_right.deproject(frameset_right, depth_scale_right);

	double deproject_duration = getTiming(time_start);

	// Transform point clouds
	cloud_right.transform(right_transform);
	cudaStreamSynchronize(*frameset_right.getStream());

	double transform_right_duration = getTiming(time_start);

	// Fuse point clouds
	fused_cloud.append(cloud_right);

	double fuse_duration = getTiming(time_start);

	// Transform fused pointcloud
	Eigen::Affine3d fused_transform = Eigen::Affine3d::Identity();
	if (!set_camera_pose) {
		interpolateTransform(fused_transform, left_transform, right_transform);
	} else {
		fused_transform = camera_transform;
	}

	fused_cloud.transform(fused_transform);

	double transform_fused_duration = getTiming(time_start);

	// Project fused pointcloud
	fused_cloud.project(fused_frameset, mirror_image);

	double project_duration = getTiming(time_start);

	// Filter fused image
	fused_frameset.filterColor(use_median_filter);

	double filter_fused_duration = getTiming(time_start);

	// Copy image from gpu to host memory
	fused_frameset.copyColorToHost(fused_frame);

	double copy_from_gpu_duration = getTiming(time_start);

	// Save point clouds to ply files for debugging
	if (save_pointclouds) {
		Pointcloud cloud_left;
		cloud_left.setStream(frameset_left.getStream());
		cloud_left.deproject(frameset_left, depth_scale_left);
		save_pointclouds_ply(cloud_left, cloud_right, fused_transform);
	}

	// Publish fused image
	const uint8_t *fused_msg_bytes = reinterpret_cast<const uint8_t *>(fused_frame);
	fused_msg.header.frame_id = "camera_left_color_optical_frame";
	fused_msg.header.stamp = frameset_msg_left->header.stamp;
	fused_msg.width = static_cast<uint>(intrinsics_fused_image.width);
	fused_msg.height = static_cast<uint>(intrinsics_fused_image.height);
	fused_msg.is_bigendian = false;
	fused_msg.step = fused_msg.width * 3 * sizeof(uint8_t);
	fused_msg.encoding = "rgb8";
	fused_msg.data.assign(fused_msg_bytes, fused_msg_bytes + (fused_msg.step * fused_msg.height));
	fused_publisher.publish(fused_msg);

	double publish_duration = getTiming(time_start);

	// Profiling
	if (enable_profiling) {
		double loop_duration = (system_clock::now() - global_timer).count() / 1e6;
		global_timer = system_clock::now();
		double callback_duration = (system_clock::now() - callback_start).count() / 1e6;
		rclcpp::Time ros_timestamp_left = frameset_msg_left->header.stamp;
		rclcpp::Time ros_timestamp_right = frameset_msg_right->header.stamp;
		double stamp_diff = (ros_timestamp_left - ros_timestamp_right).nanoseconds() / 1e6;
		rclcpp::Duration ros_latency = this->now() - frameset_msg_left->header.stamp;
		double latency = ros_latency.to_chrono<std::chrono::nanoseconds>().count() / 1e6;

		std::vector<double> vec;
		vec.push_back(loop_duration);
		vec.push_back(callback_duration);
		vec.push_back(filter_frames_duration);
		vec.push_back(deproject_duration);
		vec.push_back(transform_right_duration);
		vec.push_back(fuse_duration);
		vec.push_back(transform_fused_duration);
		vec.push_back(project_duration);
		vec.push_back(publish_duration);
		vec.push_back(latency);
		vec.push_back(stamp_diff);
		vec.push_back(copy_to_gpu_duration);
		vec.push_back(copy_from_gpu_duration);
		vec.push_back(filter_fused_duration);
		profiling_vec.push_back(vec);

		if (profiling_vec.size() == static_cast<unsigned>(profiling_size)) {
			std::cout << "Write profiling file " << profiling_filename << std::endl;
			std::ofstream profiling_file(profiling_filename);
			for (unsigned i = 0; i < profiling_fields.size(); i++) {
				profiling_file << profiling_fields[i];
				if (i != profiling_fields.size() - 1) profiling_file << ",";
			}
			profiling_file << "\n";
			for (const auto &l : profiling_vec) {
				for (unsigned i = 0; i < l.size(); i++) {
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
 * @brief Allocate memory for images from framesets.
 */
void FusionNode::allocateFrames() {
	// Allocate frames
	depth_frame_left = reinterpret_cast<uint16_t *>(
	    malloc(static_cast<unsigned>(intrinsics_depth_left.width * intrinsics_depth_left.height) * sizeof(uint16_t)));
	color_frame_left = reinterpret_cast<uint8_t *>(
	    malloc(static_cast<unsigned>(intrinsics_color_left.width * intrinsics_color_left.height) * 3 * sizeof(uint8_t)));
	depth_frame_right = reinterpret_cast<uint16_t *>(
	    malloc(static_cast<unsigned>(intrinsics_depth_right.width * intrinsics_depth_right.height) * sizeof(uint16_t)));
	color_frame_right = reinterpret_cast<uint8_t *>(malloc(
	    static_cast<unsigned>(intrinsics_color_right.width * intrinsics_color_right.height) * 3 * sizeof(uint8_t)));

	frameset_left.setStream(&cuda_stream_left);
	frameset_right.setStream(&cuda_stream_right);

	frameset_left.allocateDepthFrame(intrinsics_color_left);  // aligned to color
	frameset_left.allocateColorFrame(intrinsics_color_left);
	frameset_left.allocateDepthFrameUnaligned(intrinsics_depth_left, extrinsics_left);
	frameset_right.allocateDepthFrame(intrinsics_color_right);  // aligned to color
	frameset_right.allocateColorFrame(intrinsics_color_right);
	frameset_right.allocateDepthFrameUnaligned(intrinsics_depth_right, extrinsics_right);

	fused_frameset.setStream(&cuda_stream_left);
	fused_frameset.allocateColorFrame(intrinsics_fused_image);
	fused_frame = reinterpret_cast<uint8_t *>(malloc(
	    static_cast<unsigned>(intrinsics_fused_image.width * intrinsics_fused_image.height) * 3 * sizeof(uint8_t)));

	// Set save images
	std::string file_prefix = node_name;
	file_prefix.erase(std::remove(file_prefix.begin(), file_prefix.end(), '/'), file_prefix.end());
	frameset_left.setSaveImages(save_data, package_share_directory + "/data", file_prefix + "_left");
	frameset_right.setSaveImages(save_data, package_share_directory + "/data", file_prefix + "_right");
	fused_frameset.setSaveImages(save_data, package_share_directory + "/data", "fused");
}

/**
 * @brief Convert camera intrinsics message to intrinsics structure.
 * @param camerainfo Camera intrinsics message
 * @param intrinsics Intrinsics structure
 */
void FusionNode::cameraInfo2Intrinsics(const sensor_msgs::msg::CameraInfo &camerainfo, Intrinsics &intrinsics) {
	intrinsics.width = static_cast<int>(camerainfo.width);
	intrinsics.height = static_cast<int>(camerainfo.height);
	intrinsics.ppx = static_cast<int>(camerainfo.k[2]);
	intrinsics.ppy = static_cast<int>(camerainfo.k[5]);
	intrinsics.fx = static_cast<int>(camerainfo.k[0]);
	intrinsics.fy = static_cast<int>(camerainfo.k[4]);
	intrinsics.model = Distortion::DISTORTION_BROWN_CONRADY;
	for (uint i = 0; i < 5; i++) {
		intrinsics.coeffs[i] = static_cast<float>(camerainfo.d[i]);
	}
}

/**
 * @brief Interpolate between affine transformations.
 * @param interpolated_transform Interpolated mean affine transformation
 * @param left_transform First affine transformation
 * @param right_transform Second affine transformation
 */
void FusionNode::interpolateTransform(Eigen::Affine3d &interpolated_transform, const Eigen::Affine3d &left_transform,
                                      const Eigen::Affine3d &right_transform) {
	interpolated_transform = Eigen::Affine3d::Identity();

	Eigen::Quaterniond left_rotation(left_transform.rotation());
	Eigen::Quaterniond right_rotation(right_transform.rotation());
	// spherical linear interpolation between the two quaternions
	Eigen::Quaterniond fused_rotation = left_rotation.slerp(0.5, right_rotation);
	Eigen::Vector3d left_translation = left_transform.translation();
	Eigen::Vector3d right_translation = right_transform.translation();
	// linear interpolation between the two translation vectors
	Eigen::Vector3d fused_translation = left_translation * 0.5 + right_translation * 0.5;
	interpolated_transform.prerotate(fused_rotation);
	interpolated_transform.pretranslate(fused_translation);
	if (right_rotation.w() < 0) interpolated_transform = interpolated_transform.inverse();
}

/**
 * @brief Registration transformation message callback.
 * @param transform_msg Registration transformation message
 */
void FusionNode::transformCallback(const geometry_msgs::msg::TransformStamped::SharedPtr transform_msg) {
	right_transform = tf2::transformToEigen(transform_msg->transform);
}

/**
 * @brief Get timing in milliseconds from start time until now and set start time to now.
 * @param start_time Start time point
 * @return Time elapsed since start in milliseconds
 */
double FusionNode::getTiming(time_point &start_time) {
	if (enable_profiling) {
		cudaDeviceSynchronize();
		double timing = (system_clock::now() - start_time).count() / 1e6;
		start_time = system_clock::now();
		return timing;
	} else {
		return 0.;
	}
}

/**
 * @brief Save colored point clouds to ply files.
 * @param cloud_left Point cloud for left camera
 * @param cloud_right Point cloud for right camera
 * @param fused_transform Transformation for fused pointcloud
 */
void FusionNode::save_pointclouds_ply(Pointcloud &cloud_left, Pointcloud &cloud_right,
                                      Eigen::Affine3d &fused_transform) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_left(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl_cloud_left->resize(frameset_left.getMaskCount());
	float *pcl_cloud_left_data = pcl_cloud_left->points.data()->_PointXYZRGB::data;
	cloud_left.copyToHost(pcl_cloud_left_data);

	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator points_it = pcl_cloud_left->begin();
	     points_it != pcl_cloud_left->end(); ++points_it) {
		float *rgb_float = &points_it->_PointXYZRGB::data[3];
		uint8_t *rgb_uint8 = reinterpret_cast<uint8_t *>(rgb_float);
		points_it->b = rgb_uint8[0];
		points_it->g = rgb_uint8[1];
		points_it->r = rgb_uint8[2];
	}
	pcl::io::savePLYFileASCII(package_share_directory + "/data/cloud_left_input.ply", *pcl_cloud_left);

	cloud_left.transform(fused_transform);
	cloud_left.copyToHost(pcl_cloud_left_data);
	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator points_it = pcl_cloud_left->begin();
	     points_it != pcl_cloud_left->end(); ++points_it) {
		float *rgb_float = &points_it->_PointXYZRGB::data[3];
		uint8_t *rgb_uint8 = reinterpret_cast<uint8_t *>(rgb_float);
		points_it->b = rgb_uint8[0];
		points_it->g = rgb_uint8[1];
		points_it->r = rgb_uint8[2];
	}
	pcl::io::savePLYFileASCII(package_share_directory + "/data/cloud_left_fused.ply", *pcl_cloud_left);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_right(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl_cloud_right->resize(frameset_right.getMaskCount());
	float *pcl_cloud_right_data = pcl_cloud_right->points.data()->_PointXYZRGB::data;
	cloud_right.copyToHost(pcl_cloud_right_data);
	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator points_it = pcl_cloud_right->begin();
	     points_it != pcl_cloud_right->end(); ++points_it) {
		float *rgb_float = &points_it->_PointXYZRGB::data[3];
		uint8_t *rgb_uint8 = reinterpret_cast<uint8_t *>(rgb_float);
		points_it->b = rgb_uint8[0];
		points_it->g = rgb_uint8[1];
		points_it->r = rgb_uint8[2];
	}
	pcl::io::savePLYFileASCII(package_share_directory + "/data/cloud_right_input.ply", *pcl_cloud_right);

	cloud_right.transform(fused_transform);
	cloud_right.copyToHost(pcl_cloud_right_data);
	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator points_it = pcl_cloud_right->begin();
	     points_it != pcl_cloud_right->end(); ++points_it) {
		float *rgb_float = &points_it->_PointXYZRGB::data[3];
		uint8_t *rgb_uint8 = reinterpret_cast<uint8_t *>(rgb_float);
		points_it->b = rgb_uint8[0];
		points_it->g = rgb_uint8[1];
		points_it->r = rgb_uint8[2];
	}
	pcl::io::savePLYFileASCII(package_share_directory + "/data/cloud_right_fused.ply", *pcl_cloud_right);

	std::cout << "## Saved ply clouds" << std::endl;
	ply_counter++;
	if (ply_counter == 3) {
		exit_request->store(true);
		sync_timer->cancel();
		rclcpp::shutdown();
	}
}
