#include "fusion_node.hpp"

using namespace std::chrono_literals;

/**
 * @brief Constructor.
 * @param name Node name
 */
FusionNode::FusionNode(const std::string &name) :
	Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
	m_node_name               = name;
	m_package_share_directory = ament_index_cpp::get_package_share_directory(name);

	this->declare_parameter<bool>("verbose", false);
	this->declare_parameter<bool>("debug", false);
	this->get_parameter("verbose", m_verbose);
	this->get_parameter("debug", m_debug);

	this->declare_parameter<bool>("qos_sensor_data", false);
	this->declare_parameter<int>("qos_history_depth", 5);
	this->get_parameter("qos_sensor_data", m_qos_sensor_data);
	this->get_parameter("qos_history_depth", m_qos_history_depth);

	this->declare_parameter<bool>("save_data", false);
	this->get_parameter("save_data", m_save_data);
	this->declare_parameter("depth_scale_left", -1.0);
	this->declare_parameter("depth_scale_right", -1.0);
	this->get_parameter("depth_scale_left", m_depth_scale_left);
	this->get_parameter("depth_scale_right", m_depth_scale_right);

	this->declare_parameter("min_depth", 0.5);
	this->declare_parameter("max_depth", 2.0);
	this->get_parameter("min_depth", m_min_depth);
	this->get_parameter("max_depth", m_max_depth);

	this->declare_parameter<bool>("align_frames", false);
	this->get_parameter("align_frames", m_align_frames);

	this->declare_parameter<std::string>("topic_frameset_left", "/camera_left/frameset");
	this->get_parameter("topic_frameset_left", m_topic_frameset_left);
	this->declare_parameter<std::string>("topic_frameset_right", "/camera_right/frameset");
	this->get_parameter("topic_frameset_right", m_topic_frameset_right);

	this->declare_parameter<bool>("profiling.enable_profiling", false);
	this->get_parameter("profiling.enable_profiling", m_enable_profiling);
	this->declare_parameter<int>("profiling.log_size", 400);
	this->get_parameter("profiling.log_size", m_profiling_size);
	this->declare_parameter<std::string>("profiling.filename", m_node_name + "_profiling.txt");
	this->get_parameter("profiling.filename", m_profiling_filename);

	this->declare_parameter<bool>("vertical_image", true);
	this->get_parameter("vertical_image", m_vertical_image);
	this->declare_parameter<bool>("mirror_image", true);
	this->get_parameter("mirror_image", m_mirror_image);

	this->declare_parameter<bool>("save_pointclouds", false);
	this->get_parameter("save_pointclouds", m_save_pointclouds);

	this->declare_parameter<bool>("use_median_filter", false);
	this->get_parameter("use_median_filter", m_use_median_filter);

	this->declare_parameter<bool>("set_camera_pose", false);
	this->get_parameter("set_camera_pose", m_set_camera_pose);
	this->declare_parameter("camera_translation", std::vector<double>({ 0., 0., 0. }));
	this->get_parameter("camera_translation", m_camera_translation);
	this->declare_parameter("camera_rotation", std::vector<double>({ 0., 0., 0. }));
	this->get_parameter("camera_rotation", m_camera_rotation);

	if (m_verbose)
	{
		std::cout << "+==========[ Fusion Node ]==========+" << std::endl;
		std::cout << "Quality of service:" << std::endl;
		std::cout << " sensor data: " << m_qos_sensor_data << std::endl;
		std::cout << " history depth: " << m_qos_history_depth << std::endl;
	}
}

/**
 * @brief Destructor.
 */
FusionNode::~FusionNode() {}

/**
 * @brief Initialize ROS node.
 */
void FusionNode::init()
{
	// Camera parameters service
	m_cam_left_param_client  = this->create_client<camera_interfaces::srv::GetCameraParameters>("/camera_left/get_camera_parameters");
	m_cam_right_param_client = this->create_client<camera_interfaces::srv::GetCameraParameters>("/camera_right/get_camera_parameters");

	// Wait for left camera parameter service
	while (!m_cam_left_param_client->wait_for_service(1s))
	{
		if (!rclcpp::ok())
		{
			std::cout << "Interrupted while waiting for the left camera. Exiting." << std::endl;
			m_pExit_request->store(true);
			return;
		}
		std::cout << "Waiting for left camera..." << std::endl;
	}
	// Fetch camera parameters from service
	auto request = std::make_shared<camera_interfaces::srv::GetCameraParameters::Request>();
	auto result  = m_cam_left_param_client->async_send_request(request);
	if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == FutureReturnCode::SUCCESS)
	{
		m_camerainfo_depth_left = result.get()->depth_intrinsics;
		m_camerainfo_color_left = result.get()->color_intrinsics;
		for (unsigned long i = 0; i < 3; i++) m_extrinsics_left.translation[i] = result.get()->extrinsic_translation[i];
		for (unsigned long i = 0; i < 9; i++) m_extrinsics_left.rotation[i] = result.get()->extrinsic_rotation[i];
	}
	else
	{
		std::cout << "Failed to call service" << std::endl;
		m_pExit_request->store(true);
		return;
	}

	// Wait for right camera parameter service
	while (!m_cam_right_param_client->wait_for_service(1s))
	{
		if (!rclcpp::ok())
		{
			std::cout << "Interrupted while waiting for the right camera. Exiting." << std::endl;
			m_pExit_request->store(true);
			return;
		}
		std::cout << "Waiting for right camera..." << std::endl;
	}
	// Fetch camera parameters from service
	result = m_cam_right_param_client->async_send_request(request);
	if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == FutureReturnCode::SUCCESS)
	{
		m_camerainfo_depth_right = result.get()->depth_intrinsics;
		m_camerainfo_color_right = result.get()->color_intrinsics;
		for (unsigned long i = 0; i < 3; i++) m_extrinsics_right.translation[i] = result.get()->extrinsic_translation[i];
		for (unsigned long i = 0; i < 9; i++) m_extrinsics_right.rotation[i] = result.get()->extrinsic_rotation[i];
	}
	else
	{
		std::cout << "Failed to call service" << std::endl;
		m_pExit_request->store(true);
		return;
	}

	// Camera intrinsics
	cameraInfo2Intrinsics(m_camerainfo_depth_left, m_intrinsics_depth_left);
	cameraInfo2Intrinsics(m_camerainfo_color_left, m_intrinsics_color_left);
	cameraInfo2Intrinsics(m_camerainfo_depth_right, m_intrinsics_depth_right);
	cameraInfo2Intrinsics(m_camerainfo_color_right, m_intrinsics_color_right);
	m_intrinsics_fused_image = m_intrinsics_color_left;
	if (m_vertical_image)
	{
		m_intrinsics_fused_image.width  = m_intrinsics_color_left.height;
		m_intrinsics_fused_image.height = m_intrinsics_color_left.width;
		m_intrinsics_fused_image.fx     = m_intrinsics_color_left.fy;
		m_intrinsics_fused_image.fy     = m_intrinsics_color_left.fx;
	}
	// D435: focal length 1.93 mm 920 pixel, D455: focal length 1.88 mm 631 pixel
	m_intrinsics_fused_image.ppx = m_intrinsics_fused_image.width / 2;
	m_intrinsics_fused_image.ppy = m_intrinsics_fused_image.height / 2;

	// Virtual camera pose
	if (m_set_camera_pose)
	{
		m_camera_transform = Eigen::Affine3d::Identity();
		m_camera_transform.prerotate(Eigen::AngleAxisd(deg2rad(90.), Eigen::Vector3d::UnitZ()));
		// Axis: X: right, Y: down, Z: forward
		Eigen::Vector3d translation(m_camera_translation[0], m_camera_translation[1], m_camera_translation[2]);
		Eigen::Matrix3d rotation;
		rotation = Eigen::AngleAxisd(deg2rad(m_camera_rotation[0]), Eigen::Vector3d::UnitX()) *
				   Eigen::AngleAxisd(deg2rad(m_camera_rotation[1]), Eigen::Vector3d::UnitY()) *
				   Eigen::AngleAxisd(deg2rad(m_camera_rotation[2]), Eigen::Vector3d::UnitZ());
		m_camera_transform.pretranslate(-translation);
		m_camera_transform.prerotate(rotation.inverse());
	}

	// Quality of service
	if (m_qos_sensor_data) m_qos_profile = rclcpp::SensorDataQoS();
	// m_qos_profile = m_qos_profile.keep_last(static_cast<size_t>(m_qos_history_depth));

	m_qos_profile = m_qos_profile.keep_last(2);
	m_qos_profile = m_qos_profile.lifespan(std::chrono::milliseconds(500));
	m_qos_profile = m_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	m_qos_profile = m_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

	// Fused color image publisher
	m_fused_publisher = image_transport::create_publisher(this, "/fused_image", m_qos_profile.get_rmw_qos_profile());

	// Initialize framesets
	if (!sync_debug)
	{
		initSyncFramesets();
	}

	// Transformation subscriber
	CallbackGroup::SharedPtr callback_group_transform = this->create_callback_group(CallbackGroupType::MutuallyExclusive);
	rclcpp::SubscriptionOptions transform_subscription_options;
	transform_subscription_options.callback_group = callback_group_transform;
	m_transform_subscription                      = this->create_subscription<geometry_msgs::msg::TransformStamped>(
        "/registration/transform", rclcpp::SystemDefaultsQoS(),
        std::bind(&FusionNode::transformCallback, this, std::placeholders::_1), transform_subscription_options);

	// Init cuda
	Kernels::init_cuda(false);
	cudaStreamCreateWithFlags(&m_cuda_stream_left, cudaStreamNonBlocking);
	cudaStreamCreateWithFlags(&m_cuda_stream_right, cudaStreamNonBlocking);

	// Allocate frames
	allocateFrames();

	if (m_verbose)
	{
		std::cout << "qos:" << std::endl;
		std::cout << " sensor data: "
				  << (m_qos_profile.get_rmw_qos_profile().reliability == rmw_qos_profile_sensor_data.reliability)
				  << std::endl;
		std::cout << " history depth: " << m_qos_profile.get_rmw_qos_profile().depth << std::endl;
	}

	if (sync_debug)
	{
		// Message filter sync
		subscriber_frameset_left.subscribe(this, m_topic_frameset_left, m_qos_profile.get_rmw_qos_profile());
		subscriber_frameset_right.subscribe(this, m_topic_frameset_right, m_qos_profile.get_rmw_qos_profile());
		FramesetSyncPolicy frameset_sync_policy(4);
		frameset_sync_policy.setAgePenalty(20.);
		frameset_sync_policy.setMaxIntervalDuration(rclcpp::Duration(0, 17e6)); // rclcpp::Duration(seconds, nanoseconds)
		frameset_sync = new FramesetSync(static_cast<const FramesetSyncPolicy &>(frameset_sync_policy), subscriber_frameset_left, subscriber_frameset_right);
		frameset_sync->registerCallback(&FusionNode::framesetSyncCallback, this);
		m_sync_debug_publisher = this->create_publisher<std_msgs::msg::String>("/fusion_sync_debug", m_qos_profile);
	}
}

/**
 * @brief Initialize frameset synchronization.
 */
void FusionNode::initSyncFramesets()
{
	rclcpp::SubscriptionOptions subscription_options_left;
	m_callback_group_left                    = this->create_callback_group(CallbackGroupType::MutuallyExclusive);
	subscription_options_left.callback_group = m_callback_group_left;

	rclcpp::SubscriptionOptions subscription_options_right;
	m_callback_group_right                    = this->create_callback_group(CallbackGroupType::Reentrant);
	subscription_options_right.callback_group = m_callback_group_right;

	if (!m_direct_callback)
		m_frameset_left_subscription = this->create_subscription<camera_interfaces::msg::DepthFrameset>(m_topic_frameset_left, m_qos_profile, std::bind(&FusionNode::framesetLeftCallback, this, std::placeholders::_1), subscription_options_left);

	m_frameset_right_subscription = this->create_subscription<camera_interfaces::msg::DepthFrameset>(m_topic_frameset_right, m_qos_profile, std::bind(&FusionNode::framesetRightCallback, this, std::placeholders::_1), subscription_options_right);

	m_callback_group_timer = this->create_callback_group(CallbackGroupType::Reentrant);
	m_sync_timer           = this->create_wall_timer(std::chrono::nanoseconds(static_cast<int>(1e9 / 30)), std::bind(&FusionNode::syncFramesetsTimerCallback, this), m_callback_group_timer);

	m_frameset_left_counter.store(0);
	m_frameset_right_counter.store(0);

	if (m_verbose)
		std::cout << "sync framesets from topics: " << m_topic_frameset_left << ", " << m_topic_frameset_right << std::endl;

	std::cout << "+==========[ Fusion Node Started ]==========+" << std::endl;
}

/**
 * @brief Callback for left frameset to synchronize.
 * @param msg Left frameset message
 */
void FusionNode::framesetLeftCallback(camera_interfaces::msg::DepthFrameset::UniquePtr msg)
{
	if (!m_caminfo_left_received)
	{
		m_camerainfo_depth_left = msg->depth_info;
		m_camerainfo_color_left = msg->color_info;
		m_caminfo_left_received = true;
	}

	std::lock_guard<std::mutex> lock(m_mutex_frameset_left);
	if (m_frameset_left_msg_queue.size() > 100)
		m_frameset_left_msg_queue.pop();

	m_frameset_left_msg_queue.push(std::move(msg));

	m_frameset_left_counter++;
}

/**
 * @brief Callback for right frameset to synchronize.
 * @param msg Right frameset message
 */
void FusionNode::framesetRightCallback(camera_interfaces::msg::DepthFrameset::UniquePtr msg)
{
	if (!m_caminfo_right_received)
	{
		m_camerainfo_depth_right = msg->depth_info;
		m_camerainfo_color_right = msg->color_info;
		m_caminfo_right_received = true;
	}

	std::lock_guard<std::mutex> lock(m_mutex_frameset_right);
	if (m_frameset_right_msg_queue.size() > 100)
		m_frameset_right_msg_queue.pop();

	m_frameset_right_msg_queue.push(std::move(msg));

	m_frameset_right_counter++;
}

/**
 * @brief ROS timer callback for frameset synchonization.
 */
void FusionNode::syncFramesetsTimerCallback()
{
	time_point processing_time = system_clock::now();

	bool has_framesets = false;
	m_sync_callback_counter++;

	double frame_rate      = 30.;
	long frame_duration_ns = static_cast<long>(1e9 * 1 / frame_rate);
	long max_stamp_diff    = frame_duration_ns / 2;

	camera_interfaces::msg::DepthFrameset::UniquePtr frameset_left_msg;
	camera_interfaces::msg::DepthFrameset::UniquePtr frameset_right_msg;
	rclcpp::Time ros_timestamp_left  = this->now();
	rclcpp::Time ros_timestamp_right = ros_timestamp_left;
	long stamp_diff                  = std::numeric_limits<long>::max();

	if (!m_frameset_right_msg_queue.empty() && !m_frameset_left_msg_queue.empty())
	{
		if (m_frameset_right_msg_queue.size() <= m_frameset_left_msg_queue.size())
		{
			{
				std::unique_lock<std::mutex> lock_right(m_mutex_frameset_right);
				frameset_right_msg = std::move(m_frameset_right_msg_queue.front());
				m_frameset_right_msg_queue.pop();
				ros_timestamp_right = frameset_right_msg->header.stamp;
			}

			{
				std::unique_lock<std::mutex> lock_left(m_mutex_frameset_left);
				ros_timestamp_left = m_frameset_left_msg_queue.front()->header.stamp;
				stamp_diff         = (ros_timestamp_left - ros_timestamp_right).nanoseconds();
				while (std::abs(stamp_diff) > max_stamp_diff && m_frameset_left_msg_queue.size() > 1)
				{
					m_sync_drop_left_counter++;
					m_frameset_left_msg_queue.pop();
					ros_timestamp_left = m_frameset_left_msg_queue.front()->header.stamp;
					stamp_diff         = (ros_timestamp_left - ros_timestamp_right).nanoseconds();
				}

				if (std::abs(stamp_diff) <= max_stamp_diff)
				{
					frameset_left_msg = std::move(m_frameset_left_msg_queue.front());
					has_framesets     = true;
				}

				m_frameset_left_msg_queue.pop();
			}
		}
		else
		{
			{
				std::unique_lock<std::mutex> lock_left(m_mutex_frameset_left);
				frameset_left_msg = std::move(m_frameset_left_msg_queue.front());
				m_frameset_left_msg_queue.pop();
				ros_timestamp_left = frameset_left_msg->header.stamp;
			}

			{
				std::unique_lock<std::mutex> lock_right(m_mutex_frameset_right);
				ros_timestamp_right = m_frameset_right_msg_queue.front()->header.stamp;
				stamp_diff          = (ros_timestamp_left - ros_timestamp_right).nanoseconds();
				while (std::abs(stamp_diff) > max_stamp_diff && m_frameset_right_msg_queue.size() > 1)
				{
					m_sync_drop_right_counter++;
					m_frameset_right_msg_queue.pop();
					ros_timestamp_right = m_frameset_right_msg_queue.front()->header.stamp;
					stamp_diff          = (ros_timestamp_left - ros_timestamp_right).nanoseconds();
				}

				if (std::abs(stamp_diff) <= max_stamp_diff)
				{
					frameset_right_msg = std::move(m_frameset_right_msg_queue.front());
					has_framesets      = true;
				}

				m_frameset_right_msg_queue.pop();
			}
		}

		if (has_framesets)
		{
			rclcpp::Duration ros_latency = this->now() - frameset_right_msg->header.stamp;
			double latency               = ros_latency.to_chrono<std::chrono::nanoseconds>().count() / 1e6;

			m_sync_latency_acc += latency;
			m_sync_hit_counter++;
			m_sync_hit_counter_acc++;
		}
		else
			m_sync_missed_counter++;
	}
	else
		m_sync_missed_counter++;

	m_sync_loop_counter++;
	if (m_debug && m_sync_loop_counter % 100 == 0)
	{
		double drop_left_rate  = static_cast<double>(m_sync_drop_left_counter) / m_frameset_left_counter.load();
		double drop_right_rate = static_cast<double>(m_sync_drop_right_counter) / m_frameset_right_counter.load();
		std::cout << "+--------------------" << std::endl;
		std::cout << "sync_missed_counter: " << m_sync_missed_counter << std::endl;
		std::cout << "frameset_left_counter: " << m_frameset_left_counter.load() << std::endl;
		std::cout << "frameset_right_counter: " << m_frameset_right_counter.load() << std::endl;
		std::cout << "sync_drop_left_counter: " << m_sync_drop_left_counter << std::endl;
		std::cout << "sync_drop_right_counter: " << m_sync_drop_right_counter << std::endl;
		std::cout << "left drop rate: " << drop_left_rate << std::endl;
		std::cout << "right drop rate: " << drop_right_rate << std::endl;
		std::cout << "queue_size_left = " << m_frameset_left_msg_queue.size() << std::endl;
		std::cout << "queue_size_right = " << m_frameset_right_msg_queue.size() << std::endl;

		std::cout << "sync_loop_counter: " << m_sync_loop_counter << std::endl;
		std::cout << "sync_hit_counter: " << m_sync_hit_counter << std::endl;
		std::cout << "sync_missed_counter: " << m_sync_missed_counter << std::endl;
		double hit_rate = double(m_sync_hit_counter) / m_sync_loop_counter;
		std::cout << "hit rate: " << hit_rate << std::endl;
		double callback_fps = m_sync_callback_counter / m_sync_callback_duration_acc;
		std::cout << "callback FPS: " << callback_fps << std::endl;
		double sync_fps = m_sync_hit_counter_acc / m_sync_callback_duration_acc;
		std::cout << "sync FPS: " << sync_fps << std::endl
				  << std::endl;
		double sync_latency = m_sync_latency_acc / m_sync_hit_counter;
		std::cout << "sync latency: " << sync_latency << std::endl
				  << std::endl;

		m_sync_loop_counter       = 0;
		m_sync_hit_counter        = 0;
		m_sync_missed_counter     = 0;
		m_sync_drop_left_counter  = 0;
		m_sync_drop_right_counter = 0;
		m_frameset_left_counter.store(0);
		m_frameset_right_counter.store(0);
		m_sync_latency_acc = 0;
	}

	if (has_framesets)
		processFrames(std::move(frameset_left_msg), std::move(frameset_right_msg));

	double callback_duration = (system_clock::now() - m_sync_callback_time).count() / 1e6;
	m_sync_callback_duration_acc += callback_duration / 1e3;

	if (m_debug && m_sync_loop_counter == 0)
	{
		double processing_duration = (system_clock::now() - processing_time).count() / 1e6;
		std::cout << "processing_duration: " << processing_duration << " ms" << std::endl;
	}

	m_sync_callback_time = system_clock::now();
}

/**
 * @brief Process frames from synchonized framesets.
 * @param frameset_msg_left Left frameset message
 * @param frameset_msg_right Right frameset message
 */
void FusionNode::processFrames(camera_interfaces::msg::DepthFrameset::UniquePtr frameset_msg_left,
							   camera_interfaces::msg::DepthFrameset::UniquePtr frameset_msg_right)
{
	if (m_pExit_request->load())
	{
		m_sync_timer->cancel();
		rclcpp::shutdown();
	}

	time_point callback_start = system_clock::now();
	time_point time_start     = system_clock::now();

	m_pDepth_frame_left  = reinterpret_cast<uint16_t *>(frameset_msg_left.get()->depth_image.data.data());
	m_pColor_frame_left  = reinterpret_cast<uint8_t *>(frameset_msg_left.get()->color_image.data.data());
	m_pDepth_frame_right = reinterpret_cast<uint16_t *>(frameset_msg_right.get()->depth_image.data.data());
	m_pColor_frame_right = reinterpret_cast<uint8_t *>(frameset_msg_right.get()->color_image.data.data());

	// Left frame
	m_frameset_left.setColorFrame(m_pColor_frame_left);
	if (m_align_frames)
	{
		m_frameset_left.setDepthFrameUnaligned(m_pDepth_frame_left);
		m_frameset_left.alignDepthToColor(m_depth_scale_left);
	}
	else
		m_frameset_left.setDepthFrame(m_pDepth_frame_left);

	// Right frame
	m_frameset_right.setColorFrame(m_pColor_frame_right);
	if (m_align_frames)
	{
		m_frameset_left.setDepthFrameUnaligned(m_pDepth_frame_left);
		m_frameset_left.alignDepthToColor(m_depth_scale_left);
	}
	else
		m_frameset_right.setDepthFrame(m_pDepth_frame_right);

	double copy_to_gpu_duration = getTiming(time_start);

	const std::array<int, 4> roi_left = { 0, 0, m_intrinsics_depth_left.width, m_intrinsics_depth_left.height };
	m_frameset_left.filterDepth(m_min_depth, m_max_depth, m_depth_scale_left, roi_left);
	m_frameset_right.filterDepth(m_min_depth, m_max_depth, m_depth_scale_right);

	double filter_frames_duration = getTiming(time_start);

	// Deproject to point clouds
	Pointcloud fused_cloud;
	fused_cloud.setStream(m_frameset_left.getStream());
	fused_cloud.allocate(m_frameset_left.getMaskCount() + m_frameset_right.getMaskCount());
	fused_cloud.deproject(m_frameset_left, m_depth_scale_left);
	Pointcloud cloud_right;
	cloud_right.setStream(m_frameset_right.getStream());
	cloud_right.deproject(m_frameset_right, m_depth_scale_right);

	double deproject_duration = getTiming(time_start);

	// Transform point clouds
	cloud_right.transform(m_right_transform);
	cudaStreamSynchronize(*m_frameset_right.getStream());

	double transform_right_duration = getTiming(time_start);

	// Fuse point clouds
	fused_cloud.append(cloud_right);

	double fuse_duration = getTiming(time_start);

	// Transform fused pointcloud
	Eigen::Affine3d fused_transform = Eigen::Affine3d::Identity();
	if (!m_set_camera_pose)
		interpolateTransform(fused_transform, m_left_transform, m_right_transform);
	else
		fused_transform = m_camera_transform;

	if (m_vertical_image)
	{
		fused_transform.prerotate(Eigen::AngleAxisd(deg2rad(90.), Eigen::Vector3d::UnitZ()));
	}

	fused_cloud.transform(fused_transform);

	double transform_fused_duration = getTiming(time_start);

	// Project fused pointcloud
	fused_cloud.project(m_fused_frameset, m_mirror_image);

	double project_duration = getTiming(time_start);

	// Filter fused image
	m_fused_frameset.filterColor(m_use_median_filter);

	double filter_fused_duration = getTiming(time_start);

	// Copy image from gpu to host memory
	m_fused_frameset.copyColorToHost(m_pFused_frame);

	double copy_from_gpu_duration = getTiming(time_start);

	// Save point clouds to ply files for debugging
	if (m_save_pointclouds)
	{
		Pointcloud cloud_left;
		cloud_left.setStream(m_frameset_left.getStream());
		cloud_left.deproject(m_frameset_left, m_depth_scale_left);
		save_pointclouds_ply(cloud_left, cloud_right, fused_transform);
	}

	// Publish fused image
	const uint8_t *fused_msg_bytes = reinterpret_cast<const uint8_t *>(m_pFused_frame);
	m_fused_msg.header.frame_id    = "camera_left_color_optical_frame";
	m_fused_msg.header.stamp       = frameset_msg_left->header.stamp;
	m_fused_msg.width              = static_cast<uint>(m_intrinsics_fused_image.width);
	m_fused_msg.height             = static_cast<uint>(m_intrinsics_fused_image.height);
	m_fused_msg.is_bigendian       = false;
	m_fused_msg.step               = m_fused_msg.width * 3 * sizeof(uint8_t);
	m_fused_msg.encoding           = "rgb8";
	m_fused_msg.data.assign(fused_msg_bytes, fused_msg_bytes + (m_fused_msg.step * m_fused_msg.height));
	m_fused_publisher.publish(m_fused_msg);

	double publish_duration = getTiming(time_start);

	// Profiling
	if (m_enable_profiling)
	{
		double loop_duration             = (system_clock::now() - m_global_timer).count() / 1e6;
		m_global_timer                   = system_clock::now();
		double callback_duration         = (system_clock::now() - callback_start).count() / 1e6;
		rclcpp::Time ros_timestamp_left  = frameset_msg_left->header.stamp;
		rclcpp::Time ros_timestamp_right = frameset_msg_right->header.stamp;
		double stamp_diff                = (ros_timestamp_left - ros_timestamp_right).nanoseconds() / 1e6;
		rclcpp::Duration ros_latency     = this->now() - frameset_msg_left->header.stamp;
		double latency                   = ros_latency.to_chrono<std::chrono::nanoseconds>().count() / 1e6;

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
		m_profiling_vec.push_back(vec);

		if (m_profiling_vec.size() == static_cast<unsigned>(m_profiling_size))
		{
			std::cout << "Write profiling file " << m_profiling_filename << std::endl;
			std::ofstream profiling_file(m_profiling_filename);
			for (unsigned i = 0; i < m_profiling_fields.size(); i++)
			{
				profiling_file << m_profiling_fields[i];
				if (i != m_profiling_fields.size() - 1) profiling_file << ",";
			}
			profiling_file << "\n";
			for (const auto &l : m_profiling_vec)
			{
				for (unsigned i = 0; i < l.size(); i++)
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
 * @brief Allocate memory for images from framesets.
 */
void FusionNode::allocateFrames()
{
	// Allocate frames
	m_pDepth_frame_left  = reinterpret_cast<uint16_t *>(malloc(static_cast<unsigned>(m_intrinsics_depth_left.width * m_intrinsics_depth_left.height) * sizeof(uint16_t)));
	m_pColor_frame_left  = reinterpret_cast<uint8_t *>(malloc(static_cast<unsigned>(m_intrinsics_color_left.width * m_intrinsics_color_left.height) * 3 * sizeof(uint8_t)));
	m_pDepth_frame_right = reinterpret_cast<uint16_t *>(malloc(static_cast<unsigned>(m_intrinsics_depth_right.width * m_intrinsics_depth_right.height) * sizeof(uint16_t)));
	m_pColor_frame_right = reinterpret_cast<uint8_t *>(malloc(static_cast<unsigned>(m_intrinsics_color_right.width * m_intrinsics_color_right.height) * 3 * sizeof(uint8_t)));

	m_frameset_left.setStream(&m_cuda_stream_left);
	m_frameset_right.setStream(&m_cuda_stream_right);

	m_frameset_left.allocateDepthFrame(m_intrinsics_color_left); // aligned to color
	m_frameset_left.allocateColorFrame(m_intrinsics_color_left);
	m_frameset_left.allocateDepthFrameUnaligned(m_intrinsics_depth_left, m_extrinsics_left);
	m_frameset_right.allocateDepthFrame(m_intrinsics_color_right); // aligned to color
	m_frameset_right.allocateColorFrame(m_intrinsics_color_right);
	m_frameset_right.allocateDepthFrameUnaligned(m_intrinsics_depth_right, m_extrinsics_right);

	m_fused_frameset.setStream(&m_cuda_stream_left);
	m_fused_frameset.allocateColorFrame(m_intrinsics_fused_image);
	m_pFused_frame = reinterpret_cast<uint8_t *>(malloc(static_cast<unsigned>(m_intrinsics_fused_image.width * m_intrinsics_fused_image.height) * 3 * sizeof(uint8_t)));

	// Set save images
	std::string file_prefix = m_node_name;
	file_prefix.erase(std::remove(file_prefix.begin(), file_prefix.end(), '/'), file_prefix.end());
	m_frameset_left.setSaveImages(m_save_data, m_package_share_directory + "/data", file_prefix + "_left");
	m_frameset_right.setSaveImages(m_save_data, m_package_share_directory + "/data", file_prefix + "_right");
	m_fused_frameset.setSaveImages(m_save_data, m_package_share_directory + "/data", "fused");
}

/**
 * @brief Convert camera intrinsics message to intrinsics structure.
 * @param camerainfo Camera intrinsics message
 * @param intrinsics Intrinsics structure
 */
void FusionNode::cameraInfo2Intrinsics(const sensor_msgs::msg::CameraInfo &camerainfo, Intrinsics &intrinsics)
{
	intrinsics.width  = static_cast<int>(camerainfo.width);
	intrinsics.height = static_cast<int>(camerainfo.height);
	intrinsics.ppx    = static_cast<int>(camerainfo.k[2]);
	intrinsics.ppy    = static_cast<int>(camerainfo.k[5]);
	intrinsics.fx     = static_cast<int>(camerainfo.k[0]);
	intrinsics.fy     = static_cast<int>(camerainfo.k[4]);
	intrinsics.model  = Distortion::DISTORTION_BROWN_CONRADY;
	for (uint i = 0; i < 5; i++)
		intrinsics.coeffs[i] = static_cast<float>(camerainfo.d[i]);
}

/**
 * @brief Interpolate between affine transformations.
 * @param interpolated_transform Interpolated mean affine transformation
 * @param left_transform First affine transformation
 * @param right_transform Second affine transformation
 */
void FusionNode::interpolateTransform(Eigen::Affine3d &interpolated_transform, const Eigen::Affine3d &left_transform, const Eigen::Affine3d &right_transform)
{
	interpolated_transform = Eigen::Affine3d::Identity();

	Eigen::Quaterniond left_rotation(left_transform.rotation());
	Eigen::Quaterniond right_rotation(right_transform.rotation());
	// spherical linear interpolation between the two quaternions
	Eigen::Quaterniond fused_rotation = left_rotation.slerp(0.5, right_rotation);
	Eigen::Vector3d left_translation  = left_transform.translation();
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
void FusionNode::transformCallback(const geometry_msgs::msg::TransformStamped::SharedPtr transform_msg)
{
	m_right_transform = tf2::transformToEigen(transform_msg->transform);
}

/**
 * @brief Get timing in milliseconds from start time until now and set start time to now.
 * @param start_time Start time point
 * @return Time elapsed since start in milliseconds
 */
double FusionNode::getTiming(time_point &start_time)
{
	if (m_enable_profiling)
	{
		cudaDeviceSynchronize();
		double timing = (system_clock::now() - start_time).count() / 1e6;
		start_time    = system_clock::now();
		return timing;
	}
	else
		return 0.;
}

/**
 * @brief Save colored point clouds to ply files.
 * @param cloud_left Point cloud for left camera
 * @param cloud_right Point cloud for right camera
 * @param fused_transform Transformation for fused pointcloud
 */
void FusionNode::save_pointclouds_ply(Pointcloud &cloud_left, Pointcloud &cloud_right, Eigen::Affine3d &fused_transform)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_left(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl_cloud_left->resize(m_frameset_left.getMaskCount());
	float *pcl_cloud_left_data = pcl_cloud_left->points.data()->_PointXYZRGB::data;
	cloud_left.copyToHost(pcl_cloud_left_data);

	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator points_it = pcl_cloud_left->begin(); points_it != pcl_cloud_left->end(); ++points_it)
	{
		float *rgb_float   = &points_it->_PointXYZRGB::data[3];
		uint8_t *rgb_uint8 = reinterpret_cast<uint8_t *>(rgb_float);
		points_it->b       = rgb_uint8[0];
		points_it->g       = rgb_uint8[1];
		points_it->r       = rgb_uint8[2];
	}

	pcl::io::savePLYFileASCII(m_package_share_directory + "/data/cloud_left_input.ply", *pcl_cloud_left);

	cloud_left.transform(fused_transform);
	cloud_left.copyToHost(pcl_cloud_left_data);

	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator points_it = pcl_cloud_left->begin(); points_it != pcl_cloud_left->end(); ++points_it)
	{
		float *rgb_float   = &points_it->_PointXYZRGB::data[3];
		uint8_t *rgb_uint8 = reinterpret_cast<uint8_t *>(rgb_float);
		points_it->b       = rgb_uint8[0];
		points_it->g       = rgb_uint8[1];
		points_it->r       = rgb_uint8[2];
	}

	pcl::io::savePLYFileASCII(m_package_share_directory + "/data/cloud_left_fused.ply", *pcl_cloud_left);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_right(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl_cloud_right->resize(m_frameset_right.getMaskCount());
	float *pcl_cloud_right_data = pcl_cloud_right->points.data()->_PointXYZRGB::data;
	cloud_right.copyToHost(pcl_cloud_right_data);

	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator points_it = pcl_cloud_right->begin(); points_it != pcl_cloud_right->end(); ++points_it)
	{
		float *rgb_float   = &points_it->_PointXYZRGB::data[3];
		uint8_t *rgb_uint8 = reinterpret_cast<uint8_t *>(rgb_float);
		points_it->b       = rgb_uint8[0];
		points_it->g       = rgb_uint8[1];
		points_it->r       = rgb_uint8[2];
	}

	pcl::io::savePLYFileASCII(m_package_share_directory + "/data/cloud_right_input.ply", *pcl_cloud_right);

	cloud_right.transform(fused_transform);
	cloud_right.copyToHost(pcl_cloud_right_data);
	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator points_it = pcl_cloud_right->begin(); points_it != pcl_cloud_right->end(); ++points_it)
	{
		float *rgb_float   = &points_it->_PointXYZRGB::data[3];
		uint8_t *rgb_uint8 = reinterpret_cast<uint8_t *>(rgb_float);
		points_it->b       = rgb_uint8[0];
		points_it->g       = rgb_uint8[1];
		points_it->r       = rgb_uint8[2];
	}

	pcl::io::savePLYFileASCII(m_package_share_directory + "/data/cloud_right_fused.ply", *pcl_cloud_right);

	std::cout << "## Saved ply clouds" << std::endl;
	m_ply_counter++;
	if (m_ply_counter == 3)
	{
		m_pExit_request->store(true);
		m_sync_timer->cancel();
		rclcpp::shutdown();
	}
}

void FusionNode::framesetSyncCallback(const camera_interfaces::msg::DepthFrameset::ConstSharedPtr& frameset_msg_left,
																			const camera_interfaces::msg::DepthFrameset::ConstSharedPtr& frameset_msg_right)
{
	double sync_duration_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - m_sync_start_time).count() / 1e6;
	m_sync_start_time = Clock::now();

	rclcpp::Time stamp_left = frameset_msg_left->header.stamp;
	rclcpp::Time stamp_right = frameset_msg_right->header.stamp;
	double stamp_left_ms = stamp_left.nanoseconds() / 1e6;
	long stamp_diff = (stamp_left - stamp_right).nanoseconds();
	double stamp_diff_ms = stamp_diff / 1e6;

	// Process frames
	// processSyncedFrames(frameset_msg_left, frameset_msg_right);
	auto future_publishdepth = std::async(&FusionNode::processSyncedFrames, this, frameset_msg_left, frameset_msg_right);

	double latency_ms = (std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count() / 1e6) - stamp_left_ms;
	std::ostringstream strstr("");
	strstr << std::fixed << std::setprecision(2);
	strstr << "sync callback: " << sync_duration_ms << " ms, " << 1000 / sync_duration_ms << " fps, diff: " << stamp_diff_ms << ", latency: " << latency_ms << " ms";
	std::cout << strstr.str() << std::endl;
	auto message = std_msgs::msg::String();
	message.data = strstr.str();
	m_sync_debug_publisher->publish(message);

}

void FusionNode::processSyncedFrames(const camera_interfaces::msg::DepthFrameset::ConstSharedPtr& frameset_msg_left,
							   const camera_interfaces::msg::DepthFrameset::ConstSharedPtr& frameset_msg_right)
{
	if (m_pExit_request->load())
	{
		m_sync_timer->cancel();
		rclcpp::shutdown();
	}

	time_point callback_start = system_clock::now();
	time_point time_start     = system_clock::now();

	m_pDepth_frame_left  = reinterpret_cast<uint16_t *>(const_cast<uint8_t *>(frameset_msg_left.get()->depth_image.data.data()));
	m_pColor_frame_left  = const_cast<uint8_t *>(frameset_msg_left.get()->color_image.data.data());
	m_pDepth_frame_right = reinterpret_cast<uint16_t *>(const_cast<uint8_t *>(frameset_msg_right.get()->depth_image.data.data()));
	m_pColor_frame_right = const_cast<uint8_t *>(frameset_msg_right.get()->color_image.data.data());

	// Left frame
	m_frameset_left.setColorFrame(m_pColor_frame_left);
	if (m_align_frames)
	{
		m_frameset_left.setDepthFrameUnaligned(m_pDepth_frame_left);
		m_frameset_left.alignDepthToColor(m_depth_scale_left);
	}
	else
		m_frameset_left.setDepthFrame(m_pDepth_frame_left);

	// Right frame
	m_frameset_right.setColorFrame(m_pColor_frame_right);
	if (m_align_frames)
	{
		m_frameset_left.setDepthFrameUnaligned(m_pDepth_frame_left);
		m_frameset_left.alignDepthToColor(m_depth_scale_left);
	}
	else
		m_frameset_right.setDepthFrame(m_pDepth_frame_right);

	double copy_to_gpu_duration = getTiming(time_start);

	const std::array<int, 4> roi_left = { 0, 0, m_intrinsics_depth_left.width, m_intrinsics_depth_left.height };
	m_frameset_left.filterDepth(m_min_depth, m_max_depth, m_depth_scale_left, roi_left);
	m_frameset_right.filterDepth(m_min_depth, m_max_depth, m_depth_scale_right);

	double filter_frames_duration = getTiming(time_start);

	// Deproject to point clouds
	Pointcloud fused_cloud;
	fused_cloud.setStream(m_frameset_left.getStream());
	fused_cloud.allocate(m_frameset_left.getMaskCount() + m_frameset_right.getMaskCount());
	fused_cloud.deproject(m_frameset_left, m_depth_scale_left);
	Pointcloud cloud_right;
	cloud_right.setStream(m_frameset_right.getStream());
	cloud_right.deproject(m_frameset_right, m_depth_scale_right);

	double deproject_duration = getTiming(time_start);

	// Transform point clouds
	cloud_right.transform(m_right_transform);
	cudaStreamSynchronize(*m_frameset_right.getStream());

	double transform_right_duration = getTiming(time_start);

	// Fuse point clouds
	fused_cloud.append(cloud_right);

	double fuse_duration = getTiming(time_start);

	// Transform fused pointcloud
	Eigen::Affine3d fused_transform = Eigen::Affine3d::Identity();
	if (!m_set_camera_pose)
		interpolateTransform(fused_transform, m_left_transform, m_right_transform);
	else
		fused_transform = m_camera_transform;

	if (m_vertical_image)
	{
		fused_transform.prerotate(Eigen::AngleAxisd(deg2rad(90.), Eigen::Vector3d::UnitZ()));
	}

	fused_cloud.transform(fused_transform);

	double transform_fused_duration = getTiming(time_start);

	// Project fused pointcloud
	fused_cloud.project(m_fused_frameset, m_mirror_image);

	double project_duration = getTiming(time_start);

	// Filter fused image
	m_fused_frameset.filterColor(m_use_median_filter);

	double filter_fused_duration = getTiming(time_start);

	// Copy image from gpu to host memory
	m_fused_frameset.copyColorToHost(m_pFused_frame);

	double copy_from_gpu_duration = getTiming(time_start);

	// Save point clouds to ply files for debugging
	if (m_save_pointclouds)
	{
		Pointcloud cloud_left;
		cloud_left.setStream(m_frameset_left.getStream());
		cloud_left.deproject(m_frameset_left, m_depth_scale_left);
		save_pointclouds_ply(cloud_left, cloud_right, fused_transform);
	}

	// Publish fused image
	const uint8_t *fused_msg_bytes = reinterpret_cast<const uint8_t *>(m_pFused_frame);
	m_fused_msg.header.frame_id    = "camera_left_color_optical_frame";
	m_fused_msg.header.stamp       = frameset_msg_left->header.stamp;
	m_fused_msg.width              = static_cast<uint>(m_intrinsics_fused_image.width);
	m_fused_msg.height             = static_cast<uint>(m_intrinsics_fused_image.height);
	m_fused_msg.is_bigendian       = false;
	m_fused_msg.step               = m_fused_msg.width * 3 * sizeof(uint8_t);
	m_fused_msg.encoding           = "rgb8";
	m_fused_msg.data.assign(fused_msg_bytes, fused_msg_bytes + (m_fused_msg.step * m_fused_msg.height));
	m_fused_publisher.publish(m_fused_msg);

	// double publish_duration = getTiming(time_start);
}
