// SYSTEM
#include <experimental/filesystem>
#include <iostream>
// ROS2
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/image.hpp>
// PROJECT
#include "registration_node.hpp"

using namespace std::chrono_literals;

/**
 * @brief Contructor.
 */
RegistrationNode::RegistrationNode() :
	Node("registration_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
	registration = new Registration();

	this->declare_parameter("verbose", false);
	this->declare_parameter("qos_sensor_data", false);
	this->declare_parameter("qos_history_depth", 2);
	this->declare_parameter("resolution", 0.05);
	this->declare_parameter("spin_rate", 1.);
	this->declare_parameter("min_depth", 0.5);
	this->declare_parameter("max_depth", 2.0);
	this->declare_parameter("depth_scale", 0.0001);
	this->declare_parameter("depth_scale_left", -1.0);
	this->declare_parameter("depth_scale_right", -1.0);
	this->declare_parameter("voxelsize", 0.001);
	this->declare_parameter("roi_left", std::vector<long>({ -1, -1, -1, -1 }));
	this->declare_parameter("roi_right", std::vector<long>({ -1, -1, -1, -1 }));

	this->get_parameter("verbose", verbose);
	this->get_parameter("qos_sensor_data", qos_sensor_data);
	this->get_parameter("qos_history_depth", qos_history_depth);
	this->get_parameter("resolution", resolution);
	this->get_parameter("spin_rate", spin_rate);
	this->get_parameter("min_depth", min_depth);
	this->get_parameter("max_depth", max_depth);
	this->get_parameter("depth_scale", depth_scale);
	this->get_parameter("depth_scale_left", depth_scale_left);
	this->get_parameter("depth_scale_right", depth_scale_right);
	this->get_parameter("voxelsize", voxelsize);
	std::vector<long> roi_vec;
	this->get_parameter("roi_left", roi_vec);
	this->roi_left = { static_cast<int>(roi_vec[0]), static_cast<int>(roi_vec[1]), static_cast<int>(roi_vec[2]),
					   static_cast<int>(roi_vec[3]) };
	this->get_parameter("roi_right", roi_vec);
	this->roi_right = { static_cast<int>(roi_vec[0]), static_cast<int>(roi_vec[1]), static_cast<int>(roi_vec[2]),
						static_cast<int>(roi_vec[3]) };

	this->declare_parameter("publish_clouds", false);
	this->get_parameter("publish_clouds", publish_clouds);

	this->declare_parameter<bool>("profiling.enable_profiling", false);
	this->get_parameter("profiling.enable_profiling", enable_profiling);
	this->declare_parameter<int>("profiling.log_size", 40);
	this->get_parameter("profiling.log_size", profiling_size);
	this->declare_parameter<std::string>("profiling.filename", node_name + "_profiling.txt");
	this->get_parameter("profiling.filename", profiling_filename);

	this->declare_parameter("kernel_width", 0.005);
	this->get_parameter("kernel_width", kernel_width);
	this->declare_parameter("kernel_max_dist", 0.025);
	this->get_parameter("kernel_max_dist", kernel_max_dist);
	this->declare_parameter("rotation_epsilon", 2e-3);
	this->get_parameter("rotation_epsilon", rotation_epsilon);
	this->declare_parameter("translation_epsilon", 1e-4);
	this->get_parameter("translation_epsilon", translation_epsilon);
	this->declare_parameter("fitness_epsilon", 1e-12);
	this->get_parameter("fitness_epsilon", fitness_epsilon);
	this->declare_parameter("max_iterations", 1000);
	this->get_parameter("max_iterations", max_iterations);

	this->declare_parameter("cam_upside_down", true);
	this->get_parameter("cam_upside_down", cam_upside_down);

	if (verbose)
	{
		std::cout << "Quality of service:" << std::endl;
		std::cout << " sensor data: " << qos_sensor_data << std::endl;
		std::cout << " history depth: " << qos_history_depth << std::endl;
	}

	registration->setVerbosity(verbose);
	setResolution(resolution);
	registration->setVoxelgridSize(voxelsize);

	if (resolution / 2 < kernel_width || resolution / 2 > kernel_width)
	{
		registration->setKernelParameters(kernel_width, kernel_max_dist);
	}
}

/**
 * @brief Destructor.
 */
RegistrationNode::~RegistrationNode()
{
	// Save last icp transform
	if (save_transform)
	{
		saveTransform(last_transform, transform_filename);
	}

	delete registration;
}

/**
 * @brief Initialize registration node.
 */
void RegistrationNode::init()
{
	// Set directories
	package_share_directory = ament_index_cpp::get_package_share_directory("registration_node");
	transform_filename      = package_share_directory + "/config/transform.txt";

	// Quality of service
	if (qos_sensor_data) qos_profile = rclcpp::SensorDataQoS();
	qos_profile = qos_profile.keep_last(static_cast<size_t>(qos_history_depth));

	rmw_qos_profile_t image_rmw_qos_profile = qos_profile.get_rmw_qos_profile();
	rclcpp::QoS cloud_qos_profile           = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));

	// Subscriptions
	subscriber_depth_left.subscribe(this, topic_depth_left, image_rmw_qos_profile);
	subscriber_depth_right.subscribe(this, topic_depth_right, image_rmw_qos_profile);

	// Synchronization
	depth_sync = new ImageSync(ImageSyncPolicy(10), subscriber_depth_left, subscriber_depth_right);

	// Transform publisher
	std::string topic_transform = "/registration/transform";
	publisher_transform         = this->create_publisher<geometry_msgs::msg::TransformStamped>(topic_transform, 1);

	// Pointcloud publishers
	std::string topic_target_cloud  = "/registration/target_points";
	std::string topic_aligned_cloud = "/registration/aligned_points";
	publisher_target_cloud          = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_target_cloud, cloud_qos_profile);
	publisher_aligned_cloud =
		this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_aligned_cloud, cloud_qos_profile);

	// Init registration
	registration->setMaximumIterations(max_iterations);        // library default: 64
	registration->setRotationEpsilon(rotation_epsilon);        // libaray default: 2e-3
	registration->setTranslationEpsilon(translation_epsilon);  // libaray default: 5e-4
	registration->setEuclideanFitnessEpsilon(fitness_epsilon); // library default: 1e-5

	// Load transformation
	if (load_transform)
	{
		loadTransform(initial_transform, transform_filename);
	}

	// Turn 180 degree around z-axis for D455 mount
	start_transform = Eigen::Affine3d::Identity();
	if (cam_upside_down)
	{
		Eigen::Matrix3d rotation_180_z = Eigen::Matrix3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
		start_transform.rotate(rotation_180_z);
	}
	initial_transform = start_transform;

	// Set initial transformation manually
	if (set_initial_transform)
	{
		Eigen::Vector3d initial_translation(0., 0., 0.);
		Eigen::Vector3d initial_rotation_xyz_deg(0., 0., 0.);
		Eigen::Matrix3d initial_rotation;
		initial_rotation = Eigen::AngleAxisd(initial_rotation_xyz_deg(0) * M_PI / 180, Eigen::Vector3d::UnitX()) *
						   Eigen::AngleAxisd(initial_rotation_xyz_deg(1) * M_PI / 180, Eigen::Vector3d::UnitY()) *
						   Eigen::AngleAxisd(initial_rotation_xyz_deg(2) * M_PI / 180, Eigen::Vector3d::UnitZ());
		initial_transform = Eigen::Affine3d::Identity();
		initial_transform.rotate(initial_rotation);
		initial_transform.translate(initial_translation);
	}

	// Init cuda
	Kernels::init_cuda(verbose);
	cudaStreamCreateWithFlags(&target_cuda_stream, cudaStreamNonBlocking);
	cudaStreamCreateWithFlags(&source_cuda_stream, cudaStreamNonBlocking);

	// Camera parameters service
	m_cam_left_param_client  = this->create_client<camera_interfaces::srv::GetCameraParameters>("/camera_left/get_camera_parameters");
	m_cam_right_param_client = this->create_client<camera_interfaces::srv::GetCameraParameters>("/camera_right/get_camera_parameters");

	// Wait for left camera parameter service
	std::cout << "Fetching camera info services" << std::endl;
	while (!m_cam_left_param_client->wait_for_service(1s))
	{
		if (!rclcpp::ok())
		{
			std::cout << "Interrupted while waiting for the left camera. Exiting." << std::endl;
			exit_request->store(true);
			return;
		}
		std::cout << "Waiting for left camera..." << std::endl;
	}
	// Fetch camera parameters from service
	auto request = std::make_shared<camera_interfaces::srv::GetCameraParameters::Request>();
	auto result  = m_cam_left_param_client->async_send_request(request);
	if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == FutureReturnCode::SUCCESS)
	{
		// m_camerainfo_depth_left = result.get()->depth_intrinsics;
		// m_camerainfo_color_left = result.get()->color_intrinsics;
		// for (unsigned long i = 0; i < 3; i++) m_extrinsics_left.translation[i] = result.get()->extrinsic_translation[i];
		// for (unsigned long i = 0; i < 9; i++) m_extrinsics_left.rotation[i] = result.get()->extrinsic_rotation[i];
		this->camera_info_left  = result.get()->depth_intrinsics;
	}
	else
	{
		std::cout << "Failed to call service" << std::endl;
		exit_request->store(true);
		return;
	}

	// Wait for right camera parameter service
	while (!m_cam_right_param_client->wait_for_service(1s))
	{
		if (!rclcpp::ok())
		{
			std::cout << "Interrupted while waiting for the right camera. Exiting." << std::endl;
			exit_request->store(true);
			return;
		}
		std::cout << "Waiting for right camera..." << std::endl;
	}
	// Fetch camera parameters from service
	result = m_cam_right_param_client->async_send_request(request);
	if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == FutureReturnCode::SUCCESS)
	{
		// m_camerainfo_depth_right = result.get()->depth_intrinsics;
		// m_camerainfo_color_right = result.get()->color_intrinsics;
		// for (unsigned long i = 0; i < 3; i++) m_extrinsics_right.translation[i] = result.get()->extrinsic_translation[i];
		// for (unsigned long i = 0; i < 9; i++) m_extrinsics_right.rotation[i] = result.get()->extrinsic_rotation[i];
		this->camera_info_right = result.get()->depth_intrinsics;
	}
	else
	{
		std::cout << "Failed to call service" << std::endl;
		exit_request->store(true);
		return;
	}
	std::cout << "Camera info services received" << std::endl;

	// Get camera intrinsics
	Intrinsics target_intrinsics;
	Intrinsics source_intrinsics;
	cameraInfo2Intrinsics(camera_info_left, target_intrinsics);
	cameraInfo2Intrinsics(camera_info_right, source_intrinsics);

	// Allocate frames
	target_frameset.setStream(&target_cuda_stream);
	source_frameset.setStream(&source_cuda_stream);
	target_frameset.allocateDepthFrame(target_intrinsics);
	source_frameset.allocateDepthFrame(source_intrinsics);

	// Register depth image callback
	depth_sync->registerCallback(&RegistrationNode::depthSyncCallback, this);

	std::cout << "Registration node initialized" << std::endl;
}

/**
 * @brief Register source depth frame to target depth frame with icp algorithm.
 * @param target_depth_msg Target depth frame message
 * @param source_depth_msg Source depth frame message
 * @param target_camerainfo Target camera intrinsics message
 * @param source_camerainfo Source camera intrinsics message
 * @param depth_scale Factor to convert camera depth unit to meters
 * @param final_transform Affine transformation result
 * @param initial_transform Initial source transformtation guess
 */
void RegistrationNode::icp(const sensor_msgs::msg::Image::ConstSharedPtr& target_depth_msg,
						   const sensor_msgs::msg::Image::ConstSharedPtr& source_depth_msg,
						   sensor_msgs::msg::CameraInfo& target_camerainfo,
						   sensor_msgs::msg::CameraInfo& source_camerainfo, float depth_scale,
						   Eigen::Affine3d& final_transform, const Eigen::Affine3d& initial_transform)
{
	std::cout << "start icp" << std::endl;
	uint target_count = static_cast<uint>(target_camerainfo.width * target_camerainfo.height);
	uint source_count = static_cast<uint>(source_camerainfo.width * source_camerainfo.height);

	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	bool use_testdata = false;
	if (use_testdata)
	{
		std::string target_testdata = package_share_directory + "/data/icp_testdata/bunny_scale1m.ply";
		std::string source_testdata =
			package_share_directory + "/data/icp_testdata/bunny_scale1m_transformed_rot0.26rad15degy_trans0.5x.ply";
		pcl::io::loadPLYFile(target_testdata, *target_cloud);
		pcl::io::loadPLYFile(source_testdata, *source_cloud);
		target_count = static_cast<uint>(target_cloud->points.size());
		source_count = static_cast<uint>(source_cloud->points.size());
	}
	else
	{
		auto timer_start = std::chrono::steady_clock::now();
		if (verbose)
		{
			std::cout << "+-- Poincloud Deprojection" << std::endl;
			std::cout << "| depth pixels:         " << target_depth_msg->width * target_depth_msg->height << std::endl;
			std::cout << "| depth range:          [" << min_depth << ", " << max_depth << "]" << std::endl;
			std::cout << "| depth scale:          " << depth_scale << std::endl;
		}

		float target_depth_scale = depth_scale_left;
		float source_depth_scale = depth_scale_right;
		if (target_depth_scale < 0) target_depth_scale = depth_scale;
		if (source_depth_scale < 0) source_depth_scale = depth_scale;

		// Gpu deprojection
		uint16_t* target_depth_frame = reinterpret_cast<uint16_t*>(const_cast<uint8_t*>(&target_depth_msg->data[0]));
		uint16_t* source_depth_frame = reinterpret_cast<uint16_t*>(const_cast<uint8_t*>(&source_depth_msg->data[0]));
		target_frameset.setDepthFrame(target_depth_frame);
		target_frameset.filterDepth(min_depth, max_depth, target_depth_scale, roi_left);
		source_frameset.setDepthFrame(source_depth_frame);
		source_frameset.filterDepth(min_depth, max_depth, source_depth_scale, roi_right);
		Pointcloud target_dev_cloud;
		Pointcloud source_dev_cloud;
		cudaStreamSynchronize(target_cuda_stream);
		target_dev_cloud.setStream(&target_cuda_stream);
		target_dev_cloud.deproject(target_frameset, target_depth_scale);
		source_dev_cloud.setStream(&target_cuda_stream);
		source_dev_cloud.deproject(source_frameset, source_depth_scale);

		// Copy from gpu to host pcl pointcloud
		target_cloud->resize(target_dev_cloud.getPointCount());
		float* target_cloud_data = target_cloud->points.data()->_PointXYZ::data;
		target_dev_cloud.copyToHost(target_cloud_data);
		source_cloud->resize(source_dev_cloud.getPointCount());
		float* source_cloud_data = source_cloud->points.data()->_PointXYZ::data;
		source_dev_cloud.copyToHost(source_cloud_data);

		auto timer_end     = std::chrono::steady_clock::now();
		auto timer_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(timer_end - timer_start);
		if (verbose)
		{
			std::cout << "| time:        " << timer_elapsed.count() << " ms" << std::endl;
			std::cout << "+--" << std::endl;
		}
	}

	bool save_data = false;
	if (save_data)
	{
		pcl::io::savePLYFileASCII(package_share_directory + "/data/cam_left_icp_cloud.ply", *target_cloud);
		pcl::io::savePLYFileASCII(package_share_directory + "/data/cam_right_icp_cloud.ply", *source_cloud);
		std::cout << "## Saved ply clouds" << std::endl;
	}

	double euclidean_fitness = 0.;

	// Run icp algorithm
	registration->setSource(source_cloud);
	registration->setTarget(target_cloud);
	auto timer_start = std::chrono::steady_clock::now();
	registration->icp(final_transform, euclidean_fitness, initial_transform);
	auto timer_end      = std::chrono::steady_clock::now();
	double icp_duration = std::chrono::duration_cast<std::chrono::milliseconds>(timer_end - timer_start).count();

	// Discard transformation if fitness has not improved
	if (discard_transform)
	{
		int x_angle_limit = 30;
		int y_angle_limit = 2;
		int z_angle_limit = 2;
		if (cam_upside_down) y_angle_limit = 180 - y_angle_limit;
		Eigen::Vector3d euler_angles = extractEulerAngles(final_transform.rotation());
		bool angle_x_fit             = (std::abs(rad2deg(euler_angles(0))) < x_angle_limit);
		bool angle_y_fit             = (std::abs(rad2deg(euler_angles(1))) < y_angle_limit);
		bool angle_z_fit             = (std::abs(rad2deg(euler_angles(2))) < z_angle_limit);
		bool angles_fit              = angle_x_fit && angle_y_fit && angle_z_fit;
		if (verbose)
		{
			std::cout << "## euclidean_fitness = " << euclidean_fitness << std::endl;
			std::cout << "## best_fitness = " << best_fitness << std::endl;
			std::cout << "## angles_fit = " << angles_fit << std::endl;
		}
		if (euclidean_fitness < best_fitness && angles_fit)
		{
			best_fitness = euclidean_fitness;
		}
		else
		{
			final_transform = initial_transform;
			best_fitness *= 1.5;
			if (verbose)
			{
				std::cout << "## Transformation discarded" << std::endl;
			}
		}
	}

	// Reset initial guess if fitness too high
	if (reset_initial_guess)
	{
		double fitness_limit = resolution * 10;
		if (euclidean_fitness > fitness_limit)
		{
			if (verbose)
			{
				std::cout << "## Transform guess reset" << std::endl;
			}
			final_transform = start_transform;
		}
	}

	// Publish clouds
	if (publish_clouds)
	{
		sensor_msgs::msg::PointCloud2::SharedPtr target_cloud_msg =
			sensor_msgs::msg::PointCloud2::SharedPtr(new sensor_msgs::msg::PointCloud2);
		pcl::toROSMsg(*target_cloud, *target_cloud_msg);
		target_cloud_msg->header.frame_id = "camera_left_color_optical_frame";
		target_cloud_msg->header.stamp    = this->now();
		publisher_target_cloud->publish(*target_cloud_msg);

		pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		registration->getAlignedCloud(aligned_cloud);
		sensor_msgs::msg::PointCloud2::SharedPtr aligned_cloud_msg =
			sensor_msgs::msg::PointCloud2::SharedPtr(new sensor_msgs::msg::PointCloud2);
		pcl::toROSMsg(*aligned_cloud, *aligned_cloud_msg);
		aligned_cloud_msg->header.frame_id = "camera_left_color_optical_frame";
		aligned_cloud_msg->header.stamp    = this->now();
		publisher_aligned_cloud->publish(*aligned_cloud_msg);
	}

	// Profiling
	if (enable_profiling)
	{
		std::vector<double> vec;
		vec.push_back(euclidean_fitness);
		vec.push_back(icp_duration);
		vec.push_back(registration->getTargetSize());
		vec.push_back(registration->getSourceSize());
		profiling_vec.push_back(vec);

		if (profiling_vec.size() == static_cast<unsigned>(profiling_size))
		{
			std::cout << "Write profiling file " << profiling_filename << std::endl;
			std::ofstream profiling_file(profiling_filename);
			for (unsigned i = 0; i < profiling_fields.size(); i++)
			{
				profiling_file << profiling_fields[i];
				if (i != profiling_fields.size() - 1) profiling_file << ",";
			}
			profiling_file << "\n";
			for (const auto& l : profiling_vec)
			{
				for (unsigned i = 0; i < l.size(); i++)
				{
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
 * @brief Callback for depth frame message synchronization
 * @param depth_msg_left Left depth frame message
 * @param depth_msg_right Right depth frame message
 */
void RegistrationNode::depthSyncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg_left,
										 const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg_right)
{
	if (exit_request->load()) return;
	subscriber_depth_left.unsubscribe();
	subscriber_depth_right.unsubscribe();

	int64_t time_now                = this->now().nanoseconds();
	double time_since_last_callback = (time_now - last_callback_timestamp) / 1e6;
	if (verbose)
	{
		std::cout << "Time since last registration callback = " << time_since_last_callback << std::endl;
	}

	last_callback_timestamp = time_now;
	Eigen::Affine3d final_transform;

	if (adjust_resolution)
	{
		setResolution(current_resolution);
		registration->setVoxelgridSize(current_voxelsize);
	}
	else
	{
		setResolution(resolution);
		registration->setVoxelgridSize(voxelsize);
	}

	// Iterative closest point
	icp(depth_msg_left, depth_msg_right, camera_info_left, camera_info_right, depth_scale, final_transform,
		initial_transform);
	last_transform    = final_transform;
	initial_transform = final_transform;

	if (adjust_resolution)
	{
		// In initial phase start with low resolution (high resolution value in meters)
		if (initial_phase)
		{
			// Decrease resolution
			if (current_resolution - resolution_step > resolution)
			{
				current_resolution -= resolution_step;
			}
			else
			{
				current_resolution = resolution;
			}
			// Decrease voxel size
			if (current_resolution * voxel_resolution_factor > voxelsize)
			{
				current_voxelsize = voxel_resolution_factor * current_resolution;
			}
			else
			{
				current_voxelsize = voxelsize;
			}
			if (!(current_resolution > resolution) && !(current_voxelsize > voxelsize))
			{
				initial_phase = false;
			}
		}
	}

	// Publish transform
	try
	{
		geometry_msgs::msg::TransformStamped transform_msg = tf2::eigenToTransform(final_transform);
		transform_msg.header.stamp                         = this->now();
		transform_msg.child_frame_id                       = "camera_right_color_optical_frame";
		transform_msg.header.frame_id                      = "camera_left_color_optical_frame";
		publisher_transform->publish(transform_msg);
	}
	catch (tf2::TransformException& ex)
	{
		std::cout << "Publish transfrom error: " << ex.what() << std::endl;
	}
}

/**
 * @brief Deproject depth frame to pointcloud with cpu.
 * @param cloud Pointcloud result
 * @param depth_msg Depth frame message
 * @param camerainfo Camera intrinsics message
 * @param depth_scale Factor to convert camera depth unit to meters
 */
void RegistrationNode::deprojectDepthCpu(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
										 const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
										 sensor_msgs::msg::CameraInfo& camerainfo, float depth_scale)
{
	Intrinsics intrinsics;
	cameraInfo2Intrinsics(camerainfo, intrinsics);
	const uint16_t* depth_img = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);

	uint dropped_depth_values = 0;
	int row_step              = depth_msg->step / sizeof(uint16_t);
	for (uint j = 0; j < depth_msg->height; j++, depth_img += row_step)
	{
		for (uint i = 0; i < depth_msg->width; i++)
		{
			uint16_t d = depth_img[i];

			float scaled_depth = float(d) * depth_scale;
			if (scaled_depth <= min_depth || scaled_depth >= max_depth)
			{
				dropped_depth_values++;
				continue;
			}
			float x = (static_cast<float>(i) - intrinsics.ppx) / intrinsics.fx;
			float y = (static_cast<float>(j) - intrinsics.ppy) / intrinsics.fy;

			pcl::PointXYZ p;
			p.x = static_cast<float>(d) * x * depth_scale;
			p.y = static_cast<float>(d) * y * depth_scale;
			p.z = static_cast<float>(d) * depth_scale;
			cloud->points.push_back(p);
		}
	}

	if (verbose)
	{
		std::cout << "| dropped depth values: " << dropped_depth_values << std::endl;
		std::cout << "| result cloud size:    " << cloud->size() << std::endl;
	}
}

/**
 * @brief Declare ros node parameters.
 */
void RegistrationNode::declareParameters()
{
	parameters_callback_handle = this->add_on_set_parameters_callback(
		std::bind(&RegistrationNode::parametersCallback, this, std::placeholders::_1));

	this->declare_parameter("verbose", false);
	this->declare_parameter("qos_sensor_data", false);
	this->declare_parameter("qos_history_depth", 2);
	this->declare_parameter("resolution", 0.05);
	this->declare_parameter("spin_rate", 1.);
	this->declare_parameter("min_depth", 0.5);
	this->declare_parameter("max_depth", 2.0);
	this->declare_parameter("depth_scale", 0.0001);
	this->declare_parameter("depth_scale_left", -1.0);
	this->declare_parameter("depth_scale_right", -1.0);
	this->declare_parameter("voxelsize", 0.001);
	this->declare_parameter("roi_left", std::vector<long>({ -1, -1, -1, -1 }));
	this->declare_parameter("roi_right", std::vector<long>({ -1, -1, -1, -1 }));
}

/**
 * @brief Callback for ros node parameter changes.
 * @param parameters Changed node parameters
 * @return Result of parameter setting
 */
rcl_interfaces::msg::SetParametersResult RegistrationNode::parametersCallback(
	const std::vector<rclcpp::Parameter>& parameters)
{
	for (const auto& param : parameters)
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
		std::string parameter_name = parameter_string_tokens.back();

		// Set node parameters from yaml config
		if (parameter_string_tokens.size() == 1)
		{
			if (parameter_name == "verbose")
			{
				this->verbose = param.as_bool();
			}
			else if (parameter_name == "qos_sensor_data")
			{
				this->qos_sensor_data = param.as_bool();
			}
			else if (parameter_name == "qos_history_depth")
			{
				this->qos_history_depth = static_cast<int>(param.as_int());
			}
			else if (parameter_name == "resolution")
			{
				this->resolution = param.as_double();
				if (debug) std::cout << "set resolution = " << resolution << std::endl;
				setResolution(resolution);
			}
			else if (parameter_name == "spin_rate")
			{
				this->spin_rate = param.as_double();
				if (debug) std::cout << "set spin_rate = " << spin_rate << std::endl;
			}
			else if (parameter_name == "min_depth")
			{
				this->min_depth = static_cast<float>(param.as_double());
				if (debug) std::cout << "set min_depth = " << min_depth << std::endl;
			}
			else if (parameter_name == "max_depth")
			{
				this->max_depth = static_cast<float>(param.as_double());
				if (debug) std::cout << "set max_depth = " << max_depth << std::endl;
			}
			else if (parameter_name == "depth_scale")
			{
				this->depth_scale = static_cast<float>(param.as_double());
				if (debug) std::cout << "set depth_scale = " << depth_scale << std::endl;
			}
			else if (parameter_name == "depth_scale_left")
			{
				this->depth_scale_left = static_cast<float>(param.as_double());
				if (debug) std::cout << "set depth_scale_left = " << depth_scale_left << std::endl;
			}
			else if (parameter_name == "depth_scale_right")
			{
				this->depth_scale_right = static_cast<float>(param.as_double());
				if (debug) std::cout << "set depth_scale_right = " << depth_scale_right << std::endl;
			}
			else if (parameter_name == "voxelsize")
			{
				this->voxelsize = param.as_double();
				registration->setVoxelgridSize(voxelsize);
				if (debug) std::cout << "set voxelsize = " << voxelsize << std::endl;
			}
			else if (parameter_name == "roi_left")
			{
				std::vector<long> roi_vec = param.as_integer_array();
				if (roi_vec.size() == 4)
				{
					this->roi_left = { static_cast<int>(roi_vec[0]), static_cast<int>(roi_vec[1]), static_cast<int>(roi_vec[2]),
									   static_cast<int>(roi_vec[3]) };
				}
				else
					this->roi_left = { -1, -1, -1, -1 };
			}
			else if (parameter_name == "roi_right")
			{
				std::vector<long> roi_vec = param.as_integer_array();
				if (roi_vec.size() == 4)
				{
					this->roi_right = { static_cast<int>(roi_vec[0]), static_cast<int>(roi_vec[1]), static_cast<int>(roi_vec[2]),
										static_cast<int>(roi_vec[3]) };
				}
				else
					this->roi_right = { -1, -1, -1, -1 };
			}
		}
	}

	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	result.reason     = "success";
	return result;
}

/**
 * @brief Save transformation to disk.
 * @param transform Affine transformation to save
 * @param filename Filename
 * @return True on success
 */
bool RegistrationNode::saveTransform(Eigen::Affine3d& transform, const std::string filename)
{
	std::ofstream file;
	file.open(filename, std::ios::out | std::ios::trunc);

	const static Eigen::IOFormat matrix_format(Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n");

	if (debug)
	{
		std::cout << "Saving transform matrix:" << std::endl;
		std::cout << transform.matrix() << std::endl;
	}

	if (file.is_open())
	{
		file << transform.matrix().format(matrix_format);
		file.close();
	}
	if (file.bad())
	{
		file.close();
		std::cout << "Error saving icp transform file: " << filename << std::endl;
		return false;
	}
	if (verbose)
	{
		std::cout << "Icp transform file saved: " << filename << std::endl;
	}
	return true;
}

/**
 * @brief Load transformation from disk.
 * @param transform Affine transformation read from disk
 * @param filename Filename
 * @return True on success
 */
bool RegistrationNode::loadTransform(Eigen::Affine3d& transform, const std::string filename)
{
	std::ifstream file;
	bool matrix_bad  = false;
	bool file_exists = std::experimental::filesystem::exists(filename);
	if (file_exists)
	{
		file.open(filename, std::ios::in);
	}
	else
	{
		std::cout << "Transform file does not exist: " << filename << std::endl;
	}

	if (file.is_open())
	{
		std::vector<double> matrix_entries;
		std::string matrix_row_string;
		std::string matrix_entry;
		int matrix_row_number = 0;
		int matrix_col_number = 0;

		while (getline(file, matrix_row_string))
		{
			std::stringstream matrix_row_stringstream(matrix_row_string);
			while (getline(matrix_row_stringstream, matrix_entry, ' '))
			{
				matrix_entries.push_back(std::stod(matrix_entry));
				matrix_col_number++;
				if (matrix_col_number >= 4) break;
			}
			matrix_col_number = 0;
			matrix_row_number++;
			if (matrix_row_number >= 4) break;
		}
		file.close();

		Eigen::Matrix4d matrix = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(matrix_entries.data());
		transform.matrix()     = matrix;

		if (matrix_row_number < 3) matrix_bad = true;
	}
	if (!file_exists || matrix_bad || file.bad())
	{
		if (file.is_open()) file.close();
		transform.matrix().setIdentity();
		std::cout << "Initial transform set to identity" << std::endl;
		return false;
	}
	if (verbose)
	{
		std::cout << "Icp transform file loaded: " << filename << std::endl;
	}
	return true;
}

/**
 * @brief Convert camera intrinsics message to intrinsics structure.
 * @param camerainfo Camera intrinsics message
 * @param intrinsics Intrinsics structure
 */
void RegistrationNode::cameraInfo2Intrinsics(const sensor_msgs::msg::CameraInfo& camerainfo, Intrinsics& intrinsics)
{
	intrinsics.width  = static_cast<int>(camerainfo.width);
	intrinsics.height = static_cast<int>(camerainfo.height);
	intrinsics.ppx    = static_cast<int>(camerainfo.k[2]);
	intrinsics.ppy    = static_cast<int>(camerainfo.k[5]);
	intrinsics.fx     = static_cast<int>(camerainfo.k[0]);
	intrinsics.fy     = static_cast<int>(camerainfo.k[4]);
	intrinsics.model  = Distortion::DISTORTION_BROWN_CONRADY;
	for (uint i = 0; i < 5; i++)
	{
		intrinsics.coeffs[i] = static_cast<float>(camerainfo.d[i]);
	}
}

/**
 * @brief Set icp algorithm resolution.
 * @param resolution Icp algorithm resolution
 */
void RegistrationNode::setResolution(double resolution)
{
	double kernel_width    = resolution / 2.;
	double kernel_max_dist = kernel_width * 5;
	registration->setResolution(resolution);
	registration->setKernelParameters(kernel_width, kernel_max_dist);
}

/**
 * @brief Initialize timer for registration callback.
 */
void RegistrationNode::initTimer()
{
	this->timer = this->create_wall_timer(std::chrono::nanoseconds(static_cast<int64_t>(1e9 / (spin_rate))),
										  std::bind(&RegistrationNode::timerCallback, this));
	std::cout << "+==========[ Registration Node Started ]==========+" << std::endl;
}

/**
 * @brief Registration timer callback resubscibes to depth images.
 */
void RegistrationNode::timerCallback()
{
	if (qos_sensor_data) qos_profile = rclcpp::SensorDataQoS();
	qos_profile = qos_profile.keep_last(static_cast<size_t>(qos_history_depth));

	subscriber_depth_left.subscribe(this, topic_depth_left, qos_profile.get_rmw_qos_profile());
	subscriber_depth_right.subscribe(this, topic_depth_right, qos_profile.get_rmw_qos_profile());
}
