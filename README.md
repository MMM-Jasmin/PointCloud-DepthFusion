# DepthFusion - Smart Mirror Point Cloud Fusion Module
The smart mirror platform was developed in the context of the Low Energy Toolset for Heterogeneous Computing ([LEGaTO](https://github.com/LEGaTO-SmartMirror)) project.  
DepthFusion is a framework that extends the smart mirror by generating a mirror image of the user, based on a point cloud representation generated from color and depth images from two connected RealSense D455 cameras.  
The framework uses the Robot Operating System ([ROS2](https://docs.ros.org/en/foxy/index.html)) for distributed data centric communication across the two installed Nvidia Jetson Xavier AGX embedded computing units.

## Content
1. [Structure](#structure)
2. [Requirements](#requirements)
3. [Installation](#installation)
4. [Usage](#usage)

## Structure
- The project is structured into ROS packages
	- `camera_node`  
		- Runs on both Jetsons  
			- Left camera: smartmirror4-1  
			- Right camera: smarmirror4-2  
		- Fetches color and depth images from the RealSense D455 camera  
		- Depth images are sent to the registration node  
		- Color and depth images are sent as framesets to the fusion node  
	- `registration_node`  
		- Runs on smartmirror4-2
		- Receives depth images
		- Projects depth images to point clouds
		- Registers the right point cloud to the left and estimates a transformation from the right to the left coordinate system
		- The transformation is sent to the fusion node
	- `fusion_node`  
		- Runs on smartmirror4-1
		- Receives color and depth images bundled as framesets
		- Projects framesets to point clouds
		- Receives transformation from the registration node and transforms the right point cloud into the frame of reference of the left
		- Creates a virtual camera which is located between the two cameras by default
		- Fuses the two point clouds and projects the result onto the virtual camera plane
		- Publishes the fused output image
	- `pointcloud_processing`  
		- Provides CUDA kernels
		- Classes for point and image processing
	- `image_node`  
		- Based on OpenCV
		- Visualizes output image
	- `camera_interfaces`  
		- Specifies frameset message
		- Defines camera parameter service

## Requirements
- [Ubuntu](https://ubuntu.com) 18.04 LTS
- [GCC](https://gcc.gnu.org) 7.5.0
- [CMake](https://cmake.org) 3.22.2
- [CUDA](https://developer.nvidia.com/cuda-toolkit) 10.2
- [Eigen3](https://eigen.tuxfamily.org) 3.4.0
- [OpenCV](https://opencv.org) 4.5.2
- [Point Cloud Library](https://github.com/PointCloudLibrary/pcl.git) 1.12.0
- [Realsense SDK](https://github.com/IntelRealSense/librealsense.git) 2.50.0
- [FastGICP Library](https://github.com/SMRT-AIST/fast_gicp.git) build from 22/11/22 (automatic download during CMake build)
- [ROS2](https://docs.ros.org/en/foxy/Installation.html) "Foxy Fitzroy"
	- Required ROS2 packages  
		`rclcpp`  
		`ament_index_cpp`  
		`sensor_msgs`  
		`message_filters`  
		`tf2_ros`  
		`tf2_eigen`  
		`tf2_geometry_msgs`  
		`image_transport`  
		`pcl_conversions`  


## Installation
- Assuming installed ROS2 workspace at `/opt/dev/ros2_ws`  

### Build
- Source ROS underlay and overlay  
`$ cd /opt/dev/ros2_ws`  
`$ source <rospath>/setup.bash`  
`$ source ./install/setup.bash`  
- Build packages for the first time  
`$ cd /opt/dev/ros2_ws`  
`$ git clone https://github.com/MMM-Jasmin/PointCloud-DepthFusion`  
`$ colcon build`  
`$ source ./install/local_setup.bash`  
- Rebuild packages  
`$ cd /opt/dev/ros2_ws`  
`$ colcon build --packages-select pointcloud_processing camera_node fusion_node`  
`registration_node image_node`  
- Compile Doxygen documentation  
`$ colcon build --packages-select pointcloud_processing camera_node fusion_node`   `registration_node image_node --cmake-target-skip-unavailable --cmake-target doc`  
Documentation as HTML and LaTeX can be found under  
`/opt/dev/ros2_ws/build/<package_name>/doc`  
- Optinal colcon arguments  
	- Show CMake output  
	`--event-handlers console_direct+`  
	- Force CMake configure step  
	`--cmake-force-configure`  
	- Remove CMake cache and force configure step  
	`--cmake-clean-cache`

## Usage

### Launch
- Run nodes in different processes  
	- On smartmirror4-1  
	`$ taskset -c 2-3 nice -n -20 ros2 launch camera_node camera_left.launch.py`  
	`$ taskset -c 4-5 nice -n -20 ros2 launch fusion_node standalone.launch.py`  
	- On smartmirror4-2  
	`$ taskset -c 2-3 nice -n -20 ros2 launch camera_node camera_right.launch.py`  
	`$ taskset -c 4-5 nice -n -20 ros2 launch registration_node standalone.launch.py`  
- Run nodes on the same host in one process  
	- On smartmirror4-1  
	`$ taskset -c 2-6 nice -n -20 ros2 launch fusion_node camera_left.launch.py`  
	- On smartmirror4-2  
	`$ taskset -c 2-6 nice -n -20 ros2 launch registration_node camera_right.launch.py`  
- Output image visualization  
`$ taskset -c 7 nice -n -10 ros2 run image_node image_node`  


### Configuration Files
- Configuration files for the nodes are provided in .yaml format  
- Folder for config files  
`/opt/dev/ros2_ws/install/<package_name>/share/<package_name>/config`  
- Default config files are included in the sources. To customize the configuration, copy the `default_config.yaml` file to `config.yaml` in the config folder and edit the copy. If no `config.yaml` is found, the default config file is loaded.  

### Message Parameters
- Quality of Service  
	- Profile: SENSOR_DATA  
	- History depth: 1
- Frame id: `camera_left_color_optical_frame`  
- Topics
	- Output image  
	`/fused_image`  
	- Depth images  
	`/camera_left/depth/image`  
	`/camera_right/depth/image`  
	- Framesets  
	`/camera_left/frameset`  
	`/camera_right/frameset`  
	- Camera parameters  
	`/camera_left/depth/camera_info`  
	`/camera_right/depth/camera_info`  
	- Registration  
	`/registration/transform`  
		- Debug point clouds  
		`/registration/target_points`  
		`/registration/aligned_points`  
	
| Topic | Message Type |  
| ----------- | ----------- |  
| `/fused_image` | `sensor_msgs::msg::Image` |  
| `/camera_left/depth/image` | `sensor_msgs::msg::Image` |  
| `/camera_right/depth/image` | `sensor_msgs::msg::Image` |  
| `/camera_left/frameset` | `camera_interfaces::msg::DepthFrameset` |  
| `/camera_right/frameset` | `camera_interfaces::msg::DepthFrameset` |  
| `/camera_left/depth/camera_info` | `sensor_msgs::msg::CameraInfo` |  
| `/camera_right/depth/camera_info` | `sensor_msgs::msg::CameraInfo` |  
| `/registration/transform` | `geometry_msgs::msg::TransformStamped` |  
| `/registration/target_points` | `sensor_msgs::msg::PointCloud2` |  
| `/registration/aligned_points` | `sensor_msgs::msg::PointCloud2` |  

