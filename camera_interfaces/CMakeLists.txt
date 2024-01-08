cmake_minimum_required(VERSION 3.5)
project(camera_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/GetCameraParameters.srv"
	  "msg/DepthFrameset.msg"
	  "msg/EncodedImage.msg"
	  "msg/EncodedFrameset.msg"
    DEPENDENCIES sensor_msgs
)

ament_package()
