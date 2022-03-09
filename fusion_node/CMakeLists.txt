###########
## CMake ##
###########
cmake_minimum_required(VERSION 3.9)
message(STATUS "CMake version:\t" ${CMAKE_VERSION})
message(STATUS "CMAKE_ROOT:\t" ${CMAKE_ROOT})
cmake_policy(VERSION 3.9)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_WARN_DEPRECATED FALSE)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS TRUE)

#############
## Project ##
#############
project(fusion_node C CXX)
set(PROJECT_BINARY ${PROJECT_NAME})
set(PROJECT_LIBRARY ${PROJECT_NAME}_lib)

file(GLOB_RECURSE SOURCE_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.c)
file(GLOB_RECURSE HEADER_FILES ${PROJECT_SOURCE_DIR}/src/*.hpp ${PROJECT_SOURCE_DIR}/src/*.h)
#message(STATUS "SOURCE_FILES:\t" ${SOURCE_FILES})
#message(STATUS "HEADER_FILES:\t" ${HEADER_FILES})

add_executable(${PROJECT_BINARY} ${SOURCE_FILES} ${HEADER_FILES})

##############
## Compiler ##
##############
# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set_target_properties(${PROJECT_BINARY}
	PROPERTIES
	  CXX_STANDARD 14
		CXX_STANDARD_REQUIRED YES
		CXX_EXTENSIONS NO
)

# Filesystem
target_link_libraries(${PROJECT_BINARY} stdc++fs)

##################
## Dependencies ##
##################
#### RealSense2 ####
find_package(realsense2 REQUIRED)
message(STATUS "+==========[ RealSense2 ]===========")
if (realsense2_FOUND)
	message(STATUS "|  RealSense2 version:\t" ${realsense2_VERSION})
	message(STATUS "|  RealSense2 include:\t" ${realsense2_INCLUDE_DIR})
	message(STATUS "|  RealSense2 library:\t" ${realsense2_LIBRARY})
	include_directories("${realsense2_INCLUDE_DIR}")
	target_link_libraries(${PROJECT_BINARY} ${realsense2_LIBRARY})
else (realsense2_FOUND)
	message(STATUS "|  RealSense2 not found!")
endif (realsense2_FOUND)
message(STATUS "+===================================")

#### OpenCV ####
find_package(OpenCV REQUIRED)
message(STATUS "+==========[ OpenCV ]===========")
if (OpenCV_FOUND)
	message(STATUS "|  OpenCV version:\t" ${OpenCV_VERSION})
	#message(STATUS "|  OpenCV include:\t" ${OpenCV_INCLUDE_DIRS})
	#message(STATUS "|  OpenCV include:\t" ${OpenCV_LIBS})
	target_link_libraries(${PROJECT_BINARY} ${OpenCV_LIBS})
else (OpenCV_FOUND)
	message(STATUS "|  OpenCV not found!")
endif (OpenCV_FOUND)
message(STATUS "+===================================")

#### CUDA ####
message(STATUS "==========[ CUDA ]===========")
# Set CUDACXX if not in environment
if (NOT $ENV{CUDACXX})
	# Search for env variable CUDADIR or CUDA_HOME
	if ($ENV{CUDADIR})
		set(CUDACXX $ENV{CUDADIR}/bin/nvcc)
	elseif($ENV{CUDA_HOME})
		set(CUDACXX $ENV{CUDA_HOME}/bin/nvcc)
	endif()
endif()
# Check for CUDA
include(CheckLanguage)
check_language(CUDA)
if (CMAKE_CUDA_COMPILER)
	# CUDA Status
	enable_language(CUDA)
	message(STATUS "|  NVCC version:\t\t" ${CMAKE_CUDA_COMPILER_VERSION})
	message(STATUS "|  NVCC path:\t\t" ${CMAKE_CUDA_COMPILER})
	set(CUDA_NVCC_EXECUTABLE ${CMAKE_CUDA_COMPILER})
	# Check for and set GPU architecture
	include("cmake/CudaDetect.cmake")
	detect_installed_gpus(CUDA_NVCC_ARCHS)
	foreach(NVCC_ARCH IN LISTS CUDA_NVCC_ARCHS)
		list(APPEND CUDA_ARCH_BIN "${NVCC_ARCH} ")
	endforeach(NVCC_ARCH)
	set_property(TARGET ${PROJECT_BINARY} PROPERTY CUDA_ARCHITECTURES OFF)
	message(STATUS "|  CUDA architecture:\t\t" ${CUDA_ARCH_BIN})
	# Set CUDA language standard
	if(NOT DEFINED CMAKE_CUDA_STANDARD)
		set(CMAKE_CUDA_STANDARD 14)
		set(CMAKE_CUDA_STANDARD_REQUIRED ON)
	endif()
	# Include CUDA libraries
	target_include_directories(${PROJECT_BINARY} PRIVATE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
	# Link CUDA libraries
	find_library(CUDADRIVER_LIBRARY cuda ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES}) # only for cuMemsetD32Async
	find_library(CUDART_LIBRARY cudart ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})
	find_library(CUBLAS_LIBRARY cublas ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})
	find_library(CUBLASLT_LIBRARY cublasLt ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})
	message(STATUS "|  CUDA cudadriver library:\t" ${CUDADRIVER_LIBRARY})
	message(STATUS "|  CUDA cudart library:\t" ${CUDART_LIBRARY})
	message(STATUS "|  CUDA cublas library:\t" ${CUBLAS_LIBRARY})
	message(STATUS "|  CUDA cublasLt library:\t" ${CUBLASLT_LIBRARY})
	# Link CUDA NPP libraries
	find_library(NPPC_LIBRARY nppc ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})
	find_library(NPPIF_LIBRARY nppif ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})
	find_library(NPPITC_LIBRARY nppitc ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})
	find_library(NPPIM_LIBRARY nppim ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})
	find_library(NPPIST_LIBRARY nppist ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})
	message(STATUS "|  CUDA NPP core library:\t" ${NPPC_LIBRARY})
	message(STATUS "|  CUDA NPP filter library:\t" ${NPPIF_LIBRARY})
	message(STATUS "|  CUDA NPP threshold library:\t" ${NPPITC_LIBRARY})
	message(STATUS "|  CUDA NPP morphol. library:\t" ${NPPIM_LIBRARY})
	message(STATUS "|  CUDA NPP statistical library:\t" ${NPPIST_LIBRARY})
	target_link_libraries(${PROJECT_BINARY}
		${CUDADRIVER_LIBRARY}
		${CUDART_LIBRARY}
		${CUBLAS_LIBRARY}
		${CUBLASLT_LIBRARY}
		${NPPC_LIBRARY}
		${NPPIF_LIBRARY}
		${NPPITC_LIBRARY}
		${NPPIM_LIBRARY}
		${NPPIST_LIBRARY}
		)
else()
	message(STATUS "CUDA compiler not found! Perhaps set CUDACXX environment variable")
endif()
message(STATUS "+============================")

#### ROS2 ####
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(image_transport REQUIRED)

find_package(camera_interfaces REQUIRED)
find_package(camera_node REQUIRED)
include_directories(${camera_node_INCLUDE_DIR})
target_link_libraries(${PROJECT_BINARY} camera_node::camera_node_lib)

find_package(pointcloud_processing REQUIRED)
include_directories(${pointcloud_processing_INCLUDE_DIR})
target_link_libraries(${PROJECT_BINARY} pointcloud_processing::pointcloud_processing)

# Preprocessor define for ros2 distribution eloquent elusor
if ($ENV{ROS_DISTRO} STREQUAL "eloquent")
	target_compile_definitions(${PROJECT_BINARY} PRIVATE ROS_ELOQUENT)
endif ()
message(STATUS "+==========[ ROS2 ]===========")
message(STATUS "|  Distribution:\t" $ENV{ROS_DISTRO})
message(STATUS "+===================================")

###########
## Build ##
###########
target_include_directories(
	${PROJECT_BINARY} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
	${PROJECT_BINARY}
	"rclcpp"
	"sensor_msgs"
	"message_filters"
	"tf2_ros"
	"tf2_eigen"
	"ament_index_cpp"
	"tf2_geometry_msgs"
	"pcl_conversions"
	"image_transport"

	"camera_interfaces"
	"camera_node"
	"pointcloud_processing"
)

#############
## Install ##
#############
# Install launch, config and data files
install(
	DIRECTORY
	launch
	config
	data
	#${gpujpeg_SOURCE_DIR} include
	DESTINATION share/${PROJECT_NAME}
)

# Install node executable
install(
	TARGETS ${PROJECT_BINARY}
	DESTINATION lib/${PROJECT_NAME}
)

###################
## Documentation ##
###################
find_package(Doxygen)
if(DOXYGEN_FOUND)
	message(STATUS "Doxygen found")
	# Prepare doxygen configuration file
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
	# Add doc as target
	add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
endif(DOXYGEN_FOUND)

ament_package()
