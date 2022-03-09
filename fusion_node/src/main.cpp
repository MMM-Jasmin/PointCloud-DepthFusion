// SYSTEM
#include <cstdio>
#include <csignal>
#include <atomic>
#include <pthread.h>
#include <sys/time.h>      // setpriority
#include <sys/resource.h>  // setpriority
#include <sys/syscall.h>   // SYS_gettid
// ROS2
#include <rclcpp/rclcpp.hpp>
// PROJECT
#include "fusion_node.hpp"
#include "camera_node/camera_node.hpp"

/**
 * @brief Exit request flag.
 */
static std::atomic<bool> exit_request(false);

/**
 * @brief Check if given command line argument exists.
 * @param begin Pointer to the beginning of the argument array
 * @param end Pointer to the end of the argument array
 * @param argument The command line argument to check for
 * @return True if command line argument exists, false otherwise
 */
bool cmdArgExists(char** begin, char** end, const std::string& argument) {
	return std::find(begin, end, argument) != end;
}

/**
 * @brief Get value of given command line argument.
 * @param begin Pointer to the beginning of the argument array
 * @param end Pointer to the end of the argument array
 * @param argument Command line argument to get the value for
 * @return Pointer to the command line argument value
 */
char* getCmdArg(char** begin, char** end, const std::string& argument) {
	char** itr = std::find(begin, end, argument);
	if (itr != end && ++itr != end) {
		return *itr;
	}
	return nullptr;
}

/**
 * @brief Handler for received process signals.
 * @param signum Code of the received signal
 */
void signalHandler(int signum) {
	std::cout << "+==========[ Signal " << signum << " Abort ]==========+" << std::endl;
	exit_request.store(true);
}

/**
 * @brief Main function.
 * @param argc Number of command line arguments
 * @param argv Given command line arguments
 * @return EXIT_SUCCESS (0) on clean exit, EXIT_FAILURE (1) on error state
 */
int main(int argc, char** argv) {
	bool standalone = false;

	if (cmdArgExists(argv, argv + argc, "--standalone")) {
		standalone = true;
	}

	signal(SIGINT, signalHandler);

	std::cout << "+==========[ Fusion Node ]==========+" << std::endl;
	rclcpp::init(argc, argv);

	// std::make_shared conflicts with Eigen member variable alignment
	std::shared_ptr<FusionNode> fusion_node = std::shared_ptr<FusionNode>(new FusionNode);
	std::shared_ptr<CameraNode> camera_node = nullptr;

	rclcpp::executors::MultiThreadedExecutor executor1(rclcpp::executor::ExecutorArgs(), 2, false);

	if (!standalone) {
		camera_node = std::make_shared<CameraNode>("camera_left");
		camera_node->setExitSignal(&exit_request);
		camera_node->init();

		executor1.add_node(camera_node);
	}

	auto spin_funct1 = [&executor1]() {
		id_t tid = static_cast<unsigned>(syscall(SYS_gettid));
		int ret = ::setpriority(PRIO_PROCESS, tid, -20);
		if (ret) {
			std::cout << "Unable to set nice value for thread 1" << std::endl;
			return;
		}
		executor1.spin();
	};
	std::thread spin_thread1;
	if (!standalone) spin_thread1 = std::thread(spin_funct1);

	fusion_node->setExitSignal(&exit_request);
	fusion_node->init();
	if (exit_request.load()) {
		executor1.cancel();
		if (spin_thread1.joinable()) spin_thread1.join();
		if (rclcpp::ok()) rclcpp::shutdown();
		return EXIT_SUCCESS;
	}
	rclcpp::executors::MultiThreadedExecutor executor2(rclcpp::executor::ExecutorArgs(), 2, false);

	executor2.add_node(fusion_node);

	auto spin_funct2 = [&executor2]() {
		id_t tid = static_cast<unsigned>(syscall(SYS_gettid));
		int ret = ::setpriority(PRIO_PROCESS, tid, -20);
		if (ret) {
			std::cout << "Unable to set nice value for thread 2" << std::endl;
			return;
		}
		executor2.spin();
	};

	std::thread spin_thread2(spin_funct2);

	// Set affinity
	if (!standalone) {
		size_t thread1_core_id = 0;
		cpu_set_t cpuset1;
		CPU_ZERO(&cpuset1);
		CPU_SET(thread1_core_id, &cpuset1);
		int rc = pthread_setaffinity_np(spin_thread1.native_handle(), sizeof(cpu_set_t), &cpuset1);
		if (rc != 0) {
			std::cerr << "Error calling pthread_setaffinity_np: " << rc << std::endl;
		}
	}
	size_t thread2_core_id = 1;
	cpu_set_t cpuset2;
	CPU_ZERO(&cpuset2);
	CPU_SET(thread2_core_id, &cpuset2);
	int rc = pthread_setaffinity_np(spin_thread2.native_handle(), sizeof(cpu_set_t), &cpuset2);
	if (rc != 0) {
		std::cerr << "Error calling pthread_setaffinity_np: " << rc << std::endl;
	}

	if (!standalone) spin_thread1.join();
	spin_thread2.join();

	executor1.cancel();
	executor2.cancel();

	rclcpp::shutdown();

	std::cout << "+==========[ Shutdown ]==========+" << std::endl;
	return EXIT_SUCCESS;
}
