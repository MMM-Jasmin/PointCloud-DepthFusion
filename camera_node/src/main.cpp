// SYSTEM
#include <atomic>
#include <csignal>
#include <cstdio>
// ROS2
#include <rclcpp/rclcpp.hpp>

#include "camera_node.hpp"

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
bool cmdArgExists(char** begin, char** end, const std::string& argument)
{
	return std::find(begin, end, argument) != end;
}

/**
 * @brief Get value of given command line argument.
 * @param begin Pointer to the beginning of the argument array
 * @param end Pointer to the end of the argument array
 * @param argument Command line argument to get the value for
 * @return Pointer to the command line argument value
 */
char* getCmdArg(char** begin, char** end, const std::string& argument)
{
	char** itr = std::find(begin, end, argument);
	if (itr != end && ++itr != end)
	{
		return *itr;
	}
	return nullptr;
}

/**
 * @brief Handler for received process signals.
 * @param signum Code of the received signal
 */
void signalHandler(int signum)
{
	std::cout << "+==========[ Signal " << signum << " Abort ]==========+" << std::endl;
	exit_request.store(true);
}

/**
 * @brief Main function.
 * @param argc Number of command line arguments
 * @param argv Given command line arguments
 * @return EXIT_SUCCESS (0) on clean exit, EXIT_FAILURE (1) on error state
 */
int main(int argc, char** argv)
{
	std::string node_name = "camera_node";

	if (cmdArgExists(argv, argv + argc, "--name"))
		node_name = std::string(getCmdArg(argv, argv + argc, "--name"));

	signal(SIGINT, signalHandler);

	std::cout << "+==========[ " << node_name << " ]==========+" << std::endl;
	rclcpp::init(argc, argv);

	std::shared_ptr<CameraNode> camera_node = std::make_shared<CameraNode>(node_name);
	camera_node->setExitSignal(&exit_request);
	camera_node->init();

	//rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(), 3, false);
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(camera_node);
	executor.spin();

	//auto spin_funct1 = [&executor]() { executor.spin(); };

	//std::thread spin_thread1(spin_funct1);
	//size_t thread1_core_id = 0;
	//cpu_set_t cpuset1;
	//CPU_ZERO(&cpuset1);
	//CPU_SET(thread1_core_id, &cpuset1);
	//int rc = pthread_setaffinity_np(spin_thread1.native_handle(), sizeof(cpu_set_t), &cpuset1);

	//if (rc != 0)
	//	std::cerr << "Error calling pthread_setaffinity_np: " << rc << std::endl;

	//spin_thread1.join();

	executor.cancel();
	rclcpp::shutdown();

	std::cout << "+==========[ Shutdown ]==========+" << std::endl;
	return EXIT_SUCCESS;
}
