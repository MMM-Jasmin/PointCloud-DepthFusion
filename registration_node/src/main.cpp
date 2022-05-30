// SYSTEM
#include <atomic>
#include <csignal>
#include <cstdio>
#include <pthread.h>
#include <sys/resource.h> // setpriority
#include <sys/syscall.h>  // SYS_gettid
#include <sys/time.h>     // setpriority
// ROS2
#include <rclcpp/rclcpp.hpp>
// PROJECT
#include "registration_node.hpp"

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
	signal(SIGINT, signalHandler);

	std::cout << "+==========[ Registration Node ]==========+" << std::endl;
	rclcpp::init(argc, argv);

	// std::make_shared conflicts with Eigen member variable alignment
	std::shared_ptr<RegistrationNode> registration_node = std::shared_ptr<RegistrationNode>(new RegistrationNode);

	registration_node->setExitSignal(&exit_request);
	registration_node->init();

	//rclcpp::executors::MultiThreadedExecutor executor2(rclcpp::executor::ExecutorArgs(), 2, false);
	rclcpp::executors::SingleThreadedExecutor executor;

	executor.add_node(registration_node);
	registration_node->initTimer();
	executor.spin();

	//auto spin_funct2 = [&executor2]() {
	//	id_t tid = static_cast<unsigned>(syscall(SYS_gettid));
	//	int ret  = ::setpriority(PRIO_PROCESS, tid, -20);
	//	if (ret)
	//	{
	//		std::cout << "Unable to set nice value for thread 2" << std::endl;
	//		return;
	//	}
		
	//};

	//std::thread spin_thread2(spin_funct2);

	// Set affinity
	//if (!standalone)
	//{
	//	size_t thread1_core_id = 0;
	//	cpu_set_t cpuset1;
	//	CPU_ZERO(&cpuset1);
	//	CPU_SET(thread1_core_id, &cpuset1);
	//	int rc = pthread_setaffinity_np(spin_thread1.native_handle(), sizeof(cpu_set_t), &cpuset1);
	//	if (rc != 0)
	//	{
	//		std::cerr << "Error calling pthread_setaffinity_np: " << rc << std::endl;
	//	}
	//}
	//size_t thread2_core_id = 1;
	//cpu_set_t cpuset2;
	//CPU_ZERO(&cpuset2);
	//CPU_SET(thread2_core_id, &cpuset2);
	//int rc = pthread_setaffinity_np(spin_thread2.native_handle(), sizeof(cpu_set_t), &cpuset2);
	//if (rc != 0)
	//{
	//	std::cerr << "Error calling pthread_setaffinity_np: " << rc << std::endl;
	//}

	//if (!standalone) spin_thread1.join();
	//spin_thread2.join();

	executor.cancel();
	rclcpp::shutdown();

	std::cout << "+==========[ Shutdown ]==========+" << std::endl;
	return EXIT_SUCCESS;
}
