// SYSTEM
#include <atomic>
#include <csignal>
#include <cstdio>
// ROS2
#include <rclcpp/rclcpp.hpp>
// PROJECT
#include "fusion_node.hpp"

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
bool cmdArgExists(char **begin, char **end, const std::string &argument)
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
char *getCmdArg(char **begin, char **end, const std::string &argument)
{
	char **itr = std::find(begin, end, argument);
	if (itr != end && ++itr != end)
		return *itr;

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
int main(int argc, char **argv)
{
	bool standalone = false;

	if (cmdArgExists(argv, argv + argc, "--standalone"))
		standalone = true;

	signal(SIGINT, signalHandler);

	std::cout << "+==========[ Fusion Node ]==========+" << std::endl;
	rclcpp::init(argc, argv);

	// std::make_shared conflicts with Eigen member variable alignment
	std::shared_ptr<FusionNode> fusion_node = std::shared_ptr<FusionNode>(new FusionNode);
	fusion_node->setExitSignal(&exit_request);
	fusion_node->init();

	rclcpp::executors::MultiThreadedExecutor executor1(rclcpp::ExecutorOptions(), 3, false);

	executor1.add_node(fusion_node);
	executor1.spin();

	executor1.cancel();
	rclcpp::shutdown();

	std::cout << "+==========[ Shutdown ]==========+" << std::endl;
	return EXIT_SUCCESS;
}
