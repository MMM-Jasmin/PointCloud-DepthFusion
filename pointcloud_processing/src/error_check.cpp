#include "error_check.h"

// Cuda error handling

/**
 * @brief Check for last cuda error.
 * @param msg Message to display on error
 * @param sync True to activate cuda device synchronization
 */
void checkError(const char* msg, bool sync)
{
	if (sync) cudaDeviceSynchronize();
	cudaError_t error = cudaGetLastError();
	if (error != cudaSuccess)
	{
		std::cerr << "Cuda error: " << msg << " failed: " << cudaGetErrorString(error) << std::endl;
		exit(EXIT_FAILURE);
	}
}

/**
 * @brief Check for cuda error.
 * @param error Received cuda error state
 * @param msg Message to display on error
 * @param sync True to activate cuda device synchronization
 */
void checkError(cudaError_t error, const char* msg, bool sync)
{
	if (sync) cudaDeviceSynchronize();
	if (error != cudaSuccess)
	{
		std::cerr << "Cuda error: " << msg << " failed: " << cudaGetErrorString(error) << std::endl;
		exit(EXIT_FAILURE);
	}
}

/**
 * @brief Check for cuda npp error.
 * @param error Received cuda npp error state
 * @param msg Message to display on error
 * @param sync True to activate cuda device synchronization
 */
void checkNppError(NppStatus error, const char* msg, bool sync)
{
	if (sync) cudaDeviceSynchronize();
	if (error != NPP_NO_ERROR)
	{
		std::cerr << "Cuda npp error: " << msg << " failed: " << error << std::endl;
		exit(EXIT_FAILURE);
	}
}
