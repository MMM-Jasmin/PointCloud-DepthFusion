#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
#include <thrust/scan.h>
#include <thrust/device_vector.h>
#include <thrust/system/cuda/execution_policy.h>

namespace compactor {

#define FULL_MASK 0xFFFFFFFF
#define warp_size 32

template <typename T_in, typename Predicate>
__global__ void kernel_compute_block_counts(int* dev_block_counts, const T_in* dev_input, const int input_count,
                                            Predicate predicate) {
	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	if (idx < input_count) {
		int pred = predicate(dev_input[idx]);
		int block_count = __syncthreads_count(pred);

		if (threadIdx.x == 0) {
			dev_block_counts[blockIdx.x] = block_count;
		}
	}
}

template <typename T_out, typename T_in, typename Predicate>
__global__ void kernel_compact(T_out* dev_output, const T_in* dev_input, const int length, const int* dev_block_offsets,
                               Predicate predicate) {
	int idx = threadIdx.x + blockIdx.x * blockDim.x;

	extern __shared__ int warp_counts[];
	if (idx < length) {
		int warp_idx = threadIdx.x / warp_size;
		int warp_thread_idx = idx % warp_size;
		int predicate_result = predicate(dev_input[idx]);

		int thread_mask = static_cast<int>(FULL_MASK >> (warp_size - warp_thread_idx));
		int ballot = __ballot_sync(FULL_MASK, predicate_result) & thread_mask;
		int thread_count = __popc(ballot);

		if (warp_thread_idx == warp_size - 1) {
			warp_counts[warp_idx] = thread_count + predicate_result;
		}
		__syncthreads();

		int num_warps = blockDim.x / warp_size;
		unsigned int num_warps_mask = FULL_MASK >> (warp_size - num_warps);
		if (warp_idx == 0 && warp_thread_idx < num_warps) {
			int warp_idx_counter = 0;
			for (int j = 0; j <= 5; j++) {
				int ballot_j = __ballot_sync(num_warps_mask, warp_counts[warp_thread_idx] & (1 << j));

				warp_idx_counter += (__popc(ballot_j & thread_mask)) << j;
			}
			warp_counts[warp_thread_idx] = warp_idx_counter;
		}
		__syncthreads();

		if (predicate_result) {
			// Copy input elements to output
			// dev_output[thread_count + warp_counts[warp_idx] + dev_block_offsets[blockIdx.x]] = dev_input[idx];
			// Write thread index to output
			dev_output[thread_count + warp_counts[warp_idx] + dev_block_offsets[blockIdx.x]] = idx;
		}
	}
}

template <typename T_out, typename T_in, typename Predicate>
int compact(T_out* dev_output, T_in* dev_input, const int input_count, const int block_size, Predicate predicate,
            cudaStream_t& stream) {
	// Calculate minimum number of blocks
	std::div_t num_blocks_div = std::div(input_count, block_size);
	int num_blocks = num_blocks_div.rem ? (num_blocks_div.quot + 1) : num_blocks_div.quot;
	// Allocate device memory
	int* dev_block_counts;
	int* dev_block_offsets;
	cudaMalloc(&dev_block_counts, sizeof(int) * static_cast<uint>(num_blocks));
	cudaMalloc(&dev_block_offsets, sizeof(int) * static_cast<uint>(num_blocks));
	// Wrap device pointer with thrust pointer
	thrust::device_ptr<int> thrust_block_counts(dev_block_counts);
	thrust::device_ptr<int> thrust_block_offsets(dev_block_offsets);

	// Count number of elements in each thread block that satisfies predicate
	kernel_compute_block_counts<<<num_blocks, block_size, 0, stream>>>(dev_block_counts, dev_input, input_count,
	                                                                   predicate);

	// Compute output offset for each block in grid
	thrust::exclusive_scan(thrust::cuda::par.on(stream), thrust_block_counts, thrust_block_counts + num_blocks,
	                       thrust_block_offsets);

	// Compute output offset for each thread in warp and each warp in block
	kernel_compact<<<num_blocks, block_size, sizeof(int) * (block_size / warp_size), stream>>>(
	    dev_output, dev_input, input_count, dev_block_offsets, predicate);

	// Get number of elements in result
	int compact_count = thrust_block_offsets[num_blocks - 1] + thrust_block_counts[num_blocks - 1];

	// Free device memory
	cudaFree(dev_block_counts);
	cudaFree(dev_block_offsets);

	return compact_count;
}
}  // namespace compactor
