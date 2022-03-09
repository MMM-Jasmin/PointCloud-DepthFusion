#pragma once

// SYSTEM
#include <iostream>
// CUDA
#include <cuda_runtime.h>
// CUDA NPP
#include <npp.h>

// Cuda error handling
void checkError(const char* msg, bool sync = false);
void checkError(cudaError_t error, const char* msg, bool sync = false);
void checkNppError(NppStatus error, const char* msg, bool sync = false);
