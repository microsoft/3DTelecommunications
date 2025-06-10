// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#ifndef __CUDA_UTILITY_H__
#define __CUDA_UTILITY_H__
#include "UtilVnlMatrix.h"
#include <cuda_runtime.h>
#include "helper_cuda.h"

inline bool readout_global_memory_as_vector(float const* dev_x, int n, vnl_vector<float> &x)
{
	x.set_size(n);
	checkCudaErrors(cudaMemcpy(x.data_block(), dev_x, sizeof(float)*n, cudaMemcpyDeviceToHost));
	return true;
}


#endif