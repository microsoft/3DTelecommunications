// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#ifndef __CCLCUDAImpl_H__
#define __CCLCUDAImpl_H__
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "cuda_math_common.cuh"

#include "../Common/cuda/PinnedMemory.h"
#include "../Common/cuda/CudaHelpers.h"

class CCLCudaImpl
{
public:
	CCLCudaImpl(int width, int height, int view_num)
	{
		if (width > 0 && height > 0 && view_num > 0)
		{
			this->width_ = width;
			this->height_ = height;
			this->view_num_ = view_num;
			allocate_labels_linear_memory(width, height, view_num);
		}

	};
	~CCLCudaImpl() {};

public:
	void run_ccl(cudaArray *cu_3dArr_depths, float depth_thres_in_cm, bool bUseTopBitAsSegBit);
	void clear_small_components(int thres_pixel_count = 300);

protected:
	int* dev_labels_all_views_;
	int height_;
	int width_;
	int view_num_;
	cuda::gpu_size_data labels_num_gpu_;
	short* dev_LUT_new_labels_;

protected:
	void allocate_labels_linear_memory(int width, int height, int view_num);
};


#endif