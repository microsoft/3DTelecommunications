#pragma once
// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.

// Debug Profiling

#define CUDA_PROFILER_PRINT 0 // not implemented, Mingsong!

/// Turn on CHECK_CUDA_ERROR for GPU debugging
//#define SYNC_CHECK_CUDA_ERRORS
#ifdef SYNC_CHECK_CUDA_ERRORS
#define m_checkCudaErrors() \
{\
	cudaDeviceSynchronize(); \
	checkCudaErrors(cudaGetLastError());\
}
#else
#define m_checkCudaErrors()
#endif

/// All the following parameters should be changed ONLY in the config files
#define SQUARE_IMAGE_LENGTH 2048
#define DEPTH_SCALE 2
#define DEFAULT_FRAMES_NUM 400

#define DEFAULT_COLOR_WIDTH		(SQUARE_IMAGE_LENGTH)
#define DEFAULT_COLOR_HEIGHT	(SQUARE_IMAGE_LENGTH)
#define DEFAULT_DEPTH_WIDTH		(SQUARE_IMAGE_LENGTH / DEPTH_SCALE)
#define DEFAULT_DEPTH_HEIGHT	(SQUARE_IMAGE_LENGTH / DEPTH_SCALE)

#define DEFAULT_DEPTH_BINS_FOLDER	"D:/Data/PeabodyDepthBin"
#define DEFAULT_CALIBRATION_FOLDER	"D:/Data/PeabodyCalibration"

#define DEFAULT_FUSION_DDF_MU		2.5
#define DEFAULT_FUSION_ED_NODE_RES	6.0
#define DEFAULT_FUSION_VXL_RES		0.7
#define DEFAULT_FUSION_MU			3.5
#define DEFAULT_FUSION_ISOS_LEVEL	0.1

// maximum distance when detecting ICP correspondence
#define DEFAULT_ALIGN_MU			5.0

#define FAST_PARAMS

// in cm 0.30--400k; 0.40--200K
#define DEFAULT_BEST_VOXEL_RES		0.4
#define DEFAULT_BEST_ED_NODES_RES	4.0

#define DEFAULT_HOST	"127.0.0.1"
#define DEFAULT_PORT	"30000"


#define LOG_TO_CONSOLE_LEVEL 0

#undef KINECT_RIG

// Enables printing of frame num for BufferAggregator. 
// Keeping this flag on top of using LOGGER()->check_verbosity(Logger::Debug)
// for higher efficiency of BufferAggregator, since it is a very sensitive object.
#undef DEBUG_PRINT_FRAMENUM 