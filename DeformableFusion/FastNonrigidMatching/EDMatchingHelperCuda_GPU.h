#ifndef __EDMATCHINGHELPERCUDA_GPU_CUH__
#define __EDMATCHINGHELPERCUDA_GPU_CUH__

#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include "BlockedHessianMatrix.h"
#include <opencv2\opencv.hpp>

#include "EDMatchingHelperCudaImpl_GPU.cuh"
#include "DeformGraphCuda_GPU.h"
#include "UtilVnlMatrix.h"

#include "CameraView.h"
#include "utility.h"
#include "voxel_data_io.h"

namespace Fusion4D_GPU{
#include "EDMatchingHelperCuda.h"
}

#endif