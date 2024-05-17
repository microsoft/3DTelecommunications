#ifndef __VOLUMETRIC_FUSION_IMPL_GPU_CUH__
#define __VOLUMETRIC_FUSION_IMPL_GPU_CUH__
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "device_atomic_functions.h"
#include "geometry_types_cuda.h"
#include "CudaGlobalMemory.h"
#include "DeformGraphCudaImpl_GPU.cuh"
#include <stdint.h>

#include "utility.h"

#include "../Common/cuda/PinnedMemory.h"
#include "../Common/cuda/CudaHelpers.h"

namespace Fusion4D_GPU{
#include "mc_mesh_processing.cuh"
#include "volumetric_fusion_impl.cuh"
}

#endif