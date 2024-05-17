#include "DeformGraphCudaImpl_GPU.cuh"
#include "cuda_math_common.cuh"
#include <cuda_runtime.h>

#include "../Common/cuda/PinnedMemory.h"
#include "../Common/cuda/CudaHelpers.h"

namespace Fusion4D_GPU{
#include "DeformGraphCudaImpl.cu"
}