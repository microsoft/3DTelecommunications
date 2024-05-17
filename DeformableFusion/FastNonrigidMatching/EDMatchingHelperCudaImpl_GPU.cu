#include "EDMatchingHelperCudaImpl_GPU.cuh"
#include "../Common/cuda/PinnedMemory.h"
#include "../Common/cuda/CudaHelpers.h"

namespace Fusion4D_GPU{
#include "EDMatchingHelperCudaImpl.cu"
#include "EDMatchingHelperCudaImplInit.cu"
#include "EDMatchingHelperCudaImplRegTerm.cu"
}
