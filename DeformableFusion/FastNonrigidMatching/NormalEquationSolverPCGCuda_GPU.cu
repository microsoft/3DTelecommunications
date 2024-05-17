#include "NormalEquationSolverPCGCuda_GPU.h"
#include "PCGCuda_GPU.cuh"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

namespace Fusion4D_GPU{
#include "NormalEquationSolverPCGCuda.cu"
}