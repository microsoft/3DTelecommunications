#ifndef __EDMATCHINGLSQPROBLEMCUDA_GPU_H__
#define __EDMATCHINGLSQPROBLEMCUDA_GPU_H__
#include "DeformGraphCuda_GPU.h"
#include "EDMatchingHelperCuda_GPU.h"
#include "EDMatchingJacobianHelper.h"
#include "lmsolver.h"
#include "lmsolver_gpu.h"

namespace Fusion4D_GPU{
#include "EDMatchingLSQProblemCudaCore.h"
}

#endif