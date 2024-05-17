#include "stdafx.h"
#include "EDMatchingLSQProblemCuda.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "DeformGraph.h"

namespace Fusion4D_GPU{

#include "EDMatchingLSQProblemCudaCore_cpu.cpp"
#include "EDMatchingLSQProblemCudaCore_gpu.cpp"

}